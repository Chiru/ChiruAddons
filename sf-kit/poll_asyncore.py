try:
    import asyncore
except ImportError:
    import sys
    print sys.path
    import os
    os._exit(0)
import asyncore, json, traceback
import PythonQt, tundra
import asyncore, asynchat, socket, os

def get_or_create_local_entity(entity_name, temporary=True):
    scene = tundra.Scene().MainCameraScene()
    ent = scene.GetEntityByNameRaw(entity_name) 
    if ent is None:
        ent = scene.CreateEntityLocalRaw(["EC_Name"])
        if temporary:
            ent.SetTemporary(True)
        ent.name = entity_name
    return ent

def connect_action(entity, action_name, callback):
    act = ent.Action(action_name)
    act.connect("Triggered(QString, QString, QString, QStringList)", callback)

class LineHandler(asynchat.async_chat):
    def __init__(self, sock):
        asynchat.async_chat.__init__(self, sock)
        self.set_terminator("\r\n")
        self.data = ""

    def collect_incoming_data(self, data):
        self.data = self.data + data

    def found_terminator(self):
        if self.data.rstrip() == '.':
            self.close()
            print 'closing after done'
            return

        req = None
        if not self.data.startswith("{"):
            # support shorthand for more convenient interactive debugging
            import shlex
            tokens=shlex.split(self.data)
            req = dict(zip(tokens[::2], tokens[1::2]))
            for k, v in req.items():
                if ',' in v:
                    req[k] = v.split(',')
            print 'using', req
        
        try:
            req = req or json.loads(self.data)
        except ValueError, what:
            self.report_err(str(what))
        except:
            self.report_err("json decode error")
        else:
            self.handle_request(req)
        self.data = ""
    
    def report_err(self, msg):
        self.push(json.dumps(dict(error=msg)))
        self.push("\r\n")


    def handle_request(self, req):
        op = req.get('op')
        if not op:
            self.report_err("missing op in request")
        else:
            methname = 'op_' + str(op)
            del req['op']
            meth = getattr(self, methname, None)
            if not meth:
                self.report_err("unknown op in request")
                return
            r=None
            try:
                r = meth(**req)
            except TypeError:
                self.report_err("bad args, usage: %s" % meth.__doc__)
                print "type error, traceback:"
                traceback.print_exc()
                return
            except:
                self.report_err("handler raised unhandled exception")
                print "error handling request for", op
                traceback.print_exc()
                return
            try:
                if r is None:
                    r = dict()
                self.push(json.dumps(r))
                self.push("\r\n")
            except (OverflowError, TypeError, ValueError), e:
                self.report_err("error serializing response")

    def op_listscenes(self):
        mcs = tundra.Scene().MainCameraScene()
        return dict(scenes=list(scenes), maincamera=mcs.Name() if mcs else None)

    ## untested/unfinished
    # def op_create_entity(self, comptypes, localonly, sync, temporary):
    #     "comptypes: list of strings with desired components (eg. ['EC_Name']), localonly: bool, sync: bool, temporary: bool"
    #     ent = tundra.CreateEntity(comptypes, localonly, sync, temporary)
    #     return dict(id=ent.id)


    def action_triggered(self, name, args):
        self.push(json.dumps(dict(action=name, args=args)))
        self.push("\r\n")
        print "action pushed"

    def op_action(self, exectype, scene, entity, action, params):
        "exectype: execution type string (or bitfield), scene: scene name, entity: entity name (or integer id), action: action name, params: list of strings"

        if isinstance(exectype, basestring):
            try:
                exectype = exectype_parse(exectype)
            except ValueError:
                self.report_err("bad exectype %s" % exectype)
                return
            
        if isinstance(entity, basestring):
            ent = tundra.Scene().MainCameraScene().GetEntityByNameRaw(entity)
        else:
            ent = tundra.Scene().MainCameraScene().GetEntityRaw(entity)
        if not ent:
            self.report_err("entity not found for json aftion (name/id=%s)" % entity)
            print "ent not found"
            return
        rv = ent.Exec(exectype, action, *params)
        print "action %s executed on entity %d" % (action, ent.id)
        return dict(returnvalue=rv)

    def op_subscribe_action(self, entity_name, action_name):
        "entity_name: s/e, action_name: s/e"
        ent = get_or_create_local_entity(entity_name)
        ent.Action(action_name).connect("Triggered(QString, QString, QString, QStringList)", lambda *args: self.action_triggered(action_name, args))
        print "action connected"
        return dict(id=ent.id)

    def op_push_service_info(self, info, name):
        "data: type of data delivered, name: service name"
        self.op_action('local', None, entity, action, params)

def exectype_parse(s):
    execint = 0
    for w in s.split(","):
        if w.lower() == 'local':
            execint |= 1
        elif w.lower() == 'server':
            execint |= 2
        elif w.lower() == 'peers':
            execint |= 4
        else:
            raise ValueError("unknown exec type " +w)
    return execint

class LineServer(asyncore.dispatcher):
    def __init__(self):
        asyncore.dispatcher.__init__(self)
        self.create_socket(socket.AF_INET, socket.SOCK_STREAM)
        self.set_reuse_addr()
        self.bind(('', 4242))
        self.listen(5)
    
    def writable(self):
        return False

    def handle_accept(self):
        sock, addr = self.accept()
        print 'line client from %s (socket fd %d)' % (addr, sock.fileno())
        
        LineHandler(sock)


scenes = set()
def on_sceneadded(name):
    scenes.add(name)
    print 'added scene', name

tundra.Scene().connect("SceneAdded(QString)", on_sceneadded)

def frame_update(t):
    if asyncore.socket_map:
        asyncore.poll(0, asyncore.socket_map)
        asyncore.poll(0, asyncore.socket_map)

if 1:
    server = LineServer()
    tundra.Frame().connect("Updated(float)", frame_update)

    ## reload() didn't work on this module since it wasn't loaded by tundra as a normal module
    # def stop():
    #     tundra.Frame().disconnect("Updated(float)", frame_update)
    #     print 'disconnected sig'
    #     server.close()
    #     print 'did server.close()'
    #     for fd in asyncore.socket_map.keys():
    #         os.close(fd)
    #     print 'closed socketmap fds'
