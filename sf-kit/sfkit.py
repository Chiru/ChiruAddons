import cPickle
import sys, os, time, collections
import socket, asynchat, asyncore, json
from pprint import pprint, pformat
import difflib, logging

logger = logging.getLogger("sf")

db_filename = 'sfdata.pickle'
db_root = {'sources': []}
query_map = {}

def open_db():
    if os.path.exists(db_filename):
        db_root.clear()
        db_root.update(cPickle.load(open(db_filename)))

Source = collections.namedtuple('Source', 'name fetch_callback data_format refresh_period')

def add_source(**kw):
    src = Source(**kw)
    db_root['sources'].append(src)


def service_info(service_name):
    return db_root[service_name][1]
    
def save_db():
    ofile = open(db_filename + '.new', 'w')
    sfkit_pickle_version = 2
    cPickle.dump(db_root, ofile, sfkit_pickle_version)
    os.fsync(ofile.fileno())
    ofile.flush()
    ofile.close()
    os.rename(db_filename + '.new', db_filename)

def open_scene():
    c = json_client()
    c.connect(('localhost', 4242))
    return c

def pdiff(a, b):
    def tolines(x): return pformat(x).split('\n')
    lines = difflib.unified_diff(tolines(a), tolines(b))
    return '\n'.join(lines)

def do_refresh(src):
    print 'doing fetch'
    prev_data = data = db_root[src.name][1] if src.name in db_root else None
    try:
        q = query_map.get(src.name)
        if q:
            data = src.fetch_callback(q)
        else:
            data = src.fetch_callback()
        if not data:
            data = prev_data
            print 'empty return from fetch fun, keeping previous data'
    finally:
        # do this even in case of error so failing update won't result in in
        # infinite loop. tbd: retry few times.
        db_root[src.name] = (time.time(), data)
    save_db()
    data_changed = prev_data != data
    if data_changed:
        print 'changes detected:'
        print pdiff(prev_data, data)
        
    return data_changed

def mark_stale(sname):
    try:
        old_data = db_root[sname][1]
    except KeyError, x:
        raise ValueError(x)
    db_root[sname] = (0.0, old_data)

def time_since_update(src):
    if src.name in db_root:
        return time.time() - db_root[src.name][0]
    else:
        return sys.maxint

def src_lookup(service_name):
    for src in db_root['sources']:
        if src.name == service_name:
            return src

    raise KeyError, service_name
        

def serve(cnx, service_name):
    cnx.subscribe_action(service_name, "SetQuery")
    src = src_lookup(service_name)
    print 'looked up', service_name
    first_iter = True
    did_refresh = False
    while asyncore.socket_map:
        if time_since_update(src) > src.refresh_period:
            did_refresh = True
            print 'refreshing', src.name, '...'
            sys.stdout.flush()
            updated = do_refresh(src)
            print 'done.'
        else:
            did_refresh = False
        
        if did_refresh or first_iter:
            cnx.push_service_info(service_name,
                                  service_info(service_name))
        asyncore.loop(timeout=.1, count=1)
        first_iter = False

# def serve_updateall(cnx, service_name):
#     cnx.subscribe_action(service_name, "SetQuery")
#     while True:
#         for src in db_root['sources']:
#             if time_since_update(src) > src.refresh_period:
#                 print 'refreshing', src.name, '...'
#                 sys.stdout.flush()
#                 updated = do_refresh(src)
#                 print 'done.'
#                 cnx.push_service_info(service_name,
#                                       service_info(service_name))
#         asyncore.loop(timeout=.1, count=1)
 
class json_client(asynchat.async_chat):
    def __init__(self):
        asynchat.async_chat.__init__(self)
        self.received_data = []
        self.create_socket(socket.AF_INET, socket.SOCK_STREAM)
        self.set_terminator('\r\n')
        self.response_callback = None

    def handle_error(self):
        raise

    def push_service_info(self, sname, info):
        self.encode_and_send(op="action", exectype="local", scene=None, entity=sname, action="ServiceInfo", params=[json.dumps(info), None, []])

    def subscribe_action(self, sname, aname):
        self.encode_and_send(op="subscribe_action", entity_name=sname, action_name=aname)

    def encode_and_send(self, **kw):
        if 'op' not in kw:
            raise ValueError("missing op field in message")
        data = json.dumps(kw)
        self.push(data)
        self.push("\r\n")
        print 'sent', data

    def handle_action(self, action, args):
        print 'handle_action'
        if action == 'SetQuery':
            sname = args[0]
            query = args[1]
            query_map[sname] = query
            try:
                mark_stale(sname)
            except ValueError:
                print 'action for unknown source', sname
        else:
            print 'unknown action', action

    def handle_connect(self):
        print 'connected'

    def handle_close(self):
        asyncore.close_all() # will make serve() return

    def collect_incoming_data(self, data):
        self.received_data.append(data)
    
    def found_terminator(self):
        try:
            decoded = json.loads(''.join(self.received_data))
        except ValueError, what:
            self.close_when_done()
            print 'bad json', what
        else:
            self.received_data = []
            self.received_msg(decoded)

    def received_msg(self, msg):
        print 'received msg ', msg
        if 'action' in msg:
            self.handle_action(msg['action'], msg['args'])
        f = self.response_callback
        if f:
            self.response_callback = None
            f(msg)
        
