#!/usr/bin/env python

import worsen

import sys, re
import traceback
import urllib2
import mimetools
from cStringIO import StringIO
from gevent.server import StreamServer
from gevent.pool import Pool
import gevent.socket
from socket import IPPROTO_TCP, TCP_NODELAY
from urlparse import urlparse
import cgi
from collections import namedtuple

import time

def print_elapsed(start_time, text):
    e = time.time() - start_time
    print text % (e,)

PROXY_LISTEN_PORT = 8088

import BaseHTTPServer
# Yes, this is part of BaseHTTPServer's public interface
http_responses = BaseHTTPServer.BaseHTTPRequestHandler.responses

status_bad_request = 400
status_bad_gateway = 502

def send_response_headers(bs, statusline, hdr_items):
    def checknl(s):
        if '\n' in s:
            raise ValueError, 'unexpected newline in header: %s' % repr(s)

    checknl(statusline)
    out = 'HTTP/1.1 ' + statusline + '\r\n'
    for k, v in hdr_items:
        s = '%s: %s\r\n' % (k, v)
        checknl(s[:-2])
        out += s
    out += '\r\n'
    bs.write(out)
    print 'sending', out
    

def send_error_and_close(bs, statuscode):
    shortmsg, longmsg = http_responses[statuscode]
    bs.write('%d %s\r\n'
                'Content-Type: text/plain\r\n'
                'Connection: close\r\n'
                '\r\n'
                '%s\r\n' % (statuscode, shortmsg, longmsg))

def parse_request(raw_requestline, bs):
    reqline_tokens = raw_requestline.split()
    badreq = lambda: send_error_and_close(bs, status_bad_request)
    sanity_check_predicates = [
        lambda: raw_requestline[-1] in '\r\n',
        lambda: len(reqline_tokens) > 1 and reqline_tokens[-1].upper().startswith('HTTP/'),
        lambda: 2 <= len(reqline_tokens) <= 3, # method [path] version
        ]
        
    for p in sanity_check_predicates:
        if not p():
            badreq()
            return None

    method = reqline_tokens[0]
    if len(reqline_tokens) == 3:
        path = reqline_tokens[1]
    else:
        path = None

    header_sep = re.compile(r"\r?\n\r?\n", re.MULTILINE)
    headerbuf = bs.read_until(header_sep)
    print 'headerbuf', repr(headerbuf)
    sio_fp = StringIO(headerbuf)
    req_headers = mimetools.Message(sio_fp)
    print 'headers done'
    cl = req_headers.get("content-length", "")
    length_ok = cl.isdigit()
    if cl and not length_ok:
        badreq()
        return
    if length_ok:
        if method in ('GET', 'HEAD'):
            badreq()
            return
        req_body = bs.read_bytes(int(cl))

    elif req_headers.get("transfer-encoding"):
        if method in ('GET', 'HEAD'):
            badreq()
            return
        req_body = bs.read_all()
    else:
        req_body = None

    pc_status = req_headers.get('proxy-connection', '').lower().strip()
    cc_status = req_headers.get('connection', '').lower().strip()

    if pc_status == 'keep-alive':
        closed = False
    elif pc_status == '' and cc_status == 'keep-alive':
        closed = False
    else:
        closed = True

    return http_req(method.upper(), path, req_headers, req_body, closed)

http_req = namedtuple('http_req', 'method path headers req_body closed')

class fake_sock(object):
    def recv(self, n):
        return ''

    def sendall(self, data):
        print 'fake-sending', repr(data[:100])
        return len(data)
    
    def close(self):
        return

class buffered_socket(object):
    def __init__(self, socket):
        self.sock = socket
        self.buf = ''
        self.closed = False
        self.keep_alive = False

    def read_until(self, sep='\n', maxlen=sys.maxint):
        if isinstance(sep, basestring):
            def getoffset(): return self.buf.find(sep)
        else:
            def getoffset():
                m = sep.search(self.buf)
                return m.end() if m else -1
            
        nloffset = None
        while True:
            nloffset = getoffset()
            if nloffset != -1:
                break

            data = self.sock.recv(32768)
            if data == '': # eof
                self.closed = True
                break
            self.buf += data
            if len(self.buf) > maxlen:
                return self.get_bytes(maxlen)

        if nloffset == -1:
            # no newline found, we must be here because of EOF
            s = self.buf
            self.buf = ''
            return s
        else:
            nloffset += 1
            s = self.buf[:nloffset]
            self.buf = self.buf[nloffset:]
            return s

    def read_line(self, *args, **kw):
        return self.read_until('\n', *args, **kw)

    def read_bytes(self, n):
        while len(self.buf) < n:
            nread = min(32768, n-len(self.buf))
            data = self.sock.recv(nread)
            if data == '': # eof
                self.closed = True
                break
            self.buf += data

        s = self.buf[:n]
        self.buf = self.buf[n:]
        return s

    def read_all(self):
        return self.read_bytes(sys.maxint)

    def write(self, data):
        self.sock.sendall(data)

    def close(self):
        self.sock.close()
        print 'CLOSING SOCKET'
        self.closed = True

def new_connection(sock, address):
    bs = buffered_socket(sock)
    sock.settimeout(20.0)
    sock.setsockopt(IPPROTO_TCP, TCP_NODELAY, 1)
    new_connection2(bs, address)

def mock_connection(indata, address):
    bs = buffered_socket(fake_sock())
    bs.buf = indata
    new_connection2(bs, address)

def new_connection2(bs, address):

    while True:
        print 'reading request...'
        try:
            raw_requestline = bs.read_line(65537)
            # spec says we SHOULD ignore at least 1 empty line:
            if raw_requestline.replace('\r', '') == '\n':
                raw_requestline = bs.read_line(65537)

            if len(raw_requestline) > 65536:
                send_error_and_close(bs, 414)
                return

            if not raw_requestline:
                bs.close()
                return

            req = parse_request(raw_requestline, bs)
            if req is None:
                continue
        except gevent.socket.timeout, e:
            #a read or a write timed out.  Discard this connection
            print "Request timed out:", e
            break

        print 'proxying request'
        t = time.time()
        do_proxy(req, bs)
        print_elapsed(t, "do_proxy took %.2f seconds")
        print 'proxying done'
        if not bs.keep_alive:
            break

    bs.close()

def do_proxy(req, bs):
    parsed = urlparse(req.path)
    try:
        if parsed.scheme != 'http':
            send_error_and_close(bs, status_bad_gateway) 
            return 
        try:
            response = urllib2.urlopen(req.path)
        except urllib2.HTTPError, ex:
            response = ex
        print '%s: %s %s' % (req.path, response.code, response.msg)
        host = parsed.scheme + '://' + parsed.netloc
    except Exception, ex:
        sys.stderr.write('error while reading %s:\n' % req.path)
        traceback.print_exc()
        tb = traceback.format_exc()
        send_error_and_close(bs, status_bad_gateway)
        error_str = cgi.escape(str(ex) or ex.__class__.__name__ or 'Error')
        return ['<h1>%s</h1><h2>%s</h2><pre>%s</pre>' % (error_str, cgi.escape(req.path), cgi.escape(tb))]
    else:
        
        data = response.read()

        # headers that are processed by us or urllib2
        delete_headers = 'transfer-encoding proxy-connection'
        for tekey in delete_headers.split():
            if tekey in response.headers:
                del response.headers[tekey]

        xc_id = response.headers.get('x-transcode-profile-id', '')
        xcode_profile = get_profile(xc_id)
        if response.headers.get('content-type', '').lower() == 'image/jpeg':
            print 'jpeg seen, worsening it...'
            data = worsen.worsen_jpeg(data, xcode_profile)
            response.headers['content-length'] = str(len(data))

        if response.headers.get('content-type', '').lower() in ('image/png', 'image/gif'):
            print 'png or gif seen, worsening it...'
            data = worsen.worsen_png(data, xcode_profile)
            response.headers['content-length'] = str(len(data))
        send_response_headers(bs, '%s %s' % (response.code, response.msg),
                      response.headers.items())
        bs.write(data)

profiles = {
    'test-1': { 'jpeg-quality': 25, 'png-colors': 32},
    'test-2': { 'jpeg-quality': 5, 'png-colors': 8 }
}
 
def get_profile(id):
    return profiles.get(id, {})

def join(url1, *rest):
    if not rest:
        return url1
    url2, rest = rest[0], rest[1:]
    if url1.endswith('/'):
        if url2.startswith('/'):
            return join(url1 + url2[1:], *rest)
        else:
            return join(url1 + url2, *rest)
    elif url2.startswith('/'):
        return join(url1 + url2, *rest)
    else:
        return join(url1 + '/' + url2, *rest)

def test_simple_req():
    req_body = ('GET http://chiru.cie.fi/ HTTP/1.0\r\n'
                'Host: chiru.cie.fi\r\n'
                '\r\n')
    mock_connection(req_body, ('127.0.0.9', 2334))
    
if __name__ == '__main__':
    if '-t' in sys.argv:
        test_simple_req()
    else:
        print 'Serving on %s...' % PROXY_LISTEN_PORT
        pool = Pool(10000) # set a maximum for concurrency
        print 'using timeout', gevent.socket.getdefaulttimeout()
        StreamServer(('', PROXY_LISTEN_PORT), new_connection, spawn=pool).serve_forever()
