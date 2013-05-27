#!/usr/bin/python

import subprocess
import os, sys, time
import signal

import Queue
import socket
import threading

if 0:
    p1 = subprocess.Popen(["/home/trim/projects/Neocortex-CIE/tools/ut60.py"], stdout=subprocess.PIPE)
    print "PID: %d" % p1.pid
    for line in iter(p1.stdout.readline,''):
        print line.rstrip()    #os.killpg(proc.pid, signal.SIGTERM)
    sys.exit(0)

    p2 = subprocess.Popen(["grep", "disc"], stdin=p1.stdout, stdout=subprocess.PIPE)
    p1.stdout.close()  # Allow p1 to receive a SIGPIPE if p2 exits.
    output = p2.communicate()[0]
    print output
    sys.exit(0)

if 0:
    proc = subprocess.Popen(['./ut60'], stdout=subprocess.PIPE, preexec_fn=os.setsid)
    print "pid %d" % proc.pid
    try:
        while 1:
            line = proc.stdout.readline().rstrip()
            proc.stdout.flush()
            print "line --- %s" % line
            #for line in iter(proc.stdout.readline, ''):
            #print line.rstrip()
    except KeyboardInterrupt:
        print "terminate"
        os.killpg(proc.pid, signal.SIGTERM)
    sys.exit(0)

class CurrentMeasurement():
    def __init__(self):
        self.signalQueue = Queue.Queue(0)
        self.dataQueue = Queue.Queue(0)
        self.results = []

    def start(self):
        self.UT61ERead(self.signalQueue, self.dataQueue).start()

    def stop(self):
        self.signalQueue.put("X")
        self.results = []
        while 1:
            try:
                self.results.append(self.dataQueue.get(block=False))
            except Queue.Empty:
                break

    def printResults(self):
        print "Results"
        for r in self.results:
            print r

    class UT61ERead(threading.Thread):
        # Override Thread's __init__ method to accept the parameters needed:
        def __init__(self, iQueue, oQueue):
            print "UT60E read thread init"
            self.values = []
            self.iQueue = iQueue
            self.oQueue = oQueue
            threading.Thread.__init__ ( self )

        def run(self):
            print "UT60E: Starting ut60 reading process"
            proc = subprocess.Popen(['/home/trim/projects/Neocortex-CIE/tools/ut60e'], stdout=subprocess.PIPE, preexec_fn=os.setsid)
            print "UT60E: %d" % proc.pid
            #
            for line in iter(proc.stdout.readline,''):
                try:
                    value = float(line.rstrip())
                except ValueError:
                    value = 0.0
                print "value: %f" % value
                self.oQueue.put(value)
                #
                try:
                    s = self.iQueue.get(block=False)
                    break
                except Queue.Empty:
                    pass
            #
            print "UT60E: Gracefully existing PID %d" % proc.pid
            os.killpg(proc.pid, signal.SIGTERM)
            print "UT60E: Terminating thread"

if __name__ == "__main__":
    cm = CurrentMeasurement()
    cm.start()
    time.sleep(5)
    cm.stop()
    cm.printResults()
    print "Done!"
