#!/usr/bin/python

import sys

fn = sys.argv[1]

result = []

f = open(fn)
count = 0
r = []
while 1:
    line = f.readline()
    count += 1
    if len(line) == 0: break
    l = line.strip().split(" ")
    if l[0].split(";")[0] == "---":
        #print "start"
        r = ["-".join(l[0].split(";")[1:])]
        continue
    if l[0].split(";")[0] == "+++":
        result.append(r)
        r = []
        #print "end"
        continue
    if l[0] == "CURRENT":
        #print "append"
        r.append(float(l[1]))

maxlen = 0
#print result
for i in result:
    if len(i) > maxlen: maxlen = len(i)
print maxlen
for i in range(maxlen):
    s = ""
    for j in range(len(result)):
        try:
            s += str(result[j][i])
        except IndexError:
            s += "0.0"
        s += ";"
    print s

