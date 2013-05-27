#!/usr/bin/python

import sys

fn = sys.argv[1]

result = []

f = open(fn)
count = 0
results = []
r = []
parse = 0
while 1:
    line = f.readline()
    if len(line) == 0: break
    l = line.strip()
    if l == "Results":
        #print "START"
        parse = 1
    if l == "Done!":
        #print "END"
        parse = 0
        if len(r) != 0: results.append(r)
        r = []
    if parse == 0: continue
    try: value = float(l)
    except ValueError: continue
    r.append(value)

f.close()

maxlen = 0
for i in results:
    if len(i) > maxlen: maxlen = len(i)
print maxlen

for i in range(maxlen):
    s = ""
    for j in range(len(results)):
        try:
            s += str(results[j][i])
        except IndexError:
            s += "0.0"
        s += ";"
    print s
	
