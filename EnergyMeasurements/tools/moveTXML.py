#!/usr/bin/python

import sys,os

DELTA=-150.0

print sys.argv[1]
f = open(sys.argv[1])

fo = open("out.txml", "w")

while 1:
	line = f.readline()
	if len(line) == 0: break
	l = line.strip()
	if "Transform" in l:
		vec = l.split("\"")[1]
		v = vec.split(",")
		if float(v[2]) != 0.0:
			v[2] = str(float(v[2])+DELTA)
		l = "<attribute value=\"%s\" name=\"Transform\">" % (",".join(v))
		print l
	fo.write(l + "\n")

f.close()
fo.close()
