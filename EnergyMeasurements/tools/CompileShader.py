#!/usr/bin/python

# $1 = inputfile
# $2 = outputfile

import sys
import os

f = open(sys.argv[1], "r")
o = "const char " + sys.argv[1].split(".")[-2].split(os.sep)[-1] + "_" + sys.argv[1].split(".")[-1] + "[] ="

while 1:
	line = f.readline()
	if len(line) == 0:
		o += ";\n"
		break
	o += "\n"
	o = o + "    \"" + line.strip() + "\\n\""
f.close()
#print o

f = open(sys.argv[2], "w")
f.write(o)
f.close()	
