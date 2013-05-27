#!/usr/bin/python
#
# TXMLLoader
#
# Author: Jarkko Vatjus-Anttila <jvatjusanttila@gmail.com>
#
# For conditions of distribution and use, see copyright notice in license.txt
#

import sys, os, time
import ctypes
import math

import Neocortex_API as NC

import pyUT61E

# 0: rotation in middle of city
# 1: flyby one side to another
# 2: circle view of the city
TESTCASE = 2

class ResourceFactory():
    def __init__(self):
        pass

    def __loadResource(self, t, filename):
        r = NC.RES_createResource(t, filename)
        return r;

    def loadMesh(self, filename):
        return self.__loadResource(1, filename)

    def loadTexture(self, filename):
        return self.__loadResource(2, filename)

    def loadMaterial(self, filename):
        return self.__loadResource(4, filename)

    def destroy(self, resource):
        NC.RES_deleteResource(resource)


def initNeocortex(xsize, ysize, fullscreen, verbose):
    NC.SMG_init()
    NC.SMG_setVerbosity(verbose)
    NC.SMG_createDisplay(xsize, ysize, fullscreen)

def destroyNeocortex():
    NC.SMG_destroyDisplay()
    NC.SMG_destroy()

class TXMLimport():
    def __init__(self):
        self.currentObject = 0
        self.currentPlaceable = 0
        self.currentMesh = 0
        self.RM = ResourceFactory()
    def msg(self, m):
        #print m
        return
    def fromFile(self, filename, path):
        f = open(filename)
        while 1:
            line = f.readline()
            if len(line) == 0: break
            l = line.strip()
            arg = l.split("\"")
            try:
                if arg[0] == "</entity>":
                    self.msg("Closing object")
                    #time.sleep(1)
                    if self.currentObject != 0:
                        NC.OBJ_attachMesh(self.currentObject, self.currentMesh)
                        if self.currentPlaceable == 2:
                            NC.OBJ_setPosition(self.currentObject, self.pos_x, self.pos_y, self.pos_z)
                            NC.OBJ_setRotationDegrees(self.currentObject, self.rot_x, self.rot_y, self.rot_z)
                            NC.OBJ_setScale(self.currentObject, self.sca_x, self.sca_y, self.sca_z)
                        if self.incomingMeshTransform == 2:
                            NC.OBJ_setMeshLocalPosition(self.currentMesh, self.mesh_pos_x, self.mesh_pos_y, self.mesh_pos_z)
                            NC.OBJ_setMeshLocalRotationDegrees(self.currentMesh, self.mesh_rot_x, self.mesh_rot_y, self.mesh_rot_z)
                            NC.OBJ_setMeshLocalScale(self.currentMesh, self.mesh_sca_x, self.mesh_sca_y, self.mesh_sca_z)
                    self.currentObject = 0
                    self.currentPlaceable = 0
                    self.currentMesh = 0
                    self.incomingMeshTransform = 0
                elif arg[1] == "EC_Mesh":
                    self.msg("Starting new object")
                    self.currentObject = NC.SMG_createObject()
                    self.incomingMeshTransform = 1
                elif arg[1] == "EC_Placeable":
                    self.msg(" Incoming placeable")
                    self.currentPlaceable = 1
                elif arg[1] == "Mesh ref" or arg[3] == "Mesh ref":
                    if self.currentObject == 0:
                        self.msg(" Mesh ref, but no object")
                        continue
                    if arg[1] == "Mesh ref": n = arg[3]
                    else: n = arg[1]
                    self.msg(" Injecting mesh ref: %s/%s" % (path, n))
                    self.currentMesh = self.RM.loadMesh(path+"/"+n)
                elif arg[1] == "Mesh materials" or arg[3] == "Mesh materials":
                    if self.currentObject == 0:
                        self.msg(" Material ref, but no object")
                        continue
                    if arg[1] == "Mesh materials": n = arg[3]
                    else: n = arg[1]
                    self.msg(" Injecting materials:")
                    #materials = n.split(";")
                    submesh = 0
                    for m in n.split(";"):
                        if len(m) == 0: continue
                        self.msg(" %s/%s" % (path, m))
                        self.RM.loadMaterial(path+"/"+m)
                        self.parseMaterialTextures(path+"/"+m, path)
                        self.forceMaterial(self.currentObject, path+"/"+m, submesh)
                        submesh += 1
                elif (arg[1] == "Transform" and self.currentPlaceable == 1) or (arg[3] == "Transform" and self.currentPlaceable == 1):
                    self.msg(" new placeable:")
                    if arg[1] == "Transform": transform = arg[3].split(",")
                    else: transform = arg[1].split(",")
                    self.pos_x = float(transform[0])
                    self.pos_y = float(transform[1])
                    self.pos_z = float(transform[2])
                    self.rot_x = float(transform[3])
                    self.rot_y = float(transform[4])
                    self.rot_z = float(transform[5])
                    self.sca_x = float(transform[6])
                    self.sca_y = float(transform[7])
                    self.sca_z = float(transform[8])
                    self.msg("  position: %f %f %f" % (self.pos_x, self.pos_y, self.pos_z))
                    self.msg("  rotation: %f %f %f" % (self.rot_x, self.rot_y, self.rot_z))
                    self.msg("     scale: %f %f %f" % (self.sca_x, self.sca_y, self.sca_z))
                    self.currentPlaceable = 2
                elif (arg[1] == "Transform" and self.incomingMeshTransform == 1) or (arg[3] == "Transform" and self.incomingMeshTransform == 1):
                    self.msg(" new mesh transform:")
                    if arg[1] == "Transform": transform = arg[3].split(",")
                    else: transform = arg[1].split(",")
                    self.mesh_pos_x = float(transform[0])
                    self.mesh_pos_y = float(transform[1])
                    self.mesh_pos_z = float(transform[2])
                    self.mesh_rot_x = float(transform[3])
                    self.mesh_rot_y = float(transform[4])
                    self.mesh_rot_z = float(transform[5])
                    self.mesh_sca_x = float(transform[6])
                    self.mesh_sca_y = float(transform[7])
                    self.mesh_sca_z = float(transform[8])
                    self.msg("  position: %f %f %f" % (self.mesh_pos_x, self.mesh_pos_y, self.mesh_pos_z))
                    self.msg("  rotation: %f %f %f" % (self.mesh_rot_x, self.mesh_rot_y, self.mesh_rot_z))
                    self.msg("     scale: %f %f %f" % (self.mesh_sca_x, self.mesh_sca_y, self.mesh_sca_z))
                    self.incomingMeshTransform = 2
            except IndexError:
                pass
        f.close()
        return 0
    def parseMaterialTextures(self, filename, path):
        f = open(filename)
        while 1:
            line = f.readline()
            if len(line) == 0: break
            l = line.strip()
            arg = l.split(" ")
            if arg[0] == "texture":
                #print "Loading texture: %s" % (path+"/"+arg[1])
                self.RM.loadTexture(path+"/"+arg[1])

    def forceMaterial(self, obj, filename, submesh):
        f = open(filename)
        while 1:
            line = f.readline()
            if len(line) == 0: break;
            l = line.strip()
            if l.split(" ")[0] == "material":
                m_name = l.split(" ")[1]
                #print "Forcing material %s for submesh %d" % (m_name, submesh)
                NC.OBJ_attachMaterial(obj, m_name, submesh)
                break;
        f.close()


def Neocortex(xsize, ysize, fullscreen, verbose, txml, path):
    initNeocortex(xsize, ysize, fullscreen, verbose)
    # init resources
    RM = ResourceFactory()
    now = time.time()
    TX = TXMLimport()
    if TX.fromFile(txml, path) != 0:
        print "Error loading %s" % txml
        destroyNeortex()
        return
    print "Scene and its resources loaded in %f seconds" % (time.time()-now)
    print "Now rendering 1000 frames"

    #-317.680450,126.583626,-80.587845,-27.599993,-99.299980,0.000000
    camera = NC.SMG_createCamera()
    # For complete oulu3d scene
    if TESTCASE == 0:
        NC.SMG_setCameraPosition(camera, -111, 11, -83)
        NC.SMG_setCameraRotationDegrees(camera, 0, 0, 0)
    if TESTCASE == 1:
        NC.SMG_setCameraPosition(camera, -60, 33, 50)
        NC.SMG_setCameraRotationDegrees(camera, 0, 0, 0)
    # for testing
    #NC.SMG_setCameraPosition(camera, 11, 34, 58)
    #NC.SMG_setCameraRotationDegrees(camera, 0, 19, 3)
    NC.SMG_setCameraMode(camera, 1)

    #destroyNeocortex()
    #return

    time.sleep(0.1)
    cm = pyUT61E.CurrentMeasurement()
    cm.start()

    # render
    frames = 0
    starttime = time.time()
    currenttime = starttime
    frametime1 = starttime
    rotation = 0.0
    position = 0.0
    fpslimit = 1/20.0
    while (1):
        #print "... Rendering frame %d" % frames
        #NC.OBJ_setRotation(obj1, 100.0*math.sin(2*3.1415*100*rotation), 0, 0)
        #NC.OBJ_setRotation(obj2, 100.0*math.sin(2*3.1415*100*rotation), rotation, 0)
        #NC.OBJ_setRotation(obj3, 0, 0, rotation)
        #rotation += 1.0
        frametime2 = time.time()
        if -1 == NC.SMG_renderFrameAndSwap(frametime2-frametime1):
            break
        frames += 1
        #print frametime2-frametime1
        if fpslimit > 0.0 and (frametime2-frametime1) < fpslimit:
            #print "fpslimit %f %f, sleeping %f" % (frametime2-frametime1, fpslimit, fpslimit-frametime2+frametime1)
            time.sleep(fpslimit-frametime2+frametime1)
            frametime2 = time.time()
        #if frames == 1000: break
        currenttime += (frametime2-frametime1)
        #if (currenttime-starttime > 30.0): break
        if TESTCASE == 0: # rotate
            rotation = (currenttime-starttime)*360.0/30.0
            NC.SMG_setCameraRotationDegrees(camera, 0, rotation, 0)
        if TESTCASE == 1: # flyby
            position = (currenttime-starttime)*19.0
            NC.SMG_setCameraPosition(camera, -60, 33, 200-position)
        if TESTCASE == 2: # circle
            angle = (currenttime-starttime)/30.0*2*math.pi
            r = 180.0
            NC.SMG_setCameraPosition(camera, -60+r*math.sin(angle), 33, 0+r*math.cos(angle))
        frametime1 = frametime2
        #break
    endtime = time.time()
    cm.stop()
    print "Rendered %d frames in %f second, hence %f FPS" % (frames, endtime-starttime, frames/(endtime-starttime))
    #print "Delay three seconds"; time.sleep(3)
    # clean resources
    destroyNeocortex()
    cm.printResults()

if __name__ == "__main__":
    import getopt

    def usage():
        print "USAGE: test.py"

    try:
        opts, args = getopt.getopt(sys.argv[1:], "x:y:fv:t:p:",
            ["xsize=", "ysize=", "fullscreen", "verbose=", "txml=", "path="])
    except getopt.GetoptError, err:
        print "Options error"
        usage()
        sys.exit(0)

    xsize = 800
    ysize = 480
    fullscreen = 0
    verbose = 1
    txml = ""
    path = ""
    for o, a in opts:
        if o in ("-x", "--xsize"):
            xsize = int(a)
            if xsize < 1: xsize = 1;
        if o in ("-y", "--xyize"):
            ysize = int(a)
            if ysize < 1: ysize = 1;
        if o in ("--fullscreen"):
            fullscreen = 1
        if o in ("-v", "--verbose"):
            verbose = int(a)
            if verbose < 1: verbose = 1
            if verbose > 5: verbose = 5
        if o in ("-t", "--txml"):
            txml = str(a)
        if o in ("-p", "--path"):
            path = str(a)

    Neocortex(xsize, ysize, fullscreen, verbose, txml, path)
    print "Done!"
