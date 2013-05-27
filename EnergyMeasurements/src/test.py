#!/usr/bin/python
#
# EGLBenchmark
#
# Author: Jarkko Vatjus-Anttila <jvatjusanttila@gmail.com>
#
# For conditions of distribution and use, see copyright notice in license.txt
#

import sys, os, time
import ctypes
import math

import Neocortex_API as NC

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
    if -1 == NC.SMG_init(): return -1;
    NC.SMG_setVerbosity(verbose)
    return NC.SMG_createDisplay(xsize, ysize, fullscreen)

def destroyNeocortex():
    NC.SMG_destroyDisplay()
    NC.SMG_destroy()


def Neocortex(xsize, ysize, fullscreen, verbose, quiet=0):
    if -1 == initNeocortex(xsize, ysize, fullscreen, verbose):
        print "Neocortex init failed"
        return
    # init resources
    RM = ResourceFactory()

    mesh1   = RM.loadMesh("../resources/cube.mesh.xml")
    mesh2   = RM.loadMesh("../resources/knot.mesh.xml")
    mesh3   = RM.loadMesh("../resources/plane_nonshared.mesh.xml")
    mesh4   = RM.loadMesh("../resources/oulu3d.mesh.xml")
    t1      = RM.loadTexture("../resources/pngRGB.png")
    t2      = RM.loadTexture("../resources/front.png")
    t3      = RM.loadTexture("../resources/lightmap.tga")
    t4      = RM.loadTexture("../resources/bricks.tga")
    t5      = RM.loadTexture("../resources/sample.dds")
    t6      = RM.loadTexture("../resources/etctexture.pkm")
    mat1    = RM.loadMaterial("../resources/simpletexture.material")
    mat2    = RM.loadMaterial("../resources/lightmap.material")
    mat3    = RM.loadMaterial("../resources/flatred.material")

    obj1 = NC.SMG_createObject()
    NC.OBJ_setPosition(obj1, -2, 2, 0)
    NC.OBJ_attachMesh(obj1, mesh1)

    if 1:
        obj2 = NC.SMG_createObject()
        NC.OBJ_setPosition(obj2, 2, -2, 0)
        NC.OBJ_setScale(obj2, 0.01, 0.01, 0.01)
        NC.OBJ_attachMesh(obj2, mesh2)

        obj3 = NC.SMG_createObject()
        NC.OBJ_setPosition(obj3,  2, 2, 1)
        NC.OBJ_attachMesh(obj3, mesh1)
        NC.OBJ_attachMaterial(obj3, "dualtexture", 0)

        obj4 = NC.SMG_createObject()
        NC.OBJ_setPosition(obj4,  -2, -2, -1)
        NC.OBJ_setScale(obj4, 0.1, 0.1, 0.1)
        NC.OBJ_attachMesh(obj4, mesh4)
        NC.OBJ_attachMaterial(obj4, "flatred", 0)
        NC.OBJ_attachMaterial(obj4, "flatred", 1)
        NC.OBJ_attachMaterial(obj4, "flatred", 2)
        NC.OBJ_attachMaterial(obj4, "flatred", 3)
        NC.OBJ_attachMaterial(obj4, "flatred", 4)
        NC.OBJ_attachMaterial(obj4, "flatred", 5)
        NC.OBJ_attachMaterial(obj4, "flatred", 6)
        NC.OBJ_attachMaterial(obj4, "flatred", 7)

    camera = NC.SMG_createCamera()
    NC.SMG_setCameraPosition(camera, 0, 0, 0)
    NC.SMG_setCameraMode(camera, 1)

    #destroyNeocortex()
    #return

    # render
    frames = 0
    starttime = time.time()
    frametime1 = starttime
    rotation = 0.0
    f1 = 0.0
    f2 = 0.0
    while (1):
        if quiet == 0:
            print "... Rendering frame %d, framerendertime %f seconds" % (frames, f2-f1)
        NC.OBJ_setRotationDegrees(obj1, 100.0*math.sin(2*3.1415*rotation/360.0), 0, 0)
        NC.OBJ_setRotationDegrees(obj2, 100.0*math.sin(2*3.1415*rotation/360.0), rotation, 0)
        NC.OBJ_setRotationDegrees(obj3, 0, 0, rotation)
        NC.OBJ_setRotationDegrees(obj4, 0, 0, 2*rotation)
        rotation += 1.0
        frametime2 = time.time()
        f1 = time.time()
        if -1 == NC.SMG_renderFrameAndSwap(frametime2-frametime1):
            break
        f2 = time.time()
        frames += 1
        #print frametime2-frametime1
        frametime1 = frametime2
        #if frames == 2: break
    endtime = time.time()
    print "Rendered %d frames in %f second, hence %f FPS" % (frames, endtime-starttime, frames/(endtime-starttime))
    #print "Delay three seconds"; time.sleep(3)
    # clean resources
    NC.SMG_destroyCamera(camera)
    NC.SMG_destroyObject(obj1)
    NC.SMG_destroyObject(obj2)
    NC.SMG_destroyObject(obj3)
    #NC.SMG_destroyObject(obj4)
    if 1:
        RM.destroy(mesh1)
        RM.destroy(mesh2)
        RM.destroy(mesh3)
        RM.destroy(t1)
        RM.destroy(t2)
        RM.destroy(t3)
        RM.destroy(t4)
        RM.destroy(mat1)
        RM.destroy(mat2)
        RM.destroy(mat3)
    destroyNeocortex()

if __name__ == "__main__":
    import getopt

    def usage():
        print "USAGE: test.py"

    try:
        opts, args = getopt.getopt(sys.argv[1:], "x:y:fv:q",
            ["xsize=", "ysize=", "fullscreen", "verbose=", "quiet"])
    except getopt.GetoptError, err:
        print "Options error"
        usage()
        sys.exit(0)

    xsize = 800
    ysize = 480
    fullscreen = 0
    verbose = 1
    quiet = 0
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
        if o in ("-q", "--quiet"):
            quiet = 1

    Neocortex(xsize, ysize, fullscreen, verbose, quiet)
    print "Done!"
