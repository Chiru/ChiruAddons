/* Neocortex
 *
 * Author: Jarkko Vatjus-Anttila <jvatjusanttila@gmail.com>
 *
 * For conditions of distribution and use, see copyright notice in license.txt
 */

#include "RMesh.h"

#include "GLWrapper.h"
#include "DebugLog.h"
#include "GLMath.h"

#include <iostream>
#include <fstream>
#include <sys/stat.h>

#include <stdlib.h>
#include <string.h>

int RMesh::setMeshFromOBJ(const char *filename)
{
    DEBUG_CRITICAL("Loading of Wavefront OBJ not yet supported\n");
    return -1;
}

