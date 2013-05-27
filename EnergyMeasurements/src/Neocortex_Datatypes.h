/*
 * Neocortex
 *
 * Author: Jarkko Vatjus-Anttila <jvatjusanttila@gmail.com>
 *
 * For conditions of distribution and use, see copyright notice in license.txt
 */

#ifndef NEOCORTEX_DATATYPES_H
#define NEOCORTEX_DATATYPES_H

typedef float           __NC_FLOAT_PRIMTYPE;
typedef unsigned int    __NC_FIXED_PRIMTYPE;

typedef __NC_FLOAT_PRIMTYPE     VERTEX;
typedef __NC_FLOAT_PRIMTYPE     NORMAL;
typedef __NC_FLOAT_PRIMTYPE     TEXCOORD;
typedef __NC_FLOAT_PRIMTYPE     COLOR;
typedef __NC_FIXED_PRIMTYPE     INDEX;

typedef enum {
    NC_UINT16 = 1,
    NC_UINT32,
    NC_FLOAT16,
    NC_FLOAT32
} NC_DATATYPE;

#endif // NEOCORTEX_DATATYPES_H
