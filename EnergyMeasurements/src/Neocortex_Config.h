/**
 * Neocortex_Config.h: Global compile time flags for Neocortex
 * -----------------------------------------------------------
 *
 * Author: Jarkko Vatjus-Anttila <jvatjusanttila@gmail.com>
 *
 * For conditions of distribution and use, see copyright notice in license.txt
 */

#ifndef NEOCORTEX_CONFIG_H
#define NEOCORTEX_CONFIG_H

/******************************************************************************
 * Neocortex feature flags
 */

/// Texture format support
#define FEATURE_TEX_CRN             (1)
#define FEATURE_TEX_DDS             (1)
#define FEATURE_TEX_ETC1            (1)
#define FEATURE_TEX_JPG             (1)
#define FEATURE_TEX_PGN             (1)
#define FEATURE_TEX_TGA             (1)

/// Mesh format support
#define FEATURE_MESH_OGREXML        (1)
#define FEATURE_MESH_OBJ            (0)

/******************************************************************************
 * Neocortex platform specific extra definitions
 */

#if defined(NC_WIN32)
#define NC_MIN(a,b) min(a,b)
#ifndef M_PI
#define M_PI 3.14159265f
#endif
#ifndef _CRT_SECURE_NO_WARNINGS
#define _CRT_SECURE_NO_WARNINGS	// For removing obsolete warnings from win32 build
#endif
#endif

#if defined(NC_EGL) || defined(NC_GLX)
#define NC_MIN(a,b) std::min(a,b)
#endif

/******************************************************************************
 * Neocortex internal typedefs
 */

typedef unsigned short NC_HALFFLOAT;
typedef float NC_FLOAT;

#endif // NEOCORTEX_CONFIG_H
