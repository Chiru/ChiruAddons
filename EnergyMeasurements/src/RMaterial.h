
/* Neocortex
 *
 * Author: Jarkko Vatjus-Anttila <jvatjusanttila@gmail.com>
 *
 * For conditions of distribution and use, see copyright notice in license.txt
 */

#ifndef RMaterial_H
#define RMaterial_H

#include "Neocortex_GLHeaders.h"

#include <string>

#include "ResourceManager.h"
#include "GLWrapper.h"


typedef enum {
    MAT_AMODE_WRAP = 1,
    MAT_AMODE_CLAMP,
    MAT_AMODE_MIRROR,
    MAT_AMODE_BORDER
} MAT_AMODE;

typedef enum {
    MAT_FILTER_NONE = 1,
    MAT_FILTER_BILINEAR,
    MAT_FILTER_TRILINEAR,
    MAT_FILTER_ANISOTROPIC
} MAT_FILTER;

typedef enum {
    MAT_AREJ_ALWAYSPASS = 1,
    MAT_AREJ_ALWAYSREJECT,
    MAT_AREJ_LESS,
    MAT_AREJ_LESSEQUAL,
    MAT_AREJ_EQUAL,
    MAT_AREJ_NOTEQUAL,
    MAT_AREJ_GREATEREQUAL,
    MAT_AREJ_GREATER
} MAT_AREJ;


/// Class definition
class RMaterial : public Resource
{
public:
    RMaterial(int id);
    ~RMaterial();

    /// Mandatory resource API
    int bind(void);
    int destroy(void);
    int render(float deltatime);

    /// Color information
    int setAmbient(float r, float g, float b) {
        DEBUG_NOTIFICATION("Setting ambient color to %f %f %f\n", r, g, b);
        amb_r = r; amb_g = g; amb_b = b; return 0;
    }
    int setDiffuse(float r, float g, float b) {
        DEBUG_NOTIFICATION("Setting diffuse color to %f %f %f\n", r, g, b);
        diff_r = r; diff_g = g; diff_b = b; return 0;
    }
    int setSpecular(float r, float g, float b) {
        DEBUG_NOTIFICATION("Setting specular color to %f %f %f\n", r, g, b);
        spec_r = r; spec_g = g; spec_b = b; return 0;
    }
    int setEmissive(float r, float g, float b) {
        DEBUG_NOTIFICATION("Setting emissive color to %f %f %f\n", r, g, b);
        emis_r = r; emis_g = g; emis_b = b; return 0;
    }

    /// Special parameters
    int setSpecularPower(float power) {
        DEBUG_NOTIFICATION("Setting specular power %f\n", power);
        specularPower = power; return 0;
    }

    int setAlphaRejection(MAT_AREJ mode, float limit);

    /// Texture specific params
    int setTextureName(int order, const char *tex);
    int setTextureAddressingmode(int order, MAT_AMODE mode);
    int setTextureFilter(int order, MAT_FILTER filter);

private:
    /// Methods
    int countTextureChannels(void);

    /// Color variables
    float amb_r, amb_g, amb_b;
    float spec_r, spec_g, spec_b;
    float diff_r, diff_g, diff_b;
    float emis_r, emis_g, emis_b;

    /// Special variables
    float specularPower;
    float alphaRejection;
    MAT_AREJ alphaRejectionMode;

    /// Texture attributes
    unsigned short n_textures;
    char textures[4][64];
    MAT_AMODE tex_addressingmode[4];
    MAT_FILTER tex_filter[4];
};

#endif // RMaterial_H
