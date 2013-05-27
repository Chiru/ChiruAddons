
/* Neocortex
 *
 * Author: Jarkko Vatjus-Anttila <jvatjusanttila@gmail.com>
 *
 * For conditions of distribution and use, see copyright notice in license.txt
 */

#include "RTexture.h"
#include "GLWrapper.h"
#include "DebugLog.h"
#include "Scenegraph.h"

#include <iostream>
#include <fstream>
#include <sys/stat.h>

#include <stdlib.h>
#include <string.h>


// Following are re-definitions from crnlib.h. The file could have been included here directly
// but the current crunch does not compile on linux. Maybe will be reworked later.
#define CRNLIB_PIXEL_FMT_FOURCC(a, b, c, d) ((a) | ((b) << 8U) | ((c) << 16U) | ((d) << 24U))
typedef unsigned int crn_uint32;
typedef int crn_int32;
typedef unsigned char crn_uint8;


namespace crnlib
{
   enum pixel_format
   {
      PIXEL_FMT_INVALID               = 0,

      PIXEL_FMT_DXT1                  = CRNLIB_PIXEL_FMT_FOURCC('D', 'X', 'T', '1'),
      PIXEL_FMT_DXT2                  = CRNLIB_PIXEL_FMT_FOURCC('D', 'X', 'T', '2'),
      PIXEL_FMT_DXT3                  = CRNLIB_PIXEL_FMT_FOURCC('D', 'X', 'T', '3'),
      PIXEL_FMT_DXT4                  = CRNLIB_PIXEL_FMT_FOURCC('D', 'X', 'T', '4'),
      PIXEL_FMT_DXT5                  = CRNLIB_PIXEL_FMT_FOURCC('D', 'X', 'T', '5'),
      PIXEL_FMT_3DC                   = CRNLIB_PIXEL_FMT_FOURCC('A', 'T', 'I', '2'), // DXN_YX
      PIXEL_FMT_DXN                   = CRNLIB_PIXEL_FMT_FOURCC('A', '2', 'X', 'Y'), // DXN_XY
      PIXEL_FMT_DXT5A                 = CRNLIB_PIXEL_FMT_FOURCC('A', 'T', 'I', '1'), // ATI1N, http://developer.amd.com/media/gpu_assets/Radeon_X1x00_Programming_Guide.pdf

      // Non-standard, crnlib-specific pixel formats (some of these are supported by ATI's compressonator)
      PIXEL_FMT_DXT5_CCxY             = CRNLIB_PIXEL_FMT_FOURCC('C', 'C', 'x', 'Y'),
      PIXEL_FMT_DXT5_xGxR             = CRNLIB_PIXEL_FMT_FOURCC('x', 'G', 'x', 'R'),
      PIXEL_FMT_DXT5_xGBR             = CRNLIB_PIXEL_FMT_FOURCC('x', 'G', 'B', 'R'),
      PIXEL_FMT_DXT5_AGBR             = CRNLIB_PIXEL_FMT_FOURCC('A', 'G', 'B', 'R'),

      PIXEL_FMT_DXT1A                 = CRNLIB_PIXEL_FMT_FOURCC('D', 'X', '1', 'A'),

      PIXEL_FMT_R8G8B8                = CRNLIB_PIXEL_FMT_FOURCC('R', 'G', 'B', 'x'),
      PIXEL_FMT_L8                    = CRNLIB_PIXEL_FMT_FOURCC('L', 'x', 'x', 'x'),
      PIXEL_FMT_A8                    = CRNLIB_PIXEL_FMT_FOURCC('x', 'x', 'x', 'A'),
      PIXEL_FMT_A8L8                  = CRNLIB_PIXEL_FMT_FOURCC('L', 'x', 'x', 'A'),
      PIXEL_FMT_A8R8G8B8              = CRNLIB_PIXEL_FMT_FOURCC('R', 'G', 'B', 'A')
   };

   const crn_uint32 cDDSMaxImageDimensions = 8192U;

   // Total size of header is sizeof(uint32)+cDDSSizeofDDSurfaceDesc2;
   const crn_uint32 cDDSSizeofDDSurfaceDesc2 = 124;

   // "DDS "
   const crn_uint32 cDDSFileSignature = 0x20534444;

   struct DDCOLORKEY
   {
      crn_uint32 dwUnused0;
      crn_uint32 dwUnused1;
   };

   struct DDPIXELFORMAT
   {
      crn_uint32 dwSize;
      crn_uint32 dwFlags;
      crn_uint32 dwFourCC;
      crn_uint32 dwRGBBitCount;     // ATI compressonator and crnlib sometimes place a FOURCC code here
      crn_uint32 dwRBitMask;
      crn_uint32 dwGBitMask;
      crn_uint32 dwBBitMask;
      crn_uint32 dwRGBAlphaBitMask;
   };

   struct DDSCAPS2
   {
      crn_uint32 dwCaps;
      crn_uint32 dwCaps2;
      crn_uint32 dwCaps3;
      crn_uint32 dwCaps4;
   };

   struct DDSURFACEDESC2
   {
      crn_uint32 dwSize;
      crn_uint32 dwFlags;
      crn_uint32 dwHeight;
      crn_uint32 dwWidth;
      union
      {
         crn_int32 lPitch;
         crn_uint32 dwLinearSize;
      };
      crn_uint32 dwBackBufferCount;
      crn_uint32 dwMipMapCount;
      crn_uint32 dwAlphaBitDepth;
      crn_uint32 dwUnused0;
      crn_uint32 lpSurface;
      DDCOLORKEY unused0;
      DDCOLORKEY unused1;
      DDCOLORKEY unused2;
      DDCOLORKEY unused3;
      DDPIXELFORMAT ddpfPixelFormat;
      DDSCAPS2 ddsCaps;
      crn_uint32 dwUnused1;
   };

#if 0

   const crn_uint32 DDSD_CAPS                   = 0x00000001;
   const crn_uint32 DDSD_HEIGHT                 = 0x00000002;
   const crn_uint32 DDSD_WIDTH                  = 0x00000004;
   const crn_uint32 DDSD_PITCH                  = 0x00000008;

   const crn_uint32 DDSD_BACKBUFFERCOUNT        = 0x00000020;
   const crn_uint32 DDSD_ZBUFFERBITDEPTH        = 0x00000040;
   const crn_uint32 DDSD_ALPHABITDEPTH          = 0x00000080;

   const crn_uint32 DDSD_LPSURFACE              = 0x00000800;

   const crn_uint32 DDSD_PIXELFORMAT            = 0x00001000;
   const crn_uint32 DDSD_CKDESTOVERLAY          = 0x00002000;
   const crn_uint32 DDSD_CKDESTBLT              = 0x00004000;
   const crn_uint32 DDSD_CKSRCOVERLAY           = 0x00008000;

   const crn_uint32 DDSD_CKSRCBLT               = 0x00010000;
   const crn_uint32 DDSD_MIPMAPCOUNT            = 0x00020000;
   const crn_uint32 DDSD_REFRESHRATE            = 0x00040000;
   const crn_uint32 DDSD_LINEARSIZE             = 0x00080000;

   const crn_uint32 DDSD_TEXTURESTAGE           = 0x00100000;
   const crn_uint32 DDSD_FVF                    = 0x00200000;
   const crn_uint32 DDSD_SRCVBHANDLE            = 0x00400000;
   const crn_uint32 DDSD_DEPTH                  = 0x00800000;

   const crn_uint32 DDSD_ALL                    = 0x00fff9ee;

   const crn_uint32 DDPF_ALPHAPIXELS            = 0x00000001;
   const crn_uint32 DDPF_ALPHA                  = 0x00000002;
   const crn_uint32 DDPF_FOURCC                 = 0x00000004;
   const crn_uint32 DDPF_PALETTEINDEXED8        = 0x00000020;
   const crn_uint32 DDPF_RGB                    = 0x00000040;
   const crn_uint32 DDPF_LUMINANCE              = 0x00020000;

   const crn_uint32 DDSCAPS_COMPLEX             = 0x00000008;
   const crn_uint32 DDSCAPS_TEXTURE             = 0x00001000;
   const crn_uint32 DDSCAPS_MIPMAP              = 0x00400000;

   const crn_uint32 DDSCAPS2_CUBEMAP            = 0x00000200;
   const crn_uint32 DDSCAPS2_CUBEMAP_POSITIVEX  = 0x00000400;
   const crn_uint32 DDSCAPS2_CUBEMAP_NEGATIVEX  = 0x00000800;

   const crn_uint32 DDSCAPS2_CUBEMAP_POSITIVEY  = 0x00001000;
   const crn_uint32 DDSCAPS2_CUBEMAP_NEGATIVEY  = 0x00002000;
   const crn_uint32 DDSCAPS2_CUBEMAP_POSITIVEZ  = 0x00004000;
   const crn_uint32 DDSCAPS2_CUBEMAP_NEGATIVEZ  = 0x00008000;

   const crn_uint32 DDSCAPS2_VOLUME             = 0x00200000;
#endif
} // namespace crnlib


static bool print_dds_info(const void *pData, crn_uint32 data_size, unsigned int *width, unsigned int *height, unsigned int *format, unsigned int *mipmaps)
{
   if ((data_size < 128) || (*reinterpret_cast<const crn_uint32*>(pData) != crnlib::cDDSFileSignature))
      return false;

   const crnlib::DDSURFACEDESC2 &desc = *reinterpret_cast<const crnlib::DDSURFACEDESC2*>((reinterpret_cast<const crn_uint8*>(pData) + sizeof(crn_uint32)));
   if (desc.dwSize != sizeof(crnlib::DDSURFACEDESC2))
      return false;

   DEBUG_INFO("DDS file information:\n");
   DEBUG_INFO("File size: %u, Dimensions: %ux%u, Pitch/LinearSize: %u\n", data_size, desc.dwWidth, desc.dwHeight, desc.dwLinearSize);
   DEBUG_INFO("MipMapCount: %u, AlphaBitDepth: %u\n", desc.dwMipMapCount, desc.dwAlphaBitDepth);

   *width = desc.dwWidth;
   *height = desc.dwHeight;
   *format = desc.ddpfPixelFormat.dwFourCC;
   *mipmaps = desc.dwMipMapCount;
   return true;

#if 0 // Rest of the code is a copy from crunch project. Will be adapted if needed, but only as a reminder here

   const char *pDDSDFlagNames[] =
   {
      "DDSD_CAPS", "DDSD_HEIGHT", "DDSD_WIDTH", "DDSD_PITCH",
      NULL, "DDSD_BACKBUFFERCOUNT", "DDSD_ZBUFFERBITDEPTH", "DDSD_ALPHABITDEPTH",
      NULL, NULL, NULL, "DDSD_LPSURFACE",
      "DDSD_PIXELFORMAT", "DDSD_CKDESTOVERLAY", "DDSD_CKDESTBLT", "DDSD_CKSRCOVERLAY",
      "DDSD_CKSRCBLT", "DDSD_MIPMAPCOUNT", "DDSD_REFRESHRATE", "DDSD_LINEARSIZE",
      "DDSD_TEXTURESTAGE", "DDSD_FVF", "DDSD_SRCVBHANDLE", "DDSD_DEPTH"
   };

   DEBUG_INFO("DDSD Flags: 0x%08X\n", desc.dwFlags);
   for (unsigned int i = 0; i < sizeof(pDDSDFlagNames)/sizeof(pDDSDFlagNames[0]); i++)
      if ((pDDSDFlagNames[i]) && (desc.dwFlags & (1 << i)))
         DEBUG_INFO(" %s\n", pDDSDFlagNames[i]);

   DEBUG_INFO("ddpfPixelFormat.dwFlags: 0x%08X\n", desc.ddpfPixelFormat.dwFlags);
   if (desc.ddpfPixelFormat.dwFlags & crnlib::DDPF_ALPHAPIXELS)     DEBUG_INFO(" DDPF_ALPHAPIXELS\n");
   if (desc.ddpfPixelFormat.dwFlags & crnlib::DDPF_ALPHA)           DEBUG_INFO(" DDPF_ALPHA\n");
   if (desc.ddpfPixelFormat.dwFlags & crnlib::DDPF_FOURCC)          DEBUG_INFO(" DDPF_FOURCC\n");
   if (desc.ddpfPixelFormat.dwFlags & crnlib::DDPF_PALETTEINDEXED8) DEBUG_INFO(" DDPF_PALETTEINDEXED8\n");
   if (desc.ddpfPixelFormat.dwFlags & crnlib::DDPF_RGB)             DEBUG_INFO(" DDPF_RGB\n");
   if (desc.ddpfPixelFormat.dwFlags & crnlib::DDPF_LUMINANCE)       DEBUG_INFO(" DDPF_LUMINANCE\n");

   DEBUG_INFO("ddpfPixelFormat.dwFourCC: 0x%08X '%c' '%c' '%c' '%c'\n",
      desc.ddpfPixelFormat.dwFourCC,
      std::max(32U, desc.ddpfPixelFormat.dwFourCC & 0xFF),
      std::max(32U, (desc.ddpfPixelFormat.dwFourCC >> 8) & 0xFF),
      std::max(32U, (desc.ddpfPixelFormat.dwFourCC >> 16) & 0xFF),
      std::max(32U, (desc.ddpfPixelFormat.dwFourCC >> 24) & 0xFF));

   DEBUG_INFO("dwRGBBitCount: %u 0x%08X\n",
      desc.ddpfPixelFormat.dwRGBBitCount, desc.ddpfPixelFormat.dwRGBBitCount);

   DEBUG_INFO("dwRGBBitCount as FOURCC: '%c' '%c' '%c' '%c'\n",
      std::max(32U, desc.ddpfPixelFormat.dwRGBBitCount & 0xFF),
      std::max(32U, (desc.ddpfPixelFormat.dwRGBBitCount >> 8) & 0xFF),
      std::max(32U, (desc.ddpfPixelFormat.dwRGBBitCount >> 16) & 0xFF),
      std::max(32U, (desc.ddpfPixelFormat.dwRGBBitCount >> 24) & 0xFF));

   DEBUG_INFO("dwRBitMask: 0x%08X, dwGBitMask: 0x%08X, dwBBitMask: 0x%08X, dwRGBAlphaBitMask: 0x%08X\n",
      desc.ddpfPixelFormat.dwRBitMask, desc.ddpfPixelFormat.dwGBitMask, desc.ddpfPixelFormat.dwBBitMask, desc.ddpfPixelFormat.dwRGBAlphaBitMask);

   DEBUG_INFO("ddsCaps.dwCaps: 0x%08X\n", desc.ddsCaps.dwCaps);
   if (desc.ddsCaps.dwCaps & crnlib::DDSCAPS_COMPLEX)   DEBUG_INFO(" DDSCAPS_COMPLEX\n");
   if (desc.ddsCaps.dwCaps & crnlib::DDSCAPS_TEXTURE)   DEBUG_INFO(" DDSCAPS_TEXTURE\n");
   if (desc.ddsCaps.dwCaps & crnlib::DDSCAPS_MIPMAP)    DEBUG_INFO(" DDSCAPS_MIPMAP\n");

   DEBUG_INFO("ddsCaps.dwCaps2: 0x%08X\n", desc.ddsCaps.dwCaps2);
   const char *pDDCAPS2FlagNames[] =
   {
      NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL,
      NULL, "DDSCAPS2_CUBEMAP", "DDSCAPS2_CUBEMAP_POSITIVEX", "DDSCAPS2_CUBEMAP_NEGATIVEX",
      "DDSCAPS2_CUBEMAP_POSITIVEY", "DDSCAPS2_CUBEMAP_NEGATIVEY", "DDSCAPS2_CUBEMAP_POSITIVEZ", "DDSCAPS2_CUBEMAP_NEGATIVEZ",
      NULL, NULL, NULL, NULL,
      NULL, "DDSCAPS2_VOLUME"
   };
   for (unsigned int i = 0; i < sizeof(pDDCAPS2FlagNames)/sizeof(pDDCAPS2FlagNames[0]); i++)
      if ((pDDCAPS2FlagNames[i]) && (desc.ddsCaps.dwCaps2 & (1 << i)))
         DEBUG_INFO(" %s\n", pDDCAPS2FlagNames[i]);

   DEBUG_INFO("ddsCaps.dwCaps3: 0x%08X, ddsCaps.dwCaps4: 0x%08X\n",
      desc.ddsCaps.dwCaps3, desc.ddsCaps.dwCaps4);

   return true;
#endif
}

GLuint RTexture::loadDXTTextureFromDDS(const char *filename)
{
    unsigned int length;
    unsigned char *buffer, *mipmap;
    unsigned int width=0, height=0, format=0;
    unsigned int texID;
    unsigned int glSize, glFormat;
    unsigned int total_mipmaps;
    unsigned int miplevel;

    buffer = (unsigned char *) ResourceManager::Instance()->readBinaryFile(filename, &length);
    if (buffer == NULL) return 0;

    DEBUG_NOTIFICATION("DDS File read into memory, length %d bytes\n", (int)length);
    print_dds_info(buffer, length, &width, &height, &format, &total_mipmaps);

    texID = 0;
    miplevel = 0;
    mipmap = (unsigned char *)&buffer[sizeof(crnlib::DDSURFACEDESC2)+4];

    switch(format)
    {
#if defined(GL_COMPRESSED_RGB_S3TC_DXT1_EXT)
    case crnlib::PIXEL_FMT_DXT1:
        glFormat = GL_COMPRESSED_RGB_S3TC_DXT1_EXT; // 0x83f0
        glSize = width*height*3/6;
        break;
#endif
#if defined(GL_COMPRESSED_RGBA_S3TC_DXT1_EXT)
    case crnlib::PIXEL_FMT_DXT1A:
        glFormat = GL_COMPRESSED_RGBA_S3TC_DXT1_EXT; // 0x83f1
        glSize = width*height*4/6;
        break;
#endif

#ifndef NC_EGL

#if defined(GL_COMPRESSED_RGBA_S3TC_DXT3_EXT)
    case crnlib::PIXEL_FMT_DXT3:
        glFormat = GL_COMPRESSED_RGBA_S3TC_DXT3_EXT; // 0x83f2
        glSize = width*height*4/4;
        break;
#endif
#if defined(GL_COMPRESSED_RGBA_S3TC_DXT5_EXT)
    case crnlib::PIXEL_FMT_DXT5:
        glFormat = GL_COMPRESSED_RGBA_S3TC_DXT5_EXT; // 0x83f3
        glSize = width*height*4/4;
        break;
#endif

#else // defined NC_EGL

    // GLES2 does not have the following defined legally, so we use hardcoded values
    case crnlib::PIXEL_FMT_DXT3:
        if (!Scenegraph::Instance()->queryFeature(SC_FEATURE_TEXTURING_DXT3))
        {
            DEBUG_WARNING("DXT3 texturing not supported by hardware, aborting\n");
            delete buffer;
            return 0;
        }
        glFormat = 0x83f2; //GL_COMPRESSED_RGBA_S3TC_DXT3_EXT;
        glSize = width*height*4/4;
        break;
    case crnlib::PIXEL_FMT_DXT5:
        if (!Scenegraph::Instance()->queryFeature(SC_FEATURE_TEXTURING_DXT5))
        {
            DEBUG_WARNING("DXT5 texturing not supported by hardware, aborting\n");
            delete buffer;
            return 0;
        }
        glFormat = 0x83f3; // GL_COMPRESSED_RGBA_S3TC_DXT5_EXT;
        glSize = width*height*4/4;
        break;
#endif

    default:
        DEBUG_WARNING("Unrecognized pixel format 0x0%x.\n", format);
        delete buffer;
        return 0;
    }
    DEBUG_NOTIFICATION("Processing mipmap level %d, size %dx%d, size %d, format 0x%x\n", miplevel, width, height, glSize, glFormat);
    if (-1 == createCompressedGLTexture(&texID, mipmap, glSize, miplevel, width, height, glFormat))
    {
        DEBUG_WARNING("Texture generation failed\n");
        return 0;
    }

#if 0 // Following code is able to read mipmaps from the DDS, if those are available. However, ATM mipmaps are autogenerated always.
    // Power of two?? if not then skip mipmaps, for now
    if (!isPowerOfTwo(width) || !isPowerOfTwo(height))
    {
        DEBUG_WARNING("Skipping mipmap reading, since width or height not power of two\n");
        break;
    }
    width /= 2;
    height /= 2;
    miplevel++;
    mipmap = (unsigned char *) &mipmap[glSize];
#endif

    delete buffer;
    return texID;
}
