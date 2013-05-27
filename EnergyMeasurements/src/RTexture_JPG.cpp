
/* Neocortex
 *
 * Author: Jarkko Vatjus-Anttila <jvatjusanttila@gmail.com>
 *
 * For conditions of distribution and use, see copyright notice in license.txt
 */

#include "RTexture.h"
#include "GLWrapper.h"
#include "DebugLog.h"

#include <iostream>
#include <fstream>
#include <sys/stat.h>

#include <stdlib.h>
#include <string.h>

#include "jpeglib.h"

GLuint RTexture::loadRGBTextureFromJPG(const char *filename)
{
    /* these are standard libjpeg structures for reading(decompression) */
    struct jpeg_decompress_struct cinfo;
    struct jpeg_error_mgr jerr;
    /* libjpeg data structure for storing one row, that is, scanline of an image */
    JSAMPROW row_pointer[1];

    unsigned int location = 0;
    unsigned int i;
    unsigned char *raw_image;
    FILE *infile;
    GLuint texID;

    infile = fopen( filename, "rb" );
    if ( !infile )
    {
        DEBUG_WARNING("Unable to open JPG file %s\n", filename);
        return 0;
    }

    /* here we set up the standard libjpeg error handler */
    cinfo.err = jpeg_std_error( &jerr );
    /* setup decompression process and source, then read JPEG header */
    jpeg_create_decompress( &cinfo );
    /* this makes the library read from infile */
    jpeg_stdio_src( &cinfo, infile );
    /* reading the image header which contains image information */
    jpeg_read_header( &cinfo, TRUE );
    /* Uncomment the following to output image information, if needed. */

    DEBUG_NOTIFICATION("JPEG info: size %dx%d, color components/pixel %d, Colorspace %d\n",
                       cinfo.image_width, cinfo.image_height, cinfo.num_components, cinfo.jpeg_color_space);

    /* Start decompression jpeg here */
    jpeg_start_decompress( &cinfo );

    /* allocate memory to hold the uncompressed image */
    raw_image = new unsigned char [cinfo.output_width*cinfo.output_height*cinfo.num_components];

    /* now actually read the jpeg into the raw buffer */
    row_pointer[0] = (unsigned char *)malloc( cinfo.output_width*cinfo.num_components );
    /* read one scan line at a time */
    while( cinfo.output_scanline < cinfo.image_height )
    {
        jpeg_read_scanlines( &cinfo, row_pointer, 1 );
        for( i=0; i<cinfo.image_width*cinfo.num_components; i++)
            raw_image[location++] = row_pointer[0][i];
    }
    /* wrap up decompression, destroy objects, free pointers and close open files */
    jpeg_finish_decompress( &cinfo );
    jpeg_destroy_decompress( &cinfo );
    free( row_pointer[0] );
    fclose( infile );

    DEBUG_NOTIFICATION("JPG file loaded successfully\n");

    /*
     * GL texture generation part
     */
    texID = createGLTexture(raw_image, cinfo.image_width, cinfo.image_height, false);
    if (texID == 0) delete raw_image;
    return texID;
}
