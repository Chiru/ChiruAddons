#include "Neocortex_API.h"

#define UT60 1

#if (UT60 == 1) // support code for current consumption reader

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <sys/ioctl.h>
#include <pthread.h>

#define BAUDRATE B2400
#define MODEMDEVICE "/dev/ttyUSB0"
#define _POSIX_SOURCE 1 /* POSIX compliant source */

#ifndef FALSE
#define FALSE 0
#endif
#ifndef TRUE
#define TRUE 1
#endif

volatile int STOP=FALSE;
unsigned int Valid=1;
unsigned int Overflow;
float Digit;
unsigned char buf[255];

void parse(unsigned char *data)
{
    char digits[] = { data[1], data[2], '.', data[3], data[4], data[5], 0 };
    float value;
    int i;
    value = atof(digits);
    printf("CURRENT %f\n",value);
    fflush(NULL);
    return;
}

void *slave_UT60(void *x){
    int fd,c, res;
    unsigned int mcr;
    struct termios newtio;
    /*
    * Open modem device for reading and writing and not as controlling tty
    * because we don't want to get killed if linenoise sends CTRL-C.
    *
    */

    fd = open(MODEMDEVICE, O_RDWR | O_NOCTTY);
    if (fd < 0) {
        fprintf(stderr, "Unable to open port %s\n", MODEMDEVICE);
        pthread_exit(0);
        return NULL;
    }

    //tcgetattr(fd, &oldtio); /* save current serial port settings */
    bzero(&newtio, sizeof(newtio)); /* clear struct for new port settings */

    newtio.c_cflag = B19200 | CS7 | PARENB | PARODD | CREAD | CLOCAL;
    newtio.c_iflag = 0;
    newtio.c_oflag = 0;
    newtio.c_lflag = ICANON;

    tcflush(fd, TCIFLUSH);
    tcsetattr(fd, TCSANOW, &newtio);
    #if 1
    int line_bits;
    ioctl(fd, TIOCMGET, &line_bits);
    line_bits |= TIOCM_DTR;
    line_bits &= ~TIOCM_RTS;
    ioctl(fd, TIOCMSET, &line_bits);
    #endif

    while (STOP==FALSE) {
        res = read(fd,buf,255);
        Valid = 0;
        Digit = 0.0;
        Overflow = 0;

        if(res == 14){
            Valid = 1;
            //UT60Decode();
            parse(buf);
            //UT60Print();
        }
        else {
    //        if (res > 0)
          printf("packet size %d\n", res);
        }
    }
    pthread_exit(0);
    return NULL;
}

static pthread_t UT60_thread;

void init_UT60(void)
{
    pthread_create(&UT60_thread,NULL,slave_UT60,NULL);
}

void stop_UT60(void)
{
    STOP = TRUE;
    pthread_join(UT60_thread,NULL);
}

#define UT60_INIT() init_UT60();
#define UT60_STOP() stop_UT60();

#else // UT60 == TRUE
#define UT60_INIT()
#define UT60_STOP()
#endif


#define ASSET_HOME "/home/vatjjar/projects/mobileEnergyModel"

#include "GL/gl.h"

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#define NC_GID_RESOURCE_MESH        1
#define NC_GID_RESOURCE_TEXTURE     2
#define NC_GID_RESOURCE_MATERIAL    4

static int rMesh = 0;
static int rMaterial = 0;
static int rTexture = 0;

int loadAsset(int type, const char *name)
{
    fprintf(stderr, "Loading resource, type=%d, name=%s\n", type, name);
    return RES_createResource(type, name);
}

void loadAssets(const char *mesh, const char *material, const char *texture)
{
    rMesh     = loadAsset(NC_GID_RESOURCE_MESH, mesh);
    rMaterial = loadAsset(NC_GID_RESOURCE_MATERIAL, material);
    rTexture  = loadAsset(NC_GID_RESOURCE_TEXTURE, texture);
}

void deleteAssets(void)
{
    RES_deleteResource(rMesh);
    RES_deleteResource(rMaterial);
    RES_deleteResource(rTexture);
}

static int obj1, con, camera;

void testcase_vertexTest(int LOD)
{
    const char *mesh_table[] = {
        ASSET_HOME "/Plane_4.mesh.xml",
        ASSET_HOME "/Plane_40000.mesh.xml",
        ASSET_HOME "/Plane_65025.mesh.xml",
        ASSET_HOME "/Plane_65025.mesh.xml",
        ASSET_HOME "/Plane_65025.mesh.xml"
    };
    const char *material_table[] = {
        ASSET_HOME "/Plane_lod1.material",
    };
    const char *texture_table[] = {
        ASSET_HOME "/tex_1x1.png"
    };
    const char *extra_mesh[] = {
        ASSET_HOME "/Plane_65025_2.mesh.xml",
        ASSET_HOME "/Plane_65025_3.mesh.xml",
        ASSET_HOME "/Plane_65025_4.mesh.xml",
        ASSET_HOME "/Plane_65025_5.mesh.xml"
    };

    loadAssets(mesh_table[LOD], material_table[0], texture_table[0]);

    //obj1 = SMG_createObject();
    //OBJ_setPosition(obj1, 0, 0, -5);
    //OBJ_setRotationDegrees(obj1, 90, 0, 0);
    //OBJ_attachMesh(obj1, rMesh);
    //OBJ_attachMaterial(obj1, "default", 0);

    //if (LOD == 3 || LOD == 4)
    {
        int i, o;
        int num;
        if (LOD == 4) num = 12;
        else if (LOD == 3) num = 6;
        else if (LOD == 2) num = 3;
        else if (LOD == 1) num = 1;
        else num = 1;
        float step = 0.001f;
        float pos = -1-step*(num-1);

        for (i=0; i<num; i++)
        {
            //loadAsset(NC_GID_RESOURCE_MESH, extra_mesh[i%4]);
            o = SMG_createObject();
            OBJ_setPosition(o, 0, 0, pos);
            fprintf(stderr, "Placing object to %f %f %f\n", 0.0f, 0.0f, pos);
            OBJ_setRotationDegrees(o, 90, 0, 0);
            OBJ_attachMesh(o, rMesh);
            OBJ_attachMaterial(o, "default", 0);
            pos += step;
        }
    }
}

void testcase_batchTest(int LOD)
{
    unsigned int i;
    const int batches[] = { 1, 2, 8, 16, 32 };
    const char *mesh_table[] = {
        ASSET_HOME "/Plane_4.mesh.xml",
    };
    const char *material_table[] = {
        ASSET_HOME "/Plane_lod1.material",
    };
    const char *texture_table[] = {
        ASSET_HOME "/tex_1x1.png"
    };

    loadAssets(mesh_table[0], material_table[0], texture_table[0]);

    float step = 0.001f;
    float pos = -1-step*(batches[LOD]-1);
    for (i=0; i<batches[LOD]; i++)
    {
        obj1 = SMG_createObject();
        OBJ_setPosition(obj1, 0, 0, pos);
        fprintf(stderr, "Placing object to %f %f %f\n", 0.0f, 0.0f, pos);
        OBJ_setRotationDegrees(obj1, 90, 0, 0);
        OBJ_attachMesh(obj1, rMesh);
        OBJ_attachMaterial(obj1, "default", 0);
        pos += step;
    }
}

void testcase_textureTest(int LOD)
{
    const char *mesh_table[] = {
        ASSET_HOME "/Plane_4.mesh.xml",
    };
    const char *material_table[] = {
        ASSET_HOME "/Plane_lod1.material",
        ASSET_HOME "/Plane_lod2.material",
        ASSET_HOME "/Plane_lod3.material",
        ASSET_HOME "/Plane_lod4.material",
        ASSET_HOME "/Plane_lod5.material"
    };
    const char *texture_table[] = {
        ASSET_HOME "/tex_1x1.png",
        ASSET_HOME "/tex_793x793.png",
        ASSET_HOME "/tex_1773x1773.png",
        ASSET_HOME "/tex_1773x1773.png",
        ASSET_HOME "/tex_2048x2048.png"
    };
    const char *extra_tex[] = {
        ASSET_HOME "/tex_1773x1773_2.png", // LOD3 extra
        ASSET_HOME "/tex_2048x2048_2.png", // LOD4 extra
        ASSET_HOME "/tex_2048x2048_3.png",
        //ASSET_HOME "/tex_2048x2048_2.png",
        //ASSET_HOME "/tex_2048x2048_3.png",
        //ASSET_HOME "/tex_2048x2048_4.png",
        //ASSET_HOME "/tex_2048x2048_2.png",
        //ASSET_HOME "/tex_2048x2048_3.png"
    };
    const char *mat_names[] = {
        "default", "extra1", "extra2", "extra3" //, "default", "extra1", "extra2", "extra3", "default"
    };

    loadAssets(mesh_table[0], material_table[LOD], texture_table[LOD]);

    //obj1 = SMG_createObject();
    //OBJ_setPosition(obj1, 0, 0, -4);
    //OBJ_setRotationDegrees(obj1, 90, 0, 0);
    //OBJ_attachMesh(obj1, rMesh);
    //OBJ_attachMaterial(obj1, "default", 0);

    //if (LOD == 3 || LOD == 4)
    {
        int i, o;
        int num;
        int lIndex = 0;
        if (LOD == 4) { num = 3; lIndex = 1; }
        else if (LOD == 3) num = 2;
        else num = 1;
        float step = 0.001f;
        float pos = -1-step*(num-1);

        loadAsset(NC_GID_RESOURCE_MATERIAL, ASSET_HOME "/extra1.material");

        for (i=0; i<num; i++)
        {
            fprintf(stderr, "---\n");
            if (i >= 1)
                loadAsset(NC_GID_RESOURCE_TEXTURE, extra_tex[lIndex+i-1]);
            o = SMG_createObject();
            OBJ_setPosition(o, 0, 0, pos);
            fprintf(stderr, "Placing object to %f %f %f, mesh %s, material %s\n", 0.0f, 0.0f, pos, "blah", mat_names[i]);
            OBJ_setRotationDegrees(o, 90, 0, 0);
            OBJ_attachMesh(o, rMesh);
            OBJ_attachMaterial(o, mat_names[i], 0);
            pos += step;
        }
        fprintf(stderr, "---\n");
    }
}

int main(int argc, char *args[])
{
    float t1;
    float t2;
    float start, end;
    float total;
    float fpslimit;
    int frames;
    int testcase;
    int lod, xres, yres, verb;
    float wait;

    // Usage: test.exe [testcase 0-4] [LOD 0-4]
    fprintf(stderr, "usage: %s testcase lod xres yres verbose fpslimit\n", args[0]);

    testcase = 0;
    lod = 0;
    xres = 1280;
    yres = 720;
    verb = 0;
    fpslimit = 0.0f;
    if (argc >= 2) testcase = atoi(args[1]);
    if (argc >= 3) lod = atoi(args[2]);
    if (argc >= 4) xres = atoi(args[3]);
    if (argc >= 5) yres = atoi(args[4]);
    if (argc >= 6) verb = atoi(args[5]);
    if (argc >= 7) fpslimit = atof(args[6]);

    SMG_init();
    SMG_setVerbosity(verb);
    SMG_createDisplay(xres, yres, FALSE);

    t1 = SMG_getSystemtime();
    switch(testcase)
    {
    case 0:
        fprintf(stderr, "testcase VERTEX (%d), lod %d, x=%d, y=%d, verbose=%d, fpslimit=%2.1f\n", testcase, lod, xres, yres, verb, fpslimit);
        testcase_vertexTest(lod);
        break;
    case 1:
        fprintf(stderr, "testcase BATCH (%d), lod %d, x=%d, y=%d, verbose=%d, fpslimit=%2.1f\n", testcase, lod, xres, yres, verb, fpslimit);
        testcase_batchTest(lod);
        break;
    case 2:
        fprintf(stderr, "testcase TEXTURE (%d), lod %d, x=%d, y=%d, verbose=%d, fpslimit=%2.1f\n", testcase, lod, xres, yres, verb, fpslimit);
        testcase_textureTest(lod);
        break;
    default:
        fprintf(stderr, "invalid testcase ID %d. Rendering empty display\n", testcase);
        break;
        //SMG_destroyDisplay();
        //SMG_destroy();
        //return -1;
    }

    // And the default camera:
    camera = SMG_createCamera();
    SMG_setCameraPosition(camera, 0, 0, 0);
    SMG_setCameraMode(camera, 1);
    t2 = SMG_getSystemtime();
    fprintf(stderr, "Assets and scene loaded in %f seconds\n", t2-t1);

    total = 0.0f;
    frames = 0;
    start = SMG_getSystemtime();
    fprintf(stderr, "Now rendering 1000 frames\n");
    UT60_INIT(); printf("---;%d;%d;%d;%d\n", testcase, lod, xres, yres);
    while(1)
    {
        t1 = SMG_getSystemtime();
        if (-1 == SMG_renderFrame(1.0f/30.f))
        {
            fprintf(stderr, "User abort!\n");
            break;
        }
        //t2 = SMG_getSystemtime();
        SMG_swapFrame();
        t2 = SMG_getSystemtime();
        if (fpslimit == 0.0f) wait = 0.0f;
        else wait = (1.0f/fpslimit)-(t2-t1);
        //fprintf(stderr, "Frametime %f, wait %f\n", t2-t1, wait);
        if (wait > 0.0f)
        {
            //fprintf(stderr, "waitng %f microseconds\n", wait*100000.f);
            usleep((unsigned int)(wait*1000000.0f));
            //fprintf(stderr, "Done\n");
        }
        frames++;
        t2 = SMG_getSystemtime();
        total += (t2-t1);
        //if (frames == 1000) { fprintf(stderr, "Framelimit!\n"); break; } // no more than 1000 frames
        if (total >= 30.0f) { fprintf(stderr, "Timelimit!\n"); break; } // no more than 20sek
    }
    end = SMG_getSystemtime();
    UT60_STOP(); printf("+++;%d;%d;%d;%d\n", testcase, lod, xres, yres);
    fprintf(stderr, "Done rendering %d frames in %f seconds, average frame GL command issue time %f, hence %f FPS (frametime %f)\n",
        frames, end-start, total/frames, frames/(end-start), (end-start)/frames);

    deleteAssets();
    SMG_destroyDisplay();
    SMG_destroy();}
