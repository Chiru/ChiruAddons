#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <sys/ioctl.h>
#include <iostream>

#define BAUDRATE B2400            
#define MODEMDEVICE "/dev/ttyUSB0"
#define _POSIX_SOURCE 1 /* POSIX compliant source */

#define FALSE 0
#define TRUE 1

volatile int STOP=FALSE; 

/*
 * Stuff we know about:
 */
unsigned int Valid=1;
unsigned int Hold,Rel, Rms, Overflow, Valid;
float Digit;
char UnitS[10];
enum kinda_unit {V, A, Hz, C, MOhm};
enum kinda_unit Unit;

unsigned char buf[255];

void UT60Print(void){
  if(Valid){
    if(Overflow){
      printf("-null-");
    } else {
      printf("%g",Digit);
    };
    printf(" %s",UnitS);
    if(Rms)printf(" Rms");
    if(Hold)printf(" Hold");
    if(Rel)printf(" Rel");
    printf("\n");
  };
};

void UT60Decode(void){
  int j;
  unsigned int dot;
  float fdot;
    dot = 0;
    fdot = 1;
    for(j=1;j<8;j+=2){
     unsigned int d = ((0x07&buf[j])<<4) + (0x0F&buf[1+j]);
     dot |= ((0x08 & buf[j])>>3);
     if(dot)fdot *= 10;
     switch(d){
       case 0x7D : d = 0; break;
       case 0x05 : d = 1; break;
       case 0x5B : d = 2; break;
       case 0x1F : d = 3; break;
       case 0x27 : d = 4; break;
       case 0x3E : d = 5; break;
       case 0x7E : d = 6; break;
       case 0x15 : d = 7; break;
       case 0x7F : d = 8; break;
       case 0x3F : d = 9; break;
       case 0x68 : d = 0; Overflow = 1; break;
       default : d = 0;
     };
     if(dot){
       Digit = (float)d/(fdot) + (Digit);
     } else {
       Digit = (float)d + (Digit*10.0);
     };
    };
    if(buf[10]&0x08)Digit /= 1000.0;    // milli amps
    if(buf[9]&0x08)Digit /= 1000000.0; // micro amps
    if(buf[1]&0x08)Digit = -Digit;
    Rms = 0;
    switch (0x0F&buf[0]){
      case 1  : strcpy(UnitS,"Hz ");Unit = Hz; break;
      case 3  : strcpy(UnitS,"MOhm") ; Unit = MOhm; break;
      case 5  : strcpy(UnitS,"C  "); Unit = C; break;
      case 11 : {
         Rms = 1;
         if((buf[12]&0x08) != 0){
           strcpy(UnitS,"A  "); Unit = A; break;
         }else{
           strcpy(UnitS,"V  "); Unit = V; break;
         };
      };
      default: {
         if((buf[12]&0x08) != 0){
           strcpy(UnitS,"A  "); Unit = A; break;
         }else{
           strcpy(UnitS,"V  "); Unit = V; break;
         };
      };
    };
    Hold = (buf[11]&0x01) != 0;
    Rel = (buf[11]&0x02) != 0;

};

void parse(unsigned char *data)
{
    char digits[] = { data[1], data[2], '.', data[3], data[4], data[5], 0 };
    float value;
    int i;
    value = atof(digits);
    printf("%f\n",value);
    fflush(NULL);
    return;
#if 1
for (i=0; i<14; i++)
    printf("0x%02x ", data[i]);
printf(" : ");
for (i=0; i<14; i++)
    if (data[i] > 0x20)
        printf("%c", data[i]);
    else printf(".");
printf("\n");
    return;
#else
    lastmode = mode;

    bat = (data[7] & 2) ? true : false;
    rel = (data[8] & 2) ? true : false;
    hold = (data[11] & 2) ? true : false;

    if (data[7] & 0x04)
        value *= -1;

    if (data[10] & 8)
        power = DC;
    else if (data[10] & 4)
        power = AC;

    if (data[10] & 2)
        range = AUTO;
    else
        range = MANUAL;

    if (data[7] & 1)
        load = OVERLOAD;
    else if (data[9] & 8)
        load = UNDERLOAD;
    else
        load = NORMAL;

    if (data[9] & 4)
        peak = MAX;
    else if (data[9] & 2)
        peak = MIN;

    if (data[10] & 1)
        fmode = FREQUENCE;

    if (data[7] & 8)
        fmode = DUTY;

    double multp = 1;
    switch (data[6]) {
    case '1':
        mode = DIODE;
        break;

    case '2':
        mode = FREQUENCY;

        switch (data[0]) {
        case '0':
            multp = 1e-2;
            break;
        case '1':
            multp = 1e-1;
            break;
        case '3':
            multp = 1;
            break;
        case '4':
            multp = 1e1;
            break;
        case '5':
            multp = 1e2;
            break;
        case '6':
            multp = 1e3;
            break;
        case '7':
            multp = 1e4;
            break;
        default:
            throw std::exception();
        }
        break;

    case '3':
        mode = RESISTANCE;

        switch (data[0]) {
        case '0':
            multp = 1e-2;
            break;
        case '1':
            multp = 1e-1;
            break;
        case '2':
            multp = 1;
            break;
        case '3':
            multp = 1e1;
            break;
        case '4':
            multp = 1e2;
            break;
        case '5':
            multp = 1e3;
            break;
        case '6':
            multp = 1e4;
            break;
        default:
            throw std::exception();
        }
        break;

    case '5':
        mode = CONDUCTANCE;
        break;

    case '6':
        mode = CAPACITANCE;

        switch (data[0]) {
        case '0':
            multp = 1e-12;
            break;
        case '1':
            multp = 1e-11;
            break;
        case '2':
            multp = 1e-10;
            break;
        case '3':
            multp = 1e-9;
            break;
        case '4':
            multp = 1e-8;
            break;
        case '5':
            multp = 1e-7;
            break;
        case '6':
            multp = 1e-6;
            break;
        case '7':
            multp = 1e-5;
            break;
        default:
            throw std::exception();
        }
        break;

    case 0x3b: // V
        mode = VOLTAGE;

        switch (data[0]) {
        case '0':
            multp = 1e-4;
            break;
        case '1':
            multp = 1e-3;
            break;
        case '2':
            multp = 1e-2;
            break;
        case '3':
            multp = 1e-1;
            break;
        case '4':
            multp = 1e-5;
            break;
        default:
            throw std::exception();
        }
        break;

    case '0': // A
        mode = CURENT;
        if (data[0] == '0')
            multp = 1e-3;
        else {
            throw std::exception();
        }
        break;

    case 0x3d: // uA
        mode = CURENT;

        switch (data[0]) {
        case '0':
            multp = 1e-8;
            break;
        case '1':
            multp = 1e-7;
            break;
        default:
            throw std::exception();
        }

        break;
    case 0x3f: // mA
        mode = CURENT;

        switch (data[0]) {
        case '0':
            multp = 1e-6;
            break;
        case '1':
            multp = 1e-5;
            break;
        default:
            throw std::exception();
        }

        break;
    default:
        throw std::exception();
    }

    value *= multp;

    if (mode != lastmode) {
        max = 0;
        min = 0;
        sample = 0;
        avarage = value;
    }

    if (value > max)
        max = value;

    if (value < min)
        min = value;

    avarage = (sample * avarage + value) / ++sample;
#endif
}


main()
{
  int fd,c, res;
  unsigned int mcr;
  struct termios newtio;
/* 
 * Open modem device for reading and writing and not as controlling tty
 * because we don't want to get killed if linenoise sends CTRL-C.
 *
 */

#if 0
 fd = open(MODEMDEVICE, O_RDONLY | O_NOCTTY );
 if (fd <0) {perror(MODEMDEVICE); return(-1); }

 bzero(&newtio, sizeof(newtio));

 newtio.c_cflag = BAUDRATE | CS8 | CLOCAL | CREAD ;
 newtio.c_iflag = ICRNL;
 newtio.c_oflag = 0;
 newtio.c_lflag = ~ICANON;
 newtio.c_cc[VINTR]    = 0;     /* Ctrl-c */ 
 newtio.c_cc[VQUIT]    = 0;     /* Ctrl-\ */
 newtio.c_cc[VERASE]   = 0;     /* del */
 newtio.c_cc[VKILL]    = 0;     /* @ */
 newtio.c_cc[VEOF]     = 0;     /* Ctrl-d */
 newtio.c_cc[VTIME]    = 1;     /* inter-character timer unused */
 newtio.c_cc[VMIN]     = 14;     /* blocking read until 1 character arrives */
 newtio.c_cc[VSWTC]    = 0;     /* '\0' */
 newtio.c_cc[VSTART]   =17;     /* Ctrl-q */ 
 newtio.c_cc[VSTOP]    =19;     /* Ctrl-s */
 newtio.c_cc[VSUSP]    = 0;     /* Ctrl-z */
 newtio.c_cc[VEOL]     = 0;     /* '\0' */
 newtio.c_cc[VREPRINT] = 0;     /* Ctrl-r */
 newtio.c_cc[VDISCARD] = 0;     /* Ctrl-u */
 newtio.c_cc[VWERASE]  = 0;     /* Ctrl-w */
 newtio.c_cc[VLNEXT]   = 0;     /* Ctrl-v */
 newtio.c_cc[VEOL2]    = 0;     /* '\0' */

 tcsetattr(fd,TCSANOW,&newtio);
 mcr = 0;
 ioctl(fd,TIOCMGET,&mcr);
 mcr &= ~TIOCM_RTS;
 mcr |= TIOCM_DTR;
 ioctl(fd,TIOCMSET,&mcr);
 tcflush(fd, TCIFLUSH);

#else

 fd = open(MODEMDEVICE, O_RDWR | O_NOCTTY);
 if (fd < 0) {
     perror("Unable to open port ");
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
}
