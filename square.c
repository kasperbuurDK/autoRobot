/*
 * An example SMR program.
 *
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <unistd.h>
#include <string.h>
#include <stdlib.h>
#include <signal.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include "rhd.h"
#include "componentserver.h"
#include "xmlio.h"

struct xml_in *xmldata;
struct xml_in *xmllaser;
struct {
    double x, y, z, omega, phi, kappa, code, id, crc;
} gmk;
typedef struct {
    double speedL, speedR;
    double x, y, theta;
    int time;
} dataPacket;
double visionpar[10];
double laserpar[10];
#define LOG_SIZE 10000 //-------!!!!!----------
#define SAMPLE_RATE 0.01
#define LIMIT 0.5
#define WHITE 128
#define BLACK 0

void serverconnect(componentservertype *s);
void xml_proc(struct xml_in *x);
void xml_proca(struct xml_in *x);
componentservertype lmssrv, camsrv;

symTableElement *
getinputref(const char *sym_name, symTableElement *tab) {
    int i;
    for (i = 0; i < getSymbolTableSize('r'); i++)
        if (strcmp(tab[i].name, sym_name) == 0)
            return &tab[i];
    return 0;
}

symTableElement *
getoutputref(const char *sym_name, symTableElement *tab) {
    int i;
    for (i = 0; i < getSymbolTableSize('w'); i++)
        if (strcmp(tab[i].name, sym_name) == 0)
            return &tab[i];
    return 0;
}
/*****************************************
* odometry
*/
#define WHEEL_DIAMETER   0.06522    /* m */
#define WHEEL_SEPARATION 0.26    /* m */
#define DELTA_M (M_PI * WHEEL_DIAMETER / 2000)
#define ROBOTPORT    8000 //24902
#define K1 (M_PI*(WHEEL_SEPARATION/2))/180
#define K 0.01

typedef struct {
    //input signals
    int left_enc, right_enc; // encoderticks
    // parameters
    double w;    // wheel separation
    double cr, cl;   // meters per encodertick
    //output signals
    double right_pos, left_pos;
    double x, y, theta;
    double delta_v, theta_ref;
    // internal variables
    int left_enc_old, right_enc_old;
} odotype;
void reset_odo(odotype *p);
void update_odo(odotype *p);

/********************************************
* Motion control
*/
#define K_LINE 0.1

typedef struct {//input
    int cmd;
    int curcmd;
    double speedcmd;
    double dist;
    double angle;
    double left_pos, right_pos;
    // parameters
    double w;
    //output
    double motorspeed_l, motorspeed_r;
    int finished;
    int ls_ref, ls;
    double delta_v;
    // internal variables
    double startpos;
} motiontype;

typedef struct {
    int state, oldstate;
    int time;
} smtype;

enum {
    mot_stop = 1, mot_move, mot_follow,mot_turn
};
enum {
    ms_init, ms_fwd, ms_follow, ms_turn, ms_end, ms_measureWithIR
};
enum {
    left = 6, middel = 4, right = 2
};


void update_motcon(motiontype *p);
void angular_controller(odotype *p);
int linesensorMinimumIntensity(symTableElement *linesensor);
int fwd(double dist, double speed, int time);
int turn(double angle, double speed, int time);
void sm_update(smtype *p);
int follow(double dist, double speed, int dir,int time);
void line_controller(motiontype *p);
double centerOfMass();

// SMR input/output data
symTableElement *inputtable, *outputtable;
symTableElement *lenc, *renc, *linesensor, *irsensor, *speedl, *speedr, *resetmotorr, *resetmotorl;
odotype odo;
smtype mission;
motiontype mot;




int main()
{
    int missonCount = 0;
    int running, n = 0, arg, time = 0;
    double dist = 0, angle = 0, speed = 0;
    dataPacket dataLog[LOG_SIZE];


    /* Establish connection to robot sensors and actuators.*/
    if (rhdConnect('w', "localhost", ROBOTPORT) != 'w') {
        printf("Can't connect to rhd \n");
        exit(EXIT_FAILURE);
    }
    printf("connected to robot \n");
    if ((inputtable = getSymbolTable('r')) == NULL) {
        printf("Can't connect to rhd \n");
        exit(EXIT_FAILURE);
    }
    if ((outputtable = getSymbolTable('w')) == NULL) {
        printf("Can't connect to rhd \n");
        exit(EXIT_FAILURE);
    }
    // connect to robot I/O variables
    lenc = getinputref("encl", inputtable);
    renc = getinputref("encr", inputtable);
    linesensor = getinputref("linesensor", inputtable);
    irsensor = getinputref("irsensor", inputtable);

    speedl = getoutputref("speedl", outputtable);
    speedr = getoutputref("speedr", outputtable);
    resetmotorr = getoutputref("resetmotorr", outputtable);
    resetmotorl = getoutputref("resetmotorl", outputtable);
    // **************************************************
    //  Camera server code initialization
    //
    /* Create endpoint */
    lmssrv.port = 24919;
    strcpy(lmssrv.host, "127.0.0.1");
    strcpy(lmssrv.name, "laserserver");
    lmssrv.status = 1;
    camsrv.port = 24920;
    strcpy(camsrv.host, "127.0.0.1");
    camsrv.config = 1;
    strcpy(camsrv.name, "cameraserver");
    camsrv.status = 1;

    if (camsrv.config) {
        int errno = 0;
        camsrv.sockfd = socket(AF_INET, SOCK_STREAM, 0);
        if (camsrv.sockfd < 0) {
            perror(strerror(errno));
            fprintf(stderr, " Can not make  socket\n");
            exit(errno);
        }
        serverconnect(&camsrv);
        xmldata = xml_in_init(4096, 32);
        printf(" camera server xml initialized \n");
    }
    // **************************************************
    //  LMS server code initialization
    //
    /* Create endpoint */
    lmssrv.config = 1;
    if (lmssrv.config) {
        char buf[256];
        int errno = 0, len;
        lmssrv.sockfd = socket(AF_INET, SOCK_STREAM, 0);
        if (lmssrv.sockfd < 0) {
            perror(strerror(errno));
            fprintf(stderr, " Can not make  socket\n");
            exit(errno);
        }
        serverconnect(&lmssrv);
        if (lmssrv.connected) {
            xmllaser = xml_in_init(4096, 32);
            printf(" laserserver xml initialized \n");
            len = sprintf(buf, "scanpush cmd='zoneobst'\n");
            send(lmssrv.sockfd, buf, len, 0);
        }
    }
    /* Read sensors and zero our position.*/
    rhdSync();
    odo.w = 0.256;
    odo.cr = DELTA_M;
    odo.cl = odo.cr;
    odo.left_enc = lenc->data[0];
    odo.right_enc = renc->data[0];

    reset_odo(&odo);
    printf("position: %f, %f\n", odo.left_pos, odo.right_pos);
    mot.w = odo.w;
    running = 1;
    mission.state = ms_init;
    mission.oldstate = -1;
    while (running) {
        if (lmssrv.config && lmssrv.status && lmssrv.connected) {
            while ((xml_in_fd(xmllaser, lmssrv.sockfd) > 0))
                xml_proca(xmllaser);
        }
        if (camsrv.config && camsrv.status && camsrv.connected) {
            while ((xml_in_fd(xmldata, camsrv.sockfd) > 0))
                xml_proc(xmldata);
        }

        rhdSync();
        odo.left_enc = lenc->data[0];
        odo.right_enc = renc->data[0];
        update_odo(&odo);

        /****************************************
        / mission statemachine
        ****************************************/
        sm_update(&mission);
        switch (mission.state) {
            case ms_init:
                dist = 0.1;
                speed = 0.4;
                n = 4;
                angle = (90.0) / 180 * M_PI;
                mission.state = ms_measureWithIR;

                break;

            case ms_follow:
                if (follow(dist, speed, middel, mission.time)) mission.state = ms_end;
                break;

            case ms_measureWithIR:
                for (int i = 1; i < irsensor->length-2; i++) {
                    printf("%d = %d , ", i, irsensor->data[i]);
                }

                printf("\n");
                dist = 0.5;
                mission.state = ms_fwd;
                n--;
                if (n == 0) mission.state = ms_end;

                /*
                 IRSENSORS
                 0 left side sensor
                 1 front left sensor
                 2 front middle sensor
                 3 front right sensor
                 4 right side sensor
                 */
                break;
            case ms_fwd:
                if (fwd(dist, speed, mission.time)) mission.state = ms_measureWithIR;

                break;

            case ms_turn:
                if (turn(angle, 0.3, mission.time)) {
                    n = n - 1;
                    if (n == 0) {
                        mission.state = ms_end;
                    } else
                        mission.state = ms_fwd;
                }
                break;

            case ms_end:
                mot.cmd = mot_stop;
                running = 0;
                break;
        }
        /*  end of mission  */

        /*dataLog[missonCount].speedL = mot.motorspeed_l;
        dataLog[missonCount].speedR = mot.motorspeed_r;
         */
        dataLog[missonCount].x = odo.x;
        dataLog[missonCount].y = odo.y;
        dataLog[missonCount].theta = odo.theta;
        dataLog[missonCount].time = mission.time;
        missonCount++;

        mot.left_pos = odo.left_pos;
        mot.right_pos = odo.right_pos;
        update_motcon(&mot);
        speedl->data[0] = 100 * mot.motorspeed_l;
        speedl->updated = 1;
        speedr->data[0] = 100 * mot.motorspeed_r;
        speedr->updated = 1;
        if (time % 100 == 0) time++;
        /* stop if keyboard is activated*/
        ioctl(0, FIONREAD, &arg);
        if (arg != 0) running = 0;
    }/* end of main control loop */
    // Log data to file
    FILE *fp = fopen("dataLogg.dat", "wr");
    for (int i = 0; i < missonCount; i++) {
        dataPacket dataSample = dataLog[i];
        //fprintf(fp, "%d,%d,%0.8f,%0.8f\n", i, dataSample.time, dataSample.speedL, dataSample.speedR);
        fprintf(fp, "%d,%d,%0.8f,%0.8f,%0.8f\n", i, dataSample.time, dataSample.x, dataSample.y, dataSample.theta);
    }
    /*fprintf(fp, "Laser: ");
    for (int i = 0 ; i < 10; i++){
        fprintf(fp, "%d = %0.3f ", i, laserpar[i]);
    }*/
    fprintf(fp, "\n");
    fclose(fp);
    speedl->data[0] = 0;
    speedl->updated = 1;
    speedr->data[0] = 0;
    speedr->updated = 1;
    rhdSync();
    rhdDisconnect();
    exit(0);
}

/*
 * Routines to convert encoder values to positions.
 * Encoder steps have to be converted to meters, and
 * roll-over has to be detected and corrected.
 */
void reset_odo(odotype *p) {
    p->right_pos = p->left_pos = 0.0;
    p->right_enc_old = p->right_enc;
    p->left_enc_old = p->left_enc;
    p->theta = 0.0;
    p->theta_ref = 0.0;
    p->y = 0.0;
    p->x = 0.0;
}

void update_odo(odotype *p) {
    int delta;
    double theta_i, u, ur, ul;

    delta = p->right_enc - p->right_enc_old;
    if (delta > 0x8000) delta -= 0x10000;
    else if (delta < -0x8000) delta += 0x10000;
    p->right_enc_old = p->right_enc;
    p->right_pos += delta * p->cr;
    ur = delta * p->cr;

    delta = p->left_enc - p->left_enc_old;
    if (delta > 0x8000) delta -= 0x10000;
    else if (delta < -0x8000) delta += 0x10000;
    p->left_enc_old = p->left_enc;
    p->left_pos += delta * p->cl;
    ul = delta * p->cl;

    // odemetry
    u = (ur+ul)/2;
    theta_i = (ur-ul)/WHEEL_SEPARATION; //---!!!---
    p->y += u*sin(p->theta);
    p->x += u*cos(p->theta);
    p->theta += theta_i;
    //if (p->theta > M_PI) p->theta = (p->theta)*(-1)+(p->theta-M_PI);
    //if (p->theta < -M_PI) p->theta = (p->theta)*(-1)+(p->theta+M_PI);
}

void update_motcon(motiontype *p)
{
    if (p->cmd != 0) {
        p->finished = 0;
        switch (p->cmd) {
            case mot_stop:
                p->curcmd = mot_stop;
                break;
            case mot_move:
                p->startpos = (p->left_pos + p->right_pos) / 2;
                p->curcmd = mot_move;
                break;
            case mot_follow:
                p->startpos = (p->left_pos + p->right_pos) / 2;
                p->curcmd = mot_follow;
                break;
            case mot_turn:
                p->startpos = odo.theta;
                p->curcmd = mot_turn;
                break;
        }
        p->cmd = 0;
    }

    switch (p->curcmd) {
        case mot_stop:
            p->motorspeed_l = 0;
            p->motorspeed_r = 0;
            break;
        case mot_move:
            if ((p->right_pos + p->left_pos) / 2 - p->startpos > p->dist) {
                p->finished = 1;
                p->motorspeed_l = 0;
                p->motorspeed_r = 0;
            } else {
                angular_controller(&odo);
                if (odo.delta_v > LIMIT) odo.delta_v = LIMIT;
                p->motorspeed_l = p->speedcmd - (odo.delta_v/2);  // exercie 7 1
                p->motorspeed_r = p->speedcmd + (odo.delta_v/2);  // exercie 7 1
            }
            break;
        case mot_follow:
            if ((p->right_pos + p->left_pos) / 2 - p->startpos > p->dist) {
                p->finished = 1;
                p->motorspeed_l = 0;
                p->motorspeed_r = 0;
            } else {
                //mot.ls = linesensorMinimumIntensity(linesensor);
                mot.ls = centerOfMass();
                line_controller(&mot);
                if (mot.delta_v > LIMIT) mot.delta_v = LIMIT;
                p->motorspeed_l = p->speedcmd + (mot.delta_v/2);  // exercie 7 1
                p->motorspeed_r = p->speedcmd - (mot.delta_v/2);  // exercie 7 1
            }
            break;
        case mot_turn:
            if (fabs(odo.theta - p->startpos) < fabs(p->angle)) {
                if (p->angle > 0) {
                    p->motorspeed_r = p->speedcmd / 2;
                    p->motorspeed_l = -p->speedcmd / 2;
                } else {
                    p->motorspeed_r = -p->speedcmd / 2;
                    p->motorspeed_l = p->speedcmd / 2;
                }
            } else {
                p->motorspeed_l = 0;
                p->motorspeed_r = 0;
                p->finished = 1;
            }
            break;
    }
}

int fwd(double dist, double speed, int time) {
    static double vMax;
    if (time == 0) {
        mot.cmd = mot_move;
        mot.speedcmd = 0;
        mot.dist = dist;
        vMax = 0;
        return 0;
    } else {
        if (mot.speedcmd < speed && vMax == 0) {
            mot.speedcmd = SAMPLE_RATE*LIMIT*time;
            if(mot.speedcmd > speed) mot.speedcmd = speed;
        } else {
            vMax = sqrt(2 * LIMIT * (mot.dist - ((mot.right_pos + mot.left_pos) / 2 - mot.startpos)));
            if (mot.speedcmd > vMax) mot.speedcmd = vMax;
        }
        return mot.finished;
    }
}

int follow(double dist, double speed, int dir,int time) {
    static double vMax;
    if (time == 0) {
        mot.cmd = mot_follow;
        mot.speedcmd = 0;
        mot.dist = dist;
        mot.ls_ref = dir;
        vMax = 0;
        return 0;
    } else {
        if (mot.speedcmd < speed && vMax == 0) {
            mot.speedcmd = SAMPLE_RATE*LIMIT*time;
            if(mot.speedcmd > speed) mot.speedcmd = speed;
        } else {
            vMax = sqrt(2 * LIMIT * (mot.dist - ((mot.right_pos + mot.left_pos) / 2 - mot.startpos)));
            if (mot.speedcmd > vMax) mot.speedcmd = vMax;
        }

        return mot.finished;
    }
}

int turn(double angle, double speed, int time) {
    static double vMax;
    if (time == 0) {
        mot.cmd = mot_turn;
        mot.speedcmd = 0;
        mot.angle = angle;
        vMax = 0;
        return 0;
    } else
    if (mot.speedcmd < speed && vMax == 0) {
        mot.speedcmd = SAMPLE_RATE * LIMIT * time;
        if(mot.speedcmd > speed) mot.speedcmd = speed;
    } else {
        vMax = sqrt(2 * LIMIT * (K1*(fabs(mot.angle) - fabs(odo.theta - mot.startpos))*(180/M_PI)));
        if (mot.speedcmd >= vMax && vMax >= 0.025) mot.speedcmd = vMax;
    }
    odo.theta_ref = odo.theta;
    return mot.finished;
}

void angular_controller(odotype *p)   // exercise 7 1
{
    p->delta_v = K*(p->theta_ref-p->theta);
}

double blackWhiteTrans(double inputValue, int sensorNo)
{

    double blackArray[] = {45.4114832535885, 46.6411483253589, 45.4593301435407, 45.5119617224880,
                           45.8612440191388, 45.6124401913876, 46.1052631578947, 45.8277511961723};

    double whiteArray[] = {64.3253588516746, 62.8325358851675, 65.1913875598086, 63.1148325358852,
                           76.9952153110048, 72.4019138755981, 67.5598086124402, 61.3732057416268};

    double diffValue = whiteArray[sensorNo]-blackArray[sensorNo];
    return (inputValue - whiteArray[sensorNo] + diffValue)/diffValue;
}

int linesensorMinimumIntensity(symTableElement *linesensor)
{
    int index = 0;
    double calibD;
    double minim = blackWhiteTrans(linesensor->data[0], 0);
    for (int i = 1; i < linesensor->length; i++){
        calibD = blackWhiteTrans(linesensor->data[i], i);
        if (calibD < minim){
            index = i;
            minim = calibD;
        }
    }
    return index;
}

void line_controller(motiontype *p)   // exercise 7 1
{
    p->delta_v = K_LINE*(p->ls_ref-p->ls);
}

double centerOfMass()   // exercise 7 1
{
    double num = 0, det = 0;
    for (int i = 0; i < linesensor->length; i++){
        num += (i+1)*(1-blackWhiteTrans(linesensor->data[i], i));
        det += (1-blackWhiteTrans(linesensor->data[i], i));
    }
    double res = num/det;
    //printf("Indez = %0.5f\n", res);
    return res;
}

void sm_update(smtype *p) {
    if (p->state != p->oldstate) {
        p->time = 0;
        p->oldstate = p->state;
    } else {
        p->time++;
    }
}



