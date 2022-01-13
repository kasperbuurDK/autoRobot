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
#include <jmorecfg.h>
#include "rhd.h"
#include "componentserver.h"
#include "xmlio.h"


/* TO DO
 * Follow line give Seg. fault when no line is found.
 * An example SMR program.
 *
 */
#define SIM
#ifdef SIM
#define WHITE 128
#define BLACK 0
#define ROBOTPORT    8000
#else
const double blackArray[] = {45.4114832535885, 46.6411483253589, 45.4593301435407, 45.5119617224880,
                           45.8612440191388, 45.6124401913876, 46.1052631578947, 45.8277511961723};

    const double whiteArray[] = {64.3253588516746, 62.8325358851675, 65.1913875598086, 63.1148325358852,
                           76.9952153110048, 72.4019138755981, 67.5598086124402, 61.3732057416268};
    #define WHITE 61
    #define BLACK 46
    #define ROBOTPORT    24902 //24902
#endif

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
#define LOG_SIZE 40000 //-------!!!!!----------
#define SAMPLE_RATE 0.01
#define LIMIT 0.5
#define LINE_ERROR_OFFSET 0.01 // non-systematic error offset for line sensor crossing detection.
#define ORIGO_TO_LASER 0.26 // length from origo to laser unit.
#define ORIGO_TO_LINE 0.23 // length from origo to laser unit.
#define SMR_LENGTH 0.36

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
#define K_LINE 0.2

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
    int dir;
} motiontype;

typedef struct {
    int state, oldstate, prevstate, stage, oldstage, prevstage;
    int time;
} smtype;

typedef struct {
    double x, y, theta;
} pose;

enum {
    mot_stop = 1, mot_move, mot_follow,mot_turn
};
enum {
    ms_init, ms_obs1, ms_obs2, ms_obs3, ms_obs4, ms_obs5, ms_obs6, ms_end, ms_error
};
enum {
    sbms_init, sbms_stage1, sbms_stage2, sbms_stage3, sbms_stage4, sbms_stage5, sbms_stage6, sbms_stage7, sbms_stage8, sbms_stage9, sbms_stage10, sbms_stage11, sbms_stage12, sbms_end
};
enum {
    left = 6, middel = 4, right = 2
};


void update_motcon(motiontype *p);
void angular_controller(odotype *p);
int linesensorMinimumIntensity(symTableElement *linesensor);
double blackWhiteTrans(double, int);
int fwd(double dist, double speed, int time);
int turn(double angle, double speed, int time);
void sm_update(smtype *p);
int follow(double dist, double speed, int dir, int linePos, int time);
void line_controller(motiontype *p, int ref);
double centerOfMass();
boolean isCrossing();
boolean checkLaserDist(double dist, int zone);
boolean checkLines(int lineLow, int lineHigh);

// SMR input/output data
symTableElement *inputtable, *outputtable;
symTableElement *lenc, *renc, *linesensor, *irsensor, *speedl, *speedr, *resetmotorr, *resetmotorl;
odotype odo;
smtype mission;
motiontype mot;
dataPacket dataLog[LOG_SIZE];
int missonCount = 0;

int main()
{
    int running, n = 0, arg, time = 0, zone;
    int timeStamp1 = 1, timeStamp2 = -1;
    double dist = 0, angle = 0, speed = 0, wallDist = 0;
    pose poseA, poseB;
    poseA.x = 0;
    poseB.x = 0;
    poseA.y = 0;
    poseB.y = 0;




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
    mot.w = odo.w;
    running = 1;
    mission.state = ms_init;
    mission.stage = sbms_init;
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
                dist = 2;
                speed = 0.2;
                n = 4;
                angle = (90.0) / 180 * M_PI;
                if (!checkLaserDist(wallDist,zone)) {
                    printf("Laser init!\n");
                    mission.state = ms_obs3;
                }
                /*printf("Line Data = ");
                for (int i = 0; i < linesensor->length; i++){
                    printf("%d = %d , ",i,linesensor->data[i]);
                }
                printf("\n");*/
                break;
            case ms_obs1:  // Start 3.4 -0.1 1.57  --------- DONE -------------
                switch (mission.stage) {
                    case sbms_init:
                        dist = 10;
                        speed = 0.2; // 0.2 gives +-0.06, 0.4 gives +-0.12
                        wallDist = 0.3;
                        zone = 5; // 1 -> 9 zones of laser sensor.
                        angle = (180.0) / 180 * M_PI;
                        mission.stage = sbms_stage1;
                        break;
                    case sbms_stage1:
                        if (follow(dist, speed, right, middel, mission.time)) mission.state = ms_error;
                        if (checkLaserDist(wallDist,zone)) {
                            printf("Close!\n");
                            mot.cmd = mot_stop;
                            mission.stage = sbms_stage2;
                        }
                        break;
                    case sbms_stage2:
                        dist = fmin(laserpar[4],laserpar[5]);
                        dist += (fabs(odo.y) + ORIGO_TO_LASER);
                        printf("OBSTACLE 1: Distance to box = %0.3f\n",dist);
                        speed = 0.4;
                        dist = 10;
                        mission.stage = sbms_stage3;
                        break;
                    case sbms_stage3: // prepare for obs2
                        if (turn(angle, speed, mission.time)) {
                            angle = odo.theta;
                            mission.stage = sbms_stage4;
                        }
                        break;
                    case sbms_stage4:
                        if (follow(dist, speed, right, left, mission.time)) mission.stage = sbms_stage5;
                        if (checkLines(middel, left+1)){
                            mot.dist = ORIGO_TO_LINE;
                            angle = odo.theta*(-1);
                            mot.cmd = mot_follow;
                        }
                        break;
                    case sbms_stage5:
                        if (turn(angle, speed, mission.time)) mission.stage = sbms_stage6;
                        if (checkLines(left-1, left)){
                            angle = (-30.0) / 180 * M_PI;
                            mot.cmd = mot_turn;
                        }
                        break;
                    case sbms_stage6:
                        if (follow(dist, speed, left, middel, mission.time)) mission.state = ms_error;
                        if (checkLaserDist(wallDist,zone)) {
                            printf("Close!\n");
                            mot.cmd = mot_stop;
                            mission.state = ms_obs2;
                        }
                        break;
                }
                break;
            case ms_obs2: //Start 4.82 1.706 0.015
                switch (mission.stage) {
                    case sbms_init:
                        dist = 10;
                        speed = 0.1;
                        wallDist = 0.3;
                        zone = 5; // 1 -> 9 zones of laser sensor.
                        angle = (-90.0) / 180 * M_PI;
                        mission.stage = sbms_stage1;
                        break;
                    case sbms_stage1:
                        if (follow(dist, speed, middel, middel, mission.time)) mission.state = ms_error;
                        if (isCrossing()) {
                            mission.stage = sbms_stage2;
                        }
                        break;
                    case sbms_stage2:
                        if (fwd(dist, speed, mission.time)) mission.state = ms_error;
                        if (checkLaserDist(wallDist,1) && checkLaserDist(wallDist,9)) {
                            printf("Close!\n");
                            mot.cmd = mot_stop;
                            speed = 0.3;
                            mission.stage = sbms_stage3;
                        }
                        break;
                    case sbms_stage3:
                        if (fwd(-dist, speed, mission.time)) mission.state = ms_error;
                        if (isCrossing()) {
                            mission.stage = sbms_stage4;
                        }
                        break;
                    case sbms_stage4:
                        if (checkLaserDist(wallDist,1) && checkLaserDist(wallDist,9)) {
                            printf("Close!\n");
                            mission.stage = sbms_stage5;
                        }
                        break;
                    case sbms_stage5:
                        if (!checkLaserDist(wallDist,1) && !checkLaserDist(wallDist,9)) {
                            printf("Far!\n");
                            mot.dist = ORIGO_TO_LINE;
                            mot.cmd = mot_follow;
                            mission.stage = sbms_stage6;
                        }
                        break;
                    case sbms_stage6:
                        if (turn(angle, speed, mission.time)) {
                            angle = odo.theta;
                            mission.stage = sbms_stage7;
                        }
                        break;
                    case sbms_stage7:
                        if (fwd(dist, speed, mission.time)) mission.stage = sbms_stage8;;
                        if (isCrossing()) {
                            mot.dist = ORIGO_TO_LINE;
                            mot.cmd = mot_move;
                            angle = (90.0) / 180 * M_PI;
                        }
                        break;
                    case sbms_stage8:
                        if (turn(angle, speed, mission.time)) {
                            angle = odo.theta;
                            mission.stage = sbms_stage9;
                        }
                        break;
                    case sbms_stage9:
                        if (follow(dist, speed, middel, middel, mission.time)) mission.stage = sbms_stage10;;
                        if (isCrossing()) {
                            mot.dist = ORIGO_TO_LINE;
                            mot.cmd = mot_move;
                            angle = (90.0) / 180 * M_PI;
                        }
                        break;
                    case sbms_stage10:
                        if (turn(angle, speed, mission.time)) mission.stage = sbms_stage11;
                        break;
                    case sbms_stage11:
                        if (follow(dist, speed, middel, middel, mission.time)) mission.state = ms_error;
                        if (isCrossing()) {
                            mot.cmd = mot_move;
                            mission.stage = sbms_stage12;
                        }
                        break;
                    case sbms_stage12:
                        if (!isCrossing()) {
                            mission.stage = sbms_end;
                        }
                        break;
                    case sbms_end:
                        if (follow(dist, speed, middel, middel, mission.time)) mission.state = ms_error;
                        if (isCrossing()) {
                            mission.state = ms_end;
                        }
                        break;
                }
                break;

            case ms_obs3:
                switch (mission.stage) {
                    case sbms_init:
                        dist = 4;
                        speed = 0.2;
                        mission.stage = sbms_stage1;
                        timeStamp1 = -1;
                        timeStamp2 = -1;

                        break;

                    case sbms_stage1:
                        if (follow(dist, speed, middel, middel, mission.time)) mission.stage = sbms_stage2;

                        if (irsensor->data[0] > 94 && timeStamp1 == -1) {
                            printf("Found first pole of gate\n");
                            poseA.x = odo.x;
                            poseA.y = odo.y;
                            poseA.theta = odo.theta;
                            timeStamp1 = mission.time;
                            printf("Pose is %f and %f\n", poseA.x, poseA.y);
                        }

                        if ((timeStamp1 != -1) && (irsensor->data[0] == 94) && (timeStamp2 == -1)) {
                            printf("Cleared pole 1\n");
                            timeStamp2 = mission.time;

                        }

                        if (irsensor->data[0] > 94 && timeStamp2 != -1){
                            printf("Found second pole of gate\n");
                            poseB.x = odo.x;
                            poseB.y = odo.y;
                            poseA.theta = odo.theta;
                            printf("Pose is %f and %f\n", poseA.x, poseA.y);
                            dist = sqrt(pow(poseB.x - poseA.x, 2) + pow(poseB.y - poseA.y, 2));
                            printf("Dist  is %f\n", dist);
                            // mission.stage = sbms_stage2;

                            mot.dist = -dist;
                            mot.cmd = mot_move;
                            angle = (90.0) / 180 * M_PI;
                            speed = 0.1;

                        }
                        break;

                    case sbms_stage2:

                        if (turn(angle, speed, mission.time )) {
                            dist = 2.5*SMR_LENGTH;
                            mission.stage = sbms_stage3;
                        }
                        break;

                    case sbms_stage3:
                        if (fwd(dist, speed, mission.time)) {
                            mission.stage = sbms_stage4;
                            angle = -(M_PI/2);
                        }

                        break;

                    case sbms_stage4:
                        if (turn(angle, speed, mission.time)){
                            dist = 1.5;
                            mission.stage = sbms_stage5;
                        }

                        break;

                    case sbms_stage5:
                        if (fwd(dist, speed, mission.time)) {
                            mission.stage = sbms_stage6;

                        }

                        if (isCrossing()){
                            mot.dist = ORIGO_TO_LINE;
                            mot.cmd = mot_move;
                            angle = M_PI/2;
                        }
                        break;

                    case sbms_stage6:
                        if (turn(angle, speed, mission.time)){
                            mission.stage = sbms_end;
                        }
                        break;
                    case sbms_end:

                        mission.state = ms_obs4;
                        mission.stage = sbms_init;
                        break;

                    default:
                        break;

                }

                break;


            case ms_obs4:

                switch (mission.stage) {

                    case sbms_init:
                        dist = 4;
                        speed = 0.2;
                        mission.stage = sbms_stage1;

                        break;
                    case sbms_stage1: // follow line until reach crossing

                        if (follow(dist, speed, middel, middel, mission.time)) {
                            printf("STOPPED IN obs3_STAGE1 because of reached distance\n");
                            mission.stage = sbms_end;
                        }
                        if (isCrossing()) {
                            mission.stage = sbms_stage2;
                            printf("FOUND CROSS IN MS_obs3 - SBMS1");
                            angle = M_PI/2;
                            speed = 0.2;
                            dist = 10;

                        }
                        break;
                    case sbms_stage2:   //turn left 90 degrees

                        if (turn(angle, speed, mission.time)) {
                            dist = 4;
                            speed = 0.2;
                            printf("Done turning, ready to follow wall\n");
                            timeStamp1 = -1;
                            timeStamp2 = -1;
                            mission.stage = sbms_stage3;

                        }

                        break;
                    case sbms_stage3:  //follow wall until opening is marked
                        //
                        if (fwd(dist, speed, mission.time)) {
                            printf("STOPPED in ms_OBS3-sbms_stage3 due to reached dist");
                            mission.stage = ms_end;
                        }
                        //TODO implement correction relative to wall, a wall controller

                        if (mission.time > 100) {
                            if (irsensor->data[0] == 94 && timeStamp1 == -1) {
                                printf("Found opening in wall\n");
                                poseA.x = odo.x;
                                poseA.y = odo.y;
                                poseA.theta = odo.theta;
                                timeStamp1 = mission.time;
                                printf("Pose is %f and %f\n", poseA.x, poseA.y);
                            }
                            if (timeStamp1 != -1 && irsensor->data[0] > 94) {
                                printf("WALL opening ended\n");
                                timeStamp2 = mission.time;
                                poseB.x = odo.x;
                                poseB.y = odo.y;
                                printf("Pose is %f and %f\n", poseB.x, poseB.y);
                                poseA.theta = odo.theta;
                                dist = sqrt(pow(poseB.x - poseA.x, 2) + pow(poseB.y - poseA.y, 2));
                                printf("Dist  is %f\n", dist);
                                angle = M_PI/2;

                                mission.stage = sbms_stage5;
                            }
                        }
                        break;

                    case sbms_stage4: //TODO get this to work // backwards half the distance of the gate
                        if (fwd(-dist, speed, mission.time)) {
                            angle = M_PI/2;
                            mission.stage = sbms_stage5;
                        }
                        break;

                    case sbms_stage5:  // turn left to face gate
                        if (turn(angle, speed, mission.time)) {
                            dist = 2*SMR_LENGTH;
                            speed = 0.1;
                            mission.stage = sbms_stage6;
                        }
                        break;

                    case sbms_stage6:   // drive through gate
                        if (fwd(dist, speed, mission.time)) {

                            angle = M_PI/2;
                           mission.stage = sbms_stage7;
                        }



                        break;

                    case sbms_stage7:   // turn left 90 degress to follow wall
                        if (turn(angle, speed, mission.time)) {
                            mission.stage = sbms_stage8;
                            speed = 0.2;
                            dist = 2.5;
                        }
                        break;

                    case sbms_stage8: //

                        if (fwd(dist, speed, mission.time)) {
                           angle = M_PI/2;
                            mission.stage = sbms_stage9;
                        }

                        if (isCrossing()){
                            mot.dist = ORIGO_TO_LINE;
                            mot.cmd = mot_move;
                        }

                        break;

                    case sbms_stage9:
                        if (turn(angle, speed, mission.time)) {
                         dist = 2;
                         speed = 0.2;
                         mission.stage = sbms_stage10;
                        }
                        break;

                    case sbms_stage10:
                        if (fwd(dist, speed, mission.time)){
                            mission.stage = sbms_end;
                        }

                        if (isCrossing()){
                            mot.dist = ORIGO_TO_LINE;
                            mot.cmd = mot_move;
                        }
                        break;

                    case sbms_end:
                        printf("END of obs4\n");
                        printf("POSE %f , %f, %f \n", odo.x, odo.y, odo.theta);
                        dist = 3;
                        speed = 0.2;
                        mission.state = ms_end;
                        mission.stage = sbms_init;
                        break;
                    default:
                        break;

                }
                break;


            case ms_obs5:
                if (fwd(dist, speed, mission.time)) mission.state = ms_end;
                break;
            case ms_obs6:
                if (fwd(dist, speed, mission.time)) mission.state = ms_end;
                break;
            case ms_end:
                mot.cmd = mot_stop;
                running = 0;
                printf("End: State = %d : Stage = %d\n", mission.prevstate, mission.prevstage);
                break;
            case ms_error:
                mot.cmd = mot_stop;
                running = 0;
                printf("Error: State = %d : Stage = %d\n", mission.prevstate, mission.prevstage);
                break;
        }
        /*  end of mission  */
        dataLog[missonCount].x = mot.ls;
        dataLog[missonCount].y = mot.ls_ref;
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
        fprintf(fp, "%d,%d,%0.8f,%0.8f\n", i, dataSample.time, dataSample.x, dataSample.y);
    }
    printf("Laser: ");
    for (int i = 0 ; i < 10; i++){
        printf("%d = %0.3f ", i, laserpar[i]);
    }
    printf("\n");
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
            if ((p->dist >= 0 && (p->right_pos + p->left_pos) / 2 - p->startpos >= p->dist)||(p->dist < 0 && p->startpos - (p->right_pos + p->left_pos) / 2 >= abs(p->dist))) {
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
                //printf("CoM = %d\n", mot.ls);
                int ref = mot.ls_ref;
                int lsInt = (int) ceil(mot.ls);
                if (blackWhiteTrans(linesensor->data[lsInt], lsInt) != BLACK){
                    if (mot.dir == left) {
                        ref = right;
                    } else {
                        ref = left;
                    }
                }
                line_controller(&mot, ref);
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
        if (mot.dist < 0) mot.speedcmd = -mot.speedcmd;
        return mot.finished;
    }
}

int follow(double dist, double speed, int dir, int linePos, int time) {
    static double vMax;
    if (time == 0) {
        mot.cmd = mot_follow;
        mot.speedcmd = 0;
        mot.dist = dist;
        mot.ls_ref = linePos;
        mot.dir = dir;
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
        if (mot.speedcmd >= vMax && vMax >= LIMIT/2) mot.speedcmd = vMax;
    }
    odo.theta_ref = odo.theta;
    return mot.finished;
}

void angular_controller(odotype *p)   // exercise 7 1
{
    p->delta_v = K*(p->theta_ref-p->theta);
}

#ifdef SIM
double blackWhiteTrans(double inputValue, int dump)
{
    double diffValue = WHITE-BLACK;
    return (inputValue - WHITE + diffValue)/diffValue;
}
#else
double blackWhiteTrans(double inputValue, int sensorNo)
{
    double diffValue = whiteArray[sensorNo]-blackArray[sensorNo];
    return (inputValue - whiteArray[sensorNo] + diffValue)/diffValue;
}
#endif



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

void line_controller(motiontype *p, int ref)
{
    p->delta_v = K_LINE*(ref-p->ls);
}

double centerOfMass()
{
    double num = 0, det = 0;
    for (int i = 0; i < linesensor->length; i++){
        num += (i+1)*(1-blackWhiteTrans(linesensor->data[i], i));
        det += (1-blackWhiteTrans(linesensor->data[i], i));
        //printf("%d, ", linesensor->data[i]);
    }
    double res = mot.ls_ref; // default middel sensor -> drive straight.
    if (det != 0) res = num/det; // Seg fault prevention
    //printf("res = %0.4f\n",res);
    return res-1; // Index in linesensor data array.
}

boolean isCrossing()
{
    double sum = 0;
    for (int i = 0; i < linesensor->length; i++){
        if (blackWhiteTrans(linesensor->data[i], i) <= LINE_ERROR_OFFSET){
            sum++;
        }
    }
    //printf("sum = %0.4f\n", sum);
    dataLog[missonCount].theta = odo.theta;
    if (sum >= linesensor->length-2) return TRUE;
    return FALSE;
}

boolean checkLaserDist(double dist, int zone)
{
    //printf("Laser = %0.4f -> odoy = %0.4f\n", laserpar[zone-1], fabs(odo.y));
    if (laserpar[zone-1] <= dist) return TRUE;
    return FALSE;
}

boolean checkLines(int lineLow, int lineHigh){
    for (int i = lineLow-1; i < lineHigh; i++){
        //printf("Line = %d -> %d\n", i, linesensor->data[i]);
        if (linesensor->data[i] == BLACK) return TRUE;
    }
    return FALSE;
}

void sm_update(smtype *p)
{
    if (p->stage != p->oldstage) {
        p->time = 0;
        p->prevstage = p->oldstage;
        p->oldstage = p->stage;
    } else {
        p->time++;
    }
    if (p->state != p->oldstate) {
        p->stage = ms_init;
        p->prevstate = p->oldstate;
        p->oldstate = p->state;
    }
}



