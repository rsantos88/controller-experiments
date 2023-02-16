#include "Cia402device.h"
#include "SocketCanPort.h"
#include "imu3dmgx510.h"
#include <iostream>
#include <chrono>
#include <thread>

#include "imu3dmgx510.h"
#include "fcontrol.h"

/* Programa test en modo posicion del cuello en bucle abierto */

// pitch, roll to lenght of tendons in meters
void pr2tendons(double pitch, double roll, std::vector<double> & lengths)
{
    //degrees to rad (motor units)
    //pitch = pitch*M_PI/180;
    //roll = roll*M_PI/180;

    // pitch & roll -> lenghts
    lengths[0] = -0.001* (pitch/1.5);
    lengths[1] =  0.001* (pitch/3 - roll/1.732);
    lengths[2] =  0.001* (pitch/3 + roll/1.732);
}

int main ()
{

    std::chrono::system_clock::time_point start, current;
    double lg0 = 0.003; //meters
    double radio = 0.0075; //meters


    //m1 setup
    SocketCanPort pm1("can0");
    CiA402SetupData sd1(2048,24,0.001, 0.144, 10);
    CiA402Device m1 (1, &pm1, &sd1);
    m1.StartNode();
    m1.SwitchOn();
    m1.SetupPositionMode(10, 10);

    //m2
    SocketCanPort pm2("can0");
    CiA402SetupData sd2(2048,24,0.001, 0.144, 10);
    CiA402Device m2 (2, &pm2, &sd2);
    m2.StartNode();
    m2.SwitchOn();
    m2.SetupPositionMode(10, 10);

    //m3
    SocketCanPort pm3("can0");
    CiA402SetupData sd3(2048,24,0.001, 0.144, 10);
    CiA402Device m3 (3, &pm3, &sd3);
    m3.StartNode();
    m3.SwitchOn();
    m3.SetupPositionMode(10, 10);


    // time per step
    double sleeptime = 1;


    // Input /output
    std::vector<double> pose[2];
    std::vector<double> lengths(3);
    std::vector<double> posan(3);


    // Step inputs (pitch, roll)

    pose[0] = { -10, 0};
    pose[1] = { 10, 0};

    start = std::chrono::system_clock::now();

    for(int p=0; p<5; p++)
    {
        for(int i=0; i<2; i++)
        {
            printf("moving to pose [%d]: [%f] [%f]\n", i, pose[i][0], pose[i][1]);

            pr2tendons(pose[i][0], pose[i][1], lengths);

            posan[0] = (lg0-lengths[0])/radio;
            posan[1] = (lg0-lengths[1])/radio;
            posan[2] = (lg0-lengths[2])/radio;

            printf("tendons: [%f] [%f] [%f]\n", lengths[0], lengths[1], lengths[2]);
            printf("target angular pos : [%f] [%f] [%f]\n", posan[0], posan[1], posan[2]);

            m1.SetPosition(posan[0]);
            m2.SetPosition(posan[1]);
            m3.SetPosition(posan[2]);

            sleep(2);
            }
            printf("stopped\n");
        }


    // home
    m1.SetPosition(0);
    m2.SetPosition(0);
    m3.SetPosition(0);

    sleep(1);
    m1.ForceSwitchOff();
    m2.ForceSwitchOff();
    m3.ForceSwitchOff();

    sleep(1);
    m1.PrintStatus();
    m2.PrintStatus();
    m3.PrintStatus();

    return 0;
}

