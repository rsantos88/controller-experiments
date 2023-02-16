#include "Cia402device.h"
#include "SocketCanPort.h"
#include <iostream>
#include <chrono>
#include "fcontrol.h"

/* Programa para identificar el perfil del motor en modo velocidad */

int main ()
{
    double targetVel = -360; // input velocity in deg/s
    targetVel = targetVel *M_PI/180; // 1 ° × π / 180 = 0,01745 rad

    //control loop sampling time
    double freq=50; //sensor use values: 50,100,500...
    double dts=1/freq;
    SamplingTime Ts;
    Ts.SetSamplingTime(dts); // 0.020

    // file results
    ofstream testingFile;
    testingFile.open("../data.csv", ofstream::out);
    testingFile << "Time, TargetVel, CurrentVel "<< endl;

    //m1 setup
    SocketCanPort pm1("can0");
    CiA402SetupData sd1(2048,24,0.001, 0.144, 10);
    CiA402Device m1 (1, &pm1, &sd1);
    m1.StartNode();
    m1.SwitchOn();
    m1.Setup_Velocity_Mode(10);

    m1.SetVelocity(targetVel);

    for(double i=0; i<=10; i=i+dts){ // 1seg

        printf("timeLoop: %f, targetVel (rad/s): %f, currentVel (rad/s): %f\n", i, targetVel, m1.GetVelocity());
        testingFile << i << "," << targetVel << "," << m1.GetVelocity() << endl;
        Ts.WaitSamplingTime();
    }

    m1.SetVelocity(0);

    m1.ForceSwitchOff();
    sleep(1);
    m1.PrintStatus();
    return 1;
}
