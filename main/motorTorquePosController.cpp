#include "Cia402device.h"
#include "SocketCanPort.h"
#include "fcontrol.h"
#include <iostream>
#include <chrono>
#include <thread>

/* Test de control fraccionario en modo Torque */

int main ()
{
    double targetPos = 0; // input velocity in deg/s
    targetPos = targetPos *M_PI/180; // 1 ° × π / 180 = 0,01745 rad

    // variables to calculate velocity
//    double previousPos = 0;
//    std::chrono::time_point<std::chrono::system_clock> previousTime = std::chrono::system_clock::now();

    //control loop sampling time
    double freq=200; //sensor use values: 50,100,500...
    double dts=1/freq;
    SamplingTime Ts;
    Ts.SetSamplingTime(dts); // 0.020

    // file results
//    ofstream testingFile;
//    testingFile.open("../data.csv", ofstream::out);
//    testingFile << "Time, CurrentVel, VelError, ControlSignal" << endl;

    //m1 setup
    SocketCanPort pm1("can0");
    CiA402SetupData sd1(2048,24,0.001, 0.144, 10);
    CiA402Device m1 (1, &pm1, &sd1);
    m1.Reset();
    m1.StartNode();
    m1.SwitchOn();
    m1.Setup_Torque_Mode();

    // controller for motor
    FPDBlock fraccController(5.36e-05, 0, 0, dts); //double new_kp = 1, double new_kd = 1, double new_fex = 1, double new_dts = 0.001

    //printf("init pos: %f\n", m1.GetPosition()*180/M_PI);
    for(double i=0; i<=120; i=i+dts){ // 1seg

        //double currentVel = m1.GetVelocityTP(previousPos, previousTime);
        //previousPos = m1.GetPosition();
        //previousTime = std::chrono::system_clock::now();

        double currentPos = m1.GetPosition();
        double posError = targetPos - currentPos;

        // Control process
        double cS = fraccController.OutputUpdate(posError);

        if (!std::isnormal(cS))
        {
            cS = 0.0;
        }

        m1.SetTorque(cS);
        //printf("GetVelocityTP: %f \n", currentVel*180/M_PI);
        printf("currentPos: (%f) Position Error: (%f)  controlSignal: (%f) timeLoop: (%f)\n", currentPos*180/M_PI, posError, cS, i);
        //testingFile << i << "," << currentVel*180/M_PI << "," << velError << "," << cS << "," << endl;
        Ts.WaitSamplingTime();
    }
    
    //m1.SetVelocity(0);
    //printf("final pos: %f\n", m1.GetPosition()*180/M_PI);

    //m1.ForceSwitchOff();
    //sleep(1);
    m1.PrintStatus();
    return 1;
}
