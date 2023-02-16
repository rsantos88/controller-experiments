#include "Cia402device.h"
#include "SocketCanPort.h"
#include "fcontrol.h"
#include <iostream>
#include <chrono>
#include <thread>


int main ()
{
    double targetVel = 0; // input velocity in deg/s
    targetVel = targetVel *M_PI/180; // 1 ° × π / 180 = 0,01745 rad

    // variables to calculate velocity
    double previousPos = 0;
    std::chrono::time_point<std::chrono::system_clock> previousTime = std::chrono::system_clock::now();

    //control loop sampling time
    double freq=100; //sensor use values: 50,100,500...
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
    //PIDBlock controller(0.000173,0.00153,0,dts); // Kp=1, Ki=1, Kd=0
    //PIDBlock controller(0.000184,0.000905,0,dts); // Kp=1, Ki=1, Kd=0
    PIDBlock controller(0,5.36e-05,0,dts); // Kp=1, Ki=1, Kd=0
    //FPDBlock fraccController(1.2105e-07, 5.2368e-07, -0.9000, dts); //double new_kp = 1, double new_kd = 1, double new_fex = 1, double new_dts = 0.001

    //printf("init pos: %f\n", m1.GetPosition()*180/M_PI);
    for(double i=0; i<=120; i=i+dts){ // 1seg

        double currentVel = m1.GetVelocityTP();

        double velError = targetVel - currentVel;

        // Control process
        double cS = controller.OutputUpdate(velError);

        if (!std::isnormal(cS))
        {
            cS = 0.0;
        }

        m1.SetTorque(cS);
        //printf("GetVelocityTP: %f \n", currentVel*180/M_PI);
        printf("currentVel: (%f) velError: (%f)  controlSignal: (%f) timeLoop: (%f)\n", currentVel*180/M_PI, velError, cS, i);
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
