#include "Cia402device.h"
#include "SocketCanPort.h"
#include "fcontrol.h"
#include <iostream>
#include <chrono>
#include <thread>

/* Test de un solo motor haciendo una onda senoidal en modo velocidad */

int main ()
{
    // variables to calculate velocity
    std::chrono::time_point<std::chrono::system_clock> previousTime = std::chrono::system_clock::now();

    //control loop sampling time
    double freq=50; //sensor use values: 50,100,500...
    double dts=1/freq;
    SamplingTime Ts;
    Ts.SetSamplingTime(dts); // 0.020

    // file results
    ofstream testingFile;
    testingFile.open("../data.csv", ofstream::out);
    testingFile << "Time, TargetVel, CurrentVel, CurrentPos" << endl;

    //m1 setup
    SocketCanPort pm1("can0");
    CiA402SetupData sd1(2048,24,0.001, 0.144, 10);
    CiA402Device m1 (1, &pm1, &sd1);
    m1.StartNode();
    m1.SwitchOn();
    m1.Setup_Velocity_Mode(10);

    // controller for motor
    PIDBlock controller(0.015,36,0,dts);

    //Trayectoria senoidal
    double A = 1; //Amplitud de la onda senoidal
    double targetVel; // señal de salida senoidal

    for(double t=0; t<=60; t+=dts){ // 1seg

        targetVel = A* sin(1.0*t);

        double currentVel = m1.GetVelocityTP();
        double velError = targetVel - currentVel;

        // Control process
        double cS = controller.OutputUpdate(velError);

        if (!std::isnormal(cS))
        {
            cS = 0.0;
        }

        m1.SetVelocity(cS);
        printf("currentVel: (%f) velError: (%f)  controlSignal: (%f)\n", currentVel*180/M_PI, velError, cS);
        testingFile << t << "," << targetVel << "," << currentVel << "," << m1.GetPosition() << "," << endl;
        Ts.WaitSamplingTime();
    }    
    
    m1.SetVelocity(0);
    printf("final pos: %f\n", m1.GetPosition()*180/M_PI);

    m1.ForceSwitchOff();
    sleep(1);
    m1.PrintStatus();
    return 1;
}
