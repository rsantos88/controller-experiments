#include "Cia402device.h"
#include "CiA301CommPort.h"
#include "SocketCanPort.h"
#include <iostream>
#include <stdio.h>
#include "fcontrol.h"
#include "IPlot.h"
#include <chrono>

#include "imu3dmgx510.h"

#define LG0 0.003 //meters
#define RADIO 0.0075 //meters

int main ()
{
    // inputs
    vector<double> targetPose{0, -20}; // {roll, pitch}
    double targetTime = 120; //seconds

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


    // SENSOR
    double freq=50; //sensor use values: 50,100,500...
    IMU3DMGX510 imu("/dev/ttyUSB0",freq);
    double pitch,roll, yaw;

    //Sampling time
    double dts=1/freq;
    SamplingTime Ts;
    Ts.SetSamplingTime(dts);

    // Controller
    std::vector<double> ierror(2); // ERROR
    std::vector<double> cs(2); //CONTROL SIGNAL


    PIDBlock controllerRollPosition(2.6, 20.85, 0, dts);
    PIDBlock controllerPitchPosition(2.6, 20.85, 0, dts);

    imu.set_streamon();

    for (double t=0;t<10;t+=dts)
    {
       imu.GetPitchRollYaw(pitch,roll,yaw);
    }
    cout<<"Calibrated"<<endl;

    std::vector<double> xd(2);

    // file results
    ofstream testingFile;
    testingFile.open("../data.csv", ofstream::out);
    testingFile << "Time, Roll target, Pitch target, Roll IMU, Pitch IMU" << endl;

    auto start = std::chrono::system_clock::now();

    while(1)
    {

        imu.GetPitchRollYaw(pitch,roll,yaw);

        double rollError  = targetPose[0] - roll*180/M_PI;
        double pitchError = targetPose[1] - pitch*180/M_PI;

        //Control process
        double rollCs = controllerRollPosition.OutputUpdate(rollError);
        if (!std::isnormal(rollCs))
        {
            rollCs = 0.0;
        }
        xd[0] = rollCs;

        double pitchCs = controllerPitchPosition.OutputUpdate(pitchError);
        if (!std::isnormal(pitchCs))
        {
            pitchCs = 0.0;
        }
        xd[1] = pitchCs;

        /* Conversions:
         * x[0]      -> roll
         * x[1]      -> pitch
         * l1,l2,l3  -> tendon velocities in meters/seg */

        double l1 =  0.001 * (xd[1]/1.5);
        double l2 =  0.001 * ( (xd[0]/1.732) -(xd[1]/3)  );
        double l3 = -0.001 * ( (xd[1]/3) + (xd[0]/1.732) );

        double posAng1 = (LG0-l1)/RADIO;
        double posAng2 = (LG0-l2)/RADIO;
        double posAng3 = (LG0-l3)/RADIO;

        auto current = std::chrono::system_clock::now();
        std::chrono::duration<float,std::milli> duration = current - start;

        cout << "-----------------------------\n" << endl;
        cout << "-> Time:" << duration.count() << endl;
        cout << "-> Roll: " << roll*180/M_PI << "  Pitch: " << pitch*180/M_PI << endl;
        cout << "-> RollTarget: " << targetPose[0] << " PitchTarget:  " << targetPose[1] << " >>>>> RollError: " << rollError << "  PitchError: " << pitchError << endl;
        cout << "-> Angular position: >>>>> V1: " << posAng1 << " V2: " << posAng2 << " V3: " << posAng3 << endl;
        cout << "-----------------------------\n" << endl;

        //Uncomment it to receive data from testing
        testingFile << duration.count() << "," << targetPose[0] << "," << targetPose[1] << "," << roll*180/M_PI << "," << pitch*180/M_PI << endl;

        m1.SetPosition(posAng1);
        m2.SetPosition(posAng2);
        m3.SetPosition(posAng3);

        Ts.WaitSamplingTime();
    }

    testingFile.close();

    m1.ForceSwitchOff();
    m2.ForceSwitchOff();
    m3.ForceSwitchOff();
    sleep(1);
    m1.PrintStatus();
    m2.PrintStatus();
    m3.PrintStatus();


    return 0;

}

