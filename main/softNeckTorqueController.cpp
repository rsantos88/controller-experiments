#include "Cia402device.h"
#include "SocketCanPort.h"
#include <iostream>
#include <algorithm>
#include <chrono>
#include "fcontrol.h"
#include "imu3dmgx510.h"

/* Programa de controlador de 3 hilos en modo Torque
   Nota: en modo torque no funciona correctamente debido a las limitaciones de potencia de los
   motores */

// pitch, roll to Force ratio of motors
void pr2torques(double pitch, double roll, std::vector<double> & torques)
{
    double li = 0.01;

    //degrees to rad (motor units). Changed the sign
    pitch = - pitch*M_PI/180;
    roll =  - roll*M_PI/180;

    // pitch & roll -> lenghts
    torques[2] =  li - std::min({pitch + (roll/2), roll, 0.0}); // P3
    torques[1] =  roll + torques[2];
    torques[0] =  pitch + 0.5 * (torques[1] + torques[2]);
    //torques[0] = pitch + (roll/2) + torques[2];
}


int main ()
{

    // Step inputs (pitch, roll)
    std::vector<double> targetPose{0,0};
    std::vector<double> f(3);

    //m1 setup
    SocketCanPort pm1("can0");
    CiA402SetupData sd1(2048,24,0.001, 0.144, 10.0);
    CiA402Device m1 (1, &pm1, &sd1);
    m1.Reset();
    m1.StartNode();    
    m1.SwitchOn();
    m1.Setup_Torque_Mode();

    //m2 setup
    SocketCanPort pm2("can0");
    CiA402SetupData sd2(2048,24,0.001, 0.144, 10.0);
    CiA402Device m2 (2, &pm2, &sd2);
    m2.Reset();
    m2.StartNode();
    m2.SwitchOn();
    m2.Setup_Torque_Mode();

    //m3 setup
    SocketCanPort pm3("can0");
    CiA402SetupData sd3(2048,24,0.001, 0.144, 10.0);
    CiA402Device m3 (3, &pm3, &sd3);
    m3.Reset();
    m3.StartNode();
    m3.SwitchOn();
    m3.Setup_Torque_Mode();

    //sensor config
    double freq=50; //sensor use values: 50,100,500...
    double dts=1/freq; // 0.020s
    IMU3DMGX510 imu("/dev/ttyUSB0",freq);

    double pitch,roll, yaw;

    for (double t=0;t<10;t+=dts)
    {
       imu.GetPitchRollYaw(pitch,roll,yaw);
    }
    cout<<"Calibrated"<<endl;

    // controller
    PIDBlock controllerPitch(1.1769, 7.2173, 0, dts);
    PIDBlock controllerRoll(1.1769, 7.2173, 0, dts);

    //Sampling time
    SamplingTime Ts;
    Ts.SetSamplingTime(dts);;

    while(1)
    {
        imu.GetPitchRollYaw(pitch,roll,yaw);

        double pitchError = targetPose[0] - pitch*180/M_PI;
        double rollError  = targetPose[1] - roll*180/M_PI;

        //Control process

        double pitchCs = controllerPitch.OutputUpdate(pitchError);
        if (!std::isnormal(pitchCs))
        {
            pitchCs = 0.0;
        }

        double rollCs = controllerRoll.OutputUpdate(rollError);
        if (!std::isnormal(rollCs))
        {
            rollCs = 0.0;
        }

        pr2torques(0, rollCs, f);

        cout << "-----------------------------\n" << endl;
        cout << "-> Pitch: " << pitch*180/M_PI << " Roll: " << roll*180/M_PI << endl;
        cout << "-> PitchTarget: " << targetPose[0] << " RollTarget:  " << targetPose[1] << " >>>>> PitchError: " << pitchError << "  RollError: " << rollError << endl;
        cout << "-> Motor torques >>>>> t1: " << f[0] << " t2: " << f[1] << " t3: " << f[2] << endl;
        cout << "-----------------------------\n" << endl;

        m1.SetTorque(f[0]);
        m2.SetTorque(f[1]);
        m3.SetTorque(f[2]);

        //std::this_thread::sleep_for(std::chrono::milliseconds(20));
        Ts.WaitSamplingTime();
    }

    return 1;
}
