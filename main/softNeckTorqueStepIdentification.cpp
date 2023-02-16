#include "Cia402device.h"
#include "SocketCanPort.h"
#include <iostream>
#include <algorithm>
#include <chrono>
#include "fcontrol.h"
#include "imu3dmgx510.h"


// pitch, roll to Force ratio of motors
void pr2torques(double pitch, double roll, std::vector<double> & torques)
{
    double li = 0.01;

    //degrees to rad (motor units)
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

    std::chrono::system_clock::time_point start, current;

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

//    // file results
//    FILE *file;
//    file = fopen("../data.csv","w+");
//    fprintf(file, "time, target_pitch, sensor_pitch, target_roll, sensor_roll\n");

    //sensor config
    double freq=50; //sensor use values: 50,100,500...
    double dts=1/freq; // 0.020s
    //IMU3DMGX510 imu("/dev/ttyUSB0",freq);

//    double pitch,roll, yaw;

//    for (double t=0;t<10;t+=dts)
//    {
//       imu.GetPitchRollYaw(pitch,roll,yaw);
//    }
//    cout<<"Calibrated"<<endl;

    //Sampling time
    SamplingTime Ts;
    Ts.SetSamplingTime(dts);

    //Time per step
    double stepTime = 2; // 2 secons

    // Input /output
    std::vector<double> pose[2];
    std::vector<double> f(3);

    // Step inputs (pitch, roll)

    pose[0] = { 20, 0};
    pose[1] = { -30, 0};

    start = std::chrono::system_clock::now();

    for(int p=0; p<1; p++)
    {
        for(int i=0; i<1; i++)
        {
            printf("moving to pose [%d]: [%f] [%f]\n", i, pose[i][0], pose[i][1]);

            pr2torques(pose[i][0], pose[i][1], f);

            printf("motors torque: %f %f %f\n", f[0], f[1], f[2]);

            m1.SetTorque(f[0]);
            m2.SetTorque(f[1]);
            m3.SetTorque(f[2]);

            for(double t=0; t<=stepTime; t=t+dts)
            {
                //imu.GetPitchRollYaw(pitch,roll,yaw);

                // rad to deg
                //pitch = pitch*180/M_PI;
                //roll  = roll*180/M_PI;

                //printf("> IMU : (%f %f)\n", pitch, roll);

                current = std::chrono::system_clock::now();
                std::chrono::duration<float,std::milli> duration = current - start;

//                fprintf(file,"%.4f,", duration.count());
//                fprintf(file,"%.4f, %.4f,",  pose[i][0], -pitch); // pitch target, pitch sensor
//                fprintf(file,"%.4f, %.4f\n", pose[i][1], -roll); // roll target,  roll sensor

            Ts.WaitSamplingTime(); // 0.020seg

            }
            printf("stopped\n");

        }

    }

    printf("moving to home\n");

    pr2torques(0, 0, f);

    printf("motors torque: %f %f %f\n", f[0], f[1], f[2]);

    m1.SetTorque(f[0]);
    m2.SetTorque(f[1]);
    m3.SetTorque(f[2]);


    return 1;
}
