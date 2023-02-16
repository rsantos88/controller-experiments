#include "Cia402device.h"
#include "SocketCanPort.h"
#include "imu3dmgx510.h"
#include <iostream>
#include "fcontrol.h"
#include <chrono>
#include "imu3dmgx510.h"

// pitch, roll to lenght of tendons in meters
void pr2tendons(double pitch, double roll, std::vector<double> & vel)
{
    vel[0] = - (pitch/1.5);
    vel[1] =  ( pitch/3 - roll/1.732);
    vel[2] =  ( pitch/3 + roll/1.732);

//    for(int v=0; v<3; v++){
//        if(vel[v]<0)
//            vel[v] = vel[v] * 0,982340486; // multiplico por el factor de diferencia cuando la velocidad es negativa
//    }
}

int main ()
{

    std::chrono::system_clock::time_point start, end;

    //m1 setup
    SocketCanPort pm1("can0");
    CiA402SetupData sd1(2048,24,0.001, 0.144, 10);
    CiA402Device m1 (1, &pm1, &sd1);
    m1.StartNode();
    m1.SwitchOn();
    m1.Setup_Velocity_Mode(10);

    //m2
    SocketCanPort pm2("can0");
    CiA402SetupData sd2(2048,24,0.001, 0.144, 10);
    CiA402Device m2 (2, &pm2, &sd2);
    m2.StartNode();
    m2.SwitchOn();
    m2.Setup_Velocity_Mode(10);

    //m3
    SocketCanPort pm3("can0");
    CiA402SetupData sd3(2048,24,0.001, 0.144, 10);
    CiA402Device m3 (3, &pm3, &sd3);
    m3.StartNode();
    m3.SwitchOn();
    m3.Setup_Velocity_Mode(10);

    // SENSOR
    double freq=50; //sensor use values: 50,100,500...
    double dts=1/freq; // 0.020s

//    IMU3DMGX510 imu("/dev/ttyUSB0",freq);
//    double pitch,roll, yaw;

//    for (double t=0;t<10;t+=dts)
//    {
//       imu.GetPitchRollYaw(pitch,roll,yaw);
//    }

//    cout<<"Calibrated"<<endl;


    //Sampling time
    SamplingTime Ts;
    Ts.SetSamplingTime(dts);

    // time per step
    double steptime  = 1;
    double sleeptime = 3;

    // file results
    FILE *file;
    file = fopen("../data.csv","w+");
    fprintf(file, "time, target_pitch, sensor_pitch, target_roll, sensor_roll\n");

    // Input /output
    std::vector<double> pose[2];
    std::vector<double> v(3);

    //First position
    printf("Running first position...\n");
    pr2tendons(5,0,v);

    m1.SetVelocity(v[0]);
    m2.SetVelocity(v[1]);
    m3.SetVelocity(v[2]);

    sleep(steptime);

    m1.SetVelocity(0);
    m2.SetVelocity(0);
    m3.SetVelocity(0);

    printf("Stopped\n");
    sleep(sleeptime);


    // Step inputs (pitch, roll)

    pose[0] = {  6, 0};
    pose[1] = { -6, 0};

    for(int p=0; p<10; p++)
    {
        for(int i=0; i<2; i++)
        {
            printf("moving to pose [%d]: [%f] [%f]\n", i, pose[i][0], pose[i][1]);

            pr2tendons(pose[i][0], pose[i][1], v);
            printf("tendons: [%f] [%f] [%f]\n", v[0], v[1], v[2]);

            m1.SetVelocity(v[0]);
            m2.SetVelocity(v[1]);
            m3.SetVelocity(v[2]);

            start = std::chrono::system_clock::now();
            //std::cout << "init pos: " << m1.GetPosition() << std::endl;

            for(int iter=0; iter< steptime/dts; iter++) // totalIterations = steptime/ 0.020s(dts)
            {
                //imu.GetPitchRollYaw(pitch,roll,yaw);

                // rad to deg
//                pitch = pitch*180/M_PI;
//                roll  = roll*180/M_PI;

                //printf("> IMU : (%f %f)\n", pitch, roll);

                //fprintf(file,"%.2f, ",iter * 0.020);
                //fprintf(file,"%.4f, %.4f,",  pose[i][0], pitch); // pitch target, pitch sensor
                //fprintf(file,"%.4f, %.4f\n", pose[i][1], roll); // roll target,  roll sensor

            Ts.WaitSamplingTime(); // 0.020seg

            }

            end = std::chrono::system_clock::now();
            auto int_s = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
            std::cout << "** pos: " << m1.GetPosition() << std::endl;
            std::cout << "Time: "<< int_s.count() <<" ms "<< std::endl;

         m1.SetVelocity(0);
         m2.SetVelocity(0);
         m3.SetVelocity(0);
         printf("stopped\n");
         sleep(sleeptime);

        }
    }
    return 0;
}

