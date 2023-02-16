#include "Cia402device.h"
#include "SocketCanPort.h"
#include "imu3dmgx510.h"
#include <iostream>
#include <chrono>
#include <thread>

#include "imu3dmgx510.h"
#include "fcontrol.h"

// compare velocity or position
bool compare(float x, float y, float epsilon = 0.0001f){
   if(fabs(x - y) < epsilon)
      return true; //they are same
      return false; //they are not same
}

// pitch, roll to lenght of tendons in meters
void pr2tendons(double pitch, double roll, std::vector<double> & lengths)
{
    //degrees to rad (motor units)
    pitch = pitch*M_PI/180;
    roll = roll*M_PI/180;

    // pitch & roll -> lenghts
    lengths[0] =  (pitch/1.5);
    lengths[1] =  (roll/1.732 - pitch/3);
    lengths[2] = -(pitch/3 + roll/1.732);
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

    // SENSOR
    double freq=50; //sensor use values: 50,100,500...
    double dts=1/freq; // 0.020s
    IMU3DMGX510 imu("/dev/ttyUSB0",freq);

    double pitch,roll, yaw;

    for (double t=0;t<10;t+=dts)
    {
       imu.GetPitchRollYaw(pitch,roll,yaw);
    }
    cout<<"Calibrated"<<endl;

    //Sampling time
    SamplingTime Ts;
    Ts.SetSamplingTime(dts);

    // time per step
    double sleeptime = 1;

    // file results
    FILE *file;
    file = fopen("../data.csv","w+");
    fprintf(file, "time, target_pitch, sensor_pitch, target_roll, sensor_roll\n");

    // Input /output
    std::vector<double> pose[2];
    std::vector<double> lengths(3);
    std::vector<double> posan(3);


    // Step inputs (pitch, roll)

    pose[0] = { -10, 0};
    pose[1] = {  10, 0};

    start = std::chrono::system_clock::now();

    for(int p=0; p<1; p++)
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

            //std::this_thread::sleep_for(std::chrono::milliseconds(10));


            for(int l=0; l<150; l++)
            {
                //printf("* current angular pos: %f %f %f\n", m1.GetPosition(), m2.GetPosition(), m3.GetPosition());
                imu.GetPitchRollYaw(pitch,roll,yaw);

                // rad to deg
                pitch = pitch*180/M_PI;
                roll  = roll*180/M_PI;

                printf("> IMU : (%f %f)\n", pitch, roll);

                current = std::chrono::system_clock::now();
                std::chrono::duration<float,std::milli> duration = current - start;

                fprintf(file,"%.4f,", duration.count());
                fprintf(file,"%.4f, %.4f,",  pose[i][0], pitch); // pitch target, pitch sensor
                fprintf(file,"%.4f, %.4f\n", pose[i][1], roll); // roll target,  roll sensor

            Ts.WaitSamplingTime(); // 0.020seg

            }
            printf("stopped\n");
        }
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

