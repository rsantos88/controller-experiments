#include <iostream>
#include <fstream>
#include <thread>
#include <mutex>

#include "Cia402device.h"
#include "CiA301CommPort.h"
#include "SocketCanPort.h"
#include "imu3dmgx510.h"
#include "fcontrol.h"


#define LG0 0.003 //meters
#define RADIO 0.0075 //meters

/* Programa para controlar el cuello en modo velocidad haciendo uso de multithrearing.
 * Nota: esta solución se descartó para evitar fallos de sincronismo. Se utilizó llamada secuencial
 * a cada uno de los controladores de los motores en modo velocidad */

class Motor {
    private:
           SocketCanPort * pm;
           CiA402SetupData * sd;
           CiA402Device * dev;
           PIDBlock * controller;
           SamplingTime * ts;

           double targetVel= 0.0;
           std::thread thread;
           std::mutex mtx;
           void motorControllerThread();
           bool stopFlag;

    public:
        Motor(int id); // Constructor
        void configMotorController(double kp, double ki, double kd, double dts);
        void startMotorControllerThread(){thread = std::thread(&Motor::motorControllerThread, this); stopFlag=false; }
        void setMotorVelocity(double vel){this->targetVel = vel;} // modify target velocities in realtime in motorControllerThread loop
        void stopMotorController() { stopFlag = true; thread.join(); dev->SetVelocity(0); dev->ForceSwitchOff(); }

};

Motor::Motor(int id){
    pm = new SocketCanPort ("can0"); // can0
    sd = new CiA402SetupData(2048, 24, 0.001, 0.144, 10); // neck motor configuration
    dev = new CiA402Device(id, pm, sd);
    dev->StartNode();
    dev->SwitchOn();
    dev->Setup_Velocity_Mode(10); // motor acceleration
}

void Motor::configMotorController(double kp, double ki, double kd, double dts){
    controller = new PIDBlock(kp,ki,kd,dts);
    ts = new SamplingTime();
    ts->SetSamplingTime(dts);
}


void Motor::motorControllerThread(){

    while(!stopFlag){

        mtx.lock();
        double currentVel = dev->GetVelocity();
        double velError = targetVel - currentVel;

        //printf("*** current Vel: %f targetVel: %f\n",currentVel,targetVel);

        // Control process
        double cS = controller->OutputUpdate(velError);

        if (!std::isnormal(cS))
        {
            cS = 0.0;
        }
        dev->SetVelocity(cS);
        mtx.unlock();

        ts->WaitSamplingTime();

    }
}

//-------------------------------------------------------------------------

// pitch, roll to velocity in meters/sec
void pr2tendons(double pitch, double roll, std::vector<double> & vel)
{
    double v1 =  0.001 * (pitch/1.5);
    double v2 =  0.001 * ( (roll/1.732) -(pitch/3)  );
    double v3 = -0.001 * ( (pitch/3) + (roll/1.732) );

    // meters/sec -> rad/s
    vel[0] = (LG0-v1)/RADIO;
    vel[1] = (LG0-v2)/RADIO;
    vel[2] = (LG0-v3)/RADIO;

    //printf("Final velocities: %f %f %f\n", vel[0], vel[1], vel[2]);
}

int main ()
{
    std::chrono::system_clock::time_point start, end;

    // SENSOR
    double freq=50; //sensor use values: 50,100,500...
    double dts=1/freq; // 0.020s

    //Sampling time
    SamplingTime Ts;
    Ts.SetSamplingTime(dts);

    cout<<"Calibrating IMU..."<<endl;

    IMU3DMGX510 imu("/dev/ttyUSB0",freq);
    double pitch,roll, yaw;

    for (double t=0;t<10;t+=dts)
    {
       imu.GetPitchRollYaw(pitch,roll,yaw);
    }

    cout<<"Calibrated!"<<endl;

    cout<<"Configuring soft neck motors..."<<endl;

    //m1 setup
    Motor m1(1);
    m1.configMotorController(0, 10, 0, 0.01); // ts = 0.01s = 50Hz
    m1.startMotorControllerThread();
    sleep(1);

    //m2
    Motor m2(2);
    m2.configMotorController(0, 10, 0, 0.01);
    m2.startMotorControllerThread();
    sleep(1);

    //m3
    Motor m3(3);
    m3.configMotorController(0, 10, 0, 0.01);
    m3.startMotorControllerThread();
    sleep(1);


    //Time per step
    double steptime  = 1;
    double sleeptime = 2;

    // file results    
    FILE *file;
    file = fopen("../data.csv","w+");
    fprintf(file, "time, target_pitch, sensor_pitch, target_roll, sensor_roll\n");



    // Input /output
    std::vector<double> pose[2];
    std::vector<double> v(3);

    //First position
    //printf("Running first position...\n");
    //pr2tendons(-10,0,v);

    //m1.setMotorVelocity(v[0]);
    //m2.setMotorVelocity(v[1]);
    //m3.setMotorVelocity(v[2]);

    //sleep(steptime);

    //m1.setMotorVelocity(0);
    //m2.setMotorVelocity(0);
    //m3.setMotorVelocity(0);

    //printf("Stopped\n");
    //sleep(sleeptime);


    // Step inputs (pitch, roll)

    pose[0] = { -20, 0};
    pose[1] = { 20, 0};

    for(int p=0; p<5; p++)
    {
        for(int i=0; i<2; i++)
        {
            printf("---- moving to pose [%d]: [%f] [%f]\n", i, pose[i][0], pose[i][1]);

            pr2tendons(pose[i][0], pose[i][1], v);
            printf("tendons: [%f] [%f] [%f]\n", v[0], v[1], v[2]);

            m1.setMotorVelocity(v[0]);
            m2.setMotorVelocity(v[1]);
            m3.setMotorVelocity(v[2]);

            start = std::chrono::system_clock::now();
            //std::cout << "init pos: " << m1.GetPosition() << std::endl;

            for(double time=0; time<= steptime; time=time+dts) // totalIterations = steptime/ 0.020s(dts)
            {
                //printf("waiting... (%f)\n", time);
                imu.GetPitchRollYaw(pitch,roll,yaw);

                // rad to deg
                pitch = pitch*180/M_PI;
                roll  = roll*180/M_PI;

                printf("> IMU : (%f %f)\n", pitch, roll);

                fprintf(file,"%.2f, ",time);
                fprintf(file,"%.4f, %.4f,",  pose[i][0], pitch); // pitch target, pitch sensor
                fprintf(file,"%.4f, %.4f\n", pose[i][1], roll); // roll target,  roll sensor

            Ts.WaitSamplingTime(); // 0.020seg

            }

            //end = std::chrono::system_clock::now();
            //auto int_s = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
            //std::cout << "** pos: " << m1.GetPosition() << std::endl;
            //std::cout << "Time: "<< int_s.count() <<" ms "<< std::endl;

         m1.setMotorVelocity(0);
         m2.setMotorVelocity(0);
         m3.setMotorVelocity(0);
         printf("stopped\n");
         sleep(sleeptime);

        }
    }
    // stop control
    m1.stopMotorController();
    m2.stopMotorController();
    m2.stopMotorController();

    return 0;
}

