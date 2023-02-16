#include "Cia402device.h"
#include "SocketCanPort.h"
#include "imu3dmgx510.h"
#include "fcontrol.h"
#include <iostream>
#include <chrono>
#include <thread>
#include <IPlot.h>


#define LG0 0.003 //meters
#define TRADIO 0.0075 // radio del tambor
#define PRADIO 0.05 // radio de la plataforma

//-------------------------------------------------------------------------

// pitch, roll to velocity in meters/sec
void pr2tendons(double pitch, double roll, std::vector<double> & vel)
{
    double T  = PRADIO / TRADIO;
    vel[0] =  pitch * T;
    vel[1] =  roll * T * sin(2*M_PI/3) + pitch * T * cos(2*M_PI/3);
    vel[2] =  roll * T * sin(4*M_PI/3) + pitch * T * cos(4*M_PI/3);

    //printf("Final angular velocities: %f %f %f\n", vel[0], vel[1], vel[2]);
}

// tense tendons:
//  true: it's stopped and tensed
bool tenseTendons(CiA402Device motor){
    motor.SwitchOn();
    motor.Setup_Torque_Mode();
    motor.SetTorque(0.07); //0.07
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    while(motor.GetVelocity() > 0.2) //0.2
        printf("Tensing thread... vel: %f\n", motor.GetVelocity());
    motor.Setup_Velocity_Mode(0);
    return true;
}

int main ()
{

    // inputs
    vector<double> targetPose{0, 0}; // {pitch, roll: 0.18}

    std::vector<double> targetVel(3);
    double tc = 0; //tensedCounter
    bool tensed = false;


    // control loop sampling time
    double freq=50; //sensor use values: 50,100,500...
    double dts=1/freq;
    SamplingTime Ts;
    Ts.SetSamplingTime(dts); // 0.020

    // Grafico
    IPlot graph(dts, "Test", "Tiempo", "Pitch");

    // configuring IMU
    IMU3DMGX510 imu("/dev/ttyUSB0",freq);
    double pitch,roll, yaw;

    cout<<"Calibrating IMU..."<<endl;

    for (double t=0;t<10;t+=dts)
    {
       imu.GetPitchRollYaw(pitch,roll,yaw);
    }

    cout<<"Calibrated!"<<endl;

    // file results
    FILE *file;
    file = fopen("../data.csv","w+");
    fprintf(file,"Time, TargetPitch, SensorPitch, TargetRoll, SensorRoll, VelM1, VelM2, VelM3\n");

    //m1 setup
    SocketCanPort pm1("can0");
    CiA402SetupData sd1(2048,24,0.001, 0.144, 10);
    CiA402Device m1 (1, &pm1, &sd1);
    m1.StartNode();
    m1.SwitchOn();
    m1.Setup_Velocity_Mode(10);

    //m2 setup
    SocketCanPort pm2("can0");
    CiA402SetupData sd2(2048,24,0.001, 0.144, 10);
    CiA402Device m2 (2, &pm2, &sd2);
    m2.StartNode();
    m2.SwitchOn();
    m2.Setup_Velocity_Mode(10);

    //m3 setup
    SocketCanPort pm3("can0");
    CiA402SetupData sd3(2048,24,0.001, 0.144, 10);
    CiA402Device m3 (3, &pm3, &sd3);
    m3.StartNode();
    m3.SwitchOn();
    m3.Setup_Velocity_Mode(10);

    //Init: tensing tendons
    tensed &= tenseTendons(m1);
    tensed &= tenseTendons(m2);
    tensed &= tenseTendons(m3);

    if( tensed )
        printf("tendons tensed successfully\n");

    // controller for motor
    PIDBlock cntrl1(0.015,36,0,dts);
    PIDBlock cntrl2(0.015,36,0,dts);
    PIDBlock cntrl3(0.015,36,0,dts);

    // test 1 with Kp = 0.363, Ki = 0.0268, Kd = 0.163
    //PIDBlock pController(0.363, 0.0268, 0.163, dts);
    //PIDBlock rController(0.363, 0.0268, 0.163, dts);

    // test 2 Kp = 8.8727 , Ki = 0.0544  , Kd = 4.7066.
    //PIDBlock pController(8.8727, 0.0544, 4.7066, dts);
    //PIDBlock rController(8.8727, 0.0544, 4.7066, dts);

    // test 3 Fractional Controller Configuration : alpha =0.9000 kp = 9.9540 ka = 5.7674
    // FPDBlock(double new_kp = 1, double new_kd = 1 (ka), double new_fex = 1 (alpha), double new_dts = 0.001);
    //FPDBlock controllerFraccPitchVelocity(9.9540, 5.7674,  0.9000, dts);
    //FPDBlock controllerFraccRollVelocity(9.9540, 5.7674,  0.9000, dts);

    //-----------------------------------------------------------------

    // test 4: alpha =-0.4900 kp =0.4865 ka =0.5949
    //FPDBlock fcPitchVelocity(0.4865, 0.5949, -0.4900, dts);
    //FPDBlock fcRollVelocity(0.4865, 0.5949, -0.4900, dts);

    // test 5: PID Kp = 1.01, Ki = 0.188, Kd = 0.171
    //PIDBlock pController(1.01, 0.188, 0.171, dts);
    //PIDBlock rController(1.01, 0.188, 0.171, dts);

    // test 6: alpha = -0.7400 kp = 1.6470 ka =1.1881 (2 rad/s)
    //FPDBlock fcPitchVelocity(1.6470, 1.1881, -0.7400, dts);
    //FPDBlock fcRollVelocity(1.6470, 1.1881, -0.7400, dts);

    // test 7: alpha = -1.1 kp = 4.3 ka =6.235 (hay sobreimpulso)
    //FPDBlock fcPitchVelocity(4.3, 6.235, -1.1, dts);
    //FPDBlock fcRollVelocity(4.3, 6.235, -1.1, dts);

    // test 8: alpha = -1.92 kp = 2.75 ka =99.7
    //FPDBlock fcPitchVelocity(2.75, 99.7, -1.92, dts);
    //FPDBlock fcRollVelocity(2.75, 99.7, -1.92, dts);

    // -- Final tests
    //test 9: FPD w=3, pm=50
    //        Pitch: alpha =-0.8500, kp = 2.5773, ka = 3.2325
    //        Roll:  alpha = -0.8600, kp = 2.6299, ka = 3.2395
    FPDBlock fcPitchVelocity(2.5773, 3.2325, -0.8500, dts);
    FPDBlock fcRollVelocity(2.6299, 3.2395, -0.8600, dts);

    // test 10: PID w=3, pm=50
    //          Pitch: Kp = 3.15 , Ki = 1.73  , Kd = 0.193
    //          Roll:  Kp = 3.17, Ki = 1.74, Kd = 0.198
    //PIDBlock pController(3.15, 1.73, 0.193, dts);
    //PIDBlock rController(3.17, 1.74, 0.198, dts);

    //std::vector<double> num = { 40.94,  595.9 , 740.8, 108.8, 1.699};
    //std::vector<double> den = {92.1, 1040, 977.4, 102.9, 1};

    //SystemBlock controllerFraccRollVelocity(num, den);
    //SystemBlock controllerFraccPitchVelocity(num, den);

    auto start = std::chrono::system_clock::now();
    std::chrono::duration<float,std::milli> duration;

    while(1){

        // ---- General Control process ----
        imu.GetPitchRollYaw(pitch,roll,yaw);

//        if( (pitch>-0.1 && pitch<0.1) && (roll>-0.1 && roll<0.1) ){
//            printf("Paso por el centro!! (%d)\n", tc);
//                tenseTendons(m1);
//                tenseTendons(m2);
//                tenseTendons(m3);
//        }


        // cambio signo para igualar sentido de giro de los motores y del sensor
        pitch = -pitch;
        roll  = -roll;

        double pitchError = targetPose[0] - pitch;
        double rollError  = targetPose[1] - roll;

        double pitchCs = fcPitchVelocity.OutputUpdate(pitchError);
        if (!std::isnormal(pitchCs))
        {
            pitchCs = 0.0;
        }

        double rollCs = fcRollVelocity.OutputUpdate(rollError);
        if (!std::isnormal(rollCs))
        {
            rollCs = 0.0;
        }

        pr2tendons(pitchCs, rollCs, targetVel);

        // -----------------------------------

        // Controller of velocity in M1

        double currentVelM1 = m1.GetVelocityTP();
        double velError1 = targetVel[0] - currentVelM1;

        // Control process
        double cS1 = cntrl1.OutputUpdate(velError1);

        if (!std::isnormal(cS1))
        {
            cS1 = 0.0;
        }

        m1.SetVelocity(cS1);

        // Controller of velocity in M2

        double currentVelM2 = m2.GetVelocityTP();
        double velError2 = targetVel[1] - currentVelM2;

        // Control process
        double cS2 = cntrl2.OutputUpdate(velError2);

        if (!std::isnormal(cS2))
        {
            cS2 = 0.0;
        }

        m2.SetVelocity(cS2);

        // Controller of velocity in M3

        double currentVelM3 = m3.GetVelocityTP();
        double velError3 = targetVel[2] - currentVelM3;

        // Control process
        double cS3 = cntrl3.OutputUpdate(velError3);

        if (!std::isnormal(cS3))
        {
            cS3 = 0.0;
        }

        m3.SetVelocity(cS3);

        auto current = std::chrono::system_clock::now();
        duration = current - start;

        // Escribimos en el archivo
        //fprintf(file,"%.4f,", duration.count());
        //fprintf(file,"%.4f, %.4f,",  targetPose[0], pitch); // pitch target, pitch sensor
        //fprintf(file,"%.4f, %.4f,",  targetPose[1], roll); // roll target,  roll sensor
        //fprintf(file,"%.4f, %.4f, %.4f\n", m1.GetVelocity(), m2.GetVelocity(), m3.GetVelocity()); // roll target,  roll sensor

        //graph.pushBack(pitch);

        printf("TC: %f pitch error: %f  roll error: %f \n", tc, pitchError, rollError);

        // Check error
        if(pitchError < 0.01 && rollError < 0.01 && tc>5) //cada 60 seg
        {
            printf(" CALIBRATING TENDONS.../n");
            tensed = false;
            sleep(1);

            if(tenseTendons(m1))
                printf("-> tensed tendon 1\n");
            if(tenseTendons(m2))
                printf("-> tensed tendon 2\n");
            if(tenseTendons(m3))
                printf("-> tensed tendon 3\n");

            tc = 0; // reset counter
        }

        tc= tc + dts;
        Ts.WaitSamplingTime();        

    }

    m1.SetVelocity(0);
    m2.SetVelocity(0);
    m3.SetVelocity(0);

    m1.ForceSwitchOff();
    m2.ForceSwitchOff();
    m3.ForceSwitchOff();

    graph.Plot();

    return 1;
}
