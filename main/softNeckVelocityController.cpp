#include "Cia402device.h"
#include "CiA301CommPort.h"
#include "SocketCanPort.h"
#include <iostream>
#include <stdio.h>
#include "fcontrol.h"
#include "IPlot.h"
#include <chrono>

#include "imu3dmgx510.h"

#include <yarp/os/Network.h>
#include <yarp/os/Property.h>
#include <yarp/os/Time.h>

#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/IEncoders.h>

/* Aplicación que controla el cuello en velocidad haciendo uso del controlador en velocidad
 * del driver en cada motor */
int main ()
{
    // inputs
    vector<double> targetPose{0, 0}; // {roll, pitch}
    double targetTime = 120; //seconds

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
    IMU3DMGX510 imu("/dev/ttyUSB0",freq);
    double pitch,roll, yaw;

    //Sampling time
    double dts=1/freq;
    SamplingTime Ts;
    Ts.SetSamplingTime(dts);

    // Controller
    std::vector<double> ierror(2); // ERROR
    std::vector<double> cs(2); //CONTROL SIGNAL

    /*
    fPDz =
    3.004e04 z^4 - 5.633e04 z^3 + 2.902e04 z^2 - 2727 z + 0.0248
    ------------------------------------------------------------
    z^4 - 0.8756 z^3 + 0.09084 z^2 - 8.281e-07 z - 6.102e-25

    Wc = 10

    Pm = 100

    fPDz =
    179.7 z^4 - 569.8 z^3 + 654.5 z^2 - 317.7 z + 53.33
    ---------------------------------------------------
    z^4 - 2.187 z^3 + 1.493 z^2 - 0.3037 z + 1.126e-12

    wc=1, pm= 100

    */

    /*
    FPDBlock controllerFraccRollVelocity(14.2553, 1.3877,  0.9400, dts);
    FPDBlock controllerFraccPitchVelocity(14.2553, 1.3877,  0.9400, dts);
    */

    std::vector<double> num = { 40.94,  595.9 , 740.8, 108.8, 1.699};
    std::vector<double> den = {92.1, 1040, 977.4, 102.9, 1};

    SystemBlock controllerFraccRollVelocity(num, den);
    SystemBlock controllerFraccPitchVelocity(num, den);

    // Reading YARP trunk (OPTIONAL)
    yarp::os::Network yarp;

    if (!yarp::os::Network::checkNetwork())
    {
        std::printf("Please start a yarp name server first\n");
        return 1;
    }

    /*
    yarp::os::Property options;
    options.put("device", "remote_controlboard");
    options.put("remote", "/teo/trunk");
    options.put("local", "/local");

    yarp::dev::PolyDriver dd(options);

    if (!dd.isValid())
    {
        std::printf("Device not available.\n");
        return 1;
    }

     yarp::dev::IEncoders *enc;

     if(!dd.view(enc))
     {
         std::printf("ERROR: Problems acquiring IEncoders robot interface\n");
         return 1;
     }

     std::printf("SUCCESS: Acquired robot interface\n");

     int jnts;
     double frontal, axial;

     enc->getAxes(&jnts);
     enc->getEncoder(0, &frontal);
     enc->getEncoder(1, &axial);

     std::printf("Receiving values of trunk: %d axes with values (%f %f)", jnts, frontal, axial);
     */

    //Once the device is correctly connected, it's set to IDLE mode to stop transmitting data till user requests it
    imu.set_streamon();

    for (double t=0;t<10;t+=dts)
    {
       imu.GetPitchRollYaw(pitch,roll,yaw);
    }
    cout<<"Calibrated"<<endl;

    std::vector<double> xd(2);

    // time
    auto timestamp = yarp::os::Time::now();

    // file results
    ofstream testingFile;
    testingFile.open("data.txt", ofstream::out);
    testingFile << "Time, Vel-M1, Vel-M2, Vel-M3, Roll, Pitch" << endl;

    while(1)
    {

        imu.GetPitchRollYaw(pitch,roll,yaw);

        double rollError  = targetPose[0] - roll*180/M_PI;
        double pitchError = targetPose[1] - pitch*180/M_PI;

        //Control process
        double rollCs = controllerFraccRollVelocity.OutputUpdate(rollError);
        if (!std::isnormal(rollCs))
        {
            rollCs = 0.0;
        }
        xd[0] = rollCs;

        double pitchCs = controllerFraccPitchVelocity.OutputUpdate(pitchError);
        if (!std::isnormal(pitchCs))
        {
            pitchCs = 0.0;
        }
        xd[1] = pitchCs;

        /* Conversions:
         * x[0]      -> roll
         * x[1]      -> pitch
         * v1,v2,v3  -> tendon velocities in meters/seg */

        double v1 = - (xd[1]/1.5);
        double v2 =   ( (xd[1]/3) - (xd[0]/1.732) );
        double v3 =   ( (xd[1]/3) + (xd[0]/1.732) );

        cout << "-----------------------------\n" << endl;
        cout << "-> Time:" << yarp::os::Time::now() - timestamp << endl;
        cout << "-> Roll: " << roll*180/M_PI << "  Pitch: " << pitch*180/M_PI << endl;
        cout << "-> RollTarget: " << targetPose[0] << " PitchTarget:  " << targetPose[1] << " >>>>> RollError: " << rollError << "  PitchError: " << pitchError << endl;
        cout << "-> Motor velocity >>>>> V1: " << v1 << " V2: " << v2 << " V3: " << v3 << endl;
        cout << "-----------------------------\n" << endl;

        m1.SetVelocity(v1);
        m2.SetVelocity(v2);
        m3.SetVelocity(v3);

        //Uncomment it to receive data from testing 
        //testingFile << yarp::os::Time::now() - timestamp << "," << v1 << "," << v2 << "," << v3 << "," << roll << "," << pitch << endl;

        Ts.WaitSamplingTime();
    }

    testingFile.close();

    return 0;

}

