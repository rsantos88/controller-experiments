#include "Cia402device.h"
#include "SocketCanPort.h"
#include <iostream>
#include <algorithm>
#include <chrono>
#include "fcontrol.h"

/* Test de cambio entre 2 posiciones del cuello en modo torque y bucle abierto */

// pitch, roll to Force ratio of motors
void pr2torques(double pitch, double roll, std::vector<double> & torques)
{
    double li = 0.01;

    //degrees to rad (motor units)
    pitch = pitch*M_PI/180;
    roll = roll*M_PI/180;

    // pitch & roll -> lenghts
    torques[2] =  li - std::min({pitch + (roll/2), roll, 0.0}); // P3
    torques[1] =  roll + torques[2];
    torques[0] =  pitch + 0.5 * (torques[1] + torques[2]);
}


int main ()
{

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

    while(1)
    {

        std::vector<double> ori {0, 20};
        std::vector<double> f(3);
        pr2torques(ori[0], ori[1], f);

        printf("motors torque: %f %f %f\n", f[0], f[1], f[2]);

        m1.SetTorque(f[0]);
        m2.SetTorque(f[1]);
        m3.SetTorque(f[2]);

        sleep(5);

        std::vector<double> ori2 {0, -20};
        pr2torques(ori2[0], ori2[1], f);

        printf("motors torque: %f %f %f\n", f[0], f[1], f[2]);

        m1.SetTorque(f[0]);
        m2.SetTorque(f[1]);
        m3.SetTorque(f[2]);

        sleep(5);

    }
    return 1;
}
