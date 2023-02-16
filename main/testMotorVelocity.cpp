#include "Cia402device.h"
#include "SocketCanPort.h"
#include <iostream>
#include <chrono>
#include <thread>
#include "fcontrol.h"

/* Test de un motor en modo velocidad */

int main ()
{
    //m1 setup
    SocketCanPort pm1("can0");
    CiA402SetupData sd1(2048,24,0.001, 0.144, 10);
    CiA402Device m1 (1, &pm1, &sd1);
    m1.Reset();
    m1.StartNode();
    m1.SwitchOn();
    m1.Setup_Velocity_Mode(10);
    
    double posdeg = 2000;
    double posrad= posdeg*M_PI/180; // 1 ° × π / 180 = 0,01745 rad

    m1.SetVelocity(posrad);

    sleep(60);
    m1.ForceSwitchOff();
    sleep(1);
    m1.PrintStatus();
    return 1;
}
