#include "Cia402device.h"
#include "SocketCanPort.h"
#include <iostream>
#include <chrono>
#include "fcontrol.h"

// Test para probar un motor en modo posición

int main ()
{
    //m1 setup
    SocketCanPort pm1("can0");
    CiA402SetupData sd1(2048,24,0.001, 0.144, 10);
    CiA402Device m1 (1, &pm1, &sd1);
    m1.Reset();
    m1.StartNode();
    m1.SwitchOn();
    m1.SetupPositionMode(10, 10);
    
    double posdeg = 360;
    double posrad= posdeg *M_PI/180; // 1 ° × π / 180 = 0,01745 rad
    printf("deg: (%f) rad: (%f)", posdeg, posrad);

    sleep(1);
    m1.SetPosition(posrad);
    while(1)
    {
        printf("current poss: %f\n", m1.GetPosition()* 180/M_PI);
    }

//    m1.SetPosition(0);
    
//    m1.ForceSwitchOff();
//    sleep(1);
//    m1.PrintStatus();


    return 1;
}
