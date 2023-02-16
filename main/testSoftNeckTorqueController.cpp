#include "Cia402device.h"
#include "CiA301CommPort.h"
#include "SocketCanPort.h"
#include <iostream>
#include <stdio.h>
#include "fcontrol.h"
#include "IPlot.h"
#include <chrono>

#include "imu3dmgx510.h"


int main ()
{

    //m1 setup
    SocketCanPort pm1("can0");
    CiA402SetupData sd1(2048,24,0.001, 0.144, 10);
    CiA402Device m1 (1, &pm1, &sd1);
    m1.StartNode();
    m1.SwitchOn();
    m1.Setup_Torque_Mode();


    m1.SetAmpRaw(1);


    return 0;
}

