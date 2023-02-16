#include "Cia402device.h"
#include "SocketCanPort.h"
#include <iostream>
#include <chrono>
#include "fcontrol.h"

/* Test para comprobar si funciona correctamente el paso de modo velocidad a modo torque
 */

int main ()
{
    //m1 setup
    SocketCanPort pm1("can0");
    CiA402SetupData sd1(2048,24,0.001, 0.144, 10.0);
    CiA402Device m1 (40, &pm1, &sd1);
    m1.Reset();
    m1.StartNode();
    m1.SwitchOn();

    printf("- Vel\n");
    m1.Setup_Velocity_Mode();
    m1.SetVelocity(10);
    sleep(5);

    printf("- Torque\n");
    m1.Setup_Torque_Mode();
    m1.SetTorque(0.08);
    sleep(5);

    printf("- Vel\n");
    m1.Setup_Velocity_Mode();
    m1.SetVelocity(10);
    sleep(5);

    printf("- Torque\n");
    //m1.Reset();
    //m1.StartNode();
    m1.SwitchOn();
    m1.Setup_Torque_Mode();
    m1.SetTorque(0.08);
    sleep(5);

    printf("- Vel\n");
    m1.Setup_Velocity_Mode();
    m1.SetVelocity(10);
    sleep(5);

    printf("- Torque\n");
    m1.Setup_Torque_Mode();
    m1.SetTorque(0.08);
    sleep(5);

    return 1;
}
