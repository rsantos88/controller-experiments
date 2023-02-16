#include "Cia402device.h"
#include "SocketCanPort.h"
#include <iostream>
#include <chrono>
#include "fcontrol.h"

/* Test para probar incrementos del torque en un motor para estudiar su comportamiento */

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

    // file results
    FILE *file;
    file = fopen("../data.csv","w+");
    fprintf(file, "torque, Velocity\n");

    // Sampling DTS
    double freq=100; //sensor use values: 50,100,500...
    double dts=1/freq; // 0.020s
    SamplingTime Ts;
    Ts.SetSamplingTime(dts);

    // timer
    std::chrono::system_clock::time_point start, current;

    for(double tor = 0.08; tor<1; tor+=0.01){

        for(double t=0.0; t<=0.06; t+=dts){
            if (t== 0.020) {
                m1.SetTorque(0.6);
                printf("++ Torque applied: %f\n", 0.8);
            }
            else {
                m1.SetTorque(tor);
                printf("Torque applied: %f\n", tor);
            }
            Ts.WaitSamplingTime(); // 0.020seg
        }
    }

    m1.SetTorque(0.2);


    return 1;
}
