#include "Cia402device.h"
#include "SocketCanPort.h"
#include <iostream>
#include <algorithm>
#include <chrono>
#include <thread>
#include "fcontrol.h"
#include "imu3dmgx510.h"

/* Test de una onda senoidal para un motor en modo torque. */

int main ()
{

    std::chrono::system_clock::time_point start, current;

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
    fprintf(file, "time, target torque, current vel\n");

    //sensor config
    double freq=50; //sensor use values: 50,100,500...
    double dts=1/freq; // 0.020s

    //Sampling time
    SamplingTime Ts;
    Ts.SetSamplingTime(dts);

    // Input /output
    std::vector<double> pose[2];
    std::vector<double> f(3);

    // Step inputs (pitch, roll)

    //Trayectoria senoidal
    double A = 1; //Amplitud de la onda senoidal
    double y; // señal de salida senoidal
    int rep=20; //Repeticiones

    start = std::chrono::system_clock::now();

    for (double t=0; t<= rep * M_PI * 2; t+=dts)
        {
            y = A* sin(1.0*t);
            //pr2torques(0, y, f);

            printf("  motor torque: %f\n", y);

            // Move motors in torque mode
            m1.SetTorque(y);
            printf("VEL: %f\n", m1.GetVelocity());


            current = std::chrono::system_clock::now();
            std::chrono::duration<float,std::milli> duration = current - start;

            fprintf(file,"%.4f,", duration.count());
            fprintf(file,"%.4f,",  y);
            fprintf(file,"%.4f\n", m1.GetVelocity());

            Ts.WaitSamplingTime(); // 0.020seg
            //std::this_thread::sleep_for(std::chrono::milliseconds(20));

        }
    printf("stopped\n");
    return 1;
}
