

#include <iostream>
#include <fstream>
#include "Cia402device.h"
#include "CiA301CommPort.h"
#include "SocketCanPort.h"
#include "math.h"

#include "fcontrol.h"
#include "IPlot.h"



int main ()
{
    bool onrobot=false;
    //Controllers
    double dts=0.01;

    string location("/home/humasoft/Escritorio/");


    string method("o2isom");
    //fpi w=25 pm=70 //kept from last experiments.
    vector<double> npi ={2.5712 , -21.9948  , -7.5175 ,  29.4688};
    vector<double> dpi ={-0.2557 ,   0.0614 ,   1.3171  ,  1.0000};


    SystemBlock pi1(npi,dpi,10);
    SystemBlock pi3(npi,dpi,10);
    SystemBlock pi2(npi,dpi,10);
//    pi2.SetSaturation(-700,700);
    //string method("w12p60pid");
//    PIDBlock pi1(8.6120054,10.826259,0.2030172,dts);
//    PIDBlock pi2(0.165,21.15,0,dts);
//    PIDBlock pi3(8.6120054,10.826259,0.2030172,dts);


    ofstream targets (location+method+".targets.csv");
    ofstream responses (location+method+".responses.csv");
    ofstream controls (location+method+".controls.csv");



        SocketCanPort pm1("can1");
        CiA402Device m1 (1, &pm1);
        SocketCanPort pm2("can1");
        CiA402Device m2 (2, &pm2);
        SocketCanPort pm3("can1");
        CiA402Device m3 (3, &pm3);

        if (onrobot)
        {
//  Remember to switch on before to keep this commented.

//        m1.Reset();
//        m2.Reset();
//        m3.Reset();

//        m1.SwitchOn();
//        sleep(1);
//        m2.SwitchOn();
//        sleep(1);
//        m3.SwitchOn();
//        sleep(1);

        m1.Setup_Torque_Mode();
        m2.Setup_Torque_Mode();
        m3.Setup_Torque_Mode();
    }

//    TableKinematics a;
//    vector<double> lengths(3);
//    long orient=1;
//    long incli=1;

//    a.GetIK(incli,orient,lengths);
//    cout << "l1 " << lengths[0]  << ", l2 " << lengths[1] << ", l3 " << lengths[2]<<endl;
//    double posan1, posan2, posan3;
//    posan1=(0.1-lengths[0])*180/(0.01*M_PI);
//    posan2=(0.1-lengths[1])*180/(0.01*M_PI);
//    posan3=(0.1-lengths[2])*180/(0.01*M_PI);
double posan1, posan2, posan3;
    posan1=80;
    posan2=-posan1/2;
    posan3=-posan1/2;
    cout << "pos1 " << posan1  << ", pos2 " << posan2 << ", pos3 " << posan3 <<endl;

    IPlot pl1(dts);

    double ep1,cs1;
    double ep2,cs2;
    double ep3,cs3;

    double interval=2; //in seconds

//    double sats=40;
//    pd1.SetSaturation(-sats,sats);
//    pd2.SetSaturation(-sats,sats);
//    pd3.SetSaturation(-sats,sats);



    for (double t=0;t<interval; t+=dts)
    {
        if (onrobot)
        {
            ep1=posan1-m1.GetPosition();
            m1.SetTorque((ep1 > pi1));

            ep2=posan2-m2.GetPosition();
            m2.SetTorque(ep2 > pi2);

            ep3=posan3-m3.GetPosition();
            m3.SetTorque(ep3 > pi3);

            pl1.pushBack(m1.GetPosition());

          //  cout << t << " , " << m1.GetPosition() << " , " << m2.GetPosition() <<  " , " << m3.GetPosition() <<endl;
//            controls << t << " , " << cs1 << " , " << cs2 <<  " , " << cs3 <<endl;
            responses << t << " , " << m1.GetPosition() << " , " << m2.GetPosition() <<  " , " << m3.GetPosition() <<endl;


//            cout << t << " , " << m1.GetVelocity() << " , " << m2.GetVelocity() <<  " , " << m3.GetVelocity() <<endl;
//            responses << t << " , " << m1.GetVelocity() << " , " << m2.GetVelocity() <<  " , " << m3.GetVelocity() <<endl;

        }
        usleep(uint(dts*1000*1000));
       // cout << t << " , " << posan1  << " , " << posan2 << " , " << posan3 << endl;
        targets << t << " , " << posan1  << " , " << posan2 << " , " << posan3 << endl;

    }


    if (onrobot)
    {
        posan1=000;
        posan2=-posan1/2;
        posan3=-posan1/2;
        for (double t=0;t<interval; t+=dts)
        {

            ep1=posan1-m1.GetPosition();
            m1.SetTorque((ep1 > pi1));

            ep2=posan2-m2.GetPosition();
            m2.SetTorque(ep2 > pi2);

            ep3=posan3-m3.GetPosition();
            m3.SetTorque(ep3 > pi3);

            usleep(uint(dts*1000*1000));

        }

        m1.SetTorque(0);
        m2.SetTorque(0);
        m3.SetTorque(0);

        sleep(1);

     }

//    pl1.Plot();//needs model
    targets.close();
    controls.close();
    responses.close();

return 0;

}
