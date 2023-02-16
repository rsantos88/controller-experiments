

#include <iostream>


#include "TestPort.h"
//#include "CanBusPort.h"
#include "Cia402device.h"
#include "CiA301CommPort.h"
#include "SocketCanPort.h"

using namespace std;

int main(int argc, char *argv[])
{


    ///prepare ports
    /// Open a port address with a PortBase Object
    SocketCanPort p1("can1");

    ///Create a joint and give a canopen id, and a 301port (by constructor)
    CiA402Device j1(31,&p1);

    ///Check the status of the device
    j1.PrintStatus();
    ///Start the node (allow sdo, pdo)
    j1.Reset();
    j1.StartNode();
    ///Change the state machine to switched on
    j1.SwitchOn();
    ///Set position operation mode
    j1.OperationMode(od::positionmode);
    ///Check the status of the device
    j1.PrintStatus();


    SocketCanPort p2("can1");
    CiA402Device j2(32,&p2);
    j2.Reset();
    j2.PrintStatus();
    j2.StartNode();
    j2.SwitchOn();
    j2.OperationMode(od::positionmode);
    j2.PrintStatus();


    SocketCanPort p3("can1");
    CiA402Device j3(33,&p1);
    j3.Reset();
    j3.PrintStatus();
    j3.StartNode();
    j3.SwitchOn();
    j3.OperationMode(od::positionmode);
    j3.PrintStatus();

    return 0;
}
