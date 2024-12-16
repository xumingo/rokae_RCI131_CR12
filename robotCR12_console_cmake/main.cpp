#include <QCoreApplication>

#include <iostream>

#include <winsock2.h>
#include <windows.h>

#include "robot.h"

using namespace xmate;

int main(int argc, char *argv[])
{
    QCoreApplication a(argc, argv);


    std::string ipaddr = "192.168.0.160";
    uint16_t port = 1338;

    // RCI连接机器人
//    xqtmate::Robot robot(ipaddr, port, XmateType::XMATE7_PRO, false);
    Robot robot(ipaddr, port, XmateType::XMATE7_PRO, false, 50); //20241212 xuminghao

    //防止网络连接失败
    Sleep(1000);
    int res = robot.getMotorState();
    std::cout<<"机器人上电状态："<<res<<std::endl;
    int power_state=1;
    robot.setMotorPower(power_state);

    return a.exec();
}
