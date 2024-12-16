/**
 * Copyright(C) 2019 Rokae Technology Co., Ltd.
 * All Rights Reserved.
 *
 * Information in this file is the intellectual property of Rokae Technology
 * Co., Ltd, And may contains trade secrets that must be stored and viewed
 * confidentially.
 */

/**
 * MOVE指令示例
 */

#include <cmath>
#include <functional>
#include <iostream>

#include "control_tools.h"
#include "ini.h"
#include "move.h"
#include "print_rci.h"
#include "rci_data/command_types.h"
#include "rci_data/robot_datas.h"
#include "robot.h"
#include "xmate_exception.h"

using namespace xmate;
using JointControl =
    std::function<JointPositions(RCI::robot::RobotState robot_state)>;
using CartesianControl =
    std::function<CartesianPose(RCI::robot::RobotState robot_state)>;

int main(int argc, char *argv[]) {
  std::string ipaddr = "192.168.0.160";
  uint16_t port = 1337;

  std::string file = "../../xmate.ini";
  INIParser ini;
  if (ini.ReadINI(file)) {
    ipaddr = ini.GetString("network", "ip");
    port = static_cast<uint16_t>(ini.GetInt("network", "port"));
  }
  xmate::Robot robot(ipaddr, port, XmateType::XMATE3);
  sleep(1);
  robot.setMotorPower(1);
  try {
    const double PI = 3.14159;
    std::array<double, 7> q_init;
    //注意，6轴xMate机器人同样传入7个角度，最后一个数默认是0
    std::array<double, 7> q_drag = {{0,PI/6,PI/3,0,PI/2,0,0}};
    q_init = robot.receiveRobotState().q;
    //回到拖拽位姿
    MOVEJ(0.2, q_init, q_drag, robot);
    //设置TCP
    robot.setCoor({{1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1}});
    robot.setLoad(0.0, {{0.0, 0.0, 0.0}}, {{0, 0, 0, 0, 0, 0, 0, 0, 0}});

    RCI::robot::RobotState robot_state = robot.receiveRobotState();
    cart_pos pos_start, pos_end, pos_middle, pos_end_x;
    pos_start.pos = robot_state.toolTobase_pos_m;

    Eigen::Matrix3d rot_start, rot_change, rot_end;
    Eigen::Vector3d trans_start, trans_change, trans_end;
    ArrayTo(pos_start.pos, rot_start, trans_start);
    Eigen::Vector3d euler(-0.1, -0.1, -0.1);
    EulerToMatrix(euler, rot_change);
    rot_end = rot_start;
    // rot_end = rot_start*rot_change;
    trans_end = trans_start;
    trans_end[1] -= 0.2;
    ToArray(rot_end, trans_end, pos_end.pos);
    pos_middle = pos_start;
    pos_middle.pos[7] -= 0.1;
    pos_middle.pos[3] -= 0.1;
    robot.setCollisionBehavior({{15,30,20, 8,10, 6 }});
    MOVEL(0.2, pos_start, pos_end, robot);
    robot.setCollisionBehavior({{15,30,20, 8,10, 6 }});
    MOVEL(0.2, pos_end, pos_start, robot);
    MOVEC(0.2, pos_start, pos_middle, pos_end, robot);
    MOVEC(0.2, pos_end, pos_middle, pos_start, robot);

    double time = 0;

    if(robot.GetRobotRunningState()!=RobotRunningState::DRAGING){
        robot.startDrag(RCI::robot::DragSpace::CARTESIAN_DRAG, RCI::robot::DragType::FREELY);
        while(time<30){
            time+=0.001;
            std::array<double,16> tool_base = robot.receiveRobotState().toolTobase_pos_m;
            usleep(1000);
        }
        robot.stopDrag();
    }



  } catch (xmate::ControlException &e) {
    std::cout << e.what() << std::endl;
    return 0;
  }

    //笛卡爾空間運動
  robot.setMotorPower(1);
  robot.startMove(
      RCI::robot::StartMoveRequest::ControllerMode::kCartesianPosition,
      RCI::robot::StartMoveRequest::MotionGeneratorMode::kCartesianPosition);

  uint64_t init_time;
  std::array<double, 16> init_position;
  static bool init = true;
  double init_psi;
  double time = 0;

  CartesianControl cartesian_position_callback;
  cartesian_position_callback =
      [&](RCI::robot::RobotState robot_state) -> CartesianPose {
    time += 0.001;
    if (init == true) {
      init_position = robot_state.toolTobase_pos_m;
      init_psi = robot_state.psi_m;
      init = false;
    }
    constexpr double kRadius = 0.2;
    double angle = M_PI / 4 * (1 - std::cos(M_PI / 5 * time));
    double delta_x = kRadius * std::sin(angle);
    double delta_z = kRadius * (std::cos(angle) - 1);

    CartesianPose output{};
    output.toolTobase_pos_c = init_position;
    output.toolTobase_pos_c[11] += delta_z;
    output.psi_c = init_psi;

    return output;
  };

  robot.Control(cartesian_position_callback);
  return 0;
}
