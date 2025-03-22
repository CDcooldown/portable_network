#include <ros/ros.h>
#include <iostream>
#include <string>
#include <TeamInfo.h>
#include "dtransmit.hpp"
#include "ActionCommand.h"
#include <map>


void setVector3(geometry_msgs::Vector3& vec, double x, double y, double z) {
    vec.x = x;
    vec.y = y;
    vec.z = z;
}

void mirrorVector3(geometry_msgs::Vector3& vec) {
    vec.x *= -1;
    vec.y *= -1;
    vec.z += 180.;
    while (vec.z > 180)
      vec.z -= 360;
    while (vec.z < -180)
      vec.z += 360;
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "dnetwork_node");
    ros::NodeHandle nh("~");

    ros::Rate rate(60);
    dtransmit::DTransmit* transmitter;
    transmitter = new dtransmit::DTransmit("", false);
    transmitter->addRawRecv(3939, [](void *buffer, std::size_t size) {
                             if (1) {
                               dmsgs::TeamInfo team_info =
                                   *(dmsgs::TeamInfo *)buffer;
                               if (1) {
                                 team_info.recv_timestamp = ros::Time::now();
                                 std::cout <<"Heard message!!" << std::endl;
                                 cout << team_info << endl;
                               }
                             }
                           });
    transmitter->startService();

    dmsgs::ActionCommand command;
    dmsgs::TeamInfo teaminfo;

    teaminfo.player_number = 1;
    setVector3(teaminfo.dest, 100, 100, 0);
    setVector3(teaminfo.final_dest, 100, -100, 180);

    setVector3(teaminfo.attack_target, 450, 0, 0);

    teaminfo.see_ball = true;
    setVector3(teaminfo.ball_global, 200, 200, 180);

    teaminfo.see_circle = true;
    setVector3(teaminfo.circle_global, 5, 5, 180);

    teaminfo.see_goal = true;
    setVector3(teaminfo.goal_global, 450, 130, 180);

    teaminfo.robot_pos.z = 90; // 初始朝向

    teaminfo.state = dmsgs::TeamInfo::ASSISTING;

    double angle = 0.0;
    int radius = 0;
    double angle_increment = 0.05; // 调整角度增量以控制运动速度


    boost::array<uint8_t, 6> mates_online = {true, true, false, true, false, false};
    // teaminfo.mates_online = mates_online;

    while (ros::ok()) {
  
    // teaminfo.attack_right = !teaminfo.attack_right;
    teaminfo.player_number =  1;
    
    mirrorVector3(teaminfo.attack_target);
    mirrorVector3(teaminfo.ball_global);
    
    transmitter->sendRaw(3838, (void *)&teaminfo, sizeof(teaminfo));
    std::cout << "I'm sending Message!!" << std::endl;
    rate.sleep();

    }

    return 0;
    
}