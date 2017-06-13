#ifndef ROS_OROCOS_JOINT_TRAJECTORY_SERVER_H_
#define ROS_OROCOS_JOINT_TRAJECTORY_SERVER_H_

#include <rtt/RTT.hpp>
#include <string>
#include <rst-rt/robot/JointState.hpp>
#include <rst-rt/kinematics/JointAngles.hpp>
#include <rtt/TaskContext.hpp>
#include <rtt/Component.hpp>
#include <rtt/Port.hpp>
#include <urdf/model.h>
#include <trajectory_msgs/JointTrajectory.h>

class ros_orocos_joint_trajectory_server: public RTT::TaskContext{
public:
    ros_orocos_joint_trajectory_server(std::string const & name);

    bool configureHook();
    bool startHook();
    void updateHook();

private:
    bool attachToRobot(const std::string& robot_name);

    std::string _robot_name;
    std::map<std::string, std::vector<std::string> > _map_kin_chains_joints;
    std::map<std::string, boost::shared_ptr<RTT::OutputPort<rstrt::kinematics::JointAngles> > > _kinematic_chains_output_ports;
    std::map<std::string, rstrt::kinematics::JointAngles> _kinematic_chains_desired_joint_state_map;

    RTT::InputPort<trajectory_msgs::JointTrajectory> _joint_trajectory_port;
    trajectory_msgs::JointTrajectory _joint_trajectory_msg;

    bool _send_joint_trj;
    int _trj_counter;
};


#endif
