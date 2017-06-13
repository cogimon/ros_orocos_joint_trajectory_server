#include <ros_orocos_joint_trajectory_server.h>
#include <rtt/plugin/PluginLoader.hpp>
#include <ros/ros.h>
#include <rtt_rosclock/rtt_rosclock.h>
#include <rtt_roscomm/rtt_rostopic.h>
#include <algorithm>

ros_orocos_joint_trajectory_server::ros_orocos_joint_trajectory_server(std::string const & name):
    RTT::TaskContext(name),
    _send_joint_trj(false),
    _trj_counter(0)
{
    this->setActivity(new RTT::Activity(1, 0.01));

    this->addOperation("attachToRobot", &ros_orocos_joint_trajectory_server::attachToRobot,
                this, RTT::ClientThread);
}

bool ros_orocos_joint_trajectory_server::attachToRobot(const std::string &robot_name)
{
    _robot_name = robot_name;
    RTT::log(RTT::Info)<<"Robot name: "<<_robot_name<<RTT::endlog();

    RTT::TaskContext* task_ptr = this->getPeer(robot_name);
    if(!task_ptr){
        RTT::log(RTT::Error)<<"Can not getPeer("<<robot_name<<")"<<RTT::endlog();
        return false;}

    RTT::log(RTT::Info)<<"Found Peer "<<robot_name<<RTT::endlog();

    RTT::OperationCaller<std::map<std::string, std::vector<std::string> >(void) > getKinematicChainsAndJoints
        = task_ptr->getOperation("getKinematicChainsAndJoints");

    _map_kin_chains_joints = getKinematicChainsAndJoints();

    std::map<std::string, std::vector<std::string> >::iterator it;
    for(it = _map_kin_chains_joints.begin(); it != _map_kin_chains_joints.end(); it++)
    {
        std::string kin_chain_name = it->first;
        std::vector<std::string> joint_names = it->second;


        _kinematic_chains_output_ports[kin_chain_name] =
                boost::shared_ptr<RTT::OutputPort<rstrt::kinematics::JointAngles> >(
                            new RTT::OutputPort<rstrt::kinematics::JointAngles>(
                                kin_chain_name+"_"+"JointPositionCtrl"));
        this->addPort(*(_kinematic_chains_output_ports.at(kin_chain_name))).
                doc(kin_chain_name+"_"+"JointPositionCtrl port");
        _kinematic_chains_output_ports.at(kin_chain_name)->connectTo(
                    task_ptr->ports()->getPort(kin_chain_name+"_"+"JointPositionCtrl"));


        rstrt::kinematics::JointAngles tmp(joint_names.size());
        _kinematic_chains_desired_joint_state_map[kin_chain_name] = tmp;


        RTT::log(RTT::Info)<<"Added "<<kin_chain_name<<" port and data"<<RTT::endlog();
    }

    return true;
}

bool ros_orocos_joint_trajectory_server::configureHook()
{
    this->addPort(_joint_trajectory_port).doc("Joint Trajectory from ROS node");

    return true;
}

bool ros_orocos_joint_trajectory_server::startHook()
{
    _joint_trajectory_port.createStream(rtt_roscomm::topic("joint_trajectory"));

    return true;
}

void ros_orocos_joint_trajectory_server::updateHook()
{
    if(!_send_joint_trj)
    {
        RTT::FlowStatus fs = _joint_trajectory_port.read(_joint_trajectory_msg);
        if(fs == 2)
        {
            _send_joint_trj = true;
        }
    }


    if(_send_joint_trj && _joint_trajectory_msg.points.size() > 0)
    {
        std::map<std::string, boost::shared_ptr<RTT::OutputPort<rstrt::kinematics::JointAngles> > >::iterator it;
        for(it = _kinematic_chains_output_ports.begin(); it != _kinematic_chains_output_ports.end(); it++)
        {
            for(unsigned int i = 0; i < _map_kin_chains_joints.at(it->first).size(); ++i)
            {
                int id = std::distance(_joint_trajectory_msg.joint_names.begin(),
                                       std::find(_joint_trajectory_msg.joint_names.begin(), _joint_trajectory_msg.joint_names.end(),
                                       _map_kin_chains_joints.at(it->first)[i]));
                _kinematic_chains_desired_joint_state_map.at(it->first).angles[i] =
                        _joint_trajectory_msg.points[_trj_counter].positions[id];
            }

            _kinematic_chains_output_ports.at(it->first)->write(
                        _kinematic_chains_desired_joint_state_map.at(it->first));
        }
        _trj_counter++;

        if(_trj_counter >= _joint_trajectory_msg.points.size()){
            _send_joint_trj = false;
            _trj_counter = 0;}


    }

}

ORO_CREATE_COMPONENT_LIBRARY()ORO_LIST_COMPONENT_TYPE(ros_orocos_joint_trajectory_server)
