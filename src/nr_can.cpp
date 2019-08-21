/*
 * mr1_can.cpp
 *
 *  Created on: Feb 27, 2019
 *      Author: yuto
 */

#include <ros/ros.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/Float32.h>

#include <boost/array.hpp>

#include <can_msgs/CanFrame.h>

#define CAN_MTU 8

template<typename T>
union _Encapsulator
{
    T data;
    uint64_t i;
};

template <typename T>
void can_unpack(const boost::array<uint8_t, CAN_MTU> &buf, T &data)
{
    _Encapsulator<T> _e;

    for(int i = 0; i < sizeof(T); i++)
    {
        _e.i = (_e.i << 8) | (uint64_t)(buf[i]);
    }

    data = _e.data;
}

template<typename T>
void can_pack(boost::array<uint8_t, CAN_MTU> &buf, const T data)
{
    _Encapsulator<T> _e;
    _e.data = data;

    for(int i = sizeof(T); i > 0;)
    {
        i--;
        buf[i] = _e.i & 0xff;
        _e.i >>= 8;
    }
}

class NrCanNode
{
public:
    NrCanNode(void);

private:
    void baseCmdCallback(const std_msgs::UInt8::ConstPtr& msg);
    void motor0CmdVelCallback(const std_msgs::Float32::ConstPtr& msg);
    void motor1CmdVelCallback(const std_msgs::Float32::ConstPtr& msg);
    void motor2CmdVelCallback(const std_msgs::Float32::ConstPtr& msg);
    void motor3CmdVelCallback(const std_msgs::Float32::ConstPtr& msg);

    void moveslipperCmdCallback(const std_msgs::UInt8::ConstPtr& msg);
    void setcollectingcaseCmdCallback(const std_msgs::UInt8::ConstPtr& msg);
    void setcollectingcaseCmdPosCallback(const std_msgs::Float32::ConstPtr& msg);

    void canRxCallback(const can_msgs::CanFrame::ConstPtr &msg);

    template<typename T>
    void sendData(const uint16_t id, const T data);

    ros::NodeHandle _nh;
    ros::Publisher _can_tx_pub;
    ros::Subscriber _can_rx_sub;

    ros::Publisher  _moveslipper_status_pub;
    ros::Subscriber _moveslipper_cmd_sub;

    ros::Subscriber	_base_cmd_sub;
    ros::Subscriber _base_motor0_cmd_vel_sub;
    ros::Subscriber _base_motor1_cmd_vel_sub;
    ros::Subscriber _base_motor2_cmd_vel_sub;
    ros::Subscriber _base_motor3_cmd_vel_sub;

    ros::Publisher  _set_collectingcase_status_pub;
    ros::Subscriber _set_collectingcase_cmd_sub;
    ros::Subscriber _set_collectingcase_cmd_pos_sub;


    static constexpr uint16_t id_moveslipperStatus	        = 0x300;
    static constexpr uint16_t id_moveslipperCmd	        = 0x301;

    static constexpr uint16_t id_base_motor0_cmd	    = 0x4a6;
    static constexpr uint16_t id_base_motor0_cmd_vel    = 0x4a7;
    static constexpr uint16_t id_base_motor1_cmd	    = 0x4a2;
    static constexpr uint16_t id_base_motor1_cmd_vel    = 0x4a3;
    static constexpr uint16_t id_base_motor2_cmd	    = 0x4a8;
    static constexpr uint16_t id_base_motor2_cmd_vel    = 0x4a9;
    static constexpr uint16_t id_base_motor3_cmd	    = 0x4a8;
    static constexpr uint16_t id_base_motor3_cmd_vel    = 0x4a9;

    static constexpr uint16_t id_set_collectingcase_cmd         = 0x4f4;
    static constexpr uint16_t id_set_collectingcase_cmd_pos     = 0x4f5;
    static constexpr uint16_t id_set_collectingcase_status      = 0x4f7;
};

NrCanNode::NrCanNode(void)
{
    _can_tx_pub				    = _nh.advertise<can_msgs::CanFrame>("can_tx", 10);
    _can_rx_sub				    = _nh.subscribe<can_msgs::CanFrame>("can_rx", 10, &NrCanNode::canRxCallback, this);

    _moveslipper_status_pub	    = _nh.advertise<std_msgs::UInt8>("moveslipper/status", 10);
    _moveslipper_cmd_sub		    = _nh.subscribe<std_msgs::UInt8>("moveslipper/cmd", 10, &NrCanNode::moveslipperCmdCallback, this);

    _base_cmd_sub			    = _nh.subscribe<std_msgs::UInt8>("base/cmd", 10 , &NrCanNode::baseCmdCallback, this);
    _base_motor0_cmd_vel_sub	= _nh.subscribe<std_msgs::Float32>("base/motor0_cmd_vel", 10, &NrCanNode::motor0CmdVelCallback, this);
    _base_motor1_cmd_vel_sub	= _nh.subscribe<std_msgs::Float32>("base/motor1_cmd_vel", 10, &NrCanNode::motor1CmdVelCallback, this);
    _base_motor2_cmd_vel_sub	= _nh.subscribe<std_msgs::Float32>("base/motor2_cmd_vel", 10, &NrCanNode::motor2CmdVelCallback, this);
    _base_motor3_cmd_vel_sub	= _nh.subscribe<std_msgs::Float32>("base/motor3_cmd_vel", 10, &NrCanNode::motor3CmdVelCallback, this);

    _set_collectingcase_status_pub      = _nh.advertise<std_msgs::UInt8>("motor_status", 10);

    _set_collectingcase_cmd_sub	        = _nh.subscribe<std_msgs::UInt8>("set_collectingcase_cmd", 10, &NrCanNode::setcollectingcaseCmdCallback, this);
    _set_collectingcase_cmd_pos_sub	    = _nh.subscribe<std_msgs::Float32>("set_collectingcase_cmd_pos", 10, &NrCanNode::setcollectingcaseCmdPosCallback, this);
}


void NrCanNode::baseCmdCallback(const std_msgs::UInt8::ConstPtr& msg)
{
    this->sendData(id_base_motor0_cmd, msg->data);
    this->sendData(id_base_motor1_cmd, msg->data);
    this->sendData(id_base_motor2_cmd, msg->data);
    this->sendData(id_base_motor3_cmd, msg->data);
}

void NrCanNode::motor0CmdVelCallback(const std_msgs::Float32::ConstPtr& msg)
{
    this->sendData(id_base_motor0_cmd_vel, msg->data);
}

void NrCanNode::motor1CmdVelCallback(const std_msgs::Float32::ConstPtr& msg)
{
    this->sendData(id_base_motor1_cmd_vel, msg->data);
}

void NrCanNode::motor2CmdVelCallback(const std_msgs::Float32::ConstPtr& msg)
{
    this->sendData(id_base_motor2_cmd_vel, msg->data);
}

void NrCanNode::motor3CmdVelCallback(const std_msgs::Float32::ConstPtr& msg)
{
    this->sendData(id_base_motor3_cmd_vel, msg->data);
}

void NrCanNode::moveslipperCmdCallback(const std_msgs::UInt8::ConstPtr& msg)
{
    this->sendData(id_moveslipperCmd, msg->data);
}

void NrCanNode::setcollectingcaseCmdCallback(const std_msgs::UInt8::ConstPtr& msg)
{
    this->sendData(id_set_collectingcase_cmd, msg->data);
}

void NrCanNode::setcollectingcaseCmdPosCallback(const std_msgs::Float32::ConstPtr& msg)
{
    this->sendData(id_set_collectingcase_cmd_pos, msg->data);
}

void NrCanNode::canRxCallback(const can_msgs::CanFrame::ConstPtr &msg)
{
    std_msgs::UInt8 _moveslipper_status_msg;
    std_msgs::UInt8 _set_collectingcase_status_msg;

    switch(msg->id)
    {
        case id_moveslipperStatus:
            can_unpack(msg->data, _moveslipper_status_msg.data);
            _moveslipper_status_pub.publish(_moveslipper_status_msg);
            break;

        case id_set_collectingcase_status:
            can_unpack(msg->data, _set_collectingcase_status_msg.data);
            _set_collectingcase_status_pub.publish(_set_collectingcase_status_msg);
            break;

        default:
            break;
    }
}

template<typename T>
void NrCanNode::sendData(const uint16_t id, const T data)
{
    can_msgs::CanFrame frame;
    frame.id = id;
    frame.is_rtr = false;
    frame.is_extended = false;
    frame.is_error = false;

    frame.dlc = sizeof(T);

    can_pack<T>(frame.data, data);

    _can_tx_pub.publish(frame);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "nr_can");
    ROS_INFO("nr_can node has started.");

    NrCanNode *nrCanNode = new NrCanNode();

    ros::spin();
}
