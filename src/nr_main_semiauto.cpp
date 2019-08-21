/*
 * mr1_main_semiauto.cpp
 *
 *  Created on: Feb 27, 2019
 *      Author: yuto
 */

#include <ros/ros.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/Joy.h>
#include <vector>
#include <string>

#include "Coordinates_.hpp"

enum class ControllerStatus : uint16_t
{
    shutdown    = 0x0000,
    reset       = 0x0001,
};

enum class ControllerCommands : uint16_t//全体でのコマンド一覧
{
    shutdown, // shutdown
	reset,

    elavate_case,
    descent_case,
    send_slipper,
    shrink_cylinder,

};

//昇降と受け渡しで使うエアシリ
enum class moveslipperCommands : uint16_t//エアシリのコマンド一覧
{
    shutdown_cmd        = 0b0000,
    reset_cmd           = 0b0000,

    elavate_case_cmd     = 0b0001,
	send_slipper_cmd     = 0b0010,
};

//足回りのモータ
enum class BaseCommands : uint16_t//足回りのコマンド一覧
{
    shutdown_cmd    = 0x0000,
    reset_cmd       = 0x0001,
};

enum class MotorCommands : uint8_t//モータのコマンド一覧
{
    shutdown_cmd    = 0x0000,
    reset_cmd       = 0x0001,
    homing_cmd      = 0x0010,
};

class NrMain
{
public:
    NrMain(void);

private:
    void motorStatusCallback(const std_msgs::UInt8::ConstPtr &msg);
    void joyCallback(const sensor_msgs::Joy::ConstPtr &msg);

    ros::NodeHandle nh_;

    //足回り
    ros::Publisher base_cmd_pub;
    std_msgs::UInt16 base_cmd_msg;

    //エアシリ
    ros::Subscriber move_slipper_status_sub;
    ros::Publisher move_slipper_cmd_pub;
    std_msgs::UInt16 move_slipper_cmd_msg;

    //アーム
    ros::Subscriber motor_status_sub;
    ros::Publisher set_collectingcase_cmd_pub;
    ros::Publisher set_collectingcase_cmd_pos_pub;
    std_msgs::UInt8 set_collectingcase_cmd_msg;
    std_msgs::Float32 set_collectingcase_cmd_pos_msg;

    ros::Subscriber joy_sub;

    const std::vector<ControllerCommands> *command_list;

    static const std::vector<ControllerCommands> commands;

    //この下の関数はcanにこのコマンドを行うために送るデータを決定する関数
    void shutdown(void);
    void reset(void);

    void set_arm(void);
    void reset_arm(void);

    void elavate_case(void);
    void descent_case(void);

    void send_slipper(void);
    void shrink_cylinder(void);

    static int ButtonA;
	static int ButtonB;
	static int ButtonX;
	static int ButtonY;
	static int ButtonLB;
	static int ButtonRB;
	static int ButtonSelect;
	static int ButtonStart;
	static int ButtonLeftThumb;
	static int ButtonRightThumb;

	static int AxisDPadX;
	static int AxisDPadY;
	static int AxisLeftThumbX;
	static int AxisLeftThumbY;
	static int AxisRightThumbX;
    static int AxisRightThumbY;
    static int AxisLeftTrigger;
    static int AxisRightTrigger;
};

int NrMain::ButtonA = 0;
int NrMain::ButtonB = 1;
int NrMain::ButtonX = 2;
int NrMain::ButtonY = 3;
int NrMain::ButtonLB = 4;
int NrMain::ButtonRB = 5;
int NrMain::ButtonSelect = 6;
int NrMain::ButtonStart = 7;
int NrMain::ButtonLeftThumb = 9;
int NrMain::ButtonRightThumb = 10;

int NrMain::AxisDPadX = 0;
int NrMain::AxisDPadY = 1;
int NrMain::AxisLeftThumbX = 6;
int NrMain::AxisLeftThumbY = 7;
int NrMain::AxisRightThumbX = 3;
int NrMain::AxisRightThumbY = 4;
int NrMain::AxisLeftTrigger = 2;
int NrMain::AxisRightTrigger = 5;

const std::vector<ControllerCommands> NrMain::commands
    {
		ControllerCommands::elavate_case,
		ControllerCommands::descent_case,

		ControllerCommands::send_slipper,
		ControllerCommands::shrink_cylinder,
    };

NrMain::NrMain(void)
{
	//多分メッセージの宣言みたいなことしてる
    this->base_cmd_pub = nh_.advertise<std_msgs::UInt16>("base/cmd", 10);

    this->motor_status_sub = nh_.subscribe<std_msgs::UInt8>("motor_status", 10, &NrMain::motorStatusCallback, this);
    this->set_collectingcase_cmd_pub = nh_.advertise<std_msgs::UInt8>("set_collectingcase_cmd", 10);
    this->set_collectingcase_cmd_pos_pub = nh_.advertise<std_msgs::Float32>("set_collectingcase_cmd_pos", 10);

    this->move_slipper_cmd_pub = nh_.advertise<std_msgs::UInt16>("move_slipper/cmd", 1);

    this->joy_sub = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &NrMain::joyCallback, this);

    nh_.getParam("ButtonA", ButtonA);
    nh_.getParam("ButtonB", ButtonB);
    nh_.getParam("ButtonX", ButtonX);
    nh_.getParam("ButtonY", ButtonY);
    nh_.getParam("ButtonLB", ButtonLB);
    nh_.getParam("ButtonRB", ButtonRB);
    nh_.getParam("ButtonSelect", ButtonSelect);
    nh_.getParam("ButtonStart", ButtonStart);
    nh_.getParam("ButtonLeftThumb", ButtonLeftThumb);
    nh_.getParam("ButtonRightThumb", ButtonRightThumb);

    nh_.getParam("AxisLeftThumbX", AxisLeftThumbX);
    nh_.getParam("AxisLeftThumbY", AxisLeftThumbY);
    nh_.getParam("AxisRightThumbX", AxisRightThumbX);
    nh_.getParam("AxisRightThumbY", AxisRightThumbY);

    this->command_list = &NrMain::commands;
}

void NrMain::motorStatusCallback(const std_msgs::UInt8::ConstPtr &msg)
{
	ControllerCommands status = (ControllerCommands)msg->data;

    switch (status)
    {
        case ControllerCommands::shutdown:
            if (this->base_last_status != ControllerCommands::shutdown)
            {
                this->shutdown();
            }
            break;

        case ControllerCommands::reset:
            if (this->base_last_status == ControllerCommands::shutdown)
            {
                this->reset();
                this->_has_base_reseted = true;
            }
            break;

        default:
            break;
    }

    base_last_status = status;
}

void NrMain::joyCallback(const sensor_msgs::Joy::ConstPtr &joy)
{
    static bool last_a = false;
    static bool last_b = false;
    static bool last_x = false;
    static bool last_y = false;

    bool _a = joy->buttons[ButtonA];
    bool _b = joy->buttons[ButtonB];
    bool _x = joy->buttons[ButtonX];
    bool _y = joy->buttons[ButtonY];
    bool _select = joy->buttons[ButtonSelect];
    bool _start = joy->buttons[ButtonStart];

    int arm_status = 0;
    int elevator_status = 0;
    int send_status = 0;

    if (_start)
    {
        this->shutdown();
    }

    if(_select)
    {
    	this->reset();
    	arm_status = 0;
    	elevator_status = 0;
    	send_status = 0;
    }

    if (_a && !last_a)
    {
    	switch(arm_status)
    	{
    		case 2:
    			NrMain::reset_arm();
    			arm_status = 0;
    		break;

    		default:
    			NrMain::set_arm();
    			arm_status =+ 1;
    			break;
    	}
    }
    else if (_b && !last_b)
    {
    	if(elevator_status = 0)
    	{
        	NrMain::elavate_case();
        	ROS_INFO("elevated the case.");
        	elevator_status = 1;
    	}
    	else
    	{
        	NrMain::descent_case();
        	ROS_INFO("descent the case.");
        	elevator_status = 0;
    	}
    }
    else if (_x && !last_x)
    {
    	if (send_status = 0)
    	{
        	NrMain::send_slipper();
        	ROS_INFO("sending the slipper.");
        	send_status = 1;
    	}
    	else
    	{
    		NrMain::shrink_cylinder();
    		ROS_INFO("sending completed.");
    		send_status = 0;
    	}
    }
    else if (_y && !last_y)
    {
    }

    last_a = _a;
    last_b = _b;
    last_x = _x;
    last_y = _y;

    if (this->_is_manual_enabled)
    {
        double vel_x = joy->axes[AxisRightThumbX];
        double vel_y = joy->axes[AxisRightThumbY];
        double vel_yaw_l = (joy->axes[AxisLeftTrigger] - 1.0) * (1.0 - 0.0) / (- 1.0 - 1.0) + 0.0;
        double vel_yaw_r = (joy->axes[AxisRightTrigger] - 1.0) * (- 1.0 - 0.0) / (- 1.0 - 1.0) + 0.0;
        double vel_yaw = vel_yaw_l + vel_yaw_r;

        double vel_norm = hypot(vel_x, vel_y);
        if (vel_norm > 1.0)
        {
            vel_x /= vel_norm;
            vel_y /= vel_norm;
        }

        this->cmd_vel_msg.linear.x = -vel_x;
        this->cmd_vel_msg.linear.y = vel_y;
        this->cmd_vel_msg.angular.z = vel_yaw;
        this->cmd_vel_pub.publish(this->cmd_vel_msg);
    }
}

void NrMain::shutdown(void)
{
    move_slipper_cmd_msg.data = (uint16_t)moveslipperCommands::shutdown_cmd;
    move_slipper_cmd_pub.publish(move_slipper_cmd_msg);

    base_cmd_msg.data = (uint16_t)BaseCommands::shutdown_cmd;
    base_cmd_pub.publish(base_cmd_msg);

    set_collectingcase_cmd_msg.data = (uint8_t)MotorCommands::shutdown_cmd;
    set_collectingcase_cmd_pub.publish(set_collectingcase_cmd_msg);
}

void NrMain::reset(void)
{
    ROS_INFO("reset.");

    move_slipper_cmd_msg.data = (uint16_t)moveslipperCommands::reset_cmd;
    move_slipper_cmd_pub.publish(move_slipper_cmd_msg);

    base_cmd_msg.data = (uint16_t)BaseCommands::reset_cmd;
    base_cmd_pub.publish(base_cmd_msg);
    NrMain::reset_arm();

    set_collectingcase_cmd_msg.data = (uint8_t)MotorCommands::reset_cmd;
    set_collectingcase_cmd_pub.publish(set_collectingcase_cmd_msg);



    if (this->_op_mode == OpMode::full_op)
    {
        base_cmd_msg.data = (uint16_t)BaseCommands::operational_cmd;
        base_cmd_pub.publish(base_cmd_msg);
    }

    this->clear_flags();
}

void NrMain::set_arm(void)//次の位置へとアームを動かす
{
	set_collectingcase_cmd_msg.data = angle;//angleには移動値を入れる予定
	set_collectingcase_cmd_pos_pub.publish(set_collectingcase_cmd_pos_msg);
}

void NrMain::reset_arm(void)//アームを初期位置へと戻す
{
	set_collectingcase_cmd_msg.data = 0.0;
	set_collectingcase_cmd_pos_pub.publish(set_collectingcase_cmd_pos_msg);
}

void NrMain::elavate_case(void)//昇降
{
	move_slipper_status_sub.data |= (uint16_t)moveslipperCommands::elavate_case_cmd;
	move_slipper_cmd_pub.publish(move_slipper_status_sub);
}

void NrMain::descent_case(void)//下降
{
	move_slipper_status_sub.data &= ~(uint16_t)moveslipperCommands::elavate_case_cmd;
	move_slipper_cmd_pub.publish(move_slipper_status_sub);
}

void NrMain::send_slipper(void)//スリッパの受け渡し
{
	move_slipper_status_sub.data ^= (uint16_t)moveslipperCommands::send_slipper_cmd;
	move_slipper_cmd_pub.publish(move_slipper_status_sub);
}

void NrMain::shrink_cylinder(void)//受け渡しで展開したエアシリを元に戻す
{
	move_slipper_status_sub.data ^= (uint16_t)moveslipperCommands::send_slipper_cmd;
	move_slipper_cmd_pub.publish(move_slipper_status_sub);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "nr_main");

    NrMain *instance = new NrMain();
    ROS_INFO("NR main node has started.");

    ros::spin();
    ROS_INFO("NR main node has been terminated.");
}
