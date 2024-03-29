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

int arm_status = 0;

//昇降と受け渡しで使うエアシリ
enum class moveslipperCommands : uint8_t//エアシリのコマンド一覧
{
    shutdown_cmd        = 0x00,
    enable_cmd           = 0x10,

    elavate_case_cmd     = 0x01,
};

//モータ
enum class MotorCommands : uint8_t
{
    shutdown_cmd    = 0x0000,
    enable_cmd       = 0x0001,
};

class NrMain
{
public:
    NrMain(void);

private:
    void joyCallback(const sensor_msgs::Joy::ConstPtr &msg);

    ros::NodeHandle nh_;

    //足回り
    ros::Publisher base_cmd_pub;
    std_msgs::UInt8 base_cmd_msg;

    //エアシリ
    ros::Subscriber move_slipper_status_sub;
    ros::Publisher move_slipper_cmd_pub;
    std_msgs::UInt8 move_slipper_cmd_msg;

    //アーム
    ros::Subscriber motor_status_sub;
    ros::Publisher set_collectingcase_cmd_pub;
    ros::Publisher set_collectingcase_cmd_vel_pub;
    std_msgs::UInt8 set_collectingcase_cmd_msg;
    std_msgs::Float32 set_collectingcase_cmd_vel_msg;

    ros::Subscriber joy_sub;

    ros::Publisher cmd_vel_pub;
    geometry_msgs::Twist cmd_vel_msg;

    //この下の関数はcanにこのコマンドを行うために送るデータを決定する関数
    void shutdown(void);
    void enable(void);

    void set_arm(float);

    void elavate_case(uint8_t);

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

NrMain::NrMain(void)
{
	//多分メッセージの宣言みたいなことしてる
    this->base_cmd_pub = nh_.advertise<std_msgs::UInt8>("base/cmd", 10);

    this->set_collectingcase_cmd_pub = nh_.advertise<std_msgs::UInt8>("set_collectingcase_cmd", 10);
    this->set_collectingcase_cmd_vel_pub = nh_.advertise<std_msgs::Float32>("set_collectingcase_cmd_vel", 10);

    this->move_slipper_cmd_pub = nh_.advertise<std_msgs::UInt8>("moveslipper/cmd", 1);

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

}

void NrMain::joyCallback(const sensor_msgs::Joy::ConstPtr &joy)
{
    static bool last_a = false;
    static bool last_b = false;
    static bool last_x = false;
    static bool last_y = false;
    static bool last_start = false;
    static bool last_select = false;
    static float last_padx = 0.0;
 
    bool _a = joy->buttons[ButtonA];
    bool _b = joy->buttons[ButtonB];
    bool _x = joy->buttons[ButtonX];
    bool _y = joy->buttons[ButtonY];
    bool _start = joy->buttons[ButtonStart];
    bool _select = joy->buttons[ButtonSelect];//selectと書いてあるが、実際はbackである
    float _padx = joy->axes[AxisDPadX];


    if (_select)
    {
       ROS_INFO("shutdown"); 
       this->shutdown();
    }
    else if(_start)
    {
        ROS_INFO("enable");
    	this->enable();
    }
    else if(_a && !last_a)
    {
        ROS_INFO("move arm");
	this->NrMain::set_arm(-0.1);
    } 
   else if (_b && !last_b)
    {
        ROS_INFO("elevated the case.");
 	NrMain::elavate_case(0x01);
 
    }
    else if (_x && !last_x)
    {
	ROS_INFO("stop arm");
	this->NrMain::set_arm(0.0);
    }
    else if(_y && !last_y)
    {
	ROS_INFO("return arm");
	this->NrMain::set_arm(0.1);
    }

    last_a = _a;
    last_b = _b;
    last_x = _x;
    last_y = _y;
    last_padx = _padx;

 }

void NrMain::shutdown(void)
{

    move_slipper_cmd_msg.data = (uint8_t)moveslipperCommands::shutdown_cmd;
    move_slipper_cmd_pub.publish(move_slipper_cmd_msg);

    base_cmd_msg.data = (uint8_t)MotorCommands::shutdown_cmd;
    base_cmd_pub.publish(base_cmd_msg);

    set_collectingcase_cmd_msg.data = (uint8_t)MotorCommands::shutdown_cmd;
    set_collectingcase_cmd_pub.publish(set_collectingcase_cmd_msg);
}

void NrMain::enable(void)
{
    move_slipper_cmd_msg.data = (uint8_t)moveslipperCommands::enable_cmd;
    move_slipper_cmd_pub.publish(move_slipper_cmd_msg);

    base_cmd_msg.data = (uint8_t)MotorCommands::enable_cmd;
    base_cmd_pub.publish(base_cmd_msg);

    set_collectingcase_cmd_msg.data = (uint8_t)MotorCommands::enable_cmd;
    set_collectingcase_cmd_pub.publish(set_collectingcase_cmd_msg);
}

void NrMain::set_arm(float angle)//アームを動かす
{
	set_collectingcase_cmd_msg.data = (uint8_t)MotorCommands::shutdown_cmd;
	set_collectingcase_cmd_pub.publish(set_collectingcase_cmd_msg);
	set_collectingcase_cmd_msg.data = (uint8_t)MotorCommands::enable_cmd;
	set_collectingcase_cmd_pub.publish(set_collectingcase_cmd_msg);
	set_collectingcase_cmd_vel_msg.data = angle;
	set_collectingcase_cmd_vel_pub.publish(set_collectingcase_cmd_vel_msg);
}

void NrMain::elavate_case(uint8_t data)//昇降
{
	move_slipper_cmd_msg.data = data;
	move_slipper_cmd_pub.publish(move_slipper_cmd_msg);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "nr_main");

    NrMain *instance = new NrMain();
    ROS_INFO("NR main node has started.");

    ros::spin();
    ROS_INFO("NR main node has been terminated.");
}
