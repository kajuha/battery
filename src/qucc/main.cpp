// #include <ros/ros.h>
#include <rclcpp/rclcpp.hpp>
// #include <std_msgs/Float64.h>
#include <std_msgs/msg/float64.hpp>
// #include <sensor_msgs/BatteryState.h>
#include <sensor_msgs/msg/battery_state.hpp>

#include <unistd.h>
#include <pthread.h>
#include <semaphore.h>
#include <fcntl.h>
#include <termios.h>
#include <sys/time.h>
#include <stdlib.h>
#include <limits.h>

#include <iostream>
#include <queue>

#include "qucc.h"

#include <string.h>
#define __FILENAME__ (strrchr(__FILE__, '/') ? strrchr(__FILE__, '/') + 1 : __FILE__)

using namespace std;

int main(int argc, char* argv[])
{
	// ros::init(argc, argv, "battery");
	rclcpp::init(argc, argv);
	// ros::NodeHandle nh("~");
	rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("battery",
		rclcpp::NodeOptions().allow_undeclared_parameters(true).automatically_declare_parameters_from_overrides(true));

	std::string serial_port;
	int baud_rate;
	int slave_num;
	std::string bms_model;
	double QUERY_BATTERY_SEC;

	// #if 0
	// ros::param::get("~serial_port", serial_port);
	// ros::param::get("~baud_rate", baud_rate);
	// ros::param::get("~bms_model", bms_model);
	// #else
	// nh.getParam("serial_port", serial_port);
	// nh.getParam("baud_rate", baud_rate);
	// nh.getParam("bms_model", bms_model);
	// #endif
	rclcpp::Parameter param_serial_port = node->get_parameter("serial_port");
	serial_port = param_serial_port.as_string();
	rclcpp::Parameter param_baud_rate = node->get_parameter("baud_rate");
	baud_rate = param_baud_rate.as_int();
	rclcpp::Parameter param_slave_num = node->get_parameter("slave_num");
	slave_num = param_slave_num.as_int();
	rclcpp::Parameter param_bms_model = node->get_parameter("bms_model");
	bms_model = param_bms_model.as_string();
	rclcpp::Parameter param_QUERY_BATTERY_SEC = node->get_parameter("QUERY_BATTERY_SEC");
	QUERY_BATTERY_SEC = param_QUERY_BATTERY_SEC.as_double();

	#if 1
	auto parameters_and_prefixes = node->list_parameters({}, 0);
	rclcpp::Parameter value;
	for (auto name: parameters_and_prefixes.names) {
		value = node->get_parameter(name.c_str());
		RCLCPP_INFO(node->get_logger(), "Param[name:%s, value:%s]", name.c_str(), value.value_to_string().c_str());
	}
	#endif

	RCLCPP_INFO(node->get_logger(), "%s, %d", serial_port.c_str(), baud_rate);

	RCLCPP_INFO(node->get_logger(), "FILE NAME : %s", __FILENAME__);
	RCLCPP_INFO(node->get_logger(), "BMS MODEL : %s", bms_model.c_str());

	// ros::Publisher battery_pub = nh.advertise<sensor_msgs::BatteryState>("battery", 1000, true);
	rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr battery_pub =
		node->create_publisher<sensor_msgs::msg::BatteryState>("/battery", 1);

	// sensor_msgs::BatteryState batteryState;
	sensor_msgs::msg::BatteryState batteryState;

    char real_name[NAME_MAX] = {'\0', };

	realpath(serial_port.c_str(), real_name);

	RCLCPP_INFO(node->get_logger(), "%s->%s %d", serial_port.c_str(), real_name, baud_rate);

	Qucc qucc = Qucc(node, real_name, baud_rate);

	if (qucc.initSerial() == false) {
		return 0;
	}

	// ros::Rate r(1000);
	rclcpp::Rate r(1000);

	// #define QUERY_BATTERY_SEC 1.0
	// double time_cur = ros::Time::now().toSec();
	double time_cur = rclcpp::Clock().now().seconds();
	double time_pre = time_cur;
	double time_diff;

	// while (ros::ok())
	while (rclcpp::ok())
	{
		qucc.receiveQuccState();

		if (qucc.isParsed) {
			qucc.isParsed = false;

			// batteryState.header.stamp = ros::Time::now();
			batteryState.header.stamp = rclcpp::Clock().now();
			batteryState.voltage = qucc._quccInfo.voltage_v;
			batteryState.current = qucc._quccInfo.current_a;
			batteryState.capacity = qucc._quccInfo.remaining_capacity_ah;
			batteryState.design_capacity = qucc._quccInfo.norminal_capacity_ah;
			batteryState.percentage = qucc._quccInfo.remaining_capacity_percent / 100.0;
			batteryState.power_supply_technology = batteryState.POWER_SUPPLY_TECHNOLOGY_LIFE;
			// battery_pub.publish(batteryState);
			battery_pub->publish(batteryState);

			#if 1
			RCLCPP_INFO(node->get_logger(), "%lf: %6.2f V, %6.2f A, %6.2f %%", time_cur, qucc._quccInfo.voltage_v, qucc._quccInfo.current_a, qucc._quccInfo.remaining_capacity_percent);
			#endif
		}

		// time_cur = ros::Time::now().toSec();
		time_cur = rclcpp::Clock().now().seconds();
		time_diff = time_cur - time_pre;
		if ( time_diff > QUERY_BATTERY_SEC ) {
			qucc.sendQuccCmd();

			time_pre = time_cur;
		}

		// ros::spinOnce();
		rclcpp::spin_some(node);

		r.sleep();
	}

	qucc.closeSerial();

	rclcpp::shutdown();

	return 0;
}