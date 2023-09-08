#include <chrono>
#include <memory>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>
#include <pthread.h>
#include <sys/fcntl.h>
#include <time.h>
#include <sys/types.h>
#include <vector>
#include <iostream>
#include <dirent.h>
#include <signal.h>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/magnetic_field.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"

//interface package srv include...
#include "interfaces/srv/imu_reset.hpp"


#define SERIAL_PORT	"/dev/IMU"
#define SERIAL_SPEED	B115200

typedef struct IMU_DATA
{
	double dQuaternion_x = 0.0;
	double dQuaternion_y = 0.0;
	double dQuaternion_z = 0.0;
	double dQuaternion_w = 1.0;

	double dAngular_velocity_x = 0.0;
	double dAngular_velocity_y = 0.0;
	double dAngular_velocity_z = 0.0;
	
	double dLinear_acceleration_x = 0.0;
	double dLinear_acceleration_y = 0.0;
	double dLinear_acceleration_z = 0.0;
    
	double dEuler_angle_Roll = 0.0;
	double dEuler_angle_Pitch = 0.0;
	double dEuler_angle_Yaw = 0.0;

}IMU_DATA;
IMU_DATA _pIMU_data;

int serial_fd = -1;
double time_offset_in_seconds;
double dSend_Data[10];
double m_dRoll, m_dPitch, m_dYaw;
//single_used TF
bool m_bSingle_TF_option = true; //false;

using namespace std::chrono_literals;

//iahrs_driver class define
class IAHRS : public rclcpp::Node
{
public:
	IAHRS() : Node("iahrs_driver")
	{
		tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(this);
		imu_data_pub = this->create_publisher<sensor_msgs::msg::Imu>("imu/data", rclcpp::SensorDataQoS());

		//Service ListUp
		euler_angle_reset_srv_ = create_service<interfaces::srv::ImuReset>(
        	"all_data_reset", 
		std::bind(&IAHRS::Euler_angle_reset_callback, this, std::placeholders::_1, std::placeholders::_2));

		//PARAM
		this->declare_parameter("m_bSingle_TF_option");
		m_bSingle_TF_option_param = this->get_parameter("m_bSingle_TF_option");
		//Get Param
		m_bSingle_TF_option = m_bSingle_TF_option_param.as_bool();

	}

	////value//////////////////////////////////////////////////////////////////////////////
	std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;
	geometry_msgs::msg::TransformStamped transformStamped;
	sensor_msgs::msg::Imu imu_data_msg;
	rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_data_pub;
	rclcpp::Parameter m_bSingle_TF_option_param;

	////function//////////////////////////////////////////////////////////////////////////////
	int serial_open ()
	{
		printf ("Try to open serial: %s\n", SERIAL_PORT); 

		serial_fd = open(SERIAL_PORT, O_RDWR|O_NOCTTY);
		if (serial_fd < 0) 
		{
			printf ("Error unable to open %s\n", SERIAL_PORT);
			return -1;
		}
		printf ("%s open success\n", SERIAL_PORT);

		struct termios tio;
		tcgetattr(serial_fd, &tio);
		cfmakeraw(&tio);
		tio.c_cflag = CS8|CLOCAL|CREAD;
		tio.c_iflag &= ~(IXON | IXOFF);
		cfsetspeed(&tio, SERIAL_SPEED);
		tio.c_cc[VTIME] = 0;
		tio.c_cc[VMIN] = 0;

		int err = tcsetattr(serial_fd, TCSAFLUSH, &tio);
		if (err != 0) 
		{
			printf ("Error tcsetattr() function return error\n");
			close(serial_fd);
			serial_fd = -1;
			return -1;
		}
		return 0;
	}

	static unsigned long GetTickCount() 
	{
		struct timespec ts;
	
		clock_gettime (CLOCK_MONOTONIC, &ts);

		return ts.tv_sec*1000 + ts.tv_nsec/1000000;
	}

	int SendRecv(const char* command, double* returned_data, int data_length)
	{
		#define COMM_RECV_TIMEOUT	30	

		char temp_buff[256];
		read (serial_fd, temp_buff, 256);

		int command_len = strlen(command);
		int n = write(serial_fd, command, command_len);

		if (n < 0) return -1;

		const int buff_size = 1024;
		int  recv_len = 0;
		char recv_buff[buff_size + 1];

		unsigned long time_start = GetTickCount();

		while (recv_len < buff_size) 
		{
			int n = read (serial_fd, recv_buff + recv_len, buff_size - recv_len);
			if (n < 0) 
			{
				return -1;
			}
			else if (n == 0) 
			{
				usleep(1000);
			}
			else if (n > 0) 
			{
				recv_len += n;

				if (recv_buff[recv_len - 1] == '\r' || recv_buff[recv_len - 1] == '\n') 
				{
					break;
				}
			}

			unsigned long time_current = GetTickCount();
			unsigned long time_delta = time_current - time_start;

			if (time_delta >= COMM_RECV_TIMEOUT) break;
		}
		recv_buff[recv_len] = '\0';

		if (recv_len > 0) 
		{
			if (recv_buff[0] == '!') 
			{
				return -1;
			}
		}

		if (strncmp(command, recv_buff, command_len - 1) == 0) 
		{
			if (recv_buff[command_len - 1] == '=') {
				int data_count = 0;

				char* p = &recv_buff[command_len];
				char* pp = NULL;

				for (int i = 0; i < data_length; i++) 
				{
					if (p[0] == '0' && p[1] == 'x') 
					{
						returned_data[i] = strtol(p+2, &pp, 16);
						data_count++;
					}
					else 
					{
						returned_data[i] = strtod(p, &pp);
						data_count++;
					}

					if (*pp == ',') 
					{
						p = pp + 1;
					}
					else 
					{
						break;
					}
				}
				return data_count;
			}
		}
		return 0;
	}

private:

	rclcpp::Service<interfaces::srv::ImuReset>::SharedPtr euler_angle_reset_srv_;

	bool Euler_angle_reset_callback(
		const std::shared_ptr<interfaces::srv::ImuReset::Request> request, 
		const std::shared_ptr<interfaces::srv::ImuReset::Response> response)
	{
		bool bResult = false;
		double dSend_Data[10];
		SendRecv("ra\n", dSend_Data, 10);
		bResult = true;
		response->result = bResult;
		return true;
	}


};


int main (int argc, char** argv)
{
	rclcpp::init(argc, argv);
	// Create a function for when messages are to be sent.
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	auto node = std::make_shared<IAHRS>();
	IAHRS iahrs;

	// These values do not need to be converted
	iahrs.imu_data_msg.linear_acceleration_covariance[0] = 0.0064;
	iahrs.imu_data_msg.linear_acceleration_covariance[4] = 0.0063;
	iahrs.imu_data_msg.linear_acceleration_covariance[8] = 0.0064;
	iahrs.imu_data_msg.angular_velocity_covariance[0] = 0.032*(M_PI/180.0);
	iahrs.imu_data_msg.angular_velocity_covariance[4] = 0.028*(M_PI/180.0);
	iahrs.imu_data_msg.angular_velocity_covariance[8] = 0.006*(M_PI/180.0);
	iahrs.imu_data_msg.orientation_covariance[0] = 0.013*(M_PI/180.0);
	iahrs.imu_data_msg.orientation_covariance[4] = 0.011*(M_PI/180.0);
	iahrs.imu_data_msg.orientation_covariance[8] = 0.006*(M_PI/180.0);


	rclcpp::WallRate loop_rate(100);
	iahrs.serial_open();
	iahrs.SendRecv("za\n", dSend_Data, 10);	// Euler Angle -> '0.0' Reset
	usleep(10000);
	printf("                       | Z axis \n");
	printf("                       | \n");
	printf("                       |   / X axis \n");
	printf("                   ____|__/____ \n");
	printf("      Y axis     / *   | /    /| \n");
	printf("      _________ /______|/    // \n");
	printf("               /___________ // \n");
	printf("              |____iahrs___|/ \n");

	while(rclcpp::ok())
    	{
		rclcpp::spin_some(node);
		if (serial_fd >= 0) 
		{
			const int max_data = 10;
			double data[max_data];
			int no_data = 0;
			no_data = iahrs.SendRecv("g\n", data, max_data);	// Read angular_velocity _ wx, wy, wz 
			if (no_data >= 3) 
			{
				// angular_velocity
				iahrs.imu_data_msg.angular_velocity.x = _pIMU_data.dAngular_velocity_x = data[0]*(M_PI/180.0);
				iahrs.imu_data_msg.angular_velocity.y = _pIMU_data.dAngular_velocity_y = data[1]*(M_PI/180.0);
				iahrs.imu_data_msg.angular_velocity.z = _pIMU_data.dAngular_velocity_z = data[2]*(M_PI/180.0);
			}
			no_data = iahrs.SendRecv("a\n", data, max_data);	// Read linear_acceleration 	unit: m/s^2
			if (no_data >= 3) 
			{
				//// linear_acceleration   g to m/s^2
				iahrs.imu_data_msg.linear_acceleration.x = _pIMU_data.dLinear_acceleration_x = data[0] * 9.80665;
				iahrs.imu_data_msg.linear_acceleration.y = _pIMU_data.dLinear_acceleration_y = data[1] * 9.80665;
				iahrs.imu_data_msg.linear_acceleration.z = _pIMU_data.dLinear_acceleration_z = data[2] * 9.80665;
			}
			no_data = iahrs.SendRecv("e\n", data, max_data);	// Read Euler angle
			if (no_data >= 3) 
			{
				// Euler _ rad
				_pIMU_data.dEuler_angle_Roll  = data[0]*(M_PI/180.0);
				_pIMU_data.dEuler_angle_Pitch = data[1]*(M_PI/180.0);
				_pIMU_data.dEuler_angle_Yaw	  = data[2]*(M_PI/180.0);
			}

			tf2::Quaternion q;
			q.setRPY(_pIMU_data.dEuler_angle_Roll , _pIMU_data.dEuler_angle_Pitch, _pIMU_data.dEuler_angle_Yaw);
			// orientation
			iahrs.imu_data_msg.orientation.x = q.x();
			iahrs.imu_data_msg.orientation.y = q.y();
			iahrs.imu_data_msg.orientation.z = q.z();
			iahrs.imu_data_msg.orientation.w = q.w();
			iahrs.imu_data_msg.header.stamp = node->now();
			iahrs.imu_data_msg.header.frame_id = "imu_link";

			// publish the IMU data
			iahrs.imu_data_pub->publish(iahrs.imu_data_msg);

			if(m_bSingle_TF_option)
			{
				// Update the timestamp of the transform
				iahrs.transformStamped.header.stamp = node->now();
				iahrs.transformStamped.header.frame_id = "base_link";   // Parent frame ID
				iahrs.transformStamped.child_frame_id = "imu_link";       // IMU frame ID
				// Set the transformation translation (position)
				iahrs.transformStamped.transform.translation.x = 0.0;
				iahrs.transformStamped.transform.translation.y = 0.0;
				iahrs.transformStamped.transform.translation.z = 0.2;
				iahrs.transformStamped.transform.rotation.x = q.x();
				iahrs.transformStamped.transform.rotation.y = q.y();
				iahrs.transformStamped.transform.rotation.z = q.z();
				iahrs.transformStamped.transform.rotation.w = q.w();
				// Publish the transform
				iahrs.tf_broadcaster->sendTransform(iahrs.transformStamped);
			}
		}

        loop_rate.sleep();
    }

	close (serial_fd);
	rclcpp::shutdown();

    return 0;
}
