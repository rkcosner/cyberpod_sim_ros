#include "cyberpod_sim_ros/converter_node.hpp" 

using namespace Eigen;

ros::NodeHandle *nh_;
ros::NodeHandle *nhParams_;
ros::Publisher pub_input_;
ros::Publisher pub_state_;
ros::Subscriber sub_ctrl_info_;
ros::Subscriber sub_state_hardware_;

cyberpod_sim_ros::state stateCurrent_;
cyberpod_sim_ros::input inputCurrent_;
cyberpod_sim_ros::statehardware stateHardware_;
cyberpod_sim_ros::ctrlinfo ctrl_info_;

void sendTransformCurrent(void)
{
	static tf::TransformBroadcaster odom_broadcaster;
	geometry_msgs::TransformStamped odom_trans;
	odom_trans.header.stamp = ros::Time::now();
	odom_trans.header.frame_id = "world";
	odom_trans.child_frame_id = "cyberpod/base_link";

	odom_trans.transform.translation.x = stateCurrent_.x;
	odom_trans.transform.translation.y = stateCurrent_.y;
	odom_trans.transform.translation.z = 0.195;
	
	Quaterniond cyberpod_q;
	Vector3d cyberpod_eul(0.0,stateCurrent_.psi,stateCurrent_.theta);
	eul2quatZYX(cyberpod_eul,cyberpod_q);

	geometry_msgs::Quaternion odom_quat;
	odom_quat.w = cyberpod_q.w();
	odom_quat.x = cyberpod_q.x();
	odom_quat.y = cyberpod_q.y();
	odom_quat.z = cyberpod_q.z();
	odom_trans.transform.rotation = odom_quat;

	odom_broadcaster.sendTransform(odom_trans);
}

void stateCallback(const cyberpod_sim_ros::statehardware::ConstPtr msg)
{
	stateCurrent_.status = static_cast<uint8_t>(2);
	stateCurrent_.time = msg->time;
	stateCurrent_.x = msg->state[0];
	stateCurrent_.y = msg->state[1];
	stateCurrent_.theta = msg->state[2];
	stateCurrent_.v = msg->state[3];
	stateCurrent_.thetaDot = msg->state[4];
	stateCurrent_.psi = msg->state[5];
	stateCurrent_.psiDot = msg->state[6];
	std::copy(msg->state.begin(),msg->state.end(),stateCurrent_.stateVec.begin());

	stateCurrent_.header.stamp = ros::Time::now();
	stateCurrent_.header.frame_id = "lame";
	
	pub_state_.publish(stateCurrent_);
	// sendTransformCurrent();
}

void ctrlCallback(const cyberpod_sim_ros::ctrlinfo::ConstPtr msg)
{
	inputCurrent_.status = static_cast<uint8_t>(2);
	inputCurrent_.header.stamp = ros::Time::now();
	inputCurrent_.header.frame_id = "lame";
	inputCurrent_.inputVec[0] = msg->data[1];
	inputCurrent_.inputVec[1] = msg->data[1];

	pub_input_.publish(inputCurrent_);
}

int main (int argc, char *argv[])
{
	// Init ros
	ros::init(argc,argv,"converter");

	// Create useless broadcaster so that transforms get published
	tf::TransformBroadcaster useless_broadcaster;

	// Instanciate NodeHandles
	nhParams_ = new ros::NodeHandle("~");
	nh_ = new ros::NodeHandle();

	// Init pubs, subs and srvs
	sub_ctrl_info_ = nh_->subscribe<cyberpod_sim_ros::ctrlinfo>("/cyberpod/ctrl_info", 1,ctrlCallback);
	sub_state_hardware_ = nh_->subscribe<cyberpod_sim_ros::statehardware>("/cyberpod/state", 1,stateCallback);
	pub_state_ = nh_->advertise<cyberpod_sim_ros::state>("/cyberpod_sim_ros/state", 1);
	pub_input_ = nh_->advertise<cyberpod_sim_ros::input>("/cyberpod_sim_ros/inputDes", 1);
	// Display node info
	ROS_INFO("Converter node successfuly started with:");

	ros::spin();

	return 0;
}