#include "cyberpod_sim_ros/sensor_node.hpp"


using namespace Eigen;

ros::NodeHandle *nh_;
ros::NodeHandle *nhParams_;
ros::ServiceServer srv_ui_;
ros::Subscriber sub_state_;
ros::Publisher pub_sensor_;
ros::Publisher pub_state_measured_; 

double enc_v_variance_, theta_error_, theta_dot_error_, theta_dot_variance_;
cyberpod_sim_ros::state state_current_;
double dt_,v_last_,theta_last_;
double encL_pos_,encR_pos_;
double Rw_ = 0.195;
double L_ = 0.5;

void stateCallback(const cyberpod_sim_ros::state::ConstPtr msg)
{
	// Receive True State (no noise)
	state_current_ = *msg;

	cyberpod_sim_ros::state state_msg = state_current_;

	double thetaDot = state_current_.thetaDot;

	// add in random noise
	
	state_msg.x += ( ((double)rand()/(double)RAND_MAX) *2-1)*enc_v_variance_;
	state_msg.v += ( ((double)rand()/(double)RAND_MAX) *2-1)*enc_v_variance_;
	state_msg.psi += theta_error_; //( ((double)rand()/(double)RAND_MAX) *2-1)*psi_variance_;
	state_msg.psiDot += theta_dot_error_;//( ((double)rand()/(double)RAND_MAX) *2-1)*psi_dot_variance_;

	state_msg.stateVec[0] = state_msg.x; 
	state_msg.stateVec[3] = state_msg.v;
	state_msg.stateVec[5] = state_msg.psi;
	state_msg.stateVec[6] = state_msg.psiDot; 
	
	// publish measured state
	pub_sensor_.publish(state_msg);
}

int main (int argc, char *argv[])
{
	// Init ros
	ros::init(argc,argv,"sensor");

	// Instanciate NodeHandles
	nhParams_ = new ros::NodeHandle("~");
	nh_ = new ros::NodeHandle();

	// Init pubs, subs and srvs
	sub_state_ = nh_->subscribe<cyberpod_sim_ros::state>("state_true", 1, stateCallback);
	pub_sensor_ = nh_->advertise<cyberpod_sim_ros::state>("state_measured", 1);
	//pub_sensor_ = nh_->advertise<cyberpod_sim_ros::sensor>("sensor", 1);

	// Retreive params
	double variance = 0.0; //0.001;
	nhParams_->param<double>("enc_v_variance",enc_v_variance_, variance);//0.001);
	nhParams_->param<double>("theta_error",theta_error_, 0.0); //0.001);
	nhParams_->param<double>("theta_dot_error",theta_dot_error_, 0.0); //0.001);
	nhParams_->param<double>("theta_dot_variance",theta_dot_variance_,0.001);

	// Display node info
	ROS_INFO("Sensor node successfuly started with:");
	ROS_INFO("___theta_error=%4f", theta_error_);
	ROS_INFO("___theta_dot_error=%4f", theta_dot_error_);
	ros::spin();

	return 0;
}
