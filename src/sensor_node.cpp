#include "cyberpod_sim_ros/sensor_node.hpp"


using namespace Eigen;

ros::NodeHandle *nh_;
ros::NodeHandle *nhParams_;
ros::ServiceServer srv_ui_;
ros::Subscriber sub_state_;
ros::Publisher pub_sensor_;
ros::Publisher pub_state_measured_; 

double enc_v_variance_,psi_error_,psi_dot_error_,theta_dot_variance_;
cyberpod_sim_ros::state state_current_;
double dt_,v_last_,theta_last_;
double encL_pos_,encR_pos_;
double Rw_ = 0.195;
double L_ = 0.5;

void stateCallback(const cyberpod_sim_ros::state::ConstPtr msg)
{
	//compute sensor values (no noise)
	state_current_ = *msg;
	double encoderL_vel = (state_current_.v - state_current_.thetaDot*L_)/Rw_;
	double encoderR_vel = (state_current_.v + state_current_.thetaDot*L_)/Rw_;

	double thetaDot = state_current_.thetaDot;

	double psi = state_current_.psi;
	double psiDot = state_current_.psiDot;

	double steering = 0.;
	double battV = 60.;

	// add in random noise
	encoderL_vel += ( ((double)rand()/(double)RAND_MAX) *2-1)*enc_v_variance_;
	encoderR_vel += ( ((double)rand()/(double)RAND_MAX) *2-1)*enc_v_variance_;
	psi += psi_error_;
	psiDot += psi_dot_error_;
	thetaDot += ( ((double)rand()/(double)RAND_MAX) *2-1)*theta_dot_variance_;

	//update encoder positions;
	encL_pos_ += encoderL_vel*dt_;
	encR_pos_ += encoderR_vel*dt_;

	cyberpod_sim_ros::sensor sensor_msg;

	sensor_msg.time = (uint32_t) (state_current_.time*1E6);
	sensor_msg.data.push_back(encL_pos_);
	sensor_msg.data.push_back(encR_pos_);
	sensor_msg.data.push_back(encoderL_vel);
	sensor_msg.data.push_back(encoderR_vel);
	sensor_msg.data.push_back(thetaDot);
	sensor_msg.data.push_back(psi);
	sensor_msg.data.push_back(psiDot);
	sensor_msg.data.push_back(steering);
	sensor_msg.data.push_back(battV);

	pub_sensor_.publish(sensor_msg);
}

void stateCallbackNoEKF(const cyberpod_sim_ros::state::ConstPtr msg)
{
	// Receive True State (no noise)
	state_current_ = *msg;

	cyberpod_sim_ros::state state_msg = state_current_;

	double thetaDot = state_current_.thetaDot;

	// add in random noise
	
	state_msg.x += ( ((double)rand()/(double)RAND_MAX) *2-1)*enc_v_variance_;
	state_msg.v += ( ((double)rand()/(double)RAND_MAX) *2-1)*enc_v_variance_;
	state_msg.psi += psi_error_; //( ((double)rand()/(double)RAND_MAX) *2-1)*psi_variance_;
	state_msg.psiDot += psi_dot_error_;//( ((double)rand()/(double)RAND_MAX) *2-1)*psi_dot_variance_;

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
	sub_state_ = nh_->subscribe<cyberpod_sim_ros::state>("state_true", 1, stateCallbackNoEKF);
	pub_sensor_ = nh_->advertise<cyberpod_sim_ros::state>("state_measured", 1);
	//pub_sensor_ = nh_->advertise<cyberpod_sim_ros::sensor>("sensor", 1);

	// Retreive params
	double variance = 0.0; //0.001;
	nhParams_->param<double>("enc_v_variance",enc_v_variance_, variance);//0.001);
	nhParams_->param<double>("psi_error",psi_error_, 0.0); //0.001);
	nhParams_->param<double>("psi_dot_error",psi_dot_error_, 0.0); //0.001);
	nhParams_->param<double>("theta_dot_variance",theta_dot_variance_,0.001);

	// Display node info
	ROS_INFO("Sensor node successfuly started!");

	ros::spin();

	return 0;
}
