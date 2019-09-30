#include "cyberpod_sim_ros/safety_filter_node.hpp"

ros::NodeHandle *nh_;
ros::NodeHandle *nhParams_;

ros::Subscriber sub_cmdDes_;
ros::Subscriber sub_state_;
ros::Subscriber sub_obstacle_;
ros::Subscriber sub_velodyne_;

ros::Publisher pub_cmd_;
ros::Publisher pub_info_;
ros::Publisher pub_backupTraj_;
ros::Publisher cmdDesVizu_pub_;
ros::Publisher cmdVizu_pub_;
ros::Publisher closestPointVizu_pub_;

Vector3d vDesVec_(0.0,0.0,0.0);
Vector3d eulVec_(0.0,0.0,0.0);
Quaterniond quatVec_(1.0,0.0,0.0,0.0);
Vector3d closestPointWorldCurr_;
Vector3d closestPointWorldCurrSmooth_;
double closestPointDist_;
double closestPointDistSmooth_;
Vector3d vDesVecSmooth_(0.0,0.0,0.0);

Matrix3d rotBody2World_ = Matrix3d::Identity(3,3);

cyberpod_sim_ros::cmd cmdDes_;
cyberpod_sim_ros::cmd cmdAct_;
cyberpod_sim_ros::state stateCurrent_;
cyberpod_sim_ros::filterInfo filter_info_;

visualization_msgs::Marker cmdDesVizu_;
visualization_msgs::Marker cmdVizu_;
visualization_msgs::Marker closestPointVizu_;
visualization_msgs::Marker coneVizu_;

sensor_msgs::PointCloud2 obstaclesMsg_;
pcl::PointCloud<pcl::PointXYZ> obstaclesPointCloud_;

uint32_t iterInput_;
uint32_t iterState_;
uint32_t iterObstacles_;
double backup_Tmax_;
double terminalVelMax_;
int32_t passTrough_;
double tau_safety_;
double tau_backup_;
double safety_buffer_;
double safety_buffer_soft_;
double tNm1 = -1.0;
double dtCurrent_;
double smoothing_tau_vDes_;
double smoothing_tau_obstacle_;
double cone_radius_;
double cone_lat_, cone_long_;

Vector3d posOffest_(0.0,0.0,0.0);

double integration_dt_;
uint32_t integration_steps_;

std::vector<std::pair<double,state_t>> backTraj_;
std::vector<double> hSafetyList_;
std::vector<double> hBackupList_;
uint32_t closestPointInPCLcurrent_ = 0;
uint32_t closestPointInPCLbackup_ = 0;

nav_msgs::Path backTrajMsg_;

bool pclInitialized = false;
bool closestPointInitialized = false;

// Controller gains
double KiVz_;
double KpVz_;
double KpVxy_;
double KpAttitude_;
double KdAttitude_;
double KpOmegaz_;
double maxInclination_;
double hoverThrust_;
double uzInt_;

CyberTimer<10000> timerDistanceToPCL;
CyberTimer<1> timerDynamics;
CyberTimer<1> timerBackupCtrl;
CyberTimer<1000> timerIntegration;
CyberTimer<1000> timerRotatePCL;

uint32_t safetySet(const double x[STATE_LENGTH], 
                         double &h)
{
	h = 0.0;
	uint32_t idxDistMin = -1;
	if(pclInitialized && obstaclesPointCloud_.size()>0)
	{
		timerDistanceToPCL.tic();
		double dx, dy, dz, dist;
		h = std::numeric_limits<double>::infinity();
		
		for(uint32_t i = 0; i<obstaclesPointCloud_.size(); i++)
		{
			dx = obstaclesPointCloud_.points[i].x - x[0];
			dy = obstaclesPointCloud_.points[i].y - x[1];
			dz = obstaclesPointCloud_.points[i].z - x[2];
			dist = sqrt(dx*dx + dy*dy + dz*dz);
			if(dist < h)
			{
				h = dist;
				idxDistMin = i;
			}
		}
		timerDistanceToPCL.toc();
	}
	return idxDistMin;
}

void backupSet(const double x[STATE_LENGTH], 
                     double &h)
{
	const Vector3d v(x[7],x[8],x[9]);
	const Vector3d omega(x[10],x[11],x[12]);
	const Vector4d q(x[3],x[4],x[5],x[6]);
	const Vector4d qUnit(1.0,0.0,0.0,0.0);
	filter_info_.vBackup = v.norm();
	const double tmp = (terminalVelMax_*terminalVelMax_);
	h = 1 - v.squaredNorm()/tmp; - omega.squaredNorm()/tmp - (q-qUnit).squaredNorm()/tmp;
}

void backupController(const double t,
                      const double x[STATE_LENGTH],
                            double u[INPUT_LENGTH],
                            double vDes[CMD_LENGTH])
{
	timerBackupCtrl.tic();

	//Compute orientations from quaternion
	Quaterniond q(x[3],x[4],x[5],x[6]);

	Matrix3d rotBody2World;
	quat2rotm(q,rotBody2World);

	Vector3d eul;
	quat2eulZYX(q,eul);
	Vector3d zBodyInWorld = rotBody2World*(Vector3d() << 0, 0, 1).finished();

	// Compute body frame velocity
	Vector3d v(x[7],x[8],x[9]);
	Vector3d vWorldNoYaw = rotz(-eul(2))*v;

	// Compute desired velocity
	Vector3d vDesVec(0.0,0.0,0.0);
	double yawDotDes = 0.0;
	if(pclInitialized && obstaclesPointCloud_.size()>closestPointInPCLcurrent_)
	{
		// yawDotDes = cmdDes_.vDes[3];
		
		Vector3d closestPointWorld(obstaclesPointCloud_.points[closestPointInPCLcurrent_].x - x[0],
		                           obstaclesPointCloud_.points[closestPointInPCLcurrent_].y - x[1],
		                           obstaclesPointCloud_.points[closestPointInPCLcurrent_].z - x[2]);
		Vector3d closestPointUAV = rotz(-eul(2))*closestPointWorld;
		const double distToPCL = closestPointUAV.norm();
		closestPointUAV.normalize();

		const double vMaxSafety = 1.0;
		const double maxProjection = 2*vMaxSafety*(distToPCL - (safety_buffer_+0.5*safety_buffer_soft_))/safety_buffer_soft_;
		const double vDesDotPcl = closestPointUAV.dot(vDesVec);

		if(vDesDotPcl>maxProjection)
		{
			vDesVec = vDesVec + (maxProjection - vDesDotPcl)*closestPointUAV;
		}
	}

	vDes[0] = vDesVec(0);
	vDes[1] = vDesVec(1);
	vDes[2] = vDesVec(2);
	vDes[3] = yawDotDes;

	// Compute error
	const double vxError = saturate(KpVxy_*(vDesVec(0) - vWorldNoYaw(0)),-deg2rad(maxInclination_),deg2rad(maxInclination_));
	const double vyError = saturate(KpVxy_*(vDesVec(1) - vWorldNoYaw(1)),-deg2rad(maxInclination_),deg2rad(maxInclination_));
	const double vzError = vDesVec(2) - vWorldNoYaw(2);

	const double rollError = -vyError-eul(0);
	const double pitchError = vxError-eul(1);
	const double rollErrorDot = -x[10];
	const double pitchErrorDot = -x[11];
	const double yawErrorDot = yawDotDes - x[12];

	// Compute vz PI output
	const double uz = saturate(KpVz_*vzError + uzInt_/zBodyInWorld(2),hoverThrust_-0.3,hoverThrust_+0.3);
	const double uroll = saturate(KpAttitude_*rollError + KdAttitude_*rollErrorDot,-0.5,0.5);
	const double upitch = saturate(KpAttitude_*pitchError + KdAttitude_*pitchErrorDot,-0.5,0.5);
	const double uomegaz = saturate(KpOmegaz_*yawErrorDot,-0.2,0.2);

	// Fill up message 
	u[0] = uz - uroll - upitch - uomegaz;
	u[1] = uz - uroll + upitch + uomegaz;
	u[2] = uz + uroll + upitch - uomegaz;
	u[3] = uz + uroll - upitch + uomegaz;

	// Clamp inputs
	saturateInPlace(u,0.0,1.0,INPUT_LENGTH);

	timerBackupCtrl.toc();
}

void dynamicsCL(const state_t &x,
                      state_t &xDot,
                const double t)
{
	xDot = x;
	double inputBackup[INPUT_LENGTH];
	double cmdBackup[CMD_LENGTH];
	backupController(t,x.data(),inputBackup,cmdBackup);

	timerDynamics.tic();
   uavDynamics(x.data(),inputBackup,xDot.data(),t);
   timerDynamics.toc();
}

void regulationMapSelection(const double &hBackup,
                            const double &hSafety,
                            const double vBak[CMD_LENGTH])
{
	double lambdaBackup = 1-exp(-hBackup/tau_backup_);
	double lambdaSafety = 1-exp(-hSafety/tau_safety_);
	saturateInPlace(lambdaBackup,0.0,1.0);
	saturateInPlace(lambdaSafety,0.0,1.0);
	double lamda = lambdaBackup*lambdaSafety;

	filter_info_.lambdaBackup = lambdaBackup;
	filter_info_.lambdaSafety = lambdaSafety;
	filter_info_.lamda = lamda;
	
	Vector3d closestPointUAV = rotz(-eulVec_(2))*closestPointWorldCurrSmooth_;
	closestPointUAV.normalize();
	const double distToPCL = closestPointDistSmooth_;
	Vector3d vDesVec = vDesVecSmooth_;

	const double vMaxSafety = 5.0;
	const double maxProjection = 2*vMaxSafety*(distToPCL - (safety_buffer_+safety_buffer_soft_))/(2.0*safety_buffer_soft_);
	const double vDesDotPcl = closestPointUAV.dot(vDesVec);

	if(vDesDotPcl>maxProjection)
	{
		vDesVec = vDesVec + (maxProjection - vDesDotPcl)*closestPointUAV;
	}

	std::array<double,CMD_LENGTH> cmdDes{vDesVec(0),vDesVec(1),vDesVec(2)};
	cmdDes[CMD_LENGTH-1] = cmdDes_.vDes[CMD_LENGTH-1];

	cyberpod_sim_ros::cmd cmdActTmp;
	for(uint32_t i=0; i<CMD_LENGTH; i++)
	{
		cmdActTmp.vDes[i] = lamda*cmdDes[i] + (1.0-lamda)*vBak[i];
		filter_info_.vDesRaw[i] = cmdDes_.vDes[i];
		filter_info_.vDes[i] = cmdDes[i];
		filter_info_.vBak[i] = vBak[i];
		filter_info_.vAct[i] = cmdActTmp.vDes[i];
	}

	if(!passTrough_)
	{
		cmdAct_.vDes = cmdActTmp.vDes;
	}

	const Vector3d vActWorldVec = rotz(eulVec_(2))*Vector3d(cmdActTmp.vDes[0],
	                                                         cmdActTmp.vDes[1],
	                                                         cmdActTmp.vDes[2]);

	const Vector3d vDesWorldVec = rotz(eulVec_(2))*vDesVecSmooth_;

	cmdDesVizu_.points[0].x = stateCurrent_.x;
	cmdDesVizu_.points[0].y = stateCurrent_.y;
	cmdDesVizu_.points[0].z = stateCurrent_.z;
	cmdDesVizu_.points[1].x = stateCurrent_.x + vDesWorldVec(0);
	cmdDesVizu_.points[1].y = stateCurrent_.y + vDesWorldVec(1);
	cmdDesVizu_.points[1].z = stateCurrent_.z + vDesWorldVec(2);

	cmdVizu_.points[0].x = stateCurrent_.x;
	cmdVizu_.points[0].y = stateCurrent_.y;
	cmdVizu_.points[0].z = stateCurrent_.z;
	cmdVizu_.points[1].x = stateCurrent_.x + vActWorldVec(0);
	cmdVizu_.points[1].y = stateCurrent_.y + vActWorldVec(1);
	cmdVizu_.points[1].z = stateCurrent_.z + vActWorldVec(2);
}

void filterInput(void)
{
	iterInput_ = cmdDes_.header.seq;
	cmdAct_.header.seq = iterInput_;
	cmdAct_.header.stamp = ros::Time::now();
	cmdAct_.header.frame_id = std::to_string(iterState_)+std::string("|")+std::to_string(iterObstacles_);


	// Compute initial conditions
	state_t x0BackupTraj{stateCurrent_.x-posOffest_(0),
	                     stateCurrent_.y-posOffest_(1),
	                     stateCurrent_.z-posOffest_(2),
	                     stateCurrent_.qw,
	                     stateCurrent_.qx,
	                     stateCurrent_.qy,
	                     stateCurrent_.qz,
	                     stateCurrent_.vx,
	                     stateCurrent_.vy,
	                     stateCurrent_.vz,
	                     stateCurrent_.omegax,
	                     stateCurrent_.omegay,
	                     stateCurrent_.omegaz,
	                     stateCurrent_.omega1,
	                     stateCurrent_.omega2,
	                     stateCurrent_.omega3,
	                     stateCurrent_.omega4};
	                     
	double tmp;	                     
	closestPointInPCLcurrent_ = safetySet(x0BackupTraj.data(), tmp);
	double uBackup[INPUT_LENGTH];
	double vBackup[CMD_LENGTH];
	backupController(0.0,x0BackupTraj.data(),uBackup,vBackup);

	if(passTrough_)
		cmdAct_.vDes = cmdDes_.vDes;
	else
		std::copy(std::begin(vBackup),std::end(vBackup),cmdAct_.vDes.begin());

	// Don't perform filtering if pointcloud not readu
	if(!pclInitialized)
		return;

	timerIntegration.tic();
	// Initialize variables before integration
	stepper_t stepper;
	auto itBegin = make_n_step_iterator_begin(stepper, dynamicsCL, x0BackupTraj, 0.0, integration_dt_, integration_steps_);
	auto itEnd = make_n_step_iterator_end(stepper, dynamicsCL, x0BackupTraj);
	backTrajMsg_.poses.resize(0);
	geometry_msgs::PoseStamped pose;
	pose.header.frame_id = "/world";
	uint32_t iBT = 0;
	double hSafetyMin = std::numeric_limits<double>::infinity();
	closestPointInPCLbackup_ = 0;
	uint32_t hSafetyIdx = 0;
	uint32_t idxToSafety = -1;
	uzInt_ = hoverThrust_;

	// Integrate backup trajectory
	for(auto it = itBegin; it!=itEnd; it++)
	{
		// Fill up backup trajectory vector
		backTraj_[iBT].first = integration_dt_*static_cast<double>(iBT);
		copy(it->begin(), it->end(), backTraj_[iBT].second.begin());

		uzInt_ -= integration_dt_*KiVz_*backTraj_[iBT].second[9];
		saturateInPlace(uzInt_,hoverThrust_-0.1,hoverThrust_+0.1);

		// Compute backup and safety set
		backupSet(backTraj_[iBT].second.data(), hBackupList_[iBT]);
		closestPointInPCLcurrent_ = safetySet(backTraj_[iBT].second.data(), hSafetyList_[iBT]);
		if(hSafetyList_[iBT]<hSafetyMin)
		{
			hSafetyMin = hSafetyList_[iBT];
			hSafetyIdx = iBT;
			closestPointInPCLbackup_ = closestPointInPCLcurrent_;
		}
		if(idxToSafety == -1 && hBackupList_[iBT]>=0)
			idxToSafety = iBT;

		// Fill up backup trajectory message
		pose.pose.position.x = backTraj_[iBT].second[0] + posOffest_(0);
		pose.pose.position.y = backTraj_[iBT].second[1] + posOffest_(1);
		pose.pose.position.z = backTraj_[iBT].second[2] + posOffest_(2);
		pose.pose.orientation.w = backTraj_[iBT].second[3];
		pose.pose.orientation.x = backTraj_[iBT].second[4];
		pose.pose.orientation.y = backTraj_[iBT].second[5];
		pose.pose.orientation.z = backTraj_[iBT].second[6];
		backTrajMsg_.poses.push_back(pose);

		iBT++;
	}

	timerIntegration.toc();

	hoverThrust_ = uzInt_;

	// If backup set not reached set final point as idxToSafety
	if(idxToSafety==-1)
		idxToSafety = iBT-1;

	if(closestPointInPCLcurrent_!=-1)
	{
		closestPointWorldCurr_(0) = obstaclesPointCloud_.points[closestPointInPCLbackup_].x - stateCurrent_.x + posOffest_(0);
		closestPointWorldCurr_(1) = obstaclesPointCloud_.points[closestPointInPCLbackup_].y - stateCurrent_.y + posOffest_(1);
		closestPointWorldCurr_(2) = obstaclesPointCloud_.points[closestPointInPCLbackup_].z - stateCurrent_.z + posOffest_(2);

		const Vector3d closestPointUavPos(backTraj_[hSafetyIdx].second[0],backTraj_[hSafetyIdx].second[1],backTraj_[hSafetyIdx].second[2]);
		closestPointDist_ = (closestPointWorldCurr_ - closestPointUavPos).norm();

		if(!closestPointInitialized)
		{
			closestPointInitialized = true;
			closestPointWorldCurrSmooth_ = closestPointWorldCurr_;
			closestPointDistSmooth_ = closestPointDist_;
			vDesVecSmooth_ = vDesVec_;
		}
		else
		{
			if(smoothing_tau_obstacle_<=0.0)
			{
				closestPointWorldCurrSmooth_= closestPointWorldCurr_;
				closestPointDistSmooth_ = closestPointDist_;
			}
			else
			{
				closestPointWorldCurrSmooth_+= dtCurrent_*(closestPointWorldCurr_ - closestPointWorldCurrSmooth_)/smoothing_tau_obstacle_;
				closestPointDistSmooth_ += dtCurrent_*(closestPointDist_ - closestPointDistSmooth_)/smoothing_tau_obstacle_;
			}
			if(smoothing_tau_vDes_<=0.0)
				vDesVecSmooth_ = vDesVec_;
			else
				vDesVecSmooth_ += dtCurrent_*(vDesVec_ - vDesVecSmooth_)/smoothing_tau_vDes_;
		}

		closestPointVizu_.points[0].x = stateCurrent_.x;
		closestPointVizu_.points[0].y = stateCurrent_.y;
		closestPointVizu_.points[0].z = stateCurrent_.z;
		closestPointVizu_.points[1].x = stateCurrent_.x + closestPointWorldCurrSmooth_(0);
		closestPointVizu_.points[1].y = stateCurrent_.y + closestPointWorldCurrSmooth_(1);
		closestPointVizu_.points[1].z = stateCurrent_.z + closestPointWorldCurrSmooth_(2);
	}



	double hBackup = hBackupList_[iBT-1];
	hSafetyMin = hSafetyMin - safety_buffer_;
	double tBackup = backTraj_[idxToSafety].first;
	regulationMapSelection(backup_Tmax_ - tBackup,hSafetyMin,vBackup);

	filter_info_.tBackup = tBackup;
	filter_info_.hBackup = hBackup;
	filter_info_.hSafety = hSafetyMin;
	filter_info_.tSafety = backTraj_[hSafetyIdx].first;
	filter_info_.hCurrent = hSafetyList_[0];
	
	cmdAct_.status = static_cast<uint8_t>(STATUS::RUNNING);
}

void inputCallback(const cyberpod_sim_ros::cmd::ConstPtr msg)
{
	cmdDes_ = *msg;
	vDesVec_(0) = cmdDes_.vDes[0];
	vDesVec_(1) = cmdDes_.vDes[1];
	vDesVec_(2) = cmdDes_.vDes[2];		
}

void stateCallback(const cyberpod_sim_ros::state::ConstPtr msg)
{
	stateCurrent_ = *msg;
	iterState_ = stateCurrent_.header.seq;

	quatVec_ = Quaterniond(stateCurrent_.qw,
	                       stateCurrent_.qx,
	                       stateCurrent_.qy,
	                       stateCurrent_.qz);
	quat2eulZYX(quatVec_,eulVec_);
	quat2rotm(quatVec_,rotBody2World_);

	if(tNm1<0)
		tNm1 = stateCurrent_.time;
	else
	{
		dtCurrent_ = stateCurrent_.time - tNm1;
		tNm1 = stateCurrent_.time;
	}

	filterInput();
	pub_cmd_.publish(cmdAct_);
	pub_info_.publish(filter_info_);
	pub_backupTraj_.publish(backTrajMsg_);
	cmdDesVizu_pub_.publish(cmdDesVizu_);
	cmdVizu_pub_.publish(cmdVizu_);
	closestPointVizu_pub_.publish(closestPointVizu_);
	closestPointVizu_pub_.publish(coneVizu_);
}

void pointcloudCallback(const sensor_msgs::PointCloud2::ConstPtr msg)
{}

void obstacleCallback(const sensor_msgs::PointCloud2::ConstPtr msg)
{
	timerRotatePCL.tic();

	obstaclesMsg_ = *msg;
	iterObstacles_ = obstaclesMsg_.header.seq;

	pcl::PCLPointCloud2 out_cloud;
	pcl_conversions::toPCL(obstaclesMsg_, out_cloud);
	pcl::fromPCLPointCloud2(out_cloud,obstaclesPointCloud_);

	if(iterState_>0)
	{
		Quaterniond q(stateCurrent_.qw,stateCurrent_.qx,stateCurrent_.qy,stateCurrent_.qz);
		Matrix3d rotBody2World;
		quat2rotm(q,rotBody2World);

		for(uint32_t i = 0; i<obstaclesPointCloud_.size(); i++)
		{
			Vector3d pt(obstaclesPointCloud_.points[i].x,
				obstaclesPointCloud_.points[i].y,
				obstaclesPointCloud_.points[i].z+0.07+0.0377);

			pt = rotBody2World*pt;
			obstaclesPointCloud_.points[i].x = pt(0);
			obstaclesPointCloud_.points[i].y = pt(1);
			obstaclesPointCloud_.points[i].z = pt(2);
		}

		posOffest_(0) = stateCurrent_.x;
		posOffest_(1) = stateCurrent_.y;
		posOffest_(2) = stateCurrent_.z;

		if(!pclInitialized)
			pclInitialized = true;

		Vector3d lidarOffset(0.0,0.0,0.07+0.0377);
		lidarOffset = rotBody2World_*lidarOffset;

		coneVizu_.pose.position.x = stateCurrent_.x + lidarOffset(0);
		coneVizu_.pose.position.y = stateCurrent_.y + lidarOffset(1);
		coneVizu_.pose.position.z = stateCurrent_.z + lidarOffset(2);
		coneVizu_.pose.orientation.w = stateCurrent_.qw;
		coneVizu_.pose.orientation.x = stateCurrent_.qx;
		coneVizu_.pose.orientation.y = stateCurrent_.qy;
		coneVizu_.pose.orientation.z = stateCurrent_.qz;
	}


	timerRotatePCL.toc();

	// std::cout << "timerDistanceToPCL: " << timerDistanceToPCL.getAverage()*1.0e6 << "us, count: " << timerDistanceToPCL.getAverageCount() << std::endl;
	// std::cout << "timerDynamics: " << timerDynamics.getAverage()*1.0e6 << "us, count: " << timerDynamics.getAverageCount() << std::endl;
	// std::cout << "timerBackupCtrl: " << timerBackupCtrl.getAverage()*1.0e6 << "us, count: " << timerBackupCtrl.getAverageCount() << std::endl;
	// std::cout << "timerIntegration: " << timerIntegration.getAverage()*1.0e6 << "us, count: " << timerIntegration.getAverageCount() << std::endl;
	// std::cout << "timerRotatePCL: " << timerRotatePCL.getAverage()*1.0e6 << "us, count: " << timerRotatePCL.getAverageCount() << std::endl << std::endl;
}

void createCone(void)
{
	coneVizu_.points[0].x = 0.0;
	coneVizu_.points[0].y = 0.0;
	coneVizu_.points[0].z = 0.0;
	coneVizu_.points[1].x = cone_radius_*cos(cone_lat_)*cos(cone_long_);
	coneVizu_.points[1].y = cone_radius_*cos(cone_lat_)*sin(cone_long_);
	coneVizu_.points[1].z = cone_radius_*sin(cone_lat_);
	coneVizu_.points[2].x = cone_radius_*cos(0.0)*cos(cone_long_);
	coneVizu_.points[2].y = cone_radius_*cos(0.0)*sin(cone_long_);
	coneVizu_.points[2].z = cone_radius_*sin(0.0);

	coneVizu_.points[3].x = 0.0;
	coneVizu_.points[3].y = 0.0;
	coneVizu_.points[3].z = 0.0;
	coneVizu_.points[4].x = cone_radius_*cos(0.0)*cos(cone_long_);
	coneVizu_.points[4].y = cone_radius_*cos(0.0)*sin(cone_long_);
	coneVizu_.points[4].z = cone_radius_*sin(0.0);
	coneVizu_.points[5].x = cone_radius_*cos(-cone_lat_)*cos(cone_long_);
	coneVizu_.points[5].y = cone_radius_*cos(-cone_lat_)*sin(cone_long_);
	coneVizu_.points[5].z = cone_radius_*sin(-cone_lat_);

	coneVizu_.points[6].x = 0.0;
	coneVizu_.points[6].y = 0.0;
	coneVizu_.points[6].z = 0.0;
	coneVizu_.points[7].x = cone_radius_*cos(cone_lat_)*cos(0.0);
	coneVizu_.points[7].y = cone_radius_*cos(cone_lat_)*sin(0.0);
	coneVizu_.points[7].z = cone_radius_*sin(cone_lat_);
	coneVizu_.points[8].x = cone_radius_*cos(cone_lat_)*cos(cone_long_);
	coneVizu_.points[8].y = cone_radius_*cos(cone_lat_)*sin(cone_long_);
	coneVizu_.points[8].z = cone_radius_*sin(cone_lat_);

	coneVizu_.points[9].x = 0.0;
	coneVizu_.points[9].y = 0.0;
	coneVizu_.points[9].z = 0.0;
	coneVizu_.points[10].x = cone_radius_*cos(cone_lat_)*cos(-cone_long_);
	coneVizu_.points[10].y = cone_radius_*cos(cone_lat_)*sin(-cone_long_);
	coneVizu_.points[10].z = cone_radius_*sin(cone_lat_);
	coneVizu_.points[11].x = cone_radius_*cos(cone_lat_)*cos(0.0);
	coneVizu_.points[11].y = cone_radius_*cos(cone_lat_)*sin(0.0);
	coneVizu_.points[11].z = cone_radius_*sin(cone_lat_);

	coneVizu_.points[12].x = 0.0;
	coneVizu_.points[12].y = 0.0;
	coneVizu_.points[12].z = 0.0;
	coneVizu_.points[13].x = cone_radius_*cos(0.0)*cos(-cone_long_);
	coneVizu_.points[13].y = cone_radius_*cos(0.0)*sin(-cone_long_);
	coneVizu_.points[13].z = cone_radius_*sin(0.0);
	coneVizu_.points[14].x = cone_radius_*cos(cone_lat_)*cos(-cone_long_);
	coneVizu_.points[14].y = cone_radius_*cos(cone_lat_)*sin(-cone_long_);
	coneVizu_.points[14].z = cone_radius_*sin(cone_lat_);

	coneVizu_.points[15].x = 0.0;
	coneVizu_.points[15].y = 0.0;
	coneVizu_.points[15].z = 0.0;
	coneVizu_.points[16].x = cone_radius_*cos(-cone_lat_)*cos(-cone_long_);
	coneVizu_.points[16].y = cone_radius_*cos(-cone_lat_)*sin(-cone_long_);
	coneVizu_.points[16].z = cone_radius_*sin(-cone_lat_);
	coneVizu_.points[17].x = cone_radius_*cos(0.0)*cos(-cone_long_);
	coneVizu_.points[17].y = cone_radius_*cos(0.0)*sin(-cone_long_);
	coneVizu_.points[17].z = cone_radius_*sin(0.0);

	coneVizu_.points[18].x = 0.0;
	coneVizu_.points[18].y = 0.0;
	coneVizu_.points[18].z = 0.0;
	coneVizu_.points[19].x = cone_radius_*cos(-cone_lat_)*cos(cone_long_);
	coneVizu_.points[19].y = cone_radius_*cos(-cone_lat_)*sin(cone_long_);
	coneVizu_.points[19].z = cone_radius_*sin(-cone_lat_);
	coneVizu_.points[20].x = cone_radius_*cos(-cone_lat_)*cos(0.0);
	coneVizu_.points[20].y = cone_radius_*cos(-cone_lat_)*sin(0.0);
	coneVizu_.points[20].z = cone_radius_*sin(-cone_lat_);

	coneVizu_.points[21].x = 0.0;
	coneVizu_.points[21].y = 0.0;
	coneVizu_.points[21].z = 0.0;
	coneVizu_.points[22].x = cone_radius_*cos(-cone_lat_)*cos(0.0);
	coneVizu_.points[22].y = cone_radius_*cos(-cone_lat_)*sin(0.0);
	coneVizu_.points[22].z = cone_radius_*sin(-cone_lat_);
	coneVizu_.points[23].x = cone_radius_*cos(-cone_lat_)*cos(-cone_long_);
	coneVizu_.points[23].y = cone_radius_*cos(-cone_lat_)*sin(-cone_long_);
	coneVizu_.points[23].z = cone_radius_*sin(-cone_lat_);
}

int main(int argc, char *argv[])
{

	// Init ros
	ros::init(argc,argv,"safety_filter");

	// Instanciate NodeHandles
	nhParams_ = new ros::NodeHandle("~");
	nh_ = new ros::NodeHandle();

	// Init pubs, subs and srvs
	sub_state_ = nh_->subscribe<cyberpod_sim_ros::state>("uav_state", 1, stateCallback);
	sub_cmdDes_ = nh_->subscribe<cyberpod_sim_ros::cmd>("uav_cmd_des", 1, inputCallback);
	sub_obstacle_ = nh_->subscribe<sensor_msgs::PointCloud2>("/obstacles", 1, obstacleCallback);
	sub_velodyne_ = nh_->subscribe<sensor_msgs::PointCloud2>("/velodyne_points", 1, pointcloudCallback);

	pub_cmd_ = nh_->advertise<cyberpod_sim_ros::cmd>("uav_cmd", 1);
	pub_info_ = nh_->advertise<cyberpod_sim_ros::filterInfo>("safety_filter_info", 1);
	pub_backupTraj_ = nh_->advertise<nav_msgs::Path>("backup_traj", 1);
	cmdDesVizu_pub_ = nh_->advertise<visualization_msgs::Marker>("uav_cmd_des_vizu", 1);
	cmdVizu_pub_ = nh_->advertise<visualization_msgs::Marker>("uav_cmd_vizu", 1);
	closestPointVizu_pub_ = nh_->advertise<visualization_msgs::Marker>("uav_closest_point_vizu", 1);

	// Retreive params
	nhParams_->param<int32_t>("pass_through",passTrough_,0);
	nhParams_->param<double>("integration_dt",integration_dt_,0.01);
	nhParams_->param<double>("backup_Tmax",backup_Tmax_,1.0);
	nhParams_->param<double>("safety_buffer",safety_buffer_,0.3);
	nhParams_->param<double>("safety_buffer_soft",safety_buffer_soft_,0.3);
	nhParams_->param<double>("terminal_vel_max",terminalVelMax_,0.05);
	nhParams_->param<double>("tau_backup",tau_backup_,0.1);
	nhParams_->param<double>("smoothing_tau_obstacle",smoothing_tau_obstacle_,0.1);
	nhParams_->param<double>("smoothing_tau_vDes",smoothing_tau_vDes_,0.1);
	nhParams_->param<double>("cone_radius",cone_radius_,2.0);
	nhParams_->param<double>("cone_latitude",cone_lat_,0.50614548308);
	nhParams_->param<double>("cone_longitude",cone_long_,0.77667151714);

	nhParams_->param<double>("hoverThrust",hoverThrust_,0.52);
	nhParams_->param<double>("KiVz",KiVz_,5.0);
	nhParams_->param<double>("KpVz",KpVz_,1.0);
	nhParams_->param<double>("KpVxy",KpVxy_,0.7);
	nhParams_->param<double>("KpAttitude",KpAttitude_,10.0);
	nhParams_->param<double>("KdAttitude",KdAttitude_,1.0);
	nhParams_->param<double>("KpOmegaz",KpOmegaz_,2.0);
	nhParams_->param<double>("maxInclination",maxInclination_,30.0);

	integration_steps_ = round(backup_Tmax_/integration_dt_);
	tau_safety_ = 0.5*safety_buffer_soft_/3.0;

	// Initialize variables
	iterInput_ = 0;
	iterState_ = 0;
	backTraj_ = std::vector<std::pair<double,state_t>>(integration_steps_+1,std::pair<double,state_t>(0.0,state_t(STATE_LENGTH)));
	hSafetyList_= std::vector<double>(integration_steps_+1,0.0);
	hBackupList_= std::vector<double>(integration_steps_+1,0.0);
	backTrajMsg_.header.frame_id = "/world";
	backTrajMsg_.poses.reserve(integration_steps_+1);
	cmdDesVizu_.header.frame_id = "/world";
	cmdDesVizu_.ns = "cyberpod_sim_ros";
	cmdDesVizu_.points.resize(2);
	cmdDesVizu_.color.a = 1.0;
	cmdDesVizu_.color.r = 0.0;
	cmdDesVizu_.color.g = 1.0;
	cmdDesVizu_.color.b = 0.0;
	cmdDesVizu_.scale.x = 0.05;
	cmdDesVizu_.scale.y = 0.1;
	cmdDesVizu_.scale.z = 0.1;
	cmdDesVizu_.id = 0;
	cmdDesVizu_.type = visualization_msgs::Marker::ARROW;
	cmdDesVizu_.action = visualization_msgs::Marker::ADD;

	cmdVizu_ = cmdDesVizu_;
	cmdVizu_.id = 1;
	cmdVizu_.color.b = 1.0;
	cmdVizu_.color.g = 0.0;

	closestPointVizu_ = cmdDesVizu_;
	closestPointVizu_.id = 2;
	closestPointVizu_.color.r = 1.0;
	closestPointVizu_.color.g = 0.0;

	coneVizu_ = cmdDesVizu_;
	coneVizu_.type = visualization_msgs::Marker::TRIANGLE_LIST;
	coneVizu_.id = 3;
	coneVizu_.points.resize(24);
	coneVizu_.color.r = 1.0;
	coneVizu_.color.g = 0.0;
	coneVizu_.color.a = 0.5;
	coneVizu_.scale.x = 1.0;
	coneVizu_.scale.y = 1.0;
	coneVizu_.scale.z = 1.0;
	createCone();

	// Display node info
	ROS_INFO("Safety Filter node successfuly started with:");
	ROS_INFO("___pass_through=%u",passTrough_);
	ROS_INFO("___integration_dt=%.3f",integration_dt_);
	ROS_INFO("___backup_Tmax=%.3f",backup_Tmax_);
	ROS_INFO("___safety_buffer=%.3f",safety_buffer_);
	ROS_INFO("___safety_buffer_soft=%.3f",safety_buffer_soft_);
	ROS_INFO("___terminal_vel_max=%.3f",terminalVelMax_);
	ROS_INFO("___tau_safety=%.3f",tau_safety_);
	ROS_INFO("___tau_backup=%.3f",tau_backup_);
	ROS_INFO("___smoothing_tau_obstacle=%.3f",smoothing_tau_obstacle_);
	ROS_INFO("___smoothing_tau_vDes=%.3f",smoothing_tau_vDes_);
	ROS_INFO("___cone_radius=%.3f",cone_radius_);
	ROS_INFO("___cone_lat=%.3f",cone_lat_);
	ROS_INFO("___cone_long=%.3f",cone_long_);

	ROS_INFO("___hoverThrust=%f",hoverThrust_);
	ROS_INFO("___KiVz=%f",KiVz_);
	ROS_INFO("___KpVz=%f",KpVz_);
	ROS_INFO("___KpVxy=%f",KpVxy_);
	ROS_INFO("___KpAttitude=%f",KpAttitude_);
	ROS_INFO("___KdAttitude=%f",KdAttitude_);
	ROS_INFO("___KpOmegaz=%f",KpOmegaz_);
	ROS_INFO("___maxInclination=%f",maxInclination_);

	// Take it for a spin
	ros::spin();

	return 0;
}

