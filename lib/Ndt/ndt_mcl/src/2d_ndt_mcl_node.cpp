#include "time_patch.h"
#include "ndt_mcl.h"
#include "ndt_map/ndt_map.h"

/////////////////////////////////////////////////////////////////////////////////////////////////////////
// Initial pose stuff (quick and dirty as always)
/////////////////////////////////////////////////////////////////////////////////////////////////////////
bool userInitialPose = false;
bool hasNewInitialPose = false;

double ipos_x = 0, ipos_y = 0, ipos_yaw = 0;
double ivar_x = 0, ivar_y = 0, ivar_yaw = 0;

void initialPoseReceived(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg) 
{
	tf::Pose ipose;
	tf::poseMsgToTF(msg->pose.pose, ipose);
	ipos_x = ipose.getOrigin().x();
	ipos_y = ipose.getOrigin().y();
	double pitch, roll;
	ipose.getBasis().getEulerYPR(ipos_yaw, pitch, roll);

	ivar_x = msg->pose.covariance[0];
	ivar_x = msg->pose.covariance[6];
	ivar_x = msg->pose.covariance[35];

	hasNewInitialPose = true;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////

Eigen::Affine3d getAsAffine(float x, float y, float yaw) 
{
	Eigen::Matrix3d m;
	m = Eigen::AngleAxisd(0, Eigen::Vector3d::UnitX())
		* Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY())
		* Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ());
	Eigen::Translation3d v(x, y, 0);
	Eigen::Affine3d T = v*m;

	return T;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////

NDTMCL *ndtmcl;

// Laser sensor offset
float offx = 0;
float offy = 0;
float offa = 0;

static bool has_sensor_offset_set = false;
static bool isFirstLoad = true;
Eigen::Affine3d Told, Todo;

/////////////////////////////////////////////////////////////////////////////////////////////////////////

double time_end;
std::string tf_odo_topic = "odom_base_link";
std::string tf_state_topic = "base_link";
std::string tf_laser_link = "base_laser_link";

#if 0
double getDoubleTime()
{
	struct timeval time;
	gettimeofday(&time, NULL);
	return time.tv_sec + time.tv_usec * 1e-6;
}
#endif


/**
* Callback for laser scan messages
*/
void callback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
	static int counter = 0;
	counter++;

	static tf::TransformListener tf_listener;
	double time_now = getDoubleTime();
	double looptime = time_end - time_now;
	fprintf(stderr, "Lt( %.1lfms %.1lfHz seq:%d) -", looptime * 1000, 1.0 / looptime, scan->header.seq);

	if (has_sensor_offset_set == false) return;
	double gx, gy, gyaw, x, y, yaw;

	///Get state information
	tf::StampedTransform transform;
	tf_listener.waitForTransform("world", tf_state_topic, scan->header.stamp, ros::Duration(1.0));
	///Ground truth --- Not generally available so should be changed to the manual initialization
	try {
		tf_listener.lookupTransform("world", tf_state_topic, scan->header.stamp, transform);
		gyaw = tf::getYaw(transform.getRotation());
		gx = transform.getOrigin().x();
		gy = transform.getOrigin().y();
	}
	catch (tf::TransformException ex) {
		ROS_ERROR("%s", ex.what());
		return;
	}

	///Odometry 
	try {
		tf_listener.lookupTransform("world", tf_odo_topic, scan->header.stamp, transform);
		yaw = tf::getYaw(transform.getRotation());
		x = transform.getOrigin().x();
		y = transform.getOrigin().y();
	}
	catch (tf::TransformException ex) {
		ROS_ERROR("%s", ex.what());
		return;
	}


	///Number of scans
	int N = (scan->angle_max - scan->angle_min) / scan->angle_increment;
	/////
	/// Pose conversions
	////

	Eigen::Affine3d T = getAsAffine(x, y, yaw);
	Eigen::Affine3d Tgt = getAsAffine(gx, gy, gyaw);

	if (userInitialPose && hasNewInitialPose)
	{
		gx = ipos_x;
		gy = ipos_y;
		gyaw = ipos_yaw;
	}

	if (isFirstLoad || hasNewInitialPose)
	{
		fprintf(stderr, "Initializing to (%lf, %lf, %lf)\n", gx, gy, gyaw);
		/// Initialize the particle filter
		ndtmcl->initializeFilter(gx, gy, gyaw, 0.2, 0.2, 2.0*M_PI / 180.0, /*150*/1000);
		Told = T;
		Todo = Tgt;
		hasNewInitialPose = false;
	}

	///Calculate the differential motion from the last frame
	Eigen::Affine3d Tmotion = Told.inverse() * T;
	Todo = Todo*Tmotion; ///< just integrates odometry for the visualization

	Told = T;

	///Calculate the laser pose with respect to the base
	float dy = offy;
	float dx = offx;
	float alpha = atan2(dy, dx);
	float L = sqrt(dx*dx + dy*dy);

	///Laser pose in base frame
	float lpx = L * cos(alpha);
	float lpy = L * sin(alpha);
	float lpa = offa;

	///Laser scan to PointCloud expressed in the base frame
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	for (int j = 0; j<N; j++) 
	{
		double r = scan->ranges[j];
		if (r >= scan->range_min && r<scan->range_max && r>0.3 && r<20.0) 
		{
			double a = scan->angle_min + j*scan->angle_increment;
			pcl::PointXYZ pt;
			pt.x = r*cos(a + lpa) + lpx;
			pt.y = r*sin(a + lpa) + lpy;
			pt.z = 0.1 + 0.02 * (double)rand() / (double)RAND_MAX;
			cloud->push_back(pt);
		}
	}

	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	/// Now we have the sensor origin and pointcloud -- Lets do MCL
	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	ndtmcl->updateAndPredict(Tmotion, *cloud); ///<predicts, updates and resamples if necessary (ndt_mcl.hpp)

	Eigen::Vector3d dm = ndtmcl->getMean(); ///Maximum aposteriori pose
	Eigen::Matrix3d cov = ndtmcl->pf.getDistributionVariances(); ///Pose covariance

	time_end = getDoubleTime();
	fprintf(stderr, "Time elapsed %.1lfms (%lf %lf %lf) \n", (time_end - time_now) * 1000, dm[0], dm[1], dm[2]);
	isFirstLoad = false;
}


int main1(int argc, char **argv) 
{
	double resolution = 0.4;
	time_end = getDoubleTime();

	std::string tf_base_link, input_laser_topic;

	//////////////////////////////////////////////////////////
	//////////////////////////////////////////////////////////
	/// Parameters for the mapper
	//////////////////////////////////////////////////////////
	//////////////////////////////////////////////////////////
	bool loadMap = false; ///< flag to indicate that we want to load a map
	std::string mapName("basement.ndmap"); ///<name and the path to the map

	bool saveMap = true;						///< indicates if we want to save the map in a regular intervals
	std::string output_map_name = std::string("ndt_mapper_output.ndmap");

	//////////////////////////////////////////////////////////
	//////////////////////////////////////////////////////////
	/// Set the values from a config file
	//////////////////////////////////////////////////////////
	//////////////////////////////////////////////////////////

	paramHandle.param<std::string>("input_laser_topic", input_laser_topic, std::string("/base_scan"));
	paramHandle.param<std::string>("tf_base_link", tf_state_topic, std::string("/base_link"));
	paramHandle.param<std::string>("tf_laser_link", tf_laser_link, std::string("/hokuyo1_link"));

	bool use_sensor_pose;
	paramHandle.param<bool>("use_sensor_pose", use_sensor_pose, false);
	double sensor_pose_x, sensor_pose_y, sensor_pose_th;
	paramHandle.param<double>("sensor_pose_x", sensor_pose_x, 0.);
	paramHandle.param<double>("sensor_pose_y", sensor_pose_y, 0.);
	paramHandle.param<double>("sensor_pose_th", sensor_pose_th, 0.);

	paramHandle.param<bool>("load_map_from_file", loadMap, false);
	paramHandle.param<std::string>("map_file_name", mapName, std::string("basement.ndmap"));

	paramHandle.param<bool>("save_output_map", saveMap, true);
	paramHandle.param<std::string>("output_map_file_name", output_map_name, std::string("ndt_mapper_output.ndmap"));

	paramHandle.param<double>("map_resolution", resolution, 0.2);
	bool forceSIR = false;
	paramHandle.param<bool>("forceSIR", forceSIR, false);

	paramHandle.param<bool>("set_initial_pose", userInitialPose, false);
	paramHandle.param<double>("initial_pose_x", ipos_x, 0.);
	paramHandle.param<double>("initial_pose_y", ipos_y, 0.);
	paramHandle.param<double>("initial_pose_yaw", ipos_yaw, 0.);

	if (userInitialPose == true) hasNewInitialPose = true;

	//////////////////////////////////////////////////////////
	/// Prepare the map

	//////////////////////////////////////////////////////////
	fprintf(stderr, "USING RESOLUTION %lf\n", resolution);
	perception_oru::NDTMap ndmap(new perception_oru::LazyGrid(resolution));

	ndmap.setMapSize(80.0, 80.0, 1.0);

	if (loadMap) 
	{
		fprintf(stderr, "Loading Map from '%s'\n", mapName.c_str());
		ndmap.loadFromJFF(mapName.c_str());
	}

	ndtmcl = new NDTMCL(resolution, ndmap, -0.5);
	if (forceSIR)
		ndtmcl->forceSIR = true;

	fprintf(stderr, "*** FORCE SIR = %d****", forceSIR);

	offa = sensor_pose_th;
	offx = sensor_pose_x;
	offy = sensor_pose_y;


	has_sensor_offset_set = true;

	fprintf(stderr, "Sensor Pose = (%lf %lf %lf)\n", offx, offy, offa);

	return 0;
}
