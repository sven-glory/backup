#include <stdafx.h>
#include "ndt_map/pointcloud_utils.h"
#include "ndt_fuser/RobotLocalization.h"
#include "AffinePosture.h"

Eigen::Affine3d operator * (const Eigen::Affine3d& v, double k)
{
	double vx = v.translation().x();
	double vy = v.translation().y();
	double w = v.rotation().eulerAngles(0, 1, 2)[2];
	Eigen::Affine3d p = Eigen::Translation<double, 3>(k * vx, k * vy, 0)*
		Eigen::AngleAxis<double>(0, Eigen::Vector3d::UnitX()) *
		Eigen::AngleAxis<double>(0, Eigen::Vector3d::UnitY()) *
		Eigen::AngleAxis<double>(k*w, Eigen::Vector3d::UnitZ());
	
	return p;
}

///////////////////////////////////////////////////////////////////////////////

CLabeledPointCloud::CLabeledPointCloud(int nScannerId, const CStampedPclCloud& cloud)
{
	m_nScannerId = nScannerId;
	m_cloud = cloud;
}

CLabeledPointCloud::CLabeledPointCloud(int nScannerId)
{
	m_nScannerId = nScannerId;
}

///////////////////////////////////////////////////////////////////////////////

CRobotLocalization::CRobotLocalization(double map_reso) :
	CNdtLocalization(map_reso)
{
	m_pScannerGroupParam = NULL;
	vel.setIdentity();
	Tnow.setIdentity();
	last_odom.setIdentity();
}

void CRobotLocalization::SetScannerGroupParam(CScannerGroupParam* pParam)
{
	m_pScannerGroupParam = pParam;
}

//
//   设置定位初始姿态。
//
void CRobotLocalization::SetPose(const Eigen::Affine3d& initPose)
{
	count_clouds = 0;
	m_nSlideDataCount = 0;
	Tnow = initPose;
}

//
//   当收到里程变化数据时进行的回调。
//
void CRobotLocalization::OnReceiveOdometry(const CStampedAffine& odomMove)
{
	m_odometry.GetAffine3dObject() = last_odom * odomMove;
}

//
//   当收到激光数据时进行的回调。
//
void CRobotLocalization::OnReceiveLaserScan(int nScannerId, const CStampedPclCloud& cloud)
{
	// 标记传感器编号、过滤点云，并标记时间戳
	CLabeledPointCloud labeledCloud(nScannerId);
	FilterCloud(cloud, labeledCloud.m_cloud);
	labeledCloud.m_cloud.Stamp(cloud.m_uTime);

	// 将标记好传感器编号的点云暂存到点云集合中
	m_clouds.push_back(labeledCloud);
}

//
//   处理已收到的里程和激光扫描数据，并根据机器人速度对它们进行插补。
//
void CRobotLocalization::CollectOdomLaserData(unsigned int tmStart)
{
	// 取得当前时间
	m_tmStart = tmStart;

	// 计算从取得里量程时到当前的时延
	unsigned int tmElapse = m_tmStart - m_odometry.m_uTime;

	// 计算得到里程后的延时期间产生的姿态变化，并把它叠加到里程数据中去
	Eigen::Affine3d TFromOdom = vel * (tmElapse / 1000.0);
	odomAdjusted = m_odometry * TFromOdom;

	// 计算汇总的点云
	cloudAdjusted.clear();
	for (size_t i = 0; i < m_clouds.size(); i++)
	{
		CLaserScannerParam& ScannerParam = m_pScannerGroupParam->at(i);
		Eigen::Affine3d sensor_pose = PostureToAffine(ScannerParam.m_pst);

		// 计算点云数据的过期时间
		CLabeledPointCloud& cloud = m_clouds[i];
		tmElapse = m_tmStart - cloud.m_cloud.m_uTime;

		// 根据数据对传感器位姿进行插补
		Eigen::Affine3d TFromScan = vel * (tmElapse / 1000.0);
		sensor_pose = sensor_pose * TFromScan;

		// 将点云变换到机器人参考系内
		perception_oru::transformPointCloudInPlace(sensor_pose, cloud.m_cloud);

		// 合成点云
		cloudAdjusted += cloud.m_cloud;
	}

	m_clouds.clear();
}

//
//   进行异步的定位并返回姿态结果。
//
bool CRobotLocalization::AsynLocalize(Eigen::Affine3d& pose, bool realTimeRunning)
{
	unsigned int tmStart = GetTickCount();

	// 如果处于实时运行状态，现在应汇总里程和激光扫描数据
	// 注意：(非实时运行状态下，程序应另行安排时机调用CollectOdomLaserData()函数)
	if (realTimeRunning)
		CollectOdomLaserData(tmStart);

	// 计算里程姿态的变化量
	Eigen::Affine3d Tm = getOdomMove(odomAdjusted);

	// 根据校正后的里程姿态和里程变化量，估算出新的里程姿态
	Eigen::Affine3d odometry = Tnow * Tm;
	estTnow = odometry;

	// 根据汇总的点云进行定位
	bool ok = Localize(cloudAdjusted, odometry);
	Tnow = odometry;

	// 如果需要补偿定位运算时间，需要在此对结果姿态进行插补
	if (realTimeRunning)
	{
		// 取得当前的时间(经过NDT运算，又产生了一些延时)
		unsigned int tmElapse = GetTickCount() - tmStart;

		// 再对上面的定位结果进行插补
		Eigen::Affine3d TFromStart = vel * (tmElapse / 1000.0);
		Tnow = Tnow * TFromStart;
	}
//	if (ok)
//		pose = Tnow;

	count_clouds++;

	pose = FilterOutput(Tnow);

	return ok;
}

//
//   对计算出的结果姿态进行平滑滤波，并输出滤波后的结果。
//
Eigen::Affine3d CRobotLocalization::FilterOutput(const Eigen::Affine3d& pose)
{
	// 如平滑滤波缓冲区未满，直接加入新姿态
	if (m_nSlideDataCount < LOC_SLIDE_MEAN_SIZE)
		m_recentPoses[m_nSlideDataCount++] = pose;

	// 如缓冲区已满，则数据前移后加入新姿态
	else
	{
		for (int i = 0; i < LOC_SLIDE_MEAN_SIZE - 1; i++)
			m_recentPoses[i] = m_recentPoses[i + 1];
		m_recentPoses[LOC_SLIDE_MEAN_SIZE - 1] = pose;
	}

	// 计算姿态平滑结果
	double sumX = 0, sumY = 0, sumThita = 0;
	for (int i = 0; i < m_nSlideDataCount; i++)
	{
		sumX += m_recentPoses[i].translation().x();
		sumY += m_recentPoses[i].translation().y();
		sumThita += m_recentPoses[i].rotation().eulerAngles(0, 1, 2)(2) + 2*M_PI;
	}
	double aveX = sumX / m_nSlideDataCount;
	double aveY = sumY / m_nSlideDataCount;
	double aveThita = sumThita / m_nSlideDataCount - 2 * M_PI;

	Eigen::Affine3d output =
		Eigen::Translation<double, 3>(aveX, aveY, 0) *
		Eigen::AngleAxis<double>( aveThita, Eigen::Vector3d::UnitZ());

	return output;
}

//
//   接收到里程数据后的回调函数。
//   返回值：相对于上一次调用时的姿态变化量。
//
Eigen::Affine3d CRobotLocalization::getOdomMove(Eigen::Affine3d& odometry)
{
	Eigen::Affine3d Tm;
	if (count_clouds == 0)
		Tm.setIdentity();
	else
		Tm = last_odom.inverse() * odometry;

	last_odom = odometry;

	return Tm;
}


#if 0
//
//   在指定的范围内进行扩展定位(静态定位)。
// 
int CRobotLocalization::LocalizeEx(CPclPointCloud &cloud_in, Eigen::Affine3d& odometry)
{
	odometry = odomAdjusted;
	return CNdtLocalization::LocalizeEx(cloudAdjusted, odometry);
}
#endif
