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
//   ���ö�λ��ʼ��̬��
//
void CRobotLocalization::SetPose(const Eigen::Affine3d& initPose)
{
	count_clouds = 0;
	m_nSlideDataCount = 0;
	Tnow = initPose;
}

//
//   ���յ���̱仯����ʱ���еĻص���
//
void CRobotLocalization::OnReceiveOdometry(const CStampedAffine& odomMove)
{
	m_odometry.GetAffine3dObject() = last_odom * odomMove;
}

//
//   ���յ���������ʱ���еĻص���
//
void CRobotLocalization::OnReceiveLaserScan(int nScannerId, const CStampedPclCloud& cloud)
{
	// ��Ǵ�������š����˵��ƣ������ʱ���
	CLabeledPointCloud labeledCloud(nScannerId);
	FilterCloud(cloud, labeledCloud.m_cloud);
	labeledCloud.m_cloud.Stamp(cloud.m_uTime);

	// ����Ǻô�������ŵĵ����ݴ浽���Ƽ�����
	m_clouds.push_back(labeledCloud);
}

//
//   �������յ�����̺ͼ���ɨ�����ݣ������ݻ������ٶȶ����ǽ��в岹��
//
void CRobotLocalization::CollectOdomLaserData(unsigned int tmStart)
{
	// ȡ�õ�ǰʱ��
	m_tmStart = tmStart;

	// �����ȡ��������ʱ����ǰ��ʱ��
	unsigned int tmElapse = m_tmStart - m_odometry.m_uTime;

	// ����õ���̺����ʱ�ڼ��������̬�仯�����������ӵ����������ȥ
	Eigen::Affine3d TFromOdom = vel * (tmElapse / 1000.0);
	odomAdjusted = m_odometry * TFromOdom;

	// ������ܵĵ���
	cloudAdjusted.clear();
	for (size_t i = 0; i < m_clouds.size(); i++)
	{
		CLaserScannerParam& ScannerParam = m_pScannerGroupParam->at(i);
		Eigen::Affine3d sensor_pose = PostureToAffine(ScannerParam.m_pst);

		// ����������ݵĹ���ʱ��
		CLabeledPointCloud& cloud = m_clouds[i];
		tmElapse = m_tmStart - cloud.m_cloud.m_uTime;

		// �������ݶԴ�����λ�˽��в岹
		Eigen::Affine3d TFromScan = vel * (tmElapse / 1000.0);
		sensor_pose = sensor_pose * TFromScan;

		// �����Ʊ任�������˲ο�ϵ��
		perception_oru::transformPointCloudInPlace(sensor_pose, cloud.m_cloud);

		// �ϳɵ���
		cloudAdjusted += cloud.m_cloud;
	}

	m_clouds.clear();
}

//
//   �����첽�Ķ�λ��������̬�����
//
bool CRobotLocalization::AsynLocalize(Eigen::Affine3d& pose, bool realTimeRunning)
{
	unsigned int tmStart = GetTickCount();

	// �������ʵʱ����״̬������Ӧ������̺ͼ���ɨ������
	// ע�⣺(��ʵʱ����״̬�£�����Ӧ���а���ʱ������CollectOdomLaserData()����)
	if (realTimeRunning)
		CollectOdomLaserData(tmStart);

	// ���������̬�ı仯��
	Eigen::Affine3d Tm = getOdomMove(odomAdjusted);

	// ����У����������̬����̱仯����������µ������̬
	Eigen::Affine3d odometry = Tnow * Tm;
	estTnow = odometry;

	// ���ݻ��ܵĵ��ƽ��ж�λ
	bool ok = Localize(cloudAdjusted, odometry);
	Tnow = odometry;

	// �����Ҫ������λ����ʱ�䣬��Ҫ�ڴ˶Խ����̬���в岹
	if (realTimeRunning)
	{
		// ȡ�õ�ǰ��ʱ��(����NDT���㣬�ֲ�����һЩ��ʱ)
		unsigned int tmElapse = GetTickCount() - tmStart;

		// �ٶ�����Ķ�λ������в岹
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
//   �Լ�����Ľ����̬����ƽ���˲���������˲���Ľ����
//
Eigen::Affine3d CRobotLocalization::FilterOutput(const Eigen::Affine3d& pose)
{
	// ��ƽ���˲�������δ����ֱ�Ӽ�������̬
	if (m_nSlideDataCount < LOC_SLIDE_MEAN_SIZE)
		m_recentPoses[m_nSlideDataCount++] = pose;

	// �绺����������������ǰ�ƺ��������̬
	else
	{
		for (int i = 0; i < LOC_SLIDE_MEAN_SIZE - 1; i++)
			m_recentPoses[i] = m_recentPoses[i + 1];
		m_recentPoses[LOC_SLIDE_MEAN_SIZE - 1] = pose;
	}

	// ������̬ƽ�����
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
//   ���յ�������ݺ�Ļص�������
//   ����ֵ���������һ�ε���ʱ����̬�仯����
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
//   ��ָ���ķ�Χ�ڽ�����չ��λ(��̬��λ)��
// 
int CRobotLocalization::LocalizeEx(CPclPointCloud &cloud_in, Eigen::Affine3d& odometry)
{
	odometry = odomAdjusted;
	return CNdtLocalization::LocalizeEx(cloudAdjusted, odometry);
}
#endif
