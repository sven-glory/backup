#ifndef __CRobotLocalization
#define __CRobotLocalization

#include "NdtLocalization.h"
#include "ScannerParam.h"
#include "TimeStamp.h"

#define LOC_SLIDE_MEAN_SIZE                4

//
//   定义具有时间戳的姿态。
//
class CStampedAffine : public Eigen::Affine3d, public CTimeStamp
{
public:
	CStampedAffine(Eigen::Affine3d& affine, CTimeStamp& stamp) : 
		Eigen::Affine3d(affine), CTimeStamp(stamp)
	{
	}
	
	CStampedAffine(Eigen::Affine3d& affine, unsigned int uTime) : 
		Eigen::Affine3d(affine), CTimeStamp(uTime)
	{
	}

	CStampedAffine(Eigen::Affine3d& affine) :	Eigen::Affine3d(affine)
	{
	}

	CStampedAffine() {}

	Eigen::Affine3d& GetAffine3dObject()
	{
		Eigen::Affine3d* p = static_cast<Eigen::Affine3d*>(this);
		return *p;
	}
};

//
//   定义具有时间戳的PCL点云。
//
class CStampedPclCloud : public CPclPointCloud, public CTimeStamp
{
public:
	CStampedPclCloud(CPclPointCloud& cloud, CTimeStamp& stamp) :
		CPclPointCloud(cloud), CTimeStamp(stamp)
	{
	}
	
	CStampedPclCloud(CPclPointCloud& cloud, unsigned int uTime) :
		CPclPointCloud(cloud), CTimeStamp(uTime)
	{
	}

	CStampedPclCloud(CPclPointCloud& cloud) : 
		CPclPointCloud(cloud)
	{
	}

	CStampedPclCloud() {}
};

// 标记好传感器编号的点云
class CLabeledPointCloud
{
public:
	int m_nScannerId;        // 激光扫描器编号
	CStampedPclCloud m_cloud;

public:
	CLabeledPointCloud(int nScannerId, const CStampedPclCloud& cloud);
	CLabeledPointCloud(int nScannerId);
};

//////////////////////////////////////////////////////////////////////////////
//   适用于多激光扫描器的定NDT定位。

///////////////////////////////////////////////////////////////////////////////
//
//   CRobotLocalization封装了对具有多个激光器的机器人进行定位的方法。
//
//   说明：
//      1. 机器人可以拥有多个激光器(各自有不同的安装姿态)
//      2. 定位只针对机器人本体，不针对任何一个激光器
//      3. 里程数据、激光扫描数据都具有时间戳，并且它们可以异步到来
//      4. 定位算法考虑了机器人的速度因素，利用时间戳进行了补偿
//
///////////////////////////////////////////////////////////////////////////////

class CRobotLocalization : public CNdtLocalization
{
private:
	unsigned int   m_tmStart;                  // 进行插补之前的时刻
	CStampedAffine m_odometry;                 // 未经插补的里程姿态
	vector<CLabeledPointCloud> m_clouds;       // 未经插补的点云集合
	Eigen::Affine3d last_odom;                 // 上一次的里程姿态(用来计算姿态变化量)
	Eigen::Affine3d m_recentPoses[LOC_SLIDE_MEAN_SIZE];  // 用于输出姿态平滑滤波的阶数
	int             m_nSlideDataCount;         // 已经存入姿态平滑缓冲区的数据的个数

protected:
	CScannerGroupParam* m_pScannerGroupParam;  // 激光器组参数
	Eigen::Affine3d vel;                       // 速度向量
	Eigen::Affine3d odomAdjusted;              // 插补之后得到的里程姿态
	CPclPointCloud  cloudAdjusted;             // 插补之后得到的汇总点云
	Eigen::Affine3d estTnow;                   // 未经激光定位校正的当前姿态
	Eigen::Affine3d Tnow;                      // 定位后的结果姿态
	unsigned int count_clouds;                 // 已收到的点云的数量

private:
	// 对计算出的结果姿态进行平滑滤波，并输出滤波后的结果
	Eigen::Affine3d FilterOutput(const Eigen::Affine3d& pose);

protected:
	// 接收到里程数据后的回调函数
	Eigen::Affine3d getOdomMove(Eigen::Affine3d& odometry);

	// 处理已收到的里程和激光扫描数据，并根据机器人速度对它们进行插补
	virtual void CollectOdomLaserData(unsigned int tmStart);

public:
	CRobotLocalization(double map_reso = DEFAULT_MAP_RESO);

	// 设置激光扫描器组参数
	void SetScannerGroupParam(CScannerGroupParam* pParam);

	// 设置定位初始姿态
	virtual void SetPose(const Eigen::Affine3d& initPos);

	// 设置机器人的瞬时速度
	void SetVelocity(const Eigen::Affine3d& v) { vel = v; }

	// 当收到里程变化数据时进行的回调
	void OnReceiveOdometry(const CStampedAffine& odomMove);

	// 当收到激光数据时进行的回调
	void OnReceiveLaserScan(int nScannerId, const CStampedPclCloud& cloud);
	
	// 进行异步的定位并返回姿态结果
	virtual bool AsynLocalize(Eigen::Affine3d& pose, bool realTimeRunning = true);

	// 在指定的范围内进行扩展定位(静态定位)
//	virtual int LocalizeEx(CPclPointCloud &cloud_in, Eigen::Affine3d& odometry);
};
#endif
