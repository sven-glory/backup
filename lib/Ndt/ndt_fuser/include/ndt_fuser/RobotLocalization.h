#ifndef __CRobotLocalization
#define __CRobotLocalization

#include "NdtLocalization.h"
#include "ScannerParam.h"
#include "TimeStamp.h"

#define LOC_SLIDE_MEAN_SIZE                4

//
//   �������ʱ�������̬��
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
//   �������ʱ�����PCL���ơ�
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

// ��Ǻô�������ŵĵ���
class CLabeledPointCloud
{
public:
	int m_nScannerId;        // ����ɨ�������
	CStampedPclCloud m_cloud;

public:
	CLabeledPointCloud(int nScannerId, const CStampedPclCloud& cloud);
	CLabeledPointCloud(int nScannerId);
};

//////////////////////////////////////////////////////////////////////////////
//   �����ڶ༤��ɨ�����Ķ�NDT��λ��

///////////////////////////////////////////////////////////////////////////////
//
//   CRobotLocalization��װ�˶Ծ��ж���������Ļ����˽��ж�λ�ķ�����
//
//   ˵����
//      1. �����˿���ӵ�ж��������(�����в�ͬ�İ�װ��̬)
//      2. ��λֻ��Ի����˱��壬������κ�һ��������
//      3. ������ݡ�����ɨ�����ݶ�����ʱ������������ǿ����첽����
//      4. ��λ�㷨�����˻����˵��ٶ����أ�����ʱ��������˲���
//
///////////////////////////////////////////////////////////////////////////////

class CRobotLocalization : public CNdtLocalization
{
private:
	unsigned int   m_tmStart;                  // ���в岹֮ǰ��ʱ��
	CStampedAffine m_odometry;                 // δ���岹�������̬
	vector<CLabeledPointCloud> m_clouds;       // δ���岹�ĵ��Ƽ���
	Eigen::Affine3d last_odom;                 // ��һ�ε������̬(����������̬�仯��)
	Eigen::Affine3d m_recentPoses[LOC_SLIDE_MEAN_SIZE];  // ���������̬ƽ���˲��Ľ���
	int             m_nSlideDataCount;         // �Ѿ�������̬ƽ�������������ݵĸ���

protected:
	CScannerGroupParam* m_pScannerGroupParam;  // �����������
	Eigen::Affine3d vel;                       // �ٶ�����
	Eigen::Affine3d odomAdjusted;              // �岹֮��õ��������̬
	CPclPointCloud  cloudAdjusted;             // �岹֮��õ��Ļ��ܵ���
	Eigen::Affine3d estTnow;                   // δ�����ⶨλУ���ĵ�ǰ��̬
	Eigen::Affine3d Tnow;                      // ��λ��Ľ����̬
	unsigned int count_clouds;                 // ���յ��ĵ��Ƶ�����

private:
	// �Լ�����Ľ����̬����ƽ���˲���������˲���Ľ��
	Eigen::Affine3d FilterOutput(const Eigen::Affine3d& pose);

protected:
	// ���յ�������ݺ�Ļص�����
	Eigen::Affine3d getOdomMove(Eigen::Affine3d& odometry);

	// �������յ�����̺ͼ���ɨ�����ݣ������ݻ������ٶȶ����ǽ��в岹
	virtual void CollectOdomLaserData(unsigned int tmStart);

public:
	CRobotLocalization(double map_reso = DEFAULT_MAP_RESO);

	// ���ü���ɨ���������
	void SetScannerGroupParam(CScannerGroupParam* pParam);

	// ���ö�λ��ʼ��̬
	virtual void SetPose(const Eigen::Affine3d& initPos);

	// ���û����˵�˲ʱ�ٶ�
	void SetVelocity(const Eigen::Affine3d& v) { vel = v; }

	// ���յ���̱仯����ʱ���еĻص�
	void OnReceiveOdometry(const CStampedAffine& odomMove);

	// ���յ���������ʱ���еĻص�
	void OnReceiveLaserScan(int nScannerId, const CStampedPclCloud& cloud);
	
	// �����첽�Ķ�λ��������̬���
	virtual bool AsynLocalize(Eigen::Affine3d& pose, bool realTimeRunning = true);

	// ��ָ���ķ�Χ�ڽ�����չ��λ(��̬��λ)
//	virtual int LocalizeEx(CPclPointCloud &cloud_in, Eigen::Affine3d& odometry);
};
#endif
