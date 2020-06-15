#ifndef __CNdtLocalization
#define __CNdtLocalization

#include <ndt_registration/ndt_matcher_d2d_2d.h>
#include <ndt_map/pointcloud_utils.h>
#include <Eigen/Eigen>
#include <pcl/point_cloud.h>
#include "PclPointCloud.h"
#include "VectPose.h"

using namespace std;

#define MAX_SENSOR_RANGE                 40                // ��������Զ��Ч����
#define MIN_SENSOR_RANGE                 0.6               // �����������Ч����

#define LOCALIZE_GAUSSIAN_NEWTON         1                 // 1-��˹ţ�ٷ�

///////////////////////////////////////////////////////////////////////////////
//
//   CNdtLocalization��װ������NDT�������л�����λ�ķ�����
//
//   ˵����
//      1. ֧��Gauss-Newton��λ����
//      2. ��������ڻ����˵Ķ�λ(�ٶ�����һ�����������Ұ�װ��̬�����������ϵ�غ�)
//      3. �ṩ���ֶ�λģʽ��
//           A.����������̬�����ٶ�λ��������ΧС; 
//           B.����������̬����չ��λ(������Ҫ������˲���)��������Χ��
//
///////////////////////////////////////////////////////////////////////////////

class CNdtLocalization
{
private:
	bool   mapIsCreated;                        // NDTͼ���������ɵģ����Ǵ���紫������(�ж��Ƿ���Ҫ��������)

	bool   checkConsistency;			           // �Ƿ���㶨λ����������̬�Ĳ���ڲ�����ʱ������λ���
	double max_translation_norm;                // ��λ����������̬������������ƫ��
	double max_rotation_norm;                   // ��λ����������̬���������Ƕ�ƫ��

protected:
	double map_size_z;
	double resolution;
	double sensor_max_range;                    // ɨ����Զ����
	double sensor_min_range;                    // ɨ���������

public:
	perception_oru::NDTMap *map;		           // NDTͼ
	perception_oru::NDTMatcherD2D_2D matcher2D; // NDTƥ����

private:
	// ���������̬��ɨ����ƽ��ж�λ
	bool LocalizeNewton(const CPclPointCloud &cloud_in, Eigen::Affine3d& odometry);

public:
	// ������ĵ��ƽ��д����˳������ĵ㣬����Z������г�ʼ��
	void FilterCloud(const CPclPointCloud &cloud_in, CPclPointCloud &cloud_filtered);

public:
	CNdtLocalization(double map_reso = DEFAULT_MAP_RESO);
	~CNdtLocalization();

	// ���ö�λ���õ�NDTͼ
	bool SetMap(perception_oru::NDTMap* _map);

	// ���ļ���װ���ͼ
	bool LoadMap(FILE* fp);

	// ���á�һ���Ժ˲顱����(�ɽ�bCheck��Ϊfalse���ر�һ�¼��)
	void SetConsistencyCheck(bool bCheck, double maxTranslation = 0, double maxRotation = 0);

	// �����µ����ݣ������ж�λ����
	virtual bool Localize(const CPclPointCloud &cloud_in, Eigen::Affine3d& odometry);

	// ��ָ���ķ�Χ�ڽ�����չ��λ
	int LocalizeEx(const CPclPointCloud &cloud_in, Eigen::Affine3d& odometry);

	// ���ƥ��״̬
	void ClearMatchStatus();

public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
#endif
