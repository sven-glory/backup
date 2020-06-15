#include <stdafx.h>

#undef max
#undef min

#include "ndt_fuser/NdtLocalization.h"
#include "AffinePosture.h"
#include <functional>
#include <algorithm>
#include "time_patch.h"

#define MAX_TRANSLATION_DELTA            0.5
#define MAX_ROTATION_DELTA               0.5

//////////////////////////////////////////////////////////////////////////////

// ����չ��λ���򡱵Ķ��塣
//  ����������չ���Ƿ�Χ��X:+/- 0.6m, Y : +/- 1.2m, thita : +/- 24��
//
class CExtPoses
{
private:
	Eigen::Affine3d pose;            // ����������̬
	float  unitX;                    // X����λ���
	float  unitY;                    // Y����λ���
	float  unitAngle;                // �Ƕȼ��

public:
	CExtPoses(Eigen::Affine3d _pose)
	{
		pose = _pose;
		unitX = 0.2f;
		unitY = 0.4f;
		unitAngle = CAngle::ToRadian(8);
	}

	int count() const { return 125; }

	// ���ݸ����������ţ�ֱ����������̬
	Eigen::Affine3d operator [](int idx) const
	{
		int n[5] = { 0, 1, -1, 2, -2 };
		int i = n[(idx / 25) % 5];     // X����ƫ������
		int j = n[(idx / 5) % 5];      // Y����ƫ������
		int k = n[idx % 5];            // �Ƕȷ���ƫ������

		Eigen::Affine3d ret = pose *
			Eigen::Translation<double, 3>(i * unitX, j * unitY, 0) *
			Eigen::AngleAxis<double>(k * unitAngle, Eigen::Vector3d::UnitZ());

		return ret;
	}
};

//////////////////////////////////////////////////////////////////////////////

CNdtLocalization::CNdtLocalization(double map_reso)
{
	mapIsCreated = false;
	map = NULL;

	sensor_max_range = MAX_SENSOR_RANGE;
	sensor_min_range = MIN_SENSOR_RANGE;

	map_size_z = 0.4;

	resolution = map_reso;

	matcher2D.ITR_MAX = 30;
	matcher2D.n_neighbours = 2;

	max_translation_norm = 1.;
	max_rotation_norm = M_PI / 4;
	checkConsistency = false;
}

CNdtLocalization::~CNdtLocalization()
{
	// ���ԭ�е�ͼ���Լ����ɵģ��ڴ���Ҫ�ͷſռ�
	if (map != NULL && mapIsCreated)
		delete map;
}

//
//   ���ö�λ���õ�NDTͼ��
//
bool CNdtLocalization::SetMap(perception_oru::NDTMap* _map) 
{
	// ���ԭ�е�ͼ���Լ����ɵģ��ڴ���Ҫ�ͷſռ�
	if (map != NULL && mapIsCreated)
		delete map;
	mapIsCreated = false;

	if (_map != NULL)
	{
		map = _map;
		return true;
	}
	else
		return false;
}

//
//   ���á�һ���Ժ˲顱������
//
void CNdtLocalization::SetConsistencyCheck(bool bCheck, double maxTranslation, double maxRotation)
{
	checkConsistency = bCheck;
	max_translation_norm = maxTranslation;
	max_rotation_norm = maxRotation;
}

//
//   ������ĵ��ƽ��д����˳������ĵ㣬����Z������г�ʼ����
//
void CNdtLocalization::FilterCloud(const CPclPointCloud &cloud_in, CPclPointCloud &cloud_filtered)
{
	const double varz = 0.05;

	cloud_filtered.clear();
	pcl::PointXYZ pt;
	for (int i = 0; i < cloud_in.points.size(); i++)
	{
		pt = cloud_in.points[i];
		double d = sqrt(pt.x*pt.x + pt.y*pt.y);

		// �˳�����̫���ĵ�
		if (d > sensor_min_range)
		{
			pt.z += varz*((double)rand()) / (double)INT_MAX;
			cloud_filtered.points.push_back(pt);
		}
	}
}

//
//   ���������̬��ɨ����ƽ��ж�λ��
//
bool CNdtLocalization::LocalizeNewton(const CPclPointCloud &cloud, Eigen::Affine3d& odometry)
{
	// ���ɾֲ���ͼ
	perception_oru::NDTMap ndlocal(new perception_oru::LazyGrid(resolution));
	ndlocal.guessSize(0, 0, 0, sensor_max_range, sensor_max_range, map_size_z);
	ndlocal.loadPointCloud(cloud, sensor_max_range);
	ndlocal.computeNDTCells(CELL_UPDATE_MODE_SAMPLE_VARIANCE);

	// ���Խ��ֲ�ͼ��ģ�͵�ͼ����ƥ�䣬����ɹ���ƥ������̬������odometry��
	return matcher2D.match(*map, ndlocal, odometry, true);
}

// 
//   �����µ����ݣ������ж�λ����
//   ����ֵ��
//      true  - ��λ�ɹ�����λ����洢��odometry��
//      false - ��λʧ�ܣ�odometryֵ����
//
bool CNdtLocalization::Localize(const CPclPointCloud &cloud, Eigen::Affine3d& odometry)
{
	// ������̬�仯���������̬
	Eigen::Affine3d Tinit = odometry;
	bool ok = LocalizeNewton(cloud, Tinit);

	// �˲鶨λ��̬�����ֵ�Ƿ������
	if (ok && checkConsistency)
	{
		// ���������̬����׼��̬֮��Ĳ�ֵ
		Eigen::Affine3d diff = odometry.inverse() * Tinit;

		// �����̬�������Ϊ��λʧ��
		if (diff.translation().norm() > max_translation_norm ||
			diff.rotation().eulerAngles(0, 1, 2).norm() > max_rotation_norm)
			ok = false;
	}

	// �����λ�ɹ�������ܶ�λ�����̬��������Ȼ����̹�����̬
	if (ok)
		odometry = Tinit;

	return ok;
}

//
//   ��ָ���ķ�Χ�ڽ�����չ��λ��
//
int CNdtLocalization::LocalizeEx(const CPclPointCloud &cloud_in, Eigen::Affine3d& odometry)
{
	CExtPoses poses(odometry);

	for (int i = 0; i < poses.count(); i++)
	{
		Eigen::Affine3d Tinit = poses[i];

		// �����λ�ɹ�
		if (LocalizeNewton(cloud_in, Tinit))
		{
			odometry = Tinit;
			return i;
		}
	}

	return -1;
}

//
//   ���ƥ��״̬��
//
void CNdtLocalization::ClearMatchStatus()
{
	matcher2D.ClearMatchStatus();
	map->ClearMatchStatus();
}

//
//   ���ļ���װ���ͼ��
//
bool CNdtLocalization::LoadMap(FILE* fp)
{
	// ���ԭ�е�ͼ���Լ����ɵģ��ڴ���Ҫ�ͷſռ�
	if (map != NULL && mapIsCreated)
		delete map;

	map = new perception_oru::NDTMap(new perception_oru::LazyGrid);
	if (map == NULL)
		return false;

	if (!map->Load(fp))
		return false;

	mapIsCreated = true;
	return true;
}
