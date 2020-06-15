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

// “扩展定位区域”的定义。
//  进行区域扩展覆盖范围：X:+/- 0.6m, Y : +/- 1.2m, thita : +/- 24度
//
class CExtPoses
{
private:
	Eigen::Affine3d pose;            // 区域中心姿态
	float  unitX;                    // X方向单位间隔
	float  unitY;                    // Y方向单位间隔
	float  unitAngle;                // 角度间隔

public:
	CExtPoses(Eigen::Affine3d _pose)
	{
		pose = _pose;
		unitX = 0.2f;
		unitY = 0.4f;
		unitAngle = CAngle::ToRadian(8);
	}

	int count() const { return 125; }

	// 根据给定的索引号，直接输出结果姿态
	Eigen::Affine3d operator [](int idx) const
	{
		int n[5] = { 0, 1, -1, 2, -2 };
		int i = n[(idx / 25) % 5];     // X方向偏差索引
		int j = n[(idx / 5) % 5];      // Y方向偏差索引
		int k = n[idx % 5];            // 角度方向偏差索引

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
	// 如果原有地图是自己生成的，在此需要释放空间
	if (map != NULL && mapIsCreated)
		delete map;
}

//
//   设置定位所用的NDT图。
//
bool CNdtLocalization::SetMap(perception_oru::NDTMap* _map) 
{
	// 如果原有地图是自己生成的，在此需要释放空间
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
//   设置“一致性核查”参数。
//
void CNdtLocalization::SetConsistencyCheck(bool bCheck, double maxTranslation, double maxRotation)
{
	checkConsistency = bCheck;
	max_translation_norm = maxTranslation;
	max_rotation_norm = maxRotation;
}

//
//   对输入的点云进行处理，滤除过近的点，并对Z坐标进行初始化。
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

		// 滤除距离太近的点
		if (d > sensor_min_range)
		{
			pt.z += varz*((double)rand()) / (double)INT_MAX;
			cloud_filtered.points.push_back(pt);
		}
	}
}

//
//   根据里程姿态和扫描点云进行定位。
//
bool CNdtLocalization::LocalizeNewton(const CPclPointCloud &cloud, Eigen::Affine3d& odometry)
{
	// 生成局部地图
	perception_oru::NDTMap ndlocal(new perception_oru::LazyGrid(resolution));
	ndlocal.guessSize(0, 0, 0, sensor_max_range, sensor_max_range, map_size_z);
	ndlocal.loadPointCloud(cloud, sensor_max_range);
	ndlocal.computeNDTCells(CELL_UPDATE_MODE_SAMPLE_VARIANCE);

	// 测试将局部图与模型地图进行匹配，如果成功，匹配结果姿态保存于odometry中
	return matcher2D.match(*map, ndlocal, odometry, true);
}

// 
//   根据新的数据，仅进行定位处理。
//   返回值：
//      true  - 定位成功，定位结果存储于odometry中
//      false - 定位失败，odometry值不变
//
bool CNdtLocalization::Localize(const CPclPointCloud &cloud, Eigen::Affine3d& odometry)
{
	// 根据姿态变化更新里程姿态
	Eigen::Affine3d Tinit = odometry;
	bool ok = LocalizeNewton(cloud, Tinit);

	// 核查定位姿态与估测值是否差别过大
	if (ok && checkConsistency)
	{
		// 计算估测姿态与配准姿态之间的差值
		Eigen::Affine3d diff = odometry.inverse() * Tinit;

		// 如果姿态差过大，视为定位失败
		if (diff.translation().norm() > max_translation_norm ||
			diff.rotation().eulerAngles(0, 1, 2).norm() > max_rotation_norm)
			ok = false;
	}

	// 如果定位成功，则接受定位结果姿态，否则，仍然用里程估测姿态
	if (ok)
		odometry = Tinit;

	return ok;
}

//
//   在指定的范围内进行扩展定位。
//
int CNdtLocalization::LocalizeEx(const CPclPointCloud &cloud_in, Eigen::Affine3d& odometry)
{
	CExtPoses poses(odometry);

	for (int i = 0; i < poses.count(); i++)
	{
		Eigen::Affine3d Tinit = poses[i];

		// 如果定位成功
		if (LocalizeNewton(cloud_in, Tinit))
		{
			odometry = Tinit;
			return i;
		}
	}

	return -1;
}

//
//   清除匹配状态。
//
void CNdtLocalization::ClearMatchStatus()
{
	matcher2D.ClearMatchStatus();
	map->ClearMatchStatus();
}

//
//   从文件中装入地图。
//
bool CNdtLocalization::LoadMap(FILE* fp)
{
	// 如果原有地图是自己生成的，在此需要释放空间
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
