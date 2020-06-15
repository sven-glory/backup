#ifndef __CNdtLocalization
#define __CNdtLocalization

#include <ndt_registration/ndt_matcher_d2d_2d.h>
#include <ndt_map/pointcloud_utils.h>
#include <Eigen/Eigen>
#include <pcl/point_cloud.h>
#include "PclPointCloud.h"
#include "VectPose.h"

using namespace std;

#define MAX_SENSOR_RANGE                 40                // 传感器最远有效距离
#define MIN_SENSOR_RANGE                 0.6               // 传感器最近有效距离

#define LOCALIZE_GAUSSIAN_NEWTON         1                 // 1-高斯牛顿法

///////////////////////////////////////////////////////////////////////////////
//
//   CNdtLocalization封装了利用NDT方法进行基础定位的方法。
//
//   说明：
//      1. 支持Gauss-Newton定位方案
//      2. 仅处理基于机器人的定位(假定仅有一个激光器，且安装姿态与机器人坐标系重合)
//      3. 提供两种定位模式：
//           A.给定估测姿态，快速定位，收敛范围小; 
//           B.给定估测姿态，扩展定位(较慢，要求机器人不动)，收敛范围大
//
///////////////////////////////////////////////////////////////////////////////

class CNdtLocalization
{
private:
	bool   mapIsCreated;                        // NDT图是自我生成的，还是从外界传递来的(判断是否需要析构销毁)

	bool   checkConsistency;			           // 是否计算定位结果与估测姿态的差，并在差距过大时放弃定位结果
	double max_translation_norm;                // 定位结果与估测姿态的最大允许距离偏差
	double max_rotation_norm;                   // 定位结果与估测姿态的最大允许角度偏差

protected:
	double map_size_z;
	double resolution;
	double sensor_max_range;                    // 扫描最远距离
	double sensor_min_range;                    // 扫描最近距离

public:
	perception_oru::NDTMap *map;		           // NDT图
	perception_oru::NDTMatcherD2D_2D matcher2D; // NDT匹配器

private:
	// 根据里程姿态和扫描点云进行定位
	bool LocalizeNewton(const CPclPointCloud &cloud_in, Eigen::Affine3d& odometry);

public:
	// 对输入的点云进行处理，滤除过近的点，并对Z坐标进行初始化
	void FilterCloud(const CPclPointCloud &cloud_in, CPclPointCloud &cloud_filtered);

public:
	CNdtLocalization(double map_reso = DEFAULT_MAP_RESO);
	~CNdtLocalization();

	// 设置定位所用的NDT图
	bool SetMap(perception_oru::NDTMap* _map);

	// 从文件中装入地图
	bool LoadMap(FILE* fp);

	// 设置“一致性核查”参数(可将bCheck置为false来关闭一致检查)
	void SetConsistencyCheck(bool bCheck, double maxTranslation = 0, double maxRotation = 0);

	// 根据新的数据，仅进行定位处理
	virtual bool Localize(const CPclPointCloud &cloud_in, Eigen::Affine3d& odometry);

	// 在指定的范围内进行扩展定位
	int LocalizeEx(const CPclPointCloud &cloud_in, Eigen::Affine3d& odometry);

	// 清除匹配状态
	void ClearMatchStatus();

public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
#endif
