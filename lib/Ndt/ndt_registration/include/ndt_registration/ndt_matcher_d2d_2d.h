/*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2010, AASS Research Center, Orebro University.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of AASS Research Center nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*
*/
#ifndef NDT_MATCHER_D2D_2D_HH
#define NDT_MATCHER_D2D_2D_HH

#include <ndt_map/ndt_map.h>
#include "pcl/point_cloud.h"
#include "Eigen/Core"

#include <stdlib.h>
#include <stdio.h>
#include <math.h>

#ifdef _MFC_VER
class CDC;
class CScreenReference;
#elif defined QT_VERSION
#include <QColor>
#endif

namespace perception_oru
{
	class NDTMatchResult
	{
	public:
		double finalscore;
		int      runTime;
		int      countGood;
		int      countSourceNDT;
		bool     bMatchOK;
		int      matchCycles;

	public:
		void Reset()
		{
			bMatchOK = false;
			finalscore = 0;
			countGood = 0;
			countSourceNDT = 0;
			matchCycles = 0;
			runTime = 0;
		}

		bool IsOk()
		{
			if (countGood < 8)
				bMatchOK = false;
			else if (countGood < 50)
				bMatchOK = (countGood > countSourceNDT / 2);
			else
				bMatchOK = (countGood > countSourceNDT / 3);

			return bMatchOK;
		}
	};

	/**
	* This class implements NDT registration for 3D point cloud scans.
	*/
	class NDTMatcherD2D_2D
	{
	protected:
		Eigen::Matrix<double, 3, 3> Jest;
		Eigen::Matrix<double, 9, 3> Hest;
		Eigen::Matrix<double, 3, 9> Zest;
		Eigen::Matrix<double, 9, 9> ZHest;

		//vars for gradient
		Eigen::Matrix<double, 3, 1> xtBJ, xtBZBx, Q;

		//vars for hessian
		Eigen::Matrix<double, 3, 3> JtBJ, xtBZBJ, xtBH, xtBZBZBx, xtBZhBx;
		Eigen::Matrix<double, 1, 3> TMP1, xtB;

		double lfd1, lfd2;
		std::vector<double> resolutions;

		//auxiliary functions for MoreThuente line search
		static double MoreThuente_min(double a, double b);
		static double MoreThuente_max(double a, double b);
		static double MoreThuente_absmax(double a, double b, double c);
		static int MoreThuente_cstep(double& stx, double& fx, double& dx,
				double& sty, double& fy, double& dy,
				double& stp, double& fp, double& dp,
				bool& brackt, double stmin, double stmax);
		

	public:
		double current_resolution;

		///max iterations, set in constructor
		int ITR_MAX;

		///the change in score after which we converge. Set to 1e-3 in constructor
		double DELTA_SCORE;
		//how many neighbours to use in the objective
		int n_neighbours;

		NDTMatchResult result;
		VectNDTCells nextNDT;
		VectNDTCells nextNDT0;

	private:
		void HandleSpecialCase(Eigen::MatrixXd& H, double score_gradient_2d_norm);

	protected:
		double normalizeAngle(double a);

		//initializes stuff;
		void init(bool useDefaultGridResolutions, std::vector<double> _resolutions);

		//iteratively update the score gradient and hessian
		virtual bool update_gradient_hessian_2d(
			Eigen::MatrixXd &score_gradient,
			Eigen::MatrixXd &Hessian,
			const Eigen::Vector3d &m1,
			const Eigen::Matrix3d &C1,
			const double &likelihood,
			bool computeHessian);

		//pre-computes the derivative matrices Jest, Hest, Zest, ZHest
		void computeDerivatives_2d(Eigen::Vector3d &m1, Eigen::Matrix3d C1, bool computeHessian = true);

		//iteratively update the score gradient and hessian (2d version)
		virtual bool update_gradient_hessian_local_2d(
			Eigen::MatrixXd &score_gradient,
			Eigen::MatrixXd &Hessian,
			const Eigen::Vector3d &m1,
			const Eigen::Matrix3d &C1,
			const double &likelihood,
			const Eigen::Matrix<double, 3, 3> &_Jest,
			const Eigen::Matrix<double, 9, 3> &_Hest,
			const Eigen::Matrix<double, 3, 9> &_Zest,
			const Eigen::Matrix<double, 9, 9> &_ZHest,
			bool computeHessian);

		//pre-computes the derivative matrices Jest, Hest, Zest, ZHest
		void computeDerivativesLocal_2d(Eigen::Vector3d &m1, Eigen::Matrix3d C1,
			Eigen::Matrix<double, 3, 3> &_Jest,
			Eigen::Matrix<double, 9, 3> &_Hest,
			Eigen::Matrix<double, 3, 9> &_Zest,
			Eigen::Matrix<double, 9, 9> &_ZHest,
			bool computeHessian);

		//perform line search to find the best descent rate (Mohre&Thuente)
		//adapted from NOX
		double lineSearch2D(
			Eigen::Matrix<double, 3, 1> &increment,
			VectNDTCells &source,
			NDTMap &target);

	public:
		/**
		parametrized constructor. A default set is (false,false,true,empty_vector). parameters are:
		\param useDefaultGridResolutions --- if set, the following parameter is set to a preset value
		\param _resolutions --- if previous bool is not set, these are the resolutions (in reverse order) that we will go through
		*/
		NDTMatcherD2D_2D(bool useDefaultGridResolutions, std::vector<double> _resolutions)
		{
			this->init(useDefaultGridResolutions, _resolutions);
		}

		NDTMatcherD2D_2D()
		{
			this->init(true, std::vector<double>());
		}

		NDTMatcherD2D_2D(const NDTMatcherD2D_2D& other)
		{
			this->init(false, other.resolutions);
		}

		~NDTMatcherD2D_2D();

		// TODO: const the target
		/**
		* Register two point clouds. Use only 2D rotations This method builds an NDT
		* representation of the "fixed" point cloud and uses that for
		* registering the "moving" point cloud.
		* \param  target
		*   Reference data. NDT structure is built for this point cloud.
		* \param  source
		*   The output transformation registers this point cloud to \c target.
		* \param  T
		*   This is an input/output parameter. The initial value of \c T
		*   gives the initial pose estimate of \c moving. When the
		*   algorithm terminates, \c T holds the registration result.
		*/
		bool match(pcl::PointCloud<pcl::PointXYZ>& target,
			pcl::PointCloud<pcl::PointXYZ>& source,
			Eigen::Transform<double, 3, Eigen::Affine, Eigen::ColMajor>& T,
			bool useInitialGuess = false);

		/**
		* Registers a two ndt maps using only 2D rotation/translation.
		* \param  target
		*   Reference data.
		* \param  source
		*   The output transformation registers this point cloud to \c target.
		* \param  T
		*   This is an input/output parameter. The initial value of \c T
		*   gives the initial pose estimate of \c moving. When the
		*   algorithm terminates, \c T holds the registration result.
		*/
		bool match(NDTMap& target,
			NDTMap& source,
			Eigen::Transform<double, 3, Eigen::Affine, Eigen::ColMajor>& T,
			bool useInitialGuess = false);

		//compute the score gradient & hessian of a point cloud + transformation to an NDT (assuming a 2D transformation)
		// input: moving, fixed, tr, computeHessian
		//output: score_gradient, Hessian, returns: score!
		virtual double derivativesNDT_2d(
			const VectNDTCells &source,
			/*const */NDTMap &target,
			Eigen::MatrixXd &score_gradient,
			Eigen::MatrixXd &Hessian,
			bool computeHessian
		);

		// ����м������״̬
		void ClearMatchStatus();

#ifdef _MFC_VER
		void ShowStatus(CDC* pDC, CScreenReference& ScrnRef, int nCurStep, int nTotalSteps, unsigned long clrText);

		void Plot(CDC* pDc, CScreenReference& ScrnRef, unsigned long clrBorder, unsigned long clrFill,
			unsigned long clrBorderFail, unsigned long clrFillFail);

#elif defined QT_VERSION
		void ShowStatus(QPainter* pPainter, CScreenReference& ScrnRef, int nCurStep, int nTotalSteps, QColor clrText);

		void Plot(QPainter* pPainter, CScreenReference& ScrnRef, QColor clrBorder, QColor clrFill,
			QColor clrBorderFail, QColor clrFillFail);
#endif

	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	};

} // end namespace

#endif
