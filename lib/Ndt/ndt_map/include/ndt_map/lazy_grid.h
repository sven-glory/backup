#pragma once

#include <stdio.h>
#include <ndt_map/spatial_index.h>
#include <ndt_map/ndt_cell.h>
#include "Geometry.h"

#define DEFAULT_MAP_SIZE_X               500               // ��ͼȱʡ���Ϊ200m
#define DEFAULT_MAP_SIZE_Y               500               // ��ͼȱʡ�߶�Ϊ200m
#define DEFAULT_MAP_RESO                 0.2               // ��ͼȱʡ�ֱ���0.2m


namespace perception_oru
{
	/** \brief A spatial index represented as a grid map
	\details A grid map with delayed allocation of cells.
	*/
	class LazyGrid : public SpatialIndex
	{
	protected:
		bool initialized;
		NDTCell ****dataArray;       // NDT��Ԫ��������ָ��
		NDTCell *protoType;          // ����NDT��Ԫ�������մ����������µ�Ԫ
		std::vector<NDTCell*> activeCells;

		bool centerIsSet;            // ��������ĵ��Ƿ�������
		bool sizeIsSet;              // ����ķ�Χ�Ƿ�������
		double sizeXmeters;          // �����X����ߴ�(m)
		double sizeYmeters;          // �����Y����ߴ�(m)
		double sizeZmeters;          // �����Z����ߴ�(m)
		double cellSizeX;            // NDT��ԪX����ߴ�
		double cellSizeY;            // NDT��ԪY����ߴ�
		double cellSizeZ;            // NDT��ԪZ����ߴ�
		double centerX;              // �������ĵ��X����
		double centerY;              // �������ĵ��Y����
		double centerZ;              // �������ĵ��Z����
		int cellsCountX;             // ����X�����Ϻ���NDT��Ԫ������
		int cellsCountY;             // ����Y�����Ϻ���NDT��Ԫ������
		int cellsCountZ;             // ����Z�����Ϻ���NDT��Ԫ������
		CRectangle m_rect;           // ��Ч��������

	protected:
		// ���±߽�ֵ
		void UpdateCoveringRect();

	public:
		// ����������դ��
		LazyGrid(double cellSize = DEFAULT_MAP_RESO);

		// ����������դ��
		LazyGrid(double cellSizeX_, double cellSizeY_, double cellSizeZ_);

		// ��������һ��������
		LazyGrid(LazyGrid *another);

		// ���������������й���
		LazyGrid(double sizeXmeters, double sizeYmeters, double sizeZmeters,
			double cellSizeX, double cellSizeY, double cellSizeZ,
			double _centerX, double _centerY, double _centerZ,
			NDTCell *cellPrototype);

		virtual ~LazyGrid();

		virtual NDTCell* getCellForPoint(const pcl::PointXYZ &point);
		virtual NDTCell* addPoint(const pcl::PointXYZ &point);

		//these two don't make much sense...
		///iterator through all cells in index, points at the begining
		virtual typename SpatialIndex::CellVectorItr begin();
		virtual typename SpatialIndex::CellVectorConstItr begin() const;
		///iterator through all cells in index, points at the end
		virtual typename SpatialIndex::CellVectorItr end();
		virtual typename SpatialIndex::CellVectorConstItr end() const;
		virtual int size();

		///clone - create an empty object with same type
		virtual SpatialIndex* clone() const;
		virtual SpatialIndex* copy() const;

		///method to return all cells within a certain radius from a point
		virtual void getNeighbors(const pcl::PointXYZ &point, double radius, std::vector<NDTCell*> &cells);

		///sets the cell factory type
		virtual void setCellType(NDTCell *type);

		virtual void setCenter(double cx, double cy, double cz);
		virtual void setSize(double sx, double sy, double sz);
		
		// ɾ��ָ����ŵĵ�Ԫ
		bool deleteCell(int nIdx);
		
		// ȡ��ָ����ŵĵ�Ԫ��ָ��
		NDTCell* getCell(int nIdx);

		virtual NDTCell* getClosestNDTCell(const pcl::PointXYZ &pt, bool checkForGaussian = true);
		virtual std::vector<NDTCell*> getClosestNDTCells(const pcl::PointXYZ &pt, int &n_neigh, bool checkForGaussian = true);

		virtual inline void getCellAt(int indX, int indY, int indZ, NDTCell* &cell)
		{
			if (indX < cellsCountX && indY < cellsCountY && indZ < cellsCountZ && indX >= 0 && indY >= 0 && indZ >= 0)
			{
				cell = dataArray[indX][indY][indZ];
			}
			else 
			{
				cell = NULL;
			}
		}

		virtual inline void getCellAt(const pcl::PointXYZ& pt, NDTCell* &cell) 
		{
			int indX, indY, indZ;
			this->getIndexForPoint(pt, indX, indY, indZ);
			this->getCellAt(indX, indY, indZ, cell);
		}


		///automatically allocate the cell if needed (checks that the indexes etc. are correct).
		NDTCell* getCellAtAllocate(const pcl::PointXYZ& pt);

		//FIXME: these two are now not needed anymore
		virtual inline void getNDTCellAt(int indX, int indY, int indZ, NDTCell* &cell) 
		{
			if (indX < cellsCountX && indY < cellsCountY && indZ < cellsCountZ && indX >= 0 && indY >= 0 && indZ >= 0) 
			{
				cell = (dataArray[indX][indY][indZ]);
			}
			else 
			{
				cell = NULL;
			}
		}

		virtual inline void getNDTCellAt(const pcl::PointXYZ& pt, NDTCell* &cell) 
		{
			int indX, indY, indZ;
			this->getIndexForPoint(pt, indX, indY, indZ);
			this->getNDTCellAt(indX, indY, indZ, cell);
		}

		void getCellSize(double &cx, double &cy, double &cz);
		void getGridSize(int &cx, int &cy, int &cz);
		void getGridSizeInMeters(double &cx, double &cy, double &cz);
		void getCenter(double &cx, double &cy, double &cz);
		void getGridSizeInZMeters(double &cz);
		virtual void getIndexForPoint(const pcl::PointXYZ& pt, int &idx, int &idy, int &idz);
		
		NDTCell* getProtoType()
		{
			return protoType;
		}

		virtual void initialize();
		virtual void initializeAll();

		NDTCell ****getDataArrayPtr()
		{
			return dataArray;
		}

		// ���ļ���װ��LazyGrid
		virtual bool Load(FILE * fp);

		// ��LazyGrid���浽�ļ�
		virtual bool Save(FILE* fp);

		bool traceLine(const Eigen::Vector3d &origin, const pcl::PointXYZ &endpoint, const Eigen::Vector3d &diff, const double& maxz, std::vector<NDTCell*> &cells);

		bool isInside(const pcl::PointXYZ& pt) 
		{
			int indX, indY, indZ;
			this->getIndexForPoint(pt, indX, indY, indZ);
			return(indX < cellsCountX && indY < cellsCountY && indZ < cellsCountZ && indX >= 0 && indY >= 0 && indZ >= 0);
		}

		virtual bool checkCellforNDT(int indX, int indY, int indZ, bool checkForGaussian = true);

		// ��ͼ�м����µ�NDT��Ԫ
		bool AddNewCell(NDTCell& newCell);

		// ȡ�ø�������
		CRectangle GetCoveringRect() const { return m_rect; }

	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	};
};

