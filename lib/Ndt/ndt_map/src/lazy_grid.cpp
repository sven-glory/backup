#include <cstring>
#include <cstdio>
#include <ndt_map/lazy_grid.h>

namespace perception_oru
{
	// ����������դ��
	LazyGrid::LazyGrid(double cellSize) : protoType(NULL)
	{
		initialized = false;
		centerIsSet = false;
		sizeIsSet = false;
		cellSizeX = cellSizeY = cellSizeZ = cellSize;
		m_rect.Clear();
	}

	// ����������դ��
	LazyGrid::LazyGrid(double cellSizeX_, double cellSizeY_, double cellSizeZ_)
	{
		initialized = false;
		centerIsSet = false;
		sizeIsSet = false;
		cellSizeX = cellSizeX_;
		cellSizeY = cellSizeY_;
		cellSizeZ = cellSizeZ_;
		m_rect.Clear();
	}

	// ��������һ��������
	LazyGrid::LazyGrid(LazyGrid *another)
	{
		m_rect.Clear();

		sizeXmeters = another->sizeXmeters;
		sizeYmeters = another->sizeYmeters;
		sizeZmeters = another->sizeZmeters;

		cellSizeX = another->cellSizeX;
		cellSizeY = another->cellSizeY;
		cellSizeZ = another->cellSizeZ;

		cellsCountX = abs(ceil(sizeXmeters / cellSizeX));
		cellsCountY = abs(ceil(sizeYmeters / cellSizeY));
		cellsCountZ = abs(ceil(sizeZmeters / cellSizeZ));

		centerX = another->centerX;
		centerY = another->centerY;
		centerZ = another->centerZ;

		protoType = another->protoType->clone();
		initialize();
	}

	// ���������������й���
	LazyGrid::LazyGrid(double _sizeXmeters, double _sizeYmeters, double _sizeZmeters,
		double _cellSizeX, double _cellSizeY, double _cellSizeZ,
		double _centerX, double _centerY, double _centerZ,
		NDTCell *cellPrototype)
	{
		m_rect.Clear();

		sizeXmeters = _sizeXmeters;
		sizeYmeters = _sizeYmeters;
		sizeZmeters = _sizeZmeters;

		cellSizeX = _cellSizeX;
		cellSizeY = _cellSizeY;
		cellSizeZ = _cellSizeZ;

		cellsCountX = abs(ceil(sizeXmeters / cellSizeX));
		cellsCountY = abs(ceil(sizeYmeters / cellSizeY));
		cellsCountZ = abs(ceil(sizeZmeters / cellSizeZ));

		centerX = _centerX;
		centerY = _centerY;
		centerZ = _centerZ;

		protoType = cellPrototype->clone();
		initialize();
	}

	LazyGrid::~LazyGrid()
	{
		if (initialized)
		{
			int cnt = 0;

			//go through all cells and delete the non-NULL ones
			for (unsigned int i = 0; i < activeCells.size(); ++i)
			{
				if (activeCells[i])
				{
					delete activeCells[i];
					cnt++;
				}
			}

			// �ͷ�����NDT��Ԫ
			for (int i = 0; i < cellsCountX; i++)
			{
				for (int j = 0; j < cellsCountY; j++)
					delete[] dataArray[i][j];

				delete[] dataArray[i];
			}

			delete[] dataArray;

			if (protoType != NULL)
				delete protoType;
		}
	}

	void LazyGrid::setCenter(double cx, double cy, double cz)
	{
		centerX = cx;
		centerY = cy;
		centerZ = cz;

		centerIsSet = true;
		if (sizeIsSet)
		{
			initialize();
		}
	}

	void LazyGrid::setSize(double sx, double sy, double sz)
	{
		sizeXmeters = sx;
		sizeYmeters = sy;
		sizeZmeters = sz;

		cellsCountX = abs(ceil(sizeXmeters / cellSizeX));
		cellsCountY = abs(ceil(sizeYmeters / cellSizeY));
		cellsCountZ = abs(ceil(sizeZmeters / cellSizeZ));

		sizeIsSet = true;
		if (centerIsSet)
		{
			initialize();
		}
	}

	void LazyGrid::initializeAll()
	{
		if (!initialized)
			this->initialize();

		int idcX, idcY, idcZ;
		pcl::PointXYZ center;
		center.x = centerX;
		center.y = centerY;
		center.z = centerZ;
		this->getIndexForPoint(center, idcX, idcY, idcZ);

		pcl::PointXYZ cellCenter;
		for (int i = 0; i < cellsCountX; i++)
		{
			for (int j = 0; j < cellsCountY; j++)
			{
				for (int k = 0; k < cellsCountZ; k++)
				{
					dataArray[i][j][k] = new NDTCell();
					dataArray[i][j][k]->setDimensions(cellSizeX, cellSizeY, cellSizeZ);

					cellCenter.x = centerX + (i - idcX)*cellSizeX;
					cellCenter.y = centerY + (j - idcY)*cellSizeY;
					cellCenter.z = centerZ + (k - idcZ)*cellSizeZ;

					dataArray[i][j][k]->setCenter(cellCenter);
					activeCells.push_back(dataArray[i][j][k]);
				}
			}
		}
	}

	// ΪLazyGrid����NDT��Ԫ�ռ�
	void LazyGrid::initialize()
	{
		if (initialized)
			return;

		dataArray = new NDTCell***[cellsCountX];
		for (int i = 0; i < cellsCountX; i++)
		{
			dataArray[i] = new NDTCell**[cellsCountY];
			for (int j = 0; j < cellsCountY; j++)
			{
				dataArray[i][j] = new NDTCell*[cellsCountZ];
				//set all cells to NULL
				memset(dataArray[i][j], 0, cellsCountZ * sizeof(NDTCell*));
			}
		}

		initialized = true;
	}

	//
	//   ���ݸ����Ŀռ��λ�ã�ȡ��ָ���Ӧ��NDT��Ԫ��ָ�롣
	//
	NDTCell* LazyGrid::getCellForPoint(const pcl::PointXYZ &point)
	{
		// ��ȡ�øõ����ڵ�Ԫ��X��Y��Z�����ϵ�����ֵ
		int indX, indY, indZ;
		this->getIndexForPoint(point, indX, indY, indZ);

		// ��֤����ֵ��Χ
		if (indX >= cellsCountX || indY >= cellsCountY || indZ >= cellsCountZ || indX < 0 || indY < 0 || indZ < 0)
			return NULL;

		if (!initialized || dataArray == NULL || dataArray[indX] == NULL || dataArray[indX][indY] == NULL)
			return NULL;

		return dataArray[indX][indY][indZ];
	}

	//
	//
	//
	NDTCell* LazyGrid::getCellAtAllocate(const pcl::PointXYZ &pt)
	{
		// ��֤�ռ�����Ч��
		pcl::PointXYZ point = pt;
		if (std::isnan(point.x) || std::isnan(point.y) || std::isnan(point.z))
			return NULL;

		// ȡ�ö�Ӧ�ڸõ��NDT��Ԫ��X��Y��Z�����ϵ�������ַ
		int indX, indY, indZ;
		this->getIndexForPoint(point, indX, indY, indZ);

		// ��֤����ֵ�ĺϷ���
		if (indX >= cellsCountX || indY >= cellsCountY || indZ >= cellsCountZ || indX < 0 || indY < 0 || indZ < 0)
			return NULL;

		// ��Ԫ��ַ�������Ѿ��������
		if (!initialized || dataArray == NULL || dataArray[indX] == NULL || dataArray[indX][indY] == NULL)
			return NULL;

		// �����NDT��Ԫ�ǿյģ��������øõ�Ԫ��������
		if (dataArray[indX][indY][indZ] == NULL)
		{
			// �����������µ�Ԫ��������ߴ�
			dataArray[indX][indY][indZ] = protoType->clone();
			dataArray[indX][indY][indZ]->setDimensions(cellSizeX, cellSizeY, cellSizeZ);

			pcl::PointXYZ mapCenter;
			mapCenter.x = centerX;
			mapCenter.y = centerY;
			mapCenter.z = centerZ;

			// ��������LazyGridͼ�����ĵ������ֵ
			int idcX, idcY, idcZ;
			this->getIndexForPoint(mapCenter, idcX, idcY, idcZ);
			
			// ���㲢���ø��µ�Ԫ�����ĵ������
			pcl::PointXYZ cellCenter;
			cellCenter.x = centerX + (indX - idcX) * cellSizeX;
			cellCenter.y = centerY + (indY - idcY) * cellSizeY;
			cellCenter.z = centerZ + (indZ - idcZ) * cellSizeZ;
			dataArray[indX][indY][indZ]->setCenter(cellCenter);

			// ��¼��Ϊһ�����Ч�ĵ�Ԫ
			activeCells.push_back(dataArray[indX][indY][indZ]);
		}

		return dataArray[indX][indY][indZ];
	}

	//
	//   ��LazyGrid�м���һ���ռ�㡣
	//
	NDTCell* LazyGrid::addPoint(const pcl::PointXYZ &point_c)
	{
		// ���ݸ����ĵ㣬ȡ�ö�Ӧ��NDT��Ԫ
		NDTCell* cell = this->getCellAtAllocate(point_c);
		if (cell == NULL)
			return NULL;

		// ���õ��¼����NDT��Ԫ��ȥ
		cell->addPoint(point_c);

		return cell;
	}

	typename SpatialIndex::CellVectorItr LazyGrid::begin()
	{
		return activeCells.begin();
	}

	typename SpatialIndex::CellVectorConstItr LazyGrid::begin() const
	{
		return activeCells.begin();
	}

	typename SpatialIndex::CellVectorItr LazyGrid::end()
	{
		return activeCells.end();
	}

	typename SpatialIndex::CellVectorConstItr LazyGrid::end() const
	{
		return activeCells.end();
	}

	int LazyGrid::size()
	{
		return activeCells.size();
	}

	SpatialIndex* LazyGrid::clone() const
	{
		return new LazyGrid(cellSizeX);
	}

	//
	//   ���Ʊ�LazyGrid���õ�һ��������
	//
	SpatialIndex* LazyGrid::copy() const
	{
		// Ϊ��LazyGrid����ռ�
		LazyGrid *ret = new LazyGrid(cellSizeX);   // �������������?

		// ���������л��Ч�ĵ�Ԫ
		typename std::vector<NDTCell*>::const_iterator it;
		for (it = activeCells.begin(); it != activeCells.end(); it++)
		{
			NDTCell* r = (*it);
			if (r != NULL)
			{
				// ���ν������Ԫ�е�ԭʼ����뵽��LazyGrid��
				for (unsigned int i = 0; i < r->points_.size(); i++)
					ret->addPoint(r->points_[i]);
			}
		}

		return ret;
	}

	//
	//   �Ը����Ŀռ��Ϊ���ģ�radiusΪ�뾶��Ѱ�������ڽ���NDT��Ԫ��
	//
	void LazyGrid::getNeighbors(const pcl::PointXYZ &point, double radius, std::vector<NDTCell*> &cells)
	{
		// ȡ�øõ�����Ӧ������ֵ
		int indX, indY, indZ;
		this->getIndexForPoint(point, indX, indY, indZ);

		// �������ֵ���磬�򷵻ؿձ�
		if (indX >= cellsCountX || indY >= cellsCountY || indZ >= cellsCountZ)
		{
			cells.clear();
			return;
		}

		// �������ռ��ڽ��ĵ�Ԫ
		for (int x = indX - radius / cellSizeX; x <= indX + radius / cellSizeX; x++)
		{
			if (x < 0 || x >= cellsCountX)
				continue;

			for (int y = indY - radius / cellSizeY; y <= indY + radius / cellSizeY; y++)
			{
				if (y < 0 || y >= cellsCountY) 
					continue;

				for (int z = indZ - radius / cellSizeZ; z <= indZ + radius / cellSizeZ; z++)
				{
					if (z < 0 || z >= cellsCountZ) 
						continue;

					if (dataArray[x][y][z] == NULL) 
						continue;

					cells.push_back(dataArray[x][y][z]);
				}
			}
		}
	}

	//
	//   ���ݸ����Ŀռ���λ�ã������Ӧ��NDT��Ԫ�ĵ�ַ��
	//
	void LazyGrid::getIndexForPoint(const pcl::PointXYZ& point, int &indX, int &indY, int &indZ)
	{
		indX = floor((point.x - centerX) / cellSizeX + 0.5) + cellsCountX / 2.0;
		indY = floor((point.y - centerY) / cellSizeY + 0.5) + cellsCountY / 2.0;
		indZ = floor((point.z - centerZ) / cellSizeZ + 0.5) + cellsCountZ / 2.0;
	}

	//
	//   �������������ڽ����벻����n_neigh�����е�Ԫ����������Щ��Ԫ���б�
	//
	std::vector< NDTCell* > LazyGrid::getClosestNDTCells(const pcl::PointXYZ &point, int &n_neigh, bool checkForGaussian)
	{
		int indX, indY, indZ;
		this->getIndexForPoint(point, indX, indY, indZ);

		std::vector<NDTCell*> cells;
		int indXn, indYn, indZn;

		//the strange thing for the indeces is for convenience of writing
		//basicly, we go through 2* the number of cells and use parity to
		//decide if we subtract or add. should work nicely
		int i = n_neigh; //how many cells on each side
		for (int x = 1; x < 2 * i + 2; x++)
		{
			indXn = (x % 2 == 0) ? indX + x / 2 : indX - x / 2;
			for (int y = 1; y < 2 * i + 2; y++)
			{
				indYn = (y % 2 == 0) ? indY + y / 2 : indY - y / 2;
				for (int z = 1; z < 2 * i + 2; z++)
				{
					indZn = (z % 2 == 0) ? indZ + z / 2 : indZ - z / 2;

					// ����õ�Ԫ����NDT��Ԫ���򽫸õ�Ԫ���뷵���б�
					if (checkCellforNDT(indXn, indYn, indZn, checkForGaussian))
					{
						cells.push_back(dataArray[indXn][indYn][indZn]);
					}
				}
			}
		}

		return cells;
	}

	//
	//   ���������������ĵ�Ԫ����������ָ�롣
	//
	NDTCell* LazyGrid::getClosestNDTCell(const pcl::PointXYZ &point, bool checkForGaussian)
	{
		int indXn, indYn, indZn;
		int indX, indY, indZ;
		this->getIndexForPoint(point, indX, indY, indZ);
		NDTCell *ret = NULL;
		std::vector<NDTCell*> cells;

		if (!checkForGaussian)
		{
			//just give me whatever is in this cell
			if (checkCellforNDT(indX, indY, indZ, checkForGaussian))
			{
				ret = (dataArray[indX][indY][indZ]);
			}
			return ret;
		}

		int i = 1; //how many cells on each side

		//for(int i=1; i<= maxNumberOfCells; i++) {
		//the strange thing for the indeces is for convenience of writing
		//basicly, we go through 2* the number of cells and use parity to
		//decide if we subtract or add. should work nicely
		for (int x = 1; x < 2 * i + 2; x++)
		{
			indXn = (x % 2 == 0) ? indX + x / 2 : indX - x / 2;
			for (int y = 1; y < 2 * i + 2; y++)
			{
				indYn = (y % 2 == 0) ? indY + y / 2 : indY - y / 2;
				for (int z = 1; z < 2 * i + 2; z++)
				{
					indZn = (z % 2 == 0) ? indZ + z / 2 : indZ - z / 2;
					if (checkCellforNDT(indXn, indYn, indZn))
					{
						ret = (dataArray[indXn][indYn][indZn]);
						cells.push_back(ret);
					}
				}
			}
		}

		double minDist = INT_MAX;
		Eigen::Vector3d tmean;
		pcl::PointXYZ pt = point;
		for (unsigned int i = 0; i < cells.size(); i++)
		{
			tmean = cells[i]->getMean();
			tmean(0) -= pt.x;
			tmean(1) -= pt.y;
			tmean(2) -= pt.z;
			double d = tmean.norm();
			if (d < minDist)
			{
				minDist = d;
				ret = cells[i];
			}
		}
		cells.clear();
		return ret;
	}

	//
	//   �˶�ָ��������λ���Ƿ�����Ӧ��NDT��Ԫ��
	//   ע�⣺�ú��������˶Ըõ�Ԫ�Ƿ��ǻ�ġ�
	//
	bool LazyGrid::checkCellforNDT(int indX, int indY, int indZ, bool checkForGaussian)
	{
		if (indX < cellsCountX && indY < cellsCountY && indZ < cellsCountZ && indX >= 0 && indY >= 0 && indZ >= 0)
		{
			if (dataArray[indX][indY][indZ] != NULL)
			{
				if (dataArray[indX][indY][indZ]->hasGaussian_ || (!checkForGaussian))
					return true;
			}
		}
		return false;
	}

	//
	//   ΪLazyGrid����NDT��Ԫ���塣
	//
	void LazyGrid::setCellType(NDTCell *type)
	{
		if (type != NULL)
		{
			// ��ԭ�������壬������Ҫɾ����
			if (protoType != NULL)
				delete protoType;

			// ����ռ䲢�����µ�����
			protoType = type->clone();
		}
	}

	void LazyGrid::getCellSize(double &cx, double &cy, double &cz)
	{
		cx = cellSizeX;
		cy = cellSizeY;
		cz = cellSizeZ;
	}

	void LazyGrid::getCenter(double &cx, double &cy, double &cz)
	{
		cx = centerX;
		cy = centerY;
		cz = centerZ;
	}

	void LazyGrid::getGridSize(int &cx, int &cy, int &cz)
	{
		cx = cellsCountX;
		cy = cellsCountY;
		cz = cellsCountZ;
	}

	void LazyGrid::getGridSizeInMeters(double &cx, double &cy, double &cz)
	{
		cx = sizeXmeters;
		cy = sizeYmeters;
		cz = sizeZmeters;
	}

	void LazyGrid::getGridSizeInZMeters(double &cz)
	{
		cz = sizeZmeters;
	}

	//
	// ���ļ���װ��LazyGrid��(ע�⣺ȱ��ͨ���ԣ�)
	//
	bool LazyGrid::Load(FILE* fp)
	{
		centerIsSet = false;
		sizeIsSet = false;

		float f[3];

		// ���뵥Ԫ�ߴ�(X/Y/Z����λ����)
		if (fread(f, sizeof(float), 3, fp) != 3)
			return false;
		cellSizeX = f[0]; cellSizeY = f[1];	cellSizeZ = f[2];

		// ���볡�����ߴ�(X/Y/Z����λ����)
		if (fread(f, sizeof(float), 3, fp) != 3)
			return false;

		setSize(f[0], f[1], f[2]);

		// ����ͼ������λ��(X/Y/Z����λ����)
		if (fread(f, sizeof(float), 3, fp) != 3)
			return false;
		setCenter(f[0], f[1], f[2]);

		// ����������
		int n;
		if (fread(&n, sizeof(int), 1, fp) != 1)
			return false;

		NDTCell prototype_(cellSizeX, cellSizeY, cellSizeZ);

		// װ�����е�Ԫ����
		while (!feof(fp) && prototype_.loadFromJFF(fp) >= 0)
		{
			if (!AddNewCell(prototype_))
				continue;
		}

		return true;
	}

	//
	//   ��LazyGrid���浽�ļ���
	//	
	bool LazyGrid::Save(FILE* fp)
	{
		// д�뵥Ԫ�ߴ�(X/Y/Z����λ����)
		float f[3] = { cellSizeX, cellSizeY, cellSizeZ };
		if (fwrite(f, sizeof(float), 3, fp) != 3)
			return false;

		// д�볡�����ߴ�(X/Y/Z����λ����)
		f[0] = sizeXmeters; f[1] = sizeYmeters; f[2] = sizeZmeters;
		if (fwrite(f, sizeof(float), 3, fp) != 3)
			return false;

		// д��ͼ������λ��(X/Y/Z����λ����)
		f[0] = centerX; f[1] = centerY; f[2] = centerZ;
		if (fwrite(f, sizeof(float), 3, fp) != 3)
			return false;

		// д����Ԫ����
		int n = size();
		if (fwrite(&n, sizeof(int), 1, fp) != 1)
			return false;

		// ����д������Ԫ������
		for (SpatialIndex::CellVectorItr it = begin(); it != end(); it++)
		{
			if ((*it)->writeToJFF(fp) < 0)
				return false;
		}

		return true;
	}

	//
	//   ����ɨ���ߣ��ռ����ϵ����е�Ԫ�����������cells�С�
	//
	bool LazyGrid::traceLine(const Eigen::Vector3d &origin, const pcl::PointXYZ &endpoint, const Eigen::Vector3d &diff_,
		const double& maxz, std::vector<NDTCell*> &cells)
	{
		if (endpoint.z > maxz)
			return false;

		double min1 = std::min(cellSizeX, cellSizeY);
		double min2 = std::min(cellSizeZ, cellSizeY);
		double resolution = std::min(min1, min2); // Select the smallest resolution

		if (resolution < 0.01)
		{
			fprintf(stderr, "Resolution very very small (%lf) :( \n", resolution);
			return false;
		}

		double l = diff_.norm();
		int N = l / (resolution);

		NDTCell* ptCell = NULL;

		pcl::PointXYZ po;
		po.x = origin(0); 
		po.y = origin(1);
		po.z = origin(2);

		Eigen::Vector3d diff = diff_ / (float)N;

		pcl::PointXYZ pt;
		int idxo = 0, idyo = 0, idzo = 0;
		for (int i = 0; i < N - 2; i++)
		{
			pt.x = origin(0) + ((float)(i + 1)) * diff(0);
			pt.y = origin(1) + ((float)(i + 1)) * diff(1);
			pt.z = origin(2) + ((float)(i + 1)) * diff(2);

			int idx, idy, idz;
			idx = floor((pt.x - centerX) / cellSizeX + 0.5) + cellsCountX / 2.0;
			idy = floor((pt.y - centerY) / cellSizeY + 0.5) + cellsCountY / 2.0;
			idz = floor((pt.z - centerZ) / cellSizeZ + 0.5) + cellsCountZ / 2.0;

			// We only want to check every cell once, so
			// increase the index if we are still in the same cell
			if (idx == idxo && idy == idyo && idz == idzo)
			{
				continue;
			}
			else
			{
				idxo = idx;
				idyo = idy;
				idzo = idz;
			}

			if (idx < cellsCountX && idy < cellsCountY && idz < cellsCountZ && idx >= 0 && idy >= 0 && idz >= 0) 
			{
				ptCell = dataArray[idx][idy][idz];
				if (ptCell != NULL) 
					cells.push_back(ptCell);
				else 
					this->addPoint(pt); // Add fake point to initialize!
			}
		}
		return true;
	}

	// ɾ��ָ����ŵĵ�Ԫ
	bool LazyGrid::deleteCell(int nIdx)
	{
		delete activeCells[nIdx];
		activeCells.erase(activeCells.begin() + nIdx);
		return true;
	}

	// ȡ��ָ����ŵĵ�Ԫ��ָ��
	NDTCell* LazyGrid::getCell(int nIdx)
	{
		return activeCells[nIdx];
	}

	// ��ͼ�м����µ�NDT��Ԫ
	bool LazyGrid::AddNewCell(NDTCell& newCell)
	{
		if (!initialized || dataArray == NULL)
			return false;

		pcl::PointXYZ cellCenter = newCell.getCenter();

		int indX, indY, indZ;
		this->getIndexForPoint(cellCenter, indX, indY, indZ);

		if (indX < 0 || indX >= cellsCountX)
			return false;

		if (indY < 0 || indY >= cellsCountY)
			return false;

		if (indZ < 0 || indZ >= cellsCountZ)
			return false;

		if (dataArray[indX] == NULL || dataArray[indX][indY] == NULL)
			return false;

		// ����ڸ�λ��ԭ���ʹ���NDT��Ԫ
		if (dataArray[indX][indY][indZ] != NULL)
		{
			NDTCell* ret = dataArray[indX][indY][indZ];

			ret->setDimensions(cellSizeX, cellSizeY, cellSizeZ);
			ret->setCenter(cellCenter);
			ret->setMean(newCell.getMean());
			ret->setCov(newCell.getCov());
			ret->setN(newCell.getN());
			ret->setOccupancy(newCell.getOccupancy());
			ret->hasGaussian_ = newCell.hasGaussian_;
		}
		// ��λ��û��NDT��Ԫ���ڣ��������
		else
		{
			//initialize cell
			dataArray[indX][indY][indZ] = newCell.copy();
			activeCells.push_back(dataArray[indX][indY][indZ]);

			// ����¼���ĵ�Ԫ����Ҫ������������ķ�Χ
			CPnt ptNew(cellCenter.x, cellCenter.y);
			m_rect += ptNew;
		}

		return true;
	}

	//
	//   ���±߽�ֵ��
	//
	void LazyGrid::UpdateCoveringRect()
	{
		m_rect.Clear();

		// ����д������Ԫ������
		for (SpatialIndex::CellVectorItr it = begin(); it != end(); it++)
		{
			pcl::PointXYZ cellCenter = (*it)->getCenter();
			CPnt pt(cellCenter.x, cellCenter.y);
			m_rect += pt;
		}
	}

} //end namespace
