#ifndef __CMapWorkSpace
#define __CMapWorkSpace

#include "SlamDataSet.h"
#include "ndt_map/ndt_map.h"
#include "VectPose.h"


class CMapWorkSpace
{
public:
	CSlamDataSet  m_dataset;                // ���ݼ�
	perception_oru::NDTMap m_ndtMap;		    // NDTͼ
	CVectPose m_vectPose;                   // ��̬��¼

public:
	CMapWorkSpace() {}

	bool Load(FILE* fp);
	bool Save(FILE* fp, int nFileVersion = 251);

#ifdef _MFC_VER

	// ����ģ��ͼ��
	void PlotMap(CDC* pDC, CScreenReference& ScrnRef, bool bShowPoses = false);
#endif

};
#endif
