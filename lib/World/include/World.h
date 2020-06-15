#pragma once

#include <stdio.h>
#include <time.h>
#include "PathBase.h"

//////////////////////////////////////////////////////////////////////////////
//   The class of AGVS "Map".
class DllExport CWorld : public CNodeBase, public CPathBase
{
public:
	USHORT  m_uVersion;      // Version number
	time_t  m_tmStamp;       // Time stamp
	char    m_strNote[128];  // Note for the object
	bool    m_bShowNodeID;   // Show/hide node ID

public:
	// The default constructor
	CWorld() 
	{
		m_bShowNodeID = false;
		CPath::m_pNodeBase = &(GetNodeBaseObject());
	}

	// Get the project data of a specified posture to a specified path
	bool GetProjectData(CPosture& pst, CPath* pPath, CProjectData* pData);

	// Get the project data of a specified posture to the map
	bool GetProjectData(CPosture& pst, CProjectData* pData);
		
   // Calculate the posture according to the given parameters
   bool FindPosture(USHORT uStartNode, USHORT uEndNode, float fProgress, CPosture& pst);

	// ȡ����ָ���ڵ�ĳ�����̬(�����ж��)
	int GetNodeHeadingAngle(int nNodeId, CAngle* pAngles, int nMaxNum);

	// ���һ��ֱ��·��
	bool AddLinePath(CPnt& ptStart, CPnt& ptEnd, int& nStartNodeId, int& nEndNodeId);

	// ���һ��ͨ��·��
	bool AddGenericPath(CPosture& pstStart, CPosture& pstEnd, float fLen1, float fLen2, 
		int& nStartNodeId, int& nEndNodeId);

	void ShowNodeID(bool bShow = true) {m_bShowNodeID = bShow;}

	bool Create(FILE *StreamIn);
	bool Save(FILE *StreamOut);

#ifdef _MFC_VER
	friend DllExport CArchive& operator >> (CArchive& ar, CWorld& Obj);
	friend DllExport CArchive& operator << (CArchive& ar, CWorld& Obj);

   virtual void Draw(CScreenReference& ScrnRef, CDC* pDC, LOGFONT* pLogFont, COLORREF crNode, COLORREF crPath);
#endif
};
