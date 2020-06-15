//                          - WORLD.CPP -
//
//	  Implementation of class "CWorld".
//
//   Author: Zhang Lei
//   Date:   2001. 7. 31
//

#include "stdafx.h"
#include <assert.h>
#include "World.h"
#include "LinePath.h"
#include "GenericPath.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif

CNode *pStartNode, *pEndNode;

//////////////////////////////////////////////////////////////////////////////
//   Implementation of class "CWorld".

//
//   Get the project data of a specified posture to a specified path.
//
bool CWorld::GetProjectData(CPosture& pst, CPath* pPath, CProjectData* pData)
{
	assert(pPath != NULL);
	CTraj* pTraj = pPath->MakeTraj();
	assert(pTraj != NULL);
	bool bResult = pTraj->ProjectPosture(pst, pData);
	delete pTraj;
	return bResult;
}

//
//   Get the project data of a specified posture to the map.
//
bool CWorld::GetProjectData(CPosture& pst, CProjectData* pData)
{
	USHORT u = 0xFFFF;
	CProjectData ResultData, TempData;
	bool bResult = false;
	
	for (USHORT i = 0; i < CPathBase::m_uCount; i++)
	{
		if (GetProjectData(pst, m_pPathIdx[i].m_ptr, &TempData))
		{
			if (u == 0xFFFF || TempData < ResultData)
			
			{
				ResultData = TempData;
				u = i;
				bResult = true;
			}
		}
	}

	return bResult;
}

//
//   Calculate the posture according to the given parameters.
//
bool CWorld::FindPosture(USHORT uStartNode, USHORT uEndNode, float fProgress, 
                         CPosture& pst)
{
   // If the start node and the end node are equal, the posture is on node
	if (uStartNode == uEndNode)
	{
		CNode* pNode = GetNode(uStartNode);
		if (pNode == NULL)
			return false;

      pst.SetPnt(pNode->GetPntObject());
		USHORT uNeighborNode = GetNeighborNode(uStartNode);
		CPath* pPath = GetPathPointer(uStartNode, uNeighborNode);
		CAngle angHeading = pPath->GetHeading(*pNode);
		pst.fThita = angHeading.m_fRad;
		return true;
	}

   // Else, the posture is on a path
	else
	{
		CPath* pPath = GetPathPointer(uStartNode, uEndNode);
		if (pPath == NULL)
			return false;

      // If the path's start node is not the given "uStartNode", reverse the direction
		if (pPath->m_uStartNode == uEndNode)
			fProgress = pPath->Size() - fProgress;

      // Create a temporary trajectory object
		CTraj* pTraj = pPath->MakeTraj();

      // Calculate the posture according to the trajectory
		pTraj->SetProgress(0.01f, fProgress);
		pst = pTraj->PostureFun();

		delete pTraj;
		return true;
	}
}

//
//   ȡ����ָ���ڵ�ĳ�����̬(�����ж��)
//
int CWorld::GetNodeHeadingAngle(int nNodeId, CAngle* pAngles, int nMaxNum)
{
	USHORT* pBuf = new USHORT[nMaxNum];

	// ȡ����ýڵ��������ڽڵ�ı��
	int nCount = GetAllNeighborNodes(nNodeId, pBuf, nMaxNum);
	CNode* pNode = GetNode(nNodeId);

	// �����жϸ�����·�ε���ֹ��̬
	for (int i = 0; i < nCount; i++)
	{
		CPath* pPath = CPathBase::GetPathPointer(nNodeId, pBuf[i]);
		if (pPath == NULL)
			return 0;
		
		// ȡ�ø�·������ֹ��̬��
		pAngles[i] = pPath->GetHeading(*pNode);
	}

	// ����ɾ����־
	bool* bDelete = new bool[nCount];
	for (int i = 0; i < nCount; i++)
		bDelete[i] = false;

	// �˶��Ƿ�����ͬ�Ľǣ����У���ɾ������һ��
	for (int i = 0; i < nCount; i++)
	{
		if (bDelete[i])
			continue;

		for (int j = i + 1; j < nCount; j++)
		{
			if (bDelete[j])
				continue;

			if (pAngles[i] == pAngles[j])
				bDelete[j] = true;
		}
	}

	// ɾ������Ϊ��bDelete������
	int nNewCount = 0;
	for (int i = 0; i < nCount; i++)
	{
		if (!bDelete[i])
		{
			pAngles[nNewCount++] = pAngles[i];
		}
	}

	delete []bDelete;
	delete []pBuf;

	return nNewCount;
}

//
//   ���һ��ֱ��·����
bool CWorld::AddLinePath(CPnt& ptStart, CPnt& ptEnd, int& nStartNodeId, int& nEndNodeId)
{
	// ������(���ҵ�)�����ڵ�

	// ������IDδ�ṩ��˵������Ǹ��½ڵ�
	if (nStartNodeId < 0)
	{
		// �ڵ�ͼ�м����½ڵ�
		pStartNode = CNodeBase::AddNode(ptStart);
		if (pStartNode == NULL)
			return false;
	}
	else
	{
		// ���ݽڵ��ȡ�ýڵ�ָ��
		pStartNode = CNodeBase::GetNode(nStartNodeId);

		// ����ڵ㲻���ڣ�����false
		if (pStartNode == NULL)
			return false;
	}
	nStartNodeId = pStartNode->m_uId;


	// ����յ�ָ��δ�ṩ��˵���յ���½ڵ�
	if (nEndNodeId < 0)
	{
		// �ڵ�ͼ�м����½ڵ�
		pEndNode = CNodeBase::AddNode(ptEnd);

		// ����ڵ㲻���ڣ�����false
		if (pEndNode == NULL)
			return false;
	}
	else
	{
		// ���ݽڵ��ȡ�ýڵ�ָ��
		pEndNode = CNodeBase::GetNode(nEndNodeId);

		// ����ڵ㲻���ڣ�����false
		if (pEndNode == NULL)
			return false;
	}
	nEndNodeId = pEndNode->m_uId;

	// �����·��
	int nNextID = CPathBase::NextID();
	
	CLinePath* pPath = new CLinePath(nNextID, nStartNodeId, nEndNodeId);
	if (pPath == NULL)
		return false;
	
	return CPathBase::AddPath(pPath);
}

bool CWorld::AddGenericPath(CPosture& pstStart, CPosture& pstEnd, float fLen1, 
									 float fLen2, int& nStartNodeId, int& nEndNodeId)
{

	// ������(���ҵ�)�����ڵ�

	// ������IDδ�ṩ��˵������Ǹ��½ڵ�
	if (nStartNodeId < 0)
	{
		// �ڵ�ͼ�м����½ڵ�
		pStartNode = CNodeBase::AddNode(pstStart.GetPntObject());
		if (pStartNode == NULL)
			return false;
	}
	else
	{
		// ���ݽڵ��ȡ�ýڵ�ָ��
		pStartNode = CNodeBase::GetNode(nStartNodeId);

		// ����ڵ㲻���ڣ�����false
		if (pStartNode == NULL)
			return false;
	}
	nStartNodeId = pStartNode->m_uId;


	// ����յ�ָ��δ�ṩ��˵���յ���½ڵ�
	if (nEndNodeId < 0)
	{
		// �ڵ�ͼ�м����½ڵ�
		pEndNode = CNodeBase::AddNode(pstEnd.GetPntObject());

		// ����ڵ㲻���ڣ�����false
		if (pEndNode == NULL)
			return false;
	}
	else
	{
		// ���ݽڵ��ȡ�ýڵ�ָ��
		pEndNode = CNodeBase::GetNode(nEndNodeId);

		// ����ڵ㲻���ڣ�����false
		if (pEndNode == NULL)
			return false;
	}
	nEndNodeId = pEndNode->m_uId;

	// �����·��
	int nNextID = CPathBase::NextID();
	
	CGenericPath* pPath = new CGenericPath(nNextID, nStartNodeId, nEndNodeId, pstStart, pstEnd, fLen1, fLen2);
	if (pPath == NULL)
		return false;
	
	return CPathBase::AddPath(pPath);
}


bool CWorld::Create(FILE *StreamIn)
{
	CString str;
	
#ifdef WORLD_FORMAT_VER2_0
/*
	// Get the tag character
	BYTE chTag;
	ar >> chTag;
	ASSERT(chTag == 'W');

	// Get the version number
	ar >> Obj.m_uVersion;
	ar >> Obj.m_tmStamp;
	ar >> str;
	
	strcpy(Obj.m_strNote, (const char*)str);
*/
#endif      // WORLD_FORMAT_VER2_0

	CPath::m_pNodeBase = &(GetNodeBaseObject());

	// Step 1: Init the node bank data
   if(!CNodeBase::Create(StreamIn))
      return false;

	CPathBase& PathBase = *GetPathBaseObject();

	// Step 2: Init the path bank data
   if (!PathBase.Create(StreamIn))
      return false;

	return true;
}

bool CWorld::Save(FILE *StreamOut)
{
#ifdef WORLD_FORMAT_VER2_0
/*	// Save the tag character
	BYTE chTag = 'W';
	ar << chTag;

	// Save the version number
	Obj.m_uVersion = 1;
	ar << Obj.m_uVersion;

	// Save the time stamp
	time(&Obj.m_tmStamp);
	ar << Obj.m_tmStamp;

	// Save the description
	CString str = Obj.m_strNote;
	ar << str;
*/
#endif       // WORLD_FORMAT_VER2_0

	// Step 1: Save the node bank data
   if(!CNodeBase::Save(StreamOut))
      return false;

	// Step 2: Save the path bank data
   if (!CPathBase::Save(StreamOut))
      return false;
   return true;
}

#ifdef _MFC_VER

CArchive& operator >> (CArchive& ar, CWorld& Obj)
{
	CString str;
	
#ifdef WORLD_FORMAT_VER2_0
/*
	// Get the tag character
	BYTE chTag;
	ar >> chTag;
	ASSERT(chTag == 'W');

	// Get the version number
	ar >> Obj.m_uVersion;
	ar >> Obj.m_tmStamp;
	ar >> str;
	
	strcpy(Obj.m_strNote, (const char*)str);
*/
#endif      // WORLD_FORMAT_VER2_0
	USHORT u;    // У���
	ar >> u;
	CPath::m_pNodeBase = &(Obj.GetNodeBaseObject());

	// Step 1: Init the node bank data
	ar >> *CPath::m_pNodeBase;

	// Step 2: Init the path bank data
	CPathBase& PathBase = *Obj.GetPathBaseObject();
	ar >> PathBase;

	return ar;
}

CArchive& operator << (CArchive& ar, CWorld& Obj)
{
#ifdef WORLD_FORMAT_VER2_0
/*	// Save the tag character
	BYTE chTag = 'W';
	ar << chTag;

	// Save the version number
	Obj.m_uVersion = 1;
	ar << Obj.m_uVersion;

	// Save the time stamp
	time(&Obj.m_tmStamp);
	ar << Obj.m_tmStamp;

	// Save the description
	CString str = Obj.m_strNote;
	ar << str;
*/
#endif       // WORLD_FORMAT_VER2_0

	// Step 1: Init the node bank data
	CNodeBase& NodeBase = Obj.GetNodeBaseObject();
	ar << NodeBase;

	// Step 2: Init the path bank data
	CPathBase& PathBase = *Obj.GetPathBaseObject();
	ar << PathBase;

	return ar;
}

void CWorld::Draw(CScreenReference& ScrnRef, CDC* pDC, LOGFONT* pLogFont, COLORREF crNode, COLORREF crPath)
{
	if (m_bShowNodeID)
	   CNodeBase::DrawNodeID(ScrnRef, pDC, pLogFont);

	CPathBase::Draw(ScrnRef, pDC, crPath);
	CNodeBase::Draw(ScrnRef, pDC, crNode);
}
#endif
