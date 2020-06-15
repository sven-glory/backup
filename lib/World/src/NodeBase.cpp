#include "stdafx.h"
#include <assert.h>
#include "NodeBase.h"

//////////////////////////////////////////////////////////////////////////////
//   Implementation of class "CNodeBase".                                   *

//
//   The constructor of class "CNodeBase".
//
CNodeBase::CNodeBase()
{
	m_uCount = 0;
	m_paNode = NULL;
}

//
//   The destructor of class "CNodeBase".
//
CNodeBase::~CNodeBase()
{
	Clear();
}

void CNodeBase::Clear()
{
	if (m_paNode != NULL)
	{
		delete []m_paNode;
		m_uCount = 0;
		m_paNode = NULL;
	}
}

//
//    Get the specified node object.
//
CNode* CNodeBase::GetNode(USHORT uId)
{
	// Make sure the node base was created
	if (m_paNode == NULL)
		return NULL;

	for (int i = 0; i < m_uCount; i++)
		if (m_paNode[i].m_uId == uId)
			return &m_paNode[i];

	return NULL;
}

//
//   Check if a code is a valid node ID.
//
BOOL CNodeBase::IsNode(USHORT uCode)
{
	return (GetNode(uCode) != NULL);
}

// Get the X coordinate of the left-most point
float CNodeBase::LeftMost()
{
	if (m_uCount == 0)
		return 0.0f;

	float fMost = m_paNode[0].x;
	for (USHORT i = 0; i < m_uCount; i++)
	{
		if (fMost > m_paNode[i].x)
			fMost = m_paNode[i].x;
	}

	return fMost;
}

// Get the Y coordinate of the top-most point
float CNodeBase::TopMost()
{
	if (m_uCount == 0)
		return 0.0f;

	float fMost = m_paNode[0].y;
	for (USHORT i = 0; i < m_uCount; i++)
	{
		if (fMost < m_paNode[i].y)
			fMost = m_paNode[i].y;
	}

	return fMost;
}

// Get the X coordinate of the right-most point
float CNodeBase::RightMost()
{
	if (m_uCount == 0)
		return 0.0f;

	float fMost = m_paNode[0].x;
	for (USHORT i = 0; i < m_uCount; i++)
	{
		if (fMost < m_paNode[i].x)
			fMost = m_paNode[i].x;
	}

	return fMost;
}

// Get the Y coordinate of the bottom-most point
float CNodeBase::BottomMost()
{
	if (m_uCount == 0)
		return 0.0f;

	float fMost = m_paNode[0].y;
	for (USHORT i = 0; i < m_uCount; i++)
	{
		if (fMost > m_paNode[i].y)
			fMost = m_paNode[i].y;
	}

	return fMost;
}

//
//   Get the width of the map area.
//
float CNodeBase::Width()
{
	return RightMost() - LeftMost();
}

//
//   Get the height of the map area.
//
float CNodeBase::Height()
{
	return TopMost() - BottomMost();
}

//
//   Get the coordinates of the center point.
//
CPnt CNodeBase::Center()
{
	CPnt pt;
	pt.x = (RightMost() + LeftMost()) / 2;
	pt.y = (TopMost() + BottomMost()) / 2;
	return pt;
}

int CNodeBase::NextID()
{
	int nNextID = 0;
	for (int i = 0; i < m_uCount; i++)
	{
		if (m_paNode[i].m_uId > nNextID)
			nNextID = m_paNode[i].m_uId;
	}

	return nNextID + 1;
}

//
//   Verify the node tag.
//
BOOL CNodeBase::VerifyNodeTag(CNode& nd, CRfId& Tag)
{
	return nd.VerifyTag(Tag);
}

//
//   Find the node with the specified tag ID.
//
CNode* CNodeBase::FindNodeByTag(CRfId& Tag)
{
	// Make sure the node base was created
	if (m_paNode == NULL)
		return NULL;

	for (USHORT i = 0; i < m_uCount; i++)
		if (m_paNode[i].VerifyTag(Tag))
			return &m_paNode[i];

	return NULL;
}

//
//   ����һ�����ڵ��Ƿ�������һ���ڵ��ϡ�
//   ����ֵ��
//     -1: û���䵽�κ�һ���ڵ���
//     >=0: �䵽�ڵ��Id��
//
int CNodeBase::PointHitNodeTest(CPoint& pnt, CScreenReference& ScrnRef)
{
	for (int i = 0; i < m_uCount; i++)
	{
		if (m_paNode[i].PointHitTest(pnt, ScrnRef))
			return m_paNode[i].m_uId;
	}
	return -1;
}

//
//   Create nodes bank data from a text file.
//
BOOL CNodeBase::Create(FILE *StreamIn)
{
	// Blank the node base if neccessary
	Clear();

	// Load the total number of nodes
	if (fscanf(StreamIn, "%u\n", &m_uCount) == EOF)
		return FALSE;

	// Allocate memory for the nodes
	m_paNode = new CNode[m_uCount];

	// Make sure the memory allocation is successful
	if (m_paNode == NULL)
		return FALSE;

	// Load the nodes data one by one
	for (USHORT i = 0; i < m_uCount; i++)
		if (!m_paNode[i].Create(StreamIn))
			return FALSE;

	return TRUE;
}

//
//   Save nodes bank data to a stream.
//
BOOL CNodeBase::Save(FILE *StreamOut)
{
	// Save the total number of nodes
	if (fprintf(StreamOut, "%u\n", m_uCount) == EOF)
		return FALSE;

   // Save the nodes data one by one
	for (USHORT i = 0; i < m_uCount; i++)
		if (!m_paNode[i].Save(StreamOut))
			return FALSE;

	return TRUE;
}

//
//   Add a node to the nodes data base.
//   Return:
//     0 - Success
//    -1 - Out of memory
//    -2 - A node with the same ID already exist in the data base
//
SHORT CNodeBase::AddNode(CNode& nd)
{
	if (GetNode(nd.m_uId) != NULL)
		return -2;

	CNode* pTemp = new CNode[m_uCount+1];
	if (pTemp == NULL)
		return -1;

	for (USHORT i = 0; i < m_uCount; i++)
		pTemp[i] = m_paNode[i];
	pTemp[m_uCount] = nd;
	USHORT uCount = m_uCount+1;

	Clear();
	m_paNode = pTemp;
	m_uCount = uCount;

	return 0;
}

//
//   ��ָ��λ�ü���һ���½ڵ㡣
//
CNode* CNodeBase::AddNode(CPnt& pt)
{
	int nNewID = NextID();
	
	CNode NewNode(nNewID, pt);
	if (AddNode(NewNode) < 0)
		return false;

	return GetNode(nNewID);
}

//
//   Delete a node from the nodes data base.
//   Return:
//     0 - Success
//    -1 - Out of memory
//    -2 - Such node does not exist in the data base
//
SHORT CNodeBase::RemoveNode(USHORT uId)
{
	USHORT i;

	if (GetNode(uId) == NULL)
		return -2;

	// Allocate for the new database
	CNode* pTemp = new CNode[m_uCount-1];
	if (pTemp == NULL)
		return -1;

	// Copy data from the old database
	for (i = 0; i < m_uCount; i++)
		if (m_paNode[i].m_uId != uId)
			pTemp[i] = m_paNode[i];
		else
			break;
			
	for ( ; i < m_uCount-1; i++)
		pTemp[i] = m_paNode[i+1];

	Clear();
	m_paNode = pTemp;
	m_uCount--;

	return 0;
}

//
//   Modify the ID of a node.
//   Return:
//      0 - Success
//     -1 - The Specified old node does not exist in the database
//     -2 - The specified new ID already exists in the database
//
SHORT CNodeBase::ModifyNodeID(USHORT uOldId, USHORT uNewId)
{
	CNode* pNode = GetNode(uOldId);
	if (pNode == NULL)
		return -1;

	if (GetNode(uNewId) != NULL)
		return -2;

	pNode->m_uId = uNewId;
	return 0;
}

//
//   Modify the type of a node.
//
BOOL CNodeBase::ModifyNodeType(USHORT uId, USHORT uNewType)
{
	CNode* pNode = GetNode(uId);
	if (pNode == NULL)
		return FALSE;

	pNode->m_uType = uNewType;
	return TRUE;
}

//
//   Modify the extended type of a node.
//
BOOL CNodeBase::ModifyNodeExtType(USHORT uId, SHORT uNewExtType)
{
	CNode* pNode = GetNode(uId);
	if (pNode == NULL)
		return FALSE;

	pNode->m_uExtType = uNewExtType;
	return TRUE;
}

//
//   Modify the location of a node.
//
BOOL CNodeBase::ModifyNodePoint(USHORT uId, float x, float y)
{
	CNode* pNode = GetNode(uId);
	if (pNode == NULL)
		return FALSE;

	pNode->x = x;
	pNode->y = y;
	return TRUE;
}


#ifdef _MFC_VER

CArchive& operator >> (CArchive& ar, CNodeBase& Obj)
{
	// Blank the node base if neccessary
	Obj.Clear();

	// Load the total number of nodes
	ar >> Obj.m_uCount;

	// Allocate memory for the nodes
	Obj.m_paNode = new CNode[Obj.m_uCount];

	// Make sure the memory allocation is successful
	assert(Obj.m_paNode != NULL);

	// Load the nodes data one by one
	for (USHORT i = 0; i < Obj.m_uCount; i++)
		ar >> Obj.m_paNode[i];

	return ar;
}

CArchive& operator << (CArchive& ar, CNodeBase& Obj)
{
	// Load the total number of nodes
	ar << Obj.m_uCount;

	// Load the nodes data one by one
	for (USHORT i = 0; i < Obj.m_uCount; i++)
		ar << Obj.m_paNode[i];

	return ar;
}

void CNodeBase::Draw(CScreenReference& ScrnRef, CDC* pDC, COLORREF cr)
{
	for (USHORT i = 0; i < m_uCount; i++)
	{
		CNode& Node = m_paNode[i];
		Node.Draw(ScrnRef, pDC, cr);
	}
}

void CNodeBase::DrawNodeID(CScreenReference& ScrnRef, CDC* pDC, LOGFONT* pLogFont)
{
	for (USHORT i = 0; i < m_uCount; i++)
	{
		CNode& Node = m_paNode[i];
		Node.DrawID(ScrnRef, pDC, pLogFont);
	}
}
#endif
