#include "stdafx.h"
#include "Route.h"
#include "World.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif

extern CWorld World;

//////////////////////////////////////////////////////////////////////////////
//   Implementation of class "CRoute".

CRoute::~CRoute()
{
   Clear();
}

//
//   ·�ɵĿ�����
//
void CRoute::operator = (CRoute& Obj)
{
   Clear();
   Create(Obj.m_pNodeBuf, Obj.m_nCount);
}

//
//    ���·�ɡ�
//
void CRoute::Clear()
{
   if (m_pNodeBuf != NULL)
   {
      delete []m_pNodeBuf;
      m_pNodeBuf = NULL;
      m_nCount = 0;
   }
}

BOOL CRoute::Create(USHORT* pBuf, int nCount)
{
	Clear();
   m_pNodeBuf = new USHORT[nCount];
   if (m_pNodeBuf == NULL)
      return FALSE;

   for (int i = 0; i < nCount; i++)
      m_pNodeBuf[i] = pBuf[i];

   m_nCount = nCount;
   return TRUE;
}

BOOL CRoute::Create(CVertex* pVertex, USHORT uEndNode)
{
	Clear();

   // First, caculate the number of vertexes
   int nVertexCount = 0;
   CVertex* p = pVertex;
   while (p != NULL)
   {
      nVertexCount++;
      p = p->pParent;
   }

   m_pNodeBuf = new USHORT[nVertexCount];
   if (m_pNodeBuf == NULL)
      return FALSE;

   for (int i = nVertexCount-1; i >= 0; i--)
   {
      m_pNodeBuf[i] = pVertex->uID;
      pVertex = pVertex->pParent;
   }
   m_nCount = nVertexCount;

   return TRUE;
}

//
//   ���ı��ļ���ȡһ��·�ɡ�
//
BOOL CRoute::Create(FILE* fp)
{
	int nCount;

	Clear();

	if (fscanf(fp, "%d\n", &nCount) != 1)
		return FALSE;

   m_pNodeBuf = new USHORT[nCount];
   if (m_pNodeBuf == NULL)
      return FALSE;

   for (int i = 0; i < nCount; i++)
	{
		if (fscanf(fp, "%d\t", &(m_pNodeBuf[i])) != 1)
			return FALSE;
	}
	fscanf(fp, "\n");

   m_nCount = nCount;
   return TRUE;
}

//
//   Get the start node ID.
//
USHORT CRoute::GetStartNode()
{
   if (m_pNodeBuf == NULL)
      return (USHORT)-1;
   else
      return m_pNodeBuf[0];
}

//
//   Get the end node ID.
//
USHORT CRoute::GetEndNode()
{
   if (m_pNodeBuf == NULL)
      return (USHORT)-1;
   else
      return m_pNodeBuf[m_nCount-1];
}

//
//   Get the cost of the route.
//
float CRoute::Cost()
{
   float fSum = 0;
   for (int i = 1; i < m_nCount; i++)
   {
      USHORT uNode1 = m_pNodeBuf[i-1];
      USHORT uNode2 = m_pNodeBuf[i];
      CPath* pPath = World.GetPathPointer(uNode1, uNode2);
      ASSERT(pPath != NULL);

      fSum += pPath->Cost();
   }
   return fSum;
}
