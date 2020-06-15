#ifndef __CRoute
#define __CRoute

#include "Vertex.h"

//////////////////////////////////////////////////////////////////////////////
//   "CRoute"��Ľӿڡ����ඨ���ˡ�·�ɡ��ĸ��
class CRoute
{
public:
   USHORT* m_pNodeBuf;                 // �ڵ㻺����ָ��
   int     m_nCount;                   // ·���еĽڵ�����

public:
   CRoute() 
   {
      m_pNodeBuf = NULL;
      m_nCount = 0;
   }

   ~CRoute();

   void operator = (CRoute& Obj);

	// ȡ��·�ɽڵ������
	int GetCount() {return m_nCount;}

	// clear the route
   void Clear();

   BOOL Create(USHORT* pBuf, int nCount);

   BOOL Create(CVertex* pVertex, USHORT uStartNode);

	// ���ı��ļ���ȡһ��·��
	BOOL Create(FILE* fp);

	// Get the start node ID
   USHORT GetStartNode();

   // Get the end node ID
   USHORT GetEndNode();

   // Get the cost of the route
   float Cost();
};
#endif
