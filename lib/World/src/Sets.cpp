//                               - SETS.CPP -
//
//   Defines the Blocking sets.
//

#include "stdafx.h"
#include "Sets.h"
#include "World.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif

extern CWorld World;

//////////////////////////////////////////////////////////////////////////////
//   Implmentation of class "CNodeSet".

BOOL CNodeSet::Occupied()
{
   POSITION pos = GetHeadPosition();
   for (int i = 0; i < GetCount(); i++)
   {
      USHORT uNode = GetNext(pos);
      if (World.GetNode(uNode)->Occupied())
         return TRUE;
   }
   return FALSE;
}

//
//   ���ļ��г�ʼ���ڵ㼯�ϡ�
//
BOOL CNodeSet::Create(FILE* fp)
{
	int nCount, nNode;

	// ���뼯����Ԫ������
	fscanf(fp, "%d\n", &nCount);

	// ���ζ�������Ԫ��
	for (int i = 0; i < nCount; i++)
	{
		fscanf(fp, "%d\t", &nNode);
		Add((USHORT)nNode);
	}

	fscanf(fp, "\n");
	return TRUE;
}

//
//   ������������Ԫ��д��һ�ļ��С�
//
BOOL CNodeSet::Save(FILE* fp)
{
	// ���뼯����Ԫ������
	fprintf(fp, "%d\n", GetCount());

	// ����д������Ԫ��
	POSITION pos = GetHeadPosition();
	for (int i = 0; i < GetCount(); i++)
	{
		fprintf(fp, "%d\t", GetNext(pos));
	}

	fprintf(fp, "\n");
	return TRUE;
}

void CNodeSet::Dump()
{
	POSITION pos = GetHeadPosition();
	for (int i = 0; i < GetCount(); i++)
		TRACE("%d\t", GetNext(pos));
	TRACE("\n");
}

//////////////////////////////////////////////////////////////////////////////
//   Implmentation of class "CPathSet".

BOOL CPathSet::Occupied()
{
   POSITION pos = GetHeadPosition();
   for (int i = 0; i < GetCount(); i++)
   {
      USHORT uPath = GetNext(pos);
      if (World.GetPathPointer(uPath)->Occupied())
         return TRUE;
   }
   return FALSE;
}

BOOL CPathSet::Create(FILE* fp)
{
	return TRUE;
}

BOOL CPathSet::Save(FILE* fp)
{
	return TRUE;
}

