#include "stdafx.h"
#include "RouterRules.h"

//////////////////////////////////////////////////////////////////////////////
//   "CRouterRule"���ʵ�֡�
//
//   ���ļ��ж���һ��·�ɹ���
//
BOOL CRouterRule::Create(FILE* fp)
{
	int nFromNode, nToNode;
	if (fscanf(fp, "%d\t%d\n", &nFromNode, &nToNode) == 2)
	{
		if (!m_Route.Create(fp))
			return FALSE;

		m_uFromNode = nFromNode;
		m_uToNode = nToNode;
		return TRUE;
	}
	else
		return FALSE;
}

//////////////////////////////////////////////////////////////////////////////
//   "CRouterRules"���ʵ�֡�

CRouterRules::CRouterRules()
{
	m_nCount = 0;
	m_pRules = NULL;
}

CRouterRules::~CRouterRules()
{
	Clear();
}

//
//   ������й���
//
void CRouterRules::Clear()
{
	if (m_pRules != NULL)
		delete []m_pRules;

	m_pRules = NULL;
	m_nCount = 0;
}

//
//   ���ļ��ж�������·�ɹ���
//
BOOL CRouterRules::Create(FILE* fp)
{
	int nCount;
	if (fscanf(fp, "%d", &nCount) == 1)
	{
		m_pRules = new CRouterRule[nCount];
		if (m_pRules == NULL)
			return FALSE;

		for (int i = 0; i < nCount; i++)
			if (!m_pRules[i].Create(fp))
				return FALSE;
		
		m_nCount = nCount;
		return TRUE;
	}
	else
		return FALSE;
}

//
//   ������ָ����㵽ָ���յ��·�ɡ�
//
//   ����ֵ��
//     - NULL: û�ҵ�
//     - �������ҵ���·�ɶ����ָ��
//
CRoute* CRouterRules::Find(USHORT uFromNode, USHORT uToNode)
{
	for (int i = 0; i < m_nCount; i++)
	{
		CRouterRule& Rule = m_pRules[i];
		if (Rule.m_uFromNode == uFromNode && Rule.m_uToNode == uToNode)
			return &(Rule.GetRoute());
	}

	return NULL;
}
