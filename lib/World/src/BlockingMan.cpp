#include "stdafx.h"
#include "BlockingMan.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif

///////////////////////////////////////////////////////////////////////////////
//   ���������ࡣ

//
//   ���ļ��ж���һ������
//
BOOL CBlockingRule::Create(FILE* fp)
{
	int nKeyNode;
	fscanf(fp, "%d\n", &nKeyNode);
	m_uKeyNode = (USHORT)nKeyNode;

	return CNodeSet::Create(fp);
}

///////////////////////////////////////////////////////////////////////////////
//   ������������

CBlockingManager::CBlockingManager()
{
	m_nCount = 0;
	m_pRules = NULL;
}

CBlockingManager::~CBlockingManager()
{
	if (m_pRules != NULL)
		delete []m_pRules;
}

//
//   ��һ���ļ��ж���������������
//
BOOL CBlockingManager::Create(FILE* fp)
{
	// �����������
	fscanf(fp, "%d", &m_nCount);

	// Ϊ���й������ռ�
	m_pRules = new CBlockingRule[m_nCount];
	if (m_pRules == NULL)
		return FALSE;

	// ���ζ������й���
	for (int i = 0; i < m_nCount; i++)
	{
		if (!m_pRules[i].Create(fp))
			return FALSE;
	}

	return TRUE;
}

//
//   �����������������ҳ����ڵ�uByNode�����������нڵ�ļ��ϡ�
//
BOOL CBlockingManager::FindBlockedNodes(USHORT uByNode, CNodeSet& BlockedNodes)
{
	BOOL bResult = FALSE;

	// ����ս������
	BlockedNodes.Clear();

	// ���η���������
	for (int i = 0; i < m_nCount; i++)
	{
		// ���ҵ��ؼ��ڵ���uByNodeһ�µĹ���
		if (m_pRules[i].GetKey() == uByNode)
		{
			// �������������нڵ�ļ��ϼ��뵽���������
			BlockedNodes.Add(m_pRules[i].GetBlockedNodes());
			bResult = TRUE;
		}
	}

	return bResult;
}

//
//   �����������������ҳ����ڵ㼯��ByNodes�����������нڵ�ļ��ϡ�
//
BOOL CBlockingManager::FindBlockedNodes(CNodeSet& ByNodes, CNodeSet& BlockedNodes)
{
	BOOL bResult = FALSE;

	// ����ս������
	BlockedNodes.Clear();

	POSITION pos = ByNodes.GetHeadPosition();
	for (int i = 0; i < ByNodes.GetCount(); i++)
	{
		CNodeSet s;
		if (FindBlockedNodes(ByNodes.GetNext(pos), s))
		{
			bResult = TRUE;
			BlockedNodes.Add(s);
		}
	}

	return bResult;
}
