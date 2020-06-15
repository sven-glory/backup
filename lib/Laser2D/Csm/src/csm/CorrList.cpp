#include <stdafx.h>
#include "CorrList.h"
#include "CsmScan.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#endif

///////////////////////////////////////////////////////////////////////////////

//
//   ����ɨ����������ƥ���Ӧ��
//
bool CCorrList::CreateFromScan(const CCsmScan& Scan)
{
	int nCount = Scan.m_nRays;
	if (!Create(nCount))
		return false;

	// �ڴ�����ƥ���Ӧ������
	for (int i = 0; i < nCount; i++)
	{
		CCsmScanPoint& p = Scan.m_pPoints[i];

		// ����ƥ��ɹ��ĵ㣬��¼���Ӧ������
		if (p.corr.valid)
			m_pIdx[i] = p.corr.j1;
		else
			m_pIdx[i] = -1;
	}

	return true;
}

#define CORR_CHECK_THRESHOLD             5

//
//   �˶�����С�εĵ��Ƿ������Ϊ���ص��ġ�
//
bool CCorrList::CheckSmallSegOverlap(int nSeg1Start, int nSeg1End, int nSeg2Start, int nSeg2End)
{
	int n1 = nSeg1Start - CORR_CHECK_THRESHOLD;
	if (n1 < 0)
		n1 = 0;

	int n2 = nSeg1End + CORR_CHECK_THRESHOLD;
	if (n2 > m_nCount - 1)
		n2 = m_nCount - 1;

	for (int i = n1; i <= n2; i++)
	{
		int m = m_pIdx[i];
		if (m >= 0)
		{
			if (m >= nSeg2Start && m <= nSeg2End)
				return true;
		}
	}

	return false;
}

//
//   �˶���������(��ֱ��)�ĵ��Ƿ������Ϊ�����൱�������ص��ġ�
//
bool CCorrList::CheckLineSegOverlap(int nSeg1Start, int nSeg1End, int nSeg2Start, int nSeg2End)
{
	int n1 = nSeg1Start;
	if (n1 < 0)
		n1 = 0;

	int n2 = nSeg1End;
	if (n2 > m_nCount - 1)
		n2 = m_nCount - 1;

	int nCountOverlap = 0;
	for (int i = n1; i <= n2; i++)
	{
		int m = m_pIdx[i];
		if (m >= 0)
		{
			if (m >= nSeg2Start && m <= nSeg2End)
				nCountOverlap++;
		}
	}

	if (nCountOverlap > 5 && (nCountOverlap * 2 > n2 - n1))
		return true;
	else
		return false;
}

