#include "stdafx.h"
#include "PointMatchPair.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif

///////////////////////////////////////////////////////////////////////////////

//
//   ���ݴ˵��ƥ����������ڡ���С���˷����Ĳ�����
//   ��С���˷��ľ�����ʽΪ A*X = B��������������A�����е����м�B�������е����С�
//
bool CPointMatchPair::MakeLeastSquareData(float fCoefA[2][4], float fCoefB[2])
{
	fCoefA[0][0] = -m_ptLocal.y;
	fCoefA[0][1] = m_ptLocal.x;
	fCoefA[0][2] = 1;
	fCoefA[0][3] = 0;

	fCoefA[1][0] = m_ptLocal.x;
	fCoefA[1][1] = m_ptLocal.y;
	fCoefA[1][2] = 0;
	fCoefA[1][3] = 1;

	fCoefB[0] = m_ptWorld.x;
	fCoefB[1] = m_ptWorld.y;

	return true;
}

//
//   ���ݴ˵��ƥ����������ڡ���С���˷����Ĳ���(�������)��
//   ��С���˷��ľ�����ʽΪ A*X = B��������������A�����е����м�B�������е����С�
//
bool CPointMatchPair::MakeLeastSquareData1(float fCoefA[2][2], float fCoefB[2], float fSin, float fCos)
{
	fCoefA[0][0] = 1;
	fCoefA[0][1] = 0;

	fCoefA[1][0] = 0;
	fCoefA[1][1] = 1;

	fCoefB[0] = m_ptWorld.x + m_ptLocal.y * fSin - m_ptLocal.x * fCos;
	fCoefB[1] = m_ptWorld.y - m_ptLocal.x * fSin - m_ptLocal.y * fCos;

	return true;
}
