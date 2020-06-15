#include "stdafx.h"
#include "LineMatchPair.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif

///////////////////////////////////////////////////////////////////////////////

//
//   ���ݴ�ֱ��ƥ����������ڡ���С���˷����Ĳ�����
//   ��С���˷��ľ�����ʽΪ A*X = B��������������A�����е����м�B�������е����С�
//
bool CLineMatchPair::MakeLeastSquareData(float fCoefA[3][4], float fCoefB[3])
{
	// ����������ϵ�м���Odometry��̬����ֱ�ߵľ���
	float A = m_lnWorld.a;
	float B = m_lnWorld.b;
	float C = m_lnWorld.c;

	float fJudge = A * m_pstOdometry.x + B * m_pstOdometry.y + C;
	int nSign = (fJudge >= 0) ? 1 : -1;

	fCoefA[0][0] = 0;
	fCoefA[0][1] = 0;
	fCoefA[0][2] = A;
	fCoefA[0][3] = B;

	fCoefB[0] = nSign * m_fDistToOrigin * sqrt(A * A + B * B) - C;

	// �Ƕȵ���ͳ
	fCoefA[1][0] = 1;
	fCoefA[1][1] = 0;
	fCoefA[1][2] = 0;
	fCoefA[1][3] = 0;

	fCoefB[1] = sin(m_ang);

	// �Ƕȵ�����
	fCoefA[2][0] = 0;
	fCoefA[2][1] = 1;
	fCoefA[2][2] = 0;
	fCoefA[2][3] = 0;

	fCoefB[2] = cos(m_ang);

	return true;
}
