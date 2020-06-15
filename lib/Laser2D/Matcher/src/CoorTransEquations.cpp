#include <stdafx.h>
#include "CoorTransEquations.h"
#include "Geometry.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif

#define SIN_COS_ABNORMAL_LIMIT         1.02f

//
//    �Ը���xֵ�ľ���ֵ�������ƣ�ʹ���ľ���ֵ������fLimitValue��
//    ע�⣬����fLimitValue����Ϊ������
//
float LimitAbs(float x, float fLimitValue)
{
	if (x >= 0)
	{
		return min(x, fLimitValue);
	}
	else
	{
		return max(x, -fLimitValue);
	}
}

//////////////////////////////////////////////////////////////////////////////

CCoorTransEquations::CCoorTransEquations(int nMaxRows, int nCols) : 
CLinearEquations(nMaxRows, nCols)
{
}

//
//   �����С���˷���
//
bool CCoorTransEquations::LeastSquareSolve(float* pX, int nNum)
{
	// �����С���˷�
	CTransform trans;
	float f[4];

	bool bAbnormal = false;
	if (CLinearEquations::LeastSquareSolve(f, 4))
	{
		float sin = f[0];
		float cos = f[1];

		float x = f[2];
		float y = f[3];

		// ���ǵ��������sin��cos����1���������Ҫ���⴦��
		if (fabs(sin) > SIN_COS_ABNORMAL_LIMIT)
		{
			sin = LimitAbs(sin, 1);
			cos = 0;
			bAbnormal = true;
		}
		else if (fabs(cos) > SIN_COS_ABNORMAL_LIMIT)
		{
			cos = LimitAbs(cos, 1);
			sin = 0;
			bAbnormal = true;
		}

		if (bAbnormal)
			SpecialProcess(sin, cos, x, y);
		
		pX[0] = sin;
		pX[1] = cos;
		pX[2] = x;
		pX[3] = y;
		return true;
	}
	else
		return false;
}

//
//   ����任����������⴦����̡�������Ϊ(x, y, sin(thita), cos(thita))������任���������
//   ��С���˷����������ʱ�����sin(thita)��cos(thita)��ֵ����1�����������ȷ���ڴ�����£�ͨ��
//   ǿ��sin(thita)��cos(thita)Ϊ1����������õ�һ��̣�ͨ����С���˷�����������������Ž⣬
//   �õ�����任����
//
bool CCoorTransEquations::SpecialProcess(float fSin, float fCos, float& x, float& y)
{
	float a[2][2], b[2];
	float f[2];

	int nNewRows = m_nMaxRows;
	CLinearEquations* pLsm = new CLinearEquations(nNewRows, 2);

	pLsm->Start();

	for (int i = 0; i < nNewRows / 2; i++)
	{
		int nRow = 2 * i;

		pLsm->A.SetAt(nRow, 0, 1);
		pLsm->A.SetAt(nRow, 1, 0);
		pLsm->A.SetAt(nRow+1, 0, 0);
		pLsm->A.SetAt(nRow+1, 1, 1);

		float b1, b2;
		b1 = B.GetAt(nRow, 0) - A.GetAt(nRow, 0) * fSin - A.GetAt(nRow, 1) * fCos;
		b2 = B.GetAt(nRow+1, 0) - A.GetAt(nRow, 1) * fSin + A.GetAt(nRow, 0) * fCos;

		pLsm->B.SetAt(nRow, 0, b1);
		pLsm->B.SetAt(nRow + 1, 0, b2);
	}
	pLsm->m_nRows = m_nRows;
	bool bResult = pLsm->LeastSquareSolve(f, 2);

	x = f[0];
	y = f[1];

	delete pLsm;

	return bResult;
}
