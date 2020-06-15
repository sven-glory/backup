#ifndef __CCoorTransEquations
#define __CCoorTransEquations

#include "LinearEquations.h"

///////////////////////////////////////////////////////////////////////////////
//   ���塰����任�����顱�ࡣ
class CCoorTransEquations : public CLinearEquations
{
private:
	// ����任����������⴦�����
	bool SpecialProcess(float fSin, float fCos, float& x, float& y);

public:
	CCoorTransEquations(int nMaxRows, int nCols);

	// �����С���˷�
	virtual bool LeastSquareSolve(float* pX, int nNum);
};
#endif
