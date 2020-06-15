#ifndef __CFlatReflectorFeature
#define __CFlatReflectorFeature

#include "ShortLineFeature.h"

#define REFLECTOR_REG_DISTANCE_WINDOW       1.0f
#define REFLECTOR_MAX_LEN                   1.0f     

//////////////////////////////////////////////////////////////////////////////
//   ƽ���ͷ�������͡�
class CFlatReflectorFeature : public CShortLineFeature
{
public:
	int m_nIntensity;        // ����ǿ��

public:
	CFlatReflectorFeature(float _x, float _y);
	CFlatReflectorFeature();

	// ����һ������
	virtual CPointFeature* Duplicate() const;

	// ��Ը����ĵ��ƣ��ڹ涨�ĽǶȷ�Χ�ڣ����������Ƿ��и÷������������������������λ��
	virtual bool Detect(CPosture& pst, CScan* pScan, float fStartAngle, float fEndAngle, CPnt* ptCenter);

#ifdef _MSC_VER
	// ����Ļ�ϻ��ƴ˵�����
	virtual void Plot(CScreenReference& ScrnRef, CDC* pDC, COLORREF crColor, COLORREF crSelected, int nSize,
		int nShowDisabled = DISABLED_FEATURE_UNDERTONE, bool bShowSuggested = true);
#endif
};
#endif
