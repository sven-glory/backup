#ifndef __CRoundFeature
#define __CRoundFeature

#include "PointFeature.h"

class CScan;

///////////////////////////////////////////////////////////////////////////////
//   ���塰Բ���������ࡣ
class CRoundFeature : public CPointFeature
{
private:
	float m_fRadius;                // ��״�����İ뾶

public:
	CRoundFeature();

	// ����һ������
	virtual CPointFeature* Duplicate() const;

	// ������״�����Ĳ���
	void SetParam(float fRadius);

	// ��Ը����ĵ��ƣ��ڹ涨�ĽǶȷ�Χ�ڣ����������Ƿ��и���״���������������λ��
	virtual bool Detect(CPosture& pst, CScan* pScan, float fStartAngle, float fEndAngle, CPnt* ptCenter);

	// ���ı��ļ���װ��������Ĳ���
	virtual int LoadText(FILE* fp);

	// ���������Ĳ������浽�ı��ļ���
	virtual int SaveText(FILE* fp);

	// �Ӷ������ļ���װ��������Ĳ���
	virtual int LoadBinary(FILE* fp);

	// ���������Ĳ������浽�������ļ���
	virtual int SaveBinary(FILE* fp);

#ifdef _MSC_VER
	// ����Ļ�ϻ��ƴ˵�����
	virtual void Plot(CScreenReference& ScrnRef, CDC* pDC, COLORREF crColor, COLORREF crSelected, int nSize);
#endif
};
#endif

