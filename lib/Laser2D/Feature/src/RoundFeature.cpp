#include "stdafx.h"
#include "RoundFeature.h"
#include "Scan.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif

///////////////////////////////////////////////////////////////////////////////

CRoundFeature::CRoundFeature()
{
	m_nSubType = ROUND_FEATURE;
}

//
//   ����һ������
//
CPointFeature* CRoundFeature::Duplicate() const
{
	CRoundFeature* p = new CRoundFeature;
	*p = *this;
	return p;
}

//
//   ������״�����Ĳ���
//
void CRoundFeature::SetParam(float fRadius)
{
	m_fRadius = fRadius;
}

//
//   ��Ը����ĵ��ƣ��ڹ涨�ĽǶȷ�Χ�ڣ����������Ƿ��и���״���������������λ�á�
//
bool CRoundFeature::Detect(CPosture& pst, CScan* pScan, float fStartAngle, float fEndAngle, CPnt* ptCenter)
{
	return true;
}

//
//   ���ļ���װ��Բ�������Ĳ�����
//
int CRoundFeature::LoadText(FILE* fp)
{
	if (!CPointFeature::Load(fp))
		return -1;

	float fRadius;
	if (fscanf(fp, "%f\n", &fRadius) != 1)
		return -1;

	SetParam(fRadius);
	
	return m_nSubType;
}

//
//   ��Բ�������Ĳ������浽�ļ��С�
//
int CRoundFeature::SaveText(FILE* fp)
{
	fprintf(fp, "%d\t%f\t%f\t", m_nSubType, x, y);
	fprintf(fp, "%f\n", m_fRadius);

	return m_nSubType;
}

//
//   �Ӷ������ļ���װ��������Ĳ�����
//
int CRoundFeature::LoadBinary(FILE* fp)
{
	if (CPointFeature::LoadBinary(fp) < 0)
		return -1;

	float fRadius;
	if (fread(&fRadius, sizeof(float), 1, fp) != 1)
		return -1;

	SetParam(fRadius);

	return m_nSubType;
}

//
//   ���������Ĳ������浽�������ļ��С�
//
int CRoundFeature::SaveBinary(FILE* fp)
{
	if (CPointFeature::SaveBinary(fp) < 0)
		return -1;

	if (fwrite(&m_fRadius, sizeof(float), 1, fp) != 1)
		return -1;

	return m_nSubType;
}

#ifdef _MSC_VER
//
//   ����Ļ�ϻ��ƴ�Բ��������
//
void CRoundFeature::Plot(CScreenReference& ScrnRef, CDC* pDC, COLORREF crColor, COLORREF crSelected, int nSize)
{
	Draw(ScrnRef, pDC, (128, 128, 0), 5);
}
#endif
