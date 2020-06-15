#include "stdafx.h"
#include "FlatReflectorFeature.h"
#include "Scan.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif

#define REFLECTOR_INTENSITY_GATE           5000

///////////////////////////////////////////////////////////////////////////////

//
//   ���캯����
//
CFlatReflectorFeature::CFlatReflectorFeature(float _x, float _y)
{
	x = _x;
	y = _y;
	m_nSubType = FLAT_REFLECTOR_FEATURE;
	m_fWidth = 0.1f;
	m_fAngle = 0;
	m_nWhichSideToUse = FEATURE_DIR_BOTH_SIDES;    // ���������
	m_fMaxIncidenceAngle = PI / 2;     // �����������Ϊ90��
	m_nMinNumPoints = 2;             // ���ٵ���Ϊ2
}

//
//   ���캯����
//
CFlatReflectorFeature::CFlatReflectorFeature() 
{
	m_nSubType = FLAT_REFLECTOR_FEATURE;
	m_fWidth = 0.1f;
	m_fAngle = 0;
	m_nWhichSideToUse = FEATURE_DIR_FRONT_SIDE_ONLY;    // ���������
	m_fMaxIncidenceAngle = PI / 2;     // �����������Ϊ90��
	m_nMinNumPoints = 2;             // ���ٵ���Ϊ2
//	m_fMaxLength = 300;              // �߶���󳤶�0.3m
}

//
//   ��Ը����ĵ��ƣ��ڹ涨�ĽǶȷ�Χ�ڣ����������Ƿ��иö�ֱ�߶���������������������λ�á�
//   ע�⣺ֻ���ڹ涨���漰����Ƕ����з��䡣
//
bool CFlatReflectorFeature::Detect(CPosture& pst, CScan* pScan, float fStartAngle, float fEndAngle, CPnt* ptCenter)
{
	CScanPointCloud* pCloud = pScan->GetScanPointCloudPointer();

	// ------------- �ȸ��������桢����ǵȷ����������Ƿ���ܱ�����  --------------------

	// ����Ӽ���ͷ���������ĵ�ֱ�߶�
	CLine lnRay(pst.GetPntObject(), GetPntObject());
//	if (!CheckInRay(lnRay))
//		return false;

	int nFeatureStart = -1;    // ����������ε����λ��
	int nFeatureEnd = -1;      // ����������εĽ���λ��

	// ���㼤�������������ľ���
	float fEstimatedDist = DistanceTo(pst.GetPntObject());

	// �ҵ���ʼ������
	for (int i = 0; i < pCloud->m_nCount; i++)
	{
		CScanPoint& sp = pCloud->m_pPoints[i];

		// �������С����ʼ�ǣ�ֱ������
		if (sp.a < fStartAngle)
			continue;

		// ��������ѳ�����ֹ�ǣ���������
		else if (sp.a > fEndAngle)
			break;

		// ��������������ޣ�����Ϊ�˵㲻������
		else if (fabs(fEstimatedDist - sp.r) > FEATURE_REG_DISTANCE_WINDOW)
		{
			if (nFeatureStart >= 0)
			{
				nFeatureEnd = i - 1;     // ��Ƿ����ν���λ��

				// break - ��ʱ����
				break;
			}
			continue;
		}
		// �����ǿ�ȳ������ޣ����Ҽ�����������ޣ�����Ϊ�Ƿ����˷����
		else if (sp.m_nIntensity > REFLECTOR_INTENSITY_GATE)
		{
			// ����ǵ�һ�η���
			if (nFeatureStart < 0)
			{
				nFeatureStart = i;
			}

			// �����һ�����Ѿ�������ȴ��һ�η��ַ���壬˵�����������ж������壬�������������
			else if (nFeatureEnd >= 0)
			{
				return false;
			}
		}

		// ����ѷ����������Σ������ڼ���ɨ���˷Ƿ���壬����Ϊһ����������巢�����
		else if (nFeatureStart >= 0 && nFeatureEnd < 0)
		{
			nFeatureEnd = i - 1;     // ��Ƿ����ν���λ��

////////// ��ʱ����!!
			break;
		}
	}

	// ���û�ҵ�����������false
	if (nFeatureStart < 0)
		return false;

	int nMiddle = (nFeatureStart + nFeatureEnd)/2;
	CScanPoint& spCenter = pCloud->m_pPoints[nMiddle];

	*ptCenter = spCenter.GetPntObject();
	ptCenter->a = CAngle::NormAngle(ptCenter->a);

	return true;
}

//
//   ����һ��������
//
CPointFeature* CFlatReflectorFeature::Duplicate() const
{
	CFlatReflectorFeature* p = new CFlatReflectorFeature;
	*p = *this;
	return (CPointFeature*)p;
}


#ifdef _MSC_VER

//
//   ����Ļ�ϻ��ƴ˵�������
//
void CFlatReflectorFeature::Plot(CScreenReference& ScrnRef, CDC* pDc, COLORREF crColor, COLORREF crSelected, int nSize,
	int nShowDisabled, bool bShowSuggested)
{
	// ��ɫ��ʾ
	if (!IsEnabled() && nShowDisabled == DISABLED_FEATURE_UNDERTONE)
	{
		BYTE r = GetRValue(crColor) / 3;
		BYTE g = GetGValue(crColor) / 3;
		BYTE b = GetBValue(crColor) / 3;

		crColor = RGB(r, g, b);
	}

	// �ڴˣ�֧�֡�΢��۲�ģʽ��(���ڷŴ����ܴ������£�ÿ����Ҳ�Ŵ���ʾ)
	nSize = 5;
	if (ScrnRef.m_fRatio > 500)
		nSize = 5 * ScrnRef.m_fRatio / 500;

	// �����÷����
	CPnt::Draw(ScrnRef, pDc, crColor, nSize);

	// �����Ҫ��ʾ���Ƽ����������ԣ��ڴ���Ӻ�ɫ��Ȧ
	if (bShowSuggested && IsSuggested())
		CPnt::Draw(ScrnRef, pDc, RGB(255, 0, 0), nSize, 2);
}
#endif
