#include "stdafx.h"
#include "PointFeature.h"
#include "Scan.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif

///////////////////////////////////////////////////////////////////////////////

CPointFeature::CPointFeature(float _x, float _y)
{
	x = _x;
	y = _y;
	m_nSubType = GENERIC_POINT_FEATURE;
	m_nMatchId = -1;
}

//
//   ����һ������
//
CPointFeature* CPointFeature::Duplicate() const
{
	CPointFeature* p = new CPointFeature;
	*p = *this;
	return p;
}

//
//   �жϸõ������Ƿ�ָ����ɨ�����յ���������ɨ������ꡣ�ú�����Ҫ���ڷ��档
//
bool CPointFeature::HitByLineAt(CLine& lnRay, CPnt& ptHit, float& fDist)
{
	CPnt pt;
	float f;

	// ����һ��0.1�װ뾶��Բ
	CCircle circle(*this, 0.1f);

	// �ж������߶��Ƿ��н���
	bool bHit = circle.IntersectLineAt(lnRay, pt, f);

	// ����н��㣬��������Ǻϸ�
	if (bHit)
	{
		ptHit = pt;
		fDist = f;
		return true;
	}

	// û�н��㣬���治���ã�������ǳ��ޣ�����false
	return false;
}

//
//   ��Ը����ĵ��ƣ��ڹ涨�ĽǶȷ�Χ�ڣ����������Ƿ��иõ���������������������λ�á�
//
bool CPointFeature::Detect(CPosture& pst, CScan* pScan, float fStartAngle, 
									float fEndAngle, CPnt* ptCenter)
{
	return false;
}

//
//   ���ļ���װ��������Ĳ�����
//   ����ֵ��
//     -1  : ��ȡʧ��
//     >=0 : �����������ͱ��
//
int CPointFeature::LoadText(FILE* fp)
{
	if (fscanf(fp, "%f\t%f\n", &x, &y) != 2)
		return -1;

	return m_nSubType;
}

//
//   ���������Ĳ������浽�ļ��С�
//   ����ֵ��
//     -1  : д��ʧ��
//     >=0 : �����������ͱ��
//
int CPointFeature::SaveText(FILE* fp)
{
	fprintf(fp, "%d\t%f\t%f\n", m_nSubType, x, y);
	return m_nSubType;
}

//
//   �Ӷ������ļ���װ��������Ĳ�����
//
int CPointFeature::LoadBinary(FILE* fp)
{
	// ����������
	float f[2];
	if (fread(f, sizeof(float), 2, fp) != 2)
		return -1;

	x = f[0];
	y = f[1];

	return m_nSubType;
}

//
//   ���������Ĳ������浽�������ļ��С�
//
int CPointFeature::SaveBinary(FILE* fp)
{
	if (fwrite(&m_nSubType, sizeof(int), 1, fp) != 1)
		return -1;

	if (fwrite(&x, sizeof(float), 1, fp) != 1)
		return -1;

	if (fwrite(&y, sizeof(float), 1, fp) != 1)
		return -1;

	return m_nSubType;
}

#define POINT_FEATURE_RADIUS_MM        25     // �ٶ��������뾶Ϊ25mm

#ifdef _MFC_VER
//
//   ����Ļ�ϻ��ƴ˵�������
//
void CPointFeature::Plot(CScreenReference& ScrnRef, CDC* pDC, COLORREF crColor, COLORREF crSelected, 
	int nPointSize, int nLineWidth)
{
	int nRadius = POINT_FEATURE_RADIUS_MM / 1000.0f * ScrnRef.m_fRatio;

	if (nRadius < nPointSize)
		nRadius = nPointSize;

	Draw(ScrnRef, pDC, crColor, nRadius, nLineWidth);
}

//
//   ����Ļ�ϻ��ƴ˵�������ID�š�
//
void CPointFeature::PlotId(CScreenReference& ScrnRef, CDC* pDC, COLORREF crColor)
{
	CPoint pnt = ScrnRef.GetWindowPoint(GetPntObject());

	CString str;
	str.Format(_T("%d"), id);

	COLORREF crOldColor = pDC->SetTextColor(crColor);
	pDC->TextOut(pnt.x - 20, pnt.y - 20, str);
	pDC->SetTextColor(crOldColor);
}

#elif defined QT_VERSION

//
//   ����Ļ�ϻ��ƴ˵�������
//
void CPointFeature::Plot(CScreenReference& ScrnRef, QPainter* pPainter, QColor crColor, QColor crSelected,
	int nPointSize, int nLineWidth)
{
	int nRadius = POINT_FEATURE_RADIUS_MM / 1000.0f * ScrnRef.m_fRatio;

	if (nRadius < nPointSize)
		nRadius = nPointSize;

	Draw(ScrnRef, pPainter, crColor, nRadius, nLineWidth);
}

//
//   ����Ļ�ϻ��ƴ˵�������ID�š�
//
void CPointFeature::PlotId(CScreenReference& ScrnRef, QPainter* pPainter, QColor crColor)
{
	CPoint pnt = ScrnRef.GetWindowPoint(GetPntObject());

	QString str = QString::number(id);
	pPainter->setPen(crColor);
	pPainter->drawText(pnt.x - 20, pnt.y - 20, str);
}
#endif
