#include "stdafx.h"
#include <math.h>
#include "misc.h"
#include "Scan.h"

#ifdef QT_VERSION
#include <QColor>
#include <QPoint>
#include <QPainter>
#endif

#ifdef _DEBUG
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif

static CPosture PoseZero;
extern CPostureGauss PoseGaussZero;

///////////////////////////////////////////////////////////////////////////////

CScan::CScan()
{
}

//
//   �������nNum�����CScan����
//
CScan::CScan(int nNum) : CScanPointCloud(nNum)
{
}

CScan::CScan(CScanPointCloud* pCloud)
{
	Create(pCloud);
}

CScan::~CScan()
{
}

//
//   ���ء�=��������
//
void CScan::operator = (const CScan& Scan)
{
	*(GetScanPointCloudPointer()) = *(((CScan&)Scan).GetScanPointCloudPointer());

	m_poseScanner = Scan.m_poseScanner;
	m_poseRelative = Scan.m_poseRelative;
	m_fStartAng = Scan.m_fStartAng;
	m_fEndAng = Scan.m_fEndAng;

	// Ϊ�β��������������ϣ�
}

//
//   ����������ݡ�
//
void CScan::Clear()
{
	// �����������
	CScanPointCloud::Clear();

	// ���������������
	// ..
}

//
//   ����һ���µ�ɨ�衣
//
bool CScan::Create(int nNum)
{
	// �ڴ�Ӧ���������ݳ�Ա��ʼ��Ϊ0
	Clear();

	// Ϊ���Ʒ���ռ�
	if (!CScanPointCloud::Create(nNum))
		return false;

	return true;
}

//
//   ���ݸ�����������һ���µ�ɨ��
//
bool CScan::Create(CScanPointCloud* pCloud)
{
	Clear();

	if (!Create(pCloud->m_nCount))
		return false;

	*(GetScanPointCloudPointer()) = *pCloud;

	return true;
}

//
//   ����ռ䲢���Ƶ�ǰɨ�衣
//
CScan *CScan::Duplicate()
{
	CScan *scan = new CScan(m_nCount);
	if (scan == NULL)
		return NULL;

	*scan = *this;
	return scan;
}

#if 0
//
//   ��ȫ�����ƶ�ָ���ľ��롣
//
void CScan::Move(float dx, float dy)
{
	if ((dx == 0) && (dy == 0))
		return;

	// �ƶ���������
	CScanPointCloud::Move(dx, dy);

	// �ƶ�����ͷ�ľ�ֵ��
	m_poseScanner.Move(dx, dy);
}
#endif

static int spcmp(const void *v1, const void *v2)
{
	const CScanPoint *sp1 = (CScanPoint *)v1, *sp2 = (CScanPoint *)v2;

	/* float da = NormAngle(sp1->a - sp2->a); not a sorting order */
	float da = sp1->a - sp2->a;

	if (da < 0.0)
		return -1;
	else if (da > 0.0)
		return 1;

	return 0;
}
///////////////////////////////////////////////////////////////////////////////
//
//
//   ���ݸ����ġ�ǿ�������ޡ�ֵ�������Щ�������ڡ�ǿ���⡱��
//
void CScan::MarkReflectivePoints(int nReflectiveGateValue)
{
	for (int i = 0; i < m_nCount; i++)
		m_pPoints[i].m_bHighReflective = (m_pPoints[i].m_nIntensity >= nReflectiveGateValue);
}

//
//   �����µļ���ɨ������Ч��Χ��
//
void CScan::ApplyNewScannerAngles(const CRangeSet& AngRanges)
{
	// �˳����ýǶ�֮���ɨ������
	for (int i = 0; i < m_nCount; i++)
	{
		float ang = m_pPoints[i].a;
		if (!AngRanges.Contain(ang))
		{
			m_pPoints[i].r = 0;
			m_pPoints[i].x = 0;
			m_pPoints[i].y = 0;
			m_pPoints[i].m_nIntensity = 0;
		}
	}
}

//
//   ���ù���ɨ��Ƕȷ�Χ��Լ��(���ڸ����Ƕȷ�Χ֮��Ĳ��ֽ����˳�)��
//
void CScan::ApplyScanAngleRule(float fMinAngle, float fMaxAngle)
{
	CRange range(fMinAngle, fMaxAngle);
	for (int i = 0; i < m_nCount; i++)
	{
		float ang = m_pPoints[i].a;
		if (!range.Contain(ang))
		{
			m_pPoints[i].r = 0;
			m_pPoints[i].x = 0;
			m_pPoints[i].y = 0;
			m_pPoints[i].m_nIntensity = 0;
		}
	}
}

//
//   ���ù���ɨ������Լ��(�����������뷶Χ֮��Ĳ��ֽ����˳�)��
//
void CScan::ApplyScanDistRule(float fMinDist, float fMaxDist)
{
	for (int i = 0; i < m_nCount; i++)
	{
		float r = m_pPoints[i].r / 1000;
		if (r < fMinDist || r > fMaxDist)
		{
			m_pPoints[i].r = 0;
			m_pPoints[i].x = 0;
			m_pPoints[i].y = 0;
			m_pPoints[i].m_nIntensity = 0;
		}
	}
}


//
//   �Ӷ������ļ��ж�ȡɨ�����ݡ�
//
bool CScan::LoadBinary(FILE* fp, const CPosture& pstRobot, const CLaserScannerParam& Param, int nFileVersion)
{
	// �ȼ��㼤��������̬
	CFrame frame(pstRobot);
	m_poseScanner.SetPosture(Param.m_pst);
	m_poseScanner.InvTransform(frame);

	m_poseRelative = m_poseScanner;
	m_fStartAng = Param.m_fStartAngle;
	m_fEndAng = Param.m_fEndAngle;

	if (!CScanPointCloud::LoadBinary(fp, Param.m_fStartAngle, Param.m_fEndAngle, Param.m_nLineCount, nFileVersion))
		return false;

	return true;
}

//
//   ��ɨ�����ݱ��浽ʮ�����ļ���
//
bool CScan::SaveBinary(FILE* fp, int nFileVersion)
{
	if (!CScanPointCloud::SaveBinary(fp, nFileVersion))
		return false;

	return true;
}

//
//   ���������ƽ�������ϵ�任��
//
void CScan::Transform(const CFrame& frame)
{
	CScanPointCloud::Transform(frame);
	m_poseScanner.Transform(frame);
}

//
//   ���������ƽ�������ϵ��任��
//
void CScan::InvTransform(const CFrame& frame)
{
	CScanPointCloud::InvTransform(frame);
	m_poseScanner.InvTransform(frame);
}

#define ARROW_LEN     20          // 20������

#ifdef _MFC_VER
void CScan::Plot(CScreenReference& ScrnRef, CDC* pDC, COLORREF crColorPoint, COLORREF crHighLightPoint,
	bool bShowScanner, int nPointSize)
{
	// ȡ��ɨ��ʱ����ͷ����̬
	CPosture& pstRef = m_poseScanner.GetPostureObject();

	CPen pen(PS_SOLID, 1, crColorPoint);
	CPen* pOldPen = pDC->SelectObject(&pen);

	CBrush Brush(crColorPoint);
	CBrush* pOldBrush = pDC->SelectObject(&Brush);

	CBrush HighLightBrush(crHighLightPoint);

	CPnt pt;

	for (int i = 0; i < m_nCount; i++)
	{
		if (m_pPoints[i].m_bDelete)
			continue;

		if (!m_pPoints[i].m_bHighReflective)
			pDC->SelectObject(&Brush);
		else
			pDC->SelectObject(&HighLightBrush);

		CPnt pt;

		// �����Ҫ����������ϵ����ʾ��������Ҫ��������任
		pt = m_pPoints[i];

		CPoint pnt = ScrnRef.GetWindowPoint(pt);

		int nSize = nPointSize;
		if (m_pPoints[i].m_bDelete)
			nSize /= 2;

		// �ڴˣ�֧�֡�΢��۲�ģʽ��(���ڷŴ����ܴ������£�ÿ����Ҳ�Ŵ���ʾ)
		if (nSize == 1)
		{
			if (ScrnRef.m_fRatio > 500)
				nSize = 1 * ScrnRef.m_fRatio / 500;
		}

		CRect r(pnt.x - nSize, pnt.y - nSize, pnt.x + nSize, pnt.y + nSize);
		pDC->Ellipse(&r);
	}
	pDC->SelectObject(pOldPen);
	pDC->SelectObject(pOldBrush);

	if (bShowScanner)
		m_poseScanner.Draw(ScrnRef, pDC, RGB(255, 0, 255), 40, 150, 1);
}

#elif defined QT_VERSION

void CScan::Plot(CScreenReference& ScrnRef, QPainter* pPainter, QColor crColor, QColor crHighLightPoint,
					  bool bShowScanner, int nPointSize)
{
	// ȡ��ɨ��ʱ����ͷ����̬
	CPosture& pstRef = m_poseScanner.GetPostureObject();

	QPen pen(crColor);
	pen.setWidth(1);
	pPainter->setPen(pen);

	QBrush brush(crColor);
	QBrush highLightBrush(crHighLightPoint);

	CPnt pt;

	for (int i = 0; i < m_nCount; i++)
	{
		if (m_pPoints[i].m_bDelete)
			continue;

		if (!m_pPoints[i].m_bHighReflective)
			pPainter->setBrush(brush);
		else
			pPainter->setBrush(highLightBrush);

		CPnt pt;

		// �����Ҫ����������ϵ����ʾ��������Ҫ��������任
		pt = m_pPoints[i];

		QPoint pnt = ScrnRef.GetWindowPoint(pt);

		int nSize = nPointSize;
		if (m_pPoints[i].m_bDelete)
			nSize /= 2;

		// �ڴˣ�֧�֡�΢��۲�ģʽ��(���ڷŴ����ܴ������£�ÿ����Ҳ�Ŵ���ʾ)
		if (nSize == 1)
		{
			if (ScrnRef.m_fRatio > 500)
				nSize = static_cast<int>(1 * ScrnRef.m_fRatio / 500);
		}

		QPoint d(nSize, nSize);

		QRect r(pnt-d, pnt+d);
		pPainter->drawEllipse(r);
	}

	if (bShowScanner)
		m_poseScanner.Draw(ScrnRef, pPainter, Qt::magenta, 40, 150);
}

#endif
