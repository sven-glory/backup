#include <stdafx.h>
#include <math.h>
#include "Bezier.h"
#include "Geometry.h"
#include "ScrnRef.h"
#include "Combination.h"

#define BEZIER_CURVE_DEFAULT_SEG_COUNT         100

#ifdef _DEBUG
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif

///////////////////////////////////////////////////////////////////////////////
//   ����������

//
//   ���㸡�����������η�
//
float FloatPowerInt(float x, int nRank)
{
	float f = x;

	if (nRank == 0)
		return 1;
	
	for (int i = 1; i < nRank; i++)
		f *= x;

	return f;
}

///////////////////////////////////////////////////////////////////////////////

//
//   ��������ױ��������ߡ�
//
CBezier::CBezier(int nCountKeyPoints, CPnt* pptKey)
{
	m_ptKey = NULL;
	m_pSamplePoints = NULL;

	// ��������ױ���������
	Create(nCountKeyPoints, pptKey);
}

//
//   �������ױ��������ߡ�
//
CBezier::CBezier(const CPosture& pstStart, const CPosture& pstEnd, float fLen1, float fLen2)
{
	m_ptKey = NULL;
	m_pSamplePoints = NULL;

	// �������ױ���������
	Create(pstStart, pstEnd, fLen1, fLen2);
}

//
//   ȱʡ���캯��(������Ҫ��ʽ����Create����)��
//
CBezier::CBezier()
{
	m_ptKey = NULL;
	m_pSamplePoints = NULL;
}

CBezier::~CBezier()
{
	Clear();
}

//
//   ����ռ䡣
//
void CBezier::Clear()
{
	if (m_ptKey != NULL)
	{
		delete[]m_ptKey;
		m_ptKey = NULL;
	}

	if (m_pSamplePoints != NULL)
	{
		delete[]m_pSamplePoints;
		m_pSamplePoints = NULL;
	}
}

//
//   ���ݸ����Ĺؼ������������Bezier���ߡ�
//
bool CBezier::Create(int nCountKeyPoints, CPnt* pptKey)
{
	if (m_ptKey != NULL)
		delete[]m_ptKey;

	// ����ؼ�������
	m_nCountKeyPoints = nCountKeyPoints;

	// Ϊ���йؼ������ռ�
	m_ptKey = new CPnt[m_nCountKeyPoints];
	if (m_ptKey == NULL)
		return false;

	// ���μ�¼����ê��
	for (int i = 0; i < m_nCountKeyPoints; i++)
		m_ptKey[i] = pptKey[i];

	m_nSampleCount = BEZIER_CURVE_DEFAULT_SEG_COUNT;

	// �������ɲ���������
	return CreateSamplePoints();
}

//
//   �������ױ��������ߡ�
//
bool CBezier::Create(const CPosture& pstStart, const CPosture& pstEnd, float fLen1, float fLen2)
{
	if (m_ptKey != NULL)
		delete []m_ptKey;

	m_nCountKeyPoints = 4;
	m_ptKey = new CPnt[m_nCountKeyPoints];
	if (m_ptKey == NULL)
		return NULL;

	m_ptKey[0] = pstStart;
	m_ptKey[3] = pstEnd;

	CLine ln1(pstStart, fLen1);
	m_ptKey[1] = ln1.GetEndPoint();

	CLine ln2(pstEnd, !pstEnd.GetAngle(),fLen2);
	m_ptKey[2] = ln2.GetEndPoint();

	m_nSampleCount = BEZIER_CURVE_DEFAULT_SEG_COUNT;

	// �������ɲ���������
	return CreateSamplePoints();
}

//
//   ����ѡ�㳣��K��������Bezier����
//
bool CBezier::Create(const CPosture& pstStart, const CPosture& pstEnd, float K)
{
	if (m_ptKey != NULL)
		delete[]m_ptKey;

	m_nCountKeyPoints = 4;
	m_ptKey = new CPnt[m_nCountKeyPoints];
	if (m_ptKey == NULL)
		return false;

	// ���ҵ�������̬�Ľ���
	CLine lnTemp1(pstStart, 1000.0f);
	CLine lnTemp2(pstEnd, 1000.0f);

	CPnt pt;
	float fLen1, fLen2;

	// ���������̬��ƽ�е�
	if (!lnTemp1.Intersect(lnTemp2, &pt.x, &pt.y, NULL, NULL, 1e-4))
	{
		lnTemp1.DistanceToPoint(false, pstEnd, NULL, &pt);
		float fDist = pstStart.DistanceTo(pt);
		if (fDist < 0.05f)
			return false;

		fLen1 = fLen2 = fDist * 0.5f;
	}

	// ���������̬���ڵ�ֱ���н���
	else
	{
		// ����������̬���γɵļнǴ�С

		CAngle angDiff = pstEnd.GetAngle() - pstStart.GetAngle();
		float fPhi = angDiff.m_fRad;
		if (fPhi > PI)
			fPhi = 2 * PI - fPhi;
		if (fPhi > PI / 2)
			fPhi = PI - fPhi;
		fPhi /= 2;

		if (fPhi < PI / 9)
		{
			float fDist = pstStart.DistanceTo(pstEnd);
			fLen1 = fLen2 = fDist / 2;
		}
		else
		{
			fLen1 = pstStart.DistanceTo(pt);
			fLen2 = pstEnd.DistanceTo(pt);
			fLen1 *= fabs(tan(fPhi));
			fLen2 *= fabs(tan(fPhi));
		}
	}

	// ����Bezier���ߵĵ�һ�������һ��ê��
	m_ptKey[0] = pstStart;
	m_ptKey[3] = pstEnd;

	// �ڶ���ê��
	CLine ln1(pstStart, fLen1);
	m_ptKey[1] = ln1.GetEndPoint();

	// ������ê��
	CLine ln2(pstEnd, !pstEnd.GetAngle(), fLen2);
	m_ptKey[2] = ln2.GetEndPoint();

	m_nSampleCount = BEZIER_CURVE_DEFAULT_SEG_COUNT;

	// �������ɲ���������
	return CreateSamplePoints();
}

//
//    ������ɢ���������ݡ�
//
bool CBezier::CreateSamplePoints()
{
	// �ͷŲ���������
	if (m_pSamplePoints != NULL)
		delete []m_pSamplePoints;

	// Ϊ�����������ݷֶοռ�
	m_pSamplePoints = new CCurveSamplePoint[m_nSampleCount];
	if (m_pSamplePoints == NULL)
		return false;

	// ���������ϸ������������
	m_fTotalLen = 0;
	for (int i = 0; i < m_nSampleCount; i++)
	{
		float t = (float)i / m_nSampleCount;
		SetCurT(t);

		// ���㱾С�����ߵĳ���
		float fSegLen = sqrt(dx1 * dx1 + dy1 * dy1) * 1.0f / m_nSampleCount;    // ?????? /m_nSampleCount��Ϊʲô?

		// �����ۼƵ���ǰ������߳���
		m_fTotalLen += fSegLen;   

		// �����Ӧ�ڸò����������
		CCurveSamplePoint& sp = m_pSamplePoints[i];
		sp.t = t;
		sp.fProgress = m_fTotalLen;
		sp.fSegLen = fSegLen;
		sp.GetPntObject() = TrajFun();
		sp.fTangentAngle = TangentFun().m_fRad;
		sp.fCurvature = CurvatureFun();
	}

	return true;
}

//
//   ���ء�=����������
//
void CBezier::operator = (const CBezier& Obj)
{
	Clear();

	m_nCountKeyPoints = Obj.m_nCountKeyPoints;
	Create(m_nCountKeyPoints, Obj.m_ptKey);
}

//
//   ��ָ������Ŵ�����һ�����Ƶ㡣
//
bool CBezier::AddKeyPoint(int nIdx, const CPnt& pt)
{
	// ����һ���ؼ��㣬��Ҫ����Ϊ�ؼ������ռ�
	CPnt* p = new CPnt[m_nCountKeyPoints + 1];
	if (p == NULL)
		return false;

	// ���ƹؼ��㣬�������µ�
	for (int i = 0; i < nIdx; i++)
		p[i] = m_ptKey[i];

	p[nIdx] = pt;

	for (int i = nIdx + 1; i < m_nCountKeyPoints + 1; i++)
		p[i] = m_ptKey[i - 1];

	// �ͷ�ԭ���Ĺؼ������ݿռ�
	delete []m_ptKey;

	// �������ùؼ���������
	m_ptKey = p;

	m_nCountKeyPoints++;

	// �������ɲ���������
	return CreateSamplePoints();
}

//
//   ��ָ����Ŵ��Ŀ��Ƶ��Ƴ���
//
bool CBezier::RemoveKeyPoint(int nIdx)
{
	// �������ߣ��Ѳ����ٽ����ˡ����⣬�����˵�Ҳ����ɾ��
	if (m_nCountKeyPoints == 4 || nIdx == 0 || nIdx == m_nCountKeyPoints - 1)
		return false;

	// ɾ��ָ���Ŀ��Ƶ�
	for (int i = nIdx; i < m_nCountKeyPoints - 1; i++)
		m_ptKey[i] = m_ptKey[i + 1];

	m_nCountKeyPoints--;

	// �������ɲ���������
	return CreateSamplePoints();
}

//
//   ���ݸ����Ĵ��������������ߵ��û����ݡ�
//
void CBezier::CreateUserData(void(*pProc)(CBezier*, void*), void* pParam)
{
	UserDataCreateProc = pProc;
	UserDataCreateProc(this, pParam);
}

//
//   ȡ��ָ�����û����ݡ�
//
float CBezier::GetUserData(int nSampleIdx, int nDataIdx) const
{
	return m_pSamplePoints[nSampleIdx].fUserData[nDataIdx];
}

//
//   �˶�һ�£��������Ƿ�ͻ���˹涨��Լ����
//
bool CBezier::CheckConstraints(float fMaxCurvature, float fMaxCurvatureDiff) const
{
	float t;
	for (int i = 0; i < m_nSampleCount; i++)
	{
		CCurveSamplePoint& sp = m_pSamplePoints[i];

		// Ŀǰֻ��֤����ֵ
		if (sp.fCurvature > fMaxCurvature)
			return false;
	}

	return true;
}

//
//   ���õ�ǰ�����Ա�ȷ����ǰ�㡣
//
void CBezier::SetCurT(float t)
{
	float f1 = (1-t);
	float f2 = f1 * f1;
	float f3 = f2 * f1;

	// ����켣��(m_pt.x, m_pt.y)
#if 0
	m_pt.x = f3 * m_ptKey[0].x + 3 * t * f2 * m_ptKey[1].x + 
				3 * t * t * f1 * m_ptKey[2].x + t * t * t * m_ptKey[3].x;

	m_pt.y = f3 * m_ptKey[0].y + 3 * t * f2 * m_ptKey[1].y + 
				3 * t * t * f1 * m_ptKey[2].y + t * t * t * m_ptKey[3].y;

	// ����һ�׵���(dx1, dy1)
	dx1 = 3 * (m_ptKey[1].x - m_ptKey[0].x) +
		6 * (m_ptKey[2].x - 2 * m_ptKey[1].x + m_ptKey[0].x) * t +
		3 * (m_ptKey[3].x - 3 * m_ptKey[2].x + 3 * m_ptKey[1].x - m_ptKey[0].x) * t * t;

	dy1 = 3 * (m_ptKey[1].y - m_ptKey[0].y) +
		6 * (m_ptKey[2].y - 2 * m_ptKey[1].y + m_ptKey[0].y) * t +
		3 * (m_ptKey[3].y - 3 * m_ptKey[2].y + 3 * m_ptKey[1].y - m_ptKey[0].y) * t * t;


#else

	int n = m_nCountKeyPoints - 1;
	m_pt.x = m_pt.y = 0;

	// ����켣��(m_pt.x, m_pt.y)
	for (int i = 0; i <= n; i++)
	{
		int c = Combination(n, i);
		float p1 = FloatPowerInt(1 - t, n - i);
		float p2 = FloatPowerInt(t, i);
		float f = c * p1 * p2;
		m_pt.x += m_ptKey[i].x * f;
		m_pt.y += m_ptKey[i].y * f;
	}

	// ����һ�׵���(dx1, dy1)
	dx1 = dy1 = 0;
	for (int i = 0; i <= n; i++)
	{
		int c = Combination(n, i);
		float p1 = FloatPowerInt(1 - t, n - i - 1);
		float p2 = FloatPowerInt(t, i);
		float p3 = FloatPowerInt(t, i - 1);
		float p4 = FloatPowerInt(1 - t, n - i);

		float f = -(n-i) * p1 * p2 + i * p4 * p3;
		dx1 += c * m_ptKey[i].x * f;
		dy1 += c * m_ptKey[i].y * f;
	}

	// ������׵���(dx2, dy2)
	dx2 = 6 * (m_ptKey[2].x - 2 * m_ptKey[1].x + m_ptKey[0].x) +
		6 * (m_ptKey[3].x - 3 * m_ptKey[2].x + 3 * m_ptKey[1].x - m_ptKey[0].x) * t;

	dy2 = 6 * (m_ptKey[2].y - 2 * m_ptKey[1].y + m_ptKey[0].y) +
		6 * (m_ptKey[3].y - 3 * m_ptKey[2].y + 3 * m_ptKey[1].y - m_ptKey[0].y) * t;

	// ������׵���(dx2, dy2)
	float _dx2, _dy2;
	_dx2 = _dy2 = 0;
	for (int i = 0; i <= n; i++)
	{
		int c = Combination(n, i);
		float p1 = (n - i - 1 >= 0) ? FloatPowerInt(1 - t, n - i - 1) : 0;
		float p2 = FloatPowerInt(t, i);
		float p3 = FloatPowerInt(t, i - 1);
		float p4 = FloatPowerInt(1 - t, n - i);
		float p5 = (i - 2 >= 0) ? FloatPowerInt(t, i - 2) : 0;
		float p6 = (n - i - 2 >= 0) ? FloatPowerInt(1 - t, n - i - 2) : 0;

		float f = (n-i) * ((-(n-i-1)) * p6 * p2 + p1 * i * p3) + 
			        i * ((-n+i) * p1 * p3 + p4 * (i - 1) * p5);
		_dx2 += c * m_ptKey[i].x * f;
		_dy2 += c * m_ptKey[i].y * f;
	}

	float dd3 = dx2 - _dx2;
	float dd4 = dy2 - _dy2;
#endif


	// �������ߵ�����
	float f = dx1* dx1 + dy1 * dy1;
	f = (float)sqrt(f * f * f);
	m_fCurvature = (dx1 * dy2 - dx2 * dy1) / f;

	// �������ߵ����߽�
	m_angTangent = (float)atan2(dy1, dx1);
}

//
//   ���ݾ����Ľ��Ⱦ���fLenȷ����Ӧ��tֵ��
//
float CBezier::GetTFromProgress(float fLen)
{
	float t;

	if (fLen == 0)
		return 0;
	else if (fLen >= m_fTotalLen)
		return 1;

	// ���Ҷ�Ӧ��fLen���ȵ����䣬��ͨ�����Բ岹�õ���Ӧ��tֵ

#if 0
	// ��������!
	// ͨ���۰��������ٶ�
	int nMin, nMax, n;
	nMin = 0;
	nMax = m_nSampleCount;
	n = (nMin + nMax) / 2;

	while (nMax - nMin > 2)
	{
		float fProgress = m_pSamplePoints[n].fProgress;
		if (fProgress > fLen)
		{
			nMax = n;
		}
		n = (nMin + nMax) / 2;
	}
#endif

	for (int i = 0; i < m_nSampleCount; i++)
	{
		float fProgress = m_pSamplePoints[i].fProgress;
		if (fProgress > fLen)
		{
			float fProgress_1 = m_pSamplePoints[i - 1].fProgress;
			t = (float)(i - 1) / m_nSampleCount;
			t *= 1 + (fLen - fProgress_1) / (fProgress - fProgress_1);
			return t;
		}
	}

	return 1;
}

//
//   ����������һ��pt���������������ĵ㡣
//
bool CBezier::GetClosestPoint(const CPnt& pt, CPnt* pptClosest, float* pT)
{
	// �ȼٶ���һ��Ϊ�����
	int nClosest = 0;
	CPnt ptClosestSample = m_pSamplePoints[0].GetPntObject();

	// �����ڸ��������в��Ҿ��������pt����ĵ�
	float fMinDist = ptClosestSample.DistanceTo(pt);
	for (int i = 1; i < m_nSampleCount; i++)
	{
		// ȡ��һ��������
		CPnt& ptSample = m_pSamplePoints[i].GetPntObject();

		// ��������㵽������֮��ľ���
		float fDist = ptSample.DistanceTo(pt);

		// �����������������
		if (fDist < fMinDist)
		{
			ptClosestSample = pt;
			fMinDist = fDist;
			nClosest = i;
		}
	}

	// ����ͨ���岹�������Ը��ߵľ��ȼ��������
	CPnt pt1, pt2;
	float t1, t2;

	// �������������ǵ�һ��������
	if (nClosest == 0)
	{
		pt1 = m_pSamplePoints[0];
		pt2 = m_pSamplePoints[1];

		t1 = m_pSamplePoints[0].t;
		t2 = m_pSamplePoints[1].t;
	}
	// �����������������һ��������
	else if (nClosest == m_nSampleCount - 1)
	{
		pt1 = m_pSamplePoints[m_nSampleCount - 1];
		pt2 = m_ptKey[m_nCountKeyPoints - 1];

		t1 = m_pSamplePoints[m_nSampleCount - 1].t;
		t2 = 1;
	}

	// �����������㲻�����ߵ�����
	else
	{
		// ��������뵱ǰ������������ڵ����������㼰���Ӧ��tֵ
		pt1 = m_pSamplePoints[nClosest - 1];
		pt2 = m_pSamplePoints[nClosest + 1];

		t1 = m_pSamplePoints[nClosest - 1].t;
		t2 = m_pSamplePoints[nClosest + 1].t;
	}

	// ����ֱ��
	CLine ln(pt1, pt2);

	// ���㵱ǰ��̬������ֱ�ߵ�ͶӰ��
	CPnt ptFoot;
	float fLambda;

	// ����ָ���㵽��ֱ�ߵ�ͶӰ��
	ln.DistanceToPoint(false, pt, &fLambda, &ptFoot);

	if (fLambda < 0)
	{
		if (nClosest == 0 && ptFoot.DistanceTo(m_ptKey[0]) < 0.03f)
		{
			if (pptClosest != NULL)
				*pptClosest = m_ptKey[0];

			if (pT != NULL)
				*pT = 0;

			return true;
		}
		else
			return false;
	}
	else if (fLambda > 1)
	{
		if (nClosest == m_nSampleCount - 1 && ptFoot.DistanceTo(m_ptKey[m_nCountKeyPoints - 1]) < 0.03f)
		{
			if (pptClosest != NULL)
				*pptClosest = m_ptKey[m_nCountKeyPoints - 1];

			if (pT != NULL)
				*pT = 1;

			return true;
		}
		else
			return false;
	}
	else
	{
		// ����ͶӰ�㵽ֱ�����˵ľ���
		float d1 = ptFoot.DistanceTo(pt1);
		float d2 = ptFoot.DistanceTo(pt2);

		// �õ�(�������)���������Ӧ��t
		float t = (d1 * t2 + d2 * t1) / (d1 + d2);

		// ����õ�������������
		SetCurT(t);

		if (pptClosest != NULL)
			*pptClosest = ptFoot;

		if (pT != NULL)
			*pT = t;

		return true;
	}
}

#ifdef _MFC_VER

//
//   �Ӷ������ļ�װ���������ݡ�
//
bool CBezier::Create(CArchive& ar)
{
	int nCountKeyPoints;

#if 0
	// ����ؼ�������
	ar >> nCountKeyPoints;
#else
	nCountKeyPoints = 2;
#endif

	// ��ʱ����ռ�
	CPnt* pptKey = new CPnt[nCountKeyPoints];
	if (pptKey == NULL)
		return false;

	// ����ؼ�������
	for (int i = 0; i < nCountKeyPoints; i++)
		ar >> m_ptKey[i].x >> m_ptKey[i].y;

	// ��������
	if (!Create(nCountKeyPoints, pptKey))
		return false;

	// �ͷ���ʱ�ռ�
	delete []pptKey;

	return true;
}

//
//   ���������ݱ��浽�������ļ���
//
bool CBezier::Save(CArchive& ar)
{
	ar << m_nCountKeyPoints;

	for (int i = 0; i < m_nCountKeyPoints; i++)
		ar << m_ptKey[i].x << m_ptKey[i].y;

	return true;
}

//
//   ����Ļ�ϻ��ƴ�ֱ�ߡ�
//
void CBezier::Draw(CScreenReference& ScrnRef, CDC* pDC, COLORREF crColor, int nWidth, int nPointSize, bool bShowKeyPoints, int nPenStyle)
{
	CPen pen(nPenStyle, nWidth, crColor);
	CPen* pOldPen = pDC->SelectObject(&pen);

	CPoint pntLast;

	for (int i = 0; i < 500; i++)
	{
		float t = i / 500.0f;

		SetCurT(t);
		CPoint pnt = ScrnRef.GetWindowPoint(m_pt);

		if (i != 0)
		{
			pDC->MoveTo(pntLast);
			pDC->LineTo(pnt);
		}
		pntLast = pnt;
	}

	m_ptKey[0].Draw(ScrnRef, pDC, crColor, nPointSize);
	m_ptKey[m_nCountKeyPoints - 1].Draw(ScrnRef, pDC, crColor, nPointSize);

	pDC->SelectObject(pOldPen);
}

//
//   �������Ƶ㡣
//
void CBezier::DrawCtrlPoints(CScreenReference& ScrnRef, CDC* pDC, COLORREF crColor, int nPointSize)
{
	for (int i = 1; i < m_nCountKeyPoints - 1; i++)
		m_ptKey[i].Draw(ScrnRef, pDC, crColor, nPointSize);
}
#endif
