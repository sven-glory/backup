#include "stdafx.h"
#include "ShortLineFeature.h"
#include "Scan.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif

///////////////////////////////////////////////////////////////////////////////

CShortLineFeature::CShortLineFeature() 
{
	m_nSubType = SHORT_LINE_FEATURE;
	m_fWidth = 0.1f;
	m_fAngle = 0;
	m_nWhichSideToUse = FEATURE_DIR_BOTH_SIDES;    // ˫�����
	m_fMaxIncidenceAngle = PI/2;     // �����������Ϊ90��
	m_nMinNumPoints = 2;             // ���ٵ���Ϊ2
	m_fMaxLength = 300;              // �߶���󳤶�0.3m
}

//
//   ����һ������
//
CPointFeature* CShortLineFeature::Duplicate() const
{
	CShortLineFeature* p = new CShortLineFeature;
	*p = *this;
	return p;
}

//
//   ���ö�ֱ�߶������Ĳ�����
//
void CShortLineFeature::SetParam(float fWidth, float fAngle, int nWhichSideToUse, 
											float fMaxIncidenceAngle, int nMinNumPoints,
											float fMaxLength)
{
	m_fWidth = fWidth;
	m_fAngle = fAngle;
	m_nWhichSideToUse = nWhichSideToUse;
	m_fMaxIncidenceAngle = fMaxIncidenceAngle;
	m_nMinNumPoints = nMinNumPoints;
	m_fMaxLength = fMaxLength;

	// �����Ӧ��ֱ�߶�
	CAngle ang(m_fAngle);
	m_ln.Create(GetPntObject(), ang, m_fWidth/2);
	m_ln.Resize(m_fWidth/2, 0);
}

//
//   �ж϶�ֱ�߶��Ƿ�ָ����ɨ�����յ���������ɨ������ꡣ�ú�����Ҫ���ڷ��档
//
bool CShortLineFeature::HitByLineAt(CLine& lnRay, CPnt& ptHit, float& fDist)
{
	CPnt pt;
	float f;

	// �ж������߶��Ƿ��н���
	bool bHit = lnRay.IntersectLineAt(m_ln, pt, f);
	
	// ����н��㣬��������Ǻϸ�
	if (bHit && CheckInRay(lnRay))
	{
		ptHit = pt;
		fDist = f;
		return true;
	}

	// û�н��㣬���治���ã�������ǳ��ޣ�����false
	return false;
}

//
//   ��Ը����ĵ��ƣ��ڹ涨�ĽǶȷ�Χ�ڣ����������Ƿ��иö�ֱ�߶���������������������λ�á�
//   ע�⣺ֻ���ڹ涨���漰����Ƕ����з��䡣
//
bool CShortLineFeature::Detect(CPosture& pst, CScan* pScan, float fStartAngle, float fEndAngle, CPnt* ptCenter)
{
	CScanPointCloud* pCloud = pScan->GetScanPointCloudPointer();

	// ------------- �ȸ��������桢����ǵȷ����������Ƿ���ܱ�����  --------------------
	
	// ����Ӽ���ͷ���������ĵ�ֱ�߶�
	CLine lnRay(pst.GetPntObject(), GetPntObject());
	if (!CheckInRay(lnRay))
		return false;

	// ------------------- ���濪ʼ���������Ƿ��и�����  ----------------------------

	int nFeatureStart = -1;    // ��ֱ�������ε����λ��
	int nFeatureEnd = -1;      // ��ֱ�������εĽ���λ��

	// ���㼤�������������ľ���
	float fEstimatedDist = DistanceTo(pst.GetPntObject());
	float fLastR;

	// �ڸ����ķ�Χ������������
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
			// ������ڶ��У���Ϊ�˶ν���
			if (nFeatureStart >= 0 && nFeatureEnd < 0)
			{
				// ���������ֱ�ߵ���Ҫ����Ϊ������һ����Ч������
				if (i - nFeatureStart >= m_nMinNumPoints)
				{
					nFeatureEnd = i - 1;
				}
				// ���򣬷����������Ķ�
				else
				{
					nFeatureStart = nFeatureEnd = -1;
				}
			}
			continue;
		}

		// ��������ҵ�����������ĵ㣬����Ϊ����һ����Ӧ�ڶ�ֱ�ߵĵ���Ƭ��
		else if (nFeatureStart < 0)
		{
			// ��¼�ε���ʼλ�ú͵�ǰ��ļ���
			nFeatureStart = i;
			fLastR = sp.r;
		}

		// ��������֮�䲻����������Ϊ��ǰ�ν���
		else if (fabs(sp.r - fLastR) > SHORT_LINE_MAX_DIST_CHANGE)
		{
			if (nFeatureEnd < 0)
			{
				// ���������ֱ�ߵ���Ҫ����Ϊ������һ����Ч������
				if (i - nFeatureStart >= m_nMinNumPoints)
				{
					nFeatureEnd = i - 1;

					// break - ��ʱ����!
					break;
				}
				// ���򣬷����������Ķ�
				else
				{
					nFeatureStart = nFeatureEnd = -1;
				}
			}
		}

		// ��������ι�����˵��Ҳ���Ǻϸ�Ķ�ֱ������
		else if (i - nFeatureStart > MAX_POINT_NUM_IN_SHORT_LINE)
		{
			nFeatureStart = nFeatureEnd = -1;
		}
		else
		{
			fLastR = sp.r;
		}
	}

	// ���û�ҵ�����������false
	if (nFeatureStart < 0)
		return false;
	else if (nFeatureEnd < 0)
	{
		nFeatureEnd = pCloud->m_nCount - 1;
	}

	// ͳ�ƺϸ����Ƭ�ε����ĵ�
	float fSumX = 0;
	float fSumY = 0;
	int nSegCount = nFeatureEnd - nFeatureStart + 1;

	for (int i = nFeatureStart; i <= nFeatureEnd; i++)
	{
		CScanPoint& sp = pCloud->m_pPoints[i];
		fSumX += sp.x;
		fSumY += sp.y;
	}

	// ���ĵ�
	CPnt ptCenterGravity(fSumX / nSegCount, fSumY / nSegCount);

	*ptCenter = ptCenterGravity;
	CLine ln(pst.GetPntObject(), ptCenterGravity);
	CAngle ang = ln.SlantAngle() - pst.GetAngle();

	ptCenter->r = ln.Length();
	ptCenter->a = CAngle::NormAngle2(ang.m_fRad);

	return true;
}

//
//   ����һ��������
//
CPointFeature* CShortLineFeature::Duplicate()
{
	CShortLineFeature* p = new CShortLineFeature;
	*p = *this;
	return (CPointFeature*)p;
}

//
//   �ж�ָ������������Ƿ���á�
//
bool CShortLineFeature::CheckInRay(CLine& lnRay)
{
	CAngle angReverseRay = !lnRay.SlantAngle();

	// �ж�������ߴ���һ�����䵽�߶���(������С��PI��˵���յ�����)
	CAngle angDiff = angReverseRay - m_ln.SlantAngle();
	
	CAngle angNormal;        // �߶η����

	// �������������
	if (angDiff.m_fRad < PI)
	{
		// ���������ã����㷨���
		if (m_nWhichSideToUse == FEATURE_DIR_FRONT_SIDE_ONLY ||
			 m_nWhichSideToUse == FEATURE_DIR_BOTH_SIDES)
			angNormal = m_ln.SlantAngle() + PI/2;
		else
			return false;    // ���治����
	}

	// ����������Է��棬Ҫ�������Ƿ�����ʹ��
	else if (m_nWhichSideToUse == FEATURE_DIR_FRONT_SIDE_ONLY)
	{
		return false;       // ���治����
	}

	// �������
	else
	{
		angNormal = m_ln.SlantAngle() - PI/2;
	}

	// ����ʵ������ǣ��������ǲ�����Ҫ��˵����������߲�����
	float fIncidenceAngle = angReverseRay.GetDifference(angNormal);
	if (fIncidenceAngle > m_fMaxIncidenceAngle)
		return false;
	else
		return true;
}

//
//   ���ı��ļ���װ���ֱ�߶�����������
//   ����ֵ��
//     -1  : ��ȡʧ��
//     >=0 : �����������ͱ��
//
int CShortLineFeature::LoadText(FILE* fp)
{
	if (CPointFeature::LoadText(fp) < 0)
		return -1;

	float fWidth, fAngle, fMaxIncidenceAngle, fMaxLength;
	int nWhichSideToUse, nMinNumPoints;

	if (fscanf(fp, "%f\t%f\t%d\t%f\t%d\t%f\n", &fWidth, &fAngle, &nWhichSideToUse, 
		&fMaxIncidenceAngle, &nMinNumPoints, &fMaxLength) != 6)
		return -1;
	
	// ������Ӧ����
	SetParam(fWidth, fAngle, nWhichSideToUse, fMaxIncidenceAngle, nMinNumPoints,
		fMaxLength);

	return m_nSubType;
}

//
//   ����ֱ�߶������������浽�ı��ļ��С�
//   ����ֵ��
//     -1  : д��ʧ��
//     >=0 : �����������ͱ��
//
int CShortLineFeature::SaveText(FILE* fp)
{
	fprintf(fp, "%d\t%f\t%f\t", m_nSubType, x, y);
	fprintf(fp, "%f\t%f\t%d\t%f\t%d\t%f\n", m_fWidth, m_fAngle, m_nWhichSideToUse, 
		m_fMaxIncidenceAngle, m_nMinNumPoints, m_fMaxLength);

	return m_nSubType;
}

//
//   �Ӷ������ļ���װ��������Ĳ�����
//
int CShortLineFeature::LoadBinary(FILE* fp)
{
	if (CPointFeature::LoadBinary(fp) < 0)
		return -1;

	float fWidth, fAngle, fMaxIncidenceAngle, fMaxLength;
	int nWhichSideToUse, nMinNumPoints;

	if (fread(&fWidth, sizeof(float), 1, fp) != 1)
		return -1;

	if (fread(&fAngle, sizeof(float), 1, fp) != 1)
		return -1;

	if (fread(&nWhichSideToUse, sizeof(int), 1, fp) != 1)
		return -1;

	if (fread(&fMaxIncidenceAngle, sizeof(float), 1, fp) != 1)
		return -1;

	if (fread(&nMinNumPoints, sizeof(int), 1, fp) != 1)
		return -1;

	if (fread(&fMaxLength, sizeof(float), 1, fp) != 1)
		return -1;

	// ������Ӧ����
	SetParam(fWidth, fAngle, nWhichSideToUse, fMaxIncidenceAngle, nMinNumPoints,
		fMaxLength);

	return m_nSubType;
}

//
//   ���������Ĳ������浽�������ļ��С�
//
int CShortLineFeature::SaveBinary(FILE* fp)
{
	if (CPointFeature::SaveBinary(fp) < 0)
		return -1;

	if (fwrite(&m_fWidth, sizeof(float), 1, fp) != 1)
		return -1;

	if (fwrite(&m_fAngle, sizeof(float), 1, fp) != 1)
		return -1;

	if (fwrite(&m_nWhichSideToUse, sizeof(int), 1, fp) != 1)
		return -1;

	if (fwrite(&m_fMaxIncidenceAngle, sizeof(float), 1, fp) != 1)
		return -1;

	if (fwrite(&m_nMinNumPoints, sizeof(int), 1, fp) != 1)
		return -1;

	if (fwrite(&m_fMaxLength, sizeof(float), 1, fp) != 1)
		return -1;

	return m_nSubType;
}

//
//   �����������任��
//
void CShortLineFeature::Transform(const CFrame& frame)
{
	CPointFeature::Transform(frame);
	m_ln.Transform(frame);
}

//
//   ����������任��
//
void CShortLineFeature::InvTransform(const CFrame& frame)
{
	CPointFeature::InvTransform(frame);
	m_ln.InvTransform(frame);
}

#ifdef _MFC_VER

//
//   ����Ļ�ϻ��ƴ˵�������
//
void CShortLineFeature::Plot(CScreenReference& ScrnRef, CDC* pDc, COLORREF crColor, COLORREF crSelected, int nSize)
{
//	m_ptCenter.Draw(ScrnRef, pDC, RGB(255, 0, 255), 5);

	CPen Pen(PS_SOLID, 3, crColor);
	CPen* pOldPen = pDc->SelectObject(&Pen);

	// ����ֱ�߶�
	CPnt& ptStart = m_ln.m_ptStart;
	CPnt& ptEnd = m_ln.m_ptEnd;

	CPoint pnt1 = ScrnRef.GetWindowPoint(ptStart);
	CPoint pnt2 = ScrnRef.GetWindowPoint(ptEnd);

	pDc->MoveTo(pnt1);
	pDc->LineTo(pnt2);

	pDc->SelectObject(pOldPen);

}
#endif
