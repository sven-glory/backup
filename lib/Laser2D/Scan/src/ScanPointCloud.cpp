#include "stdafx.h"
#include <string.h>
#include "ScanPointCloud.h"
#include "Frame.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif

///////////////////////////////////////////////////////////////////////////////
//   ��CScanPointCloud�����ʵ�֡�

//
//   ����ָ���ĵ��������ɶ���(ֻ����ռ�)��
//
CScanPointCloud::CScanPointCloud(int nNum)
{
	m_nCount = nNum;
	m_pPoints = new CScanPoint[nNum];

	ASSERT(m_pPoints != NULL);
}

//
//   ��������һ���������ɶ���(����ȫ����ɨ���)��
//
CScanPointCloud::CScanPointCloud(const CScanPointCloud& Cloud)
{
	m_nCount = 0;
	m_pPoints = NULL;

	*this = Cloud;
}

//
//   ���ɿյĶ���
//
CScanPointCloud::CScanPointCloud()
{
	m_nCount = 0;
	m_pPoints = NULL;
}

//
//   ����������
//
CScanPointCloud::~CScanPointCloud()
{
	Clear();
}

//
//   �������ԭ�������ݡ�
//
void CScanPointCloud::Clear()
{
	if (m_pPoints != NULL)
		delete []m_pPoints;

	m_nCount = 0;
	m_pPoints = NULL;
}

//
//   Ϊ�������ݷ���ռ䡣
//
bool CScanPointCloud::Create(int nNum)
{
	m_pPoints = new CScanPoint[nNum];

	if (m_pPoints == NULL)
	{
		m_nCount = 0;
		return false;
	}
	else
	{
		m_nCount = nNum;
		return true;
	}
}

//
//   ��㼯ĩβ���һ���㡣
//   ע�⣺�˲�����Ϊ�������·���ռ䣬ԭ�еĿռ佫���ͷš�
//
bool CScanPointCloud::Add(const CScanPoint& sp)
{
	CScanPoint* pNewBuf = new CScanPoint[m_nCount+1];
	if (pNewBuf == NULL)
		return false;

	// ����ԭ������
	memmove(pNewBuf, m_pPoints, sizeof(CScanPoint) * m_nCount);

	// �ͷ�ԭ���ĵ��ƿռ�
	delete []m_pPoints;
	
	// ָ���µĻ�����
	m_pPoints = pNewBuf;

	// ����µ�
	m_pPoints[m_nCount++] = sp;

	return true;
}

//
//   �ӵ�����ɾ��һ���㡣
//   ע�⣺ɾ������ƿռ����ж���δ�õĿռ䡣
//
bool CScanPointCloud::Delete(int nIndex)
{
	if (nIndex < 0 || nIndex >= m_nCount)
		return false;

	for (int i = nIndex; i < m_nCount - 1; i++)
		m_pPoints[i] = m_pPoints[i + 1];

	// ���ĵ����е������(���ƿռ��п����ж���δ�õĿռ�)
	m_nCount--;

	return true;
}

//
//   ���ء�=����������
//
void CScanPointCloud::operator = (const CScanPointCloud& Cloud2)
{
	if (m_nCount != Cloud2.m_nCount)
	{
		Clear();
		VERIFY(Create(Cloud2.m_nCount));
	}

	for (int i = 0; i < m_nCount; i++)
		m_pPoints[i] = Cloud2.m_pPoints[i];

	Stamp(Cloud2.m_uTime);
}

//
//   ���ء�+=����������
//
void CScanPointCloud::operator += (const CScanPointCloud& Cloud2)
{
	// �ȸ���ԭ���ĵ��ơ�
	CScanPointCloud CopyThis(*this);

	// ���ԭ��������
	Clear();

	// ���·���ռ��Ա������������Ƶ�����
	VERIFY(Create(CopyThis.m_nCount + Cloud2.m_nCount));

	int i;

	// ���ƻ�ԭ������
	for (i = 0; i < CopyThis.m_nCount; i++)
		m_pPoints[i] = CopyThis.m_pPoints[i];

	// ���������µ�������
	for (i = 0; i < Cloud2.m_nCount; i++)
		m_pPoints[CopyThis.m_nCount + i] = Cloud2.m_pPoints[i];
}

//
//   ���ı��ļ���װ��������ݡ�
//
bool CScanPointCloud::Load(FILE* file)
{
	int nCount;
	fscanf(file, "%d\n", &nCount);

	Clear();
	if (!Create(nCount))
		return false;

	for (int i = 0; i < nCount; i++)
	{
		CScanPoint& sp = m_pPoints[i];
		fscanf(file, "%f\t%f\n", &(sp.x), &(sp.y));
		sp.id = i;
	}

	return true;
}

//
//   ���������ݴ����ı��ļ���
//
bool CScanPointCloud::Save(FILE* file)
{
	fprintf(file, "%ld\n", m_nCount);
	for (int i = 0; i < m_nCount; i++)
	{
		CScanPoint& sp = m_pPoints[i];
		fprintf(file, "%.6f\t%.6f\n", sp.x, sp.y);
	}

	return true;
}

//
//   �Ӷ������ļ��ж�ȡɨ�����ݡ�
//
bool CScanPointCloud::LoadBinary(FILE* fp, float fStartAngle, float fEndAngle, int nLineCount, 
	int nFileVersion)
{
	// ����ɨ��ĽǷֱ���
	float fAngReso = (fEndAngle - fStartAngle) / nLineCount;
	
	Clear();
	if (!Create(nLineCount))
		return false;

	// ����汾��V2.10���ϣ��ڴ���Ҫ����ʱ���
	unsigned int uTimeStamp;
	if (nFileVersion >= 210)
	{
		if (fread(&uTimeStamp, sizeof(unsigned int), 1, fp) != 1)
			return false;

		Stamp(uTimeStamp);
	}

	// ���ζ������е������
	for (int i = 0; i < m_nCount; i++)
	{
		float r;
		int nIntensity;

		// ����ļ���ʽ�汾������2.00��(����+ǿ�ȹ���8���ֽ�)
		if (nFileVersion < 200)
		{
			// ���뼫��
			if (fread(&r, sizeof(float), 1, fp) != 1)
				return false;

			// ���뷴��ǿ��
			if (fread(&nIntensity, sizeof(int), 1, fp) != 1)
				return false;
		}

		// ����ļ���ʽ��2.00������(����+ǿ�ȹ���3���ֽ�)
		else
		{
			unsigned short int ur;
			unsigned char ui;
			if (fread(&ur, sizeof(unsigned short int), 1, fp) != 1)
				return false;

			if (fread(&ui, sizeof(unsigned char), 1, fp) != 1)
				return false;

			r = ur * 2.0f;                       // �����ֱ���Ϊ2mm
			nIntensity = (int)ui * 10;                // ǿ�ȷ�Χ(0-255)
		}

		if (r < 0 || r > 65500)    // r > 65500Ϊ��ʱ��ʩ�����ɨ���߳���һȦ�뾶Ϊ65534�Ļ�������
			r = 0;

		CScanPoint& sp = m_pPoints[i];

		sp.r = r;
		sp.a = fStartAngle + fAngReso * i;
		sp.m_nIntensity = nIntensity;
		
		// ��������0����ʾ������Ч
		if (r >= 0)
		{
//			sp.r /= 1000;          // Ŀǰ�ļ���ʽ��mmΪ��λ����Ҫת��Ϊm
			// ����Ͽ�������
			sp.UpdateCartisian();

			sp.x /= 1000;          // Ŀǰ�ļ���ʽ��mmΪ��λ����Ҫת��Ϊm
			sp.y /= 1000;
		}
#if 0
		// ������Ч��
		else
			sp.x = sp.y = 0;
#endif

		sp.id = i;
	}

	return true;
}

//
//   ��ɨ�����ݱ��浽ʮ�����ļ���
//
bool CScanPointCloud::SaveBinary(FILE* fp, int nFileVersion)
{
	for (int i = 0; i < m_nCount; i++)
	{
		CScanPoint& sp = m_pPoints[i];

		// V2.00�汾��ʽ����
		if (nFileVersion < 200)
		{
			fwrite(&sp.r, sizeof(float), 1, fp);
			fwrite(&sp.m_nIntensity, sizeof(int), 1, fp);
		}

		// V2.00���ϰ汾��ʽ
		else
		{
			unsigned short int ur = sp.r / 2;                   // ������˫�ֽڱ�ʾ���ֱ���Ϊ2mm
			fwrite(&ur, sizeof(unsigned short int), 1, fp);

			unsigned char ui = (unsigned char)(sp.m_nIntensity / 10);  // ǿ���õ��ֽڱ�ʾ
			fwrite(&ui, sizeof(unsigned char), 1, fp);
		}
	}

	return true;
}

//
//   ���ݸ���ĵϿ��������������е�ļ����ꡣ
//
void CScanPointCloud::UpdatePolar()
{
	for (int i = 0; i < m_nCount; i++)
		m_pPoints[i].UpdatePolar();
}

//
//   ���ݼ������������е�ĵϿ������ꡣ
//
void CScanPointCloud::UpdateCartisian()
{
	for (int i = 0; i < m_nCount; i++)
		m_pPoints[i].UpdateCartisian();
}

//
//   ��ȫ�����ƶ�ָ���ľ��롣
//
void CScanPointCloud::Move(float fDx, float fDy)
{
	for (int i = 0; i < m_nCount; i++)
		m_pPoints[i].Move(fDx, fDy);
}

//
//   ��ȫ������ָ�������ĵ������ת��
//
void CScanPointCloud::Rotate(float centerX, float centerY, float angle)
{
	if (angle != 0.0)
	{
		float sina = sin(angle);
		float cosa = cos(angle);

		CScanPoint *sp = m_pPoints;

		float x, y;
		for (int i = 0; i < m_nCount; i++, sp++)
		{
			x = sp->x - centerX;
			y = sp->y - centerY;
			sp->x = centerX + cosa * x - sina * y;
			sp->y = centerY + sina * x + cosa * y;
		}
	}
}
#if 1
//
//   �������㼯��������任��
//
void CScanPointCloud::Transform(float x, float y, float thita)
{
	CPnt pt(x, y);
	CTransform trans(pt, CAngle(thita));

	for (int i = 0; i < m_nCount; i++)
	{
		CPnt pt = trans.GetWorldPoint(m_pPoints[i]);
		m_pPoints[i].x = pt.x;
		m_pPoints[i].y = pt.y;
	}
}

//
//   ���������ƽ�������任(�ڶ�����ʽ)��
//
void CScanPointCloud::Transform(const CPosture& pstOrigin)
{
	Transform(pstOrigin.x, pstOrigin.y, pstOrigin.fThita);
}
#endif

//
//   ���������ƽ�������ϵ�任��
//
void CScanPointCloud::Transform(const CFrame& frame)
{
	for (int i = 0; i < m_nCount; i++)
		m_pPoints[i].Transform(frame);
}

//
//   ���������ƽ�������ϵ��任��
//
void CScanPointCloud::InvTransform(const CFrame& frame)
{
	for (int i = 0; i < m_nCount; i++)
		m_pPoints[i].InvTransform(frame);
}


//
//   ȡ�������X���ꡣ
//
float CScanPointCloud::LeftMost()
{
	if (m_nCount == 0)
		return -HUGE_VAL;

	float fMost = m_pPoints[0].x;
	for (USHORT i = 0; i < m_nCount; i++)
	{
		if (fMost > m_pPoints[i].x)
			fMost = m_pPoints[i].x;
	}

	return fMost;
}

//
//   ȡ�����ϵ�Y���ꡣ
//
float CScanPointCloud::TopMost()
{
	if (m_nCount == 0)
		return HUGE_VAL;

	float fMost = m_pPoints[0].y;
	for (USHORT i = 0; i < m_nCount; i++)
	{
		if (fMost < m_pPoints[i].y)
			fMost = m_pPoints[i].y;
	}

	return fMost;
}

//
//   ȡ�����ҵ�X���ꡣ
//
float CScanPointCloud::RightMost()
{
	if (m_nCount == 0)
		return HUGE_VAL;

	float fMost = m_pPoints[0].x;
	for (USHORT i = 0; i < m_nCount; i++)
	{
		if (fMost < m_pPoints[i].x)
			fMost = m_pPoints[i].x;
	}

	return fMost;
}

//
//   ȡ�����µ�Y���ꡣ
//
float CScanPointCloud::BottomMost()
{
	if (m_nCount == 0)
		return -HUGE_VAL;

	float fMost = m_pPoints[0].y;
	for (USHORT i = 0; i < m_nCount; i++)
	{
		if (fMost > m_pPoints[i].y)
			fMost = m_pPoints[i].y;
	}

	return fMost;
}

//
//   ȡ�õ��Ƶ�X���ȡ�
//
float CScanPointCloud::Width()
{
	return RightMost() - LeftMost();
}

//
//   ȡ�õ��Ƶ�Y��߶ȡ�
//
float CScanPointCloud::Height()
{
	return TopMost() - BottomMost();
}

//
//   ȡ�ô˵㼯�����ǵ��������
//
CRectangle CScanPointCloud::GetCoveringRect()
{
	CRectangle r;
	for (int i = 0; i < m_nCount; i++)
		r += m_pPoints[i];

	return r;
}

//
//   �˶Ե����Ƿ����ָ���ĵ㡣
//   ����ֵ��
//     -1: δ�ҵ�
//     0~n: �ҵ��ĵ�����
//
int CScanPointCloud::ContainPoint(const CPnt& pt, float fThreshHoldDist)
{
	for (int i = 0; i < m_nCount; i++)
		if (m_pPoints[i].DistanceTo(pt) <= fThreshHoldDist)
			return i;

	return -1;
}

//
//   ��ָ���ĵ��Ϊ���ģ���ָ���İ뾶�����е���й��ˣ�ֻ���´��ڰ뾶���ڵĵ㡣
//   ע�⣺
//      1. ���˴����������Ч�ĵ㶼�Ѿ�����ȷ�ļ���ֵ
//
void CScanPointCloud::ReduceByRadius(const CPnt& ptCenter, float dRadius)
{
	int nNewCount = 0;                     // ���Ƶ������

	for (int i = 0; i < m_nCount; i++)
	{
		// ����õ㵽���ĵ�ľ���
		float fDist = m_pPoints[i].DistanceTo(ptCenter);

		// �������С��ָ���뾶���򽫸õ������µ�����
		if (fDist < dRadius)
		{
			// �ڴ˸����ϼ���ֵ
			m_pPoints[i].r = fDist;

			if (nNewCount != i)
				m_pPoints[nNewCount] = m_pPoints[i];

			nNewCount++;
		}
	}

	// �趨�µ����е������(���ƿռ��п����ж���δ�õĿռ�)
	m_nCount = nNewCount;

	// ɾ�����б���Ϊ��m_bDelete���ĵ� (Ϊ��ִ���������???)
	RemoveDeletedPoints();
}

//
//   ��ָ���ķ����Ϊ���ģ�ָ���ĽǶȷ�Χ�����е���й��ˣ�ֻ������Ч�Ƕ��ڵĵ㡣
//   ע�⣺
//      1. ���˴����������Ч�ĵ㶼�Ѿ�����ȷ�ļ���ֵ
//      2. ��˴���󣬵��ƻ������ڻ��б���Ϊ��m_bDelete������δɾ���ĵ�
//
void CScanPointCloud::ReduceByAngle(CPosture& pst, float fViewAngle)
{
	CPnt& ptCenter = pst.GetPntObject();

	for (int i = 0; i < m_nCount; i++)
	{
		// ����ɨ����
		CScanPoint& sp = m_pPoints[i];

		// ������Щ����Ϊ����ɾ�����ĵ�
		if (sp.m_bDelete)
			continue;

		CLine ln(ptCenter, sp.GetPntObject());

		// ��¼ɨ����ɨ���
		sp.a = (ln.SlantAngle() - pst.GetAngle()).NormAngle2();

		// ����õ������ĵ����ɼнǲ��ڼ���ɨ�������ܵļнǷ�Χ�ڣ��򽫸õ�ӵ������޳�
		CAngle angDiff(sp.a);
		if (fabs(angDiff.NormAngle2()) > fViewAngle / 2)
		{
			sp.m_bDelete = true;
		}
	}
}

//
//   �ж�����ɨ�����һ��ɨ��Ǹ���
//
//   -1: �ڶ�����ɨ��Ǹ���
//    1: ��һ����ɨ��Ǹ���
//    0: ����һ��
//
int AngleSort(const void *x1, const void *x2)
{
	CScanPoint *sp1 = (CScanPoint*)x1;
	CScanPoint *sp2 = (CScanPoint*)x2;

	float d = sp1->a - sp2->a;

	if (d < 0.0)
		return -1;
	else if (d > 0.0)
		return 1;

	return 0;
}

//
//   ����ָ������̬���Ե��Ƹ���ɨ��ǽ�������
//
void CScanPointCloud::SortByAngle()
{
	qsort(m_pPoints, m_nCount, sizeof(CScanPoint), AngleSort);
}

//
//   ����Ѱ��Ƕ�����ĵ��ƣ�ɾ����Щ���ڵ��ĵ㡣
//
void CScanPointCloud::RemoveHiddenPoints(const CPosture& pst, float fDistGate)
{
	bool bNewSeg = true;

	int j;
	for (int i = 0; i < m_nCount - 1; i++)
	{
		CScanPoint* pt1 = &(m_pPoints[i]);

		// ������Щ�ѱ���Ϊ��ɾ�����ĵ�
		if (pt1->m_bDelete)
			continue;

		// ���㳬�޽�
		float fMaxAngGap = atan2(fDistGate, pt1->r);

		// ׼����¼pt1���ĵ�һ�����ڵ�����
		int nNeighbor = 0;

		// ��i�����һ����Ч���ڵ�
		for (j = i + 1; j < m_nCount; j++)
		{
			// ������ǰ��j��Ϊpt2
			CScanPoint& pt2 = m_pPoints[j];

			if (pt2.m_bDelete)
				continue;

			// ���p2�ļ�����pt1�ļ���֮��ޣ�������֮��ľ���ض�����
			if (pt2.a - pt1->a > fMaxAngGap)
				break;

			// ����p1�㵽p2��ľ��룬���С���趨�����ޣ�����Ϊp1��p2��������
			if (pt1->DistanceTo(pt2) < fDistGate)
			{
				nNeighbor = j;
				break;
			}
		}

		// ������Ч�����ڵ㣬pt1����
		if (nNeighbor == 0)
		{
			// �����ǰ�����¶εĵ�һ�㣬��˵�����Ǹ��µ�(�����κ�����������)
			if (bNewSeg)
				pt1->m_bDelete = true;

			// ����ĵ㽫��ʼһ���¶�
			bNewSeg = true;

			continue;
		}

		// p1�����ڽӵ㣬�����ǹµ�
		bNewSeg = false;

		// ��pt2Ϊ�ҵ�����pt1��������ڵ�
		CScanPoint& pt2 = m_pPoints[nNeighbor];

		// ������Ҫ����pt1��pt2֮��ĵ�
		for (j = i + 1; j < nNeighbor; j++)
		{
			CScanPoint& pt3 = m_pPoints[j];

			if (pt3.m_bDelete)
				continue;

			// ���pt1��pt2֮��ĵ㼫��������������ļ�������Ӧɾ��֮
			if (pt3.r > pt1->r && pt3.r > pt2.r)
			{
				pt3.m_bDelete = true;
			}
		}
	}

	RemoveDeletedPoints();
}

//
//   �����г��������޵ĵ��Ƴ���
//
void CScanPointCloud::RemoveOutOfRangePoints()
{
	for (int i = 0; i < m_nCount; i++)
	{
		CScanPoint& sp = m_pPoints[i];
		if (sp.r == SCAN_OUT_OF_RANGE)
		{
			sp.m_bDelete = true;
		}
	}

	RemoveDeletedPoints();
}

#define RESAMPLE_MIN_DIST	    500

//
//    ��ָ���ĽǷֱ��ʺ�ɨ�����Ե��ƽ������²�����
//    ע�⣺ԭ���ĵ��Ʊ������Ѿ����Ƕ�������ġ�
//
CScanPointCloud* CScanPointCloud::ReSample(CPosture& pstScanner, float fStartAng, float fViewAngle, float fAngReso, float fMaxRange)
{
	bool bFinish = false;

	// �����µ���Ӧ���ĵ���(���4��5��)
	int nCount = (int)(fViewAngle / fAngReso + 0.5f);

	// Ϊ�µĵ��Ʒ���ռ�
	CScanPointCloud* pNewCloud = new CScanPointCloud(nCount);

	int i = 0, j;
	for (j = 0; j < nCount; j++)
	{
		CScanPoint& spNew = pNewCloud->m_pPoints[j];

		// ���ԭ�����е����е㶼�Ѳ鿴������ô�µ����к���ĵ㶼Ӧ��ΪSCAN_OUT_OF_RANGE
		if (bFinish)
		{
			spNew.SetOutOfRange();
			continue;
		}

		// ��ȷ����Ӧ�ڸ�ɨ���ߵ�ɨ���
		float fScanAng = fStartAng + j * fAngReso;

		// ת��������ڼ���ͷ��̬����Խ�
		CAngle angScan(fScanAng);
		angScan -= pstScanner.GetAngle();

		// ����ԭ�������ҵ���Ӧ����ʼ�Ƕȵĵ�
		while (m_pPoints[i].a < angScan.NormAngle2())
		{
			i++;

			// ��һ���Ƿ�ԭ�����е����е㶼�Ѳ鿴��
			if (i > m_nCount)
			{
				bFinish = true;
				break;
			}
		}

		if (bFinish)
			continue;

		// ����ԭ�����е���ʼ����i-1��i֮��
		// �����ɨ������ԭ���Ƽ������ڵ�����ɨ�������г�����Χ�ģ�������Ҳ��Ϊ������Χ
		if (m_pPoints[i - 1].r > fMaxRange || m_pPoints[i].r > fMaxRange)
		{
			spNew.SetOutOfRange();
		}
		// �������߲���ԭ���Ƽ��������ߵĲֵ
		else
		{
			// �����µ�ɨ����
			CAngle ang(fScanAng);
			CLine ln1(pstScanner.GetPntObject(), ang, fMaxRange);

			// �ȹ�����ԭ���Ƽ������ڵ�����ɨ��������
			CPnt& pt1 = m_pPoints[i - 1];
			CPnt& pt2 = m_pPoints[i];
			CLine ln2(pt1, pt2);

			// ��������ln1, ln2�����ߵĽ���
			CPnt ptIntersect;
			float r;
			if (ln1.IntersectLineAt(ln2, ptIntersect, r))
			{
				if (fabs(r - pt1.r) > RESAMPLE_MIN_DIST || fabs(r - pt2.r) > RESAMPLE_MIN_DIST)
				{
					spNew.SetOutOfRange();
				}
				else
				{
					spNew.GetPntObject() = ptIntersect;
					spNew.a = angScan.NormAngle2();
					spNew.r = r;
					spNew.m_bDelete = false;
				}
			}
			else
			{
				spNew.SetOutOfRange();
			}
		}
	}

	return pNewCloud;
}

//
//   �ڴ˽���Щ���Ϊ��ɾ�����ĵ�����ɾ����
//
void CScanPointCloud::RemoveDeletedPoints()
{
	int k = 0;
	for (int i = 0; i < m_nCount; i++)
	{
		if (!m_pPoints[i].m_bDelete)
			m_pPoints[k++] = m_pPoints[i];
	}

	// ��󣬵��������е������
	m_nCount = k;
}


//////////////////////////////////////////////////////////////////////////////
//   ���ݸ�����ģ�ͷ�������������ݡ�

void AddNoise(CPnt& ptIn, float fNoise, CPnt& ptOut)
{
	int nAng = rand() % 360;
	CAngle ang(nAng*PI/180);
	
	int nRadius = (int)(fNoise);
	if (nRadius == 0)
		nRadius = 1;
	nRadius = rand() % nRadius;
	if (nRadius == 0)
		nRadius = 1;

	float fRadius = nRadius;

	CLine ln(ptIn, ang, fRadius);
	ptOut = ln.m_ptEnd;
}

//
//   ���ݸ�����ֱ�߷������ɵ���.
//
bool CScanPointCloud::CreateFromLine(CLine& ln, float fGap, float fNoise)
{
	// ���ԭ�е���
	Clear();

	m_nCount = (int)(ln.Length() / fGap);
	m_pPoints = new CScanPoint[m_nCount];
	if (m_pPoints == NULL)
		return false;

	for (int i = 0; i < m_nCount; i++)
	{

		CPnt pt = ln.TrajFun(i * fGap);

		AddNoise(pt, fNoise, pt);

		m_pPoints[i].GetPntObject() = pt;
	}
	return true;
}

//
//   ���ݸ�����ֱ���������ɵ���.
//
bool CScanPointCloud::CreateFromLineArray(int nCountLines, CLine* lines, float fPointDist, float fNoise)
{
	Clear();

	CScanPointCloud cloud;

	for (int i = 0; i < nCountLines; i++)
	{
		cloud.CreateFromLine(lines[i], fPointDist, fNoise);
		*this += cloud;
	}
	return true;
}

//
//   ���ݸ�����Բ�������ɵ��ơ�
//
bool CScanPointCloud::CreateFromCircle(CCircle& circle, float fPointDist, float fNoise)
{
	// ���ԭ�е���
	Clear();

	m_nCount = (int)(circle.Perimeter() / fPointDist);
	m_pPoints = new CScanPoint[m_nCount];
	if (m_pPoints == NULL)
		return false;

	float fAngleStep = 2*PI / m_nCount;

	for (int i = 0; i < m_nCount; i++)
	{
		CPnt pt = circle.m_ptCenter;

		pt.x += circle.m_fRadius * cos(fAngleStep * i);
		pt.y += circle.m_fRadius * sin(fAngleStep * i);

		AddNoise(pt, fNoise, pt);

		m_pPoints[i].GetPntObject() = pt;
	}
	return true;
}

//
//   ���ݸ�����Բ���������ɵ��ơ�
//
bool CScanPointCloud::CreateFromCircleArray(int nCountCircles, CCircle* circles, float fPointDist, float fNoise)
{
	Clear();

	CScanPointCloud cloud;

	for (int i = 0; i < nCountCircles; i++)
	{
		cloud.CreateFromCircle(circles[i], fPointDist, fNoise);
		*this += cloud;
	}
	return true;
}

#ifdef _MSC_VER

//
//   �ڵ���������ʾ�������ݡ�
//
void CScanPointCloud::Dump()
{
	for (int i = 0; i < m_nCount; i++)
		m_pPoints[i].Dump();

	TRACE(_T("\n"));
}

//
//   ���Ƶ��ơ�
//
void CScanPointCloud::Draw(CScreenReference& ScrnRef, CDC* pDC, COLORREF color, int nPointSize)
{
	for (int i = 0; i < m_nCount; i++)
		m_pPoints[i].Draw(ScrnRef, pDC,  color, nPointSize);
}
#endif
