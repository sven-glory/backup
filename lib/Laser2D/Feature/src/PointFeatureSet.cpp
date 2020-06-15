#include "stdafx.h"
#include <algorithm>
#include "PointFeatureSet.h"
#include "FlatReflectorFeature.h"
#include "ShortLineFeature.h"
#include "Scan.h"
#include "assert.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif

#define NOISE_RATIO        0.02f

//
//   �Ը����ļ�������ʩ���������õ�����������ļ������ݡ�
//
void AddNoise(float fDistIn, float& fDistOut, float fNoiseRatio)
{
	float fNoise = fDistIn * fNoiseRatio;
	int nAmp = (int)(fNoise);
	int nNoise = (nAmp == 0) ? 0 : rand() % nAmp;
	nNoise -= nAmp / 2;
	fDistOut = fDistIn + nNoise;
}

///////////////////////////////////////////////////////////////////////////////

CPointFeatureSet::CPointFeatureSet()
{
	reserve(50);
	clear();
	m_rect.Clear();
	m_pDistCache = NULL;
}

CPointFeatureSet::~CPointFeatureSet()
{
	Clear();
}

//
//   �����������캯����
//
CPointFeatureSet::CPointFeatureSet(const CPointFeatureSet& another)
{
	m_pDistCache = NULL;
	Clear();

	for (int i = 0; i < (int)another.size(); i++)
	{
		// ���ݵ����������ͷ����¿ռ�
		CPointFeature* pNewFeature = another[i]->Duplicate();
		if (pNewFeature == NULL)
			assert(false);

		push_back(pNewFeature);
	}

	CreateDistanceCache();
}

//
//   ʵ�ָ�ֵ���㡣
//
CPointFeatureSet& CPointFeatureSet::operator = (const CPointFeatureSet& another)
{
	Clear();

	for (int i = 0; i < (int)another.size(); i++)
	{
		// ���ݵ����������ͷ����¿ռ�
		CPointFeature* pNewFeature = another[i]->Duplicate();
		if (pNewFeature == NULL)
			assert(false);

		push_back(pNewFeature);
	}

	CreateDistanceCache();
	return *this;
}

//
//   ������ϡ�
//
void CPointFeatureSet::Clear()
{
	ClearDistanceCache();

	for (int i = 0; i < (int)size(); i++)
		delete at(i);

	clear();
	m_rect.Clear();
}

//
//   �������ṩ���������ͷ���ռ䡣
//
CPointFeature* CPointFeatureSet::NewPointFeature(int nSubType)
{
	// �������ͷ���ռ�
	switch (nSubType)
	{
	case GENERIC_POINT_FEATURE:              // һ�������
		return new CPointFeature;

	case FLAT_REFLECTOR_FEATURE:             // ���������
		return new CFlatReflectorFeature;

	case SHORT_LINE_FEATURE:                 // ��ֱ������
		return new CShortLineFeature;

	default:
		return NULL;
	}
}

//
//   �򼯺������һ���µ㡣
//
CPointFeatureSet& CPointFeatureSet::operator += (CPointFeature* pNewFeature)
{
	ClearDistanceCache();

	push_back(pNewFeature);
	m_rect += *pNewFeature;

	return *this;
}

//
//   �򼯺������һ���µ㡣
//
CPointFeatureSet& CPointFeatureSet::operator += (const CPointFeature& NewFeature)
{
	*this += NewFeature.Duplicate();
	return *this;
}

//
//   ����һ�����ϲ��뱾�����С�
//
CPointFeatureSet& CPointFeatureSet::operator += (const CPointFeatureSet& another)
{
	for (int i = 0; i < (int)another.size(); i++)
	{
		CPointFeature* pFeature = another[i]->Duplicate();
		*this += pFeature;
	}

	return *this;
}

//
//   ɾ��ָ����ŵĵ㡣
//
bool CPointFeatureSet::DeleteAt(int nIdx)
{
	if (nIdx < 0 && nIdx >= (int)size())
		return false;

	ClearDistanceCache();

	delete at(nIdx);
	erase(begin() + nIdx);

	UpdateCoveringRect();

	return true;
}

//
//   �������е�ļ������������ǵĵϿ������ꡣ
//
void CPointFeatureSet::UpdateCartisian()
{
	for (int i = 0; i < (int)size(); i++)
		at(i)->UpdateCartisian();
}

//
//   ���±߽�ֵ��
//
void CPointFeatureSet::UpdateCoveringRect()
{
	m_rect.Clear();

	for (int i = 0; i < (int)size(); i++)
		m_rect += *at(i);
}

//
//   ����������������������ϵ�任���ֲ�����ϵ��
//
void CPointFeatureSet::Transform(CPosture& pstLocal)
{
	// ��������任
	CTransform trans(pstLocal);

	// ���е�����������任
	for (int i = 0; i < (int)size(); i++)
	{
		CPnt pt = trans.GetLocalPoint(at(i)->GetPntObject());
		at(i)->GetPntObject() = pt;
	}

	UpdateCoveringRect();
}

//
//   �������������ɾֲ�����ϵ�任����������ϵ��
//
void CPointFeatureSet::InvTransform(CPosture& pstOrigin)
{
	// ��������任
	CTransform trans(pstOrigin);

	// ����ֱ�ߵ�����任
	for (int i = 0; i < (int)size(); i++)
	{
		CPnt pt = trans.GetWorldPoint(at(i)->GetPntObject());
		at(i)->GetPntObject() = pt;
	}

	UpdateCoveringRect();
}

//
//   �Ը����ĵ�Ϊ���ģ���ָ���ķ�ΧΪ�뾶����ȡ��һ�������Ӽ���
//
bool CPointFeatureSet::GetSubset(const CPnt& ptCenter, float fRange, CPointFeatureSet& Subset)
{
	Subset.Clear();

	for (int i = 0; i < GetCount(); i++)
	{
		CPointFeature* pFeature = at(i);
		if (pFeature->DistanceTo(ptCenter) < fRange)
			Subset += pFeature;
	}

	return true;
}

//
//   ����ָ�����λ�á�
//
bool CPointFeatureSet::ModidfyPointPos(int nId, CPnt& pt)
{
	for (int i = 0; i < GetCount(); i++)
	{
		CPointFeature* pFeature = at(i);
		if (pFeature->m_nFeatureId == nId)
		{
			pFeature->GetPntObject() = pt;
			return true;
		}
	}

	ClearDistanceCache();
	return false;
}

//
//   ͨ����pstScanner���Ը����ĵ㼯��������ɨ��������������ɾֲ�ɨ��㼯��
//   ����ֵ��
//     �����ɵĵ��������
//
int CPointFeatureSet::CreateFromSimuScan(const CPointFeatureSet& Model, const CPosture& pstScanner,
	float fMaxScanDist, bool bAddNoise)
{
	// ������㼯��ԭ��������
	Clear();

	// ɨ��ԭ��λ��
	float fMaxScanDist2 = fMaxScanDist * fMaxScanDist;

	for (int i = 0; i < (int)Model.size(); i++)
	{
		// ȡ��ָ���������ָ��
		CPointFeature* pFeature = Model[i];

		// ���쵱ǰɨ����
		CLine ln(pstScanner, *pFeature);

		// �жϴ�ɨ�����Ƿ���Ա�����
		if (!pFeature->CheckInRay(ln))
			continue;

		// ����ɨ���߳��ȵ�ƽ��ֵ
		float fLen2 = ln.Length2();

		// ���߶γ��ȳ��ޣ�����Ը���
		if (fLen2 > fMaxScanDist2)
			continue;

		float r = sqrt(fLen2);

		// ����Ƿ��棬�ڴ˶�ɨ��õ��ļ���ʩ���������
		if (bAddNoise)
			AddNoise(r, r, NOISE_RATIO);

		pFeature->r = r;                                                    // ����
		pFeature->a = (ln.SlantAngle() - pstScanner.fThita).NormAngle();    // ����

		int nType = Model[i]->GetSubType();

		// Ϊ�µĵ���������ռ�
		CPointFeature* pNewFeature = pFeature->Duplicate();

		// ��Ӵ˵�����
		*this += pNewFeature;
	}

	// �����������㰴���ǽ�������
	SortByPolarAngle();

	return GetCount();
}

//
//   �Ӹ�����ɨ��������ɷ����������
//
bool CPointFeatureSet::CreateFromScan(const CScan& Scan, CReflectorCreationParam* pParam)
{
	CReflectorCreationParam Param;
	if (pParam != NULL)
	{
		Param = *pParam;
	}

	// �������ɷ���������
	Clear();

	// �������δ��ʼ
	int nReflectorStart = -1;
	float r1 = 0;
	bool bStartAtRef = false;           // ����Ƿ��һ�߼�Ϊ�����
	int  nStartRefEndPos = -1;          // ��һ��Ϊ����������£������Ľ���λ��

	int nCount = Scan.m_nCount;
	for (int i = 0; i < nCount; i++)
	{
		const CScanPoint& sp = Scan.m_pPoints[i];

		// ����������δ��ʼ
		if (nReflectorStart < 0)
		{
			// ����������ޣ�����������һ��
			if (sp.r < 0)
				continue;
			else if (sp.m_nIntensity > Param.nMinReflectorIntensity)
			{
				nReflectorStart = i;     // ��¼����忪ʼ�������
				r1 = sp.r;               // ��¼�˴��ļ���

												 // �ж��Ƿ��һ�߼�Ϊǿ������
				if (i == 0)
					bStartAtRef = true;
			}
		}

		// ���������ѿ�ʼ�����������㷴����������
		else if (sp.r < 0 || sp.m_nIntensity < Param.nMinReflectorIntensity)
		{
			// ������ڴ˽�������Ҫ��֤����峤��û�г���
			if (sp.DistanceTo(Scan.m_pPoints[nReflectorStart]) < Param.fMaxReflectorSize)
			{
				// ����÷������ռ�ĵ������
				int nCountPoints = i - nReflectorStart;

				// ���������ʼ�ĵ�һ������壬��¼��������λ��
				if (bStartAtRef && nStartRefEndPos < 0)
					nStartRefEndPos = i;

				// �ڴ˼��㷴�����������
				int nCenterIdx = nReflectorStart + nCountPoints / 2;

				// ��֤�������뼤��ͷ�ľ����ڹ涨�ķ�Χ��

				if (Scan.m_pPoints[nCenterIdx].r / 1000.0f > 0.3f /*&&
					Scan.m_pPoints[nCenterIdx].r / 1000.0f > 30*/)
				{
					CFlatReflectorFeature* p = new CFlatReflectorFeature;
					p->GetPntObject() = Scan.m_pPoints[nCenterIdx];
					p->m_nIntensity = Scan.m_pPoints[nCenterIdx].m_nIntensity;
					p->m_nPointCount = nCountPoints;
					push_back(p);
				}
			}

			// ����������ʼ���
			nReflectorStart = -1;
			r1 = 0;
		}

		// ����������
		else
		{
			// ͬһ����������ڵ㼫���仯��������Ϊ���ǿ�����������ġ��ٷ���塱
			if (fabs(sp.r - r1) / 1000 > Param.fMaxPolarRadiusVariance)
			{
				nReflectorStart = -1;
				r1 = 0;
				i += 3;     // �������ȶ���
			}

			// �����һ�߼�Ϊ�������ߣ����ұ������һ��ҲΪ�������ߣ�������Ҫ������һ��������λ��
			else if (bStartAtRef && (i == nCount - 1))
			{
				int nCountPoints = nStartRefEndPos + nCount - nReflectorStart;
				int nPos = (nReflectorStart + nCountPoints / 2) % nCount;
				at(0)->GetPntObject() = Scan.m_pPoints[nPos];
			}
			else
				r1 = sp.r;
		}
	}

	return true;
}

//
//   ����ڲ��ĸ���֮��ľ���ֵ��
//
void CPointFeatureSet::ClearDistanceCache()
{
	if (m_pDistCache != NULL)
	{
		for (int i = 0; i < GetCount(); i++)
		{
			if (m_pDistCache[i] != NULL)
			{
				delete[]m_pDistCache[i];
				m_pDistCache[i] = NULL;
			}
		}

		delete[]m_pDistCache;
		m_pDistCache = NULL;
	}
}

//
//   Ϊ����֮��ľ������洢�ռ䡣
//
bool CPointFeatureSet::CreateDistanceCache()
{
	int nSize = (int)size();

	// ����������֮ǰҪ��ԭ�����������
	if (m_pDistCache != NULL)
		ClearDistanceCache();

	// ���·���ռ�
	m_pDistCache = new float*[nSize];
	if (m_pDistCache == NULL)
		return false;

	for (int i = 0; i < nSize; i++)
	{
		m_pDistCache[i] = new float[nSize];
		if (m_pDistCache[i] == NULL)
			return false;
	}

	// ��������������֮��ľ���
	for (int m = 0; m < nSize - 1; m++)
		for (int n = m + 1; n < nSize; n++)
			m_pDistCache[n][m] = m_pDistCache[m][n] = at(m)->DistanceTo(*at(n));

	return true;
}

//
//   ȡ��i, j����֮��ľ��롣
//
float CPointFeatureSet::PointDistance(int i, int j)
{
	if (m_pDistCache != NULL)
		return m_pDistCache[i][j];
	else
		return at(i)->DistanceTo(*at(j));
}


typedef CPointFeature* CPtrPointFeature;

//
//   ���ڼ�������ıȽϺ�����
//
bool CompFunc(const CPtrPointFeature& f1, const CPtrPointFeature &f2)
{
	return (f1->a < f2->a);
}

//
//   �����еĵ����������Ǵ�С�����˳���������
//
void CPointFeatureSet::SortByPolarAngle()
{
	sort(begin(), end(), CompFunc);
}

//
//   ���ص�����кϲ���
//
void CPointFeatureSet::MergeOverlappedPoints(float fDistGate)
{
	int nCount = (int)size();
	bool *pDelete = new bool[nCount];
	for (int i = 0; i < nCount; i++)
		pDelete[i] = false;

	// ���αȽ�������������������֮��ľ����Ƿ��fDistGate��
	for (int i = 0; i < nCount - 1; i++)
	{
		if (pDelete[i])
			continue;

		CPointFeature* pFeature1 = (CPointFeature*)at(i);

		// ȡ�ڶ�����
		for (int j = i + 1; j < nCount; j++)
		{
			if (pDelete[j])
				continue;

			CPointFeature* pFeature2 = (CPointFeature*)at(j);

			// �������������֮��ľ���ǳ������򽫵�һ�����Ϊ����֮����м�λ�ã���ɾ���ڶ���
			if (pFeature1->DistanceTo(*pFeature2) < fDistGate)
			{
				float x = (pFeature1->x + pFeature2->x) / 2;
				float y = (pFeature1->y + pFeature2->y) / 2;
				pFeature1->x = x;
				pFeature1->y = y;
				pDelete[j] = true;
			}
		}
	}

	for (int i = nCount - 1; i >= 0; i--)
	{
		if (pDelete[i])
		{
			delete at(i);
			erase(begin() + i);
		}
	}

	delete[]pDelete;

	ClearDistanceCache();

	// Ϊ���еĵ����±��
	for (int i = 0; i < (int)size(); i++)
		at(i)->id = i;
}

//
//   ���ļ���װ�������������ݡ�
//   ����ֵ��
//     < 0 : ��ȡʧ��
//     >= 0: ��ȡ����������
//
int CPointFeatureSet::LoadText(FILE* fp)
{
	Clear();

	int nCount, nSubType;

	// ��ȡ��������
	if (fscanf(fp, "%d\n", &nCount) != 1 || nCount < 0)
		return -1;

	for (int i = 0; i < nCount; i++)
	{
		// ��ȡ������������
		fscanf(fp, "%d", &nSubType);

		CPointFeature* pFeature = NewPointFeature(nSubType);
		if (pFeature == NULL)
			return -1;                // �ռ����ʧ��

		// ���������Զ��������
		if (pFeature->LoadText(fp) < 0)
			return -1;

		// ���ӵ�����
		pFeature->GetPntObject().id = i;

		// ������ָ����뵽����
		push_back(pFeature);

		// �����߽�ֵ
		m_rect += *pFeature;
	}

	CreateDistanceCache();
	return nCount;
}

//
//   �������������ݱ��浽�ļ��С�
//
int CPointFeatureSet::SaveText(FILE* fp)
{
	// д����������
	int nCount = (int)size();
	fprintf(fp, "%d\n", nCount);

	for (int i = 0; i < nCount; i++)
	{
		if (at(i)->SaveText(fp) < 0)
			return -1;
	}

	return nCount;
}

//
//   �Ӷ������ļ���װ�������������ݡ�
//   ����ֵ��
//     < 0 : ��ȡʧ��
//     >= 0: ��ȡ����������
//
int CPointFeatureSet::LoadBinary(FILE* fp)
{
	Clear();

	int nCount;

	// ��ȡ��������
	if (fread(&nCount, sizeof(int), 1, fp) != 1 || nCount < 0)
		return -1;

	for (int i = 0; i < nCount; i++)
	{
		// ��ȡ������������
		int nSubType;
		if (fread(&nSubType, sizeof(int), 1, fp) != 1)
			return -1;

		CPointFeature* pFeature = NewPointFeature(nSubType);
		if (pFeature == NULL)
			return -1;

		// ���������Զ��������
		if (pFeature->LoadBinary(fp) < 0)
			return -1;

		// ���ӵ�����
		pFeature->GetPntObject().id = i;

		// ������ָ����뵽����
		push_back(pFeature);

		// �����߽�ֵ
		m_rect += *pFeature;
	}

	CreateDistanceCache();
	return nCount;
}

//
//   �������������ݱ��浽�ļ��С�
//
int CPointFeatureSet::SaveBinary(FILE* fp)
{
	// ������������
	int nCount = (int)size();
	if (fwrite(&nCount, sizeof(int), 1, fp) != 1)
		return -1;

	for (int i = 0; i < nCount; i++)
	{
		if (at(i)->SaveBinary(fp) < 0)
			return -1;
	}

	return nCount;
}

//
//   �Ӷ������ļ���װ���û��༭���ݡ�
//
bool CPointFeatureSet::LoadUserData(FILE* fp)
{
	int n;
	for (int i = 0; i < (int)size(); i++)
	{
		if (fread(&n, sizeof(int), 1, fp) != 1)
			return false;

		// ����Ϊ0ʱ����ʹ��
		at(i)->Enable(n == 0);
	}

	return true;
}

//
//   ���û��༭���ݱ��浽�������ļ��С�
//
bool CPointFeatureSet::SaveUserData(FILE* fp)
{
	int n;
	for (int i = 0; i < (int)size(); i++)
	{
		// ����ʹ��ʱ��0
		n = !(at(i)->IsEnabled());
		if (fwrite(&n, sizeof(int), 1, fp) != 1)
			return false;
	}

	return true;
}

//
//   ������һ��PointFeatureSet�������û�ʹ�����á�
//
bool CPointFeatureSet::CopyUserData(const CPointFeatureSet& another)
{
	if (another.size() != size())
		return false;

	for (int i = 0; i < (int)size(); i++)
	{
		bool bEnabled = another.at(i)->IsEnabled();
		at(i)->Enable(bEnabled);
	}

	return true;
}

//
//   �����������任��
//
void CPointFeatureSet::Transform(const CFrame& frame)
{
	for (int i = 0; i < (int)size(); i++)
		at(i)->Transform(frame);

	UpdateCoveringRect();
}

//
//   ����������任��
//
void CPointFeatureSet::InvTransform(const CFrame& frame)
{
	for (int i = 0; i < (int)size(); i++)
		at(i)->InvTransform(frame);

	UpdateCoveringRect();
}

//   �ж�ָ����������Ƿ����������������е�ĳ����������
//
int CPointFeatureSet::PointHit(const CPnt& pt, float fDistGate)
{
	for (int i = 0; i < (int)size(); i++)
		if (at(i)->DistanceTo(pt) < fDistGate)
			return i;

	return -1;
}

#ifdef _MFC_VER

void CPointFeatureSet::Dump()
{
	for (int i = 0; i < GetCount(); i++)
	{
		CPointFeature* pFeature = (CPointFeature*)at(i);
		CPnt& pt = pFeature->GetPntObject();
		pt.Dump();
	}
}

//
//   ����Ļ�ϻ��ƴ˵��������ϡ�
//
void CPointFeatureSet::Plot(CScreenReference& ScrnRef, CDC* pDC, COLORREF cr, COLORREF crSelected, 
	int nLineWidth, bool bShowId)
{
	for (int i = 0; i < GetCount(); i++)
	{
		CPointFeature* pFeature = (CPointFeature*)at(i);
		pFeature->Plot(ScrnRef, pDC, cr, crSelected, 5, nLineWidth);

		if (bShowId)
			pFeature->PlotId(ScrnRef, pDC, RGB(0, 0, 255));
	}
}

#elif defined QT_VERSION

void CPointFeatureSet::Plot(CScreenReference& ScrnRef, QPainter* pPainter, QColor cr, QColor crSelected, 
	int nLineWidth, bool bShowId)
{
	for (int i = 0; i < GetCount(); i++)
	{
		CPointFeature* pFeature = (CPointFeature*)at(i);
		pFeature->Plot(ScrnRef, pPainter, cr, crSelected, 5, nLineWidth);

		if (bShowId)
			pFeature->PlotId(ScrnRef, pPainter, Qt::blue));
	}
}

#endif