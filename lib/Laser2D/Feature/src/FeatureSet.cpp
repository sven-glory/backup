#include "stdafx.h"
#include "FeatureSet.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif

///////////////////////////////////////////////////////////////////////////////

CPointFeatureSet& CFeatureSet::GetPointFeatureSet()
{
	CPointFeatureSet* p = dynamic_cast<CFeatureSet*>(this);
	return *p;
}

//
//   ������ݡ�
//
void CFeatureSet::Clear()
{
	m_pstObserver.SetPosture(0, 0, 0);
	CPointFeatureSet::Clear();
}

//
//   ���ļ���װ������ͼ��
//
int CFeatureSet::LoadText(FILE* fp)
{
	Clear();

	// װ��۲�����̬
	float x, y, thita;
	if (fscanf(fp, "%f\t%f\t%f\n", &x, &y, &thita) != 3)
		return 0;

	// װ������������
	return CPointFeatureSet::LoadText(fp);
}

//
//    ������ͼ�����ļ��С�
//
int CFeatureSet::SaveText(FILE* fp)
{
	// ����۲�����̬
	fprintf(fp, "%f\t%f\t%f\n", m_pstObserver.x, m_pstObserver.y, m_pstObserver.fThita);

	return CPointFeatureSet::SaveText(fp);
}

//
//   �Ӷ������ļ���װ���������ϡ�
//
int CFeatureSet::LoadBinary(FILE* fp)
{
	Clear();

	// װ��۲�����̬
	float f[3];
	if (fread(f, sizeof(float), 3, fp) != 3)
		return 0;

	m_pstObserver.x = f[0];
	m_pstObserver.y = f[1];
	m_pstObserver.fThita = f[2];

	return CPointFeatureSet::LoadBinary(fp);
}

//
//   ���������ϴ���������ļ��С�
//
int CFeatureSet::SaveBinary(FILE* fp)
{
	// ����۲�����̬
	float f[3] = { m_pstObserver.x, m_pstObserver.y, m_pstObserver.fThita };
	if (fwrite(f, sizeof(float), 3, fp) != 3)
		return 0;

	return CPointFeatureSet::SaveBinary(fp);
}

//
//   ��ԭʼɨ����������������ϡ�
//
bool CFeatureSet::CreateFromRawScan(const CScan& Scan, CFeatureCreationParam& Param)
{
	// ���ɷ��������
	if (!CPointFeatureSet::CreateFromScan(Scan, &Param.m_RefParam))
			return false;

	return true;
}

//
//   ͨ���Ը�������ģ�͵Ĳ��������������������ϡ�
//
bool CFeatureSet::CreateFromFeatureMap(const CFeatureSet& FeatureMap, const CPosture& pstScanner)
{
	return true;
}

//
//   �Ը����ĵ�Ϊ���ģ���ָ���ķ�ΧΪ�뾶����ȡ��һ�������Ӽ���
//
bool CFeatureSet::GetSubset(CPnt& ptCenter, float fRange, CFeatureSet& Subset)
{
	// �Ƚ�ȡ���������Ӽ�
	CPointFeatureSet::GetSubset(ptCenter, fRange, Subset);

	// ��ȡ�߶��������Ӽ�
	Subset.SetObserverPosture(m_pstObserver);

	return true;
}

//
//   �����������任��
//
void CFeatureSet::Transform(const CFrame& frame)
{
	m_pstObserver.Transform(frame);
	CPointFeatureSet::Transform(frame);
}

//
//   ����������任��
//
void CFeatureSet::InvTransform(const CFrame& frame)
{
	m_pstObserver.InvTransform(frame);
	CPointFeatureSet::InvTransform(frame);
}

#ifdef _MFC_VER
//
//   ����ȫ��ͼ��
//
void  CFeatureSet::Plot(CScreenReference& ScrnRef, CDC* pDc, COLORREF clrFeature, bool bShowId)
{
	CPointFeatureSet::Plot(ScrnRef, pDc, clrFeature, clrFeature, bShowId);
}
#endif