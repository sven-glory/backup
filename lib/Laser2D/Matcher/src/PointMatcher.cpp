#include <stdafx.h>
#include "PointMatcher.h"
#include "Misc.h"
#include "DebugTrace.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif

///////////////////////////////////////////////////////////////////////////////

//
//   ���òο�����ͼ��
//
void CPointMatcher::SetRefFeatures(CPointFeatureSet* pPointFeatures)
{
	m_pRefPointFeatures = pPointFeatures;
	if (pPointFeatures == NULL)
		return;

	// �ڴ˸����вο��㸳��ID�ţ��Ա�������ƥ��ʱ�γ�ƥ���
	for (int i = 0; i < m_pRefPointFeatures->size(); i++)
		m_pRefPointFeatures->at(i)->id = i;
}

//
//   ���õ�ǰ����ͼ��
//
void CPointMatcher::SetCurFeatures(CPosture& pstOdometry, CPointFeatureSet* pPointFeatures)
{
	m_pstOdometry = pstOdometry;
	m_pCurPointFeatures = pPointFeatures;

	if (pPointFeatures == NULL)
		return;

	// �ڴ˸����е�ǰ�㸳��ID�ţ��Ա�������ƥ��ʱ�γ�ƥ���
	for (int i = 0; i < m_pCurPointFeatures->size(); i++)
		m_pCurPointFeatures->at(i)->id = i;
}

//
//   ����任����������⴦����̡�������Ϊ(x, y, sin(thita), cos(thita))������任���������
//   ��С���˷����������ʱ�����sin(thita)��cos(thita)��ֵ����1�����������ȷ���ڴ�����£�ͨ��
//   ǿ��sin(thita)��cos(thita)Ϊ1����������õ�һ��̣�ͨ����С���˷�����������������Ž⣬
//   �õ�����任����
//
bool CPointMatcher::SpecialProcess(float fSin, float fCos, float& x, float& y)
{
	float a[2][2], b[2];
	float f[2];

	int nPointPairs = m_PointMatchList.GetCount();
	CLinearEquations* pLsm = new CLinearEquations(nPointPairs * 2 + 1, 2);

	pLsm->Start();

	for (int i = 0; i < nPointPairs; i++)
	{
		// ���ݵ���������о�������
		m_PointMatchList.GetPair(i).MakeLeastSquareData1(a, b, fSin, fCos);

		// �����������ݼ�����С�����������
		if (!pLsm->AddRow(a[0], b[0]))
			return false;

		if (!pLsm->AddRow(a[1], b[1]))
			return false;
	}

	bool bResult = pLsm->LeastSquareSolve(f, 2);

	x = f[0];
	y = f[1];

	delete pLsm;

	return bResult;
}

//
//   ���п���ƥ�䣬�ҵ��ӡ��ֲ�������-->��ȫ��������������任��
//   ����ֵ��
//      > 0 : ƥ��ɹ�
//      < 0 : �������(-2:��������; -3:���̾����쳣)
//
int CPointMatcher::QuickMatch(CTransform& trans)
{
	// �ֱ�Ե��ֱ�߶ν�����׼
	QuickRegisterPoints();

	// �������ֵ�ƥ���Բ�����3��ʱ������Ϊ��׼�ɹ�
	if (m_PointMatchList.GetCount() < 2)
		return FM_FEATURES_NOT_ENOUGH;

	// ���е㼯ƥ��
	if (!m_PointMatchList.FindTransform())
		return FM_MATRIX_ERROR;

	trans = m_PointMatchList.GetTransform();

	return FM_OK;
}

//
//   ���б��ؾֲ�ƥ�䡣
//
int CPointMatcher::LocalMatch(CTransform& trans)
{
	// �����׼��
	m_PointMatchList.Clear();

	for (int i = 0; i < LINE_COMPARE_COUNT; i++)
	{
		if (CoarseRegister(m_Param.m_fLineEqualLimit[i]))
			break;
	}

	// ���û����ƥ�䣬��λʧ��
	if (m_PointMatchListSet.GetCount() == 0)
		return FM_NOT_MATCHED;                // �ҵ�����ƥ�伯

	// ���н������£���ѡ�����Ž�
	for (int i = 0; i < m_PointMatchListSet.GetCount(); i++)
		m_PointMatchListSet.GetList(i).FindTransform();

	// ��ƥ�������ѡ�����ŵ���һ��
	int index = m_PointMatchListSet.FindBestMatch();
	if (index >= 0)
	{
		m_PointMatchList = m_PointMatchListSet.GetList(index);                 // ȷ��������ƥ���
		trans = m_PointMatchList.m_Trans;                              // ��������任
	}
	else
		return FM_FEATURES_NOT_ENOUGH;      // ƥ�����С��5

	return FM_OK;
}

//
//   ����ƥ�䣬�ҵ��ӡ��ֲ�������-->��ȫ��������������任��
//   ����ֵ��
//      > 0 : ƥ��ɹ�
//      < 0 : �������(-2:��������; -3:���̾����쳣)
//
int CPointMatcher::Match(CTransform& trans)
{
	// �ȳ��Խ��п���ƥ��
	int nResult = QuickMatch(trans);
	if (nResult >= 0)
		return nResult;
	else
		return LocalMatch(trans);
}

//
//   �����еĵ��������п�����׼�������ɰ�������С����˳���ƥ���
//
bool CPointMatcher::QuickRegisterPoints()
{
	// �����׼��
	m_PointMatchList.Clear();

	// ��������׼
	for (int i = 0; i < m_pRefPointFeatures->GetCount(); i++)
	{
		// �Ӳο��㼯��ȡ����һ��
		CPointFeature* pntWorld = m_pRefPointFeatures->at(i);
		if ((m_nOption == FM_OPTION_LOCALIZATION && !pntWorld->IsEnabled()) ||
			 (m_nOption == FM_OPTION_EVALUATE_FEATURES && pntWorld->IsBad()))
			continue;

		pntWorld->UpdatePolar(m_pstOdometry);

		for (int j = 0; j < m_pCurPointFeatures->GetCount(); j++)
		{
			// �Ӿֲ��㼯��ȡ����һ��
			CPointFeature* pntLocal = m_pCurPointFeatures->at(j);
			if ((m_nOption == FM_OPTION_LOCALIZATION && !pntLocal->IsEnabled()) ||
				 (m_nOption == FM_OPTION_EVALUATE_FEATURES && pntLocal->IsBad()))
				continue;

			// ����������ͼ��ǲ��
			float fRangeDiff = fabs(pntWorld->r - pntLocal->r);
			float fAngleDiff = AngleDiff(pntWorld->a, pntLocal->a);
			float fDist = pntLocal->DistanceTo(*pntWorld);

			// �������ڳ����˶�����Ĺ۲�Ǳ仯(����Ҫ��m_fAngleEqualLimitһ����Ϊ�ǶȲ������)
			float fAng = 0;// (float)fabs(0.3f / pntWorld->r);

			// ���������ڹ涨�ķ�Χ�ڣ��Ҽ�������ǲ�����涨������ʱ������Ϊ�ҵ�һ����׼��
			if (pntLocal->r < m_Param.m_fMaxRange &&
				pntLocal->r > m_Param.m_fMinRange &&
				fAngleDiff < (m_Param.m_fAngleEqualLimit + fAng) &&
				fRangeDiff < m_Param.m_fRangeEqualLimit &&
				fDist < m_Param.m_fRangeEqualLimit)
			{
				// ��ƥ�������Ӵ˵��(��������С����˳��)
				CPointMatchPair pair(j, i, *pntLocal, *pntWorld);
				pair.m_ptLocal.id = j;
				pair.m_ptWorld.id = i;
				m_PointMatchList.AddInRadiusOrder(pair);
			}
		}
	}

	// �ڴ�Ӧ��NearestN�������Բ���ƥ������ķ����������������
	m_PointMatchList.LimitMatchPairNum(m_Param.m_nAppliedClosestPoints);
//	m_PointMatchList.Filter();
	return true;
}

//
//   ���ݵ�ǰƥ����е����ݣ�����ֱ�Ӷ��������ƽ�����׼��
//
short CPointMatcher::DirectRegister(CTransform trans, CPointMatchList* tab)
{
	for (int i = 0; i < m_pCurPointFeatures->size(); i++)
	{
		// �ӱ���ɨ��㼯��ȡһ��
		CPointFeature& pnt1 = *m_pCurPointFeatures->at(i);

		if ((m_nOption == FM_OPTION_LOCALIZATION && !pnt1.IsEnabled()) ||
			 (m_nOption == FM_OPTION_EVALUATE_FEATURES && pnt1.IsBad()))
			continue;

		// ����˱��ص�����ƥ����У��������˵�
		if (tab->SearchByLocalPoint(pnt1) >= 0)
			continue;

		// ���õ�ֱ�ӱ任���ο��㼯����ϵ��
		CPnt pnt2 = trans.GetWorldPoint(pnt1);

		// ���˵���ο��㼯�е����е���бȶԣ����ǲ����������������غϵĵ�
		for (int j = 0; j < m_pRefPointFeatures->size(); j++)
		{
			CPointFeature& pnt3 = *m_pRefPointFeatures->at(j);

			if ((m_nOption == FM_OPTION_LOCALIZATION && !pnt3.IsEnabled()) ||
				 (m_nOption == FM_OPTION_EVALUATE_FEATURES && pnt3.IsBad()))
				continue;

			// ��������������ƥ����У��������˵�
			if (tab->SearchByWorldPoint(pnt3) >= 0)
				continue;

#ifdef USE_LENGTH_SQUARE_COMPARE
			float cost = pnt2.Distance2To(pnt3);

			if (cost < (m_Param.m_fSamePointMaxDist*m_Param.m_fSamePointMaxDist))
			{
				CPointMatchPair pair(pnt1, pnt3);
				tab->Add(pair);
				break;
			}
#else
			float cost = pnt2.DistanceTo(pnt3);

			// ������ֽ������غϵĵ㣬��Ϊ����һ��ƥ����
			if (cost < m_Param.m_fSamePointMaxDist)
			{
				CPointMatchPair pair(i, j, pnt1, pnt3);
				tab->Add(pair);
				break;
			}
#endif
		}
	}

	// ����ƥ���Ե������Ƿ��㹻
	if (tab->GetCount() >= m_Param.m_nLeastMatchCount)
		return 1;

	return 0;
}

//
//   ��ȫ�ַ�Χ�ڽ��д��Ե���׼���������������m_PointMatchListSet�С�
//
bool CPointMatcher::CoarseRegister(float equal_limit)
{
	float fEqualLimit, fLineValidLimit;

	m_PointMatchListSet.Clear();

	int nLocalCount = m_pCurPointFeatures->size();
	int nRefCount = m_pRefPointFeatures->GetCount();

	fEqualLimit = equal_limit;
	fLineValidLimit = m_Param.m_fMinLineLen;

	for (int i = 0; i < nLocalCount - 1; i++)
	{
		// ȡm_LocalLayer�㼯�еĵ�һ��
		CPointFeature& pnt11 = *m_pCurPointFeatures->at(i);
		if ((m_nOption == FM_OPTION_LOCALIZATION && !pnt11.IsEnabled()) ||
			 (m_nOption == FM_OPTION_EVALUATE_FEATURES && pnt11.IsBad()))
			continue;

		for (int j = i + 1; j < nLocalCount; j++)
		{
			// ȡm_LocalLayer�㼯�еĵڶ���
			CPointFeature& pnt12 = *m_pCurPointFeatures->at(j);
			if ((m_nOption == FM_OPTION_LOCALIZATION && !pnt12.IsEnabled()) ||
				(m_nOption == FM_OPTION_EVALUATE_FEATURES && pnt12.IsBad()))
				continue;

			for (int m = 0; m < nRefCount - 1; m++)
			{
				// ȡworld�㼯�еĵ�һ��
				CPointFeature& pnt21 = *m_pRefPointFeatures->at(m);
				if ((m_nOption == FM_OPTION_LOCALIZATION && !pnt21.IsEnabled()) ||
					(m_nOption == FM_OPTION_EVALUATE_FEATURES && pnt21.IsBad()))
					continue;

				for (int n = m + 1; n < nRefCount; n++)
				{
					// ȡworld�㼯�еĵڶ���
					CPointFeature& pnt22 = *m_pRefPointFeatures->at(n);
					if ((m_nOption == FM_OPTION_LOCALIZATION && !pnt22.IsEnabled()) ||
						(m_nOption == FM_OPTION_EVALUATE_FEATURES && pnt22.IsBad()))
						continue;

					// dist1Ϊpnt11��pnt12֮��ľ���
					float dist1 = pnt11.DistanceTo(pnt12);

					// dist2Ϊpnt21��pnt22֮��ľ���
					float dist2 = pnt21.DistanceTo(pnt22);

					// ���������߶εĳ��Ȳ�
					float dist_err = (float)fabs(dist1 - dist2);

					// �������߶γ��Ȳ�̫����������߶�ƥ��ʧ��
					if (dist_err > fEqualLimit)
						continue;

					// ���������߶�֮����һ�߶ι��̣�Ҳ������ƥ������
					if (dist1 < fLineValidLimit || dist2 < fLineValidLimit)
						continue;

					CPointMatchPair pair1(i, m, pnt11, pnt21);
					CPointMatchPair pair2(j, n, pnt12, pnt22);
					CPointMatchPair pair3(i, n, pnt11, pnt22);
					CPointMatchPair pair4(j, m, pnt12, pnt21);

					// �������ڵ�ƥ����а�����������
					if (m_PointMatchListSet.Search(pair1, pair2) >= 0 || m_PointMatchListSet.Search(pair3, pair4) >= 0)
						continue;

					CLine line1(pnt11, pnt12);
					CLine line2(pnt21, pnt22);

					CTransform trans;
					trans.Create(line2, line1);

					CPointMatchList tab;
					tab.Add(pair1);
					tab.Add(pair2);

					if (DirectRegister(trans, &tab) > 0)
					{
						tab.SetTransform(trans);
						m_PointMatchListSet.Add(tab);
						continue;
					}

					CLine line3(pnt22, pnt21);
					trans.Create(line3, line1);

					tab.Clear();
					tab.Add(pair3);
					tab.Add(pair4);

					if (DirectRegister(trans, &tab) > 0)
					{
						tab.SetTransform(trans);
						m_PointMatchListSet.Add(tab);
					}
				}
			}
		}
	}

	return (m_PointMatchListSet.GetCount() >= 3);
}

bool CPointMatcher::Load(FILE* fp)
{
	fscanf(fp, "%f\t%f\t%f\n", &m_pstOdometry.x, &m_pstOdometry.y, &m_pstOdometry.fThita);

	if (m_pRefPointFeatures != NULL)
		delete m_pRefPointFeatures;

	if (m_pCurPointFeatures != NULL)
		delete m_pCurPointFeatures;

	m_pRefPointFeatures = new CPointFeatureSet;
	m_pRefPointFeatures->LoadText(fp);

	m_pCurPointFeatures = new CPointFeatureSet;
	m_pCurPointFeatures->LoadText(fp);

	return true;
}

bool CPointMatcher::Save(FILE* fp)
{
	fprintf(fp, "%f\t%f\t%f\n", m_pstOdometry.x, m_pstOdometry.y, m_pstOdometry.fThita);
//	m_pRefPointFeatures->Save(fp);
//	m_pCurPointFeatures->Save(fp);
	return true;
}
