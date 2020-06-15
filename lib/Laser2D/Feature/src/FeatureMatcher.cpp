#include <stdafx.h>
#include "FeatureMatcher.h"
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
void CFeatureMatcher::SetRefFeatures(CPointFeatureSet* pPointFeatures, CLineFeatureSet* pLineFeatures)
{
	m_PointMatcher.SetRefFeatures(pPointFeatures);
	m_LineMatcher.SetRefFeatures(pLineFeatures);
}

//
//   ���õ�ǰ����ͼ��
//
void CFeatureMatcher::SetCurFeatures(CPosture& pstOdometry, CPointFeatureSet* pPointFeatures,
	CLineFeatureSet* pLineFeatures)
{
	m_pstOdometry = pstOdometry;

	m_PointMatcher.SetCurFeatures(pstOdometry, pPointFeatures);
	m_LineMatcher.SetCurFeatures(pstOdometry, pLineFeatures);
}

//
//   ���п���ƥ�䣬�ҵ��ӡ��ֲ�������-->��ȫ��������������任��
//   ����ֵ��
//      > 0 : ƥ��ɹ�
//      < 0 : �������(-2:��������; -3:���̾����쳣)
//
int CFeatureMatcher::QuickMatch(CTransform& trans, int nOption)
{
#if MATCH_OPTION == MATCH_WITH_POINTS_ONLY
	m_PointMatcher.SetMatchOption(nOption);
	return m_PointMatcher.QuickMatch(trans);
	
#elif MATCH_OPTION == MATCH_WITH_LINES_ONLY
	return m_LineMatcher.QuickMatch(trans);
#endif
}

//
//   ���б��ؾֲ�ƥ�䡣
//
int CFeatureMatcher::LocalMatch(CTransform& trans, int nOption)
{
#if MATCH_OPTION == MATCH_WITH_POINTS_ONLY
	m_PointMatcher.SetMatchOption(nOption);
	return m_PointMatcher.LocalMatch(trans);

#elif MATCH_OPTION == MATCH_WITH_LINES_ONLY
	return -2;
#endif
}

//
//   ����ƥ�䣬�ҵ��ӡ��ֲ�������-->��ȫ��������������任��
//   ����ֵ��
//      > 0 : ƥ��ɹ�
//      < 0 : �������(-2:��������; -3:���̾����쳣)
//
int CFeatureMatcher::Match(CTransform& trans, int nOption)
{
	// �ȳ��Խ��п���ƥ��
	int nResult = QuickMatch(trans, nOption);
	if (nResult >= 0)
		return nResult;
	else
		return LocalMatch(trans, nOption);
}

bool CFeatureMatcher::Load(FILE* fp)
{
	fscanf(fp, "%f\t%f\t%f\n", &m_pstOdometry.x, &m_pstOdometry.y, &m_pstOdometry.fThita);

	m_PointMatcher.Load(fp);              // ע�⣺�˴�����ĸ�ʽ�б仯����
	m_LineMatcher.Load(fp);

	return true;
}

bool CFeatureMatcher::Save(FILE* fp)
{
	fprintf(fp, "%f\t%f\t%f\n", m_pstOdometry.x, m_pstOdometry.y, m_pstOdometry.fThita);
	fprintf(fp, "0\n");

	m_PointMatcher.Save(fp);
	m_LineMatcher.Save(fp);

	return true;
}
