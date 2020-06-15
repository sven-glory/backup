#ifndef __CFeatureMatcher
#define __CFeatureMatcher

#include <vector>
#include "Geometry.h"
#include "LocalizationParam.h"
#include "PointMatchListSet.h"
#include "LineMatchList.h"
#include "CoorTransEquations.h"
#include "FeatureSet.h"
#include "PointMatcher.h"
#include "LineMatcher.h"
#include "Frame.h"

using namespace std;

#define MATCH_WITH_POINTS_ONLY                    1
#define MATCH_WITH_LINES_ONLY                     2
#define MATCH_WITH_ALL_FEATURES                   3

#define MATCH_OPTION                              MATCH_WITH_POINTS_ONLY

#define FM_OK                                      1
#define FM_FEATURES_NOT_ENOUGH                    -2
#define FM_MATRIX_ERROR                           -3
#define FM_NOT_MATCHED                            -4
#define FEATURE_MATCH_UNKOWN_ERROR                -10

//////////////////////////////////////////////////////////////////////////////
//   ������ƥ���ࡱ��˵����
class CFeatureMatcher
{
private:
	CPosture            m_pstOdometry;
	CLocalizationParam  m_Param;        // ����
	CCoorTransEquations  m_Equations;
	CTransform m_trans;

public:
	CPointMatcher     m_PointMatcher;
	CLineMatcher      m_LineMatcher;

private:
	// ���п���ƥ��
	int QuickMatch(CTransform& trans, int nOption);

	// ���б��ؾֲ�ƥ��
	int LocalMatch(CTransform& trans, int nOption);

public:
	CFeatureMatcher() : m_Equations(MAX_LINE_MATCH_COUNT * 3, 4) 
	{
	}

	// ���ö�λ����
	void SetLocalizationParam(CLocalizationParam& Param) { m_Param = Param; }

	// ���òο�����ͼ
	void SetRefFeatures(CPointFeatureSet* pPointFeatures, CLineFeatureSet* pLineFeatures);

	// ���õ�ǰ����
	void SetCurFeatures(CPosture& pstOdometry, CPointFeatureSet* pPointFeatures, 
		CLineFeatureSet* pLineFeatures);

	// ����ƥ��
	int Match(CTransform& trans, int nOption = FM_OPTION_LOCALIZATION);

	CPointMatchList& GetPointMatchList() { return m_PointMatcher.GetPointMatchList(); }

	bool Load(FILE* fp);
	bool Save(FILE* fp);
};
#endif
