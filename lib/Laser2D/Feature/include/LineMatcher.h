#ifndef __CLineMatcher
#define __CLineMatcher

#include <vector>
#include "Geometry.h"
#include "LocalizationParam.h"
#include "LineMatchList.h"
#include "CoorTransEquations.h"
#include "FeatureSet.h"

using namespace std;

#define FM_OK                                      1
#define FM_FEATURES_NOT_ENOUGH                    -2
#define FM_MATRIX_ERROR                           -3
#define FM_NOT_MATCHED                            -4
#define FEATURE_MATCH_UNKOWN_ERROR                -10

//////////////////////////////////////////////////////////////////////////////
//   ������ƥ���ࡱ��˵����
class CLineMatcher
{
private:
	CPosture             m_pstOdometry;
	CLocalizationParam   m_Param;        // ����
	CCoorTransEquations  m_Equations;
	CTransform           m_trans;

public:
	CLineFeatureSet      m_RefLineFeatures;
	CLineFeatureSet      m_CurLineFeatures;
	CLineMatchList       m_LineMatchList;

public:
	int QuickRegisterLines();

	// ���п���ƥ��
	int QuickMatch(CTransform& trans);

	// ���б��ؾֲ�ƥ��
	int LocalMatch(CTransform& trans);

public:
	CLineMatcher() : m_Equations(40, 4) 
	{
	}

	// ���ö�λ����
	void SetLocalizationParam(CLocalizationParam& Param) { m_Param = Param; }

	// ���òο�����ͼ
	void SetRefFeatures(CLineFeatureSet* pLineFeatures);

	// ���õ�ǰ����
	void SetCurFeatures(CPosture& pstOdometry, CLineFeatureSet* pLineFeatures);

//	CPointMatchList& GetPointMatchList() { return m_PointMatcher.GetPointMatchList(); }

	bool Load(FILE* fp);
	bool Save(FILE* fp);
};
#endif
