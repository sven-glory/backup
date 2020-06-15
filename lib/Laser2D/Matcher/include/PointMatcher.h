#ifndef __CPointMatcher
#define __CPointMatcher

#include <vector>
#include "Geometry.h"
#include "LocalizationParam.h"
#include "PointMatchListSet.h"
#include "CoorTransEquations.h"
#include "FeatureSet.h"

using namespace std;

#define FM_OK                                      1
#define FM_FEATURES_NOT_ENOUGH                    -2
#define FM_MATRIX_ERROR                           -3
#define FM_NOT_MATCHED                            -4
#define FEATURE_MATCH_UNKOWN_ERROR                -10

#define FM_OPTION_LOCALIZATION                    1       // ��λӦ��
#define FM_OPTION_EVALUATE_FEATURES               2       // ����������

//////////////////////////////////////////////////////////////////////////////
//   ������ƥ���ࡱ��˵����
class CPointMatcher
{
private:
	CPosture            m_pstOdometry;
	CLocalizationParam  m_Param;        // ����
	CPointMatchListSet  m_PointMatchListSet;
	CCoorTransEquations m_Equations;
	CTransform m_trans;
	int                 m_nOption;
	float               m_fAverageErr;   // ���ڶ�Ӧ���ƥ�����
	float               m_fScore;        // ƥ��÷�(0~1)

public:
	CPointFeatureSet* m_pRefPointFeatures;
	CPointFeatureSet* m_pCurPointFeatures;
	CPointMatchList     m_PointMatchList;

private:
	bool CreateUseFlags();
	void ClearUseFlags();

public:
	bool QuickRegisterPoints();

	// ����任����������⴦�����
	bool SpecialProcess(float fSin, float fCos, float& x, float& y);
	
	// ���п���ƥ��
	int QuickMatch(CTransform& trans);

	// ���б��ؾֲ�ƥ��
	int LocalMatch(CTransform& trans);

	// ���ݵ�ǰƥ����е����ݣ�����ֱ�Ӷ��������ƽ�����׼
	short DirectRegister(CTransform trans, CPointMatchList* tab);

	// ��ȫ�ַ�Χ�ڽ��д��Ե���׼���������������m_PointMatchListSet��
	bool CoarseRegister(float equal_limit);

public:
	CPointMatcher() : m_Equations(40, 4) 
	{
		m_pRefPointFeatures = NULL;
		m_pCurPointFeatures = NULL;
		m_nOption = FM_OPTION_LOCALIZATION;
	}

	// ����ƥ��ѡ��
	void SetMatchOption(int nOption) { m_nOption = nOption; }

	// ���ö�λ����
	void SetLocalizationParam(CLocalizationParam& Param) { m_Param = Param; }

	// ���òο�����ͼ
	void SetRefFeatures(CPointFeatureSet* pPointFeatures);

	// ���õ�ǰ����
	void SetCurFeatures(CPosture& pstOdometry, CPointFeatureSet* pPointFeatures);

	// ����ƥ��
	int Match(CTransform& trans);

	CPointMatchList& GetPointMatchList() { return m_PointMatchList; }

	bool Load(FILE* fp);
	bool Save(FILE* fp);
};
#endif
