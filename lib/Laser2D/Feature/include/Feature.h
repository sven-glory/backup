#ifndef __CFeature
#define __CFeature

#include "Geometry.h"
#include "ScrnRef.h"

// ��������
#define FEATURE_TYPE_POINT                    1        // ������
#define FEATURE_TYPE_LINE                     2        // ֱ������

// �����Ĺ۲췽����
#define FEATURE_DIR_FRONT_SIDE_ONLY           1        // ֻʹ��ֱ������
#define FEATURE_DIR_BACK_SIDE_ONLY            2        // ֻʹ��ֱ�߷���
#define FEATURE_DIR_BOTH_SIDES                3        // ʹ��ֱ����������

#define SIASUN_MATCHER_ANGLE_WINDOW           (15*PI/180)   // +/-15�ȿ��Ž�
#define SIASUN_MATCHER_DIST_WINDOW            (400)         // +/-400mm����

#define FEATURE_PARAM_NUM                     10       // Ԥ������չ����������

// ���ڱ�����ֹ������������ʾ��ʽ
#define DISABLED_FEATURE_NOT_SHOWN            0        // ����ʾ��ֹ��
#define DISABLED_FEATURE_UNDERTONE            1        // ��ɫ��ʾ��ֹ��
#define DISABLED_FEATURE_NORMAL               2        // ������ʾ��ֹ��

///////////////////////////////////////////////////////////////////////////////
//   ���塰���������ࡣ
class CFeature
{
public:
	int      m_nType;                        // ����������
	int      m_nSubType;                     // ������������
	int      m_nFeatureId;                   // �����ı��
	bool     m_bBad;                         // �����Ƿ��ǡ�������
	bool     m_bEnabled;                     // �����ı������
	bool     m_bSuggested;                   // �Ƿ��ǡ��Ƽ�������
	int      m_nParam[FEATURE_PARAM_NUM];    // ����չʹ�õĸ�����������
	float    m_fParam[FEATURE_PARAM_NUM];    // ����չʹ�õĸ��Ӹ���������

public:
	CFeature() 
	{
		m_nType = 0;
		m_nSubType = 0;
		m_nFeatureId = 0;
		m_bBad = false;
		m_bSuggested = false;
		m_bEnabled = true;

		// ��չ������ʼ��
		for (int i = 0; i < FEATURE_PARAM_NUM; i++)
		{
			m_nParam[i] = 0;
			m_fParam[i] = 0;
		}
	}

	// ��������������
	void SetFeatureType(int nType) { m_nType = nType; }

	// ȡ������������
	int GetFeatureType() const { return m_nType; }

	// ����������������
	void SetSubType(int nSubType) { m_nSubType = nSubType; }

	// ȡ��������������
	int GetSubType() const { return m_nSubType; }

	// �ж��Ƿ���һ��������������
	bool IsBad() const { return m_bBad; }

	// �ж������Ƿ�ʹ��
	bool IsEnabled() const { return m_bEnabled; }

	// ʹ��/��ֹ������
	void Enable(bool bTrueOrFalse) { m_bEnabled = bTrueOrFalse; }	

	// �жϸ������Ƿ��ǡ��Ƽ�������
	bool IsSuggested() const { return m_bSuggested; }

	// ����/ȡ����������
	void SetBad(bool bTrueOrFalse) { m_bBad = bTrueOrFalse; }

	// �Ƽ�/���Ƽ�������
	void Suggest(bool bTrueOrFalse) { m_bSuggested = bTrueOrFalse; }

	// ������չ����
	bool SetIntParam(int nIdx, int nParam)
	{
		if (nIdx >= FEATURE_PARAM_NUM)
			return false;

		m_nParam[nIdx] = nParam;
		return true;
	}

	// ���ø�������չ����ֵ
	bool SetFloatParam(int nIdx, float fParam)
	{
		if (nIdx >= FEATURE_PARAM_NUM)
			return false;

		m_fParam[nIdx] = fParam;
		return true;
	}

	virtual int LoadText(FILE* fp) { return 0; }
	virtual int SaveText(FILE* fp) { return 0; }
	virtual int LoadBinary(FILE* fp) { return 0; }
	virtual int SaveBinary(FILE* fp) { return 0; }

#ifdef _MSC_VER
//	virtual void Plot(CScreenReference& ScrnRef, CDC* pDC, COLORREF crColor, COLORREF crSelected, int nSize) = 0;
#endif
};
#endif
