#pragma once

#include "Geometry.h"

//
//   ��Ӧ�������ϸ���ɢ����������ݡ�
//
class CCurveSamplePoint : public CPnt
{
public:
	float t;                           // ���Ʋ���
	float fProgress;                   // ����ʼ�㵽������������߳���
	float fSegLen;                     // ����һ�����㵽��������ĳ���
	float fTangentAngle;               // ���߷����
	float fCurvature;                  // ����ֵ
	float fUserData[10];               // (Ԥ����)�������ж��������

public:
	CCurveSamplePoint()
	{
		t = 0;
		fProgress = 0;
		fSegLen = 0;
		fTangentAngle = 0;
		fCurvature = 0;

		for (int i = 0; i < 10; i++)
			fUserData[i] = 0;
	}
};

///////////////////////////////////////////////////////////////////////////////
//   ����Bezier���ߡ�
class DllExport CBezier
{
private:
	float dx1;
	float dy1;
	float dx2;
	float dy2;

	int   m_nSampleCount;                    // ����������
	CCurveSamplePoint* m_pSamplePoints;      // ���������ݻ�������ָ��

public:
	int   m_nCountKeyPoints;
	CPnt* m_ptKey;
	CPnt m_pt;              // The trajectory point
	CAngle m_angTangent;
	float  m_fCurvature;
	float  m_fTotalLen;         // ���߳���

private:
	void(*UserDataCreateProc)(CBezier*, void*);

	// ����ռ�
	void Clear();

public:
	// ��������ױ���������
	CBezier(int nCountKeyPoints, CPnt* pptKey);

	// �������ױ���������
	CBezier(const CPosture& pstStart, const CPosture& pstEnd, float fLen1, float fLen2);
	
	// ȱʡ���캯��(������Ҫ��ʽ����Create����)
	CBezier();
	
	// ��������
	~CBezier();

	// ���ء�=��������
	void operator = (const CBezier& Obj);

	// ���ݸ����Ĺؼ������������Bezier����
	bool Create(int nCountKeyPoints, CPnt* pptKey);

	// ��������Bezier����
	bool Create(const CPosture& pstStart, const CPosture& pstEnd, float fLen1, float fLen2);

	// ����ѡ�㳣��K��������Bezier����
	bool Create(const CPosture& pstStart, const CPosture& pstEnd, float K);

	// ������ɢ����������
	bool CreateSamplePoints();

	// ��ָ������Ŵ�����һ�����Ƶ�
	bool AddKeyPoint(int nIdx, const CPnt& pt);

	// ��ָ����Ŵ��Ŀ��Ƶ��Ƴ�
	bool RemoveKeyPoint(int nIdx);

	// ���ݸ����Ĵ��������������ߵ��û�����
	void CreateUserData(void(*pProc)(CBezier*, void*), void* pParam);

	// ȡ��ָ�����û�����
	float GetUserData(int nSampleIdx, int nDataIdx) const;

	// �˶�һ�£��������Ƿ�ͻ���˹涨��Լ��
	bool CheckConstraints(float fMaxCurvature, float fMaxCurvatureDiff) const;

	// ���õ�ǰ�����Ա�ȷ����ǰ��
	virtual void SetCurT(float t);

	// The trajectory generation function
	virtual CPnt TrajFun() const {return m_pt;}

	// The tangent angle generation function
	virtual CAngle TangentFun() const {return m_angTangent;}

	// The curvature generation function
	virtual float CurvatureFun() const {return m_fCurvature;}

	// ���ݾ����Ľ��Ⱦ���ȷ����Ӧ��tֵ
	float GetTFromProgress(float fLen);

	// ����������һ��pt���������������ĵ�
	bool GetClosestPoint(const CPnt& pt, CPnt* ptClosest = NULL, float* t = NULL);

	// ���ı��ļ�װ����������
	bool Create(FILE *StreamIn);

	// ���������ݱ��浽�ı��ļ�
	bool Save(FILE *StreamOut);

#ifdef _MFC_VER
	// �Ӷ������ļ�װ����������
	bool Create(CArchive& ar);

	// ���������ݱ��浽�������ļ�
	bool Save(CArchive& ar);

	// ��������
	void Draw(CScreenReference& ScrnRef, CDC* pDC, COLORREF crColor, int nWidth, int nPointSize, bool bShowKeyPoints = false, int nPenStyle = PS_SOLID);

	// �������Ƶ�
	void DrawCtrlPoints(CScreenReference& ScrnRef, CDC* pDC, COLORREF crColor, int nPointSize);
#endif
};

