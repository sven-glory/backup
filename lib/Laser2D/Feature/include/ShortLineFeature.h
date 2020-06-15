#ifndef __CShortLineFeature
#define __CShortLineFeature

#include "PointFeature.h"

#define SHORT_LINE_MAX_DIST_CHANGE            0.4f     // ����������仯��
#define MAX_POINT_NUM_IN_SHORT_LINE           25       // ��ֱ���к�����������

class CScanPointCloud;

///////////////////////////////////////////////////////////////////////////////
//   ��ֱ������(��Ϊһ�ֵ�����)��
class CShortLineFeature : public CPointFeature
{
protected:
	float m_fWidth;                // ��ֱ�߶εĿ��
	float m_fAngle;                // ��ֱ�߶������������ڽǶ�
	CLine m_ln;                    // ��Ӧ��ֱ�߶�
	int   m_nWhichSideToUse;       // ʹ��������һ��(��˫��)
	float m_fMaxIncidenceAngle;    // ÿ�������������(0~PI/2)
	int   m_nMinNumPoints;         // ���ٵ���
	float m_fMaxLength;            // �߶���󳤶�

public:
	CShortLineFeature();

	// ����һ������
	virtual CPointFeature* Duplicate() const;

	// ���ö�ֱ�������Ĳ���
	virtual void SetParam(float fWidth, float fAngle, int nWhichSideToUse, 
		float fMaxIncidenceAngle, int nMinNumPoints, float fMaxLength);

	// �ж϶�ֱ�߶��Ƿ�ָ����ɨ�����յ���������ɨ�������
	virtual bool HitByLineAt(CLine& lnRay, CPnt& ptHit, float& fDist);

	// ��Ը����ĵ��ƣ��ڹ涨�ĽǶȷ�Χ�ڣ����������Ƿ��иö�ֱ�߶���������������������λ��
	virtual bool Detect(CPosture& pst, CScan* pScan, float fStartAngle, float fEndAngle, CPnt* ptCenter);

	// ����һ������
	virtual CPointFeature* Duplicate();

	// �ж�ָ������������Ƿ����
	virtual bool CheckInRay(CLine& lnRay);

	// ���ı��ļ���װ��������Ĳ���
	virtual int LoadText(FILE* fp);

	// ���������Ĳ������浽�ı��ļ���
	virtual int SaveText(FILE* fp);

	// �Ӷ������ļ���װ��������Ĳ���
	virtual int LoadBinary(FILE* fp);

	// ���������Ĳ������浽�������ļ���
	virtual int SaveBinary(FILE* fp);

	// �����������任
	virtual void Transform(const CFrame& frame);

	// ����������任
	virtual void InvTransform(const CFrame& frame);

#ifdef _MSC_VER
	// ����Ļ�ϻ��ƴ˵�����
	virtual void Plot(CScreenReference& ScrnRef, CDC* pDC, COLORREF crColor, COLORREF crSelected, int nSize);
#endif
};
#endif

