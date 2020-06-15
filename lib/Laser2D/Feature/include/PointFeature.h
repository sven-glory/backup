#ifndef __CPointFeature
#define __CPointFeature

#include <stdio.h>
#include "Geometry.h"
#include "Feature.h"

#ifdef QT_VERSION
#include <QColor>
#endif

// ��������
#define GENERIC_POINT_FEATURE      0        // һ������
#define FLAT_REFLECTOR_FEATURE     1        // ƽ�淴�������
#define ROUND_FEATURE              2        // Բ������
#define CORNER_FEATURE             3        // �ǵ�
#define SHORT_LINE_FEATURE         4        // ��ֱ������
#define EDGE_FEATURE               5        // ��Ե����
#define IRREGULAR_PILLAR_FEATURE   6

// ͬ������������������
#define FEATURE_REG_DISTANCE_WINDOW           1.0f

class CScan;

///////////////////////////////////////////////////////////////////////////////
//   ���塰�����������ࡣ

class CPointFeature : public CFeature, public CPnt
{
public:
	int m_nMatchId;            // ��֮ƥ��ĵ�������ID��
	int m_nStartPointId;       // ԭʼ���������������Ӧ����ʼ��ID
	int m_nEndPointId;         // ԭʼ���������������Ӧ����ֹ��ID
	int m_nPointCount;         // ɨ�������

public:
	CPointFeature(float _x = 0, float _y = 0);

	// ��������λ��
	void SetCenterPoint(const CPnt& ptCenter) { GetPntObject() = ptCenter;}

	// ����һ������
	virtual CPointFeature* Duplicate() const;

	// �жϸõ������Ƿ�ָ����ɨ�����յ���������ɨ�������
	virtual bool HitByLineAt(CLine& lnRay, CPnt& ptHit, float& fDist);

	// ��Ը����ĵ��ƣ��ڹ涨�ĽǶȷ�Χ�ڣ����������Ƿ��иõ���������������������λ��
	virtual bool Detect(CPosture& pst, CScan* pScan, float fStartAngle, float fEndAngle, CPnt* ptCenter);

	// �ж�ָ������������Ƿ����
	virtual bool CheckInRay(CLine& lnRay) {return true;}

	// ���ı��ļ���װ��������Ĳ���
	virtual int LoadText(FILE* fp);

	// ���������Ĳ������浽�ı��ļ���
	virtual int SaveText(FILE* fp);

	// �Ӷ������ļ���װ��������Ĳ���
	virtual int LoadBinary(FILE* fp);

	// ���������Ĳ������浽�������ļ���
	virtual int SaveBinary(FILE* fp);

	// ���ص�����������뾶(��Ҫ������Ļ������)
	virtual float GetPointRadius() { return 0.02f; }

#ifdef _MFC_VER
	// ����Ļ�ϻ��ƴ˵�����
	virtual void Plot(CScreenReference& ScrnRef, CDC* pDC, COLORREF crColor, COLORREF crSelected, int nPointSize,
		int nLineWidth = 0);

	// ����Ļ�ϻ��ƴ˵�������ID��
	virtual void PlotId(CScreenReference& ScrnRef, CDC* pDC, COLORREF crColor);

#elif defined QT_VERSION
	// ����Ļ�ϻ��ƴ˵�����
	virtual void Plot(CScreenReference& ScrnRef, QPainter* pPainter, QColor crColor, QColor crSelected, int nPointSize,
		int nLineWidth = 0);

	// ����Ļ�ϻ��ƴ˵�������ID��
	virtual void PlotId(CScreenReference& ScrnRef, QPainter* pPainter, QColor crColor);
#endif
};
#endif
