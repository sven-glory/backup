#pragma once

#include "Geometry.h"

class CScreenReference;

//////////////////////////////////////////////////////////////////////////////
//   The interface of class "CEllipse".
class DllExport CEllipse
{
public:
	CPnt m_ptCenter;        // ��Բ����
	float m_fHalfMajorAxis; // �����᳤��
	float m_fHalfMinorAxis; // �̰��᳤��
	CAngle m_angSlant;      // ��б��

public:
	// The constructor
	CEllipse(const CPnt& ptCenter, float fHalfMajorAxis, float fHalfMiorAxis, const CAngle& angSlant);

	// Default constructor
	CEllipse() {}

	// ���ɴ���ԲԲ
	void Create(const CPnt& ptCenter, float fHalfMajorAxis, float fHalfMiorAxis, const CAngle& angSlant);

	// �ж�һ�����Ƿ�����Բ����
	bool Contain(const CPnt& pt) const;

#ifdef _MFC_VER
	// ����Ļ�ϻ��ƴ�Բ
	void Draw(CScreenReference& ScrnRef, CDC* pDC, COLORREF crLineColor, int nLineWidth, 
		COLORREF crFillColor = 0, bool bFill = false, int nPenStyle = PS_SOLID);
#elif defined QT_VERSION
	// ����Ļ�ϻ��ƴ�Բ
	void Draw(CScreenReference& ScrnRef, QPainter* pPainter, QColor crLineColor, int nLineWidth,
		QColor crFillColor, bool bFill = false, int nPenStyle = 0);
#endif
};
