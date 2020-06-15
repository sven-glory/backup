#pragma once

#include "ScrnRef.h"
#include "RectSelector.h"

// CScanView ��ͼ

// ���崦��ģʽ
#define EDIT_MODE_NORMAL                 0
#define EDIT_MODE_SEL_OBJECT             1        // ɾ����괦����
#define EDIT_MODE_SEL_RECT               2        // ɾ������������ڶ���

class CLaserSlam;
class CSlamDataSet;
class CRoboMappingDoc;

class CScanView : public CScrollView
{
	DECLARE_DYNCREATE(CScanView)

private:
	CSlamDataSet* m_pDataset;
	int           m_nCurStep;    // ��ǰ��
	int           m_nSetStep;    // ��ǰ���õĲ�(���ܲ�δ��������)

	int m_nEditMode;             // 0 - һ��; 1-ɾ������; 2-����ɾ��
	bool m_bBestFitMode;
	CScreenReference m_ScrnRef;
	CSize m_sizeTotalScroll;                        // ��¼������Χ
	float m_fWorldWidth;
	float m_fWorldHeight;
	DWORD m_dwMouseWheel;        // ��¼�����ֶ�����ʱ��
	bool m_bShowPointFeatures;   // �Ƿ���ʾ������
	bool  m_bTransformScan;
	bool  m_bSuggestFeatures;

	bool m_bControlKeyDown;
	bool          m_bStartRect;
	CRectSelector m_RectSelector;
	CPoint m_OldPoint;

	HCURSOR m_hCursorDel;
	HCURSOR m_hCursorDelThis;
	HCURSOR m_hCursorDelRect;
	HCURSOR m_hCursorSel;
	HCURSOR m_hCursorSelThis;
	HCURSOR m_hCursorSelRect;

	DWORD   m_dwTime;                          // ��¼�¼�ʱ��ı���
	bool    m_bScrollStart;
	CPnt    m_ptScroll;

private:
	CRoboMappingDoc* GetDocument() const;

	void ScaleToFitView();
	void OnMultiplyScale(CPoint point, float fRatio);
	
	// �жϵ�ǰ���ڵ��Ƿ���ָ�����д�����ĳ��ͼ�ζ���
	int PointHitObject(const CPoint& point, int nStepId, int& nParam1, int& nParam2, int& nParam3);

protected:
	CScanView();           // ��̬������ʹ�õ��ܱ����Ĺ��캯��
	virtual ~CScanView();

public:
#ifdef _DEBUG
	virtual void AssertValid() const;
#ifndef _WIN32_WCE
	virtual void Dump(CDumpContext& dc) const;
#endif
#endif
	void Close();

protected:
	virtual void OnDraw(CDC* pDC);      // ��д�Ի��Ƹ���ͼ
	virtual void OnInitialUpdate();     // �����ĵ�һ��

public:
	// �趨�����õ����ݼ�
	void SetDataSet(CSlamDataSet* pDataset);

	// �趨��ǰ�Ĳ�
	bool SetCurStep(int step, bool bApply = false);

	DECLARE_MESSAGE_MAP()
	virtual void OnUpdate(CView* /*pSender*/, LPARAM /*lHint*/, CObject* /*pHint*/);
public:
	afx_msg void OnMouseMove(UINT nFlags, CPoint point);
	afx_msg void OnSize(UINT nType, int cx, int cy);
	afx_msg void OnTimer(UINT_PTR nIDEvent);
	virtual BOOL PreCreateWindow(CREATESTRUCT& cs);
	afx_msg BOOL OnMouseWheel(UINT nFlags, short zDelta, CPoint pt);
	virtual BOOL OnScrollBy(CSize sizeScroll, BOOL bDoScroll = TRUE);
	afx_msg void OnAutoFit();
	afx_msg void OnRButtonDown(UINT nFlags, CPoint point);
	afx_msg void OnLButtonDown(UINT nFlags, CPoint point);
	afx_msg void OnLButtonUp(UINT nFlags, CPoint point);
	afx_msg void OnKeyDown(UINT nChar, UINT nRepCnt, UINT nFlags);
	afx_msg int OnMouseActivate(CWnd* pDesktopWnd, UINT nHitTest, UINT message);
	afx_msg void OnMouseHover(UINT nFlags, CPoint point);
	afx_msg void OnShowHighIntensity();
	afx_msg void OnUpdateShowHighIntensity(CCmdUI *pCmdUI);
	afx_msg void OnKeyUp(UINT nChar, UINT nRepCnt, UINT nFlags);
	afx_msg void OnMButtonDown(UINT nFlags, CPoint point);
	afx_msg void OnMButtonUp(UINT nFlags, CPoint point);
	afx_msg void OnViewPrevStep();
	afx_msg void OnUpdateViewPrevStep(CCmdUI *pCmdUI);
	afx_msg void OnViewNextStep();
	afx_msg void OnUpdateViewNextStep(CCmdUI *pCmdUI);
	afx_msg void OnViewCurStep();
	afx_msg void OnUpdateViewCurStep(CCmdUI *pCmdUI);
	afx_msg void OnViewAnyStep();
	afx_msg void OnUpdateViewAnyStep(CCmdUI *pCmdUI);
	afx_msg void OnScanCompare();
};


