#pragma once

#include "ScrnRef.h"
#include "RectSelector.h"

// CScanView 视图

// 定义处理模式
#define EDIT_MODE_NORMAL                 0
#define EDIT_MODE_SEL_OBJECT             1        // 删除鼠标处对象
#define EDIT_MODE_SEL_RECT               2        // 删除矩形区域的内对象

class CLaserSlam;
class CSlamDataSet;
class CRoboMappingDoc;

class CScanView : public CScrollView
{
	DECLARE_DYNCREATE(CScanView)

private:
	CSlamDataSet* m_pDataset;
	int           m_nCurStep;    // 当前步
	int           m_nSetStep;    // 当前设置的步(可能并未立即启用)

	int m_nEditMode;             // 0 - 一般; 1-删除对象; 2-区域删除
	bool m_bBestFitMode;
	CScreenReference m_ScrnRef;
	CSize m_sizeTotalScroll;                        // 记录滚动范围
	float m_fWorldWidth;
	float m_fWorldHeight;
	DWORD m_dwMouseWheel;        // 记录鼠标滚轮动作的时间
	bool m_bShowPointFeatures;   // 是否显示点特征
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

	DWORD   m_dwTime;                          // 记录事件时间的变量
	bool    m_bScrollStart;
	CPnt    m_ptScroll;

private:
	CRoboMappingDoc* GetDocument() const;

	void ScaleToFitView();
	void OnMultiplyScale(CPoint point, float fRatio);
	
	// 判断当前窗口点是否在指定步中触碰到某个图形对象
	int PointHitObject(const CPoint& point, int nStepId, int& nParam1, int& nParam2, int& nParam3);

protected:
	CScanView();           // 动态创建所使用的受保护的构造函数
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
	virtual void OnDraw(CDC* pDC);      // 重写以绘制该视图
	virtual void OnInitialUpdate();     // 构造后的第一次

public:
	// 设定所采用的数据集
	void SetDataSet(CSlamDataSet* pDataset);

	// 设定当前的步
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


