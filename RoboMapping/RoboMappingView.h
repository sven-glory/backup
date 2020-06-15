// 这段 MFC 示例源代码演示如何使用 MFC Microsoft Office Fluent 用户界面 
// (“Fluent UI”)。该示例仅供参考，
// 用以补充《Microsoft 基础类参考》和 
// MFC C++ 库软件随附的相关电子文档。  
// 复制、使用或分发 Fluent UI 的许可条款是单独提供的。  
// 若要了解有关 Fluent UI 许可计划的详细信息，请访问 
// http://go.microsoft.com/fwlink/?LinkId=238214。
//
// 版权所有(C) Microsoft Corporation
// 保留所有权利。

// RoboMappingView.h : CRoboMappingView 类的接口
//

#pragma once

#include "WaitBusyDlg.h"
#include "RectSelector.h"
#include "FeatureSet.h"
#include "ndt_map\ndt_map.h"

// 工作模式定义
#define MODE_CREATE_MODEL              0
#define MODE_EDIT_MODEL                1
#define MODE_MERGE_MODEL               2
#define MODE_TEST_LOCATE               3

// Edit-Model模式下的子工作类型
#define EDIT_SUBTYPE_DEL_OBJ           1                  // 删除鼠标处对象
#define EDIT_SUBTYPE_DEL_RECT          2                  // 删除矩形区域的内对象
#define EDIT_SUBTYPE_TRANSLATE         3                  // 模型平移操作
#define EDIT_SUBTYPE_ROTATE            4                  // 模型旋转操作
#define EDIT_SUBTYPE_ADD_REFLECTOR     5                  // 添加反光板操作
#define EDIT_SUBTYPE_DEL_REFLECTOR     6                  // 删除反光板操作

// Merge-Model模式下的子工作类型
#define MERGE_SUBTYPE_TRANSLATE        1                  // 粘贴块的平移操作
#define MERGE_SUBTYPE_ROTATE           2                  // 粘贴块的旋转操作

// Test-Locate模式下的子工作类型
#define TEST_SUBTYPE_INIT_POSTURE      1                  // 设置测试的初始姿态
#define TEST_SUBTYPE_INIT_TRANSLATE    2
#define TEST_SUBTYPE_INIT_ROTATE       3
#define TEST_SUBTYPE_DIRECT_LOCATE     4                  // 直接定位测试

// 坐标变换参数
class CTransformParam
{
private:
	CPnt m_ptFrom[2];
	CPnt m_ptTo[2];

public:
	CTransformParam() {}

	// 设置匹配点对
	void SetMatchPair(int nIdx, const CPnt& ptFrom, const CPnt& ptTo)
	{
		m_ptFrom[nIdx] = ptFrom;
		m_ptTo[nIdx] = ptTo;
	}
};

class CLaserSlam;

class CRoboMappingView : public CScrollView
{
private:
	bool m_bShowId;
	bool m_bBestFitMode;
	int  m_nWorkMode;            // 当前工作模式(见上面的宏定义)
	int  m_nWorkSubType;         // 当前工作模式下的类型
	int  m_nWorkStage;
	int  m_nWorkParam[5];        // 工作附加参数
	bool m_bMouseNotMove;
	bool m_bControlKeyDown;

	bool          m_bStartRect;
	CRectSelector m_RectSelector;
	CPoint m_OldPoint;
	
	bool   m_bScrollStart;
	CPnt   m_ptScroll;

	HCURSOR m_hCursorDel;
	HCURSOR m_hCursorDelThis;
	HCURSOR m_hCursorDelRect;
	HCURSOR m_hCursorAddReflector;
	HCURSOR m_hCursorReflector2;
	HCURSOR m_hCursorReflectorMove;
	HCURSOR m_hCursorTranslation;
	HCURSOR m_hCursorTranslation2;
	HCURSOR m_hCursorRotate1;
	HCURSOR m_hCursorRotate2;
	HCURSOR m_hCursorRotate3;

	CSize m_sizeTotalScroll;                        // 记录滚动范围
	float m_fWorldWidth;
	float m_fWorldHeight;

	bool m_bShowActiveSide;
	bool m_bShowSourceMatched;
	bool m_bShowTargetMatched;

	bool m_bShowHighIntensity;
	bool m_bShowReflectors;

	CWaitBusyDlg* m_pWaitBusyDlg;

	CTransformParam m_TransParam;
	CPnt m_ptFrom;
	CPnt m_ptTo;
	CPnt m_ptCur;
	CPnt m_ptRotateCenter;
	
	CPosture m_pstTest;
	CPosture m_pstTestResult;
	int      m_nTestStep;
	CPosture m_pstMove;
	bool     m_bFirstDraw;

	DWORD    m_dwTime;                          // 记录事件时间的变量
	perception_oru::NDTMap m_MapEdit;           // 用于显示平移、旋转过程的临时特征图
	perception_oru::NDTMap m_MapToMerge;        // 待合并的块
	Eigen::Transform<double, 3, Eigen::Affine, Eigen::ColMajor> m_trans;   // 待合并块所经过的累计变换

	CToolTipCtrl m_TipCtrl;
	CScan m_initScan0;
	CScan m_initScan;
	int   m_nTestTargetStep;
	int   m_nAutoTestCount;

	CString m_strStatus1;
	CString m_strStatus2;
	CString m_strStatus3;
	CString m_strStatus4;

protected: // 仅从序列化创建
	CRoboMappingView();
	DECLARE_DYNCREATE(CRoboMappingView)

private:
	void ScaleToFitView();
	int PointHitReflector(CPoint& pt, CPointFeatureSet* pPointFeatureSet);
	void OnMagnify(CPoint point);
	void OnReduce(CPoint point);
	void DoStepLocate();

	void OnDrawCreateModel(CDC* pDC);
	void OnDrawEditModel(CDC* pDC);
	void OnDrawMergeModel(CDC* pDC);
	void OnDrawTestLocate(CDC* pDC);

	void OnMouseMoveCreateModel(CPoint point);
	void OnMouseMoveEditModel(CPoint point);
	void OnMouseMoveMergeModel(CPoint point);
	void OnMouseMoveTestLocate(CPoint point);

	void OnLButtonDownCreateModel(CPoint point);
	void OnLButtonDownEditModel(CPoint point);
	void OnLButtonDownMergeModel(CPoint point);
	void OnLButtonDownTestLocate(CPoint point);

	void OnLButtonUpEditModel(CPoint point);

	void OnRButtonDownCreateModel(CPoint point);
	void OnRButtonDownEditModel(CPoint point);
	void OnRButtonDownMergeModel(CPoint point);
	void OnRButtonDownTestLocate(CPoint point);

// 特性
public:
	CRoboMappingDoc* GetDocument() const;

// 操作
public:
	void Reset();
	void Close();

	void CloseScanViewWindow();
	void RefreshModelStatus(CString str1, CString str2, CString str3);
	void AutoFitView();

// 重写
public:
	virtual void OnDraw(CDC* pDC);  // 重写以绘制该视图
	virtual BOOL PreCreateWindow(CREATESTRUCT& cs);
protected:

// 实现
public:
	virtual ~CRoboMappingView();
#ifdef _DEBUG
	virtual void AssertValid() const;
	virtual void Dump(CDumpContext& dc) const;
#endif

protected:

// 生成的消息映射函数
protected:
	afx_msg void OnRButtonUp(UINT nFlags, CPoint point);
	afx_msg void OnContextMenu(CWnd* pWnd, CPoint point);
	DECLARE_MESSAGE_MAP()
public:
	afx_msg void OnTimer(UINT_PTR nIDEvent);
	virtual void OnInitialUpdate();
	afx_msg void OnZoomIn();
	afx_msg void OnZoomOut();
	afx_msg void OnChar(UINT nChar, UINT nRepCnt, UINT nFlags);
	afx_msg void OnMouseMove(UINT nFlags, CPoint point);
	afx_msg void OnShowId();
	virtual void OnUpdate(CView* /*pSender*/, LPARAM /*lHint*/, CObject* /*pHint*/);
	afx_msg void OnMagnify();
	afx_msg void OnReduce();
	afx_msg void OnBestFit();
	afx_msg void OnLButtonDown(UINT nFlags, CPoint point);
	afx_msg void OnLButtonUp(UINT nFlags, CPoint point);
	afx_msg void OnLoadMap();
	afx_msg void OnSaveMap();
	afx_msg void OnDelSingle();
	afx_msg void OnEndBuild();
	afx_msg void OnRotation();
	afx_msg void OnSize(UINT nType, int cx, int cy);
	afx_msg void OnRButtonDown(UINT nFlags, CPoint point);
	afx_msg BOOL OnSetCursor(CWnd* pWnd, UINT nHitTest, UINT message);
	afx_msg void OnDelRect();
	afx_msg BOOL OnMouseWheel(UINT nFlags, short zDelta, CPoint pt);
	virtual BOOL OnScrollBy(CSize sizeScroll, BOOL bDoScroll = TRUE);
	afx_msg void OnTranslation();
	afx_msg void OnUpdateShowId(CCmdUI *pCmdUI);
	afx_msg void OnImportMergeFile();
	afx_msg void OnPatchTranslate();
	afx_msg void OnPatchRotate();
	afx_msg void OnAcceptMerge();
	afx_msg void OnCancelMerge();
	afx_msg void OnShowSourceMatched();
	afx_msg void OnUpdateShowSourceMatched(CCmdUI *pCmdUI);
	afx_msg void OnUpdatePatchTranslate(CCmdUI *pCmdUI);
	afx_msg void OnUpdateMerge(CCmdUI *pCmdUI);
	afx_msg void OnUpdatePatchRotate(CCmdUI *pCmdUI);
	afx_msg void OnUpdateAcceptMerge(CCmdUI *pCmdUI);
	afx_msg void OnUpdateCancelMerge(CCmdUI *pCmdUI);
	afx_msg void OnUpdateShowScene(CCmdUI *pCmdUI);
	afx_msg void OnLButtonDblClk(UINT nFlags, CPoint point);
	afx_msg void OnKeyDown(UINT nChar, UINT nRepCnt, UINT nFlags);
	afx_msg void OnKeyUp(UINT nChar, UINT nRepCnt, UINT nFlags);
	afx_msg void OnOffsetTest();
	afx_msg void OnMButtonDown(UINT nFlags, CPoint point);
	afx_msg void OnMButtonUp(UINT nFlags, CPoint point);
	afx_msg void OnPatchFit();
	afx_msg void OnLinkDataset();
	afx_msg void OnTargetMatched();
	afx_msg void OnUpdateTargetMatched(CCmdUI *pCmdUI);
	afx_msg void OnLocInitPostureTrans();
	afx_msg void OnLocInitPostureRot();
	afx_msg void OnBatchLocate();
	afx_msg void OnLoadLocateDataset();
	afx_msg void OnSetLocateInitPosture();
	afx_msg void OnStepLocate();
	afx_msg void OnUpdateStepLocate(CCmdUI *pCmdUI);
	afx_msg void OnUpdateBatchLocate(CCmdUI *pCmdUI);
	afx_msg void OnUpdateLocInitPostureTrans(CCmdUI *pCmdUI);
	afx_msg void OnUpdateLocInitPostureRot(CCmdUI *pCmdUI);
	afx_msg void OnUpdatePrepareLoc(CCmdUI *pCmdUI);
	afx_msg void OnEndLocalize();
	afx_msg void OnUpdateEndLocalize(CCmdUI *pCmdUI);
	afx_msg void OnShowHighIntensity();
	afx_msg void OnUpdateShowHighIntensity(CCmdUI *pCmdUI);
	afx_msg void OnShowReflectors();
	afx_msg void OnUpdateShowReflectors(CCmdUI *pCmdUI);
	afx_msg void OnAddReflector();
	afx_msg void OnDelReflector();
	afx_msg void OnImportRouteMap();
	afx_msg void OnUpdateTranslation(CCmdUI *pCmdUI);
	afx_msg void OnUpdateRotation(CCmdUI *pCmdUI);
	virtual BOOL PreTranslateMessage(MSG* pMsg);
};

#ifndef _DEBUG  // RoboMappingView.cpp 中的调试版本
inline CRoboMappingDoc* CRoboMappingView::GetDocument() const
   { return reinterpret_cast<CRoboMappingDoc*>(m_pDocument); }
#endif
