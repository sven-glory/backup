// ��� MFC ʾ��Դ������ʾ���ʹ�� MFC Microsoft Office Fluent �û����� 
// (��Fluent UI��)����ʾ�������ο���
// ���Բ��䡶Microsoft ������ο����� 
// MFC C++ ������渽����ص����ĵ���  
// ���ơ�ʹ�û�ַ� Fluent UI ����������ǵ����ṩ�ġ�  
// ��Ҫ�˽��й� Fluent UI ��ɼƻ�����ϸ��Ϣ������� 
// http://go.microsoft.com/fwlink/?LinkId=238214��
//
// ��Ȩ����(C) Microsoft Corporation
// ��������Ȩ����

// RoboMappingView.h : CRoboMappingView ��Ľӿ�
//

#pragma once

#include "WaitBusyDlg.h"
#include "RectSelector.h"
#include "FeatureSet.h"
#include "ndt_map\ndt_map.h"

// ����ģʽ����
#define MODE_CREATE_MODEL              0
#define MODE_EDIT_MODEL                1
#define MODE_MERGE_MODEL               2
#define MODE_TEST_LOCATE               3

// Edit-Modelģʽ�µ��ӹ�������
#define EDIT_SUBTYPE_DEL_OBJ           1                  // ɾ����괦����
#define EDIT_SUBTYPE_DEL_RECT          2                  // ɾ������������ڶ���
#define EDIT_SUBTYPE_TRANSLATE         3                  // ģ��ƽ�Ʋ���
#define EDIT_SUBTYPE_ROTATE            4                  // ģ����ת����
#define EDIT_SUBTYPE_ADD_REFLECTOR     5                  // ��ӷ�������
#define EDIT_SUBTYPE_DEL_REFLECTOR     6                  // ɾ����������

// Merge-Modelģʽ�µ��ӹ�������
#define MERGE_SUBTYPE_TRANSLATE        1                  // ճ�����ƽ�Ʋ���
#define MERGE_SUBTYPE_ROTATE           2                  // ճ�������ת����

// Test-Locateģʽ�µ��ӹ�������
#define TEST_SUBTYPE_INIT_POSTURE      1                  // ���ò��Եĳ�ʼ��̬
#define TEST_SUBTYPE_INIT_TRANSLATE    2
#define TEST_SUBTYPE_INIT_ROTATE       3
#define TEST_SUBTYPE_DIRECT_LOCATE     4                  // ֱ�Ӷ�λ����

// ����任����
class CTransformParam
{
private:
	CPnt m_ptFrom[2];
	CPnt m_ptTo[2];

public:
	CTransformParam() {}

	// ����ƥ����
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
	int  m_nWorkMode;            // ��ǰ����ģʽ(������ĺ궨��)
	int  m_nWorkSubType;         // ��ǰ����ģʽ�µ�����
	int  m_nWorkStage;
	int  m_nWorkParam[5];        // �������Ӳ���
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

	CSize m_sizeTotalScroll;                        // ��¼������Χ
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

	DWORD    m_dwTime;                          // ��¼�¼�ʱ��ı���
	perception_oru::NDTMap m_MapEdit;           // ������ʾƽ�ơ���ת���̵���ʱ����ͼ
	perception_oru::NDTMap m_MapToMerge;        // ���ϲ��Ŀ�
	Eigen::Transform<double, 3, Eigen::Affine, Eigen::ColMajor> m_trans;   // ���ϲ������������ۼƱ任

	CToolTipCtrl m_TipCtrl;
	CScan m_initScan0;
	CScan m_initScan;
	int   m_nTestTargetStep;
	int   m_nAutoTestCount;

	CString m_strStatus1;
	CString m_strStatus2;
	CString m_strStatus3;
	CString m_strStatus4;

protected: // �������л�����
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

// ����
public:
	CRoboMappingDoc* GetDocument() const;

// ����
public:
	void Reset();
	void Close();

	void CloseScanViewWindow();
	void RefreshModelStatus(CString str1, CString str2, CString str3);
	void AutoFitView();

// ��д
public:
	virtual void OnDraw(CDC* pDC);  // ��д�Ի��Ƹ���ͼ
	virtual BOOL PreCreateWindow(CREATESTRUCT& cs);
protected:

// ʵ��
public:
	virtual ~CRoboMappingView();
#ifdef _DEBUG
	virtual void AssertValid() const;
	virtual void Dump(CDumpContext& dc) const;
#endif

protected:

// ���ɵ���Ϣӳ�亯��
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

#ifndef _DEBUG  // RoboMappingView.cpp �еĵ��԰汾
inline CRoboMappingDoc* CRoboMappingView::GetDocument() const
   { return reinterpret_cast<CRoboMappingDoc*>(m_pDocument); }
#endif
