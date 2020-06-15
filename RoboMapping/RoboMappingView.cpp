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

// RoboMappingView.cpp : CRoboMappingView 类的实现
//

#include "stdafx.h"
// SHARED_HANDLERS 可以在实现预览、缩略图和搜索筛选器句柄的
// ATL 项目中进行定义，并允许与该项目共享文档代码。
#ifndef SHARED_HANDLERS
#include "RoboMapping.h"
#endif

#include "MainFrm.h"
#include "RoboMappingDoc.h"
#include "RoboMappingView.h"
#include <Afxmt.h>
#include "DebugTrace.h"
#include "TransformDlg.h"
#include "TranslationDlg.h"
#include "SlamDataSet.h"
#include "ScanView.h"
#include "PointFeatureDlg.h"
#include "OutputOptionDlg.h"
#include "FlatReflectorFeature.h"
#include "CLinkDatasetDlg.h"
#include "GotoDlg.h"
#include "WorldPointDlg.h"
#include "AffinePosture.h"
#include "PclPointCloud.h"
#include "World.h"

#include "ndt_fuser/RobotLocalization.h"
#include "ndt_fuser/DatasetLocalization.h"

#if 0
#ifdef _DEBUG
#define new DEBUG_NEW
#endif
#endif


#define MARGIN_DIST              2.0f         // 布局图边缘至少留有2米空隙
#define MAX_DRAWING_RATIO        200
#define MIN_DRAWING_RATIO        0.001f         // 布局图最小放大倍数
#define MARGIN_SIZE              20           // 布局图边缘至少留有20个像素点的空隙

#define MAGNIFY_CONST            1.2f
#define WM_SIMPLIFY_START        (WM_USER+1)
#define WM_SIMPLIFY_END          (WM_USER+2)

// 块平移状态
#define TRANSLATE_SET_TARGET_POINT     1        // 正处于设置目标点状态

// 块旋转状态
#define ROTATE_SET_REF_POINT           1        // 正处于设置旋转参考点状态
#define ROTATE_SET_TARGET_POINT        2        // 正处于设置旋转目标点状态

#define _RGB(r,g,b)          ((COLORREF)(((BYTE)(r)|((WORD)((BYTE)(g))<<8))|(((DWORD)(BYTE)(b))<<16)))

int nTimerCount = 0;

CPnt ptCenter;

CSlamDataSet SlamDataSet;

extern int nCurSlamStep;      // SLAM当前步数
extern int nCurViewStep;

CSlamDataSet DatasetToMerge;
Eigen::Affine3d odometry = PostureToAffine(0, 0, 0);
extern CSlamDataSet TestDataSet;     // 用于进行定位测试的数据集

bool bRefresh = false;

#undef max
#undef min

#include "ndt_fuser\MapFuser.h"

extern CMapFuser* pMapFuser;
CWorld World;
bool   bWorldLoaded = false;
LOGFONT LogFontNodeText;

extern CDatasetLocalization* pLocalization;

CWorldPointDlg* pWorldPointDlg = NULL;

///////////////////////////////////////////////////////////////////////////////

// CRoboMappingView

IMPLEMENT_DYNCREATE(CRoboMappingView, CScrollView)

BEGIN_MESSAGE_MAP(CRoboMappingView, CScrollView)
	ON_WM_CONTEXTMENU()
	ON_WM_RBUTTONUP()
	ON_WM_TIMER()
	ON_COMMAND(ID_MAGNIFY, &CRoboMappingView::OnZoomIn)
	ON_COMMAND(ID_REDUCE, &CRoboMappingView::OnZoomOut)
	ON_WM_CHAR()
	ON_WM_MOUSEMOVE()
	ON_COMMAND(ID_SHOW_ID, &CRoboMappingView::OnShowId)
	ON_COMMAND(ID_MAGNIFY, &CRoboMappingView::OnMagnify)
	ON_COMMAND(ID_REDUCE, &CRoboMappingView::OnReduce)
	ON_COMMAND(ID_AUTO_FIT, &CRoboMappingView::OnBestFit)
	ON_WM_LBUTTONDOWN()
	ON_WM_LBUTTONUP()
	ON_COMMAND(ID_LOAD_MAP, &CRoboMappingView::OnLoadMap)
	ON_COMMAND(ID_SAVE_MAP2, &CRoboMappingView::OnSaveMap)
	ON_COMMAND(ID_DEL_SINGLE, &CRoboMappingView::OnDelSingle)
	ON_COMMAND(ID_TRANSFORM, &CRoboMappingView::OnRotation)
	ON_WM_SIZE()
	ON_WM_RBUTTONDOWN()
	ON_WM_SETCURSOR()
	ON_COMMAND(ID_DEL_RECT, &CRoboMappingView::OnDelRect)
	ON_WM_MOUSEWHEEL()
	ON_COMMAND(ID_TRANSLATION, &CRoboMappingView::OnTranslation)
	ON_UPDATE_COMMAND_UI(ID_SHOW_ID, &CRoboMappingView::OnUpdateShowId)
	ON_COMMAND(ID_MERGE, &CRoboMappingView::OnImportMergeFile)
	ON_COMMAND(ID_PATCH_TRANSLATE, &CRoboMappingView::OnPatchTranslate)
	ON_COMMAND(ID_PATCH_ROTATE, &CRoboMappingView::OnPatchRotate)
	ON_COMMAND(ID_ACCEPT_MERGE, &CRoboMappingView::OnAcceptMerge)
	ON_COMMAND(ID_CANCEL_MERGE, &CRoboMappingView::OnCancelMerge)
	ON_COMMAND(ID_SHOW_REFLECTORS, &CRoboMappingView::OnShowSourceMatched)
	ON_UPDATE_COMMAND_UI(ID_SHOW_REFLECTORS, &CRoboMappingView::OnUpdateShowSourceMatched)
	ON_UPDATE_COMMAND_UI(ID_PATCH_TRANSLATE, &CRoboMappingView::OnUpdatePatchTranslate)
	ON_UPDATE_COMMAND_UI(ID_MERGE, &CRoboMappingView::OnUpdateMerge)
	ON_UPDATE_COMMAND_UI(ID_PATCH_ROTATE, &CRoboMappingView::OnUpdatePatchRotate)
	ON_UPDATE_COMMAND_UI(ID_ACCEPT_MERGE, &CRoboMappingView::OnUpdateAcceptMerge)
	ON_UPDATE_COMMAND_UI(ID_CANCEL_MERGE, &CRoboMappingView::OnUpdateCancelMerge)
	ON_WM_LBUTTONDBLCLK()
	ON_WM_KEYDOWN()
	ON_WM_KEYUP()
	ON_WM_MBUTTONDOWN()
	ON_WM_MBUTTONUP()
	ON_COMMAND(ID_PATCH_FIT, &CRoboMappingView::OnPatchFit)
	ON_COMMAND(ID_LINK_DATASET, &CRoboMappingView::OnLinkDataset)
	ON_COMMAND(ID_TARGET_MATCHED, &CRoboMappingView::OnTargetMatched)
	ON_UPDATE_COMMAND_UI(ID_TARGET_MATCHED, &CRoboMappingView::OnUpdateTargetMatched)
	ON_COMMAND(ID_LOC_INIT_POSTURE_TRANS, &CRoboMappingView::OnLocInitPostureTrans)
	ON_COMMAND(ID_LOC_INIT_POSTURE_ROT, &CRoboMappingView::OnLocInitPostureRot)
	ON_COMMAND(ID_BATCH_LOCALIZE, &CRoboMappingView::OnBatchLocate)
	ON_COMMAND(ID_LOCATE_DATASET, &CRoboMappingView::OnLoadLocateDataset)
	ON_COMMAND(ID_PREPARE_LOC, &CRoboMappingView::OnSetLocateInitPosture)
	ON_COMMAND(ID_LOCALIZE, &CRoboMappingView::OnStepLocate)
	ON_UPDATE_COMMAND_UI(ID_LOCALIZE, &CRoboMappingView::OnUpdateStepLocate)
	ON_UPDATE_COMMAND_UI(ID_BATCH_LOCALIZE, &CRoboMappingView::OnUpdateBatchLocate)
	ON_UPDATE_COMMAND_UI(ID_LOC_INIT_POSTURE_TRANS, &CRoboMappingView::OnUpdateLocInitPostureTrans)
	ON_UPDATE_COMMAND_UI(ID_LOC_INIT_POSTURE_ROT, &CRoboMappingView::OnUpdateLocInitPostureRot)
	ON_UPDATE_COMMAND_UI(ID_PREPARE_LOC, &CRoboMappingView::OnUpdatePrepareLoc)
	ON_COMMAND(ID_END_LOCALIZE, &CRoboMappingView::OnEndLocalize)
	ON_UPDATE_COMMAND_UI(ID_END_LOCALIZE, &CRoboMappingView::OnUpdateEndLocalize)
	ON_COMMAND(ID_MODEL_SHOW_HIGH_INTENSITY, &CRoboMappingView::OnShowHighIntensity)
	ON_UPDATE_COMMAND_UI(ID_MODEL_SHOW_HIGH_INTENSITY, &CRoboMappingView::OnUpdateShowHighIntensity)
	ON_COMMAND(ID_MODEL_SHOW_REFLECTORS, &CRoboMappingView::OnShowReflectors)
	ON_UPDATE_COMMAND_UI(ID_MODEL_SHOW_REFLECTORS, &CRoboMappingView::OnUpdateShowReflectors)
	ON_COMMAND(ID_ADD_REFLECTOR, &CRoboMappingView::OnAddReflector)
	ON_COMMAND(ID_REMOVE_REFLECTOR, &CRoboMappingView::OnDelReflector)
	ON_COMMAND(ID_IMPORT_ROUTE_MAP, &CRoboMappingView::OnImportRouteMap)
	ON_UPDATE_COMMAND_UI(ID_TRANSLATION, &CRoboMappingView::OnUpdateTranslation)
	ON_UPDATE_COMMAND_UI(ID_TRANSFORM, &CRoboMappingView::OnUpdateRotation)
END_MESSAGE_MAP()

// CRoboMappingView 构造/析构

CRoboMappingView::CRoboMappingView()
{
	// TODO: 在此处添加构造代码
	Reset();
}

CRoboMappingView::~CRoboMappingView()
{
}

void CRoboMappingView::Reset()
{
	m_bShowId = false;
	m_nWorkMode = MODE_CREATE_MODEL;
	m_nWorkSubType = 0;
	m_nWorkStage = 0;
	m_bBestFitMode = true;
	m_bStartRect = false;
	m_bShowActiveSide = false;
	m_bShowSourceMatched = true;
	m_bShowTargetMatched = false;
	m_bShowHighIntensity = false;
	m_bShowReflectors = true;

	m_pWaitBusyDlg = NULL;
	m_bMouseNotMove = false;
	m_bControlKeyDown = false;
	m_nTestStep = -1;
	m_bFirstDraw = false;

	m_bScrollStart = false;
	m_nTestTargetStep = -1;
	m_nAutoTestCount = 0;
}

BOOL CRoboMappingView::PreCreateWindow(CREATESTRUCT& cs)
{
	// TODO: 在此处通过修改
	//  CREATESTRUCT cs 来修改窗口类或样式

	return CScrollView::PreCreateWindow(cs);
}

// CRoboMappingView 绘制

//
//   处理在“MODE_CREATE_MODEL”模式下的窗口绘制。
//
void CRoboMappingView::OnDrawCreateModel(CDC* pDC)
{
	CRoboMappingDoc* pDoc = GetDocument();
	CScreenReference& ScrnRef = pDoc->m_ScrnRef;

	if (pMapFuser == NULL)
		return;

	pMapFuser->m_crit.Lock();

	int nCurStep = pMapFuser->m_nCurBuildStep;
	if (nCurStep >= 0)
		pMapFuser->matcher2D.ShowStatus(pDC, ScrnRef, nCurStep + 1, pMapFuser->m_pDataset->size(), _RGB(0, 0, 0));

	// 显示地图及机器人位姿曲线
	if (pMapFuser->map != NULL)
	{
		pMapFuser->PlotLocalization(pDC, ScrnRef, m_bShowSourceMatched, m_bShowTargetMatched, true);

		// 显示高亮点集合
		if (m_bShowHighIntensity)
			pMapFuser->highIntensityPoints.Plot(ScrnRef, pDC, _RGB(255, 150, 200), 0, 3);

		// 显示反光板集合
		if (m_bShowReflectors)
			pMapFuser->refPoints.Plot(ScrnRef, pDC, _RGB(255, 0, 0), 0, 0);
	}

	pMapFuser->m_crit.Unlock();
}

//
//   处理在“MODE_EDIT_MODEL”模式下的窗口绘制。
//
void CRoboMappingView::OnDrawEditModel(CDC* pDC)
{
	CRoboMappingDoc* pDoc = GetDocument();
	CScreenReference& ScrnRef = pDoc->m_ScrnRef;

	// 显示地图及机器人位姿曲线
	if (pMapFuser->map != NULL)
	{
		// 显示地图
		pMapFuser->PlotModelMap(pDC, ScrnRef, _RGB(0, 128, 192), _RGB(0, 0, 0), m_bShowTargetMatched, _RGB(255, 0, 0));
	}

	switch (m_nWorkSubType)
	{
	case EDIT_SUBTYPE_ADD_REFLECTOR:
	case EDIT_SUBTYPE_DEL_REFLECTOR:
		// 显示高亮点集合
		if (m_bShowHighIntensity)
			pMapFuser->highIntensityPoints.Plot(ScrnRef, pDC, _RGB(255, 150, 200), 0, 3);

		// 显示反光板集合
		if (m_bShowReflectors)
			pMapFuser->refPoints.Plot(ScrnRef, pDC, _RGB(255, 0, 0), 0, 0);

		break;

	case EDIT_SUBTYPE_TRANSLATE:
		if (m_nWorkStage == TRANSLATE_SET_TARGET_POINT)
		{
			CLine ln(m_ptFrom, m_ptCur);
			ln.Draw(ScrnRef, pDC, _RGB(128, 128, 128), 1, 1, true, PS_DOT);
		}
		break;

	case EDIT_SUBTYPE_ROTATE:
		if (m_nWorkStage == ROTATE_SET_REF_POINT)
		{
			if (m_ptRotateCenter.DistanceTo(m_ptCur) > 0.001f)
			{
				CLine ln(m_ptRotateCenter, m_ptCur);
				ln.Draw(ScrnRef, pDC, _RGB(128, 128, 128), 1, 1, true, PS_DOT);
			}
		}
		else if (m_nWorkStage == ROTATE_SET_TARGET_POINT)
		{
			if (m_ptRotateCenter.DistanceTo(m_ptFrom) > 0.001f && 
				 m_ptRotateCenter.DistanceTo(m_ptCur) > 0.001f)
			{
				CLine ln1(m_ptRotateCenter, m_ptFrom);
				ln1.Draw(ScrnRef, pDC, _RGB(128, 128, 128), 1, 1, true, PS_DOT);

				CLine ln2(m_ptRotateCenter, m_ptCur);
				ln2.Draw(ScrnRef, pDC, _RGB(128, 128, 128), 1, 1, true, PS_DOT);

				CTurnDir TurnDir = COUNTER_CLOCKWISE;
				CAngle ang = ln2.SlantAngle() - ln1.SlantAngle();
				int q = ang.Quadrant();
				if (q == 3 || q == 4)
					TurnDir = CLOCKWISE;

				CArc arc1(m_ptRotateCenter, ln1.m_ptEnd, ln2.m_ptEnd, TurnDir);
				arc1.Draw(ScrnRef, pDC, _RGB(128, 128, 128), 1, PS_DOT);
			}
		}
		break;
	}
}

//
//   处理在“MODE_EDIT_MODEL”模式下的窗口绘制。
//
void CRoboMappingView::OnDrawMergeModel(CDC* pDC)
{
	CRoboMappingDoc* pDoc = GetDocument();
	CScreenReference& ScrnRef = pDoc->m_ScrnRef;

	// 显示地图
	if (pMapFuser->map != NULL)
		pMapFuser->PlotModelMap(pDC, ScrnRef, _RGB(0, 128, 192), _RGB(0, 0, 0), m_bShowTargetMatched, _RGB(255, 0, 0));

	// 如果待合并块正处于平移状态中，需要绘制当前的平移位置
	if (m_nWorkSubType == MERGE_SUBTYPE_TRANSLATE && m_nWorkStage == TRANSLATE_SET_TARGET_POINT)
	{
		m_MapEdit.Plot(pDC, ScrnRef, _RGB(0, 128 / 2, 192 / 2), _RGB(0, 128 / 2, 192 / 2));
		CLine ln(m_ptFrom, m_ptCur);
		ln.Draw(ScrnRef, pDC, _RGB(128, 128, 128), 1, 1, true, PS_DOT);
	}

	// 如果待合并块正处于旋转状态中，需要绘制当前的旋转位置
	else if (m_nWorkSubType == MERGE_SUBTYPE_ROTATE)
	{
		if (m_nWorkStage == ROTATE_SET_REF_POINT)
		{
			m_MapEdit.Plot(pDC, ScrnRef, _RGB(0, 128 / 2, 192 / 2), _RGB(0, 128 / 2, 192 / 2));
			
			CLine ln(m_ptRotateCenter, m_ptCur);
			ln.Draw(ScrnRef, pDC, _RGB(128, 128, 128), 1, 1, true, PS_DOT);
		}
		else if (m_nWorkStage == ROTATE_SET_TARGET_POINT)
		{
			m_MapEdit.Plot(pDC, ScrnRef, _RGB(0, 128 / 2, 192 / 2), _RGB(0, 128 / 2, 192 / 2));

			CLine ln1(m_ptRotateCenter, m_ptFrom);
			ln1.Draw(ScrnRef, pDC, _RGB(128, 128, 128), 1, 1, true, PS_DOT);

			CLine ln2(m_ptRotateCenter, m_ptCur);
			ln2.Draw(ScrnRef, pDC, _RGB(128, 128, 128), 1, 1, true, PS_DOT);

			CTurnDir TurnDir = COUNTER_CLOCKWISE;
			CAngle ang = ln2.SlantAngle() - ln1.SlantAngle();
			int q = ang.Quadrant();
			if (q == 3 || q == 4)
				TurnDir = CLOCKWISE;

			CArc arc1(m_ptRotateCenter, ln1.m_ptEnd, ln2.m_ptEnd, TurnDir);
			arc1.Draw(ScrnRef, pDC, _RGB(128, 128, 128), 1, PS_DOT);
		}
	}

	m_MapToMerge.Plot(pDC, ScrnRef, _RGB(34, 177, 76), _RGB(34, 177, 76));
}

//
//   处理在“MODE_TEST_LOCATE”模式下的窗口绘制。
//
void CRoboMappingView::OnDrawTestLocate(CDC* pDC)
{
	CRoboMappingDoc* pDoc = GetDocument();
	CScreenReference& ScrnRef = pDoc->m_ScrnRef;

	// 显示地图
	if (pMapFuser->map != NULL)
	{
		// 显示地图
		pMapFuser->PlotModelMap(pDC, ScrnRef, _RGB(0, 128, 192), _RGB(0, 0, 0), m_bShowTargetMatched, _RGB(255, 0, 0));
	}

	// 如果处于设置起始状态的操作中，需要显示对应的扫描点云的状态
	switch (m_nWorkSubType)
	{
	case TEST_SUBTYPE_INIT_POSTURE:
		m_initScan.Plot(ScrnRef, pDC, _RGB(255, 0, 0), 0,true);
		break;

	case TEST_SUBTYPE_INIT_TRANSLATE:
		m_initScan.Plot(ScrnRef, pDC, _RGB(255, 0, 0), 0,true);
		if (m_nWorkStage == TRANSLATE_SET_TARGET_POINT)
		{
			CLine ln(m_ptFrom, m_ptCur);
			ln.Draw(ScrnRef, pDC, _RGB(128, 128, 128), 1, 1, true);
		}
		break;

	case TEST_SUBTYPE_INIT_ROTATE:
		m_initScan.Plot(ScrnRef, pDC, _RGB(255, 0, 0), 0, true);
		if (m_nWorkStage == ROTATE_SET_REF_POINT)
		{
			CLine ln(m_ptRotateCenter, m_ptCur);
			ln.Draw(ScrnRef, pDC, _RGB(128, 128, 128), 1, 1, true);
		}
		else if (m_nWorkStage == ROTATE_SET_TARGET_POINT)
		{
			CLine ln1(m_ptRotateCenter, m_ptFrom);
			CLine ln2(m_ptRotateCenter, m_ptCur);
			ln1.Draw(ScrnRef, pDC, _RGB(128, 128, 128), 1, 1, true);
			ln2.Draw(ScrnRef, pDC, _RGB(128, 128, 128), 1, 1, true);
		}
		break;

	case TEST_SUBTYPE_DIRECT_LOCATE:
	{
		int nCurStep = nCurSlamStep;
		if (nCurStep >= 0)
			pLocalization->matcher2D.ShowStatus(pDC, ScrnRef, nCurStep + 1, TestDataSet.size(), _RGB(0, 0, 0));

		pLocalization->PlotLocalization(pDC, ScrnRef, m_bShowSourceMatched, m_bShowTargetMatched, true);

		// 画出拉偏定位测试姿态
		if (m_nWorkStage == 3 || m_nWorkStage == 4)
			m_pstTest.Draw(ScrnRef, pDC, _RGB(0, 0, 0), 40, 150, 2);

		if (m_nWorkStage == 4)
			m_pstTestResult.Draw(ScrnRef, pDC, _RGB(0, 0, 255), 40, 150, 3);
	}
		break;
	}
}

void CRoboMappingView::OnDraw(CDC* pDC)
{
	CRoboMappingDoc* pDoc = GetDocument();
	CScreenReference& ScrnRef = pDoc->m_ScrnRef;

	ASSERT_VALID(pDoc);
	if (!pDoc)
		return;

	CRect rect;
	GetClientRect(&rect);
	CPoint scrPos = this->GetScrollPosition();

	// 画布的新尺寸
	int nWidthCanvas = rect.right;
	int nHeigthCanvas = rect.bottom;

	CDC memoryDC;
	memoryDC.CreateCompatibleDC(pDC);

	CBitmap memoryBitmap;
	memoryBitmap.CreateCompatibleBitmap(pDC, nWidthCanvas, nHeigthCanvas);

	// 背景色为白色
	CBitmap* pOldBitmap = memoryDC.SelectObject(&memoryBitmap);
	memoryDC.FillSolidRect(0, 0, nWidthCanvas, nHeigthCanvas, _RGB(255, 255, 255));

	// 绘制网格
	
	for (int i = -200; i < 200; i++)
	{
		CLine lnHoriz(CPnt(-200, i), CPnt(200, i));
		lnHoriz.Draw(ScrnRef, &memoryDC, _RGB(200, 200, 200));

		CLine lnVert(CPnt(i, -200), CPnt(i, 200));
		lnVert.Draw(ScrnRef, &memoryDC, _RGB(200, 200, 200));
	}

	if (bWorldLoaded)
		World.Draw(ScrnRef, &memoryDC, &LogFontNodeText, _RGB(128, 128, 255), _RGB(128, 128, 255));

	switch (m_nWorkMode)
	{
	case MODE_CREATE_MODEL:
		OnDrawCreateModel(&memoryDC);
		break;

	case MODE_EDIT_MODEL:
		OnDrawEditModel(&memoryDC);
		break;

	case MODE_MERGE_MODEL:
		OnDrawMergeModel(&memoryDC);
		break;

	case MODE_TEST_LOCATE:
		OnDrawTestLocate(&memoryDC);
		break;
	}

	// 将内存位图绘制结果转贴到屏幕窗口
	pDC->BitBlt(scrPos.x, scrPos.y, rect.right + scrPos.x, rect.bottom + scrPos.y, &memoryDC, 0, 0, SRCCOPY);
	memoryBitmap.DeleteObject();
	memoryDC.DeleteDC();
}

void CRoboMappingView::OnRButtonUp(UINT /* nFlags */, CPoint point)
{
}

void CRoboMappingView::OnContextMenu(CWnd* /* pWnd */, CPoint point)
{
#ifndef SHARED_HANDLERS
	theApp.GetContextMenuManager()->ShowPopupMenu(IDR_POPUP_EDIT, point.x, point.y, this, TRUE);
#endif
}


// CRoboMappingView 诊断

#ifdef _DEBUG
void CRoboMappingView::AssertValid() const
{
	CScrollView::AssertValid();
}

void CRoboMappingView::Dump(CDumpContext& dc) const
{
	CScrollView::Dump(dc);
}

CRoboMappingDoc* CRoboMappingView::GetDocument() const // 非调试版本是内联的
{
	ASSERT(m_pDocument->IsKindOf(RUNTIME_CLASS(CRoboMappingDoc)));
	return (CRoboMappingDoc*)m_pDocument;
}
#endif //_DEBUG


// CRoboMappingView 消息处理程序

void CRoboMappingView::AutoFitView()
{
	if (m_bBestFitMode)
		ScaleToFitView();
}

void CRoboMappingView::ScaleToFitView()
{
	CRoboMappingDoc* pDoc = GetDocument();
	CScreenReference& ScrnRef = pDoc->m_ScrnRef;

	CRect r;
	GetClientRect(r);
	if (r.Width() == 0)
		return;

	// Set the view port
	ScrnRef.SetViewPort(USHORT(r.Width()), USHORT(r.Height()));

	CRectangle rect;
	if (pMapFuser != NULL && pMapFuser->map != NULL)
		rect = pMapFuser->map->GetCoveringRect();
	
	ptCenter = rect.GetCenterPoint();

	m_fWorldWidth = rect.Width();
	m_fWorldWidth += 2 * MARGIN_DIST;

	m_fWorldHeight = rect.Height();
	m_fWorldHeight += 2 * MARGIN_DIST;

	// Caculate the display ratio
	r.InflateRect(-MARGIN_SIZE, -MARGIN_SIZE);
	float fWidthRatio = r.Width() / m_fWorldHeight;
	float fHeightRatio = r.Height() / m_fWorldHeight;
	ScrnRef.SetRatio(min(fWidthRatio, fHeightRatio));

	// Set the center point
	ScrnRef.SetCenterPoint(ptCenter);

	m_sizeTotalScroll.cx = (LONG)(m_fWorldWidth * ScrnRef.m_fRatio);
	m_sizeTotalScroll.cy = (LONG)(m_fWorldHeight * ScrnRef.m_fRatio);

	SetScrollSizes(MM_TEXT, m_sizeTotalScroll);
}

void CRoboMappingView::OnTimer(UINT_PTR nIDEvent)
{
	// TODO:  在此添加消息处理程序代码和/或调用默认值
	if (nIDEvent == 1)
	{
		// 步进式地处理得到的Slam数据
		if (bRefresh)
		{
			bRefresh = false;

			if (m_bBestFitMode)
				ScaleToFitView();
			Invalidate();
		}
	}
	else if (nIDEvent == 2)
	{
		if (nCurSlamStep < m_nTestTargetStep)
		{
			DoStepLocate();

			if (m_nAutoTestCount++ % 5 == 0)
				Invalidate();
		}
		else
			KillTimer(2);
	}

	CScrollView::OnTimer(nIDEvent);
}


void CRoboMappingView::OnInitialUpdate()
{
	((CRoboMappingApp*)AfxGetApp())->m_pMappingView = this;

	CScrollView::OnInitialUpdate();

	GetParent()->SetWindowText(_T("模型窗口"));

	m_hCursorDel = LoadCursor(AfxGetInstanceHandle(), MAKEINTRESOURCE(IDC_DEL));
	m_hCursorDelThis = LoadCursor(AfxGetInstanceHandle(), MAKEINTRESOURCE(IDC_DEL_THIS));
	m_hCursorDelRect = LoadCursor(AfxGetInstanceHandle(), MAKEINTRESOURCE(IDC_DEL_RECT));
	m_hCursorAddReflector = LoadCursor(AfxGetInstanceHandle(), MAKEINTRESOURCE(IDC_SEL_RECT_W));
	m_hCursorReflector2 = LoadCursor(AfxGetInstanceHandle(), MAKEINTRESOURCE(IDC_REFLECTOR2));
	m_hCursorReflectorMove = LoadCursor(AfxGetInstanceHandle(), MAKEINTRESOURCE(IDC_REFLECTOR_MOVE));
	m_hCursorTranslation = LoadCursor(AfxGetInstanceHandle(), MAKEINTRESOURCE(IDC_TRANSLATION));
	m_hCursorTranslation2 = LoadCursor(AfxGetInstanceHandle(), MAKEINTRESOURCE(IDC_TRANSLATION2));
	m_hCursorRotate1 = LoadCursor(AfxGetInstanceHandle(), MAKEINTRESOURCE(IDC_ROTATE1));
	m_hCursorRotate2 = LoadCursor(AfxGetInstanceHandle(), MAKEINTRESOURCE(IDC_ROTATE2));
	m_hCursorRotate3 = LoadCursor(AfxGetInstanceHandle(), MAKEINTRESOURCE(IDC_ROTATE3));

	EnableToolTips(TRUE);

	m_TipCtrl.Create(this, TTS_ALWAYSTIP);
	m_TipCtrl.AddTool(this);

	CRoboMappingDoc* pDoc = GetDocument();
	CScreenReference& ScrnRef = pDoc->m_ScrnRef;

	CScreenReference s1(1024, 800, 28.2f, CPnt(0, 0));

	ScrnRef = s1;


	// TODO:  在此添加专用代码和/或调用基类
	SetTimer(1, 500, NULL);
}

void CRoboMappingView::OnZoomIn()
{
	CRoboMappingDoc* pDoc = GetDocument();
	CScreenReference& ScrnRef = pDoc->m_ScrnRef;

	// TODO: 在此添加命令处理程序代码
	ScrnRef.m_fRatio *= 1.5f;
	Invalidate();
}

void CRoboMappingView::OnZoomOut()
{
	CRoboMappingDoc* pDoc = GetDocument();
	CScreenReference& ScrnRef = pDoc->m_ScrnRef;

	// TODO: 在此添加命令处理程序代码
	ScrnRef.m_fRatio /= 1.5f;
	Invalidate();
}

void CRoboMappingView::OnChar(UINT nChar, UINT nRepCnt, UINT nFlags)
{
	// TODO: 在此添加消息处理程序代码和/或调用默认值
	switch (nChar)
	{
	case ((UINT)'='):
		OnZoomIn();
		break;

	case ((UINT)'-'):
		OnZoomOut();
		break;
	}

	CScrollView::OnChar(nChar, nRepCnt, nFlags);
}

//
//   判断一个屏幕点是否落到一个反光板附近。
//
int CRoboMappingView::PointHitReflector(CPoint& point, CPointFeatureSet* pPointFeatureSet)
{
	CRoboMappingDoc* pDoc = GetDocument();
	CScreenReference& ScrnRef = pDoc->m_ScrnRef;

	int nCount = pPointFeatureSet->GetCount();
	for (int i = 0; i < nCount; i++)
	{
		CPointFeature* pFeature = pPointFeatureSet->at(i);
		CPoint pnt1 = ScrnRef.GetWindowPoint(pFeature->GetPntObject());
		if (abs(pnt1.x - point.x) < 3 && abs(pnt1.y - point.y) < 3)
		{
			return i;
		}
	}

	return -1;
}

void CRoboMappingView::RefreshModelStatus(CString str1, CString str2, CString str3)
{
	CMainFrame* pMain = (CMainFrame*)AfxGetApp()->m_pMainWnd;

	CString str = str1 + _T("         ") + str2 + _T("          ") + str3;
	CMFCRibbonBaseElement * pElement = (CMFCRibbonStatusBarPane*)pMain->m_wndStatusBar.FindElement(ID_STATUSBAR_PANE1);
	pElement->SetText(str);
	pElement->Redraw();
}

//
//   正处于“模型生成”模式下的“OnMouseMove”响应。
//
void CRoboMappingView::OnMouseMoveCreateModel(CPoint point)
{
	::SetCursor(LoadCursor(NULL, IDC_ARROW));
}

//
//   正处于“模型编辑”模式下的“OnMouseMove”响应。
//
void CRoboMappingView::OnMouseMoveEditModel(CPoint point)
{
	CRoboMappingDoc* pDoc = GetDocument();
	CScreenReference& ScrnRef = pDoc->m_ScrnRef;

	CPoint ScrollPos = GetScrollPosition();
	point += ScrollPos;

	// 取得当前点所对应的世界坐标
	CPnt pt = ScrnRef.GetWorldPoint(point);

	// 设置光标状态
	switch (m_nWorkSubType)
	{
	case EDIT_SUBTYPE_DEL_OBJ:
		::SetCursor(m_hCursorDel);
		break;

	case EDIT_SUBTYPE_DEL_RECT:
	case EDIT_SUBTYPE_ADD_REFLECTOR:
	case EDIT_SUBTYPE_DEL_REFLECTOR:
		{
		CDC* pDC = GetDC();

		//SetRop2 Specifies the new drawing mode.(MSDN)
		//R2_NOT   Pixel is the inverse of the screen color.(MSDN)
		//即：该函数用来定义绘制的颜色，而该参数则将颜色设置为原屏幕颜色的反色
		//这样，如果连续绘制两次的话，就可以恢复原来屏幕的颜色了（如下）
		//但是，这里的连续两次绘制却不是在一次消息响应中完成的
		//而是在第一次拖动响应的绘制可以显示（也就是看到的），第二次拖动绘制实现擦出（也就看不到了）
		pDC->SetROP2(R2_NOT);   //此为关键!!!
		pDC->SelectStockObject(NULL_BRUSH); //不使用画刷

		if (m_bStartRect)   //根据是否有单击判断是否可以画矩形
		{
			CPoint pt1 = m_RectSelector.Get1stPoint() - ScrollPos;
			CPoint pt2 = m_RectSelector.Get2ndPoint() - ScrollPos;
			CPoint ptOld = m_OldPoint - ScrollPos;
			CPoint ptNew = point - ScrollPos;

			pDC->Rectangle(CRect(pt1, ptOld));
			pDC->Rectangle(CRect(pt1, ptNew));
			m_OldPoint = point;
		}

		ReleaseDC(pDC);
		if (m_nWorkSubType == EDIT_SUBTYPE_ADD_REFLECTOR)
			::SetCursor(m_hCursorAddReflector);
		else
			::SetCursor(m_hCursorDelRect);
		}
		break;

	case EDIT_SUBTYPE_TRANSLATE:
		if (m_nWorkStage == 0)
			::SetCursor(m_hCursorTranslation);
		else if (m_nWorkStage == TRANSLATE_SET_TARGET_POINT)
		{
			m_TipCtrl.UpdateTipText(_T("定义平移落点"), this);
			CPnt ptMove = m_ptCur - m_ptFrom;
			CString str;
			str.Format(_T(",   平移距离: (%.3f, %.3f)"), ptMove.x, ptMove.y);
			m_strStatus2 += str;
			::SetCursor(m_hCursorTranslation2);
			Invalidate();
		}
		break;

	case EDIT_SUBTYPE_ROTATE:
		if (m_nWorkStage == 0)
			::SetCursor(m_hCursorRotate1);
		else if (m_nWorkStage == ROTATE_SET_REF_POINT)
		{
			::SetCursor(m_hCursorRotate2);

			// 需要判断直线是否过短
			if (m_ptRotateCenter.DistanceTo(m_ptCur) > 0.01f)
			{
				m_TipCtrl.UpdateTipText(_T("定义旋转参考点"), this);
				Invalidate();
			}
		}
		else if (m_nWorkStage == ROTATE_SET_TARGET_POINT)
		{
			::SetCursor(m_hCursorRotate3);

			// 需要判断直线是否过短
			if (m_ptRotateCenter.DistanceTo(m_ptCur) > 0.01f && 
				 m_ptRotateCenter.DistanceTo(m_ptFrom) > 0.01f)
			{
				m_TipCtrl.UpdateTipText(_T("定义旋转落点"), this);
				float fLen = m_ptRotateCenter.DistanceTo(m_ptFrom);
				
				CLine ln1(m_ptRotateCenter, m_ptFrom);
				CLine ln2(m_ptRotateCenter, m_ptCur);
				m_ptCur = ln2.TrajFun(fLen);

				// 刷新状态栏中的“旋转角度”数据
				CAngle ang = ln2.SlantAngle() - ln1.SlantAngle();

				CString str;
				str.Format(_T(",   旋转角度: %.1f度"), ang.Degree2());
				m_strStatus2 += str;

				Invalidate();
			}
		}
		break;
	}
}

//
//   正处于合并拼接块的模式下的“OnMouseMove”响应。
//
void CRoboMappingView::OnMouseMoveMergeModel(CPoint point)
{
	CRoboMappingDoc* pDoc = GetDocument();
	CScreenReference& ScrnRef = pDoc->m_ScrnRef;

	CPoint ScrollPos = GetScrollPosition();
	point += ScrollPos;

	// 取得当前点所对应的世界坐标
	CPnt pt = ScrnRef.GetWorldPoint(point);

	// 如果正处于平移过程中
	switch (m_nWorkSubType)
	{
	case MERGE_SUBTYPE_TRANSLATE:
		if (m_nWorkStage == 0)
			::SetCursor(m_hCursorTranslation);
		else if (m_nWorkStage == TRANSLATE_SET_TARGET_POINT)
		{
			m_TipCtrl.UpdateTipText(_T("定义平移落点"), this);
			CPnt ptMove = m_ptCur - m_ptFrom;

			CString str;
			str.Format(_T(",   平移距离: (%.3f, %.3f)"), ptMove.x, ptMove.y);
			m_strStatus2 += str;

			::SetCursor(m_hCursorTranslation2);
			Invalidate();
		}
		break;

	case MERGE_SUBTYPE_ROTATE:
		if (m_nWorkStage == 0)
			::SetCursor(m_hCursorRotate1);
		else if (m_nWorkStage == ROTATE_SET_REF_POINT)
		{
			::SetCursor(m_hCursorRotate2);

			// 需要判断直线是否过短
			if (m_ptRotateCenter.DistanceTo(m_ptCur) > 0.01f)
			{
				m_TipCtrl.UpdateTipText(_T("定义旋转参考点"), this);
				Invalidate();
			}
		}
		else if (m_nWorkStage == ROTATE_SET_TARGET_POINT)
		{
			::SetCursor(m_hCursorRotate3);

			if (m_ptRotateCenter.DistanceTo(m_ptCur) > 0.01f && m_ptRotateCenter.DistanceTo(m_ptFrom) > 0.01f)
			{
				m_TipCtrl.UpdateTipText(_T("定义旋转落点"), this);
				m_MapEdit = m_MapToMerge;
				float fLen = m_ptRotateCenter.DistanceTo(m_ptFrom);

				CLine ln1(m_ptRotateCenter, m_ptFrom);
				CLine ln2(m_ptRotateCenter, m_ptCur);
				m_ptCur = ln2.TrajFun(fLen);

				// 刷新状态栏中的“旋转角度”数据
				CAngle ang = ln2.SlantAngle() - ln1.SlantAngle();

				CString str;
				str.Format(_T(",   旋转角度: %.1f度"), ang.Degree2());
				m_strStatus2 += str;

				Invalidate();
			}
		}		
		break;
	}
}

void CRoboMappingView::OnMouseMoveTestLocate(CPoint point)
{
	CRoboMappingDoc* pDoc = GetDocument();
	CScreenReference& ScrnRef = pDoc->m_ScrnRef;

		// 如果正处于平移过程中
	switch (m_nWorkSubType)
	{
	case TEST_SUBTYPE_INIT_TRANSLATE:
		if (m_nWorkStage == 0)
			::SetCursor(m_hCursorTranslation);
		else if (m_nWorkStage == TRANSLATE_SET_TARGET_POINT)
		{
			m_TipCtrl.UpdateTipText(_T("定义平移落点"), this);
			CPnt ptMove = m_ptCur - m_ptFrom;
			::SetCursor(m_hCursorTranslation2);
			Invalidate();
		}
		break;

	case TEST_SUBTYPE_INIT_ROTATE:
		if (m_nWorkStage == 0)
			::SetCursor(m_hCursorRotate1);
		else if (m_nWorkStage == ROTATE_SET_REF_POINT)
		{
			::SetCursor(m_hCursorRotate2);
			Invalidate();
		}
		else if (m_nWorkStage == ROTATE_SET_TARGET_POINT)
		{
			::SetCursor(m_hCursorRotate3);
			if (m_ptRotateCenter.DistanceTo(m_ptCur) > 0.01f && m_ptRotateCenter.DistanceTo(m_ptFrom) > 0.01f)
			{
				m_TipCtrl.UpdateTipText(_T("定义旋转落点"), this);
				m_MapEdit = m_MapToMerge;
				CLine ln1(m_ptRotateCenter, m_ptCur);
				CLine ln2(m_ptRotateCenter, m_ptFrom);
				CAngle ang = ln1.SlantAngle() - ln2.SlantAngle();
				Invalidate();
			}
		}
		break;

	case TEST_SUBTYPE_DIRECT_LOCATE:
	{
		CDC *pDC = GetDC();
		pDC->SetROP2(R2_NOT);

		// 先擦出原有姿态
		if (!m_bFirstDraw)
			m_pstMove.Draw(ScrnRef, pDC, _RGB(0, 0, 0), 40, 150, 2);
		m_bFirstDraw = false;

		// 如果处于设置测试点状态
		if (m_nWorkStage == 1)
		{
			m_pstTest.SetPnt(m_ptCur);
		}

		// 如果处于设置方向角状态
		else if (m_nWorkStage == 2 && m_pstTest.DistanceTo(m_ptCur) > 1E-5)
		{
			CLine ln(m_pstTest, m_ptCur);
			m_pstTest.SetAngle(ln.SlantAngle());
		}

		m_pstTest.Draw(ScrnRef, pDC, _RGB(0, 0, 0), 40, 150, 2);
		m_pstMove = m_pstTest;

		pDC->SetROP2(R2_COPYPEN);
		ReleaseDC(pDC);
	}
		break;
	}
}

void CRoboMappingView::OnMouseMove(UINT nFlags, CPoint point)
{
	if (m_bMouseNotMove)
	{
		m_bMouseNotMove = false;
		return;
	}

	// TODO: 在此添加消息处理程序代码和/或调用默认值
	CMainFrame* pMain = (CMainFrame*)AfxGetApp()->m_pMainWnd;

	CRoboMappingDoc* pDoc = GetDocument();
	CScreenReference& ScrnRef = pDoc->m_ScrnRef;

	CPoint ScrollPos = GetScrollPosition();
	point += ScrollPos;

	// 判断鼠标是否触碰到某个反光板特征(鼠标精度为3个像素)
//	CString str1, str2, str3, str4;
	int nRefId;

	int nCellCount = 0;

	if (pMapFuser->map != NULL)
	{
		pMapFuser->m_crit.Lock();
		nCellCount = pMapFuser->map->numberOfActiveCells();
		pMapFuser->m_crit.Unlock();
	}

	m_strStatus1.Format(_T("单元数：%d"), nCellCount);

	// 计算当前鼠标位置
	m_ptCur = ScrnRef.GetWorldPoint(point);
	m_strStatus2.Format(_T("当前:(%.3f, %.3f)"), m_ptCur.x, m_ptCur.y);

	switch (m_nWorkMode)
	{
	case MODE_CREATE_MODEL:
		OnMouseMoveCreateModel(point);
		break;

	case MODE_EDIT_MODEL:
		OnMouseMoveEditModel(point);
		break;

	case MODE_MERGE_MODEL:
		OnMouseMoveMergeModel(point);
		break;

	case MODE_TEST_LOCATE:
		OnMouseMoveTestLocate(point);
		break;
	}

	if (pMapFuser->map != NULL && !pMapFuser->m_bRunning)
	{
		int nHitCell = pMapFuser->map->PointHitCell(m_ptCur, 3 / ScrnRef.m_fRatio);
		int nHitPose = pMapFuser->correctedPoses.PointHit(m_ptCur, 6 / ScrnRef.m_fRatio);

		if (nHitCell >= 0)
		{
			perception_oru::NDTCell* pCell = pMapFuser->map->GetCell(nHitCell);
			double mx = pCell->getMean()(0);
			double my = pCell->getMean()(1);
			double c[3][3] =
			{ 
				{pCell->getCov()(0, 0), pCell->getCov()(0, 1), pCell->getCov()(0, 2)},
				{pCell->getCov()(1, 0), pCell->getCov()(1, 1), pCell->getCov()(1, 2)},
				{pCell->getCov()(2, 0), pCell->getCov()(2, 1), pCell->getCov()(2, 2)}
			};
			for (int i = 0; i < 3; i++)
				for (int j = 0; j < 3; j++)
					c[i][j] *= 1000;

			m_strStatus3.Format(_T("Cell#%d"), nHitCell);
			CString strDetail;
			strDetail.Format(_T("(%.3f, %.3f) - %.3f %.3f %.3f, %.3f %.3f %.3f, %.3f %.3f %.3f"), mx, my,
				c[0][0], c[0][1], c[0][2],
				c[1][0], c[1][1], c[1][2],
				c[2][0], c[2][1], c[2][2]);
			m_strStatus3 += strDetail;
		}

		// 如果触碰到某个位姿
		else if (nHitPose >= 0)
		{
			CPosture pst = pMapFuser->correctedPoses.GetPosture(nHitPose);
			m_strStatus3.Format(_T("Pose#%d:(%.3f, %.3f, %.1f)"), nHitPose+1, pst.x, pst.y, CAngle::ToDegree(pst.fThita));
		}
	}
	RefreshModelStatus(m_strStatus1, m_strStatus2, m_strStatus3);
	
	CScrollView::OnMouseMove(nFlags, point);
}

void CRoboMappingView::OnUpdate(CView* /*pSender*/, LPARAM /*lHint*/, CObject* /*pHint*/)
{
	// TODO: 在此添加专用代码和/或调用基类
	CRoboMappingDoc* pDoc = GetDocument();
	CScreenReference& ScrnRef = pDoc->m_ScrnRef;

	CRectangle rect;

	float fLeftMost = rect.Left() - MARGIN_DIST;
	float fTopMost = rect.Top() + MARGIN_DIST;
	CPnt pt(fLeftMost, fTopMost);
	ScrnRef.SetLeftTopPoint(pt);

	CSize sizeTotal;
	sizeTotal.cx = (LONG)((rect.Width() + 2 * MARGIN_DIST)* ScrnRef.m_fRatio);
	sizeTotal.cy = (LONG)((rect.Height() + 2 * MARGIN_DIST)* ScrnRef.m_fRatio);
	SetScrollSizes(MM_TEXT, sizeTotal);

	Invalidate();
}

void CRoboMappingView::OnMagnify()
{
	CRoboMappingDoc* pDoc = GetDocument();
	CScreenReference& ScrnRef = pDoc->m_ScrnRef;

	ScrnRef.m_fRatio *= 1.5f;
	if (ScrnRef.m_fRatio >= MAX_DRAWING_RATIO)
		ScrnRef.m_fRatio = MAX_DRAWING_RATIO;

	m_bBestFitMode = false;

	OnUpdate(NULL, NULL, NULL);
}

void CRoboMappingView::OnReduce()
{
	CRoboMappingDoc* pDoc = GetDocument();
	CScreenReference& ScrnRef = pDoc->m_ScrnRef;

	ScrnRef.m_fRatio /= 1.5f;
	if (ScrnRef.m_fRatio <= MIN_DRAWING_RATIO)
		ScrnRef.m_fRatio = MIN_DRAWING_RATIO;

	m_bBestFitMode = false;

	OnUpdate(NULL, NULL, NULL);
}

void CRoboMappingView::OnMagnify(CPoint point)
{
	CRoboMappingDoc* pDoc = (CRoboMappingDoc*)GetDocument();
	CScreenReference& ScrnRef = pDoc->m_ScrnRef;
	ScreenToClient(&point);
	CPnt pt = ScrnRef.GetWorldPoint(point);

	m_bBestFitMode = false;

	CRect r;
	GetClientRect(r);

	// Set the view port
	ScrnRef.SetViewPort(USHORT(r.Width()), USHORT(r.Height()));
	ScrnRef.m_fRatio *= MAGNIFY_CONST;
	ScrnRef.SetPointMapping(point, pt);
	CPnt ptLeftTop = ScrnRef.GetLeftTopPoint();
	CPoint pointLeftTop = ScrnRef.GetWindowPoint(ptLeftTop);

	m_sizeTotalScroll.cx = (LONG)(m_fWorldWidth * ScrnRef.m_fRatio * 4);//han1102
	m_sizeTotalScroll.cy = (LONG)(m_fWorldHeight * ScrnRef.m_fRatio * 4);
	SetScrollSizes(MM_TEXT, m_sizeTotalScroll);
	CPoint MPoint(m_sizeTotalScroll.cx / 2, m_sizeTotalScroll.cy / 2);
	//	ScrollToPosition(MPoint);
	Invalidate();
}

void CRoboMappingView::OnReduce(CPoint point)
{
	CRoboMappingDoc* pDoc = (CRoboMappingDoc*)GetDocument();
	CScreenReference& ScrnRef = pDoc->m_ScrnRef;
	ScreenToClient(&point);
	CPnt pt = ScrnRef.GetWorldPoint(point);
	m_bBestFitMode = false;

	CRect r;
	GetClientRect(r);

	// Set the view port
	ScrnRef.SetViewPort(USHORT(r.Width()), USHORT(r.Height()));
	ScrnRef.m_fRatio /= MAGNIFY_CONST;
	ScrnRef.SetPointMapping(point, pt);
	CPnt ptLeftTop = ScrnRef.GetLeftTopPoint();
	CPoint pointLeftTop = ScrnRef.GetWindowPoint(ptLeftTop);

	m_sizeTotalScroll.cx = (LONG)(m_fWorldWidth * ScrnRef.m_fRatio * 4);
	m_sizeTotalScroll.cy = (LONG)(m_fWorldHeight * ScrnRef.m_fRatio * 4);
	SetScrollSizes(MM_TEXT, m_sizeTotalScroll);
	CPoint MPoint(m_sizeTotalScroll.cx / 2, m_sizeTotalScroll.cy / 2);
	Invalidate();
	//	ScrollToPosition(CPoint(0, 0));
}

void CRoboMappingView::OnBestFit()
{
	m_bBestFitMode = true;
	ScaleToFitView();
	Invalidate();
	//	OnUpdate(NULL, NULL, NULL);
}

void CRoboMappingView::Close()
{
	CMDIFrameWnd *pFrame = (CMDIFrameWnd*)GetParentFrame();
	CMDIChildWnd *pChild = (CMDIChildWnd *)pFrame->GetActiveFrame();
	pChild->MDIDestroy();
}

void CRoboMappingView::OnLButtonDownCreateModel(CPoint point)
{
}

void CRoboMappingView::OnLButtonDownEditModel(CPoint point)
{
	CRoboMappingDoc* pDoc = GetDocument();
	CScreenReference& ScrnRef = pDoc->m_ScrnRef;

	CPoint ScrollPos = GetScrollPosition();
	point += ScrollPos;

	// 取得当前点所对应的世界坐标
	CPnt pt = ScrnRef.GetWorldPoint(point);

	switch (m_nWorkSubType)
	{
	case EDIT_SUBTYPE_DEL_RECT:
	case EDIT_SUBTYPE_ADD_REFLECTOR:
	case EDIT_SUBTYPE_DEL_REFLECTOR:
		m_bStartRect = true;
		m_RectSelector.Set1stPoint(point);      // 取第1点
		m_OldPoint = point;
		break;

	case EDIT_SUBTYPE_TRANSLATE:                 // 平移变换模式
		switch (m_nWorkStage)
		{
		case 0:
		{
			m_ptFrom = pt;
			m_nWorkStage = TRANSLATE_SET_TARGET_POINT;
		}
		break;

		case TRANSLATE_SET_TARGET_POINT:
		{
			m_ptTo = pt;

			CPosture pst;
			pst.x = -m_ptTo.x + m_ptFrom.x;
			pst.y = -m_ptTo.y + m_ptFrom.y;
			pst.fThita = 0;

			Eigen::Transform<double, 3, Eigen::Affine, Eigen::ColMajor> tr;
			tr = Eigen::Translation<double, 3>(-pst.x, -pst.y, 0) * Eigen::AngleAxis<double>(0, Eigen::Vector3d::UnitZ());
			pMapFuser->map->Transform(tr);

			m_trans = tr * m_trans;

			Invalidate();
			m_nWorkMode = MODE_EDIT_MODEL;
			m_nWorkSubType = 0;
			m_nWorkStage = 0;
		}
		break;
		}
		break;

	case EDIT_SUBTYPE_ROTATE:
		switch (m_nWorkStage)
		{
		case 0:
		{
			m_ptRotateCenter = pt;                // 定义旋转中心
			m_nWorkStage = 1;
		}
		break;

		case 1:                                  // 定义拼接块第二个对应点
		{
			m_ptFrom = pt;
			m_nWorkStage = 2;
		}
		break;

		case 2:
		{
			// 如果距离过短无法处理，退出
			if (m_ptRotateCenter.DistanceTo(pt) < 0.01f)
				return;

			m_ptTo = pt;
			CLine ln1(m_ptRotateCenter, m_ptFrom);
			float fLen = ln1.Length();

			CLine ln2(m_ptRotateCenter, m_ptTo);
			CAngle ang = ln1.AngleToLine(ln2);

			float x0 = m_ptRotateCenter.x;
			float y0 = m_ptRotateCenter.y;
			double dx = x0 * cos(ang) - y0 * sin(ang) - x0;
			double dy = y0 * cos(ang) + x0 * sin(ang) - y0;

			Eigen::Transform<double, 3, Eigen::Affine, Eigen::ColMajor> tr;
			tr = Eigen::Translation<double, 3>(-dx, -dy, 0) *
				Eigen::AngleAxis<double>(ang.m_fRad, Eigen::Vector3d::UnitZ());

			pMapFuser->map->Transform(tr);
			m_trans = tr * m_trans;

			Invalidate();
			m_nWorkSubType = 0;
			m_nWorkStage = 0;
		}
		break;
		}
		break;
	}
}

void CRoboMappingView::OnLButtonDownMergeModel(CPoint point)
{
	CRoboMappingDoc* pDoc = GetDocument();
	CScreenReference& ScrnRef = pDoc->m_ScrnRef;

	CPoint ScrollPos = GetScrollPosition();
	point += ScrollPos;

	// 取得当前点所对应的世界坐标
	CPnt pt = ScrnRef.GetWorldPoint(point);

	switch (m_nWorkSubType)
	{
		case MERGE_SUBTYPE_TRANSLATE:                 // 拼接块坐标平移变换模式
			switch (m_nWorkStage)
			{
			case 0:
			{
				m_ptFrom = pt;
				m_nWorkStage = TRANSLATE_SET_TARGET_POINT;
			}
			break;

			case TRANSLATE_SET_TARGET_POINT:
			{
				m_ptTo = pt;

				CPosture pst;
				pst.x = -m_ptTo.x + m_ptFrom.x;
				pst.y = -m_ptTo.y + m_ptFrom.y;
				pst.fThita = 0;

				Eigen::Transform<double, 3, Eigen::Affine, Eigen::ColMajor> tr;
				tr = Eigen::Translation<double, 3>(-pst.x, -pst.y, 0) * Eigen::AngleAxis<double>(0, Eigen::Vector3d::UnitZ());
				m_MapToMerge.Transform(tr);

				m_trans = tr * m_trans;

				Invalidate();
				m_nWorkMode = MODE_MERGE_MODEL;
				m_nWorkSubType = 0;
				m_nWorkStage = 0;
			}
			break;
			}
			break;

		case MERGE_SUBTYPE_ROTATE:
			switch (m_nWorkStage)
			{
			case 0:
			{
				m_ptRotateCenter = pt;                // 定义旋转中心
				m_nWorkStage = 1;
			}
			break;

			case 1:                                  // 定义拼接块第二个对应点
			{
				m_ptFrom = pt;
				m_nWorkStage = 2;
			}
			break;

			case 2:
			{
				m_ptTo = pt;

				CLine ln1(m_ptRotateCenter, m_ptFrom);
				CLine ln2(m_ptRotateCenter, m_ptTo);
				CAngle ang = ln1.AngleToLine(ln2);

				float x0 = m_ptRotateCenter.x;
				float y0 = m_ptRotateCenter.y;
				double dx = x0 * cos(ang) - y0 * sin(ang) - x0;
				double dy = y0 * cos(ang) + x0 * sin(ang) - y0;

				Eigen::Transform<double, 3, Eigen::Affine, Eigen::ColMajor> tr;
				tr = Eigen::Translation<double, 3>(-dx, -dy, 0) *
					Eigen::AngleAxis<double>(ang.m_fRad, Eigen::Vector3d::UnitZ());

				m_MapToMerge.Transform(tr);
				m_trans = tr * m_trans;

				Invalidate();
				m_nWorkMode = MODE_MERGE_MODEL;
				m_nWorkSubType = 0;
				m_nWorkStage = 0;
			}
			break;
		}
	}
}

void CRoboMappingView::OnLButtonDownTestLocate(CPoint point)
{
	CRoboMappingDoc* pDoc = GetDocument();
	CScreenReference& ScrnRef = pDoc->m_ScrnRef;

	CPoint ScrollPos = GetScrollPosition();
	point += ScrollPos;

	// 取得当前点所对应的世界坐标
	CPnt pt = ScrnRef.GetWorldPoint(point);

	switch (m_nWorkSubType)
	{
	case TEST_SUBTYPE_INIT_POSTURE:
		break;

	case TEST_SUBTYPE_INIT_TRANSLATE:     // 定位初始姿态平移落点
		switch (m_nWorkStage)
		{
		case 0:
		{
			m_ptFrom = pt;
			m_nWorkStage = TRANSLATE_SET_TARGET_POINT;
		}
		break;

		case TRANSLATE_SET_TARGET_POINT:
		{
			m_ptTo = pt;

			CPosture pst;
			pst.x = -m_ptTo.x + m_ptFrom.x;
			pst.y = -m_ptTo.y + m_ptFrom.y;
			pst.fThita = 0;

			Eigen::Transform<double, 3, Eigen::Affine, Eigen::ColMajor> tr;
			tr = Eigen::Translation<double, 3>(-pst.x, -pst.y, 0) * Eigen::AngleAxis<double>(0, Eigen::Vector3d::UnitZ());
			//			m_MapToMerge.Transform(tr);

			m_trans = tr * m_trans;
			CPosture pstTrans = AffineToPosture(tr);
			CFrame frm(pstTrans);
			m_initScan = m_initScan0;
			m_initScan.InvTransform(frm);

			Invalidate();

			m_nWorkSubType = TEST_SUBTYPE_INIT_POSTURE;
			m_nWorkStage = 0;
			m_initScan0 = m_initScan;
		}
		break;
		}
		break;

	case TEST_SUBTYPE_INIT_ROTATE:
		switch (m_nWorkStage)
		{
		case 0:
		{
			m_ptRotateCenter = pt;                // 定义旋转中心
			m_nWorkStage = 1;
		}
		break;

		case 1:                                  // 定义拼接块第二个对应点
		{
			m_ptFrom = pt;
			m_nWorkStage = 2;
		}
		break;

		case 2:
		{
			m_ptTo = pt;

			CLine ln1(m_ptRotateCenter, m_ptFrom);
			CLine ln2(m_ptRotateCenter, m_ptTo);
			CAngle ang = ln1.AngleToLine(ln2);

			float x0 = m_ptRotateCenter.x;
			float y0 = m_ptRotateCenter.y;
			double dx = x0 - (x0 * cos(ang) + y0 * sin(ang));
			double dy = y0 - (y0 * cos(ang) - x0 * sin(ang));

			Eigen::Transform<double, 3, Eigen::Affine, Eigen::ColMajor> tr;
			tr = Eigen::Translation<double, 3>(-dx, -dy, 0) *
				Eigen::AngleAxis<double>(ang.m_fRad, Eigen::Vector3d::UnitZ());

			m_trans = tr * m_trans;

			CPosture pstTrans = AffineToPosture(tr);
			CFrame frm(pstTrans);
			m_initScan = m_initScan0;
			m_initScan.InvTransform(frm);

			Invalidate();
			m_nWorkSubType = TEST_SUBTYPE_INIT_POSTURE;
			m_nWorkStage = 0;
			m_initScan0 = m_initScan;
		}
		break;
		}
		break;

	case TEST_SUBTYPE_DIRECT_LOCATE:
		switch (m_nWorkStage)
		{
		case 0:
		{
			// 如果当前鼠标指向某一位姿，则选它为“选中位姿”
			int nPoseId = pLocalization->correctedPoses.PointHit(pt, 6 / ScrnRef.m_fRatio);
			if (nPoseId >= 0)
			{
				pLocalization->correctedPoses.Select(nPoseId);
				m_nWorkStage = 1;
				m_nTestStep = nPoseId;
				m_pstMove = m_pstTest = pLocalization->correctedPoses.GetPosture(nPoseId);
				m_bFirstDraw = true;
				Invalidate();
			}
		}
		break;

		case 1:
			m_pstTest.GetPntObject() = pt;         // 测试姿态的点位置
			m_nWorkStage = 2;
			break;

		case 2:
		{
			CLine ln(m_pstTest, pt);
			m_pstTest.SetAngle(ln.SlantAngle());

			//			m_nWorkMode = MODE_NORMAL;
			m_nWorkStage = 3;
			OnOffsetTest();
			Invalidate();
		}
		break;

		case 3:
			break;
		}
		break;
	}
}

void CRoboMappingView::OnLButtonDown(UINT nFlags, CPoint point)
{
	CRoboMappingDoc* pDoc = GetDocument();
	CScreenReference& ScrnRef = pDoc->m_ScrnRef;

	CPoint ScrollPos = GetScrollPosition();
	point += ScrollPos;

	// 取得当前点所对应的世界坐标
	CPnt pt = ScrnRef.GetWorldPoint(point);
	m_dwTime = GetTickCount();


	switch (m_nWorkMode)
	{
	case MODE_CREATE_MODEL:
		OnLButtonDownCreateModel(point);
		break;

	case MODE_EDIT_MODEL:
		OnLButtonDownEditModel(point);
		break;

	case MODE_MERGE_MODEL:
		OnLButtonDownMergeModel(point);
		break;

	case MODE_TEST_LOCATE:
		OnLButtonDownTestLocate(point);
		break;
	}

	CScrollView::OnLButtonDown(nFlags, point);
}

void CRoboMappingView::OnLButtonUpEditModel(CPoint point)
{
	CRoboMappingDoc* pDoc = GetDocument();
	CScreenReference& ScrnRef = pDoc->m_ScrnRef;

	CPoint ScrollPos = GetScrollPosition();
	point += ScrollPos;

	int nId;
	float x, y;
	int nParam0, nParam1;
	CString str;

	// 取得当前点所对应的世界坐标
	CPnt pt = ScrnRef.GetWorldPoint(point);

	// 如果处于“直接删除对象”模式
	switch (m_nWorkSubType)
	{
	case EDIT_SUBTYPE_DEL_OBJ:
	{
		CPnt pt = ScrnRef.GetWorldPoint(point);

		if (pMapFuser->map != NULL)
		{
			int nHitCell = pMapFuser->map->PointHitCell(m_ptCur, 3 / ScrnRef.m_fRatio);
			if (nHitCell >= 0)
			{
				pMapFuser->map->DeleteCell(nHitCell);
				Invalidate();
			}
		}
	}
	break;

	case EDIT_SUBTYPE_DEL_RECT:
		m_bStartRect = false; //重置绘制矩形框标志

		m_RectSelector.Set2ndPoint(point);

		// 删除矩形内的NDT-cell
		if (pMapFuser->map != NULL)
		{
			int nCount = pMapFuser->map->GetCellCount();
			for (int i = nCount - 1; i >= 0; i--)
			{
				CPnt pt;
				pMapFuser->map->GetCellPnt(i, pt);
				CPoint pnt1 = ScrnRef.GetWindowPoint(pt);
				if (m_RectSelector.ContainPoint(pnt1))
				{
					pMapFuser->map->DeleteCell(i);
					nCount--;
				}
			}
		}

		Invalidate();

	break;

	case EDIT_SUBTYPE_ADD_REFLECTOR:
		m_bStartRect = false; //重置绘制矩形框标志

		m_RectSelector.Set2ndPoint(point);

		// 根据矩形内的高亮点生成反光板
		if (pMapFuser->map != NULL)
		{
			int count = 0;
			CFlatReflectorFeature ptCenter(0, 0);
			for (size_t i = 0; i < pMapFuser->highIntensityPoints.size(); i++)
			{
				CPnt pt1 = *pMapFuser->highIntensityPoints[i];
				CPoint pnt1 = ScrnRef.GetWindowPoint(pt1);
				if (m_RectSelector.ContainPoint(pnt1))
				{
					ptCenter.x += pt1.x;
					ptCenter.y += pt1.y;
					count++;
				}
			}
			if (count == 0)
				return;

			ptCenter.x /= count;
			ptCenter.y /= count;
			pMapFuser->refPoints += ptCenter;
		}
		Invalidate();

		break;

	case EDIT_SUBTYPE_DEL_REFLECTOR:
		m_bStartRect = false; //重置绘制矩形框标志

		m_RectSelector.Set2ndPoint(point);

		// 将矩形内的反光板删除

		for (int i = pMapFuser->refPoints.size() - 1; i >= 0; i--)
		{
			CPoint pnt1 = ScrnRef.GetWindowPoint(*pMapFuser->refPoints[i]);
			if (m_RectSelector.ContainPoint(pnt1))
			{
				pMapFuser->refPoints.erase(pMapFuser->refPoints.begin() + i);
			}
		}
		Invalidate();

		break;

	}

}

void CRoboMappingView::OnLButtonUp(UINT nFlags, CPoint point)
{
	// TODO: 在此添加消息处理程序代码和/或调用默认值
	switch (m_nWorkMode)
	{
	case MODE_EDIT_MODEL:
		OnLButtonUpEditModel(point);
		break;
	}

	CScrollView::OnLButtonUp(nFlags, point);
}

//
//   从二进制文件中读入环境模型。
//
void CRoboMappingView::OnLoadMap()
{
	CFileDialog Dlg(TRUE, _T(".jff"), 0, OFN_HIDEREADONLY | OFN_READONLY, _T("NDT图(*.jff)|*.jff||", NULL));
	if (Dlg.DoModal() == IDOK)
	{
		std::string str = CStringA(Dlg.GetPathName());
		FILE* fp = fopen(str.c_str(), "rb");
		if (fp == NULL)
			return;

		if (pMapFuser->LoadMap(fp))
		{
			ScaleToFitView();
			Invalidate();
		}

		fclose(fp);
	}
}

void CRoboMappingView::OnSaveMap()
{
	CFileDialog Dlg(FALSE, _T(".jff"), 0, OFN_HIDEREADONLY | OFN_READONLY | OFN_OVERWRITEPROMPT, _T("NDT图(*.jff)|*.jff| 特征图(*.map)|*.map||"), NULL);
	if (Dlg.DoModal() == IDOK)
	{
		std::string str = CStringA(Dlg.GetPathName());
		FILE * fp = fopen(str.c_str(), "wb");
		if (fp == NULL)
			return;

		int n = str.rfind('.');
		std::string str1 = str.substr(n, str.length()-1);

		if (str1 == ".jff")
			pMapFuser->SaveMap(fp);
		else if (str1 == ".map")
			pMapFuser->SaveFeatureMap(fp);
		fclose(fp);
	}
}

void CRoboMappingView::OnDelSingle()
{
	m_nWorkMode = MODE_EDIT_MODEL;
	m_nWorkSubType = EDIT_SUBTYPE_DEL_OBJ;
}

void CRoboMappingView::OnDelRect()
{
	m_nWorkMode = MODE_EDIT_MODEL;
	m_nWorkSubType = EDIT_SUBTYPE_DEL_RECT;
}

void CRoboMappingView::OnEndBuild()
{
	// 在此保存经过修改的Dataset(.dx)和地图(.jff)

	// 先更新数据集(添加绝对位姿数据)
	for (size_t i = 0; i < pMapFuser->correctedPoses.size(); i++)
	{
		CSlamStepData& Step = SlamDataSet[i];

#if 0
		Eigen::Affine3d& Pose = pMapFuser->correctedPoses[i].pose;

		CPosture pst;
		pst.x = Pose.translation().x();
		pst.y = Pose.translation().y();
		pst.fThita = Pose.rotation().eulerAngles(0, 1, 2)(2);
#endif

		SlamDataSet[i].m_pst.GetPostureObject() = pMapFuser->correctedPoses[i];
	}

	// 在此(临时处理方案)修改数据集的长度，使之与correctedPoses同长
//	SlamDataSet.resize(pMapFuser->correctedPoses.size());

	// 可在此询问是否保存数据集和地图文件
}

void CRoboMappingView::OnSize(UINT nType, int cx, int cy)
{
	CScrollView::OnSize(nType, cx, cy);

	// TODO: 在此处添加消息处理程序代码
	if (m_bBestFitMode)
		ScaleToFitView();
}

void CRoboMappingView::OnRButtonDownCreateModel(CPoint point)
{
}

void CRoboMappingView::OnRButtonDownEditModel(CPoint point)
{
	m_nWorkMode = MODE_CREATE_MODEL;
	m_nWorkStage = 0;
	m_bShowHighIntensity = false;
	pMapFuser->ClearMatchStatus();
	Invalidate();
}

void CRoboMappingView::OnRButtonDownMergeModel(CPoint point)
{
	m_nWorkMode = MODE_MERGE_MODEL;
	m_nWorkSubType = 0;
	m_nWorkStage = 0;
//	pMapFuser->ClearMatchStatus();
	Invalidate();
}

void CRoboMappingView::OnRButtonDownTestLocate(CPoint point)
{
	switch (m_nWorkSubType)
	{
	case TEST_SUBTYPE_INIT_TRANSLATE:
	case TEST_SUBTYPE_INIT_ROTATE:
		m_nWorkSubType = TEST_SUBTYPE_INIT_POSTURE;
		m_nWorkStage = 0;
		Invalidate();
		break;

	case TEST_SUBTYPE_DIRECT_LOCATE:
		if (m_nWorkStage != 0)
		{
			m_nWorkStage = 0;
			pLocalization->ClearMatchStatus();
			Invalidate();
		}
		break;
	}
}


void CRoboMappingView::OnRButtonDown(UINT nFlags, CPoint point)
{
	// TODO: 在此添加消息处理程序代码和/或调用默认值
	switch (m_nWorkMode)
	{
	case MODE_CREATE_MODEL:
		OnRButtonDownCreateModel(point);
		break;

	case MODE_EDIT_MODEL:
		OnRButtonDownEditModel(point);
		break;

	case MODE_MERGE_MODEL:
		OnRButtonDownMergeModel(point);
		break;

	case MODE_TEST_LOCATE:
		OnRButtonDownTestLocate(point);
		break;
	}

	CScrollView::OnRButtonDown(nFlags, point);
}


BOOL CRoboMappingView::OnSetCursor(CWnd* pWnd, UINT nHitTest, UINT message)
{
	// TODO: 在此添加消息处理程序代码和/或调用默认值
	POINT point;
	::GetCursorPos(&point);
	ScreenToClient(&point);

	if (m_bScrollStart)
	{
		::SetCursor(LoadCursor(NULL, IDC_SIZEALL));
	}
	
	return CScrollView::OnSetCursor(pWnd, nHitTest, message);
}

BOOL CRoboMappingView::OnMouseWheel(UINT nFlags, short zDelta, CPoint pt)
{
	// TODO: 在此添加消息处理程序代码和/或调用默认值
	if (!m_bScrollStart)
	{
		if (zDelta > 0)
			OnMagnify(pt);
		else if (zDelta < 0)
			OnReduce(pt);

		m_dwTime = GetTickCount();
	}
	return CScrollView::OnMouseWheel(nFlags, zDelta, pt);
}


BOOL CRoboMappingView::OnScrollBy(CSize sizeScroll, BOOL bDoScroll)
{
	// TODO: 在此添加专用代码和/或调用基类
	if (sizeScroll.cy != 0)
		return false;
	else
		return true;

	return CScrollView::OnScrollBy(sizeScroll, bDoScroll);
}

#if 0
void CRoboMappingView::OnTranslation()
{
	// TODO: 在此添加命令处理程序代码
	CTranslationDlg Dlg;
	if (Dlg.DoModal() == IDOK)
	{
		CPosture pst;
		pst.x = -Dlg.m_fX;
		pst.y = -Dlg.m_fY;
		pst.fThita = 0;

		// 在此进行坐标换换
		// ..
		Eigen::Transform<double, 3, Eigen::Affine, Eigen::ColMajor> tr;
		tr = Eigen::Translation<double, 3>(-pst.x, -pst.y, 0) * Eigen::AngleAxis<double>(0, Eigen::Vector3d::UnitZ());
		pMapFuser->map->Transform(tr);

		ScaleToFitView();
		Invalidate();
	}
}
#endif

void CRoboMappingView::OnTranslation()
{
	m_nWorkMode = MODE_EDIT_MODEL;                  // 进入模型编辑模式
	m_nWorkSubType = EDIT_SUBTYPE_TRANSLATE;        // 操作子类为“模型平移”
	m_nWorkStage = 0;
}

#if 0
void CRoboMappingView::OnRotation()
{
	CTransformDlg Dlg;
	if (Dlg.DoModal() == IDOK)
	{
		CPnt ptCenter(Dlg.m_fX, Dlg.m_fY);
		CAngle ang(Dlg.m_fThita / 180 * PI);


		float x0 = ptCenter.x;
		float y0 = ptCenter.y;
		double dx = x0 - (x0 * cos(ang) + y0 * sin(ang));
		double dy = y0 - (y0 * cos(ang) - x0 * sin(ang));
//		dx = -dx;
//		dy = -dy;


		Eigen::Transform<double, 3, Eigen::Affine, Eigen::ColMajor> tr;
		tr = Eigen::Translation<double, 3>(-dx, -dy, 0) *
			Eigen::AngleAxis<double>(ang.m_fRad, Eigen::Vector3d::UnitZ());
		pMapFuser->map->Transform(tr);

		ScaleToFitView();
		Invalidate();
	}
}
#endif

void CRoboMappingView::OnRotation()
{
	m_nWorkMode = MODE_EDIT_MODEL;                  // 进入模型编辑模式
	m_nWorkSubType = EDIT_SUBTYPE_ROTATE;          // 操作子类为“模型旋转”
	m_nWorkStage = 0;
}

void CRoboMappingView::OnShowId()
{
	m_bShowId = !m_bShowId;
	Invalidate();
}

void CRoboMappingView::OnUpdateShowId(CCmdUI *pCmdUI)
{
	pCmdUI->SetCheck(m_bShowId);
}

//
//   进行多块拼接。
//
void CRoboMappingView::OnImportMergeFile()
{
	CFileDialog Dlg(TRUE, _T(".jff"), 0, OFN_HIDEREADONLY | OFN_READONLY, _T("文件 (*.jff)|*.jff||"), NULL);
	if (Dlg.DoModal() == IDOK)
	{
		std::string str = CStringA(Dlg.GetPathName());
		FILE* fp = fopen(str.c_str(), "rb");
		if (fp == NULL)
			return;

		if (m_MapToMerge.Load(fp))
		{
			m_nWorkMode = MODE_MERGE_MODEL;
			m_nWorkSubType = 0;
			m_nWorkStage = 0;
			m_trans.setIdentity();    // 先待合并块的累计变换置为单位阵
			Invalidate();
		}
		fclose(fp);
	}
}

void CRoboMappingView::OnPatchTranslate()
{
	m_nWorkMode = MODE_MERGE_MODEL;                  // 进入模型编辑模式
	m_nWorkSubType = MERGE_SUBTYPE_TRANSLATE;        // 操作子类为“附加块平移”
	m_nWorkStage = 0;
}

void CRoboMappingView::OnPatchRotate()
{
	m_nWorkMode = MODE_MERGE_MODEL;                  // 进入模型编辑模式
	m_nWorkSubType = MERGE_SUBTYPE_ROTATE;           // 操作子类为“附加块旋转”
	m_nWorkStage = 0;                                // 先在拼接块内选择旋转中心点
}

//
//   接受拼接块合并。
//
void CRoboMappingView::OnAcceptMerge()
{
	if (AfxMessageBox(_T("请确认，您是否接受目前的拼接状态？"), MB_OKCANCEL) == IDOK)
	{
		pMapFuser->map->SimpleMerge(m_MapToMerge);
		m_MapToMerge.Clear();
		m_MapToMerge.Reset();
//		m_bMergeModels = false;
		m_nWorkMode = MODE_CREATE_MODEL;
		Invalidate();
	}
}

//
//   放弃拼接块合并。
//
void CRoboMappingView::OnCancelMerge()
{
	if (AfxMessageBox(_T("您确定要放弃当前的拼接操作吗？如确定，拼接数据将丢失！"), MB_OKCANCEL) == IDOK)
	{
//		m_bMergeModels = false;
		m_MapToMerge.Clear();
		m_MapToMerge.Reset();
		m_nWorkMode = MODE_CREATE_MODEL;

		Invalidate();
	}
}

///////////////////////////////////////////////////////////////////////////////

#if 0
//
//   打开扫描观察窗口。
//
void CRoboMappingView::OpenScanViewWindow()
{
	CMainFrame *pMain = (CMainFrame *)AfxGetApp()->m_pMainWnd;
	pMain->OnViewScan();
}
#endif

//
//   关闭扫描观察窗口。
//
void CRoboMappingView::CloseScanViewWindow()
{
	CRoboMappingApp* pApp = (CRoboMappingApp*)AfxGetApp();
	pApp->m_pScanView->Close();
}

void CRoboMappingView::OnShowSourceMatched()
{
	m_bShowSourceMatched = !m_bShowSourceMatched;
	Invalidate();
}

void CRoboMappingView::OnUpdateShowSourceMatched(CCmdUI *pCmdUI)
{
	pCmdUI->SetCheck(m_bShowSourceMatched);
}

void CRoboMappingView::OnTargetMatched()
{
	m_bShowTargetMatched = !m_bShowTargetMatched;
	Invalidate();
}

void CRoboMappingView::OnUpdateTargetMatched(CCmdUI *pCmdUI)
{
	pCmdUI->SetCheck(m_bShowTargetMatched);
}

void CRoboMappingView::OnUpdateMerge(CCmdUI *pCmdUI)
{
	pCmdUI->Enable(m_nWorkMode == MODE_CREATE_MODEL);
}

void CRoboMappingView::OnUpdatePatchTranslate(CCmdUI *pCmdUI)
{
	// TODO: 在此添加命令更新用户界面处理程序代码
	pCmdUI->Enable(m_nWorkMode == MODE_MERGE_MODEL && m_nWorkSubType == 0);
}

void CRoboMappingView::OnUpdatePatchRotate(CCmdUI *pCmdUI)
{
	pCmdUI->Enable(m_nWorkMode == MODE_MERGE_MODEL && m_nWorkSubType == 0);
}


void CRoboMappingView::OnUpdateAcceptMerge(CCmdUI *pCmdUI)
{
	pCmdUI->Enable(m_nWorkMode == MODE_MERGE_MODEL && m_nWorkSubType == 0);
}


void CRoboMappingView::OnUpdateCancelMerge(CCmdUI *pCmdUI)
{
	pCmdUI->Enable(m_nWorkMode == MODE_MERGE_MODEL && m_nWorkSubType == 0);
}


void CRoboMappingView::OnUpdateShowScene(CCmdUI *pCmdUI)
{
}


void CRoboMappingView::OnLButtonDblClk(UINT nFlags, CPoint point)
{
	// TODO: 在此添加消息处理程序代码和/或调用默认值
	CRoboMappingDoc* pDoc = GetDocument();
	CScreenReference& ScrnRef = pDoc->m_ScrnRef;

	CPoint ScrollPos = GetScrollPosition();
	point += ScrollPos;

	// 取得当前点所对应的世界坐标
	CPnt pt = ScrnRef.GetWorldPoint(point);

	CScrollView::OnLButtonDblClk(nFlags, point);
}

void CRoboMappingView::OnKeyDown(UINT nChar, UINT nRepCnt, UINT nFlags)
{
	// TODO: 在此添加消息处理程序代码和/或调用默认值
	switch (nChar)
	{
	case VK_RETURN:
		GetDocument()->OnStepBuild();
		break;

	case VK_CONTROL:
		m_bControlKeyDown = true;
		break;

	case VK_TAB:
#if 0
		if (pWorldPointDlg == NULL)
		{
			pWorldPointDlg = new CWorldPointDlg;
			pWorldPointDlg->Create(IDD_WORLD_POINT, this);
		}
		pWorldPointDlg->ShowWindow(TRUE);
#endif
		break;
	}

	CScrollView::OnKeyDown(nChar, nRepCnt, nFlags);
}


void CRoboMappingView::OnKeyUp(UINT nChar, UINT nRepCnt, UINT nFlags)
{
	// TODO: 在此添加消息处理程序代码和/或调用默认值
	switch (nChar)
	{
	case VK_CONTROL:
		m_bControlKeyDown = false;
		break;

	case VK_PROCESSKEY:
		break;
	}

	CScrollView::OnKeyUp(nChar, nRepCnt, nFlags);
}

void CRoboMappingView::OnOffsetTest()
{
#if 0
	CSlamStepData& Step = TestDataSet[m_nTestStep];
	pLocalization->collectStepData(Step);

	Eigen::Affine3d pose;
	int idx = pLocalization->LocalizeEx(pose);

#else
	CScan& Scan = TestDataSet[m_nTestStep].m_scanLocal[0];     // 仅支持传感器#1的拉偏定位实验
	CPclPointCloud cloud_in;
	for (int i = 0; i < Scan.m_nCount; i++)
	{
		pcl::PointXYZ pt;
		pt.x = Scan.m_pPoints[i].x;
		pt.y = Scan.m_pPoints[i].y;
		pt.z = 0;

		cloud_in.points.push_back(pt);
	}

	CPclPointCloud cloud;
	pMapFuser->FilterCloud(cloud_in, cloud);

	Eigen::Affine3d pose = Eigen::Translation<double, 3>(m_pstTest.x, m_pstTest.y, 0) *
		Eigen::AngleAxis<double>(m_pstTest.fThita, Eigen::Vector3d::UnitZ());

	// 进行区域扩展定位(覆盖范围：X:+/-0.6m, Y:+/-1.2m, thita: +/-24度 )
	int idx = pMapFuser->CNdtLocalization::LocalizeEx(cloud, pose);
#endif

	if (idx >= 0)
	{
		m_pstTestResult.x = pose.translation()(0);
		m_pstTestResult.y = pose.translation()(1);
		m_pstTestResult.fThita = pose.rotation().eulerAngles(0, 1, 2)(2);
		m_nWorkStage = 4;
		Invalidate();
	}
}


void CRoboMappingView::OnMButtonDown(UINT nFlags, CPoint point)
{
	// 为了避免按下中键时误操作滚轮，禁止滚轮操作后200ms以内的中键操作
	if (GetTickCount() - m_dwTime > 200)
	{
		CRoboMappingDoc* pDoc = GetDocument();
		CScreenReference& ScrnRef = pDoc->m_ScrnRef;

		m_ptScroll = ScrnRef.GetWorldPoint(point);
		m_bScrollStart = true;

		::SetCursor(LoadCursor(NULL, IDC_SIZEALL));
	}

	// TODO: 在此添加消息处理程序代码和/或调用默认值
	CScrollView::OnMButtonDown(nFlags, point);
}


void CRoboMappingView::OnMButtonUp(UINT nFlags, CPoint point)
{
	// TODO: 在此添加消息处理程序代码和/或调用默认值
	if (m_bScrollStart)
	{
		CRoboMappingDoc* pDoc = GetDocument();
		CScreenReference& ScrnRef = pDoc->m_ScrnRef;
		ScrnRef.SetPointMapping(point, m_ptScroll);
		m_bScrollStart = false;
		m_bBestFitMode = false;

		SetCursor(LoadCursor(NULL, IDC_ARROW));
		Invalidate();
	}

	CScrollView::OnMButtonUp(nFlags, point);
}


//
//   将当前的附加图与原图自动匹配
//
void CRoboMappingView::OnPatchFit()
{
	perception_oru::NDTMatcherD2D_2D matcher;      // NDT匹配器
	matcher.ITR_MAX = 30;
	matcher.n_neighbours = 2;

	perception_oru::NDTMap *mapModel = pMapFuser->map;
	Eigen::Transform<double, 3, Eigen::Affine, Eigen::ColMajor> t;
	t.setIdentity();

	if (matcher.match(*pMapFuser->map, m_MapToMerge, t, true))
	{
		m_MapToMerge.Transform(t);
		Invalidate();
	}
	else
	{
		AfxMessageBox(_T("匹配失败!"));
	}
}

//
//   对两个数据集进行连接，生成更大的数据。
//
void CRoboMappingView::OnLinkDataset()
{
	float x = m_trans.translation().x();
	float y = m_trans.translation().y();
	float thita = m_trans.rotation().eulerAngles(0, 1, 2)(2);

	CLinkDatasetDlg Dlg(x, y, thita);
	if (Dlg.DoModal() == IDOK)
	{
		FILE * fp = _wfopen(Dlg.m_strPathName, _T("rb"));
		if (fp != NULL)
		{
			bool bOK = DatasetToMerge.LoadRawScanBinary(fp);
			fclose(fp);

			// 现在将两个数据集进行合并

			// 取主数据集的最后一个绝对姿态
			int lastIdx = SlamDataSet.size();
			if (lastIdx == 0)
				return;
			else
				lastIdx--;
		
			Eigen::Affine3d postureMainLast = PostureToAffine(SlamDataSet[lastIdx].m_pst);

			// 取从数据集的第一个姿态(该姿态为相对于(0,0,0)的相对位姿移动)
			Eigen::Affine3d postureMergeFirst = PostureToAffine(CPosture(0, 0, 0));
			postureMergeFirst = postureMergeFirst * m_trans;

CPosture pstFirst = AffineToPosture(postureMergeFirst);

CFrame frmLast(SlamDataSet[lastIdx].m_pst);
pstFirst.Transform(frmLast);


			Eigen::Affine3d postureMove = postureMainLast.inverse() * postureMergeFirst;

CPosture pstTmp2 = AffineToPosture(postureMainLast);
CPosture pstTmp4 = AffineToPosture(postureMove);

			// 修改从数据集的第一个姿态，然后将整个从数据集合并入主数据集中
//			for ()
			DatasetToMerge[0].m_pstMoveEst.GetPostureObject() = AffineToPosture(postureMove);
			
			SlamDataSet += DatasetToMerge;
		}
		else
		{
			AfxMessageBox(_T("文件不存在！"));
		}
	}
}



void CRoboMappingView::OnLocInitPostureTrans()
{
	m_nWorkSubType = TEST_SUBTYPE_INIT_TRANSLATE;
	m_nWorkStage = 0;
}

void CRoboMappingView::OnLocInitPostureRot()
{
	m_nWorkSubType = TEST_SUBTYPE_INIT_ROTATE;
	m_nWorkStage = 0;
}

void CRoboMappingView::OnBatchLocate()
{
	CGotoDlg Dlg((int)TestDataSet.size(), _T("连续定位(到步)："));

	if (Dlg.DoModal() != IDOK)
		return;

	if (nCurSlamStep < 0)
		nCurSlamStep = 0;

	m_nTestTargetStep = Dlg.m_nStep - 1;
	m_nAutoTestCount = 0;
	SetTimer(2, 50, NULL);
}

void CRoboMappingView::OnLoadLocateDataset()
{
	CFileDialog Dlg(TRUE, _T(".dx"), 0, OFN_HIDEREADONLY | OFN_READONLY, _T("数据集文件 (*.dx)|*.dx| 工作包文件(*.wkp)|*.wkp||", NULL));
	if (Dlg.DoModal() == IDOK)
	{
		CString strPathName = Dlg.GetPathName();

		FILE* fp = _wfopen(strPathName, _T("rb"));
		bool bOK = TestDataSet.LoadRawScanBinary(fp);
		fclose(fp);

		m_nWorkMode = MODE_TEST_LOCATE;
		m_nWorkSubType = TEST_SUBTYPE_INIT_POSTURE;

		// 定位对象采用与pMapFuser相同的NDT图
		pLocalization->SetMap(pMapFuser->map);
		pLocalization->SetScannerGroupParam(&TestDataSet.m_ScannerParam);

		m_trans.setIdentity();    // 累计变换置为单位阵
		pLocalization->GetStepPointCloud(TestDataSet[0]);
		m_initScan0 = PclCloud2Scan(pLocalization->GetStepPointCloud(TestDataSet[0]));
		m_initScan = m_initScan0;

		nCurSlamStep = 0;

		Invalidate();
	}
}

void CRoboMappingView::OnSetLocateInitPosture()
{
	odometry = m_trans;   //PostureToAffine(CPosture(0, 0, 0));
	pLocalization->SetPose(m_trans);

	nCurSlamStep = 0;
	m_nWorkSubType = TEST_SUBTYPE_DIRECT_LOCATE;
	m_nWorkStage = 0;
}

void CRoboMappingView::DoStepLocate()
{
	CSlamStepData& Step = TestDataSet[nCurSlamStep];

	Eigen::Affine3d pose;
	pLocalization->collectStepData(Step);
	pLocalization->AsynLocalize(pose, false);

	nCurSlamStep++;
	nCurViewStep = nCurSlamStep;
}

void CRoboMappingView::OnStepLocate()
{
	if (m_nWorkMode != MODE_TEST_LOCATE)
		return;

	if (nCurSlamStep < 0)
		nCurSlamStep = 0;

	// 取得总的步数
	int nTotalStep = TestDataSet.size();

	// 如果当前还没有到最后一步，则后移一步
	if (nCurSlamStep < nTotalStep - 1)
	{
		DoStepLocate();
		Invalidate();
	}
	else
	{
		AfxMessageBox(_T("已到最后一步!"));
		return;
	}
}

void CRoboMappingView::OnUpdateStepLocate(CCmdUI *pCmdUI)
{
	pCmdUI->Enable(m_nWorkMode == MODE_TEST_LOCATE && m_nWorkSubType == TEST_SUBTYPE_DIRECT_LOCATE);
}


void CRoboMappingView::OnUpdateBatchLocate(CCmdUI *pCmdUI)
{
	pCmdUI->Enable(m_nWorkMode == MODE_TEST_LOCATE && m_nWorkSubType == TEST_SUBTYPE_DIRECT_LOCATE);
}


void CRoboMappingView::OnUpdateLocInitPostureTrans(CCmdUI *pCmdUI)
{
	pCmdUI->Enable(m_nWorkMode == MODE_TEST_LOCATE && m_nWorkSubType == TEST_SUBTYPE_INIT_POSTURE);
}

void CRoboMappingView::OnUpdateLocInitPostureRot(CCmdUI *pCmdUI)
{
	pCmdUI->Enable(m_nWorkMode == MODE_TEST_LOCATE && m_nWorkSubType == TEST_SUBTYPE_INIT_POSTURE);
}

void CRoboMappingView::OnUpdatePrepareLoc(CCmdUI *pCmdUI)
{
	pCmdUI->Enable(m_nWorkMode == MODE_TEST_LOCATE && m_nWorkSubType == TEST_SUBTYPE_INIT_POSTURE);
}

void CRoboMappingView::OnEndLocalize()
{
	pLocalization->ClearMatchStatus();

	TestDataSet.clear();
	m_nWorkMode = MODE_CREATE_MODEL;
	m_nWorkSubType = 0;
	m_nWorkStage = 0;
	Invalidate();
}

void CRoboMappingView::OnUpdateEndLocalize(CCmdUI *pCmdUI)
{
	pCmdUI->Enable(m_nWorkMode == MODE_TEST_LOCATE);
}


void CRoboMappingView::OnShowHighIntensity()
{
	m_bShowHighIntensity = !m_bShowHighIntensity;
	Invalidate();
}


void CRoboMappingView::OnUpdateShowHighIntensity(CCmdUI *pCmdUI)
{
	pCmdUI->SetCheck(m_bShowHighIntensity);
}


void CRoboMappingView::OnShowReflectors()
{
	m_bShowReflectors = !m_bShowReflectors;
	Invalidate();
}


void CRoboMappingView::OnUpdateShowReflectors(CCmdUI *pCmdUI)
{
	pCmdUI->SetCheck(m_bShowReflectors);
}


void CRoboMappingView::OnAddReflector()
{
	if (m_nWorkMode != MODE_CREATE_MODEL && m_nWorkMode != MODE_EDIT_MODEL)
		return;

	m_nWorkMode = MODE_EDIT_MODEL;
	m_nWorkSubType = EDIT_SUBTYPE_ADD_REFLECTOR;
	m_nWorkStage = 0;

	if (!m_bShowHighIntensity)
		m_bShowHighIntensity = true;

	Invalidate();
}


void CRoboMappingView::OnDelReflector()
{
	if (m_nWorkMode != MODE_CREATE_MODEL && m_nWorkMode != MODE_EDIT_MODEL)
		return;

	m_nWorkMode = MODE_EDIT_MODEL;
	m_nWorkSubType = EDIT_SUBTYPE_DEL_REFLECTOR;
	m_nWorkStage = 0;
}

void CRoboMappingView::OnImportRouteMap()
{
	CFileDialog Dlg(TRUE, _T(".dat"), 0, OFN_HIDEREADONLY | OFN_READONLY, _T("文件 (*.dat)|*.dat||", NULL));
	if (Dlg.DoModal() == IDOK)
	{
		CFile file;
		VERIFY(file.Open(Dlg.GetPathName(), CFile::modeRead));
		CArchive ar(&file, CArchive::load);
		ar >> World;
		ar.Close();
		file.Close();

		bWorldLoaded = true;

		memset(&LogFontNodeText, 0, sizeof(LOGFONT));
		lstrcpy(LogFontNodeText.lfFaceName, _T("Arial"));
		LogFontNodeText.lfWeight = FW_BOLD; // Bold
		LogFontNodeText.lfItalic = TRUE;
		LogFontNodeText.lfHeight = (long)16;

		Invalidate();
	}
}


void CRoboMappingView::OnUpdateTranslation(CCmdUI *pCmdUI)
{
	pCmdUI->Enable((m_nWorkMode == MODE_EDIT_MODEL || m_nWorkMode == MODE_CREATE_MODEL) && m_nWorkSubType == 0);
}


void CRoboMappingView::OnUpdateRotation(CCmdUI *pCmdUI)
{
	pCmdUI->Enable((m_nWorkMode == MODE_EDIT_MODEL || m_nWorkMode == MODE_CREATE_MODEL) && m_nWorkSubType == 0);
}


BOOL CRoboMappingView::PreTranslateMessage(MSG* pMsg)
{
	// TODO: 在此添加专用代码和/或调用基类
#if 0
	if (pMsg->message != WM_TIMER)
	{
		if (m_TipCtrl.m_hWnd != NULL)
			m_TipCtrl.RelayEvent(pMsg);
	}
#endif

	return CScrollView::PreTranslateMessage(pMsg);
}
