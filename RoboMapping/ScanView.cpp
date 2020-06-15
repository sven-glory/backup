// ScanView.cpp : ʵ���ļ�
//

#include "stdafx.h"
#include "MainFrm.h"
#include "RoboMapping.h"
#include "RoboMappingDoc.h"
#include "ScanView.h"
#include "SlamDataSet.h"
#include "GotoDlg.h"

#define _RGB(r,g,b)          ((COLORREF)(((BYTE)(r)|((WORD)((BYTE)(g))<<8))|(((DWORD)(BYTE)(b))<<16)))

///////////////////////////////////////////////////////////////////////////////

IMPLEMENT_DYNCREATE(CScanView, CScrollView)

CScanView::CScanView()
{
	m_pDataset = NULL;
	m_nCurStep = -1;
	m_nSetStep = -1;
	m_nEditMode = EDIT_MODE_NORMAL;
	m_fWorldWidth = 104;
	m_fWorldHeight = 104;
	m_bBestFitMode = true;
	m_bShowPointFeatures = false;
	m_bStartRect = false;
	m_bTransformScan = false;
	m_bSuggestFeatures = true;
	m_bControlKeyDown = false;
	m_bScrollStart = false;
}

CScanView::~CScanView()
{
}

BEGIN_MESSAGE_MAP(CScanView, CScrollView)
	ON_WM_MOUSEMOVE()
	ON_WM_SIZE()
	ON_WM_TIMER()
	ON_WM_MOUSEWHEEL()
	ON_COMMAND(ID_AUTO_FIT, &CScanView::OnAutoFit)
	ON_WM_RBUTTONDOWN()
	ON_WM_LBUTTONDOWN()
	ON_WM_LBUTTONUP()
	ON_WM_KEYDOWN()
	ON_WM_MOUSEACTIVATE()
	ON_WM_MOUSEHOVER()
	ON_COMMAND(ID_SCAN_SHOW_HIGH_INTENSITY, &CScanView::OnShowHighIntensity)
	ON_UPDATE_COMMAND_UI(ID_SCAN_SHOW_HIGH_INTENSITY, &CScanView::OnUpdateShowHighIntensity)
	ON_WM_KEYUP()
	ON_WM_MBUTTONDOWN()
	ON_WM_MBUTTONUP()
	ON_COMMAND(ID_VIEW_PREV_STEP, &CScanView::OnViewPrevStep)
	ON_UPDATE_COMMAND_UI(ID_VIEW_PREV_STEP, &CScanView::OnUpdateViewPrevStep)
	ON_COMMAND(ID_VIEW_NEXT_STEP, &CScanView::OnViewNextStep)
	ON_UPDATE_COMMAND_UI(ID_VIEW_NEXT_STEP, &CScanView::OnUpdateViewNextStep)
	ON_COMMAND(ID_VIEW_CUR_STEP, &CScanView::OnViewCurStep)
	ON_UPDATE_COMMAND_UI(ID_VIEW_CUR_STEP, &CScanView::OnUpdateViewCurStep)
	ON_COMMAND(ID_VIEW_ANY_STEP, &CScanView::OnViewAnyStep)
	ON_UPDATE_COMMAND_UI(ID_VIEW_ANY_STEP, &CScanView::OnUpdateViewAnyStep)
	ON_COMMAND(ID_SCAN_COMPARE, &CScanView::OnScanCompare)
END_MESSAGE_MAP()

//
//   �趨�����õ����ݼ���
//
void CScanView::SetDataSet(CSlamDataSet* pDataset)
{
	m_pDataset = pDataset;
	m_nCurStep = 0;
	m_nSetStep = -1;
}

//
//   �趨��ǰ�Ĳ���
//
bool CScanView::SetCurStep(int nStep, bool bApply)
{
	if (nStep >= 0 && nStep < m_pDataset->size())
	{
		if (m_nCurStep != nStep)
		{
			if (bApply)
			{
				m_nCurStep = nStep;
				Invalidate();
			}
			else
				m_nSetStep = nStep;

			return true;
		}
	}

	m_nSetStep = -1;
	return false;
}

BOOL CScanView::PreCreateWindow(CREATESTRUCT& cs)
{
	// TODO: �ڴ����ר�ô����/����û���
	cs.lpszClass = AfxRegisterWndClass
	(CS_HREDRAW | CS_VREDRAW | CS_DBLCLKS,
		(AfxGetApp()->LoadStandardCursor(IDC_ARROW)),
		(HBRUSH)::GetStockObject(BLACK_BRUSH),
		0);

	return CScrollView::PreCreateWindow(cs);
}

void CScanView::OnInitialUpdate()
{
	((CRoboMappingApp*)AfxGetApp())->m_pScanView = this;

	CScrollView::OnInitialUpdate();
	GetParent()->SetWindowText(_T("���ݼ�����"));

	m_hCursorDel = LoadCursor(AfxGetInstanceHandle(), MAKEINTRESOURCE(IDC_DEL_W));
	m_hCursorDelThis = LoadCursor(AfxGetInstanceHandle(), MAKEINTRESOURCE(IDC_DEL_THIS_W));
	m_hCursorDelRect = LoadCursor(AfxGetInstanceHandle(), MAKEINTRESOURCE(IDC_DEL_RECT_W));
	m_hCursorSel = LoadCursor(AfxGetInstanceHandle(), MAKEINTRESOURCE(IDC_SEL_W));
	m_hCursorSelThis = LoadCursor(AfxGetInstanceHandle(), MAKEINTRESOURCE(IDC_SEL_THIS_W));
	m_hCursorSelRect = LoadCursor(AfxGetInstanceHandle(), MAKEINTRESOURCE(IDC_SEL_RECT_W));

	TRACKMOUSEEVENT tme = { 0 };
	tme.cbSize = sizeof(tme);
	tme.dwFlags = TME_HOVER;
	tme.hwndTrack = m_hWnd;
	tme.dwHoverTime = HOVER_DEFAULT;  // HOVER_DEFAULT, or the hover timeout in milliseconds.
	::TrackMouseEvent(&tme);

	ScaleToFitView();
	SetTimer(2, 500, NULL);
}

// CScanView ��ͼ
void CScanView::OnDraw(CDC* pDC)
{
	if (m_pDataset == NULL)
		return;

	CRoboMappingDoc* pDoc = (CRoboMappingDoc*)GetDocument();

	// ���û�����ݣ�ֱ�ӷ���
	if (pDoc->m_nCountSteps < 0)
		return;

	int nCurStep = m_nCurStep;
	if (nCurStep >= 0)
	{
		m_pDataset->Plot(nCurStep, m_ScrnRef, pDC, _RGB(160, 160, 160), true, m_bShowPointFeatures, _RGB(255, 255, 0));

		CString str;
		str.Format(_T("Step:%d (of %d)"), nCurStep + 1, m_pDataset->size());

		COLORREF crOldColor = pDC->SetTextColor(_RGB(0, 0, 0));
		pDC->TextOut(10, 10, str);
		pDC->SetTextColor(crOldColor);
	}
}


// CScanView ���

#ifdef _DEBUG
void CScanView::AssertValid() const
{
	CScrollView::AssertValid();
}

#ifndef _WIN32_WCE
void CScanView::Dump(CDumpContext& dc) const
{
	CScrollView::Dump(dc);
}
#endif
#endif // DEBUG
CRoboMappingDoc* CScanView::GetDocument() const // �ǵ��԰汾��������
{
	ASSERT(m_pDocument->IsKindOf(RUNTIME_CLASS(CRoboMappingDoc)));
	return (CRoboMappingDoc*)m_pDocument;
}


// CScanView ��Ϣ�������

//
//   ���������Ա���Ӧ���ڡ�
//
void CScanView::ScaleToFitView()
{
	CRect r;
	GetClientRect(r);

	// Set the view port
	m_ScrnRef.SetViewPort(USHORT(r.Width()), USHORT(r.Height()));
//	m_ScrnRef.SetScrollPos(0, 0);

	// Caculate the display ratio
	float fWidthRatio = r.Width() / m_fWorldWidth;
	float fHeightRatio = r.Height() / m_fWorldHeight;
	m_ScrnRef.SetRatio(min(fWidthRatio, fHeightRatio));

	// Set the center point
	CPoint pntWindowCenter(r.Width() / 2, r.Height() / 2);
	CPnt ptCenter(0.0f, 0.0f);
	m_ScrnRef.SetPointMapping(pntWindowCenter, ptCenter);

	m_sizeTotalScroll.cx = (LONG)(m_fWorldWidth * m_ScrnRef.m_fRatio);
	m_sizeTotalScroll.cy = (LONG)(m_fWorldHeight * m_ScrnRef.m_fRatio);

	SetScrollSizes(MM_TEXT, m_sizeTotalScroll);
}

//
//   ����ǰ�ı����߳��Ը����ı���ֵ��
//
void CScanView::OnMultiplyScale(CPoint point, float fRatio)
{
	CRoboMappingDoc* pDoc = (CRoboMappingDoc*)GetDocument();
	CScreenReference& ScrnRef = m_ScrnRef;
	ScreenToClient(&point);
	CPnt pt = ScrnRef.GetWorldPoint(point);

	m_bBestFitMode = false;

	CRect r;
	GetClientRect(r);

	// Set the view port
	ScrnRef.SetViewPort(USHORT(r.Width()), USHORT(r.Height()));
	ScrnRef.m_fRatio *= fRatio;

	// �������ŵ����ĵ�(������)
	ScrnRef.SetPointMapping(point, pt);

	// ���ù�����Χ
	m_sizeTotalScroll.cx = (LONG)(m_fWorldWidth * ScrnRef.m_fRatio);
	m_sizeTotalScroll.cy = (LONG)(m_fWorldHeight * ScrnRef.m_fRatio);
	SetScrollSizes(MM_TEXT, m_sizeTotalScroll);

	Invalidate();
}

void CScanView::OnUpdate(CView* /*pSender*/, LPARAM /*lHint*/, CObject* /*pHint*/)
{
	// TODO: �ڴ����ר�ô����/����û���
}

//
//   �жϵ�ǰ���ڵ��Ƿ���ָ�����д�����ĳ��ͼ�ζ���
//   ����ֵ��
//      0: û�д������κζ���
//      1: ������������(��ʱnParam1���ص�������ţ�nParam2���ط���ǿ�ȣ�nParam3���ص���)
//      3: ������ɢ��(��ʱnParam1���ؼ�������ţ�nParam2����ɢ����, nParam3����ɢ�㷴��ǿ��)
//
int CScanView::PointHitObject(const CPoint& point, int nStepId, int& nParam1, int& nParam2, int& nParam3)
{
	// ���㵱ǰ���λ��
	CPnt pt = m_ScrnRef.GetWorldPoint(point);

	// �������ű���ȷ����������(��3������Ϊ�����)
	float fDistGate = 3 / m_ScrnRef.m_fRatio;
	if (m_ScrnRef.m_fRatio > 500)
		fDistGate *= 3;

	// �������λ��ȡ�ö�Ӧ�ڵ�ǰ���ĵ�������ɢ����Ϣ
	int nPointHitRawPoint = m_pDataset->PointHitRawPoint(nStepId, pt, fDistGate);

	// ���������ɢ��
	if (nPointHitRawPoint >= 0)
	{
		nParam1 = ((unsigned int)nPointHitRawPoint) >> 16;              // ���������
		nParam2 = ((unsigned int) nPointHitRawPoint) & 0xFFFF;          // ɢ����
		nParam3 = m_pDataset->GetWorldRawPoint(nStepId, 0, nParam1)->m_nIntensity;     // ����ǿ��
		return 3;
	}

	return 0;
}

void CScanView::OnMouseMove(UINT nFlags, CPoint point)
{
	// TODO: �ڴ������Ϣ�����������/�����Ĭ��ֵ
	CMainFrame* pMain = (CMainFrame*)AfxGetApp()->m_pMainWnd;

	CRoboMappingDoc* pDoc = (CRoboMappingDoc*)GetDocument();
	CScreenReference& ScrnRef = m_ScrnRef;

	CPoint ScrollPos = GetScrollPosition();
	point += ScrollPos;

	// ���㵱ǰ���λ��
	CPnt pt = ScrnRef.GetWorldPoint(point);
	CString str, str1 = _T("");
	str.Format(_T("��ǰ:(%.3f, %.3f)"), pt.x, pt.y);

	// ��״̬����ʾ��ǰ���λ��(�ڵ�ǰ����)��Ӧ��������ɢ����Ϣ
	int nParam1, nParam2, nParam3;
	int nHitObject = PointHitObject(point, m_nCurStep, nParam1, nParam2, nParam3);

	switch(nHitObject)
	{
	// ������������
	case 1:
		str1.Format(_T("   ������#%d, ǿ��%d, ����%d"), nParam1, nParam2, nParam3);
		break;

	// ������ֱ������
	case 2:
		str1.Format(_T("   ֱ��#%d"), nParam1);
		break;

	// ������ɢ��
	case 3:
		str1.Format(_T("   ������#%d, ɢ��#%d, ǿ��%d"), nParam1+1, nParam2, nParam3);
		break;
	
	// �ڵ�ǰ��δ�����κζ���
	case 0:
	{
	}
	break;
	}

	str += str1;

	CMFCRibbonBaseElement * pElement = (CMFCRibbonStatusBarPane*)pMain->m_wndStatusBar.FindElement(ID_STATUSBAR_PANE2);
	pElement->SetText(str);
	pElement->Redraw();

	// ���ù��״̬
	switch (m_nEditMode)
	{
	case EDIT_MODE_NORMAL:
		::SetCursor(LoadCursor(NULL, IDC_ARROW));
		break;

	case EDIT_MODE_SEL_OBJECT:
		if (nHitObject == 1 || nHitObject == 2)
		{
			if (m_bControlKeyDown)
				::SetCursor(m_hCursorDelThis);
			else
				::SetCursor(m_hCursorSelThis);
		}
		else
		{
			if (m_bControlKeyDown)
				::SetCursor(m_hCursorDel);
			else
				::SetCursor(m_hCursorSel);
		}
		break;

	case EDIT_MODE_SEL_RECT:
	{
		CDC* pDC = GetDC();

		//SetRop2 Specifies the new drawing mode.(MSDN)
		//R2_NOT   Pixel is the inverse of the screen color.(MSDN)
		//�����ú�������������Ƶ���ɫ�����ò�������ɫ����Ϊԭ��Ļ��ɫ�ķ�ɫ
		//��������������������εĻ����Ϳ��Իָ�ԭ����Ļ����ɫ�ˣ����£�
		//���ǣ�������������λ���ȴ������һ����Ϣ��Ӧ����ɵ�
		//�����ڵ�һ���϶���Ӧ�Ļ��ƿ�����ʾ��Ҳ���ǿ����ģ����ڶ����϶�����ʵ�ֲ�����Ҳ�Ϳ������ˣ�
		pDC->SetROP2(R2_NOT);   //��Ϊ�ؼ�!!!
		pDC->SelectStockObject(NULL_BRUSH); //��ʹ�û�ˢ

		if (m_bStartRect)   //�����Ƿ��е����ж��Ƿ���Ի�����
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
	
		if (m_bControlKeyDown)
			::SetCursor(m_hCursorDelRect);
		else
			::SetCursor(m_hCursorSelRect);
	}
	break;
	}

	CScrollView::OnMouseMove(nFlags, point);
}

void CScanView::OnSize(UINT nType, int cx, int cy)
{
	CScrollView::OnSize(nType, cx, cy);

	if (m_bBestFitMode)
		ScaleToFitView();

	// TODO: �ڴ˴������Ϣ����������
}

void CScanView::OnTimer(UINT_PTR nIDEvent)
{
	// TODO: �ڴ������Ϣ�����������/�����Ĭ��ֵ
	if (nIDEvent == 2)
	{
		if (m_nSetStep >= 0 && m_nCurStep != m_nSetStep)
		{
			m_nCurStep = m_nSetStep;
			m_nSetStep = -1;
			Invalidate();
		}
	}

	CScrollView::OnTimer(nIDEvent);
}

BOOL CScanView::OnMouseWheel(UINT nFlags, short zDelta, CPoint pt)
{
	// TODO: �ڴ������Ϣ�����������/�����Ĭ��ֵ
	if (!m_bScrollStart)
	{
		if (zDelta > 0)
			OnMultiplyScale(pt, 1.2f);
		else if (zDelta < 0)
			OnMultiplyScale(pt, 1.0f / 1.2f);

		m_dwMouseWheel = GetTickCount();
	}

	return CScrollView::OnMouseWheel(nFlags, zDelta, pt);
}

BOOL CScanView::OnScrollBy(CSize sizeScroll, BOOL bDoScroll)
{
	// ����˴��й����������ϴ�MouseWheel�¼������ʱ��̫�̣����ֹ�˴ε�Scroll����
	if (GetTickCount() - m_dwMouseWheel < 100)
		return false;
	else
		return CScrollView::OnScrollBy(sizeScroll, bDoScroll);
}

//
//   ��ʾ�Զ���Ӧ���ڴ�С��
//
void CScanView::OnAutoFit()
{
	m_bBestFitMode = true;
	ScaleToFitView();
	Invalidate();
}

//
//   �رձ��ӿڡ�
//
void CScanView::Close()
{
	CMDIFrameWnd *pFrame = (CMDIFrameWnd*)GetParentFrame();
	CMDIChildWnd *pChild = (CMDIChildWnd *)pFrame->GetActiveFrame();

	pChild->MDIDestroy();
}

void CScanView::OnRButtonDown(UINT nFlags, CPoint point)
{
	// TODO: �ڴ������Ϣ�����������/�����Ĭ��ֵ
	if (m_nEditMode != EDIT_MODE_NORMAL)
		m_nEditMode = EDIT_MODE_NORMAL;

	CScrollView::OnRButtonDown(nFlags, point);
}

void CScanView::OnLButtonDown(UINT nFlags, CPoint point)
{
	CScrollView::OnLButtonDown(nFlags, point);
}

void CScanView::OnLButtonUp(UINT nFlags, CPoint point)
{
	CScrollView::OnLButtonUp(nFlags, point);
}

void CScanView::OnKeyDown(UINT nChar, UINT nRepCnt, UINT nFlags)
{
	// TODO: �ڴ������Ϣ�����������/�����Ĭ��ֵ
	switch (nChar)
	{
	case VK_RETURN:
		break;

	case VK_CONTROL:
		m_bControlKeyDown = true;
		break;
		
	case VK_LEFT:
		OnViewPrevStep();
		break;

	case VK_RIGHT:
		OnViewNextStep();
		break;
		
	}

	CScrollView::OnKeyDown(nChar, nRepCnt, nFlags);
}

void CScanView::OnKeyUp(UINT nChar, UINT nRepCnt, UINT nFlags)
{
	// TODO: �ڴ������Ϣ�����������/�����Ĭ��ֵ
	switch (nChar)
	{
	case VK_RETURN:
		//		GetDocument()->OnStepPreview();
		break;

	case VK_CONTROL:
		m_bControlKeyDown = false;
		break;

	case VK_PROCESSKEY:
		break;
	}

	CScrollView::OnKeyUp(nChar, nRepCnt, nFlags);
}

int CScanView::OnMouseActivate(CWnd* pDesktopWnd, UINT nHitTest, UINT message)
{
	// TODO: �ڴ������Ϣ�����������/�����Ĭ��ֵ
#if 0
	CMainFrame *pMain = (CMainFrame *)AfxGetApp()->m_pMainWnd;
	pMain->m_wndRibbonBar.ShowCategory(1);
#endif

	return CScrollView::OnMouseActivate(pDesktopWnd, nHitTest, message);
}

void CScanView::OnMouseHover(UINT nFlags, CPoint point)
{
	// TODO: �ڴ������Ϣ�����������/�����Ĭ��ֵ

	CScrollView::OnMouseHover(nFlags, point);
}

void CScanView::OnShowHighIntensity()
{
	m_bShowPointFeatures = !m_bShowPointFeatures;
	Invalidate();
}

void CScanView::OnUpdateShowHighIntensity(CCmdUI *pCmdUI)
{
	pCmdUI->SetCheck(m_bShowPointFeatures);
}


void CScanView::OnMButtonDown(UINT nFlags, CPoint point)
{
	// Ϊ�˱��ⰴ���м�ʱ��������֣���ֹ���ֲ�����200ms���ڵ��м�����
	if (GetTickCount() - m_dwTime > 200)
	{
		m_ptScroll = m_ScrnRef.GetWorldPoint(point);
		m_bScrollStart = true;

		::SetCursor(LoadCursor(NULL, IDC_SIZEALL));
	}

	CScrollView::OnMButtonDown(nFlags, point);
}


void CScanView::OnMButtonUp(UINT nFlags, CPoint point)
{
	// TODO: �ڴ������Ϣ�����������/�����Ĭ��ֵ
	if (m_bScrollStart)
	{
		m_ScrnRef.SetPointMapping(point, m_ptScroll);
		m_bScrollStart = false;

		SetCursor(LoadCursor(NULL, IDC_ARROW));
		Invalidate();
	}

	CScrollView::OnMButtonUp(nFlags, point);
}


void CScanView::OnViewPrevStep()
{
	if (m_pDataset != NULL && m_nCurStep > 0)
	{
		m_nCurStep--;
		Invalidate();
	}
}

void CScanView::OnUpdateViewPrevStep(CCmdUI *pCmdUI)
{
	pCmdUI->Enable(m_pDataset != NULL && m_nCurStep > 0);
}

void CScanView::OnViewNextStep()
{
	if (m_pDataset != NULL && m_nCurStep < (int)m_pDataset->size() - 1)
	{
		m_nCurStep++;
		Invalidate();
	}
}

void CScanView::OnUpdateViewNextStep(CCmdUI *pCmdUI)
{
	pCmdUI->Enable(m_pDataset != NULL && m_nCurStep < (int)m_pDataset->size() - 1);
}

void CScanView::OnViewCurStep()
{
	// TODO: �ڴ���������������
}


void CScanView::OnUpdateViewCurStep(CCmdUI *pCmdUI)
{
	// TODO: �ڴ������������û����洦��������
}

void CScanView::OnViewAnyStep()
{
	CGotoDlg Dlg(m_pDataset->size(), _T("��ת������"));
	if (Dlg.DoModal() == IDOK)
	{
		m_nCurStep = Dlg.m_nStep - 1;
		Invalidate();
	}
}

void CScanView::OnUpdateViewAnyStep(CCmdUI *pCmdUI)
{
	pCmdUI->Enable(m_pDataset != NULL);
}


void CScanView::OnScanCompare()
{
	CPosture res;
	bool b = m_pDataset->MatchScans(m_nCurStep, 1, 0, res);
	if (b)
	{
#if 0
		float tmp = res.x;
		res.x = res.y;
		res.y = tmp;
#endif
		m_pDataset->m_ScannerParam.SetRelativePosture(0, 1, res);
		m_pDataset->CreateGlobalData();
		Invalidate();
	}
}
