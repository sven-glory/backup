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

// RoboMappingDoc.cpp : CRoboMappingDoc 类的实现
//

#include "stdafx.h"
// SHARED_HANDLERS 可以在实现预览、缩略图和搜索筛选器句柄的
// ATL 项目中进行定义，并允许与该项目共享文档代码。
#ifndef SHARED_HANDLERS
#include "RoboMapping.h"
#endif

#include "RoboMappingDoc.h"
#include "MainFrm.h"
#include "RoboMappingView.h"
#include "ScanView.h"
#include "SlamDataSet.h"
#include "GotoDlg.h"
#include "DatasetSaveOptionDlg.h"
#include "ReflectorCreateParamDlg.h"
#include "FeatureCreationParam.h"
#include "MapParamDlg.h"
#include "ScannerParamDlg.h"
#include "PclPointCloud.h"
#include "AffinePosture.h"

#include <propkey.h>
#include "Version.h"

#undef max
#undef min

// 定义两种SLAM运行的模式
#define SLAM_MODE_SINGLE_STEP            0        // 音步模式
#define SLAM_MODE_AUTO                   1        // 全速自动模式

#include "ndt_fuser\MapFuser.h"

CMapFuser* pMapFuser = NULL;
CDatasetLocalization* pLocalization = NULL;

#if 0
#ifdef _DEBUG
#define new DEBUG_NEW
#endif
#endif

extern CSlamDataSet SlamDataSet;
CSlamDataSet TestDataSet;     // 用于进行定位测试的数据集

extern bool bRefresh;

int nCurSlamStep;      // SLAM当前步数
int nCurViewStep;

CFeatureCreationParam FeatureCreationParam;

// CRoboMappingDoc

IMPLEMENT_DYNCREATE(CRoboMappingDoc, CDocument)

BEGIN_MESSAGE_MAP(CRoboMappingDoc, CDocument)
	ON_COMMAND(ID_LOAD_DATASET, &CRoboMappingDoc::OnLoadDataset)
	ON_COMMAND(ID_QUICK_BUILD, &CRoboMappingDoc::OnQuickBuild)
	ON_COMMAND(ID_END_BUILD, &CRoboMappingDoc::OnEndBuild)
	ON_UPDATE_COMMAND_UI(ID_QUICK_BUILD, &CRoboMappingDoc::OnUpdateQuickBuild)
	ON_UPDATE_COMMAND_UI(ID_END_BUILD, &CRoboMappingDoc::OnUpdateEndBuild)
	ON_UPDATE_COMMAND_UI(ID_LOAD_DATASET, &CRoboMappingDoc::OnUpdateLoadDataset)
	ON_COMMAND(ID_SAVE_DATASET, &CRoboMappingDoc::OnSaveDataset)
	ON_COMMAND(ID_STEP_BUILD, &CRoboMappingDoc::OnStepBuild)
	ON_UPDATE_COMMAND_UI(ID_STEP_BUILD, &CRoboMappingDoc::OnUpdateStepBuild)
	ON_COMMAND(ID_REF_FEATURE_PARAM, &CRoboMappingDoc::OnRefFeatureParam)
	ON_COMMAND(ID_SCANNERS_PARAM, &CRoboMappingDoc::OnScannersParam)
	ON_COMMAND(ID_DATASET_FILTER, &CRoboMappingDoc::OnDatasetFilter)
	ON_COMMAND(ID_MAP_PARAM, &CRoboMappingDoc::OnMapParam)
END_MESSAGE_MAP()


// CRoboMappingDoc 构造/析构

CRoboMappingDoc::CRoboMappingDoc()
{
	// TODO: 在此添加一次性构造代码
	m_nCountSteps = 0;
	m_nSlamMode = SLAM_MODE_SINGLE_STEP;
	m_nSlamTargetStep = 0;
	nCurSlamStep = nCurViewStep = -1;

	m_bDatasetLoaded = false;
	m_bStepConfirmed = true;
	m_fMapSizeX = DEFAULT_MAP_SIZE_X;
	m_fMapSizeY = DEFAULT_MAP_SIZE_Y;
	m_fMapReso = DEFAULT_MAP_RESO;
}

CRoboMappingDoc::~CRoboMappingDoc()
{
	if (pMapFuser != NULL)
		delete pMapFuser;

	if (pLocalization != NULL)
		delete pLocalization;
}

BOOL CRoboMappingDoc::OnNewDocument()
{
	if (!CDocument::OnNewDocument())
		return FALSE;

	// TODO: 在此添加重新初始化代码
	// (SDI 文档将重用该文档)
	LoadMapParam();
	LoadConfigParam();

	// 在此生成MapFuser
	pMapFuser = new CMapFuser(m_fMapSizeX, m_fMapSizeY, m_fMapReso);
	if (pMapFuser == NULL)
		return false;

	pLocalization = new CDatasetLocalization(m_fMapReso);
	if (pLocalization == NULL)
		return false;

	// 在标题栏上显示名字
	CString strTitle = _T("AutoMapping-V");
	strTitle += SW_VERSION;
	SetTitle(strTitle);

	return TRUE;
}


// CRoboMappingDoc 序列化

void CRoboMappingDoc::Serialize(CArchive& ar)
{
	if (ar.IsStoring())
	{
		// TODO: 在此添加存储代码
	}
	else
	{
		// TODO: 在此添加加载代码
	}
}

#ifdef SHARED_HANDLERS

// 缩略图的支持
void CRoboMappingDoc::OnDrawThumbnail(CDC& dc, LPRECT lprcBounds)
{
	// 修改此代码以绘制文档数据
	dc.FillSolidRect(lprcBounds, RGB(255, 255, 255));

	CString strText = _T("TODO: implement thumbnail drawing here");
	LOGFONT lf;

	CFont* pDefaultGUIFont = CFont::FromHandle((HFONT) GetStockObject(DEFAULT_GUI_FONT));
	pDefaultGUIFont->GetLogFont(&lf);
	lf.lfHeight = 36;

	CFont fontDraw;
	fontDraw.CreateFontIndirect(&lf);

	CFont* pOldFont = dc.SelectObject(&fontDraw);
	dc.DrawText(strText, lprcBounds, DT_CENTER | DT_WORDBREAK);
	dc.SelectObject(pOldFont);
}

// 搜索处理程序的支持
void CRoboMappingDoc::InitializeSearchContent()
{
	CString strSearchContent;
	// 从文档数据设置搜索内容。
	// 内容部分应由“;”分隔

	// 例如:     strSearchContent = _T("point;rectangle;circle;ole object;")；
	SetSearchContent(strSearchContent);
}

void CRoboMappingDoc::SetSearchContent(const CString& value)
{
	if (value.IsEmpty())
	{
		RemoveChunk(PKEY_Search_Contents.fmtid, PKEY_Search_Contents.pid);
	}
	else
	{
		CMFCFilterChunkValueImpl *pChunk = NULL;
		ATLTRY(pChunk = new CMFCFilterChunkValueImpl);
		if (pChunk != NULL)
		{
			pChunk->SetTextValue(PKEY_Search_Contents, value, CHUNK_TEXT);
			SetChunkValue(pChunk);
		}
	}
}

#endif // SHARED_HANDLERS

// CRoboMappingDoc 诊断

#ifdef _DEBUG
void CRoboMappingDoc::AssertValid() const
{
	CDocument::AssertValid();
}

void CRoboMappingDoc::Dump(CDumpContext& dc) const
{
	CDocument::Dump(dc);
}
#endif //_DEBUG


// CRoboMappingDoc 命令


void CRoboMappingDoc::OnLoadDataset()
{
	if (!SlamDataSet.IsEmpty() && AfxMessageBox(_T("当前已装入数据集，需要放弃它吗？"), MB_OKCANCEL) == IDCANCEL)
		return;

	CRoboMappingApp* pApp = (CRoboMappingApp*)AfxGetApp();
	if (LoadDataset())
	{
		nCurSlamStep = nCurViewStep = 0;
		m_nCountSteps = SlamDataSet.size();
		m_nSlamMode = SLAM_MODE_SINGLE_STEP;

		if (!m_bDatasetLoaded)
		{
			CMainFrame *pMain = (CMainFrame *)pApp->m_pMainWnd;
			pMain->OnViewScan(&SlamDataSet);

			m_bDatasetLoaded = true;
		}

		// 将数据集关联到MapFuser
		pMapFuser->SetDataSet(&SlamDataSet);

		UpdateAllViews(NULL);
	}
}

void CRoboMappingDoc::OnQuickBuild()
{
	CGotoDlg Dlg( (int)SlamDataSet.size(), _T("快速执行(到步)："));

	if (Dlg.DoModal() != IDOK)
		return;

	if (nCurSlamStep < 0)
		nCurSlamStep = 0;

	m_nSlamTargetStep = Dlg.m_nStep - 1;
	m_nSlamMode = SLAM_MODE_AUTO;
	
	pMapFuser->Start(m_nSlamTargetStep);

	CRoboMappingView* pMappingView = ((CRoboMappingApp*)AfxGetApp())->m_pMappingView;
}

void CRoboMappingDoc::OnEndBuild()
{
	CRoboMappingView* pView = ((CRoboMappingApp*)AfxGetApp())->m_pMappingView;
	pView->Reset();
	pView->OnEndBuild();
	m_bDatasetLoaded = false;
}

void CRoboMappingDoc::OnUpdateLoadDataset(CCmdUI *pCmdUI)
{
}

void CRoboMappingDoc::OnUpdateQuickBuild(CCmdUI *pCmdUI)
{
	pCmdUI->Enable(m_bDatasetLoaded);
}

void CRoboMappingDoc::OnUpdateEndBuild(CCmdUI *pCmdUI)
{
	pCmdUI->Enable(m_bDatasetLoaded);
}

bool CRoboMappingDoc::LoadConfigParam()
{
	FILE* fp = _wfopen(_T("AutoMapping.cfg"), _T("rb"));
	if (fp != NULL)
	{
		bool bOK = FeatureCreationParam.Load(fp);
		fclose(fp);
		return bOK;
	}
	return true;
}

bool CRoboMappingDoc::SaveConfigParam()
{
	FILE* fp = _wfopen(_T("AutoMapping.cfg"), _T("wb"));
	if (fp != NULL)
	{
		bool bOK = FeatureCreationParam.Save(fp);
		fclose(fp);

		return bOK;
	}
	else
		return false;
}

bool CRoboMappingDoc::LoadMapParam()
{
	FILE* fp = _wfopen(_T("AutoMap.ini"), _T("rt"));
	if (fp != NULL)
	{
		if (fscanf(fp, "%f %f %f", &m_fMapSizeX, &m_fMapSizeY, &m_fMapReso) != 3)
		{
			fclose(fp);
			return false;
		}
	}
	fclose(fp);
	return true;
}

bool CRoboMappingDoc::SaveMapParam()
{
	FILE* fp = _wfopen(_T("AutoMap.ini"), _T("wt"));
	if (fp != NULL)
	{
		if (fprintf(fp, "%f %f %f", m_fMapSizeX, m_fMapSizeY, m_fMapReso) != 3)
		{
			fclose(fp);
			return false;
		}
	}
	fclose(fp);
	return true;
}

///////////////////////////////////////////////////////////////////////////////

bool CRoboMappingDoc::LoadDataset()
{
	CFileDialog Dlg(TRUE, _T(".dx"), 0, OFN_HIDEREADONLY | OFN_READONLY, _T("数据集文件 (*.dx)|*.dx| 工作包文件(*.wkp)|*.wkp||", NULL));
	if (Dlg.DoModal() == IDOK)
	{
		m_strPathName = Dlg.GetPathName();

		FILE* fp = _wfopen(m_strPathName, _T("rb"));
		SlamDataSet.clear();
		bool bOK = SlamDataSet.LoadRawScanBinary(fp);
		fclose(fp);

		if (bOK)
		{
			// 由完整路径名获取文件名
			int n = m_strPathName.GetLength() - m_strPathName.ReverseFind('\\') - 1;
			CString strFileName = m_strPathName.Right(n);

			// 在标题栏上显示文件名
			SetTitle(strFileName);

			return true;
		}
	}
	
	return false;
}

//
//   将收集到的数据保存到二进制数据集文件中。
//
void CRoboMappingDoc::OnSaveDataset()
{
	CDatasetSaveOptionDlg Dlg0((int)SlamDataSet.size());
	if (Dlg0.DoModal() == IDOK)
	{
		// TODO: 在此添加命令处理程序代码
		CFileDialog Dlg(FALSE, _T(".dx"), 0, OFN_HIDEREADONLY | OFN_READONLY | OFN_OVERWRITEPROMPT, _T("文件 (*.dx)|*.dx||", NULL));
		if (Dlg.DoModal() == IDOK)
		{
			CString strPathName = Dlg.GetPathName();
			FILE* fp = _wfopen(strPathName, _T("wb"));
			int nFileVersion = 200;
			bool bOK = SlamDataSet.SaveRawScanBinary(fp, nFileVersion, Dlg0.m_nFrom - 1, Dlg0.m_nTo - 1, Dlg0.m_bReverseOrder, true);
			fclose(fp);
		}
	}
}


void CRoboMappingDoc::OnStepBuild()
{
	CScanView* pView = ((CRoboMappingApp*)AfxGetApp())->m_pScanView;
	CRoboMappingView* pMappingView = ((CRoboMappingApp*)AfxGetApp())->m_pMappingView;

	if (m_nSlamMode == SLAM_MODE_AUTO && !pMapFuser->m_bRunning)
		m_nSlamMode = SLAM_MODE_SINGLE_STEP;

	if (m_nSlamMode != SLAM_MODE_SINGLE_STEP)
		return;

	if (pMapFuser->StepBuild())
	{
		nCurViewStep = pMapFuser->m_nCurBuildStep;
		pView->SetCurStep(nCurViewStep, true);
		pMappingView->AutoFitView();
		pMappingView->Invalidate();
	}
	else
	{
		AfxMessageBox(_T("已到最后一步!"));
		return;
	}
}

void CRoboMappingDoc::OnUpdateStepBuild(CCmdUI *pCmdUI)
{
	pCmdUI->Enable(m_bDatasetLoaded/* && (pMapFuser->m_nCurBuildStep == nCurViewStep)*/);
}

//
//   编辑反光板参数。
//
void CRoboMappingDoc::OnRefFeatureParam()
{
	CReflectorCreateParamDlg Dlg(&FeatureCreationParam.m_RefParam);
	if (Dlg.DoModal() == IDOK)
		SaveConfigParam();
}

void CRoboMappingDoc::OnScannersParam()
{
	if (SlamDataSet.m_ScannerParam.size() == 0)
		return;

	CScannerParamDlg Dlg(&(SlamDataSet.m_ScannerParam));
	if (Dlg.DoModal() == IDOK)
	{
		SlamDataSet.m_ScannerParam = Dlg.m_Param;
		SlamDataSet.SetScannerParam(SlamDataSet.m_ScannerParam);
//		SlamDataSet.ApplyNewScannerParam(SlamDataSet.m_ScannerParam);
		UpdateAllViews(NULL);
	}
}


void CRoboMappingDoc::OnDatasetFilter()
{
	FILE* fp = fopen("ScanFilterRules.txt", "rt");
	if (fp == NULL)
		return;

	bool bOk = false;
	CScanFilterRules Rules;

	while (1)
	{
		int nScannerId, nType, nStartId, nEndId;
		float f[2];

		if (fscanf(fp, "%d %d %d %d %f %f", &nScannerId, &nType, &nStartId, &nEndId, &f[0], &f[1]) != 6)
		{
			bOk = true;
			break;
		}

		if (nType == 1)
			Rules.push_back(CScanFilterRule(nScannerId, 1, nStartId, nEndId, CAngle::ToRadian(f[0]), CAngle::ToRadian(f[1])));
		else if (nType == 2)
			Rules.push_back(CScanFilterRule(nScannerId, 2, nStartId, nEndId, f[0], f[1]));
		else
		{
			break;
		}
	}

	fclose(fp);

	if (bOk)
	{
		SlamDataSet.ApplyFilterRules(Rules);
		UpdateAllViews(NULL);
	}
}

void CRoboMappingDoc::OnMapParam()
{
	CMapParamDlg Dlg(m_fMapSizeX, m_fMapSizeY, m_fMapReso);
	if (Dlg.DoModal() == IDOK)
	{
		m_fMapSizeX = Dlg.m_fRangeX;
		m_fMapSizeY = Dlg.m_fRangeY;
		m_fMapReso = Dlg.m_fCellReso;

		SaveMapParam();
		if (pMapFuser != NULL)
		{
			delete pMapFuser;
			pMapFuser = new CMapFuser(m_fMapSizeX, m_fMapSizeY, m_fMapReso);
			if (pMapFuser == NULL)
			{
				AfxMessageBox(_T("MapFuser生成错误！"));
			}
		}

		if (pLocalization != NULL)
		{
			pLocalization = new CDatasetLocalization(m_fMapReso);
			if (pLocalization == NULL)
			{
				AfxMessageBox(_T("Localization生成错误！"));
			}
		}
	}
}
