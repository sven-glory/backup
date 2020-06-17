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

// RoboMapping.cpp : ����Ӧ�ó��������Ϊ��
//

#include "stdafx.h"
#include "afxwinappex.h"
#include "afxdialogex.h"
#include "RoboMapping.h"
#include "MainFrm.h"

#include "ChildFrm.h"
#include "RoboMappingDoc.h"
#include "RoboMappingView.h"
#include "ScanView.h"
#include "Range.h"
#include <stdio.h>
#include "Version.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#endif

// CRoboMappingApp

BEGIN_MESSAGE_MAP(CRoboMappingApp, CWinAppEx)
	ON_COMMAND(ID_APP_ABOUT, &CRoboMappingApp::OnAppAbout)
	// �����ļ��ı�׼�ĵ�����
	ON_COMMAND(ID_FILE_NEW, &CWinAppEx::OnFileNew)
	ON_COMMAND(ID_FILE_OPEN, &CWinAppEx::OnFileOpen)
END_MESSAGE_MAP()


// CRoboMappingApp ����

CRoboMappingApp::CRoboMappingApp()
{
	// TODO: ������Ӧ�ó��� ID �ַ����滻ΪΨһ�� ID �ַ�����������ַ�����ʽ
	//Ϊ CompanyName.ProductName.SubProduct.VersionInformation
	SetAppID(_T("RoboMapping.AppID.NoVersion"));

	// TODO: �ڴ˴���ӹ�����룬
	// ��������Ҫ�ĳ�ʼ�������� InitInstance ��
	m_pMappingViewTemplate = NULL;
	m_pScanViewTemplate = NULL;
	m_pMappingView = NULL;
	m_pScanView = NULL;
}

// Ψһ��һ�� CRoboMappingApp ����

CRoboMappingApp theApp;


// CRoboMappingApp ��ʼ��

BOOL CRoboMappingApp::InitInstance()
{
	// ���һ�������� Windows XP �ϵ�Ӧ�ó����嵥ָ��Ҫ
	// ʹ�� ComCtl32.dll �汾 6 ����߰汾�����ÿ��ӻ���ʽ��
	//����Ҫ InitCommonControlsEx()��  ���򣬽��޷��������ڡ�
	INITCOMMONCONTROLSEX InitCtrls;
	InitCtrls.dwSize = sizeof(InitCtrls);
	// ��������Ϊ��������Ҫ��Ӧ�ó�����ʹ�õ�
	// �����ؼ��ࡣ
	InitCtrls.dwICC = ICC_WIN95_CLASSES;
	InitCommonControlsEx(&InitCtrls);

	CWinAppEx::InitInstance();

	if (!AfxSocketInit())
	{
		AfxMessageBox(IDP_SOCKETS_INIT_FAILED);
		return FALSE;
	}

	// ��ʼ�� OLE ��
	if (!AfxOleInit())
	{
		AfxMessageBox(IDP_OLE_INIT_FAILED);
		return FALSE;
	}

	AfxEnableControlContainer();

	EnableTaskbarInteraction();

	// ʹ�� RichEdit �ؼ���Ҫ AfxInitRichEdit2()	
	// AfxInitRichEdit2();

	// ��׼��ʼ��
	// ���δʹ����Щ���ܲ�ϣ����С
	// ���տ�ִ���ļ��Ĵ�С����Ӧ�Ƴ�����
	// ����Ҫ���ض���ʼ������
	// �������ڴ洢���õ�ע�����
	// TODO: Ӧ�ʵ��޸ĸ��ַ�����
	// �����޸�Ϊ��˾����֯��
	SetRegistryKey(_T("Ӧ�ó��������ɵı���Ӧ�ó���"));
	LoadStdProfileSettings(4);  // ���ر�׼ INI �ļ�ѡ��(���� MRU)


	InitContextMenuManager();

	InitKeyboardManager();

	InitTooltipManager();
	CMFCToolTipInfo ttParams;
	ttParams.m_bVislManagerTheme = TRUE;
	theApp.GetTooltipManager()->SetTooltipParams(AFX_TOOLTIP_TYPE_ALL,
		RUNTIME_CLASS(CMFCToolTipCtrl), &ttParams);

	// ע��Ӧ�ó�����ĵ�ģ�塣  �ĵ�ģ��
	// �������ĵ�����ܴ��ں���ͼ֮�������
	m_pMappingViewTemplate = new CMultiDocTemplate(IDR_RoboMappingTYPE,
		RUNTIME_CLASS(CRoboMappingDoc),
		RUNTIME_CLASS(CChildFrame), // �Զ��� MDI �ӿ��
		RUNTIME_CLASS(CRoboMappingView));
	if (!m_pMappingViewTemplate)
		return FALSE;
	AddDocTemplate(m_pMappingViewTemplate);

	// ������ MDI ��ܴ���
	CMainFrame* pMainFrame = new CMainFrame;
	if (!pMainFrame || !pMainFrame->LoadFrame(IDR_MAINFRAME))
	{
		delete pMainFrame;
		return FALSE;
	}
	m_pMainWnd = pMainFrame;


	// ������׼ shell ���DDE�����ļ�������������
	CCommandLineInfo cmdInfo;
	ParseCommandLine(cmdInfo);



	// ��������������ָ�������  ���
	// �� /RegServer��/Register��/Unregserver �� /Unregister ����Ӧ�ó����򷵻� FALSE��
	if (!ProcessShellCommand(cmdInfo))
		return FALSE;

	// �������ѳ�ʼ���������ʾ����������и���
	pMainFrame->ShowWindow(SW_SHOWMAXIMIZED);
	pMainFrame->UpdateWindow();

	return TRUE;
}

int CRoboMappingApp::ExitInstance()
{
	//TODO: �����������ӵĸ�����Դ
	AfxOleTerm(FALSE);

	return CWinAppEx::ExitInstance();
}

//
//   ���ɡ�ɨ����ͼģ�塱��
//
BOOL CRoboMappingApp::CreateScanViewTemplate()
{
	// ��������ɹ�������ֱ�ӷ���TRUE
	if (m_pScanViewTemplate != NULL)
		return TRUE;

	m_pScanViewTemplate = new CMultiDocTemplate(IDR_RoboMappingTYPE,
		RUNTIME_CLASS(CRoboMappingDoc),
		RUNTIME_CLASS(CChildFrame), // �Զ��� MDI �ӿ��
		RUNTIME_CLASS(CScanView));
	
	if (!m_pScanViewTemplate)
		return FALSE;
	
	AddDocTemplate(m_pScanViewTemplate);
	return TRUE;
}

CDocTemplate * CRoboMappingApp::GetMappingViewTemplate() const
{
	return m_pMappingViewTemplate;
}

CDocTemplate * CRoboMappingApp::GetScanViewTemplate() const
{
	return m_pScanViewTemplate;
}

// CRoboMappingApp ��Ϣ�������


// ����Ӧ�ó��򡰹��ڡ��˵���� CAboutDlg �Ի���

class CAboutDlg : public CDialogEx
{
public:
	CAboutDlg();

// �Ի�������
#ifdef AFX_DESIGN_TIME
	enum { IDD = IDD_ABOUTBOX };
#endif

protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV ֧��

// ʵ��
protected:
	DECLARE_MESSAGE_MAP()
public:
	CString m_strSWVersion;
	CString m_strRelDate;
	CString m_strRelTime;
};

CAboutDlg::CAboutDlg() : CDialogEx(IDD_ABOUTBOX)
, m_strSWVersion(_T(""))
, m_strRelDate(_T(""))
, m_strRelTime(_T(""))
{
	m_strSWVersion = SW_VERSION;
	m_strRelDate = _T(__DATE__);
	m_strRelTime = _T(__TIME__);
}

void CAboutDlg::DoDataExchange(CDataExchange* pDX)
{
	CDialogEx::DoDataExchange(pDX);
	DDX_Text(pDX, IDC_EDIT_VERSION, m_strSWVersion);
	DDX_Text(pDX, IDC_EDIT_REL_DATE, m_strRelDate);
	DDX_Text(pDX, IDC_EDIT_REL_TIME, m_strRelTime);
}

BEGIN_MESSAGE_MAP(CAboutDlg, CDialogEx)
END_MESSAGE_MAP()

// �������жԻ����Ӧ�ó�������
void CRoboMappingApp::OnAppAbout()
{
	CAboutDlg aboutDlg;
	aboutDlg.DoModal();
}

// CRoboMappingApp �Զ������/���淽��

void CRoboMappingApp::PreLoadState()
{
	BOOL bNameValid;
	CString strName;
	bNameValid = strName.LoadString(IDS_EDIT_MENU);
	ASSERT(bNameValid);
	GetContextMenuManager()->AddMenu(strName, IDR_POPUP_EDIT);
}

void CRoboMappingApp::LoadCustomState()
{
}

void CRoboMappingApp::SaveCustomState()
{
}

// CRoboMappingApp ��Ϣ�������



