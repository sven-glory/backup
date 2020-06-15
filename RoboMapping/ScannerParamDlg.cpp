// ScannerParamDlg.cpp : 实现文件
//

#include "stdafx.h"
#include "RoboMapping.h"
#include "ScannerParamDlg.h"
#include "afxdialogex.h"
#include "Tools.h"


// CScannerParamDlg 对话框

IMPLEMENT_DYNAMIC(CScannerParamDlg, CDialogEx)

CScannerParamDlg::CScannerParamDlg(CScannerGroupParam* pParam, CWnd* pParent /*=NULL*/)
	: CDialogEx(IDD_SCANNER_PARAM_DLG, pParent)
	, m_fInstallX(0)
	, m_fInstallAngle(0)
	, m_fAppStartAngle(0)
	, m_fAppEndAngle(0)
	, m_bUseWhenCreateModel(FALSE)
	, m_bUseWhenLocalize(FALSE)
{
	m_Param = *pParam;
	m_nCurIdx = 0;
}

CScannerParamDlg::~CScannerParamDlg()
{
}

void CScannerParamDlg::DoDataExchange(CDataExchange* pDX)
{
	CDialogEx::DoDataExchange(pDX);
	DDX_Text(pDX, IDC_INSTALL_X, m_fInstallX);
	DDX_Text(pDX, IDC_INSTALL_Y5, m_fInstallY);
	DDX_Text(pDX, IDC_INSTALL_THITA, m_fInstallAngle);
	DDX_Text(pDX, IDC_VISIBLE_STAART_ANGLE, m_fAppStartAngle);
	DDX_Text(pDX, IDC_VISIBLE_END_ANGLE, m_fAppEndAngle);
	DDX_Control(pDX, IDC_SCANNER, m_comboScanner);
	DDX_Check(pDX, IDC_USE_WHEN_CREATE_MODEL, m_bUseWhenCreateModel);
	DDX_Check(pDX, IDC_USE_WHEN_LOCALIZE, m_bUseWhenLocalize);
}


BEGIN_MESSAGE_MAP(CScannerParamDlg, CDialogEx)
	ON_CBN_SELCHANGE(IDC_SCANNER, &CScannerParamDlg::OnCbnSelchangeScanner)
END_MESSAGE_MAP()


// CScannerParamDlg 消息处理程序


BOOL CScannerParamDlg::OnInitDialog()
{
	CString str;

	Init();
	CDialogEx::OnInitDialog();

	for (int i = 0; i < m_Param.size(); i++)
	{
		str.Format(_T("扫描器#%d"), i + 1);
		m_comboScanner.AddString(str);
	}
	m_comboScanner.SetCurSel(m_nCurIdx);

	// TODO:  在此添加额外的初始化

	return TRUE;  // return TRUE unless you set the focus to a control
					  // 异常: OCX 属性页应返回 FALSE
}


void CScannerParamDlg::OnOK()
{
	// TODO: 在此添加专用代码和/或调用基类

	CDialogEx::OnOK();
	Update();
}

void CScannerParamDlg::Init()
{
	m_fInstallX = m_Param[m_nCurIdx].m_pst.x;
	m_fInstallY = m_Param[m_nCurIdx].m_pst.y;
	m_fInstallAngle = CAngle::ToDegree(m_Param[m_nCurIdx].m_pst.fThita);

	m_fAppStartAngle = CAngle::ToDegree(m_Param[m_nCurIdx].m_AppAngleRange[0].fFrom);
	m_fAppEndAngle = CAngle::ToDegree(m_Param[m_nCurIdx].m_AppAngleRange[0].fTo);

	m_bUseWhenCreateModel = m_Param[m_nCurIdx].m_bUseCreateModel;
	m_bUseWhenLocalize = m_Param[m_nCurIdx].m_bUseLocalize;
}

void CScannerParamDlg::Update()
{
	CString str;
	float f;

	GetDlgItem(IDC_INSTALL_X)->GetWindowText(str);
	if (DecStrToFloat(str, f))
		m_fInstallX = f;

	GetDlgItem(IDC_INSTALL_Y5)->GetWindowText(str);
	if (DecStrToFloat(str, f))
		m_fInstallY = f;

	GetDlgItem(IDC_INSTALL_THITA)->GetWindowText(str);
	if (DecStrToFloat(str, f))
		m_fInstallAngle = f;

	m_Param[m_nCurIdx].m_pst.Create(m_fInstallX, m_fInstallY, CAngle::ToRadian(m_fInstallAngle));
	m_Param[m_nCurIdx].m_AppAngleRange[0].fFrom = CAngle::ToRadian(m_fAppStartAngle);
	m_Param[m_nCurIdx].m_AppAngleRange[0].fTo = CAngle::ToRadian(m_fAppEndAngle);
	m_Param[m_nCurIdx].m_bUseCreateModel = m_bUseWhenCreateModel;
	m_Param[m_nCurIdx].m_bUseLocalize = m_bUseWhenLocalize;
}

void CScannerParamDlg::OnCbnSelchangeScanner()
{
	int nIdx = m_comboScanner.GetCurSel();
	if (nIdx != m_nCurIdx)
	{
		Update();
		m_nCurIdx = nIdx;

		Init();
		UpdateData(false);
	}
}
