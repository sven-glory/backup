// TransformDlg.cpp : implementation file
//

#include "stdafx.h"
#include "RoboMapping.h"
#include "TransformDlg.h"
#include "afxdialogex.h"


// CTransformDlg dialog

IMPLEMENT_DYNAMIC(CTransformDlg, CDialogEx)

CTransformDlg::CTransformDlg(CWnd* pParent /*=NULL*/)
	: CDialogEx(IDD_TRANSFORM, pParent)
	, m_fX(0)
	, m_fY(0)
	, m_fThita(0)
{

}

CTransformDlg::~CTransformDlg()
{
}

void CTransformDlg::DoDataExchange(CDataExchange* pDX)
{
	CDialogEx::DoDataExchange(pDX);
	DDX_Text(pDX, IDC_X, m_fX);
	DDX_Text(pDX, IDC_Y, m_fY);
	DDX_Text(pDX, IDC_THITA, m_fThita);
}


BEGIN_MESSAGE_MAP(CTransformDlg, CDialogEx)
END_MESSAGE_MAP()


// CTransformDlg message handlers

BOOL CTransformDlg::OnInitDialog()
{
	CDialogEx::OnInitDialog();

	// TODO:  在此添加额外的初始化

	return TRUE;  // return TRUE unless you set the focus to a control
					  // 异常: OCX 属性页应返回 FALSE
}


void CTransformDlg::OnOK()
{
	// TODO: 在此添加专用代码和/或调用基类

	CDialogEx::OnOK();
}
