// GotoDlg.cpp : 实现文件
//

#include "stdafx.h"
#include "RoboMapping.h"
#include "GotoDlg.h"
#include "afxdialogex.h"


// CGotoDlg 对话框

IMPLEMENT_DYNAMIC(CGotoDlg, CDialogEx)

CGotoDlg::CGotoDlg(int nMaxStep, CString strNote, CWnd* pParent /*=NULL*/)
	: CDialogEx(IDD_GOTO_DLG, pParent)
	, m_nStep(nMaxStep)
{
	m_strNote = strNote;
}

CGotoDlg::~CGotoDlg()
{
}

void CGotoDlg::DoDataExchange(CDataExchange* pDX)
{
	CDialogEx::DoDataExchange(pDX);
	DDX_Text(pDX, IDC_STEP, m_nStep);
}


BEGIN_MESSAGE_MAP(CGotoDlg, CDialogEx)
END_MESSAGE_MAP()


// CGotoDlg 消息处理程序


BOOL CGotoDlg::OnInitDialog()
{
	CDialogEx::OnInitDialog();

	// TODO:  在此添加额外的初始化
	GetDlgItem(IDC_NOTE)->SetWindowText(m_strNote);

	return TRUE;  // return TRUE unless you set the focus to a control
					  // 异常: OCX 属性页应返回 FALSE
}
