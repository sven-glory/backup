// OutputOptionDlg.cpp : 实现文件
//

#include "stdafx.h"
#include "RoboMapping.h"
#include "OutputOptionDlg.h"
#include "afxdialogex.h"


// COutputOptionDlg 对话框

IMPLEMENT_DYNAMIC(COutputOptionDlg, CDialogEx)

COutputOptionDlg::COutputOptionDlg(bool bUseReflectors, bool bUseLines, CWnd* pParent /*=NULL*/)
	: CDialogEx(IDD_EXPORT_OPTION, pParent)
	, m_bUseReflectors(bUseReflectors)
	, m_bUseLines(bUseLines)
{
}

COutputOptionDlg::~COutputOptionDlg()
{
}

void COutputOptionDlg::DoDataExchange(CDataExchange* pDX)
{
	CDialogEx::DoDataExchange(pDX);
	DDX_Check(pDX, IDC_USE_REFLECTORS, m_bUseReflectors);
	DDX_Check(pDX, IDC_USE_LINES, m_bUseLines);
}


BEGIN_MESSAGE_MAP(COutputOptionDlg, CDialogEx)
END_MESSAGE_MAP()


// COutputOptionDlg 消息处理程序


void COutputOptionDlg::OnOK()
{
	// TODO: 在此添加专用代码和/或调用基类

	CDialogEx::OnOK();

}
