// WorldPointDlg.cpp: 实现文件
//

#include "stdafx.h"
#include "RoboMapping.h"
#include "WorldPointDlg.h"
#include "afxdialogex.h"


// CWorldPointDlg 对话框

IMPLEMENT_DYNAMIC(CWorldPointDlg, CDialogEx)

CWorldPointDlg::CWorldPointDlg(CWnd* pParent /*=nullptr*/)
	: CDialogEx(IDD_WORLD_POINT, pParent)
	, m_fX(0)
	, m_fY(0)
{

}

CWorldPointDlg::~CWorldPointDlg()
{
}

void CWorldPointDlg::DoDataExchange(CDataExchange* pDX)
{
	CDialogEx::DoDataExchange(pDX);
	DDX_Text(pDX, IDC_X, m_fX);
	DDX_Text(pDX, IDC_Y, m_fY);
}


BEGIN_MESSAGE_MAP(CWorldPointDlg, CDialogEx)
	ON_WM_TIMER()
END_MESSAGE_MAP()


// CWorldPointDlg 消息处理程序


BOOL CWorldPointDlg::OnInitDialog()
{
	CDialogEx::OnInitDialog();

	// TODO:  在此添加额外的初始化
	SetTimer(2, 100, NULL);
	return TRUE;  // return TRUE unless you set the focus to a control
					  // 异常: OCX 属性页应返回 FALSE
}


void CWorldPointDlg::OnTimer(UINT_PTR nIDEvent)
{
	// TODO: 在此添加消息处理程序代码和/或调用默认值
	if (nIDEvent == 2)
	{

	}

	CDialogEx::OnTimer(nIDEvent);
}
