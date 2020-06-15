// MapParamDlg.cpp: 实现文件
//

#include "stdafx.h"
#include "RoboMapping.h"
#include "MapParamDlg.h"
#include "afxdialogex.h"


// CMapParamDlg 对话框

IMPLEMENT_DYNAMIC(CMapParamDlg, CDialogEx)

CMapParamDlg::CMapParamDlg(float fRangeX, float fRangeY, float fReso, CWnd* pParent /*=nullptr*/)
	: CDialogEx(IDD_OPTION, pParent)
{
	m_fRangeX = fRangeX;
	m_fRangeY = fRangeY;
	m_fCellReso = fReso;
}

CMapParamDlg::~CMapParamDlg()
{
}

void CMapParamDlg::DoDataExchange(CDataExchange* pDX)
{
	CDialogEx::DoDataExchange(pDX);
	DDX_Text(pDX, IDC_MAP_RANGE_X, m_fRangeX);
	DDX_Text(pDX, IDC_MAP_RANGE_Y, m_fRangeY);
	DDX_Text(pDX, IDC_CELL_RESO, m_fCellReso);
}


BEGIN_MESSAGE_MAP(CMapParamDlg, CDialogEx)
END_MESSAGE_MAP()


// CMapParamDlg 消息处理程序


void CMapParamDlg::OnOK()
{
	// TODO: 在此添加专用代码和/或调用基类

	CDialogEx::OnOK();
}
