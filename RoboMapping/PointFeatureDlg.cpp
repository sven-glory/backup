// PointFeatureDlg.cpp : 实现文件
//

#include "stdafx.h"
#include "RoboMapping.h"
#include "PointFeatureDlg.h"
#include "afxdialogex.h"


// CPointFeatureDlg 对话框

IMPLEMENT_DYNAMIC(CPointFeatureDlg, CDialogEx)

CPointFeatureDlg::CPointFeatureDlg(int nIdx, CPointFeature* pFeature, CWnd* pParent /*=NULL*/)
	: CDialogEx(IDD_POINT_FEATURE, pParent)
	, m_fX(0)
	, m_fY(0)
	, m_nIdx(0)
{
	m_nIdx = nIdx;
	m_pFeature = pFeature;
	m_fX = m_pFeature->x;
	m_fY = m_pFeature->y;
}

CPointFeatureDlg::~CPointFeatureDlg()
{
}

void CPointFeatureDlg::DoDataExchange(CDataExchange* pDX)
{
	CDialogEx::DoDataExchange(pDX);
	DDX_Text(pDX, IDC_POINT_X, m_fX);
	DDX_Text(pDX, IDC_POINT_Y, m_fY);
	DDX_Text(pDX, IDC_ID, m_nIdx);
}


BEGIN_MESSAGE_MAP(CPointFeatureDlg, CDialogEx)
END_MESSAGE_MAP()


// CPointFeatureDlg 消息处理程序


BOOL CPointFeatureDlg::OnInitDialog()
{
	CDialogEx::OnInitDialog();

	// TODO:  在此添加额外的初始化

	return TRUE;  // return TRUE unless you set the focus to a control
					  // 异常: OCX 属性页应返回 FALSE
}

void CPointFeatureDlg::OnOK()
{
	// TODO: 在此添加专用代码和/或调用基类

	CDialogEx::OnOK();

	m_pFeature->x = m_fX;
	m_pFeature->y = m_fY;
}
