// CLinkDatasetDlg.cpp: 实现文件
//

#include "stdafx.h"
#include "RoboMapping.h"
#include "CLinkDatasetDlg.h"
#include "afxdialogex.h"

#include "SlamDataSet.h"

// CLinkDatasetDlg 对话框

IMPLEMENT_DYNAMIC(CLinkDatasetDlg, CDialogEx)

///////////////////////////////////////////////////////////////////////////////

CLinkDatasetDlg::CLinkDatasetDlg(float x, float y, float thita, CWnd* pParent /*=nullptr*/)
	: CDialogEx(IDD_LINK_DATASET_DLG, pParent)
	, m_fX(x)
	, m_fY(y)
	, m_fThita(thita)
{

}

CLinkDatasetDlg::~CLinkDatasetDlg()
{
}

void CLinkDatasetDlg::DoDataExchange(CDataExchange* pDX)
{
	CDialogEx::DoDataExchange(pDX);
	DDX_Text(pDX, IDC_X, m_fX);
	DDX_Text(pDX, IDC_Y, m_fY);
	DDX_Text(pDX, IDC_THITA, m_fThita);
}


BEGIN_MESSAGE_MAP(CLinkDatasetDlg, CDialogEx)
	ON_BN_CLICKED(IDC_BROWSE, &CLinkDatasetDlg::OnBnClickedBrowse)
END_MESSAGE_MAP()


// CLinkDatasetDlg 消息处理程序

void CLinkDatasetDlg::OnBnClickedBrowse()
{
	CFileDialog Dlg(TRUE, _T(".dx"), 0, OFN_HIDEREADONLY | OFN_READONLY, _T("文件 (*.dx)|*.dx||", NULL));
	if (Dlg.DoModal() == IDOK)
	{
		m_strPathName = Dlg.GetPathName();
		GetDlgItem(IDC_DATASET_MERGED)->SetWindowText(m_strPathName);
	}
}

void CLinkDatasetDlg::OnOK()
{
	// TODO: 在此添加专用代码和/或调用基类
	CDialogEx::OnOK();
	GetDlgItem(IDC_DATASET_MERGED)->GetWindowText(m_strPathName);
}
