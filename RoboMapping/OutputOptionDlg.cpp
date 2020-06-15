// OutputOptionDlg.cpp : ʵ���ļ�
//

#include "stdafx.h"
#include "RoboMapping.h"
#include "OutputOptionDlg.h"
#include "afxdialogex.h"


// COutputOptionDlg �Ի���

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


// COutputOptionDlg ��Ϣ�������


void COutputOptionDlg::OnOK()
{
	// TODO: �ڴ����ר�ô����/����û���

	CDialogEx::OnOK();

}
