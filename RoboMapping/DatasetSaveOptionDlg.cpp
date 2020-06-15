// DatasetSaveOptionDlg.cpp : ʵ���ļ�
//

#include "stdafx.h"
#include "RoboMapping.h"
#include "DatasetSaveOptionDlg.h"
#include "afxdialogex.h"


// CDatasetSaveOptionDlg �Ի���

IMPLEMENT_DYNAMIC(CDatasetSaveOptionDlg, CDialogEx)

CDatasetSaveOptionDlg::CDatasetSaveOptionDlg(int nMax, CWnd* pParent /*=NULL*/)
	: CDialogEx(IDD_SAVE_DATASET_OPTION, pParent)
	, m_nFrom(1)
	, m_nTo(nMax)
	, m_bReverseOrder(FALSE)
{

}

CDatasetSaveOptionDlg::~CDatasetSaveOptionDlg()
{
}

void CDatasetSaveOptionDlg::DoDataExchange(CDataExchange* pDX)
{
	CDialogEx::DoDataExchange(pDX);
	DDX_Text(pDX, IDC_FROM, m_nFrom);
	DDX_Text(pDX, IDC_TO, m_nTo);
	DDX_Check(pDX, IDC_REVERSE_ORDER, m_bReverseOrder);
}


BEGIN_MESSAGE_MAP(CDatasetSaveOptionDlg, CDialogEx)
END_MESSAGE_MAP()


// CDatasetSaveOptionDlg ��Ϣ�������


void CDatasetSaveOptionDlg::OnOK()
{
	// TODO: �ڴ����ר�ô����/����û���

	CDialogEx::OnOK();
}
