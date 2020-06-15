// GotoDlg.cpp : ʵ���ļ�
//

#include "stdafx.h"
#include "RoboMapping.h"
#include "GotoDlg.h"
#include "afxdialogex.h"


// CGotoDlg �Ի���

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


// CGotoDlg ��Ϣ�������


BOOL CGotoDlg::OnInitDialog()
{
	CDialogEx::OnInitDialog();

	// TODO:  �ڴ���Ӷ���ĳ�ʼ��
	GetDlgItem(IDC_NOTE)->SetWindowText(m_strNote);

	return TRUE;  // return TRUE unless you set the focus to a control
					  // �쳣: OCX ����ҳӦ���� FALSE
}
