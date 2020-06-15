// WaitBusyDlg.cpp : implementation file
//

#include "stdafx.h"
#include "RoboMapping.h"
#include "WaitBusyDlg.h"
#include "afxdialogex.h"

#define WM_SIMPLIFY_START                  (WM_USER+1)

// CWaitBusyDlg dialog

IMPLEMENT_DYNAMIC(CWaitBusyDlg, CDialogEx)

CWaitBusyDlg::CWaitBusyDlg(CWnd* pParent)
	: CDialogEx(IDD_WAIT_BUSY, pParent)
{
	m_hParentWnd = pParent->m_hWnd;
}

CWaitBusyDlg::~CWaitBusyDlg()
{
}

void CWaitBusyDlg::DoDataExchange(CDataExchange* pDX)
{
	CDialogEx::DoDataExchange(pDX);
}


BEGIN_MESSAGE_MAP(CWaitBusyDlg, CDialogEx)
	ON_WM_TIMER()
END_MESSAGE_MAP()


// CWaitBusyDlg message handlers


BOOL CWaitBusyDlg::OnInitDialog()
{
	CDialogEx::OnInitDialog();

	// TODO:  Add extra initialization here
	SetTimer(2, 100, NULL);

	return TRUE;  // return TRUE unless you set the focus to a control
					  // EXCEPTION: OCX Property Pages should return FALSE
}

void CWaitBusyDlg::OnTimer(UINT_PTR nIDEvent)
{
	// TODO: Add your message handler code here and/or call default
	if (nIDEvent == 2)
	{
		::PostMessage(m_hParentWnd, WM_SIMPLIFY_START, 0, 0);
		KillTimer(2);
	}

	CDialogEx::OnTimer(nIDEvent);
}

