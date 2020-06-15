// TranslationDlg.cpp : 实现文件
//

#include "stdafx.h"
#include "RoboMapping.h"
#include "TranslationDlg.h"
#include "afxdialogex.h"


// CTranslationDlg 对话框

IMPLEMENT_DYNAMIC(CTranslationDlg, CDialogEx)

CTranslationDlg::CTranslationDlg(CWnd* pParent /*=NULL*/)
	: CDialogEx(IDD_TRANSLATION, pParent)
	, m_fX(0)
	, m_fY(0)
{

}

CTranslationDlg::~CTranslationDlg()
{
}

void CTranslationDlg::DoDataExchange(CDataExchange* pDX)
{
	CDialogEx::DoDataExchange(pDX);
	DDX_Text(pDX, IDC_X, m_fX);
	DDX_Text(pDX, IDC_Y, m_fY);
}


BEGIN_MESSAGE_MAP(CTranslationDlg, CDialogEx)
END_MESSAGE_MAP()


// CTranslationDlg 消息处理程序
