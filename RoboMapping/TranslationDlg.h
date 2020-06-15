#pragma once


// CTranslationDlg 对话框

class CTranslationDlg : public CDialogEx
{
	DECLARE_DYNAMIC(CTranslationDlg)

public:
	CTranslationDlg(CWnd* pParent = NULL);   // 标准构造函数
	virtual ~CTranslationDlg();

// 对话框数据
#ifdef AFX_DESIGN_TIME
	enum { IDD = IDD_TRANSLATION };
#endif

protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV 支持

	DECLARE_MESSAGE_MAP()
public:
	float m_fX;
	float m_fY;
};
