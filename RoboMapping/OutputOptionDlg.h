#pragma once


// COutputOptionDlg 对话框

class COutputOptionDlg : public CDialogEx
{
	DECLARE_DYNAMIC(COutputOptionDlg)

public:
	COutputOptionDlg(bool bUseReflector, bool bUseLines, CWnd* pParent = NULL);   // 标准构造函数
	virtual ~COutputOptionDlg();

// 对话框数据
#ifdef AFX_DESIGN_TIME
	enum { IDD = IDD_OUTPUT_OPTION };
#endif

protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV 支持

	DECLARE_MESSAGE_MAP()
public:
	BOOL m_bUseReflectors;
	BOOL m_bUseLines;
	virtual void OnOK();
};
