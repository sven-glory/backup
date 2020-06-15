#pragma once


// CWaitBusyDlg dialog

class CWaitBusyDlg : public CDialogEx
{
	DECLARE_DYNAMIC(CWaitBusyDlg)
private:
	HWND m_hParentWnd;

public:
	CWaitBusyDlg(CWnd* pParent = NULL);   // standard constructor
	virtual ~CWaitBusyDlg();

// Dialog Data
#ifdef AFX_DESIGN_TIME
	enum { IDD = IDD_WAIT_BUSY };
#endif

protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV support

	DECLARE_MESSAGE_MAP()
public:
	afx_msg void OnTimer(UINT_PTR nIDEvent);
	virtual BOOL OnInitDialog();
};
