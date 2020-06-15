#pragma once


// CWorldPointDlg 对话框

class CWorldPointDlg : public CDialogEx
{
	DECLARE_DYNAMIC(CWorldPointDlg)

public:
	CWorldPointDlg(CWnd* pParent = nullptr);   // 标准构造函数
	virtual ~CWorldPointDlg();

// 对话框数据
#ifdef AFX_DESIGN_TIME
	enum { IDD = IDD_WORLD_POINT };
#endif

protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV 支持

	DECLARE_MESSAGE_MAP()
public:
	float m_fX;
	float m_fY;
	virtual BOOL OnInitDialog();
	afx_msg void OnTimer(UINT_PTR nIDEvent);
};
