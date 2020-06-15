#pragma once


// CLinkDatasetDlg 对话框

class CLinkDatasetDlg : public CDialogEx
{
	DECLARE_DYNAMIC(CLinkDatasetDlg)
public:
	CString m_strPathName;

public:
	CLinkDatasetDlg(float x, float y, float thita, CWnd* pParent = nullptr);   // 标准构造函数
	virtual ~CLinkDatasetDlg();

// 对话框数据
#ifdef AFX_DESIGN_TIME
	enum { IDD = IDD_LINK_DATASET_DLG };
#endif

protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV 支持

	DECLARE_MESSAGE_MAP()
public:
	float m_fX;
	float m_fY;
	float m_fThita;
	afx_msg void OnBnClickedBrowse();
	virtual void OnOK();
};
