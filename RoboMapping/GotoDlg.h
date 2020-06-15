#pragma once


// CGotoDlg 对话框

class CGotoDlg : public CDialogEx
{
	DECLARE_DYNAMIC(CGotoDlg)

private:
	CString m_strNote;

public:
	CGotoDlg(int nMaxStep, CString strNote, CWnd* pParent = NULL);   // 标准构造函数
	virtual ~CGotoDlg();

// 对话框数据
#ifdef AFX_DESIGN_TIME
	enum { IDD = IDD_GOTO_DLG };
#endif

protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV 支持

	DECLARE_MESSAGE_MAP()
public:
	int m_nStep;
	virtual BOOL OnInitDialog();
};
