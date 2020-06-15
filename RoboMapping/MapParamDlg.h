#pragma once


// CMapParamDlg 对话框

class CMapParamDlg : public CDialogEx
{
	DECLARE_DYNAMIC(CMapParamDlg)

public:
	CMapParamDlg(float fRangeX, float fRangeY, float fReso, CWnd* pParent = NULL);   // 标准构造函数
	virtual ~CMapParamDlg();

// 对话框数据
#ifdef AFX_DESIGN_TIME
	enum { IDD = IDD_OPTION };
#endif

protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV 支持

public:
	float m_fRangeX;
	float m_fRangeY;
	float m_fCellReso;

	DECLARE_MESSAGE_MAP()
	virtual void OnOK();
};
