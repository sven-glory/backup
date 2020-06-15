#pragma once


// CTransformDlg dialog

class CTransformDlg : public CDialogEx
{
	DECLARE_DYNAMIC(CTransformDlg)

public:
	CTransformDlg(CWnd* pParent = NULL);   // standard constructor
	virtual ~CTransformDlg();

// Dialog Data
#ifdef AFX_DESIGN_TIME
	enum { IDD = IDD_TRANSFORM };
#endif

protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV support

	DECLARE_MESSAGE_MAP()
public:
	float m_fX;
	float m_fY;
	float m_fThita;
	virtual BOOL OnInitDialog();
	virtual void OnOK();
};
