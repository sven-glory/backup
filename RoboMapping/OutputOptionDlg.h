#pragma once


// COutputOptionDlg �Ի���

class COutputOptionDlg : public CDialogEx
{
	DECLARE_DYNAMIC(COutputOptionDlg)

public:
	COutputOptionDlg(bool bUseReflector, bool bUseLines, CWnd* pParent = NULL);   // ��׼���캯��
	virtual ~COutputOptionDlg();

// �Ի�������
#ifdef AFX_DESIGN_TIME
	enum { IDD = IDD_OUTPUT_OPTION };
#endif

protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV ֧��

	DECLARE_MESSAGE_MAP()
public:
	BOOL m_bUseReflectors;
	BOOL m_bUseLines;
	virtual void OnOK();
};
