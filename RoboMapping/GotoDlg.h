#pragma once


// CGotoDlg �Ի���

class CGotoDlg : public CDialogEx
{
	DECLARE_DYNAMIC(CGotoDlg)

private:
	CString m_strNote;

public:
	CGotoDlg(int nMaxStep, CString strNote, CWnd* pParent = NULL);   // ��׼���캯��
	virtual ~CGotoDlg();

// �Ի�������
#ifdef AFX_DESIGN_TIME
	enum { IDD = IDD_GOTO_DLG };
#endif

protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV ֧��

	DECLARE_MESSAGE_MAP()
public:
	int m_nStep;
	virtual BOOL OnInitDialog();
};
