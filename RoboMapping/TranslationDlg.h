#pragma once


// CTranslationDlg �Ի���

class CTranslationDlg : public CDialogEx
{
	DECLARE_DYNAMIC(CTranslationDlg)

public:
	CTranslationDlg(CWnd* pParent = NULL);   // ��׼���캯��
	virtual ~CTranslationDlg();

// �Ի�������
#ifdef AFX_DESIGN_TIME
	enum { IDD = IDD_TRANSLATION };
#endif

protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV ֧��

	DECLARE_MESSAGE_MAP()
public:
	float m_fX;
	float m_fY;
};
