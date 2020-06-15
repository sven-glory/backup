#pragma once

#include "ScannerParam.h"

// CScannerParamDlg �Ի���

class CScannerParamDlg : public CDialogEx
{
	DECLARE_DYNAMIC(CScannerParamDlg)
private:
	int m_nCurIdx;

public:
	CScannerGroupParam m_Param;

private:
	void Init();
	void Update();

public:
	CScannerParamDlg(CScannerGroupParam* pParam, CWnd* pParent = NULL);   // ��׼���캯��
	virtual ~CScannerParamDlg();

// �Ի�������
#ifdef AFX_DESIGN_TIME
	enum { IDD = IDD_SCANNER_PARAM_DLG };
#endif

protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV ֧��

	DECLARE_MESSAGE_MAP()
public:
	virtual BOOL OnInitDialog();
	float m_fInstallX;
	float m_fInstallY;
	float m_fInstallAngle;
	float m_fAppStartAngle;
	float m_fAppEndAngle;
	virtual void OnOK();
	CComboBox m_comboScanner;
	afx_msg void OnCbnSelchangeScanner();
	BOOL m_bUseWhenCreateModel;
	BOOL m_bUseWhenLocalize;
};
