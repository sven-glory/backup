#pragma once

#include "ReflFeatureCreateParam.h"

// CReflectorCreateParamDlg �Ի���

class CReflectorCreateParamDlg : public CDialogEx
{
	DECLARE_DYNAMIC(CReflectorCreateParamDlg)

private:
	CReflectorCreationParam* m_pParam;

public:
	CReflectorCreateParamDlg(CReflectorCreationParam* pParam, CWnd* pParent = NULL);   // ��׼���캯��
	virtual ~CReflectorCreateParamDlg();

// �Ի�������
#ifdef AFX_DESIGN_TIME
	enum { IDD = IDD_REF_CREATE_PARAM };
#endif

protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV ֧��

	DECLARE_MESSAGE_MAP()
public:
	int m_nIntensityGate;
	float m_fMaxSize;
	float m_fMaxRadiusVariance;
	virtual void OnOK();
	float m_fMinDistBetweenRef;
};
