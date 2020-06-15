#pragma once

#include "ReflFeatureCreateParam.h"

// CReflectorCreateParamDlg 对话框

class CReflectorCreateParamDlg : public CDialogEx
{
	DECLARE_DYNAMIC(CReflectorCreateParamDlg)

private:
	CReflectorCreationParam* m_pParam;

public:
	CReflectorCreateParamDlg(CReflectorCreationParam* pParam, CWnd* pParent = NULL);   // 标准构造函数
	virtual ~CReflectorCreateParamDlg();

// 对话框数据
#ifdef AFX_DESIGN_TIME
	enum { IDD = IDD_REF_CREATE_PARAM };
#endif

protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV 支持

	DECLARE_MESSAGE_MAP()
public:
	int m_nIntensityGate;
	float m_fMaxSize;
	float m_fMaxRadiusVariance;
	virtual void OnOK();
	float m_fMinDistBetweenRef;
};
