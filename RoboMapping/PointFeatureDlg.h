#pragma once

#include "PointFeature.h"

// CPointFeatureDlg �Ի���

class CPointFeatureDlg : public CDialogEx
{
	DECLARE_DYNAMIC(CPointFeatureDlg)
private:
//	int            m_nIdx;
	CPointFeature* m_pFeature;

public:
	CPointFeatureDlg(int nIdx, CPointFeature* pFeature, CWnd* pParent = NULL);   // ��׼���캯��
	virtual ~CPointFeatureDlg();

// �Ի�������
#ifdef AFX_DESIGN_TIME
	enum { IDD = IDD_POINT_FEATURE };
#endif

protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV ֧��

	DECLARE_MESSAGE_MAP()
public:
	virtual BOOL OnInitDialog();
	virtual void OnOK();
	float m_fX;
	float m_fY;
	int m_nIdx;
};
