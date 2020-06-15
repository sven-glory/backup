// PointFeatureDlg.cpp : ʵ���ļ�
//

#include "stdafx.h"
#include "RoboMapping.h"
#include "PointFeatureDlg.h"
#include "afxdialogex.h"


// CPointFeatureDlg �Ի���

IMPLEMENT_DYNAMIC(CPointFeatureDlg, CDialogEx)

CPointFeatureDlg::CPointFeatureDlg(int nIdx, CPointFeature* pFeature, CWnd* pParent /*=NULL*/)
	: CDialogEx(IDD_POINT_FEATURE, pParent)
	, m_fX(0)
	, m_fY(0)
	, m_nIdx(0)
{
	m_nIdx = nIdx;
	m_pFeature = pFeature;
	m_fX = m_pFeature->x;
	m_fY = m_pFeature->y;
}

CPointFeatureDlg::~CPointFeatureDlg()
{
}

void CPointFeatureDlg::DoDataExchange(CDataExchange* pDX)
{
	CDialogEx::DoDataExchange(pDX);
	DDX_Text(pDX, IDC_POINT_X, m_fX);
	DDX_Text(pDX, IDC_POINT_Y, m_fY);
	DDX_Text(pDX, IDC_ID, m_nIdx);
}


BEGIN_MESSAGE_MAP(CPointFeatureDlg, CDialogEx)
END_MESSAGE_MAP()


// CPointFeatureDlg ��Ϣ�������


BOOL CPointFeatureDlg::OnInitDialog()
{
	CDialogEx::OnInitDialog();

	// TODO:  �ڴ���Ӷ���ĳ�ʼ��

	return TRUE;  // return TRUE unless you set the focus to a control
					  // �쳣: OCX ����ҳӦ���� FALSE
}

void CPointFeatureDlg::OnOK()
{
	// TODO: �ڴ����ר�ô����/����û���

	CDialogEx::OnOK();

	m_pFeature->x = m_fX;
	m_pFeature->y = m_fY;
}
