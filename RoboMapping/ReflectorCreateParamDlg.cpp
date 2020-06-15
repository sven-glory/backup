// ReflectorCreateParamDlg.cpp : ʵ���ļ�
//

#include "stdafx.h"
#include "RoboMapping.h"
#include "ReflectorCreateParamDlg.h"
#include "afxdialogex.h"


// CReflectorCreateParamDlg �Ի���

IMPLEMENT_DYNAMIC(CReflectorCreateParamDlg, CDialogEx)

CReflectorCreateParamDlg::CReflectorCreateParamDlg(CReflectorCreationParam* pParam, CWnd* pParent /*=NULL*/)
	: CDialogEx(IDD_REF_CREATE_PARAM, pParent)
	, m_fMinDistBetweenRef(0)
{
	m_pParam = pParam;

	m_nIntensityGate = pParam->nMinReflectorIntensity;
	m_fMaxSize = pParam->fMaxReflectorSize;
	m_fMaxRadiusVariance = pParam->fMaxPolarRadiusVariance;
	m_fMinDistBetweenRef = pParam->fMinDistBetweenReflectors;
}

CReflectorCreateParamDlg::~CReflectorCreateParamDlg()
{
}

void CReflectorCreateParamDlg::DoDataExchange(CDataExchange* pDX)
{
	CDialogEx::DoDataExchange(pDX);
	DDX_Text(pDX, IDC_REF_INTENSITY_GATE, m_nIntensityGate);
	DDX_Text(pDX, IDC_REF_MAX_SIZE, m_fMaxSize);
	DDX_Text(pDX, IDC_REF_MAX_RADIUS_VARIANCE, m_fMaxRadiusVariance);
	DDX_Text(pDX, IDC_MIN_DIST_BETWEEN_REF, m_fMinDistBetweenRef);
}


BEGIN_MESSAGE_MAP(CReflectorCreateParamDlg, CDialogEx)
END_MESSAGE_MAP()


// CReflectorCreateParamDlg ��Ϣ�������


void CReflectorCreateParamDlg::OnOK()
{
	// TODO: �ڴ����ר�ô����/����û���
	CDialogEx::OnOK();

	m_pParam->nMinReflectorIntensity = m_nIntensityGate;
	m_pParam->fMaxReflectorSize = m_fMaxSize;
	m_pParam->fMaxPolarRadiusVariance = m_fMaxRadiusVariance;
	m_pParam->fMinDistBetweenReflectors = m_fMinDistBetweenRef;
}
