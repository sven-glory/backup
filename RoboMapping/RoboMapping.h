// ��� MFC ʾ��Դ������ʾ���ʹ�� MFC Microsoft Office Fluent �û����� 
// (��Fluent UI��)����ʾ�������ο���
// ���Բ��䡶Microsoft ������ο����� 
// MFC C++ ������渽����ص����ĵ���  
// ���ơ�ʹ�û�ַ� Fluent UI ����������ǵ����ṩ�ġ�  
// ��Ҫ�˽��й� Fluent UI ��ɼƻ�����ϸ��Ϣ������� 
// http://go.microsoft.com/fwlink/?LinkId=238214��
//
// ��Ȩ����(C) Microsoft Corporation
// ��������Ȩ����

// RoboMapping.h : RoboMapping Ӧ�ó������ͷ�ļ�
//
#pragma once

#ifndef __AFXWIN_H__
	#error "�ڰ������ļ�֮ǰ������stdafx.h�������� PCH �ļ�"
#endif

#include "resource.h"       // ������


// CRoboMappingApp:
// �йش����ʵ�֣������ RoboMapping.cpp
//

//#include "RoboMappingView.h"
//#include "ScanView.h"

class CRoboMappingView;
class CScanView;

class CRoboMappingApp : public CWinAppEx
{
private:
	CMultiDocTemplate* m_pMappingViewTemplate;
	CMultiDocTemplate* m_pScanViewTemplate;

public:
	CRoboMappingView* m_pMappingView;
	CScanView*        m_pScanView;

public:
	CRoboMappingApp();

	BOOL CreateScanViewTemplate();

	CDocTemplate * GetMappingViewTemplate() const;
	CDocTemplate * GetScanViewTemplate() const;

// ��д
public:
	virtual BOOL InitInstance();
	virtual int ExitInstance();

// ʵ��
	UINT  m_nAppLook;
	virtual void PreLoadState();
	virtual void LoadCustomState();
	virtual void SaveCustomState();

	afx_msg void OnAppAbout();
	DECLARE_MESSAGE_MAP()
};

extern CRoboMappingApp theApp;
