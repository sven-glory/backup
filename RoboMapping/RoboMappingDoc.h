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

// RoboMappingDoc.h : CRoboMappingDoc ��Ľӿ�
//


#pragma once

#include "ScrnRef.h"

class CRoboMappingDoc : public CDocument
{
private:
	CString m_strPathName;
	float   m_fMapSizeX;
	float   m_fMapSizeY;
	float   m_fMapReso;

private:
	bool LoadConfigParam();
	bool SaveConfigParam();

	bool LoadMapParam();
	bool SaveMapParam();

protected: // �������л�����
	CRoboMappingDoc();
	DECLARE_DYNCREATE(CRoboMappingDoc)

// ����
public:
	int   m_nSlamMode;           // SLAM�����ģʽ: 0-����; 1-ȫ���Զ�
	int   m_nCountSteps;
	int   m_nSlamTargetStep;     // SLAM����е�ִ��Ŀ�경��

	CScreenReference m_ScrnRef;
	CPnt m_ptCenter;
	bool     m_bDatasetLoaded;
	bool     m_bStepConfirmed;

// ����
public:
	bool LoadDataset();

// ��д
public:
	virtual BOOL OnNewDocument();
	virtual void Serialize(CArchive& ar);
#ifdef SHARED_HANDLERS
	virtual void InitializeSearchContent();
	virtual void OnDrawThumbnail(CDC& dc, LPRECT lprcBounds);
#endif // SHARED_HANDLERS

// ʵ��
public:
	virtual ~CRoboMappingDoc();
#ifdef _DEBUG
	virtual void AssertValid() const;
	virtual void Dump(CDumpContext& dc) const;
#endif

protected:

// ���ɵ���Ϣӳ�亯��
protected:
	DECLARE_MESSAGE_MAP()

#ifdef SHARED_HANDLERS
	// ����Ϊ����������������������ݵ� Helper ����
	void SetSearchContent(const CString& value);
#endif // SHARED_HANDLERS
public:
	afx_msg void OnLoadDataset();
	afx_msg void OnQuickBuild();
	afx_msg void OnEndBuild();
	afx_msg void OnUpdateQuickBuild(CCmdUI *pCmdUI);
	afx_msg void OnUpdateEndBuild(CCmdUI *pCmdUI);
	afx_msg void OnUpdateLoadDataset(CCmdUI *pCmdUI);
	afx_msg void OnSaveDataset();
	afx_msg void OnStepBuild();
	afx_msg void OnUpdateStepBuild(CCmdUI *pCmdUI);
	afx_msg void OnRefFeatureParam();
	afx_msg void OnScannersParam();
	afx_msg void OnDatasetFilter();
	afx_msg void OnMapParam();
};
