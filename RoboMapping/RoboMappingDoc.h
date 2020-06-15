// 这段 MFC 示例源代码演示如何使用 MFC Microsoft Office Fluent 用户界面 
// (“Fluent UI”)。该示例仅供参考，
// 用以补充《Microsoft 基础类参考》和 
// MFC C++ 库软件随附的相关电子文档。  
// 复制、使用或分发 Fluent UI 的许可条款是单独提供的。  
// 若要了解有关 Fluent UI 许可计划的详细信息，请访问 
// http://go.microsoft.com/fwlink/?LinkId=238214。
//
// 版权所有(C) Microsoft Corporation
// 保留所有权利。

// RoboMappingDoc.h : CRoboMappingDoc 类的接口
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

protected: // 仅从序列化创建
	CRoboMappingDoc();
	DECLARE_DYNCREATE(CRoboMappingDoc)

// 特性
public:
	int   m_nSlamMode;           // SLAM浏览的模式: 0-步进; 1-全速自动
	int   m_nCountSteps;
	int   m_nSlamTargetStep;     // SLAM浏览中的执行目标步数

	CScreenReference m_ScrnRef;
	CPnt m_ptCenter;
	bool     m_bDatasetLoaded;
	bool     m_bStepConfirmed;

// 操作
public:
	bool LoadDataset();

// 重写
public:
	virtual BOOL OnNewDocument();
	virtual void Serialize(CArchive& ar);
#ifdef SHARED_HANDLERS
	virtual void InitializeSearchContent();
	virtual void OnDrawThumbnail(CDC& dc, LPRECT lprcBounds);
#endif // SHARED_HANDLERS

// 实现
public:
	virtual ~CRoboMappingDoc();
#ifdef _DEBUG
	virtual void AssertValid() const;
	virtual void Dump(CDumpContext& dc) const;
#endif

protected:

// 生成的消息映射函数
protected:
	DECLARE_MESSAGE_MAP()

#ifdef SHARED_HANDLERS
	// 用于为搜索处理程序设置搜索内容的 Helper 函数
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
