#pragma once


// CDatasetSaveOptionDlg 对话框

class CDatasetSaveOptionDlg : public CDialogEx
{
	DECLARE_DYNAMIC(CDatasetSaveOptionDlg)
private:
	int m_nMax;

public:
	CDatasetSaveOptionDlg(int nMax, CWnd* pParent = NULL);   // 标准构造函数
	virtual ~CDatasetSaveOptionDlg();

// 对话框数据
#ifdef AFX_DESIGN_TIME
	enum { IDD = IDD_SAVE_DATASET_OPTION };
#endif

protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV 支持

	DECLARE_MESSAGE_MAP()
public:
	int m_nFrom;
	int m_nTo;
	virtual void OnOK();
	BOOL m_bReverseOrder;
};
