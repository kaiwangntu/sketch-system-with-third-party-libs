#pragma once
#include "../KWResearchWorkDoc.h"
#include "../KWResearchWorkView.h"
#include "afxwin.h"

class CControlPanel;
// CCPCreation dialog

class CCPCreation : public CDialog
{
	DECLARE_DYNAMIC(CCPCreation)

public:
	CCPCreation(CWnd* pParent = NULL);   // standard constructor
	virtual ~CCPCreation();

	CCPCreation(CKWResearchWorkDoc * docDataIn,CControlPanel * cpDataIn);


// Dialog Data
	enum { IDD = IDD_CP_Creation };

protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV support

	CKWResearchWorkDoc * pDoc;
	CControlPanel * pCP;


	DECLARE_MESSAGE_MAP()

protected:
	virtual BOOL OnCommand(WPARAM wParam, LPARAM lParam);

public:
	void Init();
	afx_msg void OnBnClickedCrReadcontour();
	afx_msg void OnBnClickedCrWritecontour();
	afx_msg void OnBnClickedCrAdjustcontourview();
	afx_msg void OnBnClickedCrGeneratemesh();
	afx_msg void OnBnClickedModAlgo();
};
