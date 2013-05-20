// CPCreation.cpp : implementation file
//

#include "stdafx.h"
#include "../KWResearchWork.h"
#include "CPCreation.h"
#include "ControlPanel.h"

// CCPCreation dialog

IMPLEMENT_DYNAMIC(CCPCreation, CDialog)

CCPCreation::CCPCreation(CWnd* pParent /*=NULL*/)
	: CDialog(CCPCreation::IDD, pParent)
{

}

CCPCreation::~CCPCreation()
{
}

CCPCreation::CCPCreation(CKWResearchWorkDoc * docDataIn,CControlPanel * cpDataIn)
{
	this->pDoc=docDataIn;
	this->pCP=cpDataIn;
}

void CCPCreation::DoDataExchange(CDataExchange* pDX)
{
	CDialog::DoDataExchange(pDX);
}


BEGIN_MESSAGE_MAP(CCPCreation, CDialog)
	ON_BN_CLICKED(IDC_CR_ReadContour, &CCPCreation::OnBnClickedCrReadcontour)
	ON_BN_CLICKED(IDC_CR_WriteContour, &CCPCreation::OnBnClickedCrWritecontour)
	ON_BN_CLICKED(IDC_CR_AdjustContourView, &CCPCreation::OnBnClickedCrAdjustcontourview)
	ON_BN_CLICKED(IDC_CR_GenerateMesh, &CCPCreation::OnBnClickedCrGeneratemesh)
	ON_BN_CLICKED(IDC_MOD_ALGO_TJ, &CCPCreation::OnBnClickedModAlgo)
END_MESSAGE_MAP()


// CCPCreation message handlers
void CCPCreation::Init()
{
	//render ref plane in creation mode
	if (pDoc->GetMeshCreation().GetRenderRefPlane()[0])
	{
		CButton*   m_Check=(CButton*)this->GetDlgItem(IDC_CR_BLUEPLANE);
		m_Check->SetCheck(BST_CHECKED);
	}
	else
	{
		CButton*   m_Check=(CButton*)this->GetDlgItem(IDC_CR_BLUEPLANE);
		m_Check->SetCheck(BST_UNCHECKED);
	}
	if (pDoc->GetMeshCreation().GetRenderRefPlane()[1])
	{
		CButton*   m_Check=(CButton*)this->GetDlgItem(IDC_CR_GREENPLANE);
		m_Check->SetCheck(BST_CHECKED);
	}
	else
	{
		CButton*   m_Check=(CButton*)this->GetDlgItem(IDC_CR_GREENPLANE);
		m_Check->SetCheck(BST_UNCHECKED);
	}
	if (pDoc->GetMeshCreation().GetRenderRefPlane()[2])
	{
		CButton*   m_Check=(CButton*)this->GetDlgItem(IDC_CR_REDPLANE);
		m_Check->SetCheck(BST_CHECKED);
	}
	else
	{
		CButton*   m_Check=(CButton*)this->GetDlgItem(IDC_CR_REDPLANE);
		m_Check->SetCheck(BST_UNCHECKED);
	}
	if (pDoc->GetMeshCreation().GetRenderCN())
	{
		CButton*   m_Check=(CButton*)this->GetDlgItem(IDC_CR_CN);
		m_Check->SetCheck(BST_CHECKED);
	}
	else
	{
		CButton*   m_Check=(CButton*)this->GetDlgItem(IDC_CR_CN);
		m_Check->SetCheck(BST_UNCHECKED);
	}
	if (pDoc->GetMeshCreation().GetRenderOnlyUserSketch())
	{
		CButton*   m_Check=(CButton*)this->GetDlgItem(IDC_CR_US);
		m_Check->SetCheck(BST_CHECKED);
	}
	else
	{
		CButton*   m_Check=(CButton*)this->GetDlgItem(IDC_CR_US);
		m_Check->SetCheck(BST_UNCHECKED);
	}
	if (pDoc->GetMeshCreation().GetAutoRotState())
	{
		CButton*   m_Check=(CButton*)this->GetDlgItem(IDC_CR_AUTOROT);
		m_Check->SetCheck(BST_CHECKED);
	}
	else
	{
		CButton*   m_Check=(CButton*)this->GetDlgItem(IDC_CR_AUTOROT);
		m_Check->SetCheck(BST_UNCHECKED);
	}
	UpdateData(FALSE);
}

BOOL CCPCreation::OnCommand(WPARAM wParam, LPARAM lParam)
{
	// TODO: Add your specialized code here and/or call the base class
	WORD   wID   =   LOWORD(   wParam   );  
	WORD   wNF   =   HIWORD(   wParam   );  
	if(wID ==IDC_CR_BLUEPLANE && wNF == BN_CLICKED)  
	{  
		CButton*   m_Check=(CButton*)this->GetDlgItem(IDC_CR_BLUEPLANE);
		int iState=m_Check->GetCheck();
		if (m_Check->GetCheck()==BST_CHECKED)
		{
			pDoc->GetMeshCreation().GetRenderRefPlane()[0]=true;
		} 
		else
		{
			pDoc->GetMeshCreation().GetRenderRefPlane()[0]=false;
		}
	}  
	else if(wID ==IDC_CR_GREENPLANE && wNF == BN_CLICKED)  
	{  
		CButton*   m_Check=(CButton*)this->GetDlgItem(IDC_CR_GREENPLANE);
		if (m_Check->GetCheck()==BST_CHECKED)
		{
			pDoc->GetMeshCreation().GetRenderRefPlane()[1]=true;
		} 
		else
		{
			pDoc->GetMeshCreation().GetRenderRefPlane()[1]=false;
		}
	}
	else if(wID ==IDC_CR_REDPLANE && wNF == BN_CLICKED)  
	{  
		CButton*   m_Check=(CButton*)this->GetDlgItem(IDC_CR_REDPLANE);
		if (m_Check->GetCheck()==BST_CHECKED)
		{
			pDoc->GetMeshCreation().GetRenderRefPlane()[2]=true;
		} 
		else
		{
			pDoc->GetMeshCreation().GetRenderRefPlane()[2]=false;
		}
	}
	else if (wID ==IDC_CR_CN && wNF == BN_CLICKED)
	{
		CButton*   m_Check=(CButton*)this->GetDlgItem(IDC_CR_CN);
		if (m_Check->GetCheck()==BST_CHECKED)
		{
			pDoc->GetMeshCreation().SetRenderCN(true);
		} 
		else
		{
			pDoc->GetMeshCreation().SetRenderCN(false);
		}
	}
	else if (wID ==IDC_CR_US && wNF == BN_CLICKED)
	{
		CButton*   m_Check=(CButton*)this->GetDlgItem(IDC_CR_US);
		if (m_Check->GetCheck()==BST_CHECKED)
		{
			pDoc->GetMeshCreation().SetRenderOnlyUserSketch(true);
		} 
		else
		{
			pDoc->GetMeshCreation().SetRenderOnlyUserSketch(false);
		}
	}
	else if (wID ==IDC_CR_SS && wNF == BN_CLICKED)
	{
		if (pDoc->GetMeshCreation().GetCS2Surf()!=NULL)
		{
			CButton*   m_Check=(CButton*)this->GetDlgItem(IDC_CR_SS);
			if (m_Check->GetCheck()==BST_CHECKED)
			{
				pDoc->GetMeshCreation().GetCS2Surf()->SetRenderSS(true);
			} 
			else
			{
				pDoc->GetMeshCreation().GetCS2Surf()->SetRenderSS(false);
			}
		}
	}
	else if (wID ==IDC_CR_AUTOROT && wNF == BN_CLICKED)
	{
		CButton*   m_Check=(CButton*)this->GetDlgItem(IDC_CR_AUTOROT);
		if (m_Check->GetCheck()==BST_CHECKED)
		{
			pDoc->GetMeshCreation().SetAutoRotState(true);
		} 
		else
		{
			pDoc->GetMeshCreation().SetAutoRotState(false);
		}
	}
	pDoc->UpdateAllViews((CView*)pCP);
	return CDialog::OnCommand(wParam, lParam);
}

void CCPCreation::OnBnClickedCrReadcontour()
{
	// TODO: Add your control notification handler code here
	CWnd* pButton=this->GetDlgItem(IDC_CR_ReadContour);
	pButton->EnableWindow(FALSE);

	CFileDialog dlg(TRUE,".contour","*.contour",OFN_HIDEREADONLY,
		"Contour Files(*.CONTOUR)|*.contour||",AfxGetMainWnd());   
	if(dlg.DoModal() == IDOK)         
	{
		BeginWaitCursor();
		CString strPath = dlg.GetPathName();       

		char* pFileName=new char[MAX_PATH];
		memset(pFileName,0x00,MAX_PATH);
		memcpy(pFileName,strPath,strPath.GetLength()+1);
		pDoc->GetMeshCreation().ReadContourFromFile(pFileName);
		EndWaitCursor();
	}

	pDoc->UpdateAllViews((CView*)pCP);
	pButton->EnableWindow(TRUE);
}

void CCPCreation::OnBnClickedCrWritecontour()
{
	// TODO: Add your control notification handler code here
	CWnd* pButton=this->GetDlgItem(IDC_CR_WriteContour);
	pButton->EnableWindow(FALSE);


	CFileDialog dlg(FALSE,".contour","*.contour",OFN_HIDEREADONLY,
		"Contour Files(*.CONTOUR)|*.contour||",AfxGetMainWnd());   
	if(dlg.DoModal() == IDOK)         
	{
		BeginWaitCursor();
		CString strPath = dlg.GetPathName();       

		char* pFileName=new char[MAX_PATH];
		memset(pFileName,0x00,MAX_PATH);
		memcpy(pFileName,strPath,strPath.GetLength()+1);
		pDoc->GetMeshCreation().WriteContourToFile(pFileName);
		EndWaitCursor();
	}

	pDoc->UpdateAllViews((CView*)pCP);
	pButton->EnableWindow(TRUE);
}

void CCPCreation::OnBnClickedCrAdjustcontourview()
{
	// TODO: Add your control notification handler code here
	CWnd* pButton=this->GetDlgItem(IDC_CR_AdjustContourView);
	pButton->EnableWindow(FALSE);

	if (!pDoc->GetMesh().empty())
	{
		return;
	}
	pDoc->GetMeshCreation().AdjustContourView();

	pDoc->UpdateAllViews((CView*)pCP);
	pButton->EnableWindow(TRUE);
}

void CCPCreation::OnBnClickedCrGeneratemesh()
{
	// TODO: Add your control notification handler code here
	CWnd* pButton=this->GetDlgItem(IDC_CR_GenerateMesh);
	pButton->EnableWindow(FALSE);

//	pDoc->GetMeshCreation().GenerateMesh(pDoc->GetMesh(),pDoc->GetDefaultColor());
	pDoc->SetTitle("Untitled.obj");
	pDoc->SetModifiedFlag(TRUE);	


	//disable the preview checkbox
	pDoc->SetRenderPreMesh(MESH_EXIST_VIEW);
	//CWnd*   m_Check=(CCPGeneral*)(pCP->GetCPGeneral())->GetDlgItem(IDC_CR_PREMESH);
	//m_Check->EnableWindow(FALSE);
	
	//do not display the reference planes
	CButton*   m_CheckButton=(CButton*)this->GetDlgItem(IDC_CR_BLUEPLANE);
	m_CheckButton->SetCheck(BST_UNCHECKED);
	pDoc->GetMeshCreation().GetRenderRefPlane()[0]=false;
	m_CheckButton=(CButton*)this->GetDlgItem(IDC_CR_REDPLANE);
	m_CheckButton->SetCheck(BST_UNCHECKED);
	pDoc->GetMeshCreation().GetRenderRefPlane()[1]=false;
	m_CheckButton=(CButton*)this->GetDlgItem(IDC_CR_GREENPLANE);
	m_CheckButton->SetCheck(BST_UNCHECKED);
	pDoc->GetMeshCreation().GetRenderRefPlane()[2]=false;



	pDoc->UpdateAllViews((CView*)pCP);
	pButton->EnableWindow(TRUE);
}

void CCPCreation::OnBnClickedModAlgo()
{
	// TODO: Add your control notification handler code here
	//update the render subspace info
	if (pDoc->GetMeshCreation().GetCS2Surf()->GetRenderSS())
	{
		CButton*   m_Check=(CButton*)this->GetDlgItem(IDC_CR_SS);
		m_Check->SetCheck(BST_CHECKED);
	}
	else
	{
		CButton*   m_Check=(CButton*)this->GetDlgItem(IDC_CR_SS);
		m_Check->SetCheck(BST_UNCHECKED);
	}
	UpdateData(FALSE);
	pDoc->UpdateAllViews(NULL);
}


