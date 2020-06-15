#pragma once

#include "SidePath.h"
#include "ScpTraj.h"

//////////////////////////////////////////////////////////////////////////////
//   The interface of class "CScpPath".
class DllExport CScpPath : public CSidePath
{
private:
	CScpTraj* m_pTraj;

public:
	// The default constructor
	CScpPath();
	~CScpPath();

	// Size caculation function
	virtual float SizeFun();

	// Make a trajectory from the path
	virtual CTraj* MakeTraj();

	virtual bool Create(FILE *StreamIn);

#ifdef _MFC_VER
	virtual bool Create(CArchive& ar);

	virtual void Draw(CScreenReference& ScrnRef, CDC* pDC, COLORREF cr, int nWidth = 1);
#endif
};
