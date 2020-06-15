#pragma once

#include "Path.h"
#include "SideTraj.h"

//////////////////////////////////////////////////////////////////////////////
//   The interface of class "TSidePath" - a class defining side-type paths.
class DllExport CSidePath : public CPath
{
public:
	CAngle m_angHeading;     // The vehicle's heading angle on the path

public:
	// The default constructor
	CSidePath() {}

	// Get the vehicle's required heading at the specified node
	virtual CAngle& GetHeading(CNode& nd);

	// Size caculation function
	virtual float SizeFun();

	// Make a trajectory from the path
	virtual CTraj* MakeTraj();

	virtual bool Create(FILE *StreamIn);
	virtual bool Save(FILE *StreamOut);

#ifdef _MFC_VER
	virtual bool Create(CArchive& ar);
	virtual bool Save(CArchive& ar);

	virtual void Draw(CScreenReference& ScrnRef, CDC* pDC, COLORREF cr, int nWidth = 1);
	virtual int PointHitTest(CPoint& pnt, CScreenReference& ScrnRef);
#endif
};
