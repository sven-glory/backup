#include <stdafx.h>
#include "ndt_fuser/MapWorkSpace.h"

///////////////////////////////////////////////////////////////////////////////

bool CMapWorkSpace::Load(FILE* fp)
{
	if (!m_dataset.LoadRawScanBinary(fp))
		return false;

	if (!m_ndtMap.Load(fp))
		return false;

	return m_vectPose.Load(fp);
}

bool CMapWorkSpace::Save(FILE* fp, int nFileVersion)
{
	if (!m_dataset.SaveRawScanBinary(fp, nFileVersion))
		return false;

	if (!m_ndtMap.Save(fp))
		return false;

	return m_vectPose.Save(fp);
}

#ifdef _MFC_VER

//
//   »æÖÆÄ£ÐÍÍ¼¡£
//
void CMapWorkSpace::PlotMap(CDC* pDC, CScreenReference& ScrnRef, bool bShowPoses)
{

}
#elif defined QT_VERSION

#endif