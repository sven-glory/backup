#ifndef __CSimuScanner
#define __CSimuScanner

#include "Geometry.h"
#include "ScrnRef.h"
#include "Scan.h"
#include "FeatureMap.h"
#include "MovingObj.h"

#define SCANNER_RESO       0.25f
#define SCANNER_RESO_RAD   (SCANNER_RESO/180.f*PI)
#define SCAN_COUNT         ((int)(360/SCANNER_RESO))
#define MAX_LINE_NUM       2000
#define MAX_CIRCLE_NUM     1000

class CLaserBeam;

class CSimuScanner
{
private:
	CPnt m_ptScanner;
	float m_fAmpRatio;

	float m_fAngReso;
	CFeatureMap* m_pFeatureMap;

public:
	int   m_nScanPointCount;
	float m_fDist[SCAN_COUNT];
	CLaserBeam m_ScanData[SCAN_COUNT];
	CMovingObjSet m_MovingObjSet;


public:
	static void AddNoise(CPnt& ptIn, float fNoise, CPnt& ptOut);
	static void AddNoise(float fDistIn, float& fDistOut);

public:
	CSimuScanner(float fAngReso = SCANNER_RESO_RAD);

	// ≥ı ºªØ∑¬’Ê…®√Ë∆˜
	bool Init(CFeatureMap* pFeatureMap);

	int ContourScan(CPnt& ptScanner, float fStartAng, float fEndAng, float fMaxRange);

	void DrawMovingObj(CScreenReference& ScrnRef, CDC* pDc, COLORREF color);

	void Plot(CDC* pDC, COLORREF crColor);
};

CScan* NewSimulateScan(CPosture& pstScannerActual, CPosture& pstScannerEstimate, float dViewAngle, float fMaxRange);

#endif
