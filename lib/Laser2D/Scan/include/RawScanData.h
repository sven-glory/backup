#ifndef __CRawScanData
#define __CRawScanData

#include <list>
#include <vector>
#include "Geometry.h"

using namespace std;

typedef vector<double> RangeReadings;

class CRawScanData
{
public:
	int           nCount;                     // ÿ��ɨ������
	float         fStartAngle;                // ��ʼɨ���
	float         fEndAngle;                  // ��ֹɨ���
	RangeReadings Readings;                   // ɨ��������
	CPosture      pstOdometry;                // ɨ��ʱ��������̬
};

typedef CRawScanData* CScanDataPtr;
typedef list<CScanDataPtr> ScanDataSet;

#endif
