#ifndef __CScan
#define __CScan

#include <vector>
#include "ScanPointCloud.h"
#include "RawScanData.h"
#include "PostureGauss.h"
#include "ScrnRef.h"
#include "Frame.h"
#include "ScannerParam.h"

#ifdef QT_VERSION
#include <QColor>
#endif

using namespace std;

///////////////////////////////////////////////////////////////////////////////
//   ��CScan����ʵ���ˡ���ά����ɨ�衱�ĸ��

class CScan : public CScanPointCloud
{
public:
	CPostureGauss m_poseRelative;        // ����ͷ�������̬
	CPostureGauss m_poseScanner;         // ����ͷ�Ĺ�����̬
	float         m_fStartAng;           // ����ͷ��ʼɨ���(����)
	float         m_fEndAng;             // ����ͷ��ֹɨ���(����)

public:
	CScan();

	// �������nNum�����CScan����
	CScan(int nNum);

	CScan(CScanPointCloud* pCloud);

	~CScan();

	// ���ء�=��������
	void operator = (const CScan& Scan);

	// �����������
	void Clear();

	// ����һ���µ�ɨ��
	bool Create(int nNum);

	// ���ݸ�����������һ���µ�ɨ��
	bool Create(CScanPointCloud* pCloud);

	void SortByAngle();
	void SortByAngle(float fViewAngle);

	// �Ƴ���Щ���ڵ��ĵ�
	void RemoveHiddenPoints(float fDistGate);

	// ����ռ䲢���Ƶ�ǰɨ��
	CScan *Duplicate();

	// ����ǰɨ������һ��ɨ�輯���кϲ�
	void Merge(CScan& scan2, bool bNewPointsOnly = true);

	// ��ȫ������תָ������̬
	void Rotate(float angle);

	void RotatePos(float angle, float cx, float cy);

	// ��ָ���ĽǷֱ��ʺ�ɨ�����Ե��ƽ������²���
	CScan* ReSample(float fStartAng, float fViewAngle, float fAngReso, float fMaxRange);

	// ����ָ����ɨ��Ƿ�Χ����ȡ���Ƶ�һ���Ӽ�
	CScan* GetSubset(float fStartAngle, float fEndAngle);

	// ���ݸ����ġ�ǿ�������ޡ�ֵ�������Щ�������ڡ�ǿ���⡱
	void MarkReflectivePoints(int nReflectiveGateValue);

	// �����µļ���ɨ������Ч��Χ
	void ApplyNewScannerAngles(const CRangeSet& AngleRange);

	// ���ù���ɨ��Ƕȷ�Χ��Լ��
	void ApplyScanAngleRule(float fMinAngle, float fMaxAngle);

	// ���ù���ɨ������Լ��
	void ApplyScanDistRule(float fMinDist, float fMaxDist);

	// ���ı��ļ��ж�ȡɨ������
	bool Load(FILE* fp);

	// ��ɨ�����ݱ��浽�ı��ļ�
	bool Save(FILE* fp);

	// �Ӷ������ļ��ж�ȡɨ������
	bool LoadBinary(FILE* fp, const CPosture& pstRobot, const CLaserScannerParam& Param, int nFileVersion);

	// ��ɨ�����ݱ��浽�������ļ�
	bool SaveBinary(FILE* fp, int nFileVersion);

	// ���������ƽ�������ϵ�任
	virtual void Transform(const CFrame& frame);

	// ���������ƽ�������ϵ��任
	virtual void InvTransform(const CFrame& frame);

#ifdef _MFC_VER
	void Plot(CScreenReference& ScrnRef, CDC* pDC, COLORREF crColor, COLORREF crHighLightPoint,
		bool bShowScanner, int nPointSize = 1);
#elif defined QT_VERSION
	void Plot(CScreenReference& ScrnRef, QPainter* pPainter, QColor crColor, QColor crHighLightPoint,
		bool bShowScanner, int nPointSize = 1);

#endif
};
#endif

