#ifndef __CLaserScannerParam
#define __CLaserScannerParam

#include <vector>
#include "Geometry.h"
#include "Range.h"

using namespace std;

// ����ɨ��������
class CLaserScannerParam
{
public:
	float m_fStartAngle;               // ��ʼɨ���(��λ������)
	float m_fEndAngle;                 // ��ֹɨ���(��λ������)
	int   m_nLineCount;                // ɨ������
	float m_fReso;                     // �ֱ���(��λ������/��)
	CPosture m_pst;                    // ����ڻ����˵İ�װ��̬
	CRangeSet m_AppAngleRange;         // ʵ�ʿ��ýǶȷ�Χ(�ɴ�������װλ�þ���)
	bool  m_bUseCreateModel;           // �Ƿ����ô�ɨ������ģ(������RoboMapping�����)
	bool  m_bUseLocalize;              // �Ƿ����ô�ɨ������λ(������RoboMapping�����)

public:
	CLaserScannerParam();

	// ������ʼ�ǡ���ֹ�ߺ�ɨ������
	void Set(float fStartAngle, float fEndAngle, int nLineCount);

	// �Ӷ������ļ��ж�ȡ����
	bool LoadBinary(FILE* fp, int nFileVersion = 200);

	// ��������ļ���д�����
	bool SaveBinary(FILE* fp, int nFileVersion = 200);
};

// ������ɨ�����顱������
class CScannerGroupParam : public  vector<CLaserScannerParam>
{
public:
	void SetRelativePosture(int nRefScanner, int nCurScanner, const CPosture& pst);

	// �Ӷ������ļ��ж�ȡ����
	bool LoadBinary(FILE* fp, int nFileVersion = 200);

	// ��������ļ���д�����
	bool SaveBinary(FILE* fp, int nFileVersion = 200);

	// ���ø�ɨ�����Ƿ�Ӧ���ڽ�ģ
	void UseWhenCreateModel(int nIdx, bool bYesOrNo);

	// ���ø�ɨ�����Ƿ�Ӧ���ڶ�λ
	void UseWhenLocalize(int nIdx, bool bYesOrNo);
};
#endif
