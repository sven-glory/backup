#ifndef __CScanPoint
#define __CScanPoint

#include "Geometry.h"

#define SCAN_OUT_OF_RANGE              99999L

///////////////////////////////////////////////////////////////////////////////
//   ��Ӧ��ÿһ������ɨ���ߵ�ɨ�����ݡ�
class CLaserBeam
{
public:
	float m_fDist;          // ɨ�����(��λ:m)
	int m_nIntensity;       // ����ǿ������

public:
	CLaserBeam()
	{
		m_fDist = 0;
		m_nIntensity = 0;
	}
};

///////////////////////////////////////////////////////////////////////////////
// ��CScanPoint���ඨ���˶�άɨ���ĸ��
class CScanPoint : public CPnt
{
public:
	short m_nLineID;     // ��Ӧ��ֱ�߶εı��(����õ�����һ��ֱ�߶���),����-1
	int   m_nIntensity;  // ����ǿ��
	bool  m_bDelete;     // ɾ����־
	bool  m_bHighReflective; // ǿ�����־

public:
	CScanPoint(float _x = 0, float _y = 0, unsigned _id = 0) :
		CPnt(_x, _y, _id)
	{
		m_nLineID = -1;
		m_nIntensity = 0;
		m_bDelete = false;
		m_bHighReflective = false;
	}

	// ����ɨ����Ϊ�����볬�ޡ�
	void SetOutOfRange()
	{
		x = y = 0;
		a = 0;
		r = SCAN_OUT_OF_RANGE;
		m_bDelete = false;
	}
};
#endif
