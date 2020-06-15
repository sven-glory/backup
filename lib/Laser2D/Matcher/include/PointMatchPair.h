#ifndef __CPointMatchPair
#define __CPointMatchPair

#include "Geometry.h"

///////////////////////////////////////////////////////////////////////////////
//   "CPointMatchPair"��Ķ��塣
class CPointMatchPair
{
public:
	CPnt m_ptLocal;               // �ھֲ�����ϵ�еĵ�
	CPnt m_ptWorld;               // ��ȫ������ϵ�еĵ�
	int      m_nLocalId;              // �ֲ�ֱ�߶ε�ID��
	int      m_nWorldId;              // ȫ��ֱ�߶ε�ID��
	CPnt m_ptLocalToWorld;      // �ֲ������Ӱ�䵽ȫ������ϵ���λ��

public:
	CPointMatchPair(int nLocalId, int nWorldId, const CPnt& pnt1, const CPnt& pnt2)
	{
		Create(nLocalId, nWorldId, pnt1, pnt2);
	}

	CPointMatchPair() {}

	void Create(int nLocalId, int nWorldId, const CPnt& pnt1, const CPnt& pnt2)
	{
		m_nLocalId = nLocalId;
		m_nWorldId = nWorldId;
		m_ptLocal = pnt1;
		m_ptWorld = pnt2;
	}

	// ���ݴ˵��ƥ����������ڡ���С���˷����Ĳ���
	bool MakeLeastSquareData(float fCoefA[2][4], float fCoefB[2]);

	// ���ݴ˵��ƥ����������ڡ���С���˷����Ĳ���(�������)
	bool MakeLeastSquareData1(float fCoefA[2][2], float fCoefB[2], float fSin, float fCos);
};
#endif
