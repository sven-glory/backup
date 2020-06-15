#ifndef __CLineMatchPair
#define __CLineMatchPair

#include "Geometry.h"

///////////////////////////////////////////////////////////////////////////////
//   "CLineMatchPair"��Ķ��塣
class CLineMatchPair
{
private:
	float m_fDistToOrigin;         // �ھֲ�����ϵ�У�ԭ�㵽��ֱ�ߵľ���

public:
	CLine m_lnLocal;               // �ھֲ�����ϵ�е�ֱ�߶�
	CLine m_lnWorld;               // ��ȫ������ϵ�е�ֱ�߶�
	int   m_nLocalId;              // �ֲ�ֱ�߶ε�ID��
	int   m_nWorldId;              // ȫ��ֱ�߶ε�ID��
	CPosture m_pstOdometry;        // ��ȫ������ϵ�е�Odometry��̬
	CAngle m_ang;                  // ����Ƕ�ֵ(����һ������sin(x), �ڶ�������cos(x)�е�x)

public:
#if 0
	CLineMatchPair(const CLine& lnLocal, const CLine& lnWorld, CPosture& pstOdometry, CAngle& ang)
	{
		Create(lnLocal, lnWorld, pstOdometry, ang);
	}
#endif

	CLineMatchPair() {}

	// ����ֱ��ƥ���
	void Create(int nLocalId, int nWorldId, const CLine& lnLocal, const CLine& lnWorld, CPosture& pstOdometry, CAngle& ang)
	{
		m_nLocalId = nLocalId;
		m_nWorldId = nWorldId;
		m_lnLocal = lnLocal;
		m_lnWorld = lnWorld;
		m_pstOdometry = pstOdometry;
		m_ang = ang;

		CPnt ptOrigin(0, 0);
		m_fDistToOrigin = m_lnLocal.DistanceToPoint(false, ptOrigin);
	}

	// ���ݴ�ֱ��ƥ����������ڡ���С���˷����Ĳ���
	bool MakeLeastSquareData(float fCoefA[3][4], float fCoefB[3]);
};
#endif
