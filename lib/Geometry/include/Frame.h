#pragma once

#include "Geometry.h"

//////////////////////////////////////////////////////////////////////////////
//   �����ά������ϵ����"CFrame"��

class CFrame : public CPosture
{
public:
	CFrame(const CPosture& pstLocal);
	CFrame(const CPnt& ptOrigin, const CAngle& angSlant);
	CFrame(float _x, float _y, float _t);
	CFrame() {}

	// ������������ϵ����
	void Create(const CPosture& pstLocal);

	// ������������ϵ����
	void Create(float _x, float _y, float _t)
	{
		CPosture::Create(_x, _y, _t);
	}

	//   ���塰*=������
	CFrame& operator *= (const CFrame& another);

	//   ���塰*������
	CFrame operator * (const CFrame& another);
};
