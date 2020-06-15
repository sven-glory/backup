#include "stdafx.h"
#include "Frame.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif

//////////////////////////////////////////////////////////////////////////////
//   ʵ�ֶ�ά����ϵ����"CFrame"��

CFrame::CFrame(const CPosture& pstLocal)
{
	Create(pstLocal);
}

//
//   ����ԭ���������ǹ�������ϵ��
//
CFrame::CFrame(const CPnt& ptOrigin, const CAngle& angSlant)
{
	CPosture::SetPosture(ptOrigin, angSlant);
}

CFrame::CFrame(float _x, float _y, float _t)
{
	CPosture::SetPosture(_x, _y, _t);
}

//
//   ���ݸ�������̬��������ϵ��
//
void CFrame::Create(const CPosture& pstLocal)
{
	CPosture::SetPosture(pstLocal.x, pstLocal.y, pstLocal.fThita);
}	

//
//   ���塰*=�����㡣
//
CFrame& CFrame::operator *= (const CFrame& another)
{
	*this = *this * another;
	return *this;
}

//
//   ���塰*�����㡣
//
CFrame CFrame::operator * (const CFrame& another)
{
	CFrame frm = another;
	frm.InvTransform(*this);

	return frm;
}
