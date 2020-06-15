#ifndef __CStampedPosture
#define __CStampedPosture

#include "Geometry.h"
#include "TimeStamp.h"

//
//   �������ʱ�������̬��
//
class CStampedPosture : public CPosture, public CTimeStamp
{
public:
	CStampedPosture(const CPosture& pst, unsigned int uTime) : CPosture(pst), CTimeStamp(uTime)
	{
	}

	CStampedPosture() {}
};
#endif
