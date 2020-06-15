#pragma once

#include "ReflFeatureCreateParam.h"
#include "LineFeatureCreateParam.h"

///////////////////////////////////////////////////////////////////////////////
//   ������ȡ������
class CFeatureCreationParam
{
public:
	CReflectorCreationParam   m_RefParam;
	CLineFeatureCreationParam m_LineParam;

	CFeatureCreationParam() {}

	// �Ӷ������ļ�װ�����
	bool Load(FILE* fp)
	{
		if (!m_RefParam.Load(fp))
			return false;

		if (!m_LineParam.Load(fp))
			return false;
		
		return true;
	}

	// ���������浽�������ļ�
	bool Save(FILE* fp)
	{
		if (!m_RefParam.Save(fp))
			return false;

		if (!m_LineParam.Save(fp))
			return false;

		return true;
	}
};