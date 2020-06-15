#pragma once

// �����ʶ��ȱʡ����ֵ
#define MIN_REFLECTOR_INTENSITY         800     // ���������ǿ������
#define MAX_REFLECTOR_SIZE              1.0f    // ������������������(��㵽�յ�ľ���)
#define MAX_REFLECTOR_RADIUS_VARIANCE   1.0f    // ͬһ��������ڼ��������������(��λ:��)
#define MIN_DIST_BETWEEN_REFLECTORS     0.3f    // �������������֮�����������С����(��λ:��)

///////////////////////////////////////////////////////////////////////////////
// �ӵ������ɵ�����ʱ���õĲ���
struct CReflectorCreationParam
{
public:
	int   nMinReflectorIntensity;        // �����ǿ������
	float fMaxReflectorSize;             // ����������(��㵽�յ�ľ���)
	float fMaxPolarRadiusVariance;       // ͬһ��������ڼ��������������(��λ:��)
	float fMinDistBetweenReflectors;     // �����������֮�����������С����(��λ:��)

public:
	CReflectorCreationParam()
	{
		nMinReflectorIntensity = MIN_REFLECTOR_INTENSITY;
		fMaxReflectorSize = MAX_REFLECTOR_SIZE;
		fMaxPolarRadiusVariance = MAX_REFLECTOR_RADIUS_VARIANCE;
		fMinDistBetweenReflectors = MIN_DIST_BETWEEN_REFLECTORS;
	}

	// �Ӷ������ļ�װ�����
	bool Load(FILE* fp)
	{
		if (fread(&nMinReflectorIntensity, sizeof(int), 1, fp) != 1)
			return false;

		float f[3];
		if (fread(f, sizeof(float), 3, fp) != 3)
			return false;

		fMaxReflectorSize = f[0];
		fMaxPolarRadiusVariance = f[1];
		fMinDistBetweenReflectors = f[2];

		return true;
	}

	// ���������浽�������ļ�
	bool Save(FILE* fp)
	{
		if (fwrite(&nMinReflectorIntensity, sizeof(int), 1, fp) != 1)
			return false;

		float f[3];
		f[0] = fMaxReflectorSize;
		f[1] = fMaxPolarRadiusVariance;
		f[2] = fMinDistBetweenReflectors;

		if (fwrite(f, sizeof(float), 3, fp) != 3)
			return false;

		return true;
	}
};
