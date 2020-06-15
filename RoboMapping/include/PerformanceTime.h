#ifndef __CPerformanceTime
#define __CPerformanceTime

class CPerformanceTime
{
private:
	LARGE_INTEGER m_liPerfFreq;
	LARGE_INTEGER m_liPerfStart;

public:
	CPerformanceTime()
	{
		QueryPerformanceFrequency(&m_liPerfFreq);
	}

	void Restart()
	{
		QueryPerformanceCounter(&m_liPerfStart);
	}

	// ���غ���ʱ��
	float GetElapsedTime()
	{
		LARGE_INTEGER liPerfNow = { 0 };

		// ����CPU���е����ڵ�ʱ�� 
		QueryPerformanceCounter(&liPerfNow);

		float time = ((liPerfNow.QuadPart - m_liPerfStart.QuadPart) * 1000.0f) / m_liPerfFreq.QuadPart;
		return time;
	}
};
#endif

