#ifndef __CLinearEquations
#define __CLinearEquations

#include "CMatrix.h"

///////////////////////////////////////////////////////////////////////////////
//   ��С���˷���
class CLinearEquations
{
public:
	int m_nMaxRows;      // ���������������
	int m_nRows;         // ʵ�ʵ�������
	int m_nCols;         // �е�����

	CMatrix A;           // ��С���˷���ʽ������
	CMatrix B;           // ��С���˷���ʽ�Ҳ�������

public:
	CLinearEquations(int nMaxRows, int nCols);
	CLinearEquations();

	// �������Է������ά��
	bool Create(int nMaxRows, int nCols);

	// ���¿�ʼ���þ������
	void Start();

	// ����һ��Լ������
	bool AddRow(float* pa, float b);

	// ȡ�÷�������е�����
	int GetRows() const { return m_nRows; }

	// ȡ�÷�������е�����
	int GetCols() const { return m_nCols; }

	// ʵ�ֶ��������Է�����ĺϲ�
	bool Merge(const CLinearEquations& another);

	// �����С���˷�
	virtual bool LeastSquareSolve(float* pX, int nNum);

#ifdef _MSC_VER
	void Dump();
#endif
};
#endif
