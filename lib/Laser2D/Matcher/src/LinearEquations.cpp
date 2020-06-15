#include "stdafx.h"
#include <math.h>
#include "LinearEquations.h"
#include "DebugTrace.h"
///////////////////////////////////////////////////////////////////////////////

//
//   �����������ɵ�������ͬʱ�����ڴ�ȷ��(���ɸ���)��
//
CLinearEquations::CLinearEquations(int nMaxRows, int nCols)
{
	Create(nMaxRows, nCols);
}

CLinearEquations::CLinearEquations()
{
	m_nMaxRows = m_nRows = m_nCols = 0;
}

//
//   �������Է������ά�ȡ�
//
bool CLinearEquations::Create(int nMaxRows, int nCols)
{
	A.Create(nMaxRows, nCols);
	B.Create(nMaxRows, 1);

	m_nMaxRows = nMaxRows;
	m_nRows = 0;
	m_nCols = nCols;

	return true;
}

//
//   ��ʼ���þ��������
//
void CLinearEquations::Start()
{
	m_nRows = 0;
}

//
//   ����һ��Լ��������
//
bool CLinearEquations::AddRow(float* pa, float b)
{
	// �˶��Ƿ񳬹����������
	if (m_nRows + 1 >= m_nMaxRows)
		return false;

	// �������þ���ϵ��
	for (int i = 0; i < m_nCols; i++)
		A.SetAt(m_nRows, i, *pa++);

	B.SetAt(m_nRows, 0, b);
	
	// ��������
	m_nRows++;
	return true;
}

//
//   ʵ�ֶ��������Է�����ĺϲ���
//
bool CLinearEquations::Merge(const CLinearEquations& another)
{
	// ���Ⱥ˶Է�����������Ƿ���ͬ
	if (m_nCols != another.m_nCols)
		return false;

	// �������Ӿ���������
	for (int j = 0; j < another.GetRows(); j++)
	{
		// ����������ޣ������Ĳ��ֻ᲻�ᱻ����
		if (m_nRows++ >= m_nMaxRows)
		{
			m_nRows = m_nMaxRows;
			break;
		}

		for (int i = 0; i < m_nCols; i++)
			A.SetAt(m_nRows + j, i, another.A.GetAt(j, i));

		B.SetAt(m_nRows, 0, another.B.GetAt(j, 0));
	}

	return true;
}

//
//   �����С���˷���
//   ���������pXָ��Ļ������У���������СnNum���벻С��m_nCols��
//
bool CLinearEquations::LeastSquareSolve(float* pX, int nNum)
{
	if (pX == NULL || nNum < m_nCols)
		return false;

	// ȷ��A��B���������
	A.m_nRow = m_nRows;
	B.m_nRow = m_nRows;

	CMatrix matrix_1 = A.Transpose();
	CMatrix matrix_2 = matrix_1 * A;

	// ����ʽ��ֵ
	float det = matrix_2.Determinant();
	if (det == 0)
		return false;

	CMatrix matrix_p = matrix_2.GetAccompany() * matrix_1 * B;

	// �õ�����������������ص������Ļ�������
	for (int i = 0; i < m_nCols; i++)
		*pX++ = matrix_p.GetAt(i, 0) / det;

	return true;
}

#ifdef _MSC_VER
void CLinearEquations::Dump()
{
	DebugTrace(_T("Least square method - A:\n"));
	A.Dump();

	DebugTrace(_T("Least square method - B:\n"));
	B.Dump();
}
#endif
