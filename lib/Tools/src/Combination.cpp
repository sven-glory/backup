#include <stdafx.h>
#include "Combination.h"

///////////////////////////////////////////////////////////////////////////////

//
//   ����׳�(n!)��
//
int Factorial(int n)
{
	int m = 1;
	for (int i = 1; i <= n; i++)
		m *= i;

	return m;
}

//
//   ���������C(n, k)
//
int Combination(int n, int k)
{
	return Factorial(n) / (Factorial(k) * Factorial(n - k));
}
