#ifndef __COptimizableLine
#define __COptimizableLine

#define MAX_SIGMA                       30      // max sigma value for a line
#define MIN_LINE_LEN                    500     // ֱ�߶ε���С����
#define MAX_LINE_MERGE_DIST             80      // ����ƽ��ֱ�ߵļ��С�ڴ�ֵʱ����Ϊ��һ��ֱ�ߣ����Ժϲ�
#define MAX_LINE_MERGE_ANGLE            (5*PI/180)    // ����ֱ�ߵļн�С�ڴ�ֵʱ����Ϊ���໥ƽ��
#define MIN_CORNER_ANGLE                (PI/6)   // �γɽǵ������ֱ�ߵ���С�н�
#define MAX_HIDDEN_LINE_LEN             5000     // �����ֱ�߶ε��ӳ��߳����˾��룬���ɵĽ��㲻����

#include "Geometry.h"
#include "vector"

using namespace std;

///////////////////////////////////////////////////////////////////////////////
//   ���塰���Ż�ֱ�ߡ��ࡣ
class COptimizableLine : public CLine
{
public:
	bool m_bUse;
	vector<CPnt> m_ptKeys;

public:
	COptimizableLine(CLine& ln)
	{
		Create(ln.m_ptStart, ln.m_ptEnd);
	}

	COptimizableLine() {}
};

///////////////////////////////////////////////////////////////////////////////
//   ���塰���Ż�ֱ�߼��ϡ��ࡣ
class COptimizableLineSet
{
public:
	vector<COptimizableLine> m_LineSet;

public:
	COptimizableLineSet() {}
	
	// �����򼯺��м���һ��ֱ��
	bool AddLine(CLine& ln);

	// ����ֱ�������������ɽǵ㼯��
	int CreateCornerPoints(vector<CPnt>& ptCorners);

};

#endif
