#pragma once

#include "Geometry.h"

#define MAX_DIST_POINT_TO_POINT         0.08f   // ��1��Զ����ֱ�ڼ����������ϣ��������ڵ�������������
#define MAX_DIST_POINT_TO_LINE          0.03f   // ����ֱ�߶��ϵĵ㵽������ֱ�ߵ�����������
#define MAX_SIGMA                       0.03f   // ֱ�߶�����������sigmaֵ
#define MAX_SIGMA_RATIO                 0.2f    // ֱ��sigmaֵ(ƽ��������)��ֱ�߳��ȵ���������ֵ
#define MIN_POINTS_ON_LINE              30      // 1��Զ����ֱ�ڼ�������ֱ�߶����������ٵ���
#define MIN_LINE_LEN                    0.04f//0.2f    // ֱ�߶����������С����
#define MIN_SCAN_TO_LINE_ANGLE          TO_RADIAN(10)   // ɨ����ֱͬ���������ɽǶȵ���Сֵ(��λ������)

#define MAX_LINE_MERGE_DIST             0.08f    // ����ƽ��ֱ�ߵļ��С�ڴ�ֵʱ����Ϊ��һ��ֱ�ߣ����Ժϲ�
#define MAX_LINE_MERGE_ANGLE            (5*PI/180)    // ����ֱ�ߵļн�С�ڴ�ֵʱ����Ϊ���໥ƽ��
#define MIN_CORNER_ANGLE                (PI/6)   // �γɽǵ������ֱ�ߵ���С�н�
#define MAX_HIDDEN_LINE_LEN             25.0f    // �����ֱ�߶ε��ӳ��߳����˾��룬���ɵĽ��㲻����


///////////////////////////////////////////////////////////////////////////////
// �ӵ�������ֱ��ʱ���õĲ���
struct CLineFeatureCreationParam
{
public:
	float fMaxDistPointToPoint;          // max distance for a cloud of points (grouping)
	float fMaxDistPointToLine;				 // ����ֱ�߶��ϵĵ㵽������ֱ�ߵ�����������
	float fMaxSigma;							 // max sigma value for a line
	int   nMinPointsOnLine;					 // ֱ�߶����������ٵ���
	float fMinLineLength;					 // ֱ�߶ε���С����
	float fMinScanToLineAngle;           // ɨ������ֱ���������ɼнǵ���Сֵ(��λ:����)
	float fMaxLineMergeDist;				 // ����ƽ��ֱ�ߵļ��С�ڴ�ֵʱ����Ϊ��һ��ֱ�ߣ����Ժϲ�
	float fMaxLineMergeAngle;				 // ����ֱ�ߵļн�С�ڴ�ֵʱ����Ϊ���໥ƽ��

public:
	CLineFeatureCreationParam()
	{
		fMaxDistPointToPoint = MAX_DIST_POINT_TO_POINT;
		fMaxDistPointToLine = MAX_DIST_POINT_TO_LINE;
		fMaxSigma = MAX_SIGMA;
		nMinPointsOnLine = MIN_POINTS_ON_LINE;
		fMinLineLength = MIN_LINE_LEN;
		fMinScanToLineAngle = MIN_SCAN_TO_LINE_ANGLE;
		fMaxLineMergeDist = MAX_LINE_MERGE_DIST;
		fMaxLineMergeAngle = MAX_LINE_MERGE_ANGLE;
	}

	// �Ӷ������ļ�װ�����
	bool Load(FILE* fp)
	{
		if (fread(&nMinPointsOnLine, sizeof(int), 1, fp) != 1)
			return false;

		float f[7];
		if (fread(f, sizeof(float), 7, fp) != 7)
			return false;

		fMaxDistPointToPoint = f[0];
		fMaxDistPointToLine = f[1];
		fMaxSigma = f[2];
		fMinLineLength = f[3];
		fMinScanToLineAngle = f[4];
		fMaxLineMergeDist = f[5];
		fMaxLineMergeAngle = f[6];

		return true;
	}

	// ���������浽�������ļ�
	bool Save(FILE* fp)
	{
		if (fwrite(&nMinPointsOnLine, sizeof(int), 1, fp) != 1)
			return false;

		float f[7];
		f[0] = fMaxDistPointToPoint;
		f[1] = fMaxDistPointToLine;
		f[2] = fMaxSigma;
		f[3] = fMinLineLength;
		f[4] = fMinScanToLineAngle;
		f[5] = fMaxLineMergeDist;
		f[6] = fMaxLineMergeAngle;

		if (fwrite(f, sizeof(float), 7, fp) != 7)
			return false;

		return true;
	}
};
