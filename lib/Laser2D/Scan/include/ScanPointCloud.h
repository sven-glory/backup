#ifndef __CScanPointCloud
#define __CScanPointCloud

#include "ScrnRef.h"
#include "ScanPoint.h"
#include "TimeStamp.h"

///////////////////////////////////////////////////////////////////////////////
// ��CScanPointCloud���ඨ���ά���ơ�

class CScanPointCloud : public CTimeStamp
{
public:
	int         m_nCount;                    // ɨ�������
	CScanPoint* m_pPoints;                   // ָ��ɨ������ݻ�������ָ��

protected:
	// ����Щ���Ϊ��ɾ�����ĵ�����ɾ��
	void RemoveDeletedPoints();

public:
	// ����ָ���ĵ��������ɶ���(ֻ����ռ�)
	CScanPointCloud(int nNum);

	// ��������һ���������ɶ���
	CScanPointCloud(const CScanPointCloud& Cloud);

	// ���ɿյĶ���
	CScanPointCloud();

	~CScanPointCloud();

	// ȡ�õ��ƶ����ָ��(��Ҫ������������)
	CScanPointCloud* GetScanPointCloudPointer() {return this;}

	// �����������
	void Clear();

	// Ϊ�������ݷ���ռ�
	bool Create(int nNum);

	// ��㼯ĩβ���һ����
	bool Add(const CScanPoint& sp);

	// �ӵ�����ɾ��һ����
	bool Delete(int nIndex);

	// ���ء�=��������
	void operator = (const CScanPointCloud& Cloud2);

	// ���ء�+=��������
	void operator += (const CScanPointCloud& Cloud2);

	// ���ı��ļ���װ���������
	bool Load(FILE* file);

	// ���������ݴ����ı��ļ�
	bool Save(FILE* file);

	// �Ӷ������ļ��ж�ȡɨ������
	bool LoadBinary(FILE* fp, float fStartAngle, float fEndAngle, int nLineCount, int nFileVersion);

	// ��ɨ�����ݱ��浽�������ļ�
	bool SaveBinary(FILE* fp, int nFileVersion);

	// ���ݵϿ��������������е�ļ�����
	void UpdatePolar();

	// ���ݼ������������е�ĵϿ�������
	void UpdateCartisian();

	// ��ȫ�����ƶ�ָ���ľ���
	void Move(float fDx, float fDy);

	// ��ȫ������ָ�������ĵ������ת
	void Rotate(float centerX, float centerY, float angle);

	// ���������ƽ�������任
	void Transform(float x, float y, float thita);

	// ���������ƽ�������任(�ڶ�����ʽ)
	void Transform(const CPosture& pstOrigin);

	// ���������ƽ�������ϵ�任
	virtual void Transform(const CFrame& frame);

	// ���������ƽ�������ϵ��任
	virtual void InvTransform(const CFrame& frame);

	// ȡ�������X����
	float LeftMost();

	// ȡ�����ϵ�Y����
	float TopMost();

	// ȡ�����ҵ�X����
	float RightMost();

	// ȡ�����µ�Y����
	float BottomMost();

	// ȡ�õ��Ƶ�X����
	float Width();

	// ȡ�õ��Ƶ�Y��߶�
	float Height();

	// ȡ�ô˵��������ǵ��������
	CRectangle GetCoveringRect();

	// �˶Ե����Ƿ����ָ���ĵ�
	int ContainPoint(const CPnt& pt, float fThreshHoldDist);

	// ��ָ���ĵ�Ϊ���ģ���ָ���İ뾶�����е���й��ˣ�ֻ���´��ڰ뾶���ڵĵ�
	void ReduceByRadius(const CPnt& ptCenter, float dRadius);

	// ��ָ���ķ����Ϊ���ģ�ָ���ĽǶȷ�Χ�����е���й��ˣ�ֻ������Ч�Ƕ��ڵĵ�
	void ReduceByAngle(CPosture& pst, float fViewAngle);

	// ����ָ������̬���Ե��Ƹ���ɨ��ǽ�������
	void SortByAngle();

	// ����ѽǶ�����ĵ��ƣ�ɾ����Щ���ڵ��ĵ�
	void RemoveHiddenPoints(const CPosture& pst, float fDistGate);

	// �����г��������޵ĵ��Ƴ�
	void RemoveOutOfRangePoints();

	// ��ָ���ĽǷֱ��ʺ�ɨ�����Ե��ƽ������²���
	CScanPointCloud* ReSample(CPosture& pstScanner, float fStartAng, float fViewAngle, float fAngReso, float fMaxRange);

	// ���ݸ�����ֱ�߷������ɵ���
	bool CreateFromLine(CLine& ln, float fPointDist, float fNoise);

	// ���ݸ�����ֱ������������ɵ���
	bool CreateFromLineArray(int nCountLines, CLine* lines, float fPointDist, float fNoise);

	// ���ݸ�����Բ�������ɵ���
	bool CreateFromCircle(CCircle& circle, float fPointDist, float fNoise);

	// ���ݸ�����Բ������������ɵ���
	bool CreateFromCircleArray(int nCountCircles, CCircle* circles, float fPointDist, float fNoise);

#ifdef _MSC_VER
	// �ڵ���������ʾ��������
	void Dump();

	// ���Ƶ���
	void Draw(CScreenReference& ScrnRef, CDC* pDC, COLORREF color, int nPointSize = 1);
#endif
};

#endif
