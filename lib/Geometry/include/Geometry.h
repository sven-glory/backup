//                            - GEOMETRY.H -
//
//    Defines the following basic geometric concepts:
//      . CPnt
//      . CAngle
//      . CPosture
//      . CPostureDev
//      . CMoveDir
//      . CTurnDir
//      . CTransform
//      . CLine
//      . CArc
//      . CSpp
//      . CSpline
//      . CScp
//
//    Author: Zhang Lei
//    Date:   2001. 9. 7
//

#ifndef __Geometry
#define __Geometry

#include <math.h>
#include <stdio.h>
#include "ZTypes.h"
//#include "Tools.h"

// Defines PI
#if !defined PI
#define PI                     (3.14159265f)
#endif

#define TO_DEGREE(x)           (x/PI*180.0f)
#define TO_RADIAN(x)           (x/180.0f*PI)

class CScreenReference;
class CPosture;
class CFrame;

#ifdef QT_VERSION
class QPoint;
class QPainter;
class QColor;
#endif

//////////////////////////////////////////////////////////////////////////////
//   The interface of class "CPnt".
class DllExport CPnt
{
public:
	unsigned int id;        // ID��
	float x;                // ƽ��ֱ������Xλ��
	float y;                // ƽ��ֱ������Yλ��
	float a;                // ������Ƕ�(��λ������)
	float r;                // ���������

public:
	// The constructor
	CPnt(float fx, float fy, unsigned int _id = 0)
	{
		x = fx;
		y = fy;
		a = 0;
		r = 0;
		id = _id;
	}

	// Default constructor
	CPnt() 
	{
		x = 0;
		y = 0;
		a = 0;
		r = 0;
		id = 0;
	}

	// ���õ������
	void Set(float _x, float _y)
	{
		x = _x;
		y = _y;
	}
	
	// ���õ�ļ�����
	void SetPolar(float fAngle, float fRadius, int _id)
	{
		id = _id;
		r = fRadius;
		a = fAngle;
		x = (float)(fRadius * cos(fAngle));
		y = (float)(fRadius * sin(fAngle));
	}

	// ȡ�ö��������
	CPnt& GetPntObject() {return *this;}

	// �����ƶ�ָ���ľ���
	void Move(float dx, float dy);

	// ������ָ�������ĵ������ת
	virtual void Rotate(float fAng, float fCx = 0, float fCy = 0);

	// ������ָ�������ĵ������ת
	virtual void Rotate(float fAng, const CPnt& ptCenter);

	// ���� "=="
	bool operator ==(const CPnt& pt) const;

	// ����  "!="
	bool operator !=(const CPnt& pt) const;

	// ���� "+="
	void operator += (const CPnt& pt);

	// ���� "-="
	void operator -= (const CPnt& pt);

	// ���� "+"
	CPnt operator +(const CPnt& pt) const;

	// ���� "-"
	CPnt operator -(const CPnt& pt) const;

	// �Ƚ�������ġ���С��(���ݡ���-�¡�ԭ��)
	bool operator < (const CPnt& pt) const;

	// �Ƚ�������ġ���С��(���ݡ���-�¡�ԭ��)
	bool operator > (const CPnt& pt) const;

	// ����õ㵽��һ���ֱ�߾���
	float DistanceTo(const CPnt& pt) const;

	// ����������֮��ľ����ƽ��
	float Distance2To(const CPnt& pt2) const;

	// �ж��������Ƿ���Խ��Ƶ���Ϊ��һ����(���ǳ���)
	bool IsEqualTo(const CPnt& pt2, float limit) const;

	// ���ݵϿ�������������ļ�����
	void UpdatePolar();
	void UpdatePolar(const CPosture& pstRefFrame);

	// ���ݼ�����������ĵϿ�������
	void UpdateCartisian();

	// ����õ㵽��һ��ĽǶȾ���
	float AngleDistanceTo(const CPnt& pt) const;

	// ���ļ�װ�������
	bool Load(FILE* fp);

	// ��������д���ļ�
	bool Save(FILE* fp);

	// �����������任
	virtual void Transform(const CFrame& frame);

	// ����������任
	virtual void InvTransform(const CFrame& frame);

#ifdef _MFC_VER

	CPnt(const CPoint& point)
	{
		x = (float)point.x;
		y = (float)point.y;
		a = 0;
		r = 0;
		id = 0;
	}

	// �����������캯��
	void operator = (const CPoint& point);

	// ���� "=="
	bool operator == (const CPoint& point) const;

	// ����  "!="
	bool operator != (const CPoint& point) const;

	void Dump();

	// ����Ļ�ϻ��Ƹõ�
	void Draw(CScreenReference& ScrnRef, CDC* pDC, COLORREF color, int nPointSize = 1, int nLineWidth = 0);

#elif defined QT_VERSION

	CPnt(const QPoint& point);

	// �����������캯��
	void operator = (const QPoint& point);

	// ���� "=="
	bool operator == (const QPoint& point) const;

	// ����  "!="
	bool operator != (const QPoint& point) const;

	void Dump();

	// ����Ļ�ϻ��Ƹõ�
	void Draw(CScreenReference& ScrnRef, QPainter* pPainter, QColor color, int nPointSize = 1, int nLineWidth = 0);

#endif
};

//////////////////////////////////////////////////////////////////////////////
//   The interface of class "CAngle".
enum ANGLE_MODE {IN_RADIAN, IN_DEGREE};

class DllExport CAngle
{
public:
	static float m_fReso;        // Resolution for angle computation

public:
	float m_fRad;                // The angle value in radian

public:
	// Convert degree into radian
	static float ToRadian(float deg);

	// Convert radian into degree
	static float ToDegree(float rad);

	// Normalize an angle
	static float NormAngle(float rad);

	static float NormAngle2(float rad);

	// Set new resolution for angle computation
	static float SetReso(float fReso);

public:
	// The constructors
	CAngle(float val, int mode = IN_RADIAN);

	// ���������㹹���
	CAngle(const CPnt& pt1, const CPnt& pt2);

	// The default constructor
	CAngle() { m_fRad = 0; }

	// Get the angle value in "degree" in range [0, 360)
	float Degree();

	// Get the angle value in "degree" in range [-180, 180)
	float Degree2();

	// The quadrant of angle: 1/2/3/4
	int Quadrant();

	// ������תһ���Ƕ�
	void Rotate(float fAng);
	
	// Normalize an angle to [0, 2*PI]
	float NormAngle();

	// Normalize an angle to [-PI, PI]
	float NormAngle2();

	// Overloaded operators: "!", "+", "-", "+=", "-=", "==", ">", "<", "="
	CAngle operator -() const;
	CAngle operator !() const;
	CAngle operator +(const CAngle& Ang) const;
	CAngle operator -(const CAngle& Ang) const;
	void operator +=(const CAngle& Ang);
	void operator -=(const CAngle& Ang);
	bool operator ==(const CAngle& Ang) const;
	bool operator !=(const CAngle& Ang) const;
	bool operator >(const CAngle& Ang) const;
	bool operator <(const CAngle& Ang) const;
	bool operator >=(const CAngle& Ang) const;
	bool operator <=(const CAngle& Ang) const;
	
	void operator =(float fRad);
	CAngle operator +(float fRad) const;
	CAngle operator -(float fRad) const;
	void operator +=(float fRad);
	void operator -=(float fRad);
	bool operator ==(float fRad) const;
	bool operator !=(float fRad) const;
	bool operator >(float fRad) const;
	bool operator <(float fRad) const;
	bool operator >=(float fRad) const;
	bool operator <=(float fRad) const;

	bool ApproxEqualTo(const CAngle& ang, float fMaxDiffRad = 0) const;

	// ���㱾��������һ���ǵĲ�(ֻ������ֵ)
	float GetDifference(const CAngle& another) const;

	bool InRange(const CAngle& ang1, const CAngle& ang2) const;
};

DllExport float sin(const CAngle& Ang);
DllExport float cos(const CAngle& Ang);
DllExport float tan(const CAngle& Ang);
DllExport CAngle abs(const CAngle& Ang);
DllExport float AngleDiff(float angle1, float angle2);
DllExport float NormAngle2(float fRad);

class DllExport CPolyRegion
{
public:
	CPnt* m_pVertex;
	int       m_nCount;

public:
	CPolyRegion(int nCount = 0, CPnt* pPnt = NULL);
	~CPolyRegion();

	// �ж�ָ���ĵ��Ƿ�����ڴ˶����������
	bool Contain(const CPnt& pt) const;

	// �жϴ������Ƿ��������һ��ָ��������
	bool Contain(const CPolyRegion& PolyRgn) const;

	bool OverlapWith(const CPolyRegion& PolyRgn) const;

#ifdef _MFC_VER
	// ����Ļ�ϻ��ƴ�ֱ��
	void Draw(CScreenReference& ScrnRef, CDC* pDC, COLORREF crColor, int nWidth = 1, int nPointSize = 1, bool bBigVertex = false);
#endif
};

class vector_velocity;
class CLine;

//////////////////////////////////////////////////////////////////////////////
//   The interface of class "CPosture".
//
class DllExport CPosture : public CPnt
{
public:
	float fThita;      // The direction angle

public:
	// The constructors
	CPosture(float fX, float fY, float fAngle)
	{
		x = fX;
		y = fY;
		fThita = fAngle;
	}
	
	CPosture(float fX, float fY, const CAngle& Angle)
	{
		x = fX;
		y = fY;
		fThita = Angle.m_fRad;
	}
	
	CPosture(const CPnt& pt, const CAngle& Angle)
	{
		x = pt.x;
		y = pt.y;
		fThita = Angle.m_fRad;
	}

	// Default constructor
	CPosture() 
	{
		x = y = 0;
		fThita = 0;
	}

	void Create(float _x, float _y, float _thita)
	{
		x = _x;
		y = _y;
		fThita = _thita;
	}

	void Create(const CPnt& pt, const CAngle& ang)
	{
		x = pt.x;
		y = pt.y;
		fThita = ang.m_fRad;
	}

	void SetPnt(const CPnt& pt)
	{
		x = pt.x;
		y = pt.y;
	}
	
	void SetPnt(float fX, float fY)
	{
		x = fX;
		y = fY;
	}
	
	void SetAngle(const CAngle& ang)
	{
		fThita = ang.m_fRad;
	}
	
	void SetAngle(float fAngle)
	{
		fThita = CAngle::NormAngle(fAngle);
	}
	
	void SetPosture(const CPosture& pst)
	{
		GetPostureObject() = pst;
	}

	void SetPosture(const CPnt& pt, const CAngle& ang)
	{
		SetPnt(pt);
		SetAngle(ang);
	}
	
	void ReverseAngle()
	{
		fThita = CAngle::NormAngle(fThita + PI);
	}

	void SetPosture(float fX, float fY, float fAngle)
	{
		x = fX;
		y = fY;
		fThita = CAngle::NormAngle(fAngle);
	}

	CPosture& GetPostureObject() {return *this;}

	CAngle GetAngle() const
	{
		CAngle ang(fThita);
		return ang;
	}

	// ������ָ�������ĵ������ת
	virtual void Rotate(float fAng, float fCx = 0, float fCy = 0);

	// ����̬��ָ�������ĵ������ת
	virtual void Rotate(float fAng, CPnt ptCenter);
	
	void operator += (const CPosture& pstAnother)
	{
		x += pstAnother.x;
		y += pstAnother.y;
		fThita += pstAnother.fThita;
		fThita = CAngle::NormAngle(fThita);
	}

	void operator -= (const CPosture& pstAnother)
	{
		x -= pstAnother.x;
		y -= pstAnother.y;
		fThita -= pstAnother.fThita;
		fThita = CAngle::NormAngle(fThita);
	}

	// �ڵ�ǰ��̬�Ļ����ϣ����ݸ������ٶ������������һ��ʱ�κ������̬
	CPosture Deduce(const vector_velocity& vel, float interval);

	// ������̬�ǵ�һ�����ߵļн�
	CAngle AngleToLine(const CLine& ln) const;

	void RotatePos(float angle, float cx, float cy);

	// �����������任
	virtual void Transform(const CFrame& frame);

	// ����������任
	virtual void InvTransform(const CFrame& frame);

#ifdef _MFC_VER
	// ����Ļ�ϻ��ƴ���̬
	void Draw(CScreenReference& ScrnRef, CDC* pDC, COLORREF crColor, int nCircleRadiusMm, int nArrowLenMm, int nWidth = 1);

#elif defined QT_VERSION

	// ����Ļ�ϻ��ƴ���̬
	void Draw(CScreenReference& ScrnRef, QPainter* pPainter, QColor crColor, int nCircleRadiusMm, int nArrowLenMm, int nWidth = 1);
#endif
};

CPosture operator + (const CPosture& pst1, const CPosture& pst2);
CPosture operator - (const CPosture& pst1, const CPosture& pst2);

// ����������̬��ʱ����������������������̬�����ʱ���ٶ�����
vector_velocity EstimateVel(CPosture pst1, CPosture pst2, float interval);

//////////////////////////////////////////////////////////////////////////////
//   The interface of class "CVelocity".
class DllExport CVelocity
{
public:
	float fLinear;
	float fAngular;

public:
	// The constructor
	CVelocity(float fLin, float fAng)
	{
		fLinear = fLin;
		fAngular = fAng;
	}

	// Default constructor
	CVelocity() {}
};

class vector_velocity
{
public:
	float vel_x;
	float vel_y;
	float vel_angle;

public:
	vector_velocity(float _vx = 0, float _vy = 0, float _va = 0)
	{
		vel_x = _vx;
		vel_y = _vy;
		vel_angle = _va;
	}

	void SetZero()
	{
		vel_x = vel_y = vel_angle = 0;
	}
};

//////////////////////////////////////////////////////////////////////////////
//   The interface of class "CMoveDir".
enum MoveDirTag {FORWARD, BACKWARD, LEFTWARD = 0, RIGHTWARD = 1};

class DllExport CMoveDir
{
public:
	MoveDirTag m_tagMoveDir;      // The tag for the move direction

public:
	// The constructor
	CMoveDir(MoveDirTag tagMoveDir) {m_tagMoveDir = tagMoveDir;}

	// Default constructor
	CMoveDir() {m_tagMoveDir = FORWARD;}

	// Copy constructor
	void operator = (const CMoveDir& MoveDir);

	// Assignment of move direction
	void operator = (MoveDirTag tagMoveDir);

	// Test if 2 objects are identical
	bool operator == (const CMoveDir& MoveDir) const;

	// Test if 2 objects are not identical
	bool operator != (const CMoveDir& MoveDir) const;

	// Test if the object is of the specified turn direction
	bool operator == (MoveDirTag tagMoveDir) const;

	// Test if the object is not of the specified turn direction
	bool operator != (MoveDirTag tagMoveDir) const;

	// Get the opposite turn direction
	CMoveDir operator !();
};


//////////////////////////////////////////////////////////////////////////////
//   The interface of class "CTurnDir".
enum TurnDirTag {COUNTER_CLOCKWISE, CLOCKWISE};

class DllExport CTurnDir
{
public:
	TurnDirTag m_tagTurnDir;      // The tag for the turn direction

public:
	// The constructor
	CTurnDir(TurnDirTag tagTurnDir) {m_tagTurnDir = tagTurnDir;}

	// Default constructor
	CTurnDir() {m_tagTurnDir = COUNTER_CLOCKWISE;}

	// Copy constructor
	void operator =(CTurnDir TurnDir);

	// Assignment of turn direction
	void operator =(TurnDirTag tagTurnDir);

	// Test if 2 objects are identical
	bool operator == (const CTurnDir& TurnDir) const;

	// Test if 2 objects are not identical
	bool operator != (const CTurnDir& TurnDir) const;

	// Test if the object is of the specified turn direction
	bool operator == (TurnDirTag tagTurnDir) const ;

	// Test if the object is not of the specified turn direction
	bool operator != (TurnDirTag tagTurnDir) const;

	// Get the opposite turn direction
	CTurnDir operator !();
};

class CLine;

//////////////////////////////////////////////////////////////////////////////
//   The interface of class "CTransform".

class DllExport CTransform : public CPosture
{
public:
	CTransform(const CPosture& pstLocal);

	// The constructor
	CTransform(const CPnt& ptOrigin, const CAngle& angSlant);

	// Default constructor
	CTransform() {}
    
    // Init the origin and slant angle of the local frame
	void Init(const CPosture& pstLocal);
	
	void Create(float _x, float _y, float _angle)
	{
		CPosture::Create(_x, _y, _angle);
	}

	void Create(CLine& line1, CLine& line2);

	// A transformation from the local frame to the world frame
	CPnt GetWorldPoint(const CPnt& ptLocal) const;

	// A reverse transformation from the world frame to the local frame
	CPnt GetLocalPoint(const CPnt& ptWorld) const;

	// A transformation from the local frame to the world frame
	CPosture GetWorldPosture(const CPosture& pstLocal) const;

	// A reverse transformation from the world frame to the local frame
	CPosture GetLocalPosture(const CPosture& pstWorld) const;

	// ��һ��ֱ�ߴӾֲ�����ϵ���任����������ϵ��
	CLine GetWorldLine(const CLine& lnLocal) const;

	// ��һ��ֱ�ߴ���������ϵ���任���ֲ�����ϵ��
	CLine GetLocalLine(const CLine& lnWorld) const;

	// ȡ����任
	CTransform Inv();
};

//
//   ����ֱ��: ax + by + c = 0��
//
class DllExport CLineBase
{
public:
	int   m_nId;   // ID��
	float a;       // Normal X
	float b;       // Normal Y
	float c;       // ��ԭ��ľ���

public:
	CLineBase(float _a, float _b, float _c)
	{
		m_nId = 0;
		a = _a;
		b = _b;
		c = _c;
	}

	// ��бʽ����
	CLineBase(const CPnt& pt, float fAng);

	CLineBase()
	{
		m_nId = 0;
		a = b = c = 0;
	}

	// ֱ�Ӳ�������
	void DirectCreate(float _a, float _b, float _c)
	{
		m_nId = 0;
		a = _a;
		b = _b;
		c = _c;
	}

	// ��бʽ����ֱ��
	void Create(float x, float y, float fAng);

	// ͨ���Ե��Ƶ����Իع�������������
	bool CreateFitLine(const CPnt* sp, int num);

	CLineBase& GetLineBaseObject() { return *this; }

	// �����ֱ�ߵ�ָ����ľ���
	float DistanceToPoint(const CPnt& pt) const;

	// ���ֱ����һ�㵽��ֱ�ߵ�ͶӰ
	CPnt GetProjectPoint(const CPnt& pt) const;
	
	// ���ֱ����һֱ�ߵ���ֱ�ߵ�ͶӰ
	CLine GetProjectLine(const CLine& Line) const;

	// ȡ��ֱ�ߵ���б��(����)
	float StdSlantAngleRad() const;
};

//////////////////////////////////////////////////////////////////////////////
//   The interface of class "CLine".
class DllExport CLine : public CLineBase
{
public:
	CPnt   m_ptStart;        // The start point
	CPnt   m_ptEnd;          // The end point
	CAngle     m_angSlant;       // Slant angle
	float      m_fTotalLen;      // The length of the line

public:
	// ��һ�ֹ��캯��
	CLine(const CPnt& ptStart, const CPnt& ptEnd);

	// �ڶ��ֹ��캯��
	CLine(const CPnt& ptStart, const CAngle& angSlant, float fTotalLen);

	// �����ֹ��캯��
	CLine(const CPosture& pstStart, float fTotalLen);

	// �����ֹ��캯��
	CLine(const CLine& Line2);

	// ȱʡ���캯��
	CLine() { m_nId = 0; }

	// ȡ�����������(��Ҫ���ڼ̳�)
	CLine& GetLineObject() {return *this;}

	// �������������߶�
	bool Create(const CPnt& ptStart, const CPnt& ptEnd);

	// ������ʼ�㡢��Ǻͳ��������߶�
	bool Create(const CPnt& ptStart, const CAngle& angSlant, float fTotalLen);

	// ��������ֱ�߶ε����в���(��ǡ�����)
	void ComputeParam();

	// ������ʼ��
	CPnt& GetStartPoint() {return m_ptStart;}

	// ������ֹ��
	CPnt& GetEndPoint() {return m_ptEnd;}

	// ��ת�߶εķ���(����㡢�յ�Ե�)
	void Reverse();

	// �ı��߶εĳ���(�������ӳ�/����)
	bool Resize(float fDist1, float fDist2);

	// ȡ��ֱ�߶εĳ���
	float Length() const { return m_fTotalLen; }

	// ȡ��ֱ�߶γ��ȵ�ƽ��
	float Length2() const;

	// �켣�㺯��
	CPnt TrajFun(float fCurLen) const;

	// ȡ��ֱ�ߵ���б��
   CAngle SlantAngle() const { return m_angSlant; }

	// ȡ���߶ε���б�ǡ�
	float SlantAngleRad() const;

	// �����߶ε��е�
	CPnt GetMidpoint() const;

	// �ж�һ�����Ƿ��ڴ�ֱ��(��)��
	bool ContainPoint(const CPnt& pt, bool bExtend = false) const;

	// �ж�����ֱ���Ƿ�ƽ��
	bool IsParallelTo(const CLine& Line, float fMaxAngDiff = 0) const;

	// �ж�����ֱ���Ƿ�ֱ
	bool IsVerticalTo(const CLine& line, float fMaxAngDiff = 0) const;

	// �ж�����ֱ���Ƿ���
	bool IsColinearWith(const CLine& Line, float fMaxAngDiff, float fMaxDistDiff) const;

	// ȡ������ֱ�ߵĽ���
	bool IntersectLineAt(const CLine& Line, CPnt& pt, float& fDist) const;

	//
	//   Intersects two lines.  Returns true if intersection point exists,
	//   false otherwise.  (*px, *py) will hold intersection point,
	//   *onSegment[12] is true if point is on line segment[12].
	//   px, py, onSegment[12] might be NULL.
	//
	bool Intersect(const CLine& line2, float *px, float *py, bool *onSegment1, bool *onSegment2, float fSmallGate = 1e-6) const;

	// ���������һֱ�ߵĽǶȲ�
	CAngle AngleToLine(const CLine& line2) const;

	// �����ֱ������һ������ֱ�ߵĽǶȲ�(���������н�С��)
	CAngle AngleToUndirectionalLine(const CLine& line2) const;

	// �����ֱ�ߵ�ָ����ľ���
	float DistanceToPoint(const CPnt& pt) const;

	// �����ֱ�ߵ�ָ����ľ���
	float DistanceToPoint(bool bIsSegment, const CPnt& pt, float* pLambda = NULL, CPnt* pFootPoint = NULL) const;

	// �ж�һ�������ĵ��Ƿ񡰴���������ֱ��
	int PointHit(const CPnt& pt, float fDistGate);

	// ��ֱ����ָ�������ĵ������ת
	virtual void Rotate(float fAng, float fCx = 0, float fCy = 0);

	// ��ֱ����ָ�������ĵ������ת
	virtual void Rotate(float fAng, CPnt ptCenter);

	// �任���ֲ�����ϵ
	CLine TransformToLocal(CTransform& trans);

	// �任��ȫ������ϵ
	CLine TransformToGlobal(CTransform& trans);

	// ȡ��ֱ�߶ε�б�ʡ�Y��ؾ��(��ֱ�ߴ�ֱ��X��ʱ��)X��ؾ�
	int GetParam(float* k, float* b, float* c) const;

	// ����ֱ�ߵ������˵����ĸ�����ָ���ĵ�pt����������fDist�з��ش˽�����
	int FindNearPoint(const CPnt& pt, float* pDist = NULL) const;

	// ���㱾�߶ε���һ�߶ε�ͶӰ
	bool GetProjection(const CLine& another, CLine& lineProj, float& fProjDist, float& fProjLen) const;

	// �����������任
	virtual void Transform(const CFrame& frame);

	// ����������任
	virtual void InvTransform(const CFrame& frame);

#ifdef _MFC_VER
	// ����Ļ�ϻ��ƴ�ֱ��
	void Draw(CScreenReference& ScrnRef, CDC* pDC, COLORREF crColor, int nWidth = 1, int nPointSize = 1, bool bBigVertex = false, int nPenStyle = PS_SOLID);
#elif defined QT_VERSION
	// ����Ļ�ϻ��ƴ�ֱ��
	void Draw(CScreenReference& ScrnRef, QPainter* pPainter, QColor crColor, int nWidth = 1, int nPointSize = 1, bool bBigVertex = false, int nPenStyle = 0);
#endif
};

///////////////////////////////////////////////////////////////////////////////
//   ���塰�����ࡱ��
class CRectangle
{
private:
	bool m_bInit;                 // �����Ƿ��Ѿ���ʼ��
	CPnt m_ptLeftTop;             // �������Ͻ�λ��
	CPnt m_ptRightBottom;         // �������½�λ��

public:
	CRectangle(const CPnt& ptLeftTop, const CPnt& ptRightBottom);
	CRectangle(float fLeft, float fTop, float fRight, float fBottom);
	CRectangle();

	// ���ɾ���
	bool Create(const CPnt& ptLeftTop, const CPnt& ptRightBottom);

	// ���ɾ���
	bool Create(float fLeft, float fTop, float fRight, float fBottom);

	// �������
	void Clear();

	// ȡ�����Ͻǵ��λ��
	CPnt GetLeftTopPoint() const { return m_ptLeftTop; }

	// ȡ�����½ǵ��λ��
	CPnt GetRightBottomPoint() const { return m_ptRightBottom; }

	// ȡ�����ĵ��λ��
	CPnt GetCenterPoint() const;

	// �ж�һ�����Ƿ��ھ�������
	bool Contain(float x, float y) const;

	// �ж�һ�����Ƿ��ھ�������
	bool Contain(const CPnt& pt) const;

	// �ж�һ�߶��Ƿ��ھ�������
	bool Contain(const CLine& ln) const;

	// �ж���һ�������Ƿ��ڸþ�������
	bool Contain(const CRectangle& r) const;

	// ȡ�������X����
	float Left() const { return m_ptLeftTop.x; }

	// ȡ�����ҵ�X����
	float Right() const { return m_ptRightBottom.x; }

	// ȡ�����ϵ�Y����
	float Top() const { return m_ptLeftTop.y; }

	// ȡ�����µ�Y����
	float Bottom() const { return m_ptRightBottom.y; }

	// ȡ�þ�������Ŀ��
	float Width() const;

	// ȡ�þ�������ĸ߶�
	float Height() const;

	// �������������С�Ա����ɸ����ĵ�
	void operator += (const CPnt& pt);

	// �������������С�Ա����ɸ�����ֱ��
	void operator += (const CLine& line);

	// �������������С�Ա����ɸ����ľ�������
	void operator += (const CRectangle& rect);

	// ���ļ�װ���������
	bool Load(FILE* fp);

	// ����������д���ļ�
	bool Save(FILE* fp);

#ifdef _MFC_VER
	// ����Ļ�ϻ��ƴ˾���
	void Draw(CScreenReference& ScrnRef, CDC* pDC, COLORREF crColor, int nWidth);

#elif defined QT_VERSION
	// ����Ļ�ϻ��ƴ˾���
	void Draw(CScreenReference& ScrnRef, QPainter* pPainter, QColor crColor, int nWidth);

#endif
};

//////////////////////////////////////////////////////////////////////////////
//   The interface of class "CCircle".
class DllExport CCircle
{
public:
	CPnt m_ptCenter;        // The center of the curve
	float    m_fRadius;         // Radius of the replaced arc
	CPnt m_pt;              // The trajectory point

public:
	// The constructor
	CCircle(const CPnt& ptCenter, float fRadius);

	// Default constructor
	CCircle() {}

	// ���ɴ�Բ
	void Create(const CPnt& ptCenter, float fRadius);

	// Get the radius of its similar arc
	float Radius() const {return m_fRadius;}

	// ȡ���ܳ�
	float Perimeter() const {return 2 * PI * m_fRadius;}

	// ȡ�ô�Բ��һ�������Բ���ֱ�ߵĵ�һ������
	bool IntersectLineAt(const CLine& Line, CPnt& ptNear, float& fDist);

	// �Դ�Բ��ȡһ��ֱ�ߣ�������ȡ������ֱ�߱��浽NewLine��
	bool CutLine(const CLine& Line, CLine& NewLine) const;

	// �ж�һ�����Ƿ���Բ��(��Բ��)
	bool Contain(const CPnt& pt) const;

#ifdef _MFC_VER
	// ����Ļ�ϻ��ƴ�Բ
	void Draw(CScreenReference& ScrnRef, CDC* pDC, COLORREF crColor, int nWidth, int nPenStyle = PS_SOLID);

#elif defined QT_VERSION
	// ����Ļ�ϻ��ƴ�Բ
	void Draw(CScreenReference& ScrnRef, QPainter* pPainter, QColor crColor, int nWidth, int nPenStyle = 0);

#endif
};

//////////////////////////////////////////////////////////////////////////////
//   The interface of class "CArc".
class DllExport CArc
{
public:
	CPnt     m_ptCenter;        // The center of the curve
	CPnt     m_ptStart;         // Start point
	CPnt     m_ptEnd;           // End point
	CTurnDir m_TurnDir;         // Turn direction
	float    m_fTurnAngle;      // Curve's turn angle
	float    m_fCurRadius;      // Radius at the current point
	float    m_fRadius;         // Radius of the replaced arc
	CPnt     m_pt;              // The trajectory point
	CAngle   m_angTangent;      // The tangent angle
	float    m_fCurvature;      // The curvature

	CAngle   m_angStart;        // Slant angle of the start radius
	CTransform m_Transform;     // Coordinates transformation object

public:
	// The constructor
	CArc(const CPnt& ptCenter, const CPnt& ptStart, const CPnt& ptEnd, CTurnDir TurnDir =
		  COUNTER_CLOCKWISE);

	// Default constructor
	CArc() {}

	// Get the total turn angle of the curve 
	float TurnAngle() const {return m_fTurnAngle;}

	// Get the radius of its similar arc
	float Radius() const {return m_fRadius;}

	// Get the current radius
	virtual float CurRadius() const {return m_fCurRadius;}

	// Set the current turn angle to specified a trajectory point
	virtual void SetCurAngle(float fPhi);

	// The trajectory generation function
	virtual CPnt TrajFun() {return m_pt;}

	// The tangent angle generation function
	virtual CAngle TangentFun() {return m_angTangent;}

	// The curvature generation function
	virtual float CurvatureFun() {return m_fCurvature;}

#ifdef _MFC_VER
	// ����Ļ�ϻ��ƴ�Բ
	void Draw(CScreenReference& ScrnRef, CDC* pDC, COLORREF crColor, int nWidth, int nPenStyle = PS_SOLID);

#elif defined QT_VERSION
	// ����Ļ�ϻ��ƴ�Բ
	void Draw(CScreenReference& ScrnRef, QPainter* pPainter, QColor crColor, int nWidth, int nPenStyle = 0);

#endif
};

//////////////////////////////////////////////////////////////////////////////
//   The interface of class "CSpp".
class DllExport CSpp : public CArc
{
public:
	// The constructor
	CSpp(CPnt& ptCenter, CPnt& ptStart, CPnt& ptEnd, CTurnDir TurnDir =
		  COUNTER_CLOCKWISE);

	// Default constructor
	CSpp() {}

	// Set the current turn angle to specified a trajectory point
	virtual void SetCurAngle(float fPhi);
};


//////////////////////////////////////////////////////////////////////////////
//   The interface of class "CSpline".
class DllExport CSpline : public CSpp
{
public:
	float m_fRb;               // Radius of the center arc segment
	float m_fBeita;

private:
	void Solve1(float fPhi);
	void Solve2(float fPhi);

public:
	// The constructor
	CSpline(CPnt& ptCenter, CPnt& ptStart, CPnt& ptEnd, CTurnDir TurnDir =
			  COUNTER_CLOCKWISE);

	// Default constructor
	CSpline() {}

	// Set the current turn angle to specified a trajectory point
	virtual void SetCurAngle(float fPhi);
};

//////////////////////////////////////////////////////////////////////////////
//   The interface of class "CScp".
class DllExport CScp
{
public:
	CPnt   m_ptStart;            // Start point
	CPnt   m_ptEnd;              // End point
	CAngle m_angLane;            // Slant angle of the lanes
	CPnt   m_pt;                 // Trajectory point
	CPnt   m_ptLocal;            // Local trajectory point
	CAngle m_angTangent;         // Tangent angle with respect to world frame
	CAngle m_angTangent0;        // Tangent angle with respect to the lane
	float  m_fCurvature;         // Curvature

	float  m_fXe;                // Xe
	float  m_fYe;                // Ye
	CAngle m_angShift;           // Curve's shifting angle (to the lane)
	CTransform m_Transform;      // Coordinate transformation object

public:
	// The constructor
	CScp(CPnt& ptStart, CPnt& ptEnd, CAngle& angLane);

	// Default constructor
	CScp() {}

	// Caculate the X distance of the curve
	float GetX();

	// Set the current X distance to specified a trajectory point
	void SetCurX(float fX);

	// The trajectory generation function
	CPnt TrajFun();

	// The tangent angle generation function
	CAngle TangentFun(bool bWorldFrame = true);

	// The curvature generation function
	float CurvatureFun();

	// Get the curve's shifting angle with respect to the lane
	CAngle& ShiftAngle();

	float NewtonRoot(float fXk, float fY);

	float ScpFun(float fXk, float fY);
	float ScpFun_(float fXk);

	// Find the X coordinate of the reference point
	bool FindRefX(CPnt& pt, float& fRefX, float &fErrX);

	// Find the error in Y direction
	bool FindErrY(CPnt& pt, float& fErrY);
};

//////////////////////////////////////////////////////////////////////////////
//   The interface of class "CNewCurve".

class DllExport CNewCurve : public CArc
{
private:
	float X0, Y0;
	float X1, Y1;
	float X2, Y2;
	float X3, Y3;

	float A1, A2;
	float B1, B2, B3;
	float C1, C2, C3;
	float D1, D2, D3;
	float E1, E2;
	float F1, F2, F3, F4, F5, F6;

public:
   float Ds, DThita;

public:
	// The constructor
	CNewCurve(CPnt& ptCenter, CPnt& ptStart, CPnt& ptEnd, CTurnDir TurnDir);

	// Default constructor
	CNewCurve() {}

	// Set the current turn angle to specified a trajectory point
	void SetCurAngle(float fPhi);
};
#endif
