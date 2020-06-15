#include "stdafx.h"
#include "SlamStepData.h"
#include "ScrnRef.h"
#include "Frame.h"
#include "FeatureCreationParam.h"
#include "CsmMatcher.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif

#define SCANNER_ARROW_LEN           0.45f           // 箭头长450mm
#define SCANNER_RADIUS              0.08f           // 显示扫描器位姿的圆形的半径

extern CFeatureCreationParam FeatureCreationParam;


///////////////////////////////////////////////////////////////////////////////

//
//   拷贝构造函数。
//
CSlamStepData::CSlamStepData(const CSlamStepData& Obj)
{
	m_pstMoveEst = Obj.m_pstMoveEst;
	m_scanLocal = Obj.m_scanLocal;
	m_scanGlobal = Obj.m_scanGlobal;
	m_featureLocal = Obj.m_featureLocal;
}	

//
//   二进制文件读取原始扫描点数据。
//
bool CSlamStepData::LoadRawScanBinary(FILE* fp, const CScannerGroupParam& Param, int nFileVersion, bool bFirstStep)
{
	m_scanLocal.clear();
	m_scanLocal.resize(Param.size());

	m_scanGlobal.clear();
	m_scanGlobal.resize(Param.size());

	m_featureLocal.clear();
	m_featureLocal.resize(Param.size());

	// 如果文件版本在V2.00以下
	if (nFileVersion < 200)
	{
		// 读入观测姿态
		float f[3];
		if (fread(&f, sizeof(float), 3, fp) != 3)
			return false;

		m_pstMoveEst.x = f[0];
		m_pstMoveEst.y = f[1];
		m_pstMoveEst.fThita = f[2];
	}

	// 如果文件版本高于V2.00
	else
	{
		unsigned int u, uTimeStamp;
		if (fread(&u, sizeof(int), 1, fp) != 1)
			return false;

		// 版本在V2.1以上时，数据具有时间戳
		if (nFileVersion >= 210)
		{
			if (fread(&uTimeStamp, sizeof(int), 1, fp) != 1)
				return false;

			// 为相对姿态打上时间戳
			m_pstMoveEst.Stamp(uTimeStamp);
		}

		float f[3];

		// (u & BIT(1)) != 0 , 表示读入速度向量
		if ((u & 0x04) != 0)
		{
			if (fread(&f, sizeof(float), 3, fp) != 3)
				return false;

			m_vel.x = f[0];
			m_vel.y = f[1];
			m_vel.fThita = f[2];
		}

		// 读入观测姿态
		if (fread(&f, sizeof(float), 3, fp) != 3)
			return false;

		m_pstMoveEst.x = f[0];
		m_pstMoveEst.y = f[1];
		m_pstMoveEst.fThita = f[2];

		// (u & BIT(1)) != 0 , 表示读入绝对姿态值
		if ((u & 0x02) != 0)
		{
			if (fread(&f, sizeof(float), 3, fp) != 3)
				return false;

			// 在此读入机器人的绝对姿态
			m_pst.x = f[0];
			m_pst.y = f[1];
			m_pst.fThita = f[2];

			if (nFileVersion >= 210)
				m_pst.Stamp(uTimeStamp);
		}
	}

	// 数据集的第一步，姿态增量必须为零！
	if (bFirstStep)
		m_pstMoveEst.SetPosture(0, 0, 0);

	// 分别读取各个激光器的扫描数据
	for (size_t i = 0; i < Param.size(); i++)
	{
		// 读入局部点云数据
		if (!m_scanLocal[i].LoadBinary(fp, m_pstMoveEst, Param[i], nFileVersion))
			return false;
		
		// (基于局部数据)生成所有特征
		if (!m_featureLocal[i].CreateFromScan(m_scanLocal[i], &FeatureCreationParam.m_RefParam))
			return false;
	}

	return true;
}

//
//   将原始扫描点数据写入二进制文件。
//
bool CSlamStepData::SaveRawScanBinary(FILE* fp, int nFileVersion, bool bSaveGlobalPosture)
{
	// 如果文件版本高于2.00，需要写入姿态类型0，表示后面仅写入相对姿态
	// (如果姿态类型为0x02，则表示后面会连续写入相对姿态和绝对姿态)
	if (nFileVersion >= 200)
	{
		unsigned int u = 0;
		if (bSaveGlobalPosture)
			u |= 0x02;

		if (fwrite(&u, sizeof(int), 1, fp) != 1)
			return false;

		float f[3];
		f[0] = m_pstMoveEst.x;
		f[1] = m_pstMoveEst.y;
		f[2] = m_pstMoveEst.fThita;

		if (fwrite(&f, sizeof(float), 3, fp) != 3)
			return false;

		// 如果要求数据格式中含有绝对姿态，现在写入它
		if ((u & 0x02) != 0)
		{
			f[0] = m_pst.x;
			f[1] = m_pst.y;
			f[2] = m_pst.fThita;

			if (fwrite(&f, sizeof(float), 3, fp) != 3)
				return false;
		}
	}

	// 写入点云数据
	for (size_t i = 0; i < m_scanLocal.size(); i++)
		if (!m_scanLocal[i].SaveBinary(fp, nFileVersion))
			return false;

	return true;
}

//   清除所有数据。
//
void CSlamStepData::Clear()
{
}

//
//   根据给定的原始坐标系，将数据转换到世界坐标系下。
//
void CSlamStepData::CreateGlobalData(CPosture& pstInit, const CScannerGroupParam& ScannerGroupParam)
{
	CFrame frame1(pstInit);
	pstInit = m_pstMoveEst;
	pstInit.InvTransform(frame1);

	// 将点云变换到世界坐标系中
	for (size_t i = 0; i < m_scanLocal.size(); i++)
	{
		// 根据激光器在机器人上的相对安装位置计算其在世界坐标系下的姿态
		CFrame frmRobot(pstInit);
		const CLaserScannerParam& Param = ScannerGroupParam[i];             // 对应激光器的参数
		CPosture pstScanner(Param.m_pst);        // 激光器相对机器人的安装姿态
		pstScanner.InvTransform(frmRobot);

		// 先将全局扫描数据初始化为原局部扫描数据
		m_scanGlobal[i] = m_scanLocal[i];
//		m_featureGlobal[i] = m_featureLocal[i];

		// 应用扫描角度限制
		m_scanGlobal[i].ApplyNewScannerAngles(Param.m_AppAngleRange);
//		m_featureGlobal[i].ApplyNewScannerAngles(Param.m_AppAngleRange);
	
		// 在此标记哪些强反光点
		m_scanGlobal[i].MarkReflectivePoints(FeatureCreationParam.m_RefParam.nMinReflectorIntensity);

		// 变换到全局坐标系中
		m_scanGlobal[i].InvTransform(pstScanner);
		m_scanGlobal[i].m_poseScanner.GetPostureObject() = pstScanner;

//		m_featureGlobal[i].InvTransform(pstScanner);
	}
}

//
//   取得指定的世界散点。
//
CScanPoint* CSlamStepData::GetWorldRawPoint(int nScannerId, int nIdx)
{
	return &m_scanGlobal[nScannerId].m_pPoints[nIdx];
}

//
//   将本步的数据进行指定的坐标变换。
//
void CSlamStepData::Transform(const CFrame& frame)
{
	for (size_t i = 0; i < m_scanGlobal.size(); i++)
		m_scanGlobal[i].Transform(frame);
}

//
//   将本步的数据进行指定的坐标逆变换。
//
void CSlamStepData::InvTransform(const CFrame& frame)
{
	for (size_t i = 0; i < m_scanGlobal.size(); i++)
		m_scanGlobal[i].InvTransform(frame);
}

//
//   启用关于扫描角度范围的约束。
//
void CSlamStepData::ApplyScanAngleRule(int nScannerId, float fMinAngle, float fMaxAngle)
{
	m_scanLocal[nScannerId].ApplyScanAngleRule(fMinAngle, fMaxAngle);
	m_scanGlobal[nScannerId].ApplyScanAngleRule(fMinAngle, fMaxAngle);
}

// 启用关于扫描距离的约束
void CSlamStepData::ApplyScanDistRule(int nScannerId, float fMinDist, float fMaxDist)
{
	m_scanLocal[nScannerId].ApplyScanDistRule(fMinDist, fMaxDist);
	m_scanGlobal[nScannerId].ApplyScanDistRule(fMinDist, fMaxDist);
}

//
//   判断指定的屏幕点是否触本步数据中的某个原始点。
//   返回值：
//     -1 : 没找到
//    其它: 视结果为32位无符号数，D16-D23位(8位)代表激光器编号，(D0-D15)16位为光点的编号。
//
int CSlamStepData::PointHitRawPoint(const CPnt& pt, float fDistGate)
{
	for (size_t i = 0; i < m_scanGlobal.size(); i++)
	{
		for (size_t j = 0; j < m_scanGlobal[i].m_nCount; j++)
		{
			if (m_scanGlobal[i].m_pPoints[j].DistanceTo(pt) < fDistGate)
				return (int)((i << 16) |j);
		}
	}
	return -1;
}

#if 0
//
//   判断指定的屏幕点是否触本步数据中的机器人位姿。
//
int CSlamStepData::PointHitPose(const CPnt& pt, float fDistGate)
{
	if (fDistGate < SCANNER_RADIUS)
		fDistGate = SCANNER_RADIUS;

	return (m_WorldFeatureSet.m_pstObserver.DistanceTo(pt) < fDistGate);
}
#endif

//
//   对本步中的指定传感器扫描数据进行扫描匹配。
//
bool CSlamStepData::MatchScans(int nScanId1, int nScanId2, CPosture& result)
{
	CCsmScan cscan1(m_scanGlobal[nScanId1]);
	CCsmScan cscan2(m_scanGlobal[nScanId2]);
	CCsmMatcher Matcher;
	
	sm_result sr;
	if (Matcher.Match(&cscan1, &cscan2, sr))
	{
		result.x = sr.x[0];
		result.y = sr.x[1];
		result.fThita = sr.x[2];

		return true;
	}
	else
		return false;
}

#ifdef _MFC_VER

void CSlamStepData::Plot(CScreenReference& ScrnRef, CDC* pDC, COLORREF clrRawPoint, COLORREF clrHighLightRawPoint,
	bool bShowScanner, bool bShowFeature, COLORREF clrFeature)
{
	for (size_t i = 0; i < m_scanGlobal.size(); i++)
	{
		if (i == 1)
			clrRawPoint = RGB(0, 200, 200);

		// 绘制原始点云
		m_scanGlobal[i].Plot(ScrnRef, pDC, clrRawPoint, clrHighLightRawPoint, bShowScanner);

		// 如果需要显示特征
		if (bShowFeature)
		{
			// 先将特征转换到绝对位置，再进行显示
			CFeatureSet absFeatures = m_featureLocal[i];
			absFeatures.InvTransform(m_scanGlobal[i].m_poseScanner);
			absFeatures.Plot(ScrnRef, pDC, clrFeature);
		}
	}
}

#elif defined QT_VERSION

void CSlamStepData::Plot(CScreenReference& ScrnRef, QPainter* pPainter, QColor clrRawPoint, QColor clrHightLightRawPoint,
	bool bShowScanner, bool bShowFeature, QColor clrFeature)
{
	// 绘制原始点云
	m_scanGlobal.Plot(ScrnRef, pPainter, clrRawPoint, clrHightLightRawPoint, bShowScanner);
	if (bShowFeature)
		m_featureLocal.Plot(ScrnRef, pDC, clrFeature);

}


#endif
