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

#define SCANNER_ARROW_LEN           0.45f           // ��ͷ��450mm
#define SCANNER_RADIUS              0.08f           // ��ʾɨ����λ�˵�Բ�εİ뾶

extern CFeatureCreationParam FeatureCreationParam;


///////////////////////////////////////////////////////////////////////////////

//
//   �������캯����
//
CSlamStepData::CSlamStepData(const CSlamStepData& Obj)
{
	m_pstMoveEst = Obj.m_pstMoveEst;
	m_scanLocal = Obj.m_scanLocal;
	m_scanGlobal = Obj.m_scanGlobal;
	m_featureLocal = Obj.m_featureLocal;
}	

//
//   �������ļ���ȡԭʼɨ������ݡ�
//
bool CSlamStepData::LoadRawScanBinary(FILE* fp, const CScannerGroupParam& Param, int nFileVersion, bool bFirstStep)
{
	m_scanLocal.clear();
	m_scanLocal.resize(Param.size());

	m_scanGlobal.clear();
	m_scanGlobal.resize(Param.size());

	m_featureLocal.clear();
	m_featureLocal.resize(Param.size());

	// ����ļ��汾��V2.00����
	if (nFileVersion < 200)
	{
		// ����۲���̬
		float f[3];
		if (fread(&f, sizeof(float), 3, fp) != 3)
			return false;

		m_pstMoveEst.x = f[0];
		m_pstMoveEst.y = f[1];
		m_pstMoveEst.fThita = f[2];
	}

	// ����ļ��汾����V2.00
	else
	{
		unsigned int u, uTimeStamp;
		if (fread(&u, sizeof(int), 1, fp) != 1)
			return false;

		// �汾��V2.1����ʱ�����ݾ���ʱ���
		if (nFileVersion >= 210)
		{
			if (fread(&uTimeStamp, sizeof(int), 1, fp) != 1)
				return false;

			// Ϊ�����̬����ʱ���
			m_pstMoveEst.Stamp(uTimeStamp);
		}

		float f[3];

		// (u & BIT(1)) != 0 , ��ʾ�����ٶ�����
		if ((u & 0x04) != 0)
		{
			if (fread(&f, sizeof(float), 3, fp) != 3)
				return false;

			m_vel.x = f[0];
			m_vel.y = f[1];
			m_vel.fThita = f[2];
		}

		// ����۲���̬
		if (fread(&f, sizeof(float), 3, fp) != 3)
			return false;

		m_pstMoveEst.x = f[0];
		m_pstMoveEst.y = f[1];
		m_pstMoveEst.fThita = f[2];

		// (u & BIT(1)) != 0 , ��ʾ���������ֵ̬
		if ((u & 0x02) != 0)
		{
			if (fread(&f, sizeof(float), 3, fp) != 3)
				return false;

			// �ڴ˶�������˵ľ�����̬
			m_pst.x = f[0];
			m_pst.y = f[1];
			m_pst.fThita = f[2];

			if (nFileVersion >= 210)
				m_pst.Stamp(uTimeStamp);
		}
	}

	// ���ݼ��ĵ�һ������̬��������Ϊ�㣡
	if (bFirstStep)
		m_pstMoveEst.SetPosture(0, 0, 0);

	// �ֱ��ȡ������������ɨ������
	for (size_t i = 0; i < Param.size(); i++)
	{
		// ����ֲ���������
		if (!m_scanLocal[i].LoadBinary(fp, m_pstMoveEst, Param[i], nFileVersion))
			return false;
		
		// (���ھֲ�����)������������
		if (!m_featureLocal[i].CreateFromScan(m_scanLocal[i], &FeatureCreationParam.m_RefParam))
			return false;
	}

	return true;
}

//
//   ��ԭʼɨ�������д��������ļ���
//
bool CSlamStepData::SaveRawScanBinary(FILE* fp, int nFileVersion, bool bSaveGlobalPosture)
{
	// ����ļ��汾����2.00����Ҫд����̬����0����ʾ�����д�������̬
	// (�����̬����Ϊ0x02�����ʾ���������д�������̬�;�����̬)
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

		// ���Ҫ�����ݸ�ʽ�к��о�����̬������д����
		if ((u & 0x02) != 0)
		{
			f[0] = m_pst.x;
			f[1] = m_pst.y;
			f[2] = m_pst.fThita;

			if (fwrite(&f, sizeof(float), 3, fp) != 3)
				return false;
		}
	}

	// д���������
	for (size_t i = 0; i < m_scanLocal.size(); i++)
		if (!m_scanLocal[i].SaveBinary(fp, nFileVersion))
			return false;

	return true;
}

//   ����������ݡ�
//
void CSlamStepData::Clear()
{
}

//
//   ���ݸ�����ԭʼ����ϵ��������ת������������ϵ�¡�
//
void CSlamStepData::CreateGlobalData(CPosture& pstInit, const CScannerGroupParam& ScannerGroupParam)
{
	CFrame frame1(pstInit);
	pstInit = m_pstMoveEst;
	pstInit.InvTransform(frame1);

	// �����Ʊ任����������ϵ��
	for (size_t i = 0; i < m_scanLocal.size(); i++)
	{
		// ���ݼ������ڻ������ϵ���԰�װλ�ü���������������ϵ�µ���̬
		CFrame frmRobot(pstInit);
		const CLaserScannerParam& Param = ScannerGroupParam[i];             // ��Ӧ�������Ĳ���
		CPosture pstScanner(Param.m_pst);        // ��������Ի����˵İ�װ��̬
		pstScanner.InvTransform(frmRobot);

		// �Ƚ�ȫ��ɨ�����ݳ�ʼ��Ϊԭ�ֲ�ɨ������
		m_scanGlobal[i] = m_scanLocal[i];
//		m_featureGlobal[i] = m_featureLocal[i];

		// Ӧ��ɨ��Ƕ�����
		m_scanGlobal[i].ApplyNewScannerAngles(Param.m_AppAngleRange);
//		m_featureGlobal[i].ApplyNewScannerAngles(Param.m_AppAngleRange);
	
		// �ڴ˱����Щǿ�����
		m_scanGlobal[i].MarkReflectivePoints(FeatureCreationParam.m_RefParam.nMinReflectorIntensity);

		// �任��ȫ������ϵ��
		m_scanGlobal[i].InvTransform(pstScanner);
		m_scanGlobal[i].m_poseScanner.GetPostureObject() = pstScanner;

//		m_featureGlobal[i].InvTransform(pstScanner);
	}
}

//
//   ȡ��ָ��������ɢ�㡣
//
CScanPoint* CSlamStepData::GetWorldRawPoint(int nScannerId, int nIdx)
{
	return &m_scanGlobal[nScannerId].m_pPoints[nIdx];
}

//
//   �����������ݽ���ָ��������任��
//
void CSlamStepData::Transform(const CFrame& frame)
{
	for (size_t i = 0; i < m_scanGlobal.size(); i++)
		m_scanGlobal[i].Transform(frame);
}

//
//   �����������ݽ���ָ����������任��
//
void CSlamStepData::InvTransform(const CFrame& frame)
{
	for (size_t i = 0; i < m_scanGlobal.size(); i++)
		m_scanGlobal[i].InvTransform(frame);
}

//
//   ���ù���ɨ��Ƕȷ�Χ��Լ����
//
void CSlamStepData::ApplyScanAngleRule(int nScannerId, float fMinAngle, float fMaxAngle)
{
	m_scanLocal[nScannerId].ApplyScanAngleRule(fMinAngle, fMaxAngle);
	m_scanGlobal[nScannerId].ApplyScanAngleRule(fMinAngle, fMaxAngle);
}

// ���ù���ɨ������Լ��
void CSlamStepData::ApplyScanDistRule(int nScannerId, float fMinDist, float fMaxDist)
{
	m_scanLocal[nScannerId].ApplyScanDistRule(fMinDist, fMaxDist);
	m_scanGlobal[nScannerId].ApplyScanDistRule(fMinDist, fMaxDist);
}

//
//   �ж�ָ������Ļ���Ƿ񴥱��������е�ĳ��ԭʼ�㡣
//   ����ֵ��
//     -1 : û�ҵ�
//    ����: �ӽ��Ϊ32λ�޷�������D16-D23λ(8λ)����������ţ�(D0-D15)16λΪ���ı�š�
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
//   �ж�ָ������Ļ���Ƿ񴥱��������еĻ�����λ�ˡ�
//
int CSlamStepData::PointHitPose(const CPnt& pt, float fDistGate)
{
	if (fDistGate < SCANNER_RADIUS)
		fDistGate = SCANNER_RADIUS;

	return (m_WorldFeatureSet.m_pstObserver.DistanceTo(pt) < fDistGate);
}
#endif

//
//   �Ա����е�ָ��������ɨ�����ݽ���ɨ��ƥ�䡣
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

		// ����ԭʼ����
		m_scanGlobal[i].Plot(ScrnRef, pDC, clrRawPoint, clrHighLightRawPoint, bShowScanner);

		// �����Ҫ��ʾ����
		if (bShowFeature)
		{
			// �Ƚ�����ת��������λ�ã��ٽ�����ʾ
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
	// ����ԭʼ����
	m_scanGlobal.Plot(ScrnRef, pPainter, clrRawPoint, clrHightLightRawPoint, bShowScanner);
	if (bShowFeature)
		m_featureLocal.Plot(ScrnRef, pDC, clrFeature);

}


#endif
