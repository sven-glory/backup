#include <stdafx.h>
#include "LineMatcher.h"
#include "DebugTrace.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif

//
//    ��������ͷ��̬��һ��ֱ�߶Σ������Ӧ���߶������˵��ɨ���(�ٶ�����ʱ�뷽��ɨ��)��
//
void FindScanAngles(CPosture& pstScanner, CLine& Line, CAngle& ang1, CAngle& ang2)
{
	// ȡ�ü���ͷ��λ�úͷ���Ƕ�
	CPnt ptScanner = pstScanner.GetPntObject();
	CAngle angScanner = pstScanner.GetAngle();

	// ����Ӽ���ͷ���߶�����ֱ�ߣ����������뼤��ͷ��̬�ǵļн�
	CLine lnToStartPoint(ptScanner, Line.m_ptStart);
	CAngle angStartLine = lnToStartPoint.m_angSlant - angScanner;

	// ����Ӽ���ͷ���߶��յ��ֱ�ߣ����������뼤��ͷ��̬�ǵļн�
	CLine lnToEndPoint(ptScanner, Line.m_ptEnd);
	CAngle angEndLine = lnToEndPoint.m_angSlant - angScanner;

	// �ٶ�����ͷ������ʱ�뷽����ת���жϼ����������ɨ��ֱ�ߵ���㻹���յ�
	CAngle angDiff = angEndLine - angStartLine;

	// ��������ǶȲ���I, II���ޣ�˵����ɨ��m_ptStart
	if (angDiff.m_fRad < PI)
	{
		ang1 = angStartLine;
		ang2 = angEndLine;
	}

	// ����˵����ɨ��m_ptEnd
	else
	{
		ang1 = angEndLine;
		ang2 = angStartLine;
	}
}

///////////////////////////////////////////////////////////////////////////////

//
//   ���òο�����ͼ��
//
void CLineMatcher::SetRefFeatures(CLineFeatureSet* pLineFeatures)
{
	if (pLineFeatures != NULL)
		m_RefLineFeatures = *pLineFeatures;
	else
		m_RefLineFeatures.Clear();
}

//
//   ���õ�ǰ����ͼ��
//
void CLineMatcher::SetCurFeatures(CPosture& pstOdometry, CLineFeatureSet* pLineFeatures)
{
	m_pstOdometry = pstOdometry;

	if (pLineFeatures != NULL)
		m_CurLineFeatures = *pLineFeatures;
	else
		m_CurLineFeatures.Clear();
}

//
//   ���п���ƥ�䣬�ҵ��ӡ��ֲ�������-->��ȫ��������������任��
//   ����ֵ��
//      > 0 : ƥ��ɹ�
//      < 0 : �������(-2:��������; -3:���̾����쳣)
//
int CLineMatcher::QuickMatch(CTransform& trans)
{
	// �������ֵ�ƥ��ֱ�߶Բ�����2��ʱ������Ϊ��׼�ɹ�
	if (QuickRegisterLines() < 2)
		return FM_FEATURES_NOT_ENOUGH;

	// ����ֱ�߼�ƥ��
	if (!m_LineMatchList.FindTransform())
		return FM_MATRIX_ERROR;

	trans = m_LineMatchList.GetTransform();

	return FM_OK;
}

//
//   ���б��ؾֲ�ƥ�䡣
//
int CLineMatcher::LocalMatch(CTransform& trans)
{
//	return m_PointMatcher.LocalMatch(trans);
	return -1;
}

//
//   �����е�ֱ���������п�����׼��������ֱ��ƥ���
//
int CLineMatcher::QuickRegisterLines()
{
	// �����ԭ����ƥ��ֱ������
	m_LineMatchList.clear();

	m_trans.Init(m_pstOdometry);

	DebugTrace(_T("Dumping RefLineFeatures:\n"));
	m_RefLineFeatures.Dump();

	DebugTrace(_T("\nDumping CurLineFeatures:\n"));
	m_CurLineFeatures.Dump();

	// ��Բ���ֱ�߶μ��ϣ����ν��п���
	for (int i = 0; i < m_RefLineFeatures.GetCount(); i++)
	{
		CLineMatchPair Pair;
		CPnt ptFoot1;

		// ƥ�����(Ҫ��ÿ���ο�ֱ�߶�������һ����ǰֱ��ƥ��)
		int nMatchCount = 0;

		// ȡ�òο�ֱ�߶�
		CLineFeature& Line1 = m_RefLineFeatures.GetLineFeature(i);

		// �жϲο�ֱ�߶ε���Чʶ�����Ƿ��뵱ǰ��̬���
		CPnt& ptScanner = m_pstOdometry.GetPntObject();
		CPnt& ptStart = Line1.m_ptStart;
		CAngle ang(ptStart, ptScanner);
		CAngle angDiff = ang - Line1.SlantAngle();

		// ���ڵ�����Ч��������˶Թ������Ƿ����
		if (Line1.m_nWhichSideToUse == FEATURE_DIR_FRONT_SIDE_ONLY)
		{
			if (angDiff.Quadrant() > 2)          // �ǶȲ����180�ȣ������治��
				continue;
		}
		else if (Line1.m_nWhichSideToUse == FEATURE_DIR_BACK_SIDE_ONLY)
		{
			if (angDiff.Quadrant() <= 2)         // �ǶȲ�С��180�ȣ������治��
				continue;
		}

		Line1.m_nId = i;

		// ���㵱ǰ��̬�ڲο��߶��ϵĴ�ֱͶӰ��ptFoot1����ȡ��ͶӰ�߳���
		float fDist1 = Line1.DistanceToPoint(false, ptScanner, NULL, &ptFoot1);

		// ����ӵ�ǰ����ͷ��̬���ο�ֱ�ߵ�(��ʱ��)ת��
		CAngle ang1 = Line1.SlantAngle();

		CAngle angScan11, angScan12;

		// �����߶������˵㵽����ͷ�������뼤��ͷ��ǰ��̬�ļн�
		FindScanAngles(m_pstOdometry, Line1, angScan11, angScan12);

		// �õ�����Line2ɨ��ǵ�����Χ
		angScan11 -= SIASUN_MATCHER_ANGLE_WINDOW;
		angScan12 += SIASUN_MATCHER_ANGLE_WINDOW;

		// ���ο��쵱ǰֱ�߼��ϳ�Ա
		for (int j = 0; j < m_CurLineFeatures.GetCount(); j++)
		{
			CPnt ptOrigin(0, 0);
			CPnt ptFoot2;

			// ȡ�õ�ǰֱ�߶�
			CLineFeature& line2 = m_CurLineFeatures.GetLineFeature(j);

			CPnt ptStart = line2.m_ptStart;
			CPnt ptEnd = line2.m_ptEnd;

			CLine Line2(ptStart, ptEnd);
			Line2.m_nId = j;

			float fLen2 = Line2.Length();

			// ���㵱ǰ��̬�ڲο��߶��ϵĴ�ֱͶӰ��ptFoot2����ȡ��ͶӰ�߳���
			float fDist2 = Line2.DistanceToPoint(false, m_pstOdometry, NULL, &ptFoot2);

			// ����ӵ�ǰ��ͷ��̬����ǰֱ�ߵ�(��ʱ��)ת��
			CAngle ang2 = Line2.SlantAngle();

			CAngle angScan21, angScan22;

			// �����߶������˵㵽����ͷ�������뼤��ͷ��ǰ��̬�ļн�
			FindScanAngles(m_pstOdometry, Line2, angScan21, angScan22);

			// ����仯��������400���Ƕȱ仯������15�ȣ��������˵�����ɨ���������Χ֮��
			if ((fabs(fDist2 - fDist1) < 0.4f) &&
				(ang1.GetDifference(ang2) < SIASUN_MATCHER_ANGLE_WINDOW) /*&&
				angScan21.InRange(angScan11, angScan12) &&
				angScan22.InRange(angScan11, angScan12)*/)
			{
				Pair.Create(j, i, Line2, Line1, /*m_pstOdometry*/CPosture(0, 0, 0), ang1 - ang2);

				m_LineMatchList.Add(Pair);
			}
		}
	}

#ifdef SHOW_DEBUG_MSG
	DebugTrace(_T("pstOdometry: x=%.4f, y=%.4f, t=%.4f\n"), m_pstOdometry.x, m_pstOdometry.y, m_pstOdometry.fThita);

	DebugTrace(_T("Dumping Line match List 1:\n"));
	m_LineMatchList.Dump();
#endif

	// ����ƥ�����й��˴���ȥ����Щ������ġ������Ƕ��ض�Ӧ����
	m_LineMatchList.Filter();

#ifdef SHOW_DEBUG_MSG
	DebugTrace(_T("Dumping Line match List 2:\n"));
	m_LineMatchList.Dump();
#endif

	// �ж��Ƿ����㹻���ж�λ��ֱ��ƥ���
	int nCount = 0;
	for (int i = 0; i < m_LineMatchList.GetCount() - 1; i++)
	{
		CLine& Line1 = m_LineMatchList.at(i).m_lnLocal;
		for (int j = i + 1; j < m_LineMatchList.GetCount(); j++)
		{
			CLine& Line2 = m_LineMatchList.at(j).m_lnLocal;
			CAngle angDiff = Line1.AngleToUndirectionalLine(Line2);

			// �����ֱ�߼нǴ���30�ȣ�������ڶ�λ
			if (angDiff > CAngle(30.0f, IN_DEGREE))
				nCount++;
		}
	}

	// ���ؿ��õ�ƥ��ֱ�߶�����
	return nCount;
}

bool CLineMatcher::Load(FILE* fp)
{
	fscanf(fp, "%f\t%f\t%f\n", &m_pstOdometry.x, &m_pstOdometry.y, &m_pstOdometry.fThita);

	m_RefLineFeatures.clear();
	m_CurLineFeatures.clear();
	
	m_RefLineFeatures.LoadText(fp);
	m_CurLineFeatures.LoadText(fp);
	return true;
}

bool CLineMatcher::Save(FILE* fp)
{
	fprintf(fp, "%f\t%f\t%f\n", m_pstOdometry.x, m_pstOdometry.y, m_pstOdometry.fThita);
	fprintf(fp, "0\n");
	m_CurLineFeatures.SaveText(fp);
	return true;
}
