#include <stdafx.h>
#include "ScannerParam.h"
#include "Frame.h"

///////////////////////////////////////////////////////////////////////////////

CLaserScannerParam::CLaserScannerParam()
{
	Set(-PI, PI, 3600);
	m_pst.Create(0, 0, 0);
	m_bUseCreateModel = m_bUseLocalize = true;
}

//
//   ������ʼ�ǡ���ֹ�ߺ�ɨ��������
//
void CLaserScannerParam::Set(float fStartAngle, float fEndAngle, int nLineCount)
{
	m_fStartAngle = fStartAngle;
	m_fEndAngle = fEndAngle;
	m_nLineCount = nLineCount;
	m_fReso = (m_fEndAngle - m_fStartAngle) / nLineCount;
}

//
//   �Ӷ������ļ��ж�ȡ������
//
bool CLaserScannerParam::LoadBinary(FILE* fp, int nFileVersion)
{
	float f[3];
	int n;

	if (fread(f, sizeof(float), 2, fp) != 2)
		return false;

	if (fread(&n, sizeof(int), 1, fp) != 1)
		return false;

	Set(f[0], f[1], n);

	// ���汾��V2.00����ʱ����Ҫ���뼤��������԰�װ��̬
	if (nFileVersion >= 200)
	{
		if (fread(f, sizeof(float), 3, fp) != 3)
			return false;

		m_pst.Create(f[0], f[1], f[2]);

		// �ڴ���ʱ�趨������ʵ�ʿ��ýǶȷ�Χ(����Ӧ�������ļ�����)
		int nRangeCount;
		if (fread(&nRangeCount, sizeof(int), 1, fp) != 1)
			return false;

		for (int i = 0; i < nRangeCount; i++)
		{
			if (fread(f, sizeof(float), 2, fp) != 2)
				return false;

			CRange range(f[0], f[1]);
			m_AppAngleRange.push_back(range);
		}
	}
	else
	{
		CRange range(-PI, PI);
		m_AppAngleRange.push_back(range);
	}
	return true;
}

//
//   ��������ļ���д�������
//
bool CLaserScannerParam::SaveBinary(FILE* fp, int nFileVersion)
{
	float f[3] = { m_fStartAngle, m_fEndAngle };

	if (fwrite(f, sizeof(float), 2, fp) != 2)
		return false;

	if (fwrite(&m_nLineCount, sizeof(int), 1, fp) != 1)
		return false;

	// ���汾��2.0����ʱ����Ҫ���뼤��������԰�װ��̬
	if (nFileVersion >= 200)
	{
		f[0] = m_pst.x;
		f[1] = m_pst.y;
		f[2] = m_pst.fThita;

		if (fwrite(f, sizeof(float), 3, fp) != 3)
			return false;

		int nRangeCount = (int)m_AppAngleRange.size();
		if (fwrite(&nRangeCount, sizeof(int), 1, fp) != 1)
			return false;

		for (int i = 0; i < nRangeCount; i++)
		{
			f[0] = m_AppAngleRange[i].fFrom;
			f[1] = m_AppAngleRange[i].fTo;

			if (fwrite(f, sizeof(float), 2, fp) != 2)
				return false;
		}
	}

	return true;
}

///////////////////////////////////////////////////////////////////////////////

//
//   ���õ�ǰɨ����(���:nCurScanner)��ο�ɨ����(���:nRefScaner)�������̬��ϵ��
//   ע��ֻ�Ķ�nCurScanner�İ�װ��̬��nRefScanner�İ�װ��̬����������
//
void CScannerGroupParam::SetRelativePosture(int nRefScanner, int nCurScanner, const CPosture& pst)
{
	const CLaserScannerParam& ref = at(nRefScanner);
	CLaserScannerParam& cur = at(nCurScanner);

	CFrame frmRef(ref.m_pst);
	CPosture pst1(pst);

#if 0
	pst1.x = -pst1.x;
	pst1.y = -pst1.y;
	pst1.fThita = -pst1.fThita;
#endif

	// �õ���ǰ�������ڻ���������ϵ�ڵ���̬
	pst1.InvTransform(frmRef);
	cur.m_pst = pst1;
}

//
//   �Ӷ������ļ��ж�ȡ������
//
bool CScannerGroupParam::LoadBinary(FILE* fp, int nFileVersion)
{
	clear();
	int nScannerCount = 1;         // ��2.00�����½�֧��һ������ɨ����

	// �ӵ�2.00�濪ʼ�����ݼ�֧�ֶ������ɨ����ͬʱ�ɼ�
	if (nFileVersion >= 200)
	{
		if (fread(&nScannerCount, sizeof(int), 1, fp) != 1)
			return false;
	}

	// ���ζ��뼤��������
	for (int i = 0; i < nScannerCount; i++)
	{
		CLaserScannerParam Param;
		if (!Param.LoadBinary(fp, nFileVersion))
			return false;

		push_back(Param);
	}

	return true;
}

// ��������ļ���д�����
bool CScannerGroupParam::SaveBinary(FILE* fp, int nFileVersion)
{
	int nScannerCount = size();

	if (nFileVersion >= 200)
	{
		if (fwrite(&nScannerCount, sizeof(int), 1, fp) != 1)
			return false;
	}

	// ����д�������ɨ�����Ĳ���
	for (int i = 0; i < nScannerCount; i++)
	{
		if (!at(i).SaveBinary(fp, nFileVersion))
			return false;
	}

	return true;
}

//
//   ���ø�ɨ�����Ƿ�Ӧ���ڽ�ģ��
//
void CScannerGroupParam::UseWhenCreateModel(int nIdx, bool bYesOrNo)
{
	at(nIdx).m_bUseCreateModel = bYesOrNo;
}

//
//   ���ø�ɨ�����Ƿ�Ӧ���ڶ�λ��
//
void CScannerGroupParam::UseWhenLocalize(int nIdx, bool bYesOrNo)
{
	at(nIdx).m_bUseLocalize = bYesOrNo;
}
