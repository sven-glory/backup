//                           - TOOLS.CPP -
//
//   Implementation of some commonly used mathematical tools.
//
//   Author: Zhang Lei
//   Date:   2000. 10. 26
//

#include "stdafx.h"
#include <stdlib.h>
#include <math.h>
#include <ctype.h>
#include <assert.h>
#include "Tools.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif

#define INVALID_HEX_CHAR   16         //

static UCHAR m_pchOctTab[] = "01234567";
static UCHAR m_pchHexTab[] = "0123456789ABCDEF";

//////////////////////////////////////////////////////////////////////////////

//
//   EstimateArea: Estimate the covering area of a line in cartesian space,
//   in which the start value, end value and slope are known.
//
//   [Note]:
//     The values mentioned here can only be non-negative values, i.e.,
//          fStartValue >= 0
//          fEndValue >= 0
//          fSlope > 0
//
float EstimateArea(float fStartValue, float fEndValue, float fSlope)
{
	// If the start value is equal to the end value, return 0
	if (fStartValue == fEndValue)
		return 0.0f;

	// Estimate the covering area
	return (float)(fabs((Square(fEndValue) - Square(fStartValue)) / (2 * fSlope)));
}

//
//   EstimateEndValue: Estimate the end value of a line in cartesian
//   space, in which the start value, slope and covering area are known.
//
//   [Note]:
//     1. The values here can only be non-negative values, i.e.,
//          fStartValue >= 0
//          fEndValue >= 0
//
//     2. Depending on the condition of the 2 values, the slope can be
//        positive, negative or 0:
//          fSlope > 0 (if fStartValue < fEndValue)
//          fSlope < 0 (if fStartValue > fEndValue)
//          fSlope any value (if fStartValue == fEndValue);
//
float EstimateEndValue(float fStartValue, float fArea, float fSlope)
{
	assert (fStartValue >= 0);
	assert (fArea >= 0);

	// If the slope is 0, the end value is equal to the start value
	if (fSlope == 0)
		return fStartValue;

	// Try to caculate the square of "End Value"
	float fSquEndValue = Square(fStartValue) + 2 * fSlope * fArea;

	// This value must be non-negative
	assert (fSquEndValue >= 0);

	// Get the "End Value"
	return (float)sqrt(fSquEndValue);
}

//
//   ApprEqual: Check if the 2 floats are approximately equal.
//
BOOL ApprEqual(float x, float y, float fGate)
{
	return (fabs(x - y) < fGate);
}

//
//   SwabWord: Swab the high/low bytes of the given word.
//
void SwabWord(USHORT &uWord)
{
	union
	{
		USHORT uWord;
		UCHAR c[2];
	} DataBuf;

	DataBuf.uWord = uWord;
	_swab((char*)DataBuf.c, (char*)DataBuf.c, 2);
	uWord = DataBuf.uWord;
}

//
//   Reverse the order of bits in a word data.
//
void ReverseBitsOrder(USHORT& uWord)
{
	USHORT uTemp = 0;
	for (int i = 0; i < 16; i++)
	{
		if (uWord & BIT(i))
			uTemp |= 1;
		if (i != 15)
			uTemp <<= 1;
	}
	uWord = uTemp;
}

static USHORT BitMaskTab[16] =
{
	0x0001, 0x0002, 0x0004, 0x0008, 0x0010, 0x0020, 0x0040, 0x0080,
	0x0100, 0x0200, 0x0400, 0x0800, 0x1000, 0x2000, 0x4000, 0x8000
};

//
//   Search for "1" in the mask byte and return the index (low to high).
//
SHORT FindBit(USHORT Mask, SHORT nFromBit)
{
	for (SHORT i = nFromBit; i < 16; i++)
		if (Mask & BitMaskTab[i])
			return i;

	return -1;
}

//
//   Generate mask byte for the specified bit.
//
USHORT FindMaskWord(SHORT nBit)
{
	assert(nBit < 16);

	return BitMaskTab[nBit];
}

#if 0
// Get the nearest integer
int round(double d)
{
	double dFloor = floor(d);
	double dCeil = ceil(d);
	if (fabs(dFloor - d) < 0.5)
		return (int)dFloor;
	else
		return (int)dCeil;
}
#endif

//
//   Look up the specified character in the hex character table.
//
//   Note:
//     On success, return the index of the character;
//     On failure, return INVALID_HEX_CHAR (16)
//
USHORT LookUpHex(UCHAR ch)
{
	for (USHORT i = 0; i < 16; i++)
	{
		// If the character is found, return the index
		if (m_pchHexTab[i] == ch)
			return i;
	}

	// The char does not represent a hex char, return 16
	return INVALID_HEX_CHAR;
}

//
//   Convert an ASCII character to its hex string.
//   Example:  0x2E ==> "2E"
//
void CharToHexStr(UCHAR chAscii, UCHAR *pchHex)
{
	// The higher 4 bits
	*pchHex++ = m_pchHexTab[chAscii >> 4];

	// The lower 4 bits
	*pchHex = m_pchHexTab[chAscii & 0x0F];
}

//
//   Convert an hex string into its ASCII character.
//   Example: "2E" => 0x2E
//
//   Function return:
//     FALSE - Failure
//     TRUE  - Success
//
BOOL HexStrToChar(UCHAR *pchHex, UCHAR& chAscii)
{
	// Find the index of the 1st hex char
	USHORT uHigh = LookUpHex(toupper(*pchHex++));

	// If it's not a valid hex char, return FALSE
	if (uHigh == INVALID_HEX_CHAR)
		return FALSE;

	// Find the index of the 2nd hex char
	USHORT uLow = LookUpHex(toupper(*pchHex));

	// If it's not a valid hex char, return FALSE
	if (uLow == INVALID_HEX_CHAR)
		return FALSE;

	// Assemble the character
	chAscii = (UCHAR)((uHigh<<4) | uLow);

	// Success, return TRUE
	return TRUE;
}

//
//   Convert an ASCII character to its oct string.
//
void CharToOctStr(UCHAR chAscii, UCHAR *pchOct)
{
	_itoa(chAscii, (char*)pchOct, 8);
}

BOOL IsFloat(char* str, float* pFloat)
{
	char* p = str;
	while (*p == ' ') p++;     // Skip blanks
	if (*p == '-' || *p == '+') p++; // skip sign symbol
	int nDotCount = 0;
	
	while (*p)
	{
		if (*p == '.')
		{
			if (++nDotCount > 1)
				return FALSE;
		}
		else if (!isdigit(*p))
			return FALSE;
			
		p++;
	}
	
	if (pFloat != NULL)
		*pFloat = (float)atof(str);
	return TRUE;
}

#if _MSC_VER < 1000
	BOOL Approach(float& from, float to, float step)
	{
		if (from + step < to)
		{
			from += step;
			return FALSE;
		}

		else if (from - step > to)
		{
			from -= step;
			return FALSE;
		}

		else
		{
			from = to;
			return TRUE;
		}
	}

	BOOL Approach(int& from, int to, int step)
	{
		if (from + step < to)
		{
			from += step;
			return FALSE;
		}

		else if (from - step > to)
		{
			from -= step;
			return FALSE;
		}

		else
		{
			from = to;
			return TRUE;
		}
	}

#if (defined _MFC_VER) && (_MFC_VER < 0x0400)
		CArchive& operator >> (CArchive& ar, char& Obj)
		{
			BYTE uchTemp;
			ar >> uchTemp;
			Obj = (char)uchTemp;
			return ar;
		}        	

		CArchive& operator << (CArchive& ar, char& Obj)
		{
			ar << (BYTE)Obj;
			return ar;
		}        	

		CArchive& operator >> (CArchive& ar, SHORT& Obj)
		{
			USHORT uTemp;
			ar >> uTemp;
			Obj = (SHORT)uTemp;
			return ar;
		}        	

		CArchive& operator << (CArchive& ar, SHORT& Obj)
		{
			ar << (USHORT)Obj;
			return ar;
		}
#endif     // _MFC_VER < 0x0400

#else

SHORT HexStrToInt(CString& str)
{
	SHORT n;

	if (_stscanf((_TCHAR*)str.GetBuffer(20), _T("%X"), &n) == 1)
		return n;
	else
		return -1;
}

bool DecStrToInt(CString& str, int& i)
{
	if (_stscanf((_TCHAR*)str.GetBuffer(20), _T("%d"), &i) == 1)
		return true;
	else
		return false;
}

bool DecStrToFloat(CString& str, float& f)
{
	if (_stscanf((_TCHAR*)str.GetBuffer(20), _T("%f"), &f) == 1)
		return true;
	else
		return false;
}


CString FloatToStr(float f)
{
	CString str;
	str.Format(_T("%0-f"), f);
	return str;
}

#if defined _WIN32_WCE
time_t time(time_t* /*timer*/)
{
	CTime t = CTime::GetCurrentTime();
	return t.GetTime();
}

double difftime(time_t timer1, time_t timer0)
{
	CTime t1(timer1);
	CTime t0(timer0);
	CTimeSpan dt = t1 - t0;
	return (double)(dt.GetSeconds());
}

CArchive& operator >> (CArchive& ar, char* str)
{
	char chLen;
	ar >> chLen;

	for (int i = 0; i < chLen; i++)
		ar >> str[i];

	str[i] = 0;
	return ar;
}

CArchive& operator << (CArchive& ar, char* str)
{
	char chLen = (char) strlen(str);
	ar << chLen;

	for (int i = 0; i < chLen; i++)
		ar << str[i];

	return ar;
}
#endif

#endif   // _MSC_VER < 800
