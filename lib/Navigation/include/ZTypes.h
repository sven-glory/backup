//                               - ZTYPES.H -
//
//   Definition of macros that will be commonly used in programming.
//
//   Author: Zhang Lei
//   Date:   2000. 10. 26
//

#if !defined __ZTYPES
#define __ZTYPES

//////////////////////////////////////////////////////////////////////////////
//   Macro definition

#define CP_CHINESE_PRC        936    // Code page of Chinese (P.R.C.)

// Defines data types
#ifndef _MFC_VER               // 如果没有MFC支持
	typedef unsigned char       UCHAR;
	typedef unsigned short int  BOOL;
	typedef unsigned int        USHORT;
	typedef int                 SHORT;
	typedef unsigned long       ULONG;
	typedef long                LONG;
	typedef unsigned int        POSITION;
#endif    // _MSC_VER

#ifndef _MSC_VER
	#ifndef FALSE
	#define FALSE               ((BOOL)0)
	#endif   // FALSE

	#ifndef TRUE
	#define TRUE                ((BOOL)1)
	#endif   // TRUE
#endif      // _MSC_VER

// Template support
#if defined _MSC_VER

	#ifdef _MFC_VER
		#define _ARCHIVE_SUPPORT
	#endif
	
	#if _MSC_VER >= 1000
		#define _TEMPLATE_SUPPORT
	#endif   // _MSC_VER >= 1000
	
#else     // _MSC_VER undefined

	#ifdef __BCPLUSPLUS__
		#define _TEMPLATE_SUPPORT
	#endif
#endif

#define DllExport         __declspec(dllexport)
#define DllImport         __declspec(dllimport)

// WINCE dependant
#if defined _WIN32_WCE

extern "C" {
int __cdecl _inp(unsigned short);
int __cdecl _outp(unsigned short, int);
}

#define inp               _inp
#define outp              _outp

#pragma intrinsic(_inp)
#pragma intrinsic(_outp)

#define swab              _swab
#define itoa              _itoa

#elif defined _WIN32
#include <conio.h>
#define inp(u)           0
#define outp(u, x)       0
#endif

#define SHOW_DEBUG_MSG

#endif    // __ZTYPES
