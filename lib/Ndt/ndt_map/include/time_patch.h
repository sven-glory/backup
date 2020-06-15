#pragma once

#ifdef _WINDOWS
#include <windows.h>
#include <time.h>
#else
#include <sys/time.h>
#endif


double getDoubleTime();
