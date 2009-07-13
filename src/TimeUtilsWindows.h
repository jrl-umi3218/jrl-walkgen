/*! \file TimeUtilsWindows.h
*/

#include <walkGenJrl/walkGenJrl_API.h>

#ifdef WIN32

#ifndef TIME_UTILS_WINDOWS_H
#define TIME_UTILS_WINDOWS_H

#include < time.h >

struct timezone 
{
  int  tz_minuteswest; /* minutes W of Greenwich */
  int  tz_dsttime;     /* type of dst correction */
};

int WALK_GEN_JRL_EXPORT gettimeofday(struct timeval *tv, struct timezone *tz);

#endif /*TIME_UTILS_WINDOWS_H*/

#endif /*WIN32*/

