/*! \file TimeUtilsWindows.h

   Copyright (c) 2009, 
   @author Francois Keith.
   
   JRL-Japan, CNRS/AIST

   All rights reserved.

   Please see License.txt for further information on license.   

*/



#ifdef WIN32

#ifndef TIME_UTILS_WINDOWS_H
#define TIME_UTILS_WINDOWS_H

#include < time.h >

struct timezone 
{
  int  tz_minuteswest; /* minutes W of Greenwich */
  int  tz_dsttime;     /* type of dst correction */
};

int gettimeofday(struct timeval *tv, struct timezone *tz);

#endif /*TIME_UTILS_WINDOWS_H*/

#endif /*WIN32*/

