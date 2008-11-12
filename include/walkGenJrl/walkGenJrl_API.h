#ifndef _WALKGENJRL_API_H_
#define _WALKGENJRL_API_H_

#if defined (WIN32)
#  ifdef walkGenJrl_EXPORTS 
#    define WALK_GEN_JRL_EXPORT __declspec(dllexport)
#  else  
#    define WALK_GEN_JRL_EXPORT __declspec(dllimport)
#  endif 
#else
#  define WALK_GEN_JRL_EXPORT
#endif

#endif /*_WALKGENJRL_API_H_*/