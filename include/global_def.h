#ifndef _GLOBAL_DEF_H_
#define _GLOBAL_DEF_H_

#include <cstddef>
#include <cstdint>

#ifdef WIN32
#define __func__ __FUNCTION__
#endif

#ifndef TRUE
#define TRUE true
#endif // #ifndef TRUE

#ifndef FALSE
#define FALSE false
#endif // #ifndef FALSE

#endif // #ifndef _GLOBAL_DEF_H_
