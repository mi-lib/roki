/* roki_export.h */
/* This file was automatically generated. */
/* 2023年  5月 22日 月曜日 08:05:29 JST by zhidao */
#ifndef __ROKI_EXPORT_H__
#define __ROKI_EXPORT_H__
#include <zeda/zeda_compat.h>
#if defined(__WINDOWS__) && !defined(__CYGWIN__)
# if defined(__ROKI_BUILD_DLL__)
#  define __ROKI_EXPORT extern __declspec(dllexport)
#  define __ROKI_CLASS_EXPORT  __declspec(dllexport)
# else
#  define __ROKI_EXPORT extern __declspec(dllimport)
#  define __ROKI_CLASS_EXPORT  __declspec(dllimport)
# endif
#else
# define __ROKI_EXPORT __EXPORT
# define __ROKI_CLASS_EXPORT
#endif
#endif /* __ROKI_EXPORT_H__ */
