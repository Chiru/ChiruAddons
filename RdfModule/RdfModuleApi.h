// For conditions of distribution and use, see copyright notice in LICENSE

#pragma once

#if defined (_WINDOWS)
#if defined(RDF_MODULE_EXPORTS)
#define RDF_MODULE_API __declspec(dllexport)
#else
#define RDF_MODULE_API __declspec(dllimport)
#endif
#else
#define RDF_MODULE_API
#endif

