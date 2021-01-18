/**
 * @file RealTimeExports.h
 *
 * \brief Definitions for dll exports on Windows.
 *
 * @author Dimitar Stanev <jimstanev@gmail.com>
 */
#ifdef WIN32
#   ifdef RealTime_EXPORTS
#       define RealTime_API __declspec(dllexport)
#   else
#       define RealTime_API __declspec(dllimport)
#   endif
#else
#   define RealTime_API
#endif // WIN32
