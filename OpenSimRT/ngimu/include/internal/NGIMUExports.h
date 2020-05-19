#ifdef WIN32
#   ifdef NGIMU_EXPORTS
#       define NGIMU_API __declspec(dllexport)
#   else
#       define NGIMU_API  __declspec(dllimport)
#   endif
#else
#   define NGIMU_API
#endif // WIN32
