#ifdef WIN32
#   ifdef IMU_EXPORTS
#       define IMU_API __declspec(dllexport)
#   else
#       define IMU_API  __declspec(dllimport)
#   endif
#else
#   define IMU_API
#endif // WIN32
