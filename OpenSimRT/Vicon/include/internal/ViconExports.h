#ifdef WIN32
#   ifdef Vicon_EXPORTS
#       define Vicon_API __declspec(dllexport)
#   else
#       define Vicon_API  __declspec(dllimport)
#   endif
#else
#   define Vicon_API
#endif // WIN32
