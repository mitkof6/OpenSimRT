/**
 * @file CommonExports.h
 *
 * \brief Definitions for dll exports on Windows.
 *
 * @author Dimitar Stanev <dimitar.stanev@epfl.ch>
 */
#ifdef WIN32
#   ifdef Common_EXPORTS
#       define Common_API __declspec(dllexport)
#   else
#       define Common_API  __declspec(dllimport)
#   endif
#else
#   define Common_API
#endif // WIN32
