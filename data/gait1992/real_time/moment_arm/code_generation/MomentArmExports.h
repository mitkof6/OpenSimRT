/**
 * @file MomentArmExports.h
 *
 * \brief Definitions for dll exports on Windows.
 *
 * @author Dimitar Stanev <jimstanev@gmail.com>
 */
#ifdef WIN32
#   ifdef MomentArm_EXPORTS
#       define MomentArm_API __declspec(dllexport)
#   else
#       define MomentArm_API __declspec(dllimport)
#   endif
#else
#   define MomentArm_API
#endif // WIN32
