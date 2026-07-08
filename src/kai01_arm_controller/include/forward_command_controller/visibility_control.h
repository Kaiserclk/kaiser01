#ifndef FORWARD_COMMAND_CONTROLLER__VISIBILITY_CONTROL_H_
#define FORWARD_COMMAND_CONTROLLER__VISIBILITY_CONTROL_H_

// This logic was borrowed (then modified) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
#define FORWARD_COMMAND_CONTROLLER_EXPORT __attribute__((dllexport))
#define FORWARD_COMMAND_CONTROLLER_IMPORT __attribute__((dllimport))
#else
#define FORWARD_COMMAND_CONTROLLER_EXPORT __declspec(dllexport)
#define FORWARD_COMMAND_CONTROLLER_IMPORT __declspec(dllimport)
#endif
#ifdef FORWARD_COMMAND_CONTROLLER_BUILDING_DLL
#define FORWARD_COMMAND_CONTROLLER_PUBLIC FORWARD_COMMAND_CONTROLLER_EXPORT
#else
#define FORWARD_COMMAND_CONTROLLER_PUBLIC FORWARD_COMMAND_CONTROLLER_IMPORT
#endif
#define FORWARD_COMMAND_CONTROLLER_LOCAL
#else
#define FORWARD_COMMAND_CONTROLLER_EXPORT __attribute__((visibility("default")))
#define FORWARD_COMMAND_CONTROLLER_IMPORT
#if __GNUC__ >= 4
#define FORWARD_COMMAND_CONTROLLER_PUBLIC __attribute__((visibility("default")))
#define FORWARD_COMMAND_CONTROLLER_LOCAL __attribute__((visibility("hidden")))
#else
#define FORWARD_COMMAND_CONTROLLER_PUBLIC
#define FORWARD_COMMAND_CONTROLLER_LOCAL
#endif
#endif

#endif  // FORWARD_COMMAND_CONTROLLER__VISIBILITY_CONTROL_H_
