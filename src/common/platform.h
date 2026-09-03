#pragma once

#include <filesystem>
#include <string>

// clang-format off
#if defined(_WIN32)
#define OS_WINDOWS 1
#define OS_LINUX 0
#elif (defined(__gnu_linux__) || defined(__linux__))
#define OS_WINDOWS 0
#define OS_LINUX 1
#else
#error "Unsupported OS"
#endif

#if defined(__clang__)
#define COMPILER_MSVC 0
#define COMPILER_GCC 0
#define COMPILER_CLANG 1
#elif defined(_MSC_VER)
#define COMPILER_MSVC 1
#define COMPILER_GCC 0
#define COMPILER_CLANG 0
#elif defined(__GNUC__)
#define COMPILER_MSVC 0
#define COMPILER_GCC 1
#define COMPILER_CLANG 0
#else
#error "Unknown compiler"
#endif

#if defined(_M_X64) || defined(__x86_64__)
#define ARCH_X64 1
#define ARCH_AARCH64 0
#elif defined(_M_ARM64) || defined(__aarch64__)
#define ARCH_X64 0
#define ARCH_AARCH64 1
#else
#error "Unsupported architecture"
#endif

// warning guards for various compilers
#if COMPILER_CLANG
#define BEGIN_EXTERNAL_HEADERS \
    __pragma(clang diagnostic push) \
    __pragma(clang diagnostic ignored "-Weverything") \
    __pragma(warning(push, 0))
#define END_EXTERNAL_HEADERS \
    __pragma(warning(pop)) \
    __pragma(clang diagnostic pop)
#elif COMPILER_MSVC
#define BEGIN_EXTERNAL_HEADERS \
    __pragma(warning(push, 0)) \
    __pragma(warning(disable : 4668)) // #if FOO warns if FOO is not defined; some libs are written like that :(
#define END_EXTERNAL_HEADERS \
    __pragma(warning(pop))
#elif COMPILER_GCC
#define BEGIN_EXTERNAL_HEADERS \
    _Pragma("GCC diagnostic push") \
    _Pragma("GCC diagnostic ignored \"-Wall\"") \
    _Pragma("GCC diagnostic ignored \"-Wextra\"")
#define END_EXTERNAL_HEADERS \
    _Pragma("GCC diagnostic pop")
#else
#define BEGIN_EXTERNAL_HEADERS
#define END_EXTERNAL_HEADERS
#endif
// clang-format on

namespace platform {
// %APPDATA% or ~/.config
std::filesystem::path getUserConfigDir();
std::filesystem::path getExeDir();
std::filesystem::path getSteamvrVrPathsPath();

std::string getEnvVariable(const std::string& szEnvVarName);

bool isAnotherInstanceRunning(bool& bIsRunningViaSteam);
void shutdownCurrentInstance();

// title -> window title in the window decoration
// message -> text in the dialog box
void showMessageDialog(const std::string& title, const std::string& message);

// opens the directory in the OS' file brower
void launchDirInFileBrowser(const std::filesystem::path& szDirectory);
// launches a url in the default web browser
void launchWebpage(const std::string& szUrl);

void setThreadName(const std::string& threadName);

// utf8 stuff for imgui -> overlay -> os interop

}