#include "platform.h"

#if OS_LINUX
#include "constants.h"
#include "log.h"

#include <errno.h>
#include <fcntl.h>
#include <pthread.h>
#include <pwd.h>
#include <stdlib.h>
#include <string>
#include <sys/file.h>
#include <sys/stat.h>
#include <sys/wait.h>
#include <unistd.h>
#include <vector>

namespace platform {
constexpr int INVALID_FD = -1;

std::filesystem::path getUserConfigDir()
{
    // based on XDG base dir spec for XDG_DATA_HOME
    // https://specifications.freedesktop.org/basedir-spec/basedir-spec-latest.html

    const char* szXdgConfigHome = getenv("XDG_DATA_HOME");

    if (szXdgConfigHome != nullptr && szXdgConfigHome[0] != '\0') {
        return std::filesystem::path(szXdgConfigHome);
    }

    szXdgConfigHome = getenv("HOME");
    if (szXdgConfigHome != nullptr && szXdgConfigHome[0] != '\0') {
        return std::filesystem::path(szXdgConfigHome) / ".local" / "share";
    }

    struct passwd* user_db_entry = getpwuid(getuid());
    if (user_db_entry) {
        szXdgConfigHome = user_db_entry->pw_dir;
    }
    if (szXdgConfigHome != nullptr && szXdgConfigHome[0] != '\0') {
        return std::filesystem::path(szXdgConfigHome) / ".local" / "share";
    }

    return std::filesystem::path();
}

std::filesystem::path getRuntimeDir()
{
    // based on XDG base dir spec for XDG_RUNTIME_DIR
    // https://specifications.freedesktop.org/basedir-spec/basedir-spec-latest.html

    const char* szXdgRuntimeDir = getenv("XDG_RUNTIME_DIR");
    if (szXdgRuntimeDir != nullptr && szXdgRuntimeDir[0] != '\0') {
        return std::filesystem::path(szXdgRuntimeDir);
    }

    LOG_WARN("XDG_RUNTIME_DIR is not set, falling back to /tmp/spacecalibrator-runtime-{}", getuid());

    std::filesystem::path fallback = std::filesystem::path("/tmp") / ("spacecalibrator-runtime-" + std::to_string(getuid()));

    std::error_code ec;
    std::filesystem::create_directories(fallback, ec);
    if (ec) {
        LOG_ERROR("Failed to create runtime dir fallback {}: {}", fallback.string(), ec.message());
        return std::filesystem::path();
    }

    errno = 0;
    if (chmod(fallback.c_str(), 0700) != 0) {
        LOG_ERROR("Failed to set permissions on runtime dir fallback {}: {}", fallback.string(), strerror(errno));
    }

    return fallback;
}

std::filesystem::path getExeDir()
{
    std::error_code ec;
    std::filesystem::path exePath = std::filesystem::canonical("/proc/self/exe", ec);
    if (ec) {
        return std::filesystem::path();
    }
    return exePath.parent_path();
}

std::filesystem::path getSteamvrVrPathsPath()
{
    // ~/.config/openvr/openvrpaths.vrpath

    // based on XDG base dir spec for XDG_CONFIG_HOME
    // https://specifications.freedesktop.org/basedir-spec/basedir-spec-latest.html

    const char* szXdgConfigHome = getenv("XDG_CONFIG_HOME");

    if (szXdgConfigHome != nullptr && szXdgConfigHome[0] != '\0') {
        return std::filesystem::path(szXdgConfigHome) / "openvr" / "openvrpaths.vrpath";
    }

    szXdgConfigHome = getenv("HOME");
    if (szXdgConfigHome != nullptr && szXdgConfigHome[0] != '\0') {
        return std::filesystem::path(szXdgConfigHome) / ".config" / "openvr" / "openvrpaths.vrpath";
    }

    struct passwd* user_db_entry = getpwuid(getuid());
    if (user_db_entry) {
        szXdgConfigHome = user_db_entry->pw_dir;
    }
    if (szXdgConfigHome != nullptr && szXdgConfigHome[0] != '\0') {
        return std::filesystem::path(szXdgConfigHome) / ".config" / "openvr" / "openvrpaths.vrpath";
    }

    return std::filesystem::path();
}

std::string getEnvVariable(const std::string& szEnvVarName)
{
    const char* szEnvVar = std::getenv(szEnvVarName.c_str());
    return szEnvVar ? std::string(szEnvVar) : "";
}

constexpr const char* s_STEAM_MUTEX_KEY = "/MUTEX__SpaceCalibrator_Steam.lock";
int s_hSteamMutex = INVALID_FD; // flock on linux
bool s_isGitHubVersionInstalled = false;

bool isAnotherInstanceRunning(bool& bIsRunningViaSteam)
{
    bIsRunningViaSteam = false;
    std::string szSteamAppIdEnv = getEnvVariable("SteamAppId");
    if (spacecal::c_SPACE_CALIBRATOR_STEAM_APP_ID == szSteamAppIdEnv || spacecal::c_STEAMVR_STEAM_APP_ID == szSteamAppIdEnv) {
        // We got launched via the Steam client UI.
        bIsRunningViaSteam = true;

        std::filesystem::path szRuntimeDir = getRuntimeDir();
        std::string szLockPath = (szRuntimeDir / "spacecalibrator_steam.lock").string();

        s_hSteamMutex = open(szLockPath.c_str(), O_CREAT | O_RDWR, 0666);
        if (s_hSteamMutex == INVALID_FD) {
            return false;
        }

        errno = 0;
        if (flock(s_hSteamMutex, LOCK_EX | LOCK_NB) == INVALID_FD) {
            if (errno == EWOULDBLOCK) {
                close(s_hSteamMutex);
                s_hSteamMutex = INVALID_FD;
                return true;
            }
        }
        return false;
    }

    return false;
}
void shutdownCurrentInstance()
{
    if (s_hSteamMutex != INVALID_FD) {
        flock(s_hSteamMutex, LOCK_UN);
        close(s_hSteamMutex);
        s_hSteamMutex = INVALID_FD;
    }
}

bool spawnProcess(const std::string& executable, const std::vector<std::string>& arguments, bool bWaitForExit = false, bool bSuppressStderr = true)
{
    pid_t pid = fork();

    if (pid == 0) {
        // child proc
        if (bSuppressStderr) {
            int nullFd = open("/dev/null", O_WRONLY);
            if (nullFd != -1) {
                dup2(nullFd, STDERR_FILENO);
                close(nullFd);
            }
        }

        std::vector<char*> argv;
        argv.push_back(const_cast<char*>(executable.c_str()));
        for (const auto& arg : arguments) {
            argv.push_back(const_cast<char*>(arg.c_str()));
        }
        argv.push_back(nullptr);

        execvp(argv[0], argv.data());
        _exit(1);
    } else if (pid > 0) {
        // parent proc
        if (bWaitForExit) {
            int status = 0;
            waitpid(pid, &status, 0);
            return WIFEXITED(status) && WEXITSTATUS(status) == 0;
        }

        return true;
    }

    return false;
}

void showMessageDialog(const std::string& title, const std::string& message)
{
    LOG_INFO("displaying dialog [{}]: {}", title, message);

    if (spawnProcess("kdialog", { "--title", title, "--error", message }, true, true)) {
        return;
    }
    if (spawnProcess("zenity", { "--error", "--title", title, "--text", message, "--width=300" }, true, true)) {
        return;
    }
    if (spawnProcess("notify-send", { title, message }, true, true)) {
        return;
    }

    LOG_CRITICAL("Failed to display message dialog. Message was: {}", message);
}

void launchDirInFileBrowser(const std::filesystem::path& szDirectory)
{
    spawnProcess("xdg-open", { szDirectory.string() }, false, true);
}

void launchWebpage(const std::string& szUrl)
{
    spawnProcess("xdg-open", { szUrl }, false, true);
}

void setThreadName(const std::string& threadName)
{
    // thread name has max 16 chars
    // https://man7.org/linux/man-pages/man3/pthread_setname_np.3.html
    std::string truncatedName = threadName.substr(0, 15);
    pthread_setname_np(pthread_self(), truncatedName.c_str());
}
}
#endif