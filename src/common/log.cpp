#include "quill/Backend.h"
#include "quill/Frontend.h"
#include "quill/LogMacros.h"
#include "quill/Logger.h"
#include "quill/sinks/ConsoleSink.h"
#include "quill/sinks/FileSink.h"

#include "log.h"
#include "util.h"
#include <fmt/chrono.h>
#include <openvr_driver.h>

#include <cassert>
#include <filesystem>

namespace fs = ::std::filesystem;

namespace logging {
::quill::Logger* s_logger = nullptr;
::quill::Logger* s_loggerOpenVr = nullptr;
::quill::Logger* s_loggerIpc = nullptr;
::quill::Logger* s_loggerHooking = nullptr;
::quill::Logger* s_loggerCalibration = nullptr;
::quill::Logger* s_loggerBluetooth = nullptr;

class UiHaltOnErrorSink final : public quill::Sink {
public:
    UiHaltOnErrorSink() = default;

    void write_log(quill::MacroMetadata const* /** log_metadata **/, uint64_t /** log_timestamp **/,
        std::string_view /** thread_id **/, std::string_view /** thread_name **/,
        std::string const& /** process_id **/, std::string_view /** logger_name **/,
        quill::LogLevel log_level, std::string_view /** log_level_description **/,
        std::string_view /** log_level_short_code **/,
        std::vector<std::pair<std::string, std::string>> const* /** named_args - only populated when named args in the format placeholder are used **/,
        std::string_view /** log_message **/, std::string_view log_statement) override
    {
        // Propagate error logs to the UI
        if (log_level < quill::LogLevel::Backtrace && log_level >= quill::LogLevel::Error) {
            // @TODO: propagate to ui
        }
    }

    void flush_sink() noexcept override { }
    void run_periodic_tasks() noexcept override { }
};

class DriverLogSink final : public quill::Sink {
public:
    DriverLogSink() = default;

    void write_log(quill::MacroMetadata const* /** log_metadata **/, uint64_t /** log_timestamp **/,
        std::string_view /** thread_id **/, std::string_view /** thread_name **/,
        std::string const& /** process_id **/, std::string_view logger_name,
        quill::LogLevel log_level, std::string_view /** log_level_description **/,
        std::string_view /** log_level_short_code **/,
        std::vector<std::pair<std::string, std::string>> const* /** named_args - only populated when named args in the format placeholder are used **/,
        std::string_view log_message, std::string_view log_statement) override
    {
        // Propagate error logs to the steamVR
        std::string szLogLevel;
        switch (log_level) {
        case quill::LogLevel::TraceL3:
        case quill::LogLevel::TraceL2:
        case quill::LogLevel::TraceL1:
            szLogLevel = "Trace";
            break;
        case quill::LogLevel::Debug:
            szLogLevel = "Debug";
            break;
        case quill::LogLevel::Info:
            szLogLevel = "Info";
            break;
        case quill::LogLevel::Notice:
            szLogLevel = "Notice";
            break;
        case quill::LogLevel::Warning:
            szLogLevel = "Warning";
            break;
        case quill::LogLevel::Error:
            szLogLevel = "Error";
            break;
        case quill::LogLevel::Critical:
            szLogLevel = "Critical";
            break;
        case quill::LogLevel::Backtrace:
            szLogLevel = "Backtrace";
            break;
        case quill::LogLevel::None:
            szLogLevel = "None";
            break;
        default:
            szLogLevel = fmt::format("LogLevel_{}", (uint32_t)log_level);
            break;
        }

        std::string vrLogMsg = fmt::format("[{} : {}] {}", logger_name, szLogLevel, log_message);
        vr::VRDriverLog()->Log(vrLogMsg.c_str());
    }

    void flush_sink() noexcept override { }
    void run_periodic_tasks() noexcept override { }
};

bool IsFileOlderThan30Days(int year, int month, int day, int hour, int minute, int second)
{
    std::tm time = std::tm { second, minute, hour, day, month - 1, year - 1900 };
    auto fileTime = std::chrono::system_clock::from_time_t(std::mktime(&time));
    auto currentTime = std::chrono::system_clock::now();
    auto fileAge = currentTime - fileTime;
    return fileAge > std::chrono::days(CLEAR_LOG_FILES_AFTER_DAYS);
}

bool ParseLogFilename(std::string_view filename, int& year, int& month, int& day, int& hour, int& minute, int& second)
{
    // needs to match _YYYY_MM_DD_HH_MM_SS.log suffix + log_ prefix
    if (filename.size() < 30 || !filename.starts_with("log_") || !filename.ends_with(".log")) {
        return false;
    }

    const char* data = filename.data();
    const size_t n = filename.size();

    // scan for _YYYY_MM_DD_HH_MM_SS from back
    if (std::from_chars(data + (n - 23), data + (n - 19), year).ec != std::errc {})
        return false;
    if (std::from_chars(data + (n - 18), data + (n - 16), month).ec != std::errc {})
        return false;
    if (std::from_chars(data + (n - 15), data + (n - 13), day).ec != std::errc {})
        return false;
    if (std::from_chars(data + (n - 12), data + (n - 10), hour).ec != std::errc {})
        return false;
    if (std::from_chars(data + (n - 9), data + (n - 7), minute).ec != std::errc {})
        return false;
    if (std::from_chars(data + (n - 6), data + (n - 4), second).ec != std::errc {})
        return false;

    return true;
}

void DeleteOldLogFiles(const std::filesystem::path& directoryPath, bool isOverlay)
{
    std::string szLatestFilename = isOverlay ? "log_overlay_latest.log" : "log_driver_latest.log";
    std::string szLogStart = isOverlay ? "log_overlay_" : "log_driver_";
    for (const auto& entry : std::filesystem::directory_iterator(directoryPath)) {
        if (entry.is_regular_file() && entry.path().filename() != szLatestFilename) {
            std::string fileName = entry.path().filename().string();
            int year, month, day, hour, minute, second;

            if (fileName.substr(0, szLogStart.size()) == szLogStart && ParseLogFilename(fileName, year, month, day, hour, minute, second) && IsFileOlderThan30Days(year, month, day, hour, minute, second)) {
                std::filesystem::remove(entry.path());
            }
        }
    }
}

// Helper for declaring a logger
// @NOTE: CONSOLE_SINK crashes vr_server.exe ; so omit it from the driver binary
#ifdef _DEBUG
#define CONSOLE_SINK quill::Frontend::create_or_get_sink<quill::ConsoleSink>("console"),
#else
#define CONSOLE_SINK
#endif

#define MAKE_LOGGER_OVERLAY(title)                                                 \
    quill::Frontend::create_or_get_logger(title,                                   \
        { CONSOLE_SINK                                                             \
                quill::Frontend::create_or_get_sink<quill::FileSink>(logFilePath), \
            quill::Frontend::create_or_get_sink<quill::FileSink>(logFilePath2),    \
            quill::Frontend::create_or_get_sink<UiHaltOnErrorSink>("halt_on_error") });
#define MAKE_LOGGER_DRIVER(title)                                               \
    quill::Frontend::create_or_get_logger(title,                                \
        { quill::Frontend::create_or_get_sink<quill::FileSink>(logFilePath),    \
            quill::Frontend::create_or_get_sink<quill::FileSink>(logFilePath2), \
            quill::Frontend::create_or_get_sink<DriverLogSink>("driverlog") });

void Init(bool isOverlay)
{
    assert(s_logger == nullptr);
    assert(s_loggerOpenVr == nullptr);

    quill::Backend::start();

    auto now = std::chrono::system_clock::now();
    auto time = std::chrono::system_clock::to_time_t(now);
    auto tm = *std::localtime(&time);

    const char* logType = isOverlay ? "overlay" : "driver";

    std::string logFilePath = (util::getSpaceCalibratorLogsDir() / fmt::format("log_{}_{:%Y_%m_%d_%H_%M_%S}.log", logType, tm)).string();
    std::string logFilePath2 = (util::getSpaceCalibratorLogsDir() / fmt::format("log_{}_latest.log", logType)).string();
    DeleteOldLogFiles(util::getSpaceCalibratorLogsDir(), isOverlay);

    if (isOverlay) {
        s_logger = MAKE_LOGGER_OVERLAY("overlay");
        s_loggerOpenVr = MAKE_LOGGER_OVERLAY("openvr");
        s_loggerIpc = MAKE_LOGGER_OVERLAY("ipc");
        s_loggerHooking = MAKE_LOGGER_OVERLAY("hooking");
        s_loggerCalibration = MAKE_LOGGER_OVERLAY("calibration");
        s_loggerBluetooth = MAKE_LOGGER_OVERLAY("bluetooth");
    } else {
        s_logger = MAKE_LOGGER_DRIVER("driver");
        s_loggerOpenVr = MAKE_LOGGER_DRIVER("openvr");
        s_loggerIpc = MAKE_LOGGER_DRIVER("ipc");
        s_loggerHooking = MAKE_LOGGER_DRIVER("hooking");
        s_loggerCalibration = MAKE_LOGGER_DRIVER("calibration");
        s_loggerBluetooth = MAKE_LOGGER_DRIVER("bluetooth");
    }
}
}
