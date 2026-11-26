#pragma once

#include <quill/LogMacros.h>
#include <quill/Logger.h>
#include <quill/bundled/fmt/format.h>

#if defined(IS_DRIVER) || defined(IS_OVERLAY)
#include "log_formatters.h"
#endif
#include <string>

namespace logging {
extern ::quill::Logger* s_logger;
extern ::quill::Logger* s_loggerOpenVr;
extern ::quill::Logger* s_loggerIpc;
extern ::quill::Logger* s_loggerHooking;
extern ::quill::Logger* s_loggerCalibration;
extern ::quill::Logger* s_loggerBluetooth;
void Init(bool isOverlay);
#ifndef _DEBUG
constexpr uint32_t CLEAR_LOG_FILES_AFTER_DAYS = 30;
#else
constexpr uint32_t CLEAR_LOG_FILES_AFTER_DAYS = 7;
#endif
}

// bunch of macros for each logger

#define LOG_DEBUG(fmt, ...) QUILL_LOG_DEBUG(::logging::s_logger, fmt, ##__VA_ARGS__)
#define LOG_INFO(fmt, ...) QUILL_LOG_INFO(::logging::s_logger, fmt, ##__VA_ARGS__)
#define LOG_NOTICE(fmt, ...) QUILL_LOG_NOTICE(::logging::s_logger, fmt, ##__VA_ARGS__)
#define LOG_WARNING(fmt, ...) QUILL_LOG_WARNING(::logging::s_logger, fmt, ##__VA_ARGS__)
#define LOG_WARN(fmt, ...) QUILL_LOG_WARNING(::logging::s_logger, fmt, ##__VA_ARGS__)
#define LOG_ERROR(fmt, ...) QUILL_LOG_ERROR(::logging::s_logger, fmt, ##__VA_ARGS__)
#define LOG_CRITICAL(fmt, ...) QUILL_LOG_CRITICAL(::logging::s_logger, fmt, ##__VA_ARGS__)
#define LOG_FATAL(fmt, ...) QUILL_LOG_CRITICAL(::logging::s_logger, fmt, ##__VA_ARGS__)
#define LOG_DYNAMIC(log_level, fmt, ...) QUILL_LOG_DYNAMIC(::logging::s_logger, log_level, fmt, ##__VA_ARGS__)

#define LOG_OPENVR_DEBUG(fmt, ...) QUILL_LOG_DEBUG(::logging::s_loggerOpenVr, fmt, ##__VA_ARGS__)
#define LOG_OPENVR_INFO(fmt, ...) QUILL_LOG_INFO(::logging::s_loggerOpenVr, fmt, ##__VA_ARGS__)
#define LOG_OPENVR_NOTICE(fmt, ...) QUILL_LOG_NOTICE(::logging::s_loggerOpenVr, fmt, ##__VA_ARGS__)
#define LOG_OPENVR_WARNING(fmt, ...) QUILL_LOG_WARNING(::logging::s_loggerOpenVr, fmt, ##__VA_ARGS__)
#define LOG_OPENVR_WARN(fmt, ...) QUILL_LOG_WARNING(::logging::s_loggerOpenVr, fmt, ##__VA_ARGS__)
#define LOG_OPENVR_ERROR(fmt, ...) QUILL_LOG_ERROR(::logging::s_loggerOpenVr, fmt, ##__VA_ARGS__)
#define LOG_OPENVR_CRITICAL(fmt, ...) QUILL_LOG_CRITICAL(::logging::s_loggerOpenVr, fmt, ##__VA_ARGS__)
#define LOG_OPENVR_FATAL(fmt, ...) QUILL_LOG_CRITICAL(::logging::s_loggerOpenVr, fmt, ##__VA_ARGS__)
#define LOG_OPENVR_DYNAMIC(log_level, fmt, ...) QUILL_LOG_DYNAMIC(::logging::s_loggerOpenVr, log_level, fmt, ##__VA_ARGS__)

#define LOG_IPC_DEBUG(fmt, ...) QUILL_LOG_DEBUG(::logging::s_loggerIpc, fmt, ##__VA_ARGS__)
#define LOG_IPC_INFO(fmt, ...) QUILL_LOG_INFO(::logging::s_loggerIpc, fmt, ##__VA_ARGS__)
#define LOG_IPC_NOTICE(fmt, ...) QUILL_LOG_NOTICE(::logging::s_loggerIpc, fmt, ##__VA_ARGS__)
#define LOG_IPC_WARNING(fmt, ...) QUILL_LOG_WARNING(::logging::s_loggerIpc, fmt, ##__VA_ARGS__)
#define LOG_IPC_WARN(fmt, ...) QUILL_LOG_WARNING(::logging::s_loggerIpc, fmt, ##__VA_ARGS__)
#define LOG_IPC_ERROR(fmt, ...) QUILL_LOG_ERROR(::logging::s_loggerIpc, fmt, ##__VA_ARGS__)
#define LOG_IPC_CRITICAL(fmt, ...) QUILL_LOG_CRITICAL(::logging::s_loggerIpc, fmt, ##__VA_ARGS__)
#define LOG_IPC_FATAL(fmt, ...) QUILL_LOG_CRITICAL(::logging::s_loggerIpc, fmt, ##__VA_ARGS__)
#define LOG_IPC_DYNAMIC(log_level, fmt, ...) QUILL_LOG_DYNAMIC(::logging::s_loggerIpc, log_level, fmt, ##__VA_ARGS__)

#define LOG_HOOKING_DEBUG(fmt, ...) QUILL_LOG_DEBUG(::logging::s_loggerHooking, fmt, ##__VA_ARGS__)
#define LOG_HOOKING_INFO(fmt, ...) QUILL_LOG_INFO(::logging::s_loggerHooking, fmt, ##__VA_ARGS__)
#define LOG_HOOKING_NOTICE(fmt, ...) QUILL_LOG_NOTICE(::logging::s_loggerHooking, fmt, ##__VA_ARGS__)
#define LOG_HOOKING_WARNING(fmt, ...) QUILL_LOG_WARNING(::logging::s_loggerHooking, fmt, ##__VA_ARGS__)
#define LOG_HOOKING_WARN(fmt, ...) QUILL_LOG_WARNING(::logging::s_loggerHooking, fmt, ##__VA_ARGS__)
#define LOG_HOOKING_ERROR(fmt, ...) QUILL_LOG_ERROR(::logging::s_loggerHooking, fmt, ##__VA_ARGS__)
#define LOG_HOOKING_CRITICAL(fmt, ...) QUILL_LOG_CRITICAL(::logging::s_loggerHooking, fmt, ##__VA_ARGS__)
#define LOG_HOOKING_FATAL(fmt, ...) QUILL_LOG_CRITICAL(::logging::s_loggerHooking, fmt, ##__VA_ARGS__)
#define LOG_HOOKING_DYNAMIC(log_level, fmt, ...) QUILL_LOG_DYNAMIC(::logging::s_loggerHooking, log_level, fmt, ##__VA_ARGS__)

#define LOG_CALIB_DEBUG(fmt, ...) QUILL_LOG_DEBUG(::logging::s_loggerCalibration, fmt, ##__VA_ARGS__)
#define LOG_CALIB_INFO(fmt, ...) QUILL_LOG_INFO(::logging::s_loggerCalibration, fmt, ##__VA_ARGS__)
#define LOG_CALIB_NOTICE(fmt, ...) QUILL_LOG_NOTICE(::logging::s_loggerCalibration, fmt, ##__VA_ARGS__)
#define LOG_CALIB_WARNING(fmt, ...) QUILL_LOG_WARNING(::logging::s_loggerCalibration, fmt, ##__VA_ARGS__)
#define LOG_CALIB_WARN(fmt, ...) QUILL_LOG_WARNING(::logging::s_loggerCalibration, fmt, ##__VA_ARGS__)
#define LOG_CALIB_ERROR(fmt, ...) QUILL_LOG_ERROR(::logging::s_loggerCalibration, fmt, ##__VA_ARGS__)
#define LOG_CALIB_CRITICAL(fmt, ...) QUILL_LOG_CRITICAL(::logging::s_loggerCalibration, fmt, ##__VA_ARGS__)
#define LOG_CALIB_FATAL(fmt, ...) QUILL_LOG_CRITICAL(::logging::s_loggerCalibration, fmt, ##__VA_ARGS__)
#define LOG_CALIB_DYNAMIC(log_level, fmt, ...) QUILL_LOG_DYNAMIC(::logging::s_loggerCalibration, log_level, fmt, ##__VA_ARGS__)

#define LOG_BLUETOOTH_DEBUG(fmt, ...) QUILL_LOG_DEBUG(::logging::s_loggerBluetooth, fmt, ##__VA_ARGS__)
#define LOG_BLUETOOTH_INFO(fmt, ...) QUILL_LOG_INFO(::logging::s_loggerBluetooth, fmt, ##__VA_ARGS__)
#define LOG_BLUETOOTH_NOTICE(fmt, ...) QUILL_LOG_NOTICE(::logging::s_loggerBluetooth, fmt, ##__VA_ARGS__)
#define LOG_BLUETOOTH_WARNING(fmt, ...) QUILL_LOG_WARNING(::logging::s_loggerBluetooth, fmt, ##__VA_ARGS__)
#define LOG_BLUETOOTH_WARN(fmt, ...) QUILL_LOG_WARNING(::logging::s_loggerBluetooth, fmt, ##__VA_ARGS__)
#define LOG_BLUETOOTH_ERROR(fmt, ...) QUILL_LOG_ERROR(::logging::s_loggerBluetooth, fmt, ##__VA_ARGS__)
#define LOG_BLUETOOTH_CRITICAL(fmt, ...) QUILL_LOG_CRITICAL(::logging::s_loggerBluetooth, fmt, ##__VA_ARGS__)
#define LOG_BLUETOOTH_FATAL(fmt, ...) QUILL_LOG_CRITICAL(::logging::s_loggerBluetooth, fmt, ##__VA_ARGS__)
#define LOG_BLUETOOTH_DYNAMIC(log_level, fmt, ...) QUILL_LOG_DYNAMIC(::logging::s_loggerBluetooth, log_level, fmt, ##__VA_ARGS__)
