#pragma once

#include "quill/Backend.h"
#include "quill/Frontend.h"
#include "quill/LogMacros.h"
#include "quill/Logger.h"
#include "quill/sinks/ConsoleSink.h"
#include "quill/sinks/FileSink.h"


#include "protocol.h"
#include <quill/bundled/fmt/chrono.h>
#include <quill/bundled/fmt/format.h>
#include <volk/volk.h>


#if (__clang__)
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wswitch"
#endif // __clang__

namespace ipc::protocol {
std::string format_as(Version_t version);
}

namespace vr {
std::string format_as(EVROverlayError overlayErr);
std::string format_as(EVRInitError initErr);
std::string format_as(ETrackedPropertyError trackedPropertyErr);
std::string format_as(EVRInputError inputErr);
std::string format_as(EVRSpatialAnchorError spatialAnchorErr);
std::string format_as(EVRNotificationError notificationErr);
std::string format_as(ETrackedDeviceClass trackedDeviceClass);
std::string format_as(ETrackedDeviceProperty trackedDeviceProperty);
std::string format_as(EVRSettingsError settingsError);
std::string format_as(ETrackedControllerRole controllerRole);
std::string format_as(EVREventType eventType);
}

std::string format_as(VkResult vkResult);

#if (__clang__)
#pragma clang diagnostic pop
#endif // __clang__
