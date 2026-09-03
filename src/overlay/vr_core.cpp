#include "vr_core.h"
#include "base_station_management.h"
#include "calibration.h"
#include "configuration.h"
#include "constants.h"
#include "localisation.h"
#include "log.h"
#include "platform.h"
#include "util.h"

BEGIN_EXTERNAL_HEADERS
#include <filesystem>
#include <fmt/format.h>
#include <glaze/glaze.hpp>

END_EXTERNAL_HEADERS

namespace spacecal {

// disabled in debug builds
// @FIXME: Should this be behind an advanced or obscure toggle?
// these tracking systems are explicitly hidden in Space Calibrator, as it does not make sense for Space Calibrator to "calibrate" such trackers
constexpr const char* k_IGNORED_TRACKING_SYSTEMS[] = {
    "null", // only actual hardware is supported, null driver is a debug device and will not be accepted with space calibrator
    "standable", // virtual trackers will likely interfere with space calibrator
};

// keys for localisation
constexpr const char* k_VIRTUAL_DESKTOP_HMD_NAMES[] = {
    "virtual_desktop_hmd_none", // should never be displayed but you never know
    "virtual_desktop_hmd_gearvr",
    "virtual_desktop_hmd_oculus_go",
    "virtual_desktop_hmd_oculus_quest_1",
    "virtual_desktop_hmd_oculus_quest_2",
    "virtual_desktop_hmd_meta_quest_pro",
    "virtual_desktop_hmd_pico_4",
    "virtual_desktop_hmd_meta_quest_3",
    "virtual_desktop_hmd_meta_quest_3s",
    "virtual_desktop_hmd_pico_4_ultra",
    "virtual_desktop_hmd_samsung_galaxy_xr",
    "virtual_desktop_hmd_play_for_dream_mr",
};

constexpr const char* k_VIRTUAL_DESKTOP_SERIAL_NUMBER = "1PASH5D1P17365";

VRState* VRState::s_instance = nullptr;

bool VRState::init()
{
    if (s_instance != nullptr) {
        LOG_FATAL("Tried creating VRState more than once! Breaking singleton. Aborting...");
        return false;
    }
    s_instance = this;
    m_bIsSteamVrAvailable = false;
    m_aTrackingSystems.reserve(8); // conservative amount, should account for 99.9% of cases with ease

    m_eVrInitError = vr::VRInitError_None;
    vr::VR_Init(&m_eVrInitError, vr::VRApplication_Overlay);
    if (m_eVrInitError != vr::VRInitError_None) {
        auto szError = vr::VR_GetVRInitErrorAsEnglishDescription(m_eVrInitError);
        LOG_OPENVR_CRITICAL("vr::VR_Init failed, got {}", szError);
        platform::showMessageDialog(
            "An error occured initialising Space Calibrator Nova",
            fmt::format("Error initialising SteamVR: {}", szError));
        return false;
    }

    // ensure the interfaces we use are valid and correct
    if (!vr::VR_IsInterfaceVersionValid(vr::IVRSystem_Version)) {
        // @TODO: ui should show this
        LOG_OPENVR_CRITICAL("OpenVR interface vr::IVRSystem version is invalid! Aborting...");
        return false;
    } else if (!vr::VR_IsInterfaceVersionValid(vr::IVRSettings_Version)) {
        // @TODO: ui should show this
        LOG_OPENVR_CRITICAL("OpenVR interface vr::IVRSettings version is invalid! Aborting...");
        return false;
    } else if (!vr::VR_IsInterfaceVersionValid(vr::IVROverlay_Version)) {
        // @TODO: ui should show this
        LOG_OPENVR_CRITICAL("OpenVR interface vr::IVROverlay version is invalid! Aborting...");
        return false;
    }

    // create overlay
    if (!vr::VROverlay() || m_overlayMainHandle != vr::k_ulOverlayHandleInvalid) {
        return false;
    }

    vr::VROverlayError error = vr::VROverlay()->CreateDashboardOverlay(
        c_OPENVR_APPLICATION_KEY, "Space Calibrator",
        &m_overlayMainHandle, &m_overlayThumbnailHandle);

    if (error == vr::VROverlayError_KeyInUse) {
        LOG_OPENVR_CRITICAL("Another instance of Space Calibrator is already running");
        platform::showMessageDialog("An error occured initialising Space Calibrator Nova", "Another instance of Space Calibrator is already running");
        return false;
    } else if (error != vr::VROverlayError_None) {
        LOG_OPENVR_CRITICAL("Error creating VR overlay: {}", vr::VROverlay()->GetOverlayErrorNameFromEnum(error));
        platform::showMessageDialog(
            "An error occured initialising Space Calibrator Nova",
            fmt::format("Error creating VR overlay: {}", vr::VROverlay()->GetOverlayErrorNameFromEnum(error)));
        return false;
    }

    vr::VROverlay()->SetOverlayWidthInMeters(m_overlayMainHandle, 3.0f);
    vr::VROverlay()->SetOverlayInputMethod(m_overlayMainHandle, vr::VROverlayInputMethod_Mouse);
    vr::VROverlay()->SetOverlayFlag(m_overlayMainHandle, vr::VROverlayFlags_SendVRDiscreteScrollEvents, true);

    std::string iconPath = fmt::format("{}/icon.png", util::getSpaceCalibratorInstallDir().string());
    vr::VROverlay()->SetOverlayFromFile(m_overlayThumbnailHandle, iconPath.c_str());

    // @TODO: Non-steam stuff

    // @TODO: log hmd info

    // start base stations on startup if enabled
    if (ConfigurationManager::getInstance()->getConfiguration()->base_stations.auto_power_management_enabled && ConfigurationManager::getInstance()->getConfiguration()->base_stations.auto_turn_on_during_startup) {
        bluetooth::set_all_base_station_power_state(bluetooth::PowerState_Awake_From_Sleep);
    }

    m_bIsSteamVrAvailable = true;
    return true;
}

vr::ETrackedPropertyError VRState::getSteamVrPropString(const vr::TrackedDeviceIndex_t deviceId, vr::ETrackedDeviceProperty deviceProperty, std::string& string) const
{
    char buffer[vr::k_unMaxPropertyStringSize] = {};
    vr::ETrackedPropertyError err = vr::TrackedProp_Success;
    vr::VRSystem()->GetStringTrackedDeviceProperty(deviceId, deviceProperty, buffer, vr::k_unMaxPropertyStringSize, &err);
    if (err == vr::TrackedProp_Success) {
        string = std::string(buffer);
    }
    return err;
}

bool VRState::updateSteamVRDevice(const vr::TrackedDeviceIndex_t deviceId)
{
    vr::ETrackedDeviceClass deviceClass = vr::VRSystem()->GetTrackedDeviceClass(deviceId);

    // we dont care about these devices types
    if (deviceClass == vr::TrackedDeviceClass_Invalid // Unset
        || deviceClass == vr::TrackedDeviceClass_DisplayRedirect) // vr::IVRVirtualDisplay
        return true;

    bool isConnected = vr::VRSystem()->IsTrackedDeviceConnected(deviceId);
    m_aDevices[deviceId].bIsConnected = isConnected;

    // we don't care about the device if it is not even connected
    if (!isConnected) {
        return true;
    }

    vr::ETrackedPropertyError err = vr::TrackedProp_Success;
    std::string szTrackingSystem;
    err = getSteamVrPropString(deviceId, vr::Prop_TrackingSystemName_String, szTrackingSystem);

    // only log if its failed and not unset / not avail rn
    if (err != vr::TrackedProp_Success) {
        if (err != vr::TrackedProp_UnknownProperty && err != vr::TrackedProp_NotYetAvailable) {
            LOG_OPENVR_WARN("Failed to get tracking system string for device with id {}, got error {} ({})", deviceId, err, (uint32_t)err);
        }
        return false;
    }

#if !defined(_DEBUG)
    // if the tracking system should be ignored, ignore it
    for (size_t i = 0; i < std::size(k_IGNORED_TRACKING_SYSTEMS); i++) {
        if (strncmp(szTrackingSystem.c_str(), k_IGNORED_TRACKING_SYSTEMS[i], szTrackingSystem.size()) == 0) {
            return true; // this is a don't care
        }
    }
#endif

    std::string szDeviceModel;
    std::string szDeviceSerial;
    err = getSteamVrPropString(deviceId, vr::Prop_ModelNumber_String, szDeviceModel);
    err = getSteamVrPropString(deviceId, vr::Prop_SerialNumber_String, szDeviceSerial);

    // QUIRKS! ( a bunch of hardware lies about what it is :c )
    {
        // Possibly Virtual Desktop on a non Meta HMD
        if (deviceClass == vr::TrackedDeviceClass_HMD && szTrackingSystem == "oculus") {
            if (m_hmdMetadata.isVirtualDesktopAvailable /* && isHmdVirtualDesktop() */) {
                // VD hardcodes the sn to "1PASH5D1P17365"
                if (szDeviceSerial == k_VIRTUAL_DESKTOP_SERIAL_NUMBER && m_hmdMetadata.VD_hmdModel > ipc::protocol::VD_HmdModel_None && m_hmdMetadata.VD_hmdModel < ipc::protocol::VD_HmdModel_Count) {
                    szDeviceModel = LOCALE_GET(k_VIRTUAL_DESKTOP_HMD_NAMES[m_hmdMetadata.VD_hmdModel]);
                }
            }
        }
        // Check if the current HMD is a Pimax crystal
        else if (deviceClass == vr::TrackedDeviceClass_HMD && szTrackingSystem == "aapvr") {
            // HMD is a Pimax HMD
            vr::HmdMatrix34_t eyeToHeadLeft = vr::VRSystem()->GetEyeToHeadTransform(vr::Eye_Left);
            // Crystal's projection matrix is constant 0s or 1s except for [0][3], which stores the IPD offset from the nose
            bool isCrystalHmd = eyeToHeadLeft.m[0][0] == 1 && eyeToHeadLeft.m[0][1] == 0 && eyeToHeadLeft.m[0][2] == 0 && // IPD
                eyeToHeadLeft.m[1][0] == 0 && eyeToHeadLeft.m[1][1] == 1 && eyeToHeadLeft.m[1][2] == 0 && eyeToHeadLeft.m[1][3] == 0 && eyeToHeadLeft.m[2][0] == 0 && eyeToHeadLeft.m[2][1] == 0 && eyeToHeadLeft.m[2][2] == 1 && eyeToHeadLeft.m[2][3] == 0;

            if (isCrystalHmd) {
                // Move it outside the aapvr system ; we treat aapvr as if it were lighthouse
                szTrackingSystem = "Pimax Crystal HMD";
            }
        }
        // Pimax cystal controllers
        else if (deviceClass == vr::TrackedDeviceClass_Controller && szTrackingSystem == "oculus") {
            std::string renderModel;
            std::string connectedWirelessDongle;
            err = getSteamVrPropString(deviceId, vr::Prop_RenderModelName_String, renderModel);
            err = getSteamVrPropString(deviceId, vr::Prop_ConnectedWirelessDongle_String, connectedWirelessDongle);

            // Check if the controller claims its an oculus controller but also pimax
            if (renderModel.find("{aapvr}") != std::string::npos && renderModel.find("crystal") != std::string::npos && connectedWirelessDongle.find("lighthouse") != std::string::npos) {
                szTrackingSystem = "Pimax Crystal Controllers";
            }
        }
    }

    // track new tracking systems, prioritise HMD one at front of list
    {
        auto existing = std::find(m_aTrackingSystems.begin(), m_aTrackingSystems.end(), szTrackingSystem);
        if (existing != m_aTrackingSystems.end()) {
            if (deviceClass == vr::TrackedDeviceClass_HMD) {
                m_aTrackingSystems.erase(existing);
                m_aTrackingSystems.insert(m_aTrackingSystems.begin(), szTrackingSystem);
            }
        } else {
            m_aTrackingSystems.push_back(szTrackingSystem);
        }
    }

    // update tracking state
    vr::ETrackedControllerRole controllerRole = (vr::ETrackedControllerRole)vr::VRSystem()->GetInt32TrackedDeviceProperty(deviceId, vr::Prop_ControllerRoleHint_Int32, &err);

    m_aDevices[deviceId] = {
        .bIsConnected = isConnected,
        .dwDeviceIndex = deviceId,
        .eControllerRole = controllerRole,
        .eDeviceClass = deviceClass,
        .szTrackingSystemId = szTrackingSystem,
        .szModel = szDeviceModel,
        .szSerial = szDeviceSerial,
    };

    return true;
}

void VRState::updateVrState()
{
    if (!m_bIsSteamVrAvailable)
        return;

    if (m_aTrackingSystems.size() == 0 || m_bStateDirty) {
        m_bStateDirty = false;

        // fresh poll, go through everything because we're in a fresh state
        for (vr::TrackedDeviceIndex_t id = 0; id < vr::k_unMaxTrackedDeviceCount; ++id) {
            if (vr::VRSystem()->IsTrackedDeviceConnected(id)) {
                // only clear dirty if we successfully updated ALL devices!
                m_bStateDirty |= !updateSteamVRDevice(id);
            } else {
                m_aDevices[id].bIsConnected = false;
            }
        }

        // device props may have just become avail, attempt to retry assigning devices to handle
        for (size_t i = 0; i < CalibrationManager::getInstance()->getCalibrationCount(); i++) {
            CalibrationManager::getInstance()->getCalibration(i).tryAssigningTargets();
        }

        // an event that matters happened, re-apply calibration for good measure!
        CalibrationManager::getInstance()->apply();
    }

    vr::VREvent_t vrEvent = {};
    while (vr::VRSystem()->PollNextEvent(&vrEvent, sizeof(vrEvent))) {
        switch (vrEvent.eventType) {
            // @TODO: Handle these??
        case vr::EVREventType::VREvent_Input_TrackerActivated:
        case vr::EVREventType::VREvent_TrackedDeviceActivated: {
            LOG_OPENVR_INFO("New device connected at index {}", vrEvent.trackedDeviceIndex);
            updateSteamVRDevice(vrEvent.trackedDeviceIndex);
            m_bStateDirty = true;
            break;
        }
        case vr::EVREventType::VREvent_TrackedDeviceDeactivated:
            LOG_OPENVR_INFO("Device disconnected at index {}", vrEvent.trackedDeviceIndex);
            updateSteamVRDevice(vrEvent.trackedDeviceIndex);
            m_bStateDirty = true;
            break;
        case vr::EVREventType::VREvent_TrackedDeviceUpdated:
            LOG_OPENVR_INFO("Device update at index {}", vrEvent.trackedDeviceIndex);
            updateSteamVRDevice(vrEvent.trackedDeviceIndex);
            m_bStateDirty = true;
            break;
        case vr::EVREventType::VREvent_TrackedDeviceRoleChanged:
            LOG_OPENVR_INFO("Device role change");
            // trackedDeviceIndex is invalid so fuck if i know what changed
            // why its not populated is beyond me but oh well
            m_bStateDirty = true;
            break;

        // @TODO: nuke the insanely verbose logging from events
        // suspect: lighthouse "universe" jumps are actually chaperone being weird
        case vr::EVREventType::VREvent_ChaperoneDataHasChanged:
            invalidateAllSamples();
            LOG_OPENVR_INFO("Chaperone data change event at index {}", vrEvent.trackedDeviceIndex);
            break;
        case vr::EVREventType::VREvent_ChaperoneUniverseHasChanged:
            invalidateAllSamples();
            LOG_OPENVR_INFO("Chaperone universe change event at index {}", vrEvent.trackedDeviceIndex);
            break;
        case vr::EVREventType::VREvent_ChaperoneTempDataHasChanged:
            invalidateAllSamples();
            LOG_OPENVR_INFO("Chaperone temp data change event at index {}", vrEvent.trackedDeviceIndex);
            break;
        case vr::EVREventType::VREvent_ChaperoneSettingsHaveChanged:
            invalidateAllSamples();
            LOG_OPENVR_INFO("Chaperone settings change event at index {}", vrEvent.trackedDeviceIndex);
            break;
        case vr::EVREventType::VREvent_SeatedZeroPoseReset:
            invalidateAllSamples();
            LOG_OPENVR_INFO("Chaperone seated recenter event at index {}", vrEvent.trackedDeviceIndex);
            break;
        case vr::EVREventType::VREvent_ChaperoneFlushCache:
            invalidateAllSamples();
            LOG_OPENVR_INFO("Chaperone flush cache event at index {}", vrEvent.trackedDeviceIndex);
            break;
        case vr::EVREventType::VREvent_ChaperoneRoomSetupStarting:
            invalidateAllSamples();
            LOG_OPENVR_INFO("Chaperone room setup init event at index {}", vrEvent.trackedDeviceIndex);
            break;
        case vr::EVREventType::VREvent_ChaperoneRoomSetupCommitted:
            invalidateAllSamples();
            LOG_OPENVR_INFO("Chaperone room setup commit event at index {}", vrEvent.trackedDeviceIndex);
            break;
        case vr::EVREventType::VREvent_StandingZeroPoseReset:
            invalidateAllSamples();
            LOG_OPENVR_INFO("Chaperone standing recenter event at index {}", vrEvent.trackedDeviceIndex);
            break;
        case vr::EVREventType::VREvent_Reserved_0809:
            invalidateAllSamples();
            LOG_OPENVR_INFO("Chaperone unk 809 event at index {}", vrEvent.trackedDeviceIndex);
            break;
        case vr::EVREventType::VREvent_Reserved_0810:
            invalidateAllSamples();
            LOG_OPENVR_INFO("Chaperone unk 810 event at index {}", vrEvent.trackedDeviceIndex);
            break;
        case vr::EVREventType::VREvent_Reserved_0811:
            invalidateAllSamples();
            LOG_OPENVR_INFO("Chaperone unk 811 event at index {}", vrEvent.trackedDeviceIndex);
            break;

        case vr::EVREventType::VREvent_EnterStandbyMode:
        case vr::EVREventType::VREvent_LeaveStandbyMode:
        case vr::EVREventType::VREvent_WirelessDisconnect:
        case vr::EVREventType::VREvent_WirelessReconnect:
            invalidateAllSamples();
            break;

        // @HACK: catch a bunch of dont cares here
        case vr::EVREventType::VREvent_Input_HapticVibration:
        case vr::EVREventType::VREvent_PropertyChanged:
        case vr::EVREventType::VREvent_Reserved_0114: // no fucking clue what it is but steamvr spams it

        case vr::EVREventType::VREvent_BackgroundSettingHasChanged:
        case vr::EVREventType::VREvent_CameraSettingsHaveChanged:
        case vr::EVREventType::VREvent_ReprojectionSettingHasChanged:
        case vr::EVREventType::VREvent_ModelSkinSettingsHaveChanged:
        case vr::EVREventType::VREvent_EnvironmentSettingsHaveChanged:
        case vr::EVREventType::VREvent_PowerSettingsHaveChanged:
        case vr::EVREventType::VREvent_EnableHomeAppSettingsHaveChanged:
        case vr::EVREventType::VREvent_SteamVRSectionSettingChanged:
        case vr::EVREventType::VREvent_LighthouseSectionSettingChanged:
        case vr::EVREventType::VREvent_NullSectionSettingChanged:
        case vr::EVREventType::VREvent_UserInterfaceSectionSettingChanged:
        case vr::EVREventType::VREvent_NotificationsSectionSettingChanged:
        case vr::EVREventType::VREvent_KeyboardSectionSettingChanged:
        case vr::EVREventType::VREvent_PerfSectionSettingChanged:
        case vr::EVREventType::VREvent_DashboardSectionSettingChanged:
        case vr::EVREventType::VREvent_WebInterfaceSectionSettingChanged:
        case vr::EVREventType::VREvent_TrackersSectionSettingChanged:
        case vr::EVREventType::VREvent_LastKnownSectionSettingChanged:
        case vr::EVREventType::VREvent_DismissedWarningsSectionSettingChanged:
        case vr::EVREventType::VREvent_GpuSpeedSectionSettingChanged:
        case vr::EVREventType::VREvent_WindowsMRSectionSettingChanged:
        case vr::EVREventType::VREvent_OtherSectionSettingChanged:
        case vr::EVREventType::VREvent_AnyDriverSettingsChanged:

        case vr::EVREventType::VREvent_ButtonPress:
        case vr::EVREventType::VREvent_ButtonUnpress:
        case vr::EVREventType::VREvent_ButtonTouch:
        case vr::EVREventType::VREvent_ButtonUntouch:
        case vr::EVREventType::VREvent_Modal_Cancel:
        case vr::EVREventType::VREvent_MouseMove:
        case vr::EVREventType::VREvent_MouseButtonDown:
        case vr::EVREventType::VREvent_MouseButtonUp:
        case vr::EVREventType::VREvent_FocusEnter:
        case vr::EVREventType::VREvent_FocusLeave:
        case vr::EVREventType::VREvent_ScrollDiscrete:
        case vr::EVREventType::VREvent_TouchPadMove:
        case vr::EVREventType::VREvent_OverlayFocusChanged:
        case vr::EVREventType::VREvent_ReloadOverlays:
        case vr::EVREventType::VREvent_ScrollSmooth:
        case vr::EVREventType::VREvent_LockMousePosition:
        case vr::EVREventType::VREvent_UnlockMousePosition:
        case vr::EVREventType::VREvent_InputFocusCaptured:
        case vr::EVREventType::VREvent_InputFocusReleased:
        case vr::EVREventType::VREvent_SceneApplicationChanged:
        case vr::EVREventType::VREvent_InputFocusChanged:
        case vr::EVREventType::VREvent_SceneApplicationUsingWrongGraphicsAdapter:
        case vr::EVREventType::VREvent_ActionBindingReloaded:
        case vr::EVREventType::VREvent_HideRenderModels:
        case vr::EVREventType::VREvent_ShowRenderModels:
        case vr::EVREventType::VREvent_SceneApplicationStateChanged:
        case vr::EVREventType::VREvent_SceneAppPipeDisconnected:
        case vr::EVREventType::VREvent_KeyboardOpened_Global:
        case vr::EVREventType::VREvent_KeyboardClosed:
        case vr::EVREventType::VREvent_KeyboardClosed_Global:
        case vr::EVREventType::VREvent_KeyboardDone:
        case vr::EVREventType::VREvent_KeyboardCharInput:

        case vr::EVREventType::VREvent_RequestScreenshot:
        case vr::EVREventType::VREvent_ScreenshotTaken:
        case vr::EVREventType::VREvent_ScreenshotFailed:
        case vr::EVREventType::VREvent_SubmitScreenshotToDashboard:
        case vr::EVREventType::VREvent_ScreenshotProgressToDashboard:

        case vr::EVREventType::VREvent_PrimaryDashboardDeviceChanged:
        case vr::EVREventType::VREvent_RoomViewShown:
        case vr::EVREventType::VREvent_RoomViewHidden:
        case vr::EVREventType::VREvent_ShowUI:
        case vr::EVREventType::VREvent_ShowDevTools:
        case vr::EVREventType::VREvent_DesktopViewUpdating:
        case vr::EVREventType::VREvent_DesktopViewReady:

        case vr::EVREventType::VREvent_StartDashboard:
        case vr::EVREventType::VREvent_ElevatePrism:
        case vr::EVREventType::VREvent_OverlayClosed:
        case vr::EVREventType::VREvent_DashboardThumbChanged:
        case vr::EVREventType::VREvent_DesktopMightBeVisible:
        case vr::EVREventType::VREvent_DesktopMightBeHidden:
        case vr::EVREventType::VREvent_MutualSteamCapabilitiesChanged:
        case vr::EVREventType::VREvent_OverlayCreated:
        case vr::EVREventType::VREvent_OverlayDestroyed:

        case vr::EVREventType::VREvent_TrackedDeviceUserInteractionStarted:
        case vr::EVREventType::VREvent_TrackedDeviceUserInteractionEnded:
            break;
        case vr::EVREventType::VREvent_Quit: {
            // turn off base stations at vr shutdown if enabled
            if (ConfigurationManager::getInstance()->getConfiguration()->base_stations.auto_power_management_enabled && ConfigurationManager::getInstance()->getConfiguration()->base_stations.auto_turn_off_during_shutdown) {
                bluetooth::EPowerState_t ePowerState = ConfigurationManager::getInstance()->getConfiguration()->base_stations.off_should_use_standby ? bluetooth::PowerState_Standby : bluetooth::PowerState_Sleep;
                bluetooth::set_all_base_station_power_state(ePowerState);
            }
        } break;

        default:
            LOG_OPENVR_INFO("Unknown evt with id {}", vrEvent.eventType);
            break;
        }
    }

    if (m_hmdMetadata.isVirtualDesktopAvailable != m_lastHmdMetaState.isVirtualDesktopAvailable || m_hmdMetadata.VD_hmdModel != m_lastHmdMetaState.VD_hmdModel || m_hmdMetadata.VD_stageTrackingEnabled != m_lastHmdMetaState.VD_stageTrackingEnabled) {
        m_bStateDirty = true;
        m_lastHmdMetaState = m_hmdMetadata;
    }
}

void VRState::invalidateAllSamples()
{
    for (size_t i = 0; i < CalibrationManager::getInstance()->getCalibrationCount(); i++) {
        CalibrationManager::getInstance()->getCalibration(i).clearSamples();
        CalibrationManager::getInstance()->getCalibration(i).invalidateMetrics();
    }
}

bool VRState::isHmdVirtualDesktop() const
{
    if (!vr::VRSystem())
        return false;

    // VD sets ResourceRoot as "virtualdesktop" on the HMD device
    std::string szResourceRoot;
    vr::ETrackedPropertyError err = getSteamVrPropString(vr::k_unTrackedDeviceIndex_Hmd, vr::ETrackedDeviceProperty::Prop_ResourceRoot_String, szResourceRoot);
    if (err == vr::ETrackedPropertyError::TrackedProp_Success) {
        if (szResourceRoot == "virtualdesktop") {
            return true;
        }
    }

    return false;
}

const VRDevice_t VRState::findVrDevice(const std::string& trackingSystem, const std::string& model, const std::string& serial) const
{
    // Find the device with the matching tracking system, model and serial
    for (int i = 0; i < vr::k_unMaxTrackedDeviceCount; i++) {
        const auto& device = m_aDevices[i];

        if (!device.bIsConnected) {
            continue;
        }

        uint8_t matches = 0;

        // guard against empty data
        if (model.empty() || serial.empty() || trackingSystem.empty() || device.szModel.empty() || device.szSerial.empty() || device.szTrackingSystemId.empty()) {
            continue;
        }

        if (device.szModel == model) {
            matches++;
        }
        if (device.szSerial == serial) {
            matches++;
        }

        // Only return if:
        //   - The tracking system is identical
        //   - If the device is not a HMD, the model and serial number ALSO are identical.
        //   - If the device is the HMD, the model OR serial number are also identical.
        //
        // This handles an edge case of some device drivers being poorly developed or misbehaving, returning bad data to SteamVR and in turn, Space Calibrator
        // e.g.   SteamLink sometimes reports the Quest Pro as either "Oculus Quest Pro" or "Oculus Quest2" as it's model string
        //        It still reports the serial number correctly however as "VRLINKHMDQUESTPRO"!
        if (device.szTrackingSystemId == trackingSystem && ((matches == 2 && device.eDeviceClass != vr::TrackedDeviceClass::TrackedDeviceClass_HMD) || (matches >= 1 && device.eDeviceClass == vr::TrackedDeviceClass::TrackedDeviceClass_HMD))) {
            return device;
        }
    }

    return {};
}

const VRDevice_t VRState::getVrDevice(const size_t index) const
{
    if (0 <= index && index < vr::k_unMaxTrackedDeviceCount) {
        return m_aDevices[index];
    }
    ASSERT(0 <= index && index < vr::k_unMaxTrackedDeviceCount, "Invalid index, out of bounds read!");
    return {};
}

void VRState::identifyDevice(const vr::TrackedDeviceIndex_t deviceId) const
{
    // @TODO: implement another method of identifying the selected device! not everything has LEDs or haptic motors, and viewing LEDs is challenging in VR!
    // @TODO: VRInput??
    vr::VRSystem()->TriggerHapticPulse(deviceId, 0, 2000);
}

void VRState::debugListDevices() const
{
    for (int i = 0; i < vr::k_unMaxTrackedDeviceCount; i++) {
        const auto& device = m_aDevices[i];

        if (!device.bIsConnected) {
            continue;
        }

        LOG_OPENVR_INFO("Device [{}] : {} {} {} ({})", i, device.szTrackingSystemId, device.szModel, device.szSerial, device.eDeviceClass);
    }
}
// @TODO: Proper linux path
void VRState::tryLoadVrPaths()
{
    if (m_openvrPaths.version != 0) {
        return;
    }
    auto vrPath = platform::getSteamvrVrPathsPath();

    constexpr glz::opts options {
        .comments = true,
        .error_on_unknown_keys = false,
        .error_on_missing_keys = false,
        .error_on_const_read = false
    };

    // Get data version to decode config correctly
    std::string jsonConfigRaw;
    if (jsonConfigRaw.empty()) {
        FILE* pFile = fopen(vrPath.string().c_str(), "rb");
        fseek(pFile, 0, SEEK_END);
        size_t fileSize = ftell(pFile);
        jsonConfigRaw.resize(fileSize);
        rewind(pFile);
        fread(&jsonConfigRaw[0], 1, fileSize, pFile);
        fclose(pFile);
    }

    glz::error_ctx deserialiseErrorVersion = glz::read<glz::set_json<options>()>(m_openvrPaths, std::forward<std::string>(jsonConfigRaw), glz::context {});
    if (deserialiseErrorVersion.ec != glz::error_code::none) {
        // FUCK
        LOG_WARNING("Failed to parse openvrpaths.vrpath file \"{0}\". {1}", vrPath.string(), deserialiseErrorVersion.custom_error_message);
        m_openvrPaths.version = 0;
        return;
    }
}

#if OS_WINDOWS
#define DRIVER_ARCH_DIR "win64"
#elif OS_LINUX
#if ARCH_X64
#define DRIVER_ARCH_DIR "linux64"
#elif ARCH_AARCH64
#define DRIVER_ARCH_DIR "linuxarm64"
#else
#error "Unknown CPU arch"
#endif
#else
#error "Unsupported OS"
#endif

bool VRState::isConflictingDriverInstalled()
{
    tryLoadVrPaths();
    if (m_openvrPaths.version == 0) {
        return false;
    }
    std::error_code ec;
    std::filesystem::path exeDir = platform::getExeDir();

    // exeDir should match driverDir via external_drivers under a good install

    // check drivers dir for spacecal entry in steamvr dir
    for (const std::string& szEntry : m_openvrPaths.runtime) {
        std::filesystem::path driversDir = std::filesystem::path(szEntry) / "drivers";
        if (!std::filesystem::exists(driversDir, ec))
            continue;
        for (const auto& entry : std::filesystem::directory_iterator(driversDir)) {
            if (std::filesystem::is_regular_file(entry.path() / "bin" / DRIVER_ARCH_DIR / "driver_01spacecalibrator.dll")) {
                return true;
            }
        }
    }

    // check external drivers dirs for spacecal entry in steamvr dir
    for (const std::string& szEntry : m_openvrPaths.external_drivers) {
        std::filesystem::path driverDir = std::filesystem::path(szEntry);
        if (!std::filesystem::exists(driverDir, ec))
            continue;
        if (std::filesystem::is_regular_file(driverDir / "bin" / DRIVER_ARCH_DIR / "driver_01spacecalibrator.dll", ec)) {
            return driverDir != exeDir;
        }
    }

    return false;
}

bool VRState::isSpaceCalibratorDriverAvailable()
{
    tryLoadVrPaths();
    if (m_openvrPaths.version == 0) {
        return false;
    }
    std::error_code ec;
    std::filesystem::path exeDir = platform::getExeDir();
    // exeDir should match driverDir via external_drivers under a good install

    // check external drivers dirs for spacecal entry in steamvr dir
    for (const std::string& szEntry : m_openvrPaths.external_drivers) {
        std::filesystem::path driverDir = std::filesystem::path(szEntry);
        if (!std::filesystem::exists(driverDir, ec))
            continue;
        if (std::filesystem::is_regular_file(driverDir / "bin" / DRIVER_ARCH_DIR / "driver_01spacecalibrator.dll", ec)) {
            return driverDir == exeDir;
        }
    }

    return false;
}

bool VRState::removeConflictingDrivers()
{
    tryLoadVrPaths();
    if (m_openvrPaths.version == 0) {
        return false;
    }
    std::error_code ec;
    std::filesystem::path exeDir = platform::getExeDir();
    // exeDir should match driverDir via external_drivers under a good install

    bool bSuccess = true;

    // check drivers dir for spacecal entry in steamvr dir
    for (const std::string& szEntry : m_openvrPaths.runtime) {
        std::filesystem::path driversDir = std::filesystem::path(szEntry) / "drivers";
        if (!std::filesystem::exists(driversDir, ec))
            continue;
        for (const auto& entry : std::filesystem::directory_iterator(driversDir)) {
            if (std::filesystem::is_regular_file(entry.path() / "bin" / DRIVER_ARCH_DIR / "driver_01spacecalibrator.dll", ec)) {

                for (const auto& dir_entry : std::filesystem::recursive_directory_iterator(entry.path())) {
#if OS_WINDOWS
                    // queue the thing for deletion
                    bSuccess = bSuccess && MoveFileExW(dir_entry.path().wstring().c_str(), NULL, MOVEFILE_DELAY_UNTIL_REBOOT);
#elif OS_LINUX
// @TODO: idk???
#else
#endif
                }
            }
        }
    }

    // remove entries of spacecal from external_drivers
    std::erase_if(m_openvrPaths.external_drivers, [exeDir](const std::string& szEntry) {
        std::error_code ec;
        std::filesystem::path driverDir(szEntry);

        return (driverDir != exeDir) && // is not this install of spacecal
            std::filesystem::exists(driverDir, ec) && // folder exists
            std::filesystem::is_regular_file(driverDir / "bin" / DRIVER_ARCH_DIR / "driver_01spacecalibrator.dll", ec); // is a valid spacecal driver
    });

    auto vrPath = platform::getSteamvrVrPathsPath().string();
    constexpr glz::opts options {
        .comments = false,
        .error_on_unknown_keys = false,
        .prettify = true,
        .indentation_char = ' ',
        .indentation_width = 4,
        .error_on_missing_keys = false,
        .error_on_const_read = false,
    };
    glz::error_ctx serialiseError = glz::write_file_json<options>(m_openvrPaths, vrPath.c_str(), std::string {});
    if (serialiseError.ec != glz::error_code::none) {
        // FUCK
        LOG_WARNING("Failed to write openvrpaths file \"{0}\". {1}", vrPath, serialiseError.custom_error_message);
        return false;
    }
    return bSuccess;
}

bool VRState::registerSpaceCalibratorDriver()
{
    tryLoadVrPaths();
    if (m_openvrPaths.version == 0) {
        return false;
    }
    std::error_code ec;
    std::filesystem::path exeDir = platform::getExeDir();

    // add exeDir to runtime paths IF and ONLY IFF exeDir is NOT already present

    // check external drivers dirs for spacecal entry in steamvr dir
    bool bDriverIsInstalledAlready = false;
    for (const std::string& szEntry : m_openvrPaths.external_drivers) {
        std::filesystem::path driverDir = std::filesystem::path(szEntry);
        if (!std::filesystem::exists(driverDir, ec))
            continue;
        if (std::filesystem::is_regular_file(driverDir / "bin" / DRIVER_ARCH_DIR / "driver_01spacecalibrator.dll", ec)) {
            bDriverIsInstalledAlready = bDriverIsInstalledAlready || (driverDir == exeDir);
        }
    }

    if (bDriverIsInstalledAlready == false) {
        m_openvrPaths.external_drivers.push_back(exeDir.string());

        auto vrPath = platform::getSteamvrVrPathsPath().string();
        constexpr glz::opts options {
            .comments = false,
            .error_on_unknown_keys = false,
            .prettify = true,
            .indentation_char = ' ',
            .indentation_width = 4,
            .error_on_missing_keys = false,
            .error_on_const_read = false,
        };
        glz::error_ctx serialiseError = glz::write_file_json<options>(m_openvrPaths, vrPath.c_str(), std::string {});
        if (serialiseError.ec != glz::error_code::none) {
            // FUCK
            LOG_WARNING("Failed to write openvrpaths file \"{0}\". {1}", vrPath, serialiseError.custom_error_message);
            return false;
        }
        return true;
    }

    return true;
}

}