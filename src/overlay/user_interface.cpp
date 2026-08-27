#include "user_interface.h"
#include "base_station_management.h"
#include "calibration.h"
#include "configuration.h"
#include "constants.h"
#include "imgui.h"
#include "imgui_extensions.h"
#include "localisation.h"
#include "log.h"
#include "util.h"

namespace spacecal {

std::string g_licenses_text;
UserInterfaceState_t g_state = {};

namespace Colors {
    ImVec4 Amber(0.82f, 0.53f, 0.44f, 1.0f);
    ImVec4 Yellow(0.92f, 0.8f, 0.55f, 1.0f);
    ImVec4 Green(0.64f, 0.75f, 0.55f, 1.0f);
    ImVec4 Purple(0.71f, 0.56f, 0.68f, 1.0f);
    ImVec4 Gray(0.37f, 0.51f, 0.67f, 1.0f);
}

inline std::string getTrackingSystemFriendlyName(const std::string& szTrackingSystemName)
{

    // maps tracking system name to a unique entry per tracking device
    if (szTrackingSystemName == "lighthouse") {
        return "SteamVR Tracking"; // @NOTE: should this be localised?
    }
    if (szTrackingSystemName == "vrlink") {
        return "Steam Link"; // @NOTE: should this be localised?
    }
    if (szTrackingSystemName == "cv") {
        return "Steam Frame"; // @NOTE: should this be localised?
    }
    if (szTrackingSystemName == "oculus") {
        // @TODO: if vd then set as virtual desktop
        // return "Virtual Desktop"; // @NOTE: should this be localised?
    }

    return szTrackingSystemName;
}

inline void buildVersionInfo()
{
    ImGui::SetNextWindowPos(ImVec2(10.0f, ImGui::GetWindowHeight() - ImGui::GetFrameHeightWithSpacing()));
    ImGui::BeginChild("spacecal_version_box", ImVec2(ImGui::GetWindowWidth() - 20.0f, ImGui::GetFrameHeightWithSpacing() * 2), ImGuiChildFlags_None);
    if (g_state.bIsRunningInOverlay) {
        ImGui::TextUnformatted(LOCALE_FORMAT("app_title_vr", "Space Calibrator Nova", SPACECAL_VERSION_STRING).c_str()); // Space Calibrator 2.0.0 - close VR overlay to use mouse
    } else {
        ImGui::TextUnformatted(LOCALE_FORMAT("app_title", "Space Calibrator Nova", SPACECAL_VERSION_STRING).c_str()); // Space Calibrator 2.0.0
    }
    ImGui::EndChild();
}

inline void buildDeviceSelection(spacecal::TrackingSystemCalibration& calibration, spacecal::CalibrationDevice& device)
{
    vr::TrackedDeviceIndex_t selected = device.deviceId;

    // check if the selected device was disconnected or is not present
    if (selected != vr::k_unTrackedDeviceIndexInvalid) {
        bool matched = false;
        for (size_t i = 0; i < vr::k_unMaxTrackedDeviceCount; i++) {
            auto vrDevice = VRState::getInstance()->getVrDevice(i);
            if (!vrDevice.bIsConnected)
                continue;
            if (vrDevice.szTrackingSystemId != device.trackingSystem)
                continue;
            if (vrDevice.eDeviceClass == vr::ETrackedDeviceClass::TrackedDeviceClass_TrackingReference)
                continue;

            if (selected == vrDevice.dwDeviceIndex) {
                matched = true;
                break;
            }
        }

        if (!matched) {
            // Device is no longer present.
            selected = vr::k_unTrackedDeviceIndexInvalid;
        }
    }

    bool standby = calibration.state == CalibrationState::CONTINUOUS_IDLE;

    // select the left controller, or the first device that makes sense to select
    if (selected == vr::k_unTrackedDeviceIndexInvalid && !standby) {
        for (size_t i = 0; i < vr::k_unMaxTrackedDeviceCount; i++) {
            auto vrDevice = VRState::getInstance()->getVrDevice(i);
            if (!vrDevice.bIsConnected)
                continue;
            if (vrDevice.szTrackingSystemId != device.trackingSystem)
                continue;
            if (vrDevice.eDeviceClass == vr::ETrackedDeviceClass::TrackedDeviceClass_TrackingReference)
                continue;

            if (vrDevice.eControllerRole == vr::ETrackedControllerRole::TrackedControllerRole_LeftHand) {
                selected = vrDevice.dwDeviceIndex;
                break;
            }
        }

        if (selected == vr::k_unTrackedDeviceIndexInvalid) {
            for (size_t i = 0; i < vr::k_unMaxTrackedDeviceCount; i++) {
                auto vrDevice = VRState::getInstance()->getVrDevice(i);
                if (!vrDevice.bIsConnected)
                    continue;
                if (vrDevice.szTrackingSystemId != device.trackingSystem)
                    continue;
                if (vrDevice.eDeviceClass == vr::ETrackedDeviceClass::TrackedDeviceClass_Invalid)
                    continue;
                if (vrDevice.eDeviceClass == vr::ETrackedDeviceClass::TrackedDeviceClass_TrackingReference)
                    continue;

                selected = vrDevice.dwDeviceIndex;
                break;
            }
        }
    }

    uint64_t iterator = 0;
    if (selected == vr::k_unTrackedDeviceIndexInvalid && standby) {
        bool present = false;
        for (size_t i = 0; i < vr::k_unMaxTrackedDeviceCount; i++) {
            auto vrDevice = VRState::getInstance()->getVrDevice(i);
            if (!vrDevice.bIsConnected)
                continue;
            if (vrDevice.szTrackingSystemId != device.trackingSystem)
                continue;
            if (vrDevice.eDeviceClass == vr::ETrackedDeviceClass::TrackedDeviceClass_TrackingReference)
                continue;

            if (device.deviceModel != vrDevice.szModel)
                continue;
            if (device.deviceSerialNumber != vrDevice.szSerial)
                continue;

            present = true;
            break;
        }

        if (!present) {
            auto ghostLabel = fmt::format("< {} | {} >", device.deviceModel, device.deviceSerialNumber);
            std::string uniqueId = fmt::format("{}_pass0_{}", ghostLabel, iterator);
            iterator++;
            ImGui::BeginDisabled();
            ImGui::PushStyleColor(ImGuiCol_Header, ImVec4(0.75f, 0.38f, 0.42f, 0.20f));
            ImGui::PushID(uniqueId.c_str());
            ImGui::Selectable(ghostLabel.c_str(), true);
            ImGui::PopID();
            ImGui::PopStyleColor();
            ImGui::EndDisabled();
        }
    }

    iterator = 0;

    for (size_t i = 0; i < vr::k_unMaxTrackedDeviceCount; i++) {
        auto vrDevice = VRState::getInstance()->getVrDevice(i);
        if (!vrDevice.bIsConnected)
            continue;
        if (vrDevice.szTrackingSystemId != device.trackingSystem)
            continue;
        if (vrDevice.eDeviceClass == vr::ETrackedDeviceClass::TrackedDeviceClass_TrackingReference)
            continue;

        auto label = fmt::format("{} | {}", vrDevice.szModel, vrDevice.szSerial);
        std::string uniqueId = fmt::format("{}_pass1_{}", label, iterator);
        iterator++;
        ImGui::PushID(uniqueId.c_str());
        // @TODO: somehow i need to add icons to this
        if (ImGui::Selectable(label.c_str(), selected == vrDevice.dwDeviceIndex)) {
            selected = vrDevice.dwDeviceIndex;
        }
        ImGui::PopID();
    }

    if (selected != device.deviceId && selected < vr::k_unMaxTrackedDeviceCount) {
        auto vrDevice = VRState::getInstance()->getVrDevice(selected);
        device.deviceId = selected;
        device.trackingSystem = vrDevice.szTrackingSystemId;
        device.deviceModel = vrDevice.szModel;
        device.deviceSerialNumber = vrDevice.szSerial;
    }
}

inline void buildDeviceSelection(spacecal::TrackingSystemCalibration& calibration)
{
    if (VRState::getInstance()->getTrackingSystemCount() == 0) {
        ImGui::TextWrapped(LOCALE_GET("tracking_system_no_systems").c_str()); // No tracked devices present. Please turn on a device to continue.
        return;
    }

    ImGuiStyle& style = ImGui::GetStyle();
    ImVec2 paneSize(
        (ImGui::GetContentRegionAvail().x - style.ItemSpacing.x) / 2.0f,
        ImGui::GetTextLineHeightWithSpacing() * 5.0f + style.ItemSpacing.y * 8.0f + ImGui::GetTextLineHeight() * 3.0f + style.FramePadding.y * 2.0f);
    float textWrapWidth = paneSize.x - (style.WindowPadding.x * 2.0f);
    std::string refDesc = LOCALE_GET("reference_device_description");
    std::string targetDesc = LOCALE_GET("target_device_description");

    float refDescHeight = ImGui::CalcTextSize(refDesc.c_str(), nullptr, false, textWrapWidth).y;
    float targetDescHeight = ImGui::CalcTextSize(targetDesc.c_str(), nullptr, false, textWrapWidth).y;
    float maxDescHeight = std::max(refDescHeight, targetDescHeight);

    ImGui::BeginCard("left device pane", paneSize, ImGuiChildFlags_Borders);
    {
        ImGui::TextHeading(LOCALE_GET("reference_device").c_str());
        float descStartY = ImGui::GetCursorPosY();
        ImGui::TextWrappedDisabled(refDesc.c_str());
        ImGui::SetCursorPosY(descStartY + maxDescHeight + style.ItemSpacing.y);

        ImGui::PushItemWidth(ImGui::GetContentRegionAvail().x);
        std::string refPreview = calibration.referenceDevice.trackingSystem.empty() ? LOCALE_GET("select_reference_device") : getTrackingSystemFriendlyName(calibration.referenceDevice.trackingSystem);
        if (ImGui::BeginCombo("##ReferenceTrackingSystem", refPreview.c_str())) {
            for (size_t i = 0; i < VRState::getInstance()->getTrackingSystemCount(); i++) {
                const std::string& szTrackingSystemName = VRState::getInstance()->getTrackingSystem(i);
                const std::string szFriendlyName = getTrackingSystemFriendlyName(szTrackingSystemName);

                bool isSelected = (calibration.referenceDevice.trackingSystem == szTrackingSystemName);
                if (ImGui::Selectable(szFriendlyName.c_str(), isSelected)) {
                    calibration.referenceDevice.trackingSystem = szTrackingSystemName;
                    // if ref space is now the target space, target should be unset
                    // @TODO: should we pick the first tracking system that makes sense instead?
                    if (calibration.referenceDevice.trackingSystem == calibration.targetDevice.trackingSystem)
                        calibration.targetDevice.trackingSystem = "";
                }

                if (isSelected)
                    ImGui::SetItemDefaultFocus();
            }
            ImGui::EndCombo();
        }

        buildDeviceSelection(calibration, calibration.referenceDevice);
        ImGui::PopItemWidth();
    }
    ImGui::EndCard();

    ImGui::SameLine();

    ImGui::BeginCard("right device pane", paneSize, ImGuiChildFlags_Borders);
    {
        ImGui::TextHeading(LOCALE_GET("target_device").c_str());
        float descStartY = ImGui::GetCursorPosY();
        ImGui::TextWrappedDisabled(targetDesc.c_str());
        ImGui::SetCursorPosY(descStartY + maxDescHeight + style.ItemSpacing.y);

        ImGui::PushItemWidth(ImGui::GetContentRegionAvail().x);
        // target tracking system list is tracking system list EXCLUDING reference tracking system. one may NOT calibrate two devices of the same tracking system!
        std::string targetPreview = calibration.targetDevice.trackingSystem.empty() ? LOCALE_GET("select_target_device") : getTrackingSystemFriendlyName(calibration.targetDevice.trackingSystem);
        if (ImGui::BeginCombo("##TargetTrackingSystem", targetPreview.c_str())) {
            for (size_t i = 0; i < VRState::getInstance()->getTrackingSystemCount(); i++) {
                const std::string& szTrackingSystemName = VRState::getInstance()->getTrackingSystem(i);

                // target cannot be the reference space
                if (szTrackingSystemName == calibration.referenceDevice.trackingSystem)
                    continue;

                const std::string szFriendlyName = getTrackingSystemFriendlyName(szTrackingSystemName);
                bool isSelected = (calibration.targetDevice.trackingSystem == szTrackingSystemName);

                if (ImGui::Selectable(szFriendlyName.c_str(), isSelected)) {
                    calibration.targetDevice.trackingSystem = szTrackingSystemName;
                }

                if (isSelected)
                    ImGui::SetItemDefaultFocus();
            }
            ImGui::EndCombo();
        }

        buildDeviceSelection(calibration, calibration.targetDevice);
        ImGui::PopItemWidth();
    }
    ImGui::EndCard();

    if (ImGui::Button(LOCALE_GET("identify_selected_devices").c_str(), ImVec2(paneSize.x, ImGui::GetTextLineHeightWithSpacing() + 4.0f))) {
        // @TODO: non-blocking ; blocks for 500ms rn :(
        for (size_t i = 0; i < 100; ++i) {
            VRState::getInstance()->identifyDevice(calibration.targetDevice.deviceId);
            VRState::getInstance()->identifyDevice(calibration.referenceDevice.deviceId);
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
    }

    ImGui::SameLine();

    if (ImGui::Button(LOCALE_GET("identify_auto_detect_devices").c_str(), ImVec2(paneSize.x, ImGui::GetTextLineHeightWithSpacing() + 4.0f))) {
        calibration.autoDetectDevices();
    }
}

inline bool isSpacecalRunningStateOk()
{
    return VRState::getInstance()->isSteamVrAvailable() && CalibrationManager::getInstance()->getIpcClient().IsConnected();
}

inline void buildCalibrationCommonControls(spacecal::TrackingSystemCalibration& calibration)
{
    auto& io = ImGui::GetIO();
    ImGuiStyle& style = ImGui::GetStyle();
    ImGui::NewLine();

    float width = ImGui::GetContentRegionAvail().x - style.FramePadding.x * 2.0f;
    float scale = 1.0f / 2.0f;
    if (calibration.isValidCalibration()) {
        width -= style.FramePadding.x * 2.0f;
        scale = (1.0f / 4.0f);
    }

    ImGui::BeginDisabled(!calibration.devicesAreValid());

    if (ImGui::IconButton(ICON_MS_PLAY_ARROW, LOCALE_GET("calibration_action_start").c_str(), ImVec2(width * scale, ImGui::GetTextLineHeight() * 2))) {
        calibration.start();
    }

    ImGui::SameLine();
    if (ImGui::IconButton(ICON_MS_SYNC, LOCALE_GET("calibration_action_continuous").c_str(), ImVec2(width * scale, ImGui::GetTextLineHeight() * 2))) {
        calibration.startContinuous();
    }

    ImGui::EndDisabled();

    if (calibration.isValidCalibration()) {
        ImGui::SameLine();
        if (ImGui::IconButton(ICON_MS_CLOSE, LOCALE_GET("calibration_action_clear").c_str(), ImVec2(width * scale, ImGui::GetTextLineHeight() * 2))) {
            calibration.reset();
        }

        ImGui::SameLine();
        if (ImGui::IconButton(ICON_MS_EDIT, LOCALE_GET("calibration_action_edit").c_str(), ImVec2(width * scale, ImGui::GetTextLineHeight() * 2))) {
            calibration.state = CalibrationState::EDITING;
        }
    }

    ImGui::NewLine();
    auto speed = calibration.calibrationSpeed;

    ImGui::PushStyleVar(ImGuiStyleVar_CellPadding, ImVec2(ImGui::GetStyle().CellPadding.x + ImGui::GetStyle().ItemSpacing.x, ImGui::GetStyle().CellPadding.y));
    if (ImGui::BeginTable("SpeedSelectionTable", 4, ImGuiTableFlags_SizingFixedFit | ImGuiTableFlags_PadOuterX)) {
        ImGui::TableSetupColumn("Label", ImGuiTableColumnFlags_WidthStretch);
        ImGui::TableSetupColumn("Fast", ImGuiTableColumnFlags_WidthFixed);
        ImGui::TableSetupColumn("Slow", ImGuiTableColumnFlags_WidthFixed);
        ImGui::TableSetupColumn("VerySlow", ImGuiTableColumnFlags_WidthFixed);

        ImGui::TableNextRow();

        ImGui::TableNextColumn();
        ImGui::TextUnformatted(LOCALE_GET("calibration_speed").c_str());
        ImGui::TextWrappedDisabled(LOCALE_GET("calibration_speed_description").c_str());

        ImGui::TableNextColumn();
        if (ImGui::RadioButton(LOCALE_GET("calibration_speed_fast").c_str(), speed == CalibrationSpeed::FAST)) {
            calibration.calibrationSpeed = CalibrationSpeed::FAST;
            calibration.clearSamples();
        }

        ImGui::TableNextColumn();
        if (ImGui::RadioButton(LOCALE_GET("calibration_speed_slow").c_str(), speed == CalibrationSpeed::SLOW)) {
            calibration.calibrationSpeed = CalibrationSpeed::SLOW;
            calibration.clearSamples();
        }

        ImGui::TableNextColumn();
        if (ImGui::RadioButton(LOCALE_GET("calibration_speed_very_slow").c_str(), speed == CalibrationSpeed::VERY_SLOW)) {
            calibration.calibrationSpeed = CalibrationSpeed::VERY_SLOW;
            calibration.clearSamples();
        }

        ImGui::EndTable();
    }
    ImGui::PopStyleVar();
    ImGui::NewLine();

    // continuous settings
    if (calibration.isContinuousCalibration()) {
        if (ImGui::Checkbox(LOCALE_GET("continuous_hide_tracker").c_str(), &calibration.hideContinuousTracker)) {
            CalibrationManager::getInstance()->saveConfig();
        }

        if (g_state.bIsSettingsAdvanced) {
            if (ImGui::CheckboxWithDescription(LOCALE_GET("continuous_relative_calibration").c_str(), &calibration.isRelativeCalibration, LOCALE_GET("continuous_relative_calibration_description").c_str())) {
                calibration.forceNextCalibration();
            }
        }
    }
    if (g_state.bIsSettingsAdvanced) {
        ImGui::CheckboxWithDescription(LOCALE_GET("settings_calibrate_motion_vectors").c_str(), &calibration.calibrateMotionVectors, LOCALE_GET("settings_calibrate_motion_vectors_description").c_str());
        ImGui::CheckboxWithDescription(LOCALE_GET("settings_autofix_playspace_jumps").c_str(), &calibration.autoFixPlayspaceJumps, LOCALE_GET("settings_autofix_playspace_jumps_description").c_str());
        ImGui::CheckboxWithDescription(LOCALE_GET("settings_enforce_minimum_rotation_variance").c_str(), &calibration.enforceMinimumRotationVariance, LOCALE_GET("settings_enforce_minimum_rotation_variance_description").c_str());
    }
}

inline void drawTroubleshootView()
{
    if (!VRState::getInstance()->isSteamVrAvailable()) {
        ImGui::TextWrapped("%s", LOCALE_GET("steamvr_unavailable").c_str());
        vr::EVRInitError eVrErr = VRState::getInstance()->getVrInitError();
        switch (eVrErr) {
        case vr::EVRInitError::VRInitError_Driver_WirelessHmdNotConnected:
            ImGui::TextWrapped("%s", LOCALE_GET("vr_troubleshooting_connect_steamlink").c_str());
            break;
        case vr::EVRInitError::VRInitError_Init_HmdNotFound:
            ImGui::TextWrapped("%s", LOCALE_GET("vr_troubleshooting_hmd_not_found").c_str());
            break;
        default:
            // need to pass by ref, cant pass as literal value
            uint32_t dwVrErr = (uint32_t)eVrErr;
            ImGui::TextWrapped(LOCALE_FORMAT("vr_troubleshooting_generic", dwVrErr, eVrErr).c_str());
            break;
        }
    } else if (!CalibrationManager::getInstance()->getIpcClient().IsConnected()) {
        ImGui::TextWrapped("%s", LOCALE_GET("ipc_unavailable").c_str());
    } else {
        ImGui::TextWrapped("%s", LOCALE_GET("error_unknown_catastrophic").c_str());
    }
}

inline void buildLocaleSelector()
{
    ImGui::TextUnformatted(LOCALE_GET("settings_locale").c_str());
    ImGui::SameLine();

    std::string& szLocale = ConfigurationManager::getInstance()->getConfiguration()->ui_locale;
    Locale eLocale = LocalisationManager::getInstance()->getLocaleFromRegionString(szLocale);
    std::string selectedLocaleNativeName = LocalisationManager::getInstance()->getNativeTongueLocaleName(eLocale);

    std::string selectedLocale = fmt::format("languages_{}", szLocale);
    std::string displaySelectedLocaleName = fmt::format("{} ({})", LOCALE_GET(selectedLocale), selectedLocaleNativeName);

    if (ImGui::BeginCombo("##LocalePicker", displaySelectedLocaleName.c_str())) {
        for (size_t i = 0; i < (size_t)Locale::Count; i++) {
            const Locale eThisLocale = (Locale)i;
            std::string thisLocaleId = LocalisationManager::getInstance()->getLocaleAsRegionString(eThisLocale);
            std::string thisLocaleString = fmt::format("languages_{}", thisLocaleId);

            std::string friendlyLocaleString = LOCALE_GET(thisLocaleString);
            std::string localeNativeName = LocalisationManager::getInstance()->getNativeTongueLocaleName(eThisLocale);

            // German (Deustche)
            std::string displayLocaleName = fmt::format("{} ({})", friendlyLocaleString, localeNativeName);

            bool isSelected = (eLocale == eThisLocale);

            if (ImGui::Selectable(displayLocaleName.c_str(), isSelected)) {
                szLocale = thisLocaleId;
                // apply locale to UI
                LocalisationManager::getInstance()->setLocale(eThisLocale);
                ConfigurationManager::getInstance()->saveConfiguration();
            }

            if (isSelected)
                ImGui::SetItemDefaultFocus();
        }
        ImGui::EndCombo();
    }
}

struct PlotUiState {
    double referenceTime = 0.0;
    bool wasHoveredThisFrame = false;

    std::vector<double> calAppliedTimeBuffer;
    std::vector<double> calByRelPoseTimeBuffer;
    std::vector<int> currentGraphIndexes;
    ImPlotColormap axisVarianceColormap = 0;
    bool colormapInitialized = false;
};

PlotUiState g_PlotUiState;

constexpr size_t k_INVALID_OFFSET = (size_t)(-1);

enum class SeriesType {
    Scalar,
    Vector3d,
    CustomFunc
};

struct GraphConfig {
    const char* menuName = nullptr;
    const char* menuId = nullptr;
    const char* szAxisUnits = nullptr;
    SeriesType type = SeriesType::Scalar;
    size_t offset = k_INVALID_OFFSET;
    /**
     * weird graph rendering (eg fills or a constant line). this renders inside an active implot context so dont call implot::begin or end
     *
     * @param render_sample_count   the amount of samples we are rendering for this graph
     * @param baseSpec              implot rendering params
     */
    void (*customRenderFunc)(const CalibrationErrorMetrics& metrics, int render_sample_count, const ImPlotSpec& baseSpec) = nullptr;
    double yLimit = 0.0;
    double xScale = 1.0; // used for eg zooming in to specific graphs like velocity
    ImPlotAxisFlags yFlags = ImPlotAxisFlags_AutoFit | ImPlotAxisFlags_RangeFit;

    // ptr to next graph config to draw in the same plot
    const GraphConfig* pNext = nullptr;
};

void addApplyTicks()
{
    ImPlotSpec defaultSpec;

    if (g_PlotUiState.calAppliedTimeBuffer.empty()) {
        double x = -HUGE_VAL;
        ImPlot::PlotInfLines("##CalibrationAppliedTime", &x, 1, defaultSpec);
    } else {
        ImPlot::PlotInfLines("##CalibrationAppliedTime", g_PlotUiState.calAppliedTimeBuffer.data(), static_cast<int>(g_PlotUiState.calAppliedTimeBuffer.size()), defaultSpec);
    }

    if (g_PlotUiState.calByRelPoseTimeBuffer.empty()) {
        double x = -HUGE_VAL;
        ImPlot::PlotInfLines("##CalibrationAppliedTimeByRelPose", &x, 1, defaultSpec);
    } else {
        ImPlot::PlotInfLines("##CalibrationAppliedTimeByRelPose", g_PlotUiState.calByRelPoseTimeBuffer.data(), static_cast<int>(g_PlotUiState.calByRelPoseTimeBuffer.size()), defaultSpec);
    }

    if (ImPlot::IsPlotHovered()) {

        ImPlotSpec tagSpec;
        tagSpec.LineColor = ImVec4(0.5f, 0.5f, 1.0f, 1.0f);
        float lastMouseX = (float)ImPlot::GetPlotMousePos().x;
        ImPlot::PlotInfLines("##TagLine", &lastMouseX, 1, tagSpec);
        g_PlotUiState.wasHoveredThisFrame = true;
    }
}

void drawGraphSeriesChain(const GraphConfig* cfg, const char* pMetricsAddress, const CalibrationErrorMetrics& metrics, int count)
{
    ImPlotSpec spec;

    static thread_local std::vector<TimeSeries<double>::TimeSeriesSample> scalarScratch;
    static thread_local std::vector<TimeSeries<Eigen::Vector3d>::TimeSeriesSample> vec3Scratch;

    while (cfg != nullptr) {
        if (cfg->type == SeriesType::CustomFunc && cfg->customRenderFunc != nullptr) {
            cfg->customRenderFunc(metrics, count, spec);
        } else if (cfg->type == SeriesType::Scalar) {
            const auto& ts = *reinterpret_cast<const TimeSeries<double>*>(pMetricsAddress + cfg->offset);
            if (ts.size() > 0) {
                scalarScratch.clear();
                scalarScratch.reserve(ts.size());
                for (int i = 0; i < ts.size(); i++) {
                    scalarScratch.push_back({ g_PlotUiState.referenceTime - ts[i].time, ts[i].value });
                }
                spec.Stride = static_cast<int>(sizeof(scalarScratch[0]));
                ImPlot::PlotLine(cfg->menuName, &scalarScratch[0].time, &scalarScratch[0].value, static_cast<int>(scalarScratch.size()), spec);
            }
        } else if (cfg->type == SeriesType::Vector3d) {
            const auto& ts = *reinterpret_cast<const TimeSeries<Eigen::Vector3d>*>(pMetricsAddress + cfg->offset);
            if (ts.size() > 0) {
                vec3Scratch.clear();
                vec3Scratch.reserve(ts.size());
                for (int i = 0; i < ts.size(); i++) {
                    vec3Scratch.push_back({ g_PlotUiState.referenceTime - ts[i].time, ts[i].value });
                }
                spec.Stride = static_cast<int>(sizeof(vec3Scratch[0]));
                std::string labelX = fmt::format("{}.X", cfg->menuName);
                std::string labelY = fmt::format("{}.Y", cfg->menuName);
                std::string labelZ = fmt::format("{}.Z", cfg->menuName);

                ImPlot::PlotLine(labelX.c_str(), &vec3Scratch[0].time, &vec3Scratch[0].value.x(), static_cast<int>(vec3Scratch.size()), spec);
                ImPlot::PlotLine(labelY.c_str(), &vec3Scratch[0].time, &vec3Scratch[0].value.y(), static_cast<int>(vec3Scratch.size()), spec);
                ImPlot::PlotLine(labelZ.c_str(), &vec3Scratch[0].time, &vec3Scratch[0].value.z(), static_cast<int>(vec3Scratch.size()), spec);
            }
        }
        cfg = cfg->pNext;
    }
}

void drawComposedPlot(double timeSpan, const GraphConfig* rootCfg, const CalibrationErrorMetrics& metrics)
{
    if (ImPlot::BeginPlot(rootCfg->menuId, ImVec2(-1, 0), ImPlotFlags_None)) {
        ImPlot::SetupAxes(nullptr, rootCfg->szAxisUnits, 0, rootCfg->yFlags);
        ImPlot::SetupAxisLimits(ImAxis_X1, timeSpan * rootCfg->xScale, 0, ImGuiCond_Always);
        ImPlot::SetupAxisLimits(ImAxis_Y1, 0, rootCfg->yLimit, ImGuiCond_Appearing);

        addApplyTicks();

        const char* pMetricsAddress = reinterpret_cast<const char*>(&metrics);
        int maxCount = metrics.axisIndependence.size();

        drawGraphSeriesChain(rootCfg, pMetricsAddress, metrics, maxCount);

        ImPlot::EndPlot();
    }
}

void drawAxisVariancePlot(const CalibrationErrorMetrics& metrics, int render_sample_count, const ImPlotSpec& baseSpec)
{
    if (!g_PlotUiState.colormapInitialized) {
        g_PlotUiState.colormapInitialized = true;
        ImVec4 colors[] = { ImPlot::GetColormapColor(0), ImPlot::GetColormapColor(1), { 1, 0, 0, 1 }, { 0, 1, 0, 1 }, { 0.5, 0.5, 0.5, 1 } };
        g_PlotUiState.axisVarianceColormap = ImPlot::AddColormap("AxisVarianceColormap", colors, std::size(colors));
    }

    const auto& ts = metrics.axisIndependence;
    if (ts.size() <= 0)
        return;

    static thread_local std::vector<TimeSeries<double>::TimeSeriesSample> scratch;
    scratch.clear();
    scratch.reserve(ts.size());
    for (int i = 0; i < ts.size(); i++) {
        scratch.push_back({ g_PlotUiState.referenceTime - ts[i].time, ts[i].value });
    }

    ImPlot::PushColormap(g_PlotUiState.axisVarianceColormap);
    const auto& rawData = ts.data();
    int stride = (int)(sizeof(scratch[0]));

    std::vector<double> threshLine(render_sample_count, k_MAX_AXIS_VARIANCE_THRESHOLD);
    std::vector<double> zeroLine(render_sample_count, 0.0);

    ImPlotSpec lowSpec = baseSpec;
    lowSpec.LineColor = ImVec4(1, 0, 0, 1);
    lowSpec.FillAlpha = 0.5f;
    lowSpec.Stride = stride;

    ImPlotSpec highSpec = baseSpec;
    highSpec.LineColor = ImVec4(0, 1, 0, 1);
    highSpec.FillAlpha = 0.5f;
    highSpec.Stride = stride;

    ImPlot::PlotShaded("##VarianceLow", &scratch[0].time, &scratch[0].value, zeroLine.data(), (int)zeroLine.size(), lowSpec);
    ImPlot::PlotShaded("##VarianceHigh", &scratch[0].time, &scratch[0].value, threshLine.data(), (int)threshLine.size(), highSpec);
    ImPlot::PopColormap(1);
}

// these are defomed outside the array for the linked list to work

// axis variance needs custom func for the draw line
static const GraphConfig g_varLineNode { "Datapoint", "##Axis variance", nullptr, SeriesType::Scalar, offsetof(CalibrationErrorMetrics, axisIndependence), nullptr, 0.003, 1.0, ImPlotAxisFlags_AutoFit | ImPlotAxisFlags_RangeFit, /* .pNext = */ nullptr };
static const GraphConfig g_varianceRoot { "Axis Variance", "##Axis variance", nullptr, SeriesType::CustomFunc, 0, drawAxisVariancePlot, 0.003, 1.0, ImPlotAxisFlags_AutoFit | ImPlotAxisFlags_RangeFit, /* .pNext = */ &g_varLineNode };

// temp for seeing how bad latency really is
static const GraphConfig g_referencePoseVelocity { "Reference Velocity", "##hmdVelocity", "m/s", SeriesType::Scalar, offsetof(CalibrationErrorMetrics, pose_ref_velocity_mag), nullptr, 0.003, 0.05, ImPlotAxisFlags_AutoFit | ImPlotAxisFlags_RangeFit, /* .pNext = */ nullptr };
static const GraphConfig g_targetPoseVelocity { "Target Velocity", "##targetVelocity", "m/s", SeriesType::Scalar, offsetof(CalibrationErrorMetrics, pose_tgt_velocity_mag), nullptr, 0.003, 0.05, ImPlotAxisFlags_AutoFit | ImPlotAxisFlags_RangeFit, /* .pNext = */ &g_referencePoseVelocity };

const GraphConfig g_graph_data[] = {
    g_varianceRoot,
    g_targetPoseVelocity,
    { "Offset: Raw Computed", "##posOffsetRawComputed", "mm", SeriesType::Vector3d, offsetof(CalibrationErrorMetrics, posOffset_rawComputed), nullptr, 200.0 },
    { "Offset: Current Calibration", "##posOffsetCurrentCal", "mm", SeriesType::Vector3d, offsetof(CalibrationErrorMetrics, posOffset_currentCal), nullptr, 200.0 },
    { "Offset: RMS error", "##posOffsetRMSError", "mm", SeriesType::Vector3d, offsetof(CalibrationErrorMetrics, posOffset_rmsError), nullptr, 200.0 },
    { "Retargeting RMS Error", "##rmsError", "ms", SeriesType::Scalar, offsetof(CalibrationErrorMetrics, rmsError), nullptr, 200.0 },
    { "Processing time", "##Computation Time", "ms", SeriesType::Scalar, offsetof(CalibrationErrorMetrics, computationTime), nullptr, 200.0 },
};

constexpr size_t k_NUM_GRAPHS = ARRAY_SIZE(g_graph_data);

void draw_error_graphs(const std::string& targetDeviceName, double timeSpan, double currentTimestamp, const CalibrationErrorMetrics& metrics)
{
    constexpr int k_ROWS = 2;
    constexpr int k_COLS = (k_NUM_GRAPHS + k_ROWS - 1) / k_ROWS;
    constexpr int k_TOTAL_PLOTS = k_ROWS * k_COLS;

    if (g_PlotUiState.currentGraphIndexes.size() < k_TOTAL_PLOTS) {
        while (g_PlotUiState.currentGraphIndexes.size() < k_TOTAL_PLOTS) {
            g_PlotUiState.currentGraphIndexes.push_back(static_cast<int>(g_PlotUiState.currentGraphIndexes.size()) % (int)k_NUM_GRAPHS);
        }
    }

    g_PlotUiState.wasHoveredThisFrame = false;
    auto avail = ImGui::GetContentRegionAvail();

    ImGui::PushStyleColor(ImGuiCol_TableRowBg, ImVec4(0, 0, 0, 0));
    ImGui::PushStyleColor(ImGuiCol_TableRowBgAlt, ImVec4(0, 0, 0, 0));
    ImPlot::PushStyleColor(ImPlotCol_FrameBg, ImVec4(0, 0, 0, 0));

    std::string childId = fmt::format("##MetricsPanel_{}", targetDeviceName);
    if (!ImGui::BeginChild(childId.c_str(), avail, false, ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_NoFocusOnAppearing)) {
        ImGui::EndChild();
        ImPlot::PopStyleColor(1);
        ImGui::PopStyleColor(2);
        return;
    }

    std::string tableId = fmt::format("##MetricsTable_{}", targetDeviceName);
    if (!ImGui::BeginTable(tableId.c_str(), k_COLS, ImGuiTableFlags_RowBg)) {
        ImGui::EndChild();
        ImPlot::PopStyleColor(1);
        ImGui::PopStyleColor(2);
        return;
    }

    g_PlotUiState.referenceTime = currentTimestamp;

    g_PlotUiState.calAppliedTimeBuffer.clear();
    g_PlotUiState.calByRelPoseTimeBuffer.clear();
    for (const auto& sample : metrics.calibrationApplied.data()) {
        if (sample.value)
            g_PlotUiState.calAppliedTimeBuffer.push_back(g_PlotUiState.referenceTime - sample.time);
        else
            g_PlotUiState.calByRelPoseTimeBuffer.push_back(g_PlotUiState.referenceTime - sample.time);
    }

    const char* pMetricsAddress = reinterpret_cast<const char*>(&metrics);

    for (int r = 0; r < k_ROWS; r++) {
        ImGui::TableNextRow();
        for (int c = 0; c < k_COLS; c++) {
            int idx = r * k_COLS + c;
            if (idx >= k_NUM_GRAPHS) {
                continue;
            }

            ImGui::TableSetColumnIndex(c);
            ImGui::PushID(idx);

            ImGui::SetNextItemWidth(ImGui::GetColumnWidth());
            if (ImGui::BeginCombo("", g_graph_data[g_PlotUiState.currentGraphIndexes[idx]].menuName, 0)) {
                for (int j = 0; j < k_NUM_GRAPHS; j++) {
                    bool isSelected = (j == g_PlotUiState.currentGraphIndexes[idx]);
                    if (ImGui::Selectable(g_graph_data[j].menuName, isSelected)) {
                        g_PlotUiState.currentGraphIndexes[idx] = j;
                    }
                    if (isSelected)
                        ImGui::SetItemDefaultFocus();
                }
                ImGui::EndCombo();
            }

            const auto& rootCfg = g_graph_data[g_PlotUiState.currentGraphIndexes[idx]];
            drawComposedPlot(timeSpan, &rootCfg, metrics);

            ImGui::PopID();
        }
    }

    ImGui::EndTable();
    ImGui::EndChild();

    ImPlot::PopStyleColor(1);
    ImGui::PopStyleColor(2);
}

void page_calibration(double currentTime)
{
    ImGui::TextTitle("%s", LOCALE_GET("calibration_title").c_str());
    ImGui::Separator();

    if (!isSpacecalRunningStateOk()) {
        drawTroubleshootView();
    } else {
        const size_t dwNumCalibrations = spacecal::CalibrationManager::getInstance()->getCalibrationCount();

        if (dwNumCalibrations == 0) {
            // @TODO: no calibrations edge case handling
            ImGui::TextTitle("%s", LOCALE_GET("calibration_count_none").c_str());
        }

        for (size_t i = 0; i < dwNumCalibrations; i++) {
            spacecal::TrackingSystemCalibration& calibration = spacecal::CalibrationManager::getInstance()->getCalibration(i);
            ImGui::PushID(fmt::format("calibration_id__{}", i).c_str());

            std::string szCalibrationStatusIcon = "";
            std::string szCalibrationStatusMessage = "";
            std::string szCalibrationStatusDescription = "";
            switch (calibration.state) {
            default:
            case CalibrationState::NONE:
                if (calibration.isValidCalibration()) {
                    szCalibrationStatusIcon = ICON_MS_TASK_ALT;
                    szCalibrationStatusMessage = "calibration_status_standard";
                    szCalibrationStatusDescription = "calibration_status_standard_description";
                } else {
                    szCalibrationStatusIcon = ICON_MS_HELP;
                    szCalibrationStatusMessage = "calibration_status_none";
                    szCalibrationStatusDescription = "calibration_status_none_description";
                }
                break;
            case CalibrationState::EDITING:
                szCalibrationStatusIcon = ICON_MS_EDIT;
                szCalibrationStatusMessage = "calibration_status_edit";
                szCalibrationStatusDescription = "calibration_status_edit_description";
                break;
            case CalibrationState::CONTINUOUS:
            case CalibrationState::CONTINUOUS_IDLE:
            case CalibrationState::AUTO_DETECT_DEVICES_CONTINUOUS:
                szCalibrationStatusIcon = ICON_MS_SYNC;
                szCalibrationStatusMessage = "calibration_status_continuous";
                szCalibrationStatusDescription = "calibration_status_continuous_description";
                break;
            case CalibrationState::AUTO_DETECT_DEVICES_STANDARD:
            case CalibrationState::SAMPLE:
            case CalibrationState::START:
                szCalibrationStatusIcon = ICON_MS_TASK_ALT;
                szCalibrationStatusMessage = "calibration_status_standard";
                szCalibrationStatusDescription = "calibration_status_standard_description";
                break;
            }

            // status card drawing
            {
                ImGui::BeginCard(fmt::format("calibration_status__", i).c_str());

                ImGui::TextHeading(fmt::format("{} {}", szCalibrationStatusIcon, LOCALE_GET(szCalibrationStatusMessage)).c_str());
                ImGui::TextWrappedDisabled(LOCALE_GET(szCalibrationStatusDescription).c_str());

                if (calibration.isContinuousCalibration()) {
                    // continuous is an indeterminate calibration, ie we cant compute a "percentage" for it
                    ImGui::AlignTextToFramePadding();
                    ImGui::TextHeading("%s", LOCALE_GET("calibration_progress").c_str());
                    ImGui::SameLine();
                    ImGui::ProgressBar(-0.25f * (float)ImGui::GetTime(), ImVec2(-FLT_MIN, 0.0f), "");
                } else if (calibration.isCalibrating()) {
                    // standard calibration
                    ImGui::TextWrappedDisabled(LOCALE_GET("calibration_info_move_around_insufficient_samples").c_str());
                    float fCalibrationProgressPercent = calibration.getCalibrationProgress() * 100.0f;
                    ImGui::AlignTextToFramePadding();
                    ImGui::TextHeading("%s", LOCALE_GET("calibration_progress").c_str());
                    ImGui::SameLine();
                    ImGui::ProgressBar(calibration.getCalibrationProgress(), ImVec2(-FLT_MIN, 0), fmt::format("{:.2f}%", fCalibrationProgressPercent).c_str());
                }

                ImGui::EndCard();
            }

            auto hmdMeta = VRState::getInstance()->getHmdMeta();
            if ((g_state.bIsSettingsAdvanced && !g_state.bIgnoreStageTrackingWarning) && hmdMeta.isVirtualDesktopAvailable && /* VRState::getInstance()->isHmdVirtualDesktop() && */ !hmdMeta.VD_stageTrackingEnabled) {
                if (hmdMeta.VD_hmdModel == ipc::protocol::VD_HmdModel_OculusGo || hmdMeta.VD_hmdModel == ipc::protocol::VD_HmdModel_OculusQuest || hmdMeta.VD_hmdModel == ipc::protocol::VD_HmdModel_OculusQuest2 || hmdMeta.VD_hmdModel == ipc::protocol::VD_HmdModel_MetaQuestPro || hmdMeta.VD_hmdModel == ipc::protocol::VD_HmdModel_MetaQuest3 || hmdMeta.VD_hmdModel == ipc::protocol::VD_HmdModel_MetaQuest3S) {

                    ImGui::BeginCardDanger("calibration_no_stage_tracking");
                    ImGui::TextWrapped(LOCALE_GET("calibration_warning_stage_tracking_disabled").c_str());

                    if (g_state.bIsSettingsAdvanced) {
                        if (ImGui::IconButton(ICON_MS_VISIBILITY_OFF, LOCALE_GET("calibration_warning_stage_tracking_disabled_ignore_button").c_str())) {
                            ConfigurationManager::getInstance()->getConfiguration()->ignore_stage_tracking_warning = true;
                            ConfigurationManager::getInstance()->saveConfiguration();
                        }
                    }

                    ImGui::EndCardDanger();
                }
            }

            if (calibration.state == CalibrationState::EDITING) {
                ImGuiStyle& style = ImGui::GetStyle();
                float width = ImGui::GetContentRegionAvail().x / 3.0f - style.FramePadding.x;
                float widthF = width - style.FramePadding.x;

                ImGui::BeginDisabled(calibration.isContinuousCalibration());

                ImGui::TextHeading(LOCALE_GET("edit_calibration_rotation").c_str());

                ImGui::TextWithWidth(LOCALE_GET("edit_calibration_yaw").c_str(), width);
                ImGui::SameLine();
                ImGui::TextWithWidth(LOCALE_GET("edit_calibration_pitch").c_str(), width);
                ImGui::SameLine();
                ImGui::TextWithWidth(LOCALE_GET("edit_calibration_roll").c_str(), width);

                Eigen::Vector3d calibratedRotationEuler = calibration.calibratedRotation.toRotationMatrix().canonicalEulerAngles(2, 1, 0) * (180.0 / EIGEN_PI);

                ImGui::PushItemWidth(widthF);
                ImGui::InputDouble("##Yaw", &calibratedRotationEuler(1), 0.1, 1.0, "%.8f");
                ImGui::SameLine();
                ImGui::InputDouble("##Pitch", &calibratedRotationEuler(2), 0.1, 1.0, "%.8f");
                ImGui::SameLine();
                ImGui::InputDouble("##Roll", &calibratedRotationEuler(0), 0.1, 1.0, "%.8f");

                double rollRad = calibratedRotationEuler(0) * (EIGEN_PI / 180.0);
                double pitchRad = calibratedRotationEuler(2) * (EIGEN_PI / 180.0);
                double yawRad = calibratedRotationEuler(1) * (EIGEN_PI / 180.0);

                calibration.calibratedRotation = (Eigen::AngleAxisd(yawRad, Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd(pitchRad, Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(rollRad, Eigen::Vector3d::UnitX())).normalized();

                ImGui::TextHeading(LOCALE_GET("edit_calibration_position").c_str());

                ImGui::TextWithWidth(LOCALE_GET("edit_calibration_x").c_str(), width);
                ImGui::SameLine();
                ImGui::TextWithWidth(LOCALE_GET("edit_calibration_y").c_str(), width);
                ImGui::SameLine();
                ImGui::TextWithWidth(LOCALE_GET("edit_calibration_z").c_str(), width);

                ImGui::InputDouble("##X", &calibration.calibratedTranslation(0), 1.0, 10.0, "%.8f");
                ImGui::SameLine();
                ImGui::InputDouble("##Y", &calibration.calibratedTranslation(1), 1.0, 10.0, "%.8f");
                ImGui::SameLine();
                ImGui::InputDouble("##Z", &calibration.calibratedTranslation(2), 1.0, 10.0, "%.8f");

                ImGui::EndDisabled();
                ImGui::PopItemWidth();

                ImGui::TextHeading(LOCALE_GET("edit_calibration_scale").c_str());

                ImGui::InputDouble("##Scale", &calibration.calibratedScale, 0.0001, 0.01, "%.8f");

                if (ImGui::Button(LOCALE_GET("save_calibration_profile").c_str(), ImVec2(ImGui::GetContentRegionAvail().x, ImGui::GetTextLineHeight() * 2))) {
                    calibration.state = CalibrationState::NONE;
                    ConfigurationManager::getInstance()->saveConfiguration();
                }
            } else {

                if (calibration.isValidCalibration() && !calibration.isActive && !calibration.devicesAreValid()) {
                    ImGui::BeginCardDanger("calibration_invalid");
                    std::string szTrackingSystemUiName = getTrackingSystemFriendlyName(calibration.referenceDevice.trackingSystem);
                    ImGui::TextWrapped(LOCALE_FORMAT("calibration_error_reference_hmd_missing", szTrackingSystemUiName).c_str());
                    ImGui::EndCardDanger();
                }

                buildDeviceSelection(calibration);
                buildCalibrationCommonControls(calibration);
            }

            ImGui::PopID();
        }
    }
}

void page_graphs(double currentTime)
{
    ImGui::TextTitle("%s", LOCALE_GET("graphs_title").c_str());

    // @TODO: numbering or smth
    spacecal::TrackingSystemCalibration& calibration = spacecal::CalibrationManager::getInstance()->getCalibration(0);
    draw_error_graphs(calibration.referenceDevice.trackingSystem, k_METRIC_HISTORY_TIMESPAN, currentTime, calibration.errorMetrics);
}

void page_settings(double currentTime)
{
    ImGui::TextTitle("%s", LOCALE_GET("settings_title").c_str());

    // advanced settings
    if (ImGui::CheckboxWithDescription(LOCALE_GET("settings_advanced").c_str(), &g_state.bIsSettingsAdvanced, LOCALE_GET("settings_advanced_description").c_str())) {
        ConfigurationManager::getInstance()->getConfiguration()->advanced_settings = g_state.bIsSettingsAdvanced;
        ConfigurationManager::getInstance()->saveConfiguration();
    }

    buildLocaleSelector();

    // base station power management
    ImGui::BeginCard("base_station_power_management");
    {
        ImGui::TextHeading(LOCALE_GET("base_station_power_management").c_str());

        const float fCheckboxTextIndent = ImGui::GetFrameHeight() + ImGui::GetStyle().ItemSpacing.x;
        const char* bsPowerEnableDescKey = g_state.bBaseStationPowerManagementOffModeIsSleep
            ? "base_stations_power_mgmt_enable_description_sleep"
            : "base_stations_power_mgmt_enable_description_standby";

        if (ImGui::CheckboxWithDescription(LOCALE_GET("base_stations_power_mgmt_enable").c_str(), &g_state.bBaseStationPowerManagementEnabled, LOCALE_GET(bsPowerEnableDescKey).c_str())) {
            ConfigurationManager::getInstance()->getConfiguration()->base_stations.auto_power_management_enabled = g_state.bBaseStationPowerManagementEnabled;
            ConfigurationManager::getInstance()->saveConfiguration();
        }

        ImGui::BeginDisabled(!g_state.bBaseStationPowerManagementEnabled);
        ImGui::Indent(fCheckboxTextIndent);
        if (ImGui::CheckboxWithDescription(LOCALE_GET("base_stations_power_mgmt_on_startup").c_str(), &g_state.bBaseStationPowerManagementOnStartup, LOCALE_GET("base_stations_power_mgmt_on_startup_description").c_str())) {
            ConfigurationManager::getInstance()->getConfiguration()->base_stations.auto_turn_on_during_startup = g_state.bBaseStationPowerManagementOnStartup;
            ConfigurationManager::getInstance()->saveConfiguration();
        }

        if (ImGui::CheckboxWithDescription(LOCALE_GET("base_stations_power_mgmt_on_shutdown").c_str(), &g_state.bBaseStationPowerManagementOnShutdown, LOCALE_GET("base_stations_power_mgmt_on_shutdown_description").c_str())) {
            ConfigurationManager::getInstance()->getConfiguration()->base_stations.auto_turn_off_during_shutdown = g_state.bBaseStationPowerManagementOnShutdown;
            ConfigurationManager::getInstance()->saveConfiguration();
        }

        if (g_state.bIsSettingsAdvanced) {
            ImGui::Indent(fCheckboxTextIndent);
            if (ImGui::RadioButtonWithDescription(LOCALE_GET("base_stations_power_mgmt_standby").c_str(), !g_state.bBaseStationPowerManagementOffModeIsSleep, LOCALE_GET("base_stations_power_mgmt_standby_description").c_str())) {
                ConfigurationManager::getInstance()->getConfiguration()->base_stations.off_should_use_standby = true;
                ConfigurationManager::getInstance()->saveConfiguration();
            }

            if (ImGui::RadioButtonWithDescription(LOCALE_GET("base_stations_power_mgmt_sleep").c_str(), g_state.bBaseStationPowerManagementOffModeIsSleep, LOCALE_GET("base_stations_power_mgmt_sleep_description").c_str())) {
                ConfigurationManager::getInstance()->getConfiguration()->base_stations.off_should_use_standby = false;
                ConfigurationManager::getInstance()->saveConfiguration();
            }
            ImGui::Unindent(fCheckboxTextIndent);
        }
        ImGui::Unindent(fCheckboxTextIndent);
        ImGui::EndDisabled();

        ImGui::TextWrapped(LOCALE_GET("base_stations_power_mgmt_warning").c_str());
    }
    ImGui::EndCard();

    if (g_state.bIsSettingsAdvanced) {
        ImGui::BeginCardDanger("settings_reset");
        ImGui::TextWrappedDisabled(LOCALE_GET("settings_reset_description").c_str());
        bool bDoShowPopup = ImGui::IconButton(ICON_MS_RESET_SETTINGS, LOCALE_GET("settings_reset").c_str());
        ImGui::EndCardDanger();
        if (bDoShowPopup) {
            ImGui::OpenPopup("##settings_reset_modal");
        }
    }

    ImVec2 center = ImGui::GetMainViewport()->GetCenter();
    ImGui::SetNextWindowPos(center, ImGuiCond_Appearing, ImVec2(0.5f, 0.5f));
    ImGui::SetNextWindowSize(ImVec2(450.0f, 220.0f), ImGuiCond_Appearing);
    if (ImGui::BeginPopupModal("##settings_reset_modal", nullptr, ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_NoMove)) {
        ImGui::TextTitle(LOCALE_GET("settings_reset").c_str());
        ImGui::TextWrappedDisabled(LOCALE_GET("settings_reset_confirm").c_str());

        // anchor to bottom
        float buttonHeight = ImGui::GetFrameHeight();
        float targetY = ImGui::GetWindowHeight() - buttonHeight - ImGui::GetStyle().WindowPadding.y;
        ImGui::SetCursorPosY(targetY);

        // center the buttons
        std::string yesLabel = fmt::format("{} {}", ICON_MS_DELETE_FOREVER, LOCALE_GET("settings_reset_yes"));
        std::string noLabel = fmt::format("{} {}", ICON_MS_CLOSE, LOCALE_GET("settings_reset_no"));

        float paddingX = ImGui::GetStyle().FramePadding.x * 2.0f;
        float btn1Width = ImGui::CalcTextSize(yesLabel.c_str()).x + paddingX;
        float btn2Width = ImGui::CalcTextSize(noLabel.c_str()).x + paddingX;
        float totalWidth = btn1Width + btn2Width + ImGui::GetStyle().ItemSpacing.x;
        float startX = (ImGui::GetWindowWidth() - totalWidth) * 0.5f;
        if (startX > 0.0f) {
            ImGui::SetCursorPosX(startX);
        }

        // danger button
        ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.68f, 0.28f, 0.32f, 1.0f));
        ImGui::PushStyleColor(ImGuiCol_ButtonActive, ImVec4(0.75f, 0.38f, 0.42f, 1.0f));
        ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.72f, 0.32f, 0.36f, 1.0f));
        if (ImGui::IconButton(ICON_MS_DELETE_FOREVER, LOCALE_GET("settings_reset_yes").c_str())) {
            ConfigurationManager::getInstance()->resetConfiguration();
            ConfigurationManager::getInstance()->saveConfiguration();
            ImGui::CloseCurrentPopup();
        }
        ImGui::PopStyleColor(3);

        ImGui::SameLine();
        if (ImGui::IconButton(ICON_MS_CLOSE, LOCALE_GET("settings_reset_no").c_str())) {
            ImGui::CloseCurrentPopup();
        }
        ImGui::EndPopup();
    }
}

void page_base_station_management(double currentTime)
{
    ImGui::TextTitle("%s", LOCALE_GET("base_stations_title").c_str());

    if (!bluetooth::is_bluetooth_available()) {
        ImGui::TextDisabled(ICON_MS_BLUETOOTH_DISABLED "  ");
        ImGui::SameLine();
        ImGui::TextDisabled(LOCALE_GET("base_stations_no_bluetooth").c_str());
        return;
    }

    // bt avail
    size_t dwBaseStationCount = bluetooth::get_base_station_count();

    if (dwBaseStationCount > 0) {
        if (bluetooth::do_base_station_channels_collide()) {
            ImGui::BeginCardDanger("base_station_collision");
            ImGui::TextWrapped(LOCALE_GET("base_stations_warning_collision").c_str());

            if (ImGui::IconButton(ICON_MS_HANDYMAN, LOCALE_GET("base_stations_action_fix_collisions").c_str())) {
                bluetooth::auto_assign_base_station_channels();
            }
            ImGui::EndCardDanger();
        }

        if (ImGui::IconButton(ICON_MS_MODE_OFF_ON, LOCALE_GET("base_stations_action_wake_all").c_str())) {
            bluetooth::set_all_base_station_power_state(bluetooth::PowerState_Awake_From_Standby);
        }
        ImGui::SameLine();
        if (ImGui::IconButton(ICON_MS_LIGHT_OFF, LOCALE_GET("base_stations_action_sleep_all").c_str())) {
            bluetooth::set_all_base_station_power_state(bluetooth::PowerState_Sleep);
        }

        ImGui::Spacing();
        ImGui::Separator();
        ImGui::Spacing();
    }

    if (dwBaseStationCount < 1) {
        ImGui::TextDisabled(LOCALE_GET("base_stations_none_found").c_str());
    } else {
        uint16_t activeChannelsMask = 0; // bitwise mask; we have 16 channels on 2.0s so only up to 16 unique bits (slots) to use

        // build a bitwise mask of the occupied channels, if one is set more than once
        for (size_t i = 0; i < dwBaseStationCount; i++) {
            auto base_station = bluetooth::get_base_station(i);
            if (base_station.eType == bluetooth::BaseStationType_20) {
                auto channel = base_station.channel;
                if (IS_BASE_STATION_20_CHANNEL_VALID(channel)) {
                    uint16_t channelBit = (1U << (channel - 1));
                    activeChannelsMask |= channelBit;
                }
            }
        }

        if (ImGui::BeginTable("base_stations_grid", 2, ImGuiTableFlags_SizingStretchSame)) {
            for (size_t i = 0; i < dwBaseStationCount; i++) {
                auto base_station = bluetooth::get_base_station(i);

                ImGui::TableNextColumn();
                ImGui::BeginCard(fmt::format("card_base_station__{}", i).c_str());

                int baseStationIdx = -1;
                for (size_t i = 0; i < g_state.aBaseStations.size(); i++) {
                    if (g_state.aBaseStations[i].szBaseStationId == base_station.szSerialNumber) {
                        baseStationIdx = (int)i;
                        break;
                    }
                }

                if (baseStationIdx == -1) {
                    // new entry; add and reserve 512 bytes for nickname
                    UserInterface_BaseStationState_t& entry = g_state.aBaseStations.emplace_back(UserInterface_BaseStationState_t {
                        .bIsEditing = false,
                        .szBaseStationId = base_station.szSerialNumber,
                        .szNickname = "",
                    });
                    baseStationIdx = (int)g_state.aBaseStations.size() - 1;
                    g_state.aBaseStations[baseStationIdx].szNickname.resize(512, '\0');
                }

                const char* typeStr = (base_station.eType == bluetooth::BaseStationType_10) ? "1.0" : "2.0";
                bool hasNickname = g_state.aBaseStations[baseStationIdx].szNickname[0] != '\0';
                std::string szBaseStationName = hasNickname ? g_state.aBaseStations[baseStationIdx].szNickname.c_str() : base_station.szSerialNumber;
                std::string headerText = LOCALE_FORMAT("base_stations_header_info", szBaseStationName);

                if (g_state.aBaseStations[baseStationIdx].bIsEditing) {
                    const float lineStartY = ImGui::GetCursorPosY();
                    const float pillHeight = ImGui::GetFontSize() + (ImGui::k_PILL_PADDING_Y * 2.0f);
                    const float textOffsetY = (pillHeight - ImGui::GetFontSize()) * 0.5f;
                    ImGui::SetCursorPosY(ImGui::GetCursorPosY() + textOffsetY);
                    ImGui::AlignTextToFramePadding();
                    ImGui::TextHeading(LOCALE_GET("base_stations_nickname").c_str());
                    ImGui::SameLine();
                    ImGui::InputTextEx(
                        fmt::format("##Heading__{}", base_station.szSerialNumber).c_str(),
                        base_station.szSerialNumber.c_str(),
                        g_state.aBaseStations[baseStationIdx].szNickname.data(), (int)g_state.aBaseStations[baseStationIdx].szNickname.size(),
                        ImVec2(0, 0),
                        ImGuiInputTextFlags_None);
                    ImGui::SetCursorPosY(lineStartY);
                } else {
                    const float lineStartY = ImGui::GetCursorPosY();
                    const float pillHeight = ImGui::GetFontSize() + (ImGui::k_PILL_PADDING_Y * 2.0f);
                    const float textOffsetY = (pillHeight - ImGui::GetFontSize()) * 0.5f;
                    ImGui::SetCursorPosY(lineStartY + textOffsetY);
                    ImGui::TextHeading("%s", headerText.c_str());
                    ImGui::SameLine();
                    ImGui::SetCursorPosY(lineStartY);
                    ImGui::PillText(typeStr, Colors::Gray);
                }

                std::string statusStr;
                ImVec4 statusBgColor;
                switch (base_station.powerState) {
                case bluetooth::PowerState_Sleep:
                    statusStr = LOCALE_GET("base_stations_state_sleep");
                    statusBgColor = Colors::Amber;
                    break;
                case bluetooth::PowerState_Standby:
                    statusStr = LOCALE_GET("base_stations_state_standby");
                    statusBgColor = Colors::Yellow;
                    break;
                case bluetooth::PowerState_Awake_From_Sleep:
                case bluetooth::PowerState_Awake_From_Standby:
                case bluetooth::PowerState_Awake_TooOldFirmware:
                    statusStr = LOCALE_GET("base_stations_state_active");
                    statusBgColor = Colors::Green;
                    break;
                default:
                    statusStr = LOCALE_GET("base_stations_state_unknown");
                    statusBgColor = Colors::Purple;
                    break;
                }

                if (base_station.isFirmwareCooked) {
                    statusStr = LOCALE_GET("base_stations_state_faulty");
                    statusBgColor = Colors::Purple;
                }

                float pillWidth = ImGui::CalcTextSize(statusStr.c_str()).x + ImGui::k_PILL_PADDING_X * 2.0f;
                ImGui::SameLine(ImGui::GetContentRegionAvail().x - pillWidth);
                ImGui::PushFont(ImGui::fonts::pHeading);
                ImGui::PillText(statusStr.c_str(), statusBgColor, ImVec4(0.23f, 0.26f, 0.32f, 1.0f));
                ImGui::PopFont();

                ImGui::Spacing();

                // channels are only know on 2.0s
                if (base_station.eType == bluetooth::BaseStationType_20) {
                    if (!g_state.aBaseStations[baseStationIdx].bIsEditing) {
                        // show channel
                        if (g_state.bIsSettingsAdvanced) {
                            float currY = ImGui::GetCursorPosY();
                            ImGui::SetCursorPosY(currY - ImGui::GetStyle().ItemSpacing.y - ImGui::GetStyle().FramePadding.y - ImGui::k_PILL_PADDING_Y * 1.5f);
                            ImGui::Text("%s", LOCALE_FORMAT("base_stations_active_channel", base_station.channel).c_str());
                        }
                    } else {
                        // edit channels view
                        ImGui::Text("%s", LOCALE_GET("base_stations_channel_select_label").c_str());

                        ImGui::PushStyleVar(ImGuiStyleVar_FrameRounding, 24.0f);
                        ImVec2 buttonSize = ImVec2(48.0f, 48.0f);

                        const float itemSpacingX = ImGui::GetStyle().ItemSpacing.x;
                        float totalGridWidth = (buttonSize.x * 8) + (itemSpacingX * 7);

                        float startX = (ImGui::GetContentRegionAvail().x - totalGridWidth) * 0.5f;
                        if (startX < ImGui::GetCursorPosX()) {
                            startX = ImGui::GetCursorPosX();
                        }

                        for (int row = 0; row < 2; ++row) {
                            ImGui::SetCursorPosX(startX);

                            for (int col = 0; col < 8; ++col) {
                                uint8_t targetChannel = static_cast<uint8_t>((row * 8) + col + 1);

                                std::string channelBtnLabel = fmt::format("{}", targetChannel);

                                uint16_t channelBit = (1U << (targetChannel - 1));
                                bool isOccupied = (activeChannelsMask & channelBit) == channelBit;
                                bool isCurrent = (base_station.channel == targetChannel);

                                if (isCurrent) {
                                    ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.55f, 0.66f, 0.46f, 1.0f));
                                    ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.64f, 0.75f, 0.55f, 1.0f));
                                } else if (isOccupied) {
                                    // if the channel is used by another station make it appear disabled!
                                    ImGui::PushStyleColor(ImGuiCol_Button, ImGui::GetStyle().Colors[ImGuiCol_Button] * ImGui::GetStyle().DisabledAlpha);
                                    ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImGui::GetStyle().Colors[ImGuiCol_ButtonHovered] * ImGui::GetStyle().DisabledAlpha);
                                } else {
                                    ImGui::PushStyleColor(ImGuiCol_Button, ImGui::GetStyle().Colors[ImGuiCol_Button]);
                                    ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImGui::GetStyle().Colors[ImGuiCol_ButtonHovered]);
                                }

                                if (!isCurrent && !isOccupied) {
                                    ImGui::PushStyleColor(ImGuiCol_Border, ImVec4(0.30f, 0.33f, 0.42f, 0.40f));
                                    ImGui::PushStyleVar(ImGuiStyleVar_FrameBorderSize, ImGui::k_BORDER_WIDTH);
                                }
                                if (ImGui::Button(channelBtnLabel.c_str(), buttonSize)) {
                                    bluetooth::set_base_station_channel(i, targetChannel);
                                }
                                if (!isCurrent && !isOccupied) {
                                    ImGui::PopStyleVar();
                                    ImGui::PopStyleColor();
                                }

                                ImGui::PopStyleColor(2);

                                if (col < 7) {
                                    ImGui::SameLine();
                                }
                            }
                        }
                        ImGui::PopStyleVar();
                        ImGui::Spacing();
                    }
                }

                if (!g_state.aBaseStations[baseStationIdx].bIsEditing) {
                    if (ImGui::IconButton(ICON_MS_MODE_OFF_ON, LOCALE_GET("base_stations_btn_wake").c_str())) {
                        bluetooth::set_base_station_power_state(i, bluetooth::PowerState_Awake_From_Standby);
                    }
                    if (g_state.bIsSettingsAdvanced) {
                        // standby is unsupported on 1.0s
                        if (base_station.eType == bluetooth::BaseStationType_20) {
                            ImGui::SameLine();
                            // if we know the base station is on old firmware, disable standby, as it's an unsupported operation anyway
                            ImGui::BeginDisabled(base_station.firmwareSupportsStandby == bluetooth::EBaseStation20_StandbySupport_t::StandbySupport_Unavailable);
                            if (ImGui::IconButton(ICON_MS_ENERGY_SAVINGS_LEAF, LOCALE_GET("base_stations_btn_standby").c_str())) {
                                bluetooth::set_base_station_power_state(i, bluetooth::PowerState_Standby);
                            }
                            ImGui::EndDisabled();
                        }
                    }
                    ImGui::SameLine();
                    if (ImGui::IconButton(ICON_MS_LIGHT_OFF, LOCALE_GET("base_stations_btn_sleep").c_str())) {
                        bluetooth::set_base_station_power_state(i, bluetooth::PowerState_Sleep);
                    }
                }

                if (g_state.aBaseStations[baseStationIdx].bIsEditing) {
                    if (ImGui::IconButton(ICON_MS_SAVE, LOCALE_GET("base_stations_btn_save").c_str())) {
                        // @HACK: remove null bytes from nickname before writing to disk; should we use strlen or something similar?
                        std::string cleanedNickname = g_state.aBaseStations[baseStationIdx].szNickname.c_str();
                        ConfigurationManager::getInstance()->getConfiguration()->base_stations.nicknames[base_station.szSerialNumber] = cleanedNickname;
                        ConfigurationManager::getInstance()->saveConfiguration();

                        g_state.aBaseStations[baseStationIdx].bIsEditing = false;
                    }
                } else {
                    std::string editButtonTxt = LOCALE_GET("base_stations_btn_edit");
                    float editBtnWidth = ImGui::CalcTextSize(editButtonTxt.c_str()).x + ImGui::GetStyle().FramePadding.x * 2.0f + ImGui::CalcTextSize(ICON_MS_EDIT).x;

                    ImGui::SameLine(ImGui::GetContentRegionAvail().x - editBtnWidth);
                    if (ImGui::IconButton(ICON_MS_EDIT, editButtonTxt.c_str())) {
                        g_state.aBaseStations[baseStationIdx].bIsEditing = true;
                    }
                }

                ImGui::EndCard();

                if (i < dwBaseStationCount - 1) {
                    ImGui::Spacing();
                }
            }
            ImGui::EndTable();
        }
    }
}

inline const char* get_tracking_result_name(vr::ETrackingResult result)
{
    switch (result) {
    case vr::TrackingResult_Uninitialized:
        return "Uninitialized";
    case vr::TrackingResult_Calibrating_InProgress:
        return "Calibrating (In Progress)";
    case vr::TrackingResult_Calibrating_OutOfRange:
        return "Calibrating (Out of Range)";
    case vr::TrackingResult_Running_OK:
        return "Running (OK)";
    case vr::TrackingResult_Running_OutOfRange:
        return "Running (Out of Range)";
    case vr::TrackingResult_Fallback_RotationOnly:
        return "Fallback (Rotation Only)";
    default:
        return "Unknown";
    }
}

inline void debug_driver_pose_viewer(const char* label, const vr::DriverPose_t& pose)
{

#define SHOW_BOOL_STATUS(label, value)              \
    ImGui::Text(label ":");                         \
    ImGui::SameLine();                              \
    if (value) {                                    \
        ImGui::TextColored(Colors::Green, "TRUE");  \
    } else {                                        \
        ImGui::TextColored(Colors::Amber, "FALSE"); \
    }

    if (ImGui::CollapsingHeader(label, ImGuiTreeNodeFlags_DefaultOpen)) {
        ImGui::Indent();
        ImGui::PushID(label);
        if (ImGui::CollapsingHeader("Metadata", ImGuiTreeNodeFlags_DefaultOpen)) {
            SHOW_BOOL_STATUS("Device Connected", pose.deviceIsConnected);
            SHOW_BOOL_STATUS("Pose Is Valid", pose.poseIsValid);
            SHOW_BOOL_STATUS("Will Drift In Yaw", pose.willDriftInYaw);
            SHOW_BOOL_STATUS("Should Apply Head", pose.shouldApplyHeadModel);

            ImGui::Separator();

            ImGui::Text("Tracking Result:");
            ImGui::SameLine();
            if (pose.result == vr::TrackingResult_Running_OK) {
                ImGui::TextColored(ImVec4(0.0f, 0.8f, 1.0f, 1.0f), "%s", get_tracking_result_name(pose.result));
            } else {
                ImGui::TextColored(ImVec4(1.0f, 0.6f, 0.0f, 1.0f), "%s", get_tracking_result_name(pose.result));
            }

            ImGui::Text("Prediction Time: %f s", pose.poseTimeOffset);
        }
        if (ImGui::CollapsingHeader("Motion data")) {
            ImGui::Text("Position:     %.4f, %.4f, %.4f", pose.vecPosition[0], pose.vecPosition[1], pose.vecPosition[2]);
            ImGui::Text("Velocity:     %.4f, %.4f, %.4f", pose.vecVelocity[0], pose.vecVelocity[1], pose.vecVelocity[2]);
            ImGui::Text("Accel:        %.4f, %.4f, %.4f", pose.vecAcceleration[0], pose.vecAcceleration[1], pose.vecAcceleration[2]);
            ImGui::Separator();
            ImGui::Text("Ang Velocity: %.4f, %.4f, %.4f", pose.vecAngularVelocity[0], pose.vecAngularVelocity[1], pose.vecAngularVelocity[2]);
            ImGui::Text("Ang Accel:    %.4f, %.4f, %.4f", pose.vecAngularAcceleration[0], pose.vecAngularAcceleration[1], pose.vecAngularAcceleration[2]);
            ImGui::Separator();
            ImGui::Text("Quat (WXYZ):  %.4f, %.4f, %.4f, %.4f", pose.qRotation.w, pose.qRotation.x, pose.qRotation.y, pose.qRotation.z);
        }
        if (ImGui::CollapsingHeader("Coordinate Transforms")) {
            ImGui::TextDisabled("World From Driver-Space");
            ImGui::Text("Translation:  %.4f, %.4f, %.4f", pose.vecWorldFromDriverTranslation[0], pose.vecWorldFromDriverTranslation[1], pose.vecWorldFromDriverTranslation[2]);
            ImGui::Text("Quat (WXYZ):  %.4f, %.4f, %.4f, %.4f", pose.qWorldFromDriverRotation.w, pose.qWorldFromDriverRotation.x, pose.qWorldFromDriverRotation.y, pose.qWorldFromDriverRotation.z);

            ImGui::Separator();

            ImGui::TextDisabled("Driver From Head-Space");
            ImGui::Text("Translation:  %.4f, %.4f, %.4f", pose.vecDriverFromHeadTranslation[0], pose.vecDriverFromHeadTranslation[1], pose.vecDriverFromHeadTranslation[2]);
            ImGui::Text("Quat (WXYZ):  %.4f, %.4f, %.4f, %.4f", pose.qDriverFromHeadRotation.w, pose.qDriverFromHeadRotation.x, pose.qDriverFromHeadRotation.y, pose.qDriverFromHeadRotation.z);
        }
        ImGui::PopID();
        ImGui::Unindent();
    }
#undef SHOW_BOOL_STATUS
}

void plot_sample_rotations_on_unit_sphere(const std::vector<Sample_t>& samples)
{
    if (samples.empty())
        return;

    constexpr int kRingSegments = 32;
    constexpr int kRingPoints = kRingSegments + 1;

    const size_t ring_offset_x = 0;
    const size_t ring_offset_y = ring_offset_x + kRingPoints;
    const size_t ring_offset_z = ring_offset_y + kRingPoints;

    const size_t ref_offset_x = ring_offset_z + kRingPoints;
    const size_t ref_offset_y = ref_offset_x + samples.size();
    const size_t ref_offset_z = ref_offset_y + samples.size();

    const size_t tgt_offset_x = ref_offset_z + samples.size();
    const size_t tgt_offset_y = tgt_offset_x + samples.size();
    const size_t tgt_offset_z = tgt_offset_y + samples.size();

    const size_t total_floats = tgt_offset_z + samples.size();

    std::vector<float> buffer(total_floats, 0.0f);

    for (int i = 0; i <= kRingSegments; ++i) {
        float theta = (2.0f * static_cast<float>(M_PI) * i) / kRingSegments;
        buffer[ring_offset_x + i] = cos(theta);
        buffer[ring_offset_y + i] = sin(theta);
        buffer[ring_offset_z + i] = 0.0f;
    }

    int valid_sample_count = 0;
    for (const auto& sample : samples) {
        if (!sample.isPoseValid)
            continue; // should be a no-op

        Eigen::Vector3d ref_fwd = -sample.reference.rot.col(2);
        Eigen::Vector3d tgt_fwd = -sample.target.rot.col(2);

        buffer[ref_offset_x + valid_sample_count] = (float)ref_fwd.x();
        buffer[ref_offset_y + valid_sample_count] = (float)-ref_fwd.z();
        buffer[ref_offset_z + valid_sample_count] = (float)ref_fwd.y();

        buffer[tgt_offset_x + valid_sample_count] = (float)tgt_fwd.x();
        buffer[tgt_offset_y + valid_sample_count] = (float)-tgt_fwd.z();
        buffer[tgt_offset_z + valid_sample_count] = (float)tgt_fwd.y();

        valid_sample_count++;
    }

    if (valid_sample_count == 0)
        return;

    if (ImPlot3D::BeginPlot("Sampled Rotations", ImVec2(-1, 500))) {
        ImPlot3D::SetupAxesLimits(-1.1f, 1.1f, -1.1f, 1.1f, -1.1f, 1.1f, ImPlot3DCond_Always);
        ImPlot3D::SetupAxes("X", "Z", "Y");

        ImPlot3DSpec ringSpec;
        ringSpec.LineColor = ImVec4(0.3f, 0.3f, 0.3f, 0.5f);
        ringSpec.LineWeight = 1.0f;
        ImPlot3D::PlotLine("Equator Ring", buffer.data() + ring_offset_x, buffer.data() + ring_offset_y, buffer.data() + ring_offset_z, kRingPoints, ringSpec);

        ImPlot3DSpec refSpec;
        refSpec.Marker = ImPlot3DMarker_Circle;
        refSpec.MarkerSize = 4.0f;
        refSpec.MarkerFillColor = ImVec4(0.0f, 1.0f, 0.2f, 0.8f);
        refSpec.MarkerLineColor = ImVec4(0.0f, 1.0f, 0.2f, 0.8f);
        ImPlot3D::PlotScatter("Reference Forward", buffer.data() + ref_offset_x, buffer.data() + ref_offset_y, buffer.data() + ref_offset_z, valid_sample_count, refSpec);

        ImPlot3DSpec tgtSpec;
        tgtSpec.Marker = ImPlot3DMarker_Circle;
        tgtSpec.MarkerSize = 4.0f;
        tgtSpec.MarkerFillColor = ImVec4(1.0f, 0.2f, 0.2f, 0.8f);
        tgtSpec.MarkerLineColor = ImVec4(1.0f, 0.2f, 0.2f, 0.8f);
        ImPlot3D::PlotScatter("Target Forward", buffer.data() + tgt_offset_x, buffer.data() + tgt_offset_y, buffer.data() + tgt_offset_z, valid_sample_count, tgtSpec);

        ImPlot3D::EndPlot();
    }
}

void page_debug(double currentTime)
{
    ImGui::TextTitle("%s", LOCALE_GET("tab_page_debug").c_str());

    spacecal::TrackingSystemCalibration& calibration = spacecal::CalibrationManager::getInstance()->getCalibration(g_state.dwSelectedCalibrationIndex);
    if (calibration.referenceDevice.deviceId < vr::k_unMaxTrackedDeviceCount) {
        debug_driver_pose_viewer("Reference device DriverPose_t", CalibrationManager::getInstance()->m_poses[calibration.referenceDevice.deviceId]);
    }
    if (calibration.targetDevice.deviceId < vr::k_unMaxTrackedDeviceCount) {
        debug_driver_pose_viewer("Target device DriverPose_t", CalibrationManager::getInstance()->m_poses[calibration.targetDevice.deviceId]);
    }

    // @HACK: purely for visualisation to see if this shit even works
    plot_sample_rotations_on_unit_sphere(calibration.m_samples);
}

void page_about(double currentTime)
{
    const float k_SPACING = ImGui::GetStyle().ItemSpacing.y * 1.0f;

    ImGui::TextTitle("%s", LOCALE_GET("about_title").c_str());

    ImGui::Text(LOCALE_GET("about_description").c_str());
    ImGui::TextWrapped(LOCALE_GET("about_view_source_info").c_str());

    ImGui::Dummy(ImVec2(0, k_SPACING));

    ImGui::TextHeading(LOCALE_GET("about_contributors_title").c_str());
    ImGui::TextWrapped(LOCALE_GET("about_contributors_description").c_str());
    ImGui::BulletText("pushrax");
    ImGui::BulletText("bd_");
    ImGui::BulletText("ArticFox");
    ImGui::BulletText("hekky");
    ImGui::BulletText("pimaker");

    ImGui::Dummy(ImVec2(0, k_SPACING));

    ImGui::TextHeading(LOCALE_GET("about_translators_title").c_str());
    ImGui::TextWrapped(LOCALE_GET("about_translators_description").c_str());
    ImGui::BulletText("hekky");
    ImGui::BulletText("Hash");
    ImGui::BulletText("lenr");
    ImGui::BulletText("CucumberWorks");
    ImGui::BulletText("shau");
    ImGui::BulletText("nym (qqq10)");
    ImGui::BulletText("m3gagluk");
    ImGui::BulletText("Flippy");
    ImGui::BulletText("Nara");
    ImGui::BulletText("C'Ya");
    ImGui::BulletText("Dem01nS");

    ImGui::Dummy(ImVec2(0, k_SPACING));

    ImGui::PushStyleVar(ImGuiStyleVar_FramePadding, ImVec2(8.0f, 8.0f));

    if (ImGui::IconButton(ICON_MS_OPEN_IN_BROWSER, LOCALE_GET("about_link_github").c_str())) {
        platform::launchWebpage("https://github.com/hyblocker/OpenVR-SpaceCalibrator");
    }

    ImGui::SameLine();

    if (ImGui::IconButton(ICON_MS_OPEN_IN_BROWSER, LOCALE_GET("about_link_discord").c_str())) {
        platform::launchWebpage("https://discord.gg/YWN7Z9T8DP");
    }

    ImGui::SameLine();

    if (ImGui::IconButton(ICON_MS_FOLDER_OPEN, LOCALE_GET("about_link_logs_dir").c_str())) {
        platform::launchDirInFileBrowser(util::getSpaceCalibratorLogsDir());
    }

    ImGui::PopStyleVar();
    ImGui::Dummy(ImVec2(0, k_SPACING));

    ImGui::TextHeading(LOCALE_GET("about_licenses_title").c_str());
    ImGui::TextWrapped(LOCALE_GET("about_licenses_description").c_str());

    // throw it in a child container for funny box to make it clear this is a massive read-only block of text
    if (ImGui::BeginChild("##licenses_box", ImVec2(ImGui::GetContentRegionAvail().x - ImGui::GetStyle().WindowPadding.x, 0.0f), ImGuiChildFlags_Borders | ImGuiChildFlags_AutoResizeY)) {
        ImGui::TextUnformatted(g_licenses_text.data(), g_licenses_text.data() + g_licenses_text.size());
    }
    ImGui::EndChild();
}

// tutorial page
void page_tutorial(double currentTime)
{
    ImGui::TextTitle("%s", LOCALE_GET("learn_title").c_str());

#define DRAW_IMAGE(label, idx)                                                                                                                              \
    do {                                                                                                                                                    \
        if (ImGui::BeginChild((label), ImVec2(-1.0f, (float)g_state.textures[idx].dwHeight))) {                                                             \
            ImVec2 avail = ImGui::GetContentRegionAvail();                                                                                                  \
            ImGui::SetCursorPos(ImVec2((avail.x - (float)g_state.textures[idx].dwWidth) * 0.5f, (avail.y - (float)g_state.textures[idx].dwHeight) * 0.5f)); \
            ImGui::Image(g_state.textures[idx].hTexture, ImVec2((float)g_state.textures[idx].dwWidth, (float)g_state.textures[idx].dwHeight));              \
        }                                                                                                                                                   \
        ImGui::EndChild();                                                                                                                                  \
        ImGui::Spacing();                                                                                                                                   \
    } while (0)

    switch (g_state.dwSelectedLearnPage) {
    default:
    case LearnPage_Home: {
        constexpr float k_PREVIEW_IMAGE_SIZE_PIXELS = 140.0f;
        constexpr float k_CELL_PADDING = 4.0f;

        ImGui::PushStyleVar(ImGuiStyleVar_CellPadding, ImVec2(k_CELL_PADDING, k_CELL_PADDING));
        ImGui::BeginTable("ui_learn_toc_cards", 2, ImGuiTableFlags_None);

        ImGui::TableNextColumn();
        if (ImGui::InformationButton(ICON_MS_TARGET,
                LOCALE_GET("learn_card_standard_calibration").c_str(),
                LOCALE_GET("learn_card_standard_calibration_description").c_str(),
                g_state.textures[EImageId_Learn_Preview_Standard].hTexture,
                ImVec2(k_PREVIEW_IMAGE_SIZE_PIXELS, k_PREVIEW_IMAGE_SIZE_PIXELS))) {
            g_state.dwSelectedLearnPage = LearnPage_Standard;
        }

        ImGui::TableNextColumn();
        if (ImGui::InformationButton(ICON_MS_CYCLE,
                LOCALE_GET("learn_card_continuous_calibration").c_str(),
                LOCALE_GET("learn_card_continuous_calibration_description").c_str(),
                g_state.textures[EImageId_Learn_Preview_Continuous].hTexture,
                ImVec2(k_PREVIEW_IMAGE_SIZE_PIXELS, k_PREVIEW_IMAGE_SIZE_PIXELS))) {
            g_state.dwSelectedLearnPage = LearnPage_Continuous;
        }

        ImGui::TableNextColumn();
        if (ImGui::InformationButton(ICON_MS_SENSORS,
                LOCALE_GET("learn_card_base_station_management").c_str(),
                LOCALE_GET("learn_card_base_station_management_description").c_str(),
                g_state.textures[EImageId_Learn_Preview_BaseStations].hTexture,
                ImVec2(k_PREVIEW_IMAGE_SIZE_PIXELS, k_PREVIEW_IMAGE_SIZE_PIXELS))) {
            g_state.dwSelectedLearnPage = LearnPage_BaseStations;
        }

        ImGui::TableNextColumn();
        if (ImGui::InformationButton(ICON_MS_HANDHELD_CONTROLLER,
                LOCALE_GET("learn_card_ui_tour").c_str(),
                LOCALE_GET("learn_card_ui_tour_description").c_str(),
                g_state.textures[EImageId_Learn_Preview_UITour].hTexture,
                ImVec2(k_PREVIEW_IMAGE_SIZE_PIXELS, k_PREVIEW_IMAGE_SIZE_PIXELS))) {
            g_state.dwSelectedLearnPage = LearnPage_UITour;
        }

        ImGui::PopStyleVar();
        ImGui::EndTable();

        break;
    }
    case LearnPage_Standard: {
        ImGui::TextHeading(LOCALE_GET("learn_card_standard_calibration").c_str());
        ImGui::TextWrapped(LOCALE_GET("learn_page_standard_desc").c_str());
        ImGui::Spacing();

        ImGui::TextHeading(LOCALE_GET("learn_page_standard_select_devices_title").c_str());
        ImGui::TextWrapped(LOCALE_GET("learn_page_standard_select_devices_desc").c_str());
        ImGui::Bullet();
        ImGui::TextWrapped(LOCALE_GET("learn_page_standard_ref_device").c_str());
        ImGui::Bullet();
        ImGui::TextWrapped(LOCALE_GET("learn_page_standard_target_device").c_str());
        ImGui::Spacing();

        ImGui::TextHeading(LOCALE_GET("learn_page_standard_perform_title").c_str());
        ImGui::Bullet();
        ImGui::TextWrapped(LOCALE_GET("learn_page_standard_perform_step1").c_str());
        ImGui::Bullet();
        ImGui::TextWrapped(LOCALE_GET("learn_page_standard_perform_step2").c_str());
        ImGui::Bullet();
        ImGui::TextWrapped(LOCALE_GET("learn_page_standard_perform_step3").c_str());
        ImGui::Spacing();

        // @TODO: video demonstrating calibration method
        DRAW_IMAGE("calibrate_diagram", EImageId_LearnStandard_CalibrateDiagram);

        ImGui::TextHeading(LOCALE_GET("learn_page_standard_speeds_title").c_str());
        ImGui::TextWrapped(LOCALE_GET("learn_page_standard_speeds_desc").c_str());
        ImGui::Bullet();
        ImGui::TextWrapped(LOCALE_GET("learn_page_standard_speed_fast").c_str());
        ImGui::Bullet();
        ImGui::TextWrapped(LOCALE_GET("learn_page_standard_speed_slow").c_str());

        ImGui::Spacing();
        ImGui::Spacing();

        ImGui::TextWrapped(LOCALE_GET("learn_page_standard_completion_note").c_str());
        ImGui::Spacing();

        ImGui::BeginCardDanger("learn_note_drift");
        ImGui::TextHeading(LOCALE_GET("learn_page_standard_drift_title").c_str());
        ImGui::TextWrapped(LOCALE_GET("learn_page_standard_drift_desc").c_str());
        ImGui::EndCardDanger();

        break;
    }
    case LearnPage_Continuous: {
        ImGui::TextHeading(LOCALE_GET("learn_card_continuous_calibration").c_str());
        ImGui::TextWrapped(LOCALE_GET("learn_page_continuous_desc1").c_str());
        ImGui::TextWrapped(LOCALE_GET("learn_page_continuous_desc2").c_str());
        ImGui::BeginCardDanger("info_dedicated_tracker_cont_cal");
        ImGui::TextHeading(LOCALE_GET("learn_page_continuous_note_title").c_str());
        ImGui::TextWrapped(LOCALE_GET("learn_page_continuous_note_desc").c_str());
        ImGui::EndCardDanger();
        ImGui::Spacing();
        ImGui::Spacing();
        ImGui::TextWrapped(LOCALE_GET("learn_page_continuous_desc3").c_str());
        ImGui::TextWrapped(LOCALE_GET("learn_page_continuous_setup_instructions").c_str());

        // pic of tracker on virtual VR headset to illustrate mounting
        DRAW_IMAGE("continuous_mount", EImageId_LearnContinuous_Mounting);

        ImGui::TextHeading(LOCALE_GET("learn_page_continuous_should_use_title").c_str());
        ImGui::Bullet();
        ImGui::TextWrapped(LOCALE_GET("learn_page_continuous_should_use_standard").c_str());
        ImGui::Bullet();
        ImGui::TextWrapped(LOCALE_GET("learn_page_continuous_should_use_continuous").c_str());
        ImGui::Spacing();
        ImGui::Spacing();

        ImGui::TextHeading(LOCALE_GET("learn_page_continuous_physical_setup_title").c_str());
        ImGui::TextWrapped(LOCALE_GET("learn_page_continuous_physical_setup_desc1").c_str());
        ImGui::Spacing();
        ImGui::TextWrapped(LOCALE_GET("learn_page_continuous_physical_setup_desc2").c_str());
        ImGui::Spacing();
        ImGui::Spacing();
        break;
    }
    case LearnPage_BaseStations: {
        ImGui::TextHeading(LOCALE_GET("learn_card_base_station_management").c_str());
        ImGui::TextWrapped(LOCALE_GET("learn_page_basestation_desc").c_str());
        ImGui::Spacing();

        ImGui::TextHeading(LOCALE_GET("learn_page_basestation_1_title").c_str());
        ImGui::Spacing();
        DRAW_IMAGE("basestation_image_10_front", EImageId_LearnBaseStation_10_Front);
        ImGui::Spacing();
        ImGui::TextWrapped(LOCALE_GET("learn_page_basestation_1_desc").c_str());
        ImGui::Spacing();
        DRAW_IMAGE("basestation_image_10_channel", EImageId_LearnBaseStation_10_Channel);
        ImGui::Spacing();
        ImGui::TextHeading(LOCALE_GET("learn_page_basestation_2_title").c_str());
        ImGui::TextWrapped(LOCALE_GET("learn_page_basestation_2_desc1").c_str());
        ImGui::TextWrapped(LOCALE_GET("learn_page_basestation_2_desc2").c_str());
        ImGui::Spacing();
        DRAW_IMAGE("basestation_image_20", EImageId_LearnBaseStation_20);
        ImGui::Spacing();
        ImGui::TextHeading(LOCALE_GET("learn_page_basestation_auto_power_title").c_str());
        ImGui::TextWrapped(LOCALE_GET("learn_page_basestation_auto_power_desc").c_str());

        break;
    }
    case LearnPage_UITour: {
        ImGui::TextHeading(LOCALE_GET("learn_card_ui_tour").c_str());
        ImGui::TextWrapped(LOCALE_GET("learn_page_ui_tour_desc").c_str());
        ImGui::Spacing();

        ImGui::TextHeading(LOCALE_GET("learn_page_ui_tour_calibration_heading").c_str());
        ImGui::TextWrapped(LOCALE_GET("learn_page_ui_tour_cal_desc1").c_str());
        ImGui::TextWrapped(LOCALE_GET("learn_page_ui_tour_cal_desc2").c_str());
        ImGui::TextWrapped(LOCALE_GET("learn_page_ui_tour_cal_desc3").c_str());
        ImGui::TextWrapped(LOCALE_GET("learn_page_ui_tour_cal_desc4").c_str());
        ImGui::TextWrapped(LOCALE_GET("learn_page_ui_tour_cal_desc5").c_str());

        DRAW_IMAGE("ui_tour_0", EImageId_LearnUI_Unk0);

        ImGui::TextHeading(LOCALE_GET("learn_page_ui_tour_base_stations_heading").c_str());
        ImGui::TextWrapped(LOCALE_GET("learn_page_ui_tour_bs_desc").c_str());
        ImGui::Spacing();

        DRAW_IMAGE("ui_tour_1", EImageId_LearnUI_Unk1);

        ImGui::TextHeading(LOCALE_GET("learn_page_ui_tour_settings_heading").c_str());
        ImGui::TextWrapped(LOCALE_GET("learn_page_ui_tour_settings_desc1").c_str());
        ImGui::Spacing();
        ImGui::TextWrapped(LOCALE_GET("learn_page_ui_tour_settings_desc2").c_str());

        break;
    }
    }
    if (g_state.dwSelectedLearnPage != LearnPage_Home) {
        if (ImGui::IconButton(ICON_MS_ARROW_BACK, LOCALE_GET("learn_action_back").c_str())) {
            g_state.dwSelectedLearnPage = LearnPage_Home;
        }
    }

#undef DRAW_DUMMY_IMAGE
}

// UI CORE LAYOUT

SpaceCalibratorVerticalTab_t g_spaceCalUiTabs[] = {
    {
        .szLocaleKey = "tab_page_calibration",
        .szIcon = ICON_MS_TARGET,
        .fnDrawTab = page_calibration,
        .bIsAdvancedTab = false,
    },
    {
        .szLocaleKey = "tab_page_graphs",
        .szIcon = ICON_MS_STACKED_LINE_CHART,
        .fnDrawTab = page_graphs,
        .bIsAdvancedTab = true,
    },
    {
        .szLocaleKey = "tab_page_base_station_management",
        .szIcon = ICON_MS_SENSORS,
        .fnDrawTab = page_base_station_management,
        .bIsAdvancedTab = false,
    }, // ICON_MS_CELL_TOWER
    {
        .szLocaleKey = "tab_page_settings",
        .szIcon = ICON_MS_SETTINGS,
        .fnDrawTab = page_settings,
        .bIsAdvancedTab = false,
    },
    {
        .szLocaleKey = "tab_page_debug",
        .szIcon = ICON_MS_TERMINAL,
        .fnDrawTab = page_debug,
        .bIsAdvancedTab = true,
    },
    {
        .szLocaleKey = "tab_page_learn",
        .szIcon = ICON_MS_SCHOOL,
        .fnDrawTab = page_tutorial,
        .bIsAdvancedTab = false,
    },
    {
        .szLocaleKey = "tab_page_about",
        .szIcon = ICON_MS_INFO,
        .fnDrawTab = page_about,
        .bIsAdvancedTab = false,
    },
};
constexpr size_t k_TAB_INDEX_LEARN = 5; // @NOTE: hardcoded- adjust if we add more
constexpr size_t k_SIZE_SPACECAL_UI_TABS = sizeof(g_spaceCalUiTabs) / sizeof(g_spaceCalUiTabs[0]);

inline bool verticalTab(const SpaceCalibratorVerticalTab_t& tabData, bool selected, const ImVec2& size)
{
    ImGuiWindow* window = ImGui::GetCurrentWindow();
    if (window->SkipItems)
        return false;

    ImGuiContext& g = *GImGui;
    const ImGuiStyle& style = g.Style;
    const ImGuiID id = window->GetID(tabData.szLocaleKey);

    ImVec2 pos = window->DC.CursorPos;
    const ImRect bb(pos, ImVec2(pos.x + size.x, pos.y + size.y));
    ImGui::ItemSize(size, style.FramePadding.y);
    if (!ImGui::ItemAdd(bb, id))
        return false;

    bool hovered, held;
    bool pressed = ImGui::ButtonBehavior(bb, id, &hovered, &held);

    constexpr ImU32 k_TRANSPARENT = IM_COL32(0, 0, 0, 0);
    constexpr float k_IndicatorWidth = 4.0f;
    constexpr float k_IconTextGap = 8.0f;
    const float k_LeftTextMargin = style.FramePadding.x * 2 + k_IndicatorWidth;

    if (selected || hovered) {
        ImU32 col_bg_max = k_TRANSPARENT;
        ImU32 col_bg_min = k_TRANSPARENT;
        if (selected) {
            col_bg_max = ImGui::GetColorU32(ImGuiCol_HeaderActive);
            col_bg_max = IM_COL32_SET_ALPHA(col_bg_max, 90); // 35% opacity
            col_bg_min = IM_COL32_SET_ALPHA(col_bg_max, 27); // 10% opacity
        } else if (hovered) {
            col_bg_max = ImGui::GetColorU32(ImGuiCol_HeaderHovered);
            col_bg_max = IM_COL32_SET_ALPHA(col_bg_max, 45); // 17.5% opacity
            col_bg_min = IM_COL32_SET_ALPHA(col_bg_max, 14); // 5% opacity
        }

        ImVec2 startPos = bb.Min;
        if (selected) {
            startPos.x += k_IndicatorWidth;
        }
        window->DrawList->AddRectFilledMultiColor(startPos, bb.Max, col_bg_max, col_bg_min, col_bg_min, col_bg_max);
    }

    if (selected) {
        ImVec2 line_min = bb.Min;
        ImVec2 line_max = ImVec2(bb.Min.x + k_IndicatorWidth, bb.Max.y);
        ImU32 col_line = ImGui::GetColorU32(ImGuiCol_SliderGrab);

        window->DrawList->AddRectFilled(line_min, line_max, col_line, style.FrameRounding, ImDrawFlags_RoundCornersLeft);
    }

    if (selected && hovered) {
        g_state.bCursorOverriddenThisFrame = true;
    }

    float final_text_offset_x = k_LeftTextMargin;

    // icon rendering
    ImVec2 iconSize = ImGui::CalcTextSize(tabData.szIcon, nullptr, true);
    float icon_y_offset = (size.y - iconSize.y) * 0.5f;
    ImVec2 icon_pos = ImVec2(bb.Min.x + final_text_offset_x, bb.Min.y + icon_y_offset);
    ImGui::RenderText(icon_pos, tabData.szIcon);
    final_text_offset_x += iconSize.x + k_IconTextGap;

    // text rendering
    std::string textStr = LOCALE_GET(tabData.szLocaleKey);
    ImVec2 textSize = ImGui::CalcTextSize(textStr.c_str(), nullptr, true);
    float text_y_offset = (size.y - textSize.y) * 0.5f;
    ImVec2 text_pos = ImVec2(bb.Min.x + final_text_offset_x, bb.Min.y + text_y_offset);
    ImGui::RenderText(text_pos, textStr.c_str());

    return pressed;
}

inline void drawMainView(double currentTime)
{
    // @TODO: temp hardcode, move to header or some ui_config.h idk
    const float k_SIDEBAR_WIDTH = 220.0f;
    const float k_SIDEBAR_TAB_HEIGHT = 48.0f;
    const float k_CONTENT_AREA_PADDING = 5.0f;

    size_t lastSelectedPage = g_state.dwSelectedUiPage;

    // sidebar
    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(0.0f, 0.0f));
    ImGui::PushStyleVar(ImGuiStyleVar_ItemSpacing, ImVec2(0.0f, 0.0f));
    ImGui::BeginChild("Sidebar", ImVec2(k_SIDEBAR_WIDTH, 0), ImGuiChildFlags_None);
    ImGui::Spacing();

    for (size_t i = 0; i < k_SIZE_SPACECAL_UI_TABS; ++i) {
        if (g_spaceCalUiTabs[i].bIsAdvancedTab && !g_state.bIsSettingsAdvanced) {
            continue;
        }
        ImGui::PushID((int)i);

        float itemWidth = ImGui::GetContentRegionAvail().x - ImGui::GetStyle().FramePadding.x;

        if (verticalTab(g_spaceCalUiTabs[i], g_state.dwSelectedUiPage == i, ImVec2(itemWidth, k_SIDEBAR_TAB_HEIGHT))) {
            g_state.dwSelectedUiPage = i;
        }

        ImGui::PopID();
        ImGui::Spacing();
    }
    ImGui::EndChild();
    ImGui::PopStyleVar(2);

    ImGui::SameLine();

    // content
    ImGui::BeginChild("ContentArea", ImVec2(0, ImGui::GetFrameHeightWithSpacing() * -2.0f), ImGuiChildFlags_None);

    // reset scroll pos on page change
    if (lastSelectedPage != g_state.dwSelectedUiPage) {
        ImGui::SetScrollY(0.0f);
    }

    ImGui::Dummy(ImVec2(0, k_CONTENT_AREA_PADDING));
    ImGui::Indent(k_CONTENT_AREA_PADDING);

    if (g_state.dwSelectedUiPage < k_SIZE_SPACECAL_UI_TABS) {
        g_spaceCalUiTabs[g_state.dwSelectedUiPage].fnDrawTab(currentTime);
    }

    ImGui::Unindent(k_CONTENT_AREA_PADDING);
    ImGui::EndChild();
}

// UI ENTRY POINT

void drawInterface(bool isOverlay, double currentTime)
{
    g_state.bIsRunningInOverlay = isOverlay;
    g_state.bIsSettingsAdvanced = ConfigurationManager::getInstance()->getConfiguration()->advanced_settings;
    g_state.bIgnoreStageTrackingWarning = ConfigurationManager::getInstance()->getConfiguration()->ignore_stage_tracking_warning;
    g_state.bCursorOverriddenThisFrame = false;
    g_state.bBaseStationPowerManagementEnabled = ConfigurationManager::getInstance()->getConfiguration()->base_stations.auto_power_management_enabled;
    g_state.bBaseStationPowerManagementOnStartup = ConfigurationManager::getInstance()->getConfiguration()->base_stations.auto_turn_on_during_startup;
    g_state.bBaseStationPowerManagementOnShutdown = ConfigurationManager::getInstance()->getConfiguration()->base_stations.auto_turn_off_during_shutdown;
    g_state.bBaseStationPowerManagementOffModeIsSleep = !ConfigurationManager::getInstance()->getConfiguration()->base_stations.off_should_use_standby;
    // reset learn page index to home if we're not in the learn page
    if (g_state.dwSelectedUiPage != k_TAB_INDEX_LEARN) {
        g_state.dwSelectedLearnPage = LearnPage_Home;
    }

    // load base station nicknames from config
    if (!g_state.bNicknamesLoaded) {
        g_state.textures[EImageId_LearnStandard_CalibrateDiagram] = renderer::getRenderContext(renderer::GraphicsBackend::OpenGL)->loadTexture(util::getSpaceCalibratorImagesDir() / "img_learn_calibrate_diagram.png");
        g_state.textures[EImageId_LearnContinuous_Mounting] = renderer::getRenderContext(renderer::GraphicsBackend::OpenGL)->loadTexture(util::getSpaceCalibratorImagesDir() / "img_learn_continuous_mount.png");
        g_state.textures[EImageId_LearnBaseStation_10_Front] = renderer::getRenderContext(renderer::GraphicsBackend::OpenGL)->loadTexture(util::getSpaceCalibratorImagesDir() / "img_learn_basestation_10_front.png");
        g_state.textures[EImageId_LearnBaseStation_10_Channel] = renderer::getRenderContext(renderer::GraphicsBackend::OpenGL)->loadTexture(util::getSpaceCalibratorImagesDir() / "img_learn_basestation_10_channel.png");
        g_state.textures[EImageId_LearnBaseStation_20] = renderer::getRenderContext(renderer::GraphicsBackend::OpenGL)->loadTexture(util::getSpaceCalibratorImagesDir() / "img_learn_basestation_20.png");
        g_state.textures[EImageId_LearnUI_Unk0] = renderer::getRenderContext(renderer::GraphicsBackend::OpenGL)->loadTexture(util::getSpaceCalibratorImagesDir() / "img_learn_ui_0.png");
        g_state.textures[EImageId_LearnUI_Unk1] = renderer::getRenderContext(renderer::GraphicsBackend::OpenGL)->loadTexture(util::getSpaceCalibratorImagesDir() / "img_learn_ui_1.png");

        g_state.textures[EImageId_Learn_Preview_Standard] = renderer::getRenderContext(renderer::GraphicsBackend::OpenGL)->loadTexture(util::getSpaceCalibratorImagesDir() / "img_learn_preview_standard.png");
        g_state.textures[EImageId_Learn_Preview_Continuous] = renderer::getRenderContext(renderer::GraphicsBackend::OpenGL)->loadTexture(util::getSpaceCalibratorImagesDir() / "img_learn_preview_continuous.png");
        g_state.textures[EImageId_Learn_Preview_BaseStations] = renderer::getRenderContext(renderer::GraphicsBackend::OpenGL)->loadTexture(util::getSpaceCalibratorImagesDir() / "img_learn_preview_basestations.png");
        g_state.textures[EImageId_Learn_Preview_UITour] = renderer::getRenderContext(renderer::GraphicsBackend::OpenGL)->loadTexture(util::getSpaceCalibratorImagesDir() / "img_learn_preview_uitour.png");

        if (g_state.aBaseStations.capacity() == 0) {
            g_state.aBaseStations.reserve(64);
        }

        const auto& nicknames = ConfigurationManager::getInstance()->getConfiguration()->base_stations.nicknames;
        for (const auto& pair : nicknames) {
            const std::string& serial = pair.first;
            const std::string& nickname = pair.second;

            int baseStationIdx = -1;
            for (size_t i = 0; i < g_state.aBaseStations.size(); i++) {
                if (g_state.aBaseStations[i].szBaseStationId == serial) {
                    baseStationIdx = (int)i;
                    break;
                }
            }

            // station somehow exists already on frame one
            if (baseStationIdx != -1) {
                g_state.aBaseStations[baseStationIdx].szNickname = nickname;
                if (g_state.aBaseStations[baseStationIdx].szNickname.size() < 512) {
                    g_state.aBaseStations[baseStationIdx].szNickname.resize(512, '\0');
                }
            } else {
                // add new entry
                UserInterface_BaseStationState_t& entry = g_state.aBaseStations.emplace_back(UserInterface_BaseStationState_t {
                    .bIsEditing = false,
                    .szBaseStationId = serial,
                    .szNickname = nickname,
                });
                entry.szNickname.resize(512, '\0');
            }
        }

        g_state.bNicknamesLoaded = true;
    }

    auto& io = ImGui::GetIO();

#if _DEBUG
#define IMGUI_USE_DEBUG_WINDOW
#endif

    // disable ctrl + tab, pointless in a VR overlay https://github.com/ocornut/imgui/issues/7987
#if !defined(IMGUI_USE_DEBUG_WINDOW)
    ImGui::Shortcut(ImGuiMod_Ctrl | ImGuiKey_Tab, ImGuiInputFlags_RouteGlobal);
#endif // IMGUI_USE_DEBUG_WINDOW

    constexpr ImGuiWindowFlags k_bareWindowFlags = ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse | ImGuiWindowFlags_NoCollapse
#if !defined(IMGUI_USE_DEBUG_WINDOW)
        | ImGuiWindowFlags_NoNavFocus
#endif // IMGUI_USE_DEBUG_WINDOW
        ;

    ImGui::SetNextWindowPos(ImVec2(0.0f, 0.0f), ImGuiCond_Always);
    ImGui::SetNextWindowSize(io.DisplaySize, ImGuiCond_Always);

    ImGui::Begin("Space Calibrator", nullptr, k_bareWindowFlags);

    drawMainView(currentTime);

    buildVersionInfo();
    ImGui::End();

    // mouse cursor
    if (ImGui::IsAnyItemHovered() && !g_state.bCursorOverriddenThisFrame) {
        ImGui::SetMouseCursor(ImGuiMouseCursor_Hand);
    }

#if defined(IMGUI_USE_DEBUG_WINDOW)
    ImGui::ShowDemoWindow();
#endif // IMGUI_USE_DEBUG_WINDOW
}

void cleanupInterface()
{
    for (size_t i = 0; i < EImageId_Count; i++) {
        renderer::getRenderContext(renderer::GraphicsBackend::OpenGL)->destroyTexture(g_state.textures[i]);
    }
}
}