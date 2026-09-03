// clang-format off
#include <stdio.h>
#include <stdlib.h>
#include "util.h"
#include "window.h"
#include "platform.h"
#include "configuration.h"
#include "calibration.h"
#include "localisation.h"
#include "vr_core.h"
#include "log.h"
#include "base_station_management.h"
#include "crash_handler.h"
#if OS_WINDOWS
#include <windows.h>
#endif
// clang-format on

namespace spacecal {
extern std::string g_licenses_text;
}

#if OS_WINDOWS
// http://developer.download.nvidia.com/devzone/devcenter/gamegraphics/files/OptimusRenderingPolicies.pdf
extern "C" __declspec(dllexport) DWORD NvOptimusEnablement = 0x00000001;
// http://developer.amd.com/community/blog/2015/10/02/amd-enduro-system-for-developers/
extern "C" __declspec(dllexport) DWORD AmdPowerXpressRequestHighPerformance = 0x00000001;
#endif

// really bad cli parser
// @FIXME: should probably make a cli parsing lib and use that here eventually, but api design is effort i cba to deal with for now
void args_parse(int argc, char* argv[], spacecal::renderer::GraphicsBackend* renderer)
{
    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--renderer" && (i + 1 < argc) && renderer) {
            std::string selectedApi = argv[i + 1];
            if (selectedApi == "opengl" || selectedApi == "opengles" || selectedApi == "gl" || selectedApi == "gles") {
                *renderer = spacecal::renderer::GraphicsBackend::OpenGL;
            }
#if OS_WINDOWS
            if (selectedApi == "directx11" || selectedApi == "dx11") {
                *renderer = spacecal::renderer::GraphicsBackend::DirectX11;
            }
#endif // OS_WINDOWS
            if (selectedApi == "vulkan" || selectedApi == "vk") {
                *renderer = spacecal::renderer::GraphicsBackend::Vulkan;
            }
        }
    }
}

// cross-platform entry point ; windows needs wWinMain beause we want to hide the terminal at startup
int entry_point(int argc, char* argv[])
{

    // default to dx11 on windows and vk on linux
#if OS_WINDOWS
    // init COM
    (void)CoInitializeEx(NULL, COINIT_MULTITHREADED);
    spacecal::renderer::GraphicsBackend eGraphicsApi = spacecal::renderer::GraphicsBackend::DirectX11;
#else
    spacecal::renderer::GraphicsBackend eGraphicsApi = spacecal::renderer::GraphicsBackend::Vulkan;
#endif

    args_parse(argc, argv, &eGraphicsApi);

    // Space Calibrator has to be single instance to work well with Steam
    bool bIsRunningViaSteam = false;
    if (platform::isAnotherInstanceRunning(bIsRunningViaSteam)) {
        // another instance is running! exit!
        return 0;
    }

    util::init();
    spacecal::init_crash_handler();
    logging::Init(/* isOverlay */ true);

    // Setup config manager
    bool bInitConfigSettings = false;
    spacecal::ConfigurationManager configManager;
    if (configManager.init() == spacecal::ConfigurationError::FileNotExist) {
        bInitConfigSettings = true;
    }

    // Load locale strings
    spacecal::LocalisationManager localisationManager;
    localisationManager.init();

    // Initialise base station management
    spacecal::bluetooth::init_base_station_management();

    // Init SteamVR
    spacecal::VRState vrState;
    if (!vrState.init()) {
        // @TODO: Present error to user in friendly way
        LOG_CRITICAL("Failed to initialise VRState D:");
    } else {
        if (bInitConfigSettings) {
            vr::EVRSettingsError vrErr = vr::EVRSettingsError::VRSettingsError_None;
            bool bEnableBaseStationMgmt = vr::VRSettings()->GetBool(vr::k_pch_Lighthouse_Section, vr::k_pch_Lighthouse_EnableBluetooth_Bool, &vrErr);
        }
    }

    spacecal::Window* theWindow = new spacecal::Window;
    if (!theWindow) {
        platform::showMessageDialog("An error occured initialising Space Calibrator Nova", "Couldn't allocate enough memory for the window!");
        LOG_FATAL("Failed to allocate memory for the main window!");
        return -1;
    }

    if (!theWindow->CreateNativeWindow(eGraphicsApi)) {
        LOG_FATAL("Failed to create native window");
        return -1;
    }

    // init calibration manager or else instance will be nullptr
    spacecal::CalibrationManager* pCalibrationManager = new spacecal::CalibrationManager;

    // load license text
    std::string szLicensesPath = (util::getSpaceCalibratorInstallDir() / "LICENSE").string();
    if (std::filesystem::is_regular_file(szLicensesPath)) {
        FILE* pFile = fopen(szLicensesPath.c_str(), "rb");
        fseek(pFile, 0, SEEK_END);
        size_t fileSize = ftell(pFile);
        spacecal::g_licenses_text.resize(fileSize + 1);
        rewind(pFile);
        size_t bytesRead = fread(spacecal::g_licenses_text.data(), 1, fileSize, pFile);
        fclose(pFile);
        // ensure null terminator
        spacecal::g_licenses_text[bytesRead] = '\0';
        spacecal::g_licenses_text.resize(bytesRead);
    }

    LOG_INFO("Started Space Calibrator Nova!");

    theWindow->RunLoop();

    // close window and save settings to disk
    theWindow->Shutdown();
    spacecal::bluetooth::shutdown_base_station_management();
    spacecal::renderer::shutdownRenderer();
    spacecal::ConfigurationManager::getInstance()->saveConfiguration();

    delete pCalibrationManager;
    delete theWindow;

    // releases global mutex, ie allows other instances to run
    platform::shutdownCurrentInstance();
    spacecal::shutdown_crash_handler();

#if OS_WINDOWS
    CoUninitialize();
#endif

    return 0;
}

// platform specific entry points

#if OS_WINDOWS
int APIENTRY wWinMain(_In_ HINSTANCE hInstance, _In_opt_ HINSTANCE hPrevInstance, _In_ LPWSTR lpCmdLine, _In_ int nCmdShow)
{
    // need to convert from UTF16-LE to UTF8 bc windows is special
    int argc;
    LPWSTR* wargv = CommandLineToArgvW(GetCommandLineW(), &argc);

    std::vector<std::string> arg_strings(argc);
    std::vector<char*> argv(argc);

    for (int i = 0; i < argc; ++i) {
        int size_needed = WideCharToMultiByte(CP_UTF8, 0, wargv[i], -1, NULL, 0, NULL, NULL);
        arg_strings[i].resize(size_needed);
        WideCharToMultiByte(CP_UTF8, 0, wargv[i], -1, &arg_strings[i][0], size_needed, NULL, NULL);
        argv[i] = &arg_strings[i][0];
    }

    int result = entry_point(argc, argv.data());

    LocalFree(wargv);
    return result;
}
#elif OS_LINUX
int main(int argc, char* argv[])
{
    return entry_point(argc, argv);
}
#else
#error "Unsupported platform!"
#endif