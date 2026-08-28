#include "window.h"
#include "log.h"
#include "platform.h"
#include "renderer/renderer.h"
#include "stb_image.h"
#include "user_interface.h"
#include "util.h"
#include "vr_core.h"

#include <filesystem>
#include <fmt/format.h>

#include "imgui_extensions.h"
#include <imgui/backends/imgui_impl_glfw.h>
#include <imgui/imgui.h>
#include <imgui/imgui_internal.h>

#if OS_WINDOWS
#include <dwmapi.h>
#endif

namespace spacecal {

#if OS_WINDOWS
enum DWMA_USE_IMMSERSIVE_DARK_MODE_ENUM {
    DWMA_USE_IMMERSIVE_DARK_MODE = 20,
    DWMA_USE_IMMERSIVE_DARK_MODE_PRE_20H1 = 19,
};

const bool EnableDarkModeTopBar(const HWND windowHwmd)
{
    const BOOL darkBorder = TRUE;
    return SUCCEEDED(DwmSetWindowAttribute(windowHwmd, DWMA_USE_IMMERSIVE_DARK_MODE, &darkBorder, sizeof(darkBorder)))
        || SUCCEEDED(DwmSetWindowAttribute(windowHwmd, DWMA_USE_IMMERSIVE_DARK_MODE_PRE_20H1, &darkBorder, sizeof(darkBorder)));
}
#endif

void GLFWErrorCallback(int error, const char* description)
{
    LOG_ERROR("GLFW Error ({}): {}", error, description);
}

bool Window::CreateNativeWindow(renderer::GraphicsBackend gfxApi)
{
    if (!glfwInit()) {
        const char* szGlfwError = nullptr;
        glfwGetError(&szGlfwError);
        LOG_FATAL("Failed to initialise GLFW, got {}.", szGlfwError);
        platform::showMessageDialog("An error occured initialising Space Calibrator Nova", "Failed to initialize GLFW");
        return false;
    }

    m_desiredGraphicsBackend = gfxApi;
    if (!renderer::isGraphicsApiSupported(gfxApi)) {
        LOG_FATAL("Unsupported graphics API is selected! Falling back to OpenGL...");
    }
    m_isYFlipped = renderer::isYFlipped(gfxApi);
    m_graphicsContext = renderer::getRenderContext(m_desiredGraphicsBackend);
    ASSERT(m_graphicsContext != nullptr, "Render context must be valid!");

    glfwSetErrorCallback(GLFWErrorCallback);

    // multiple render backends require different hinting params
    m_graphicsContext->enableRendererGlfwHints();
    glfwWindowHint(GLFW_RESIZABLE, false);

    m_glfwWindow = glfwCreateWindow(m_windowWidth, m_windowHeight, "Space Calibrator", nullptr, nullptr);
    if (!m_glfwWindow) {
        LOG_FATAL("Failed to create GLFW window");
        platform::showMessageDialog("An error occured initialising Space Calibrator Nova", "Failed to create GLFW window");
        return false;
    }

    constexpr const char* k_GRAPHICS_API_STRINGS[] = {
        "DirectX11",
        "Vulkan",
        "OpenGL",
    };

    if (!m_graphicsContext->initialiseGraphicsApi(m_glfwWindow)) {
        std::string szApiInitFail = fmt::format("Failed to load {}", k_GRAPHICS_API_STRINGS[(uint32_t)gfxApi]);
        LOG_FATAL("{}", szApiInitFail);
        platform::showMessageDialog("An error occured initialising Space Calibrator Nova", szApiInitFail);
        return false;
    }

    std::string szApiInitFail = fmt::format("Initialised {} renderer!", k_GRAPHICS_API_STRINGS[(uint32_t)gfxApi]);

    // set debug string if necessary
#if _DEBUG
    std::string szDebugTitle = fmt::format("Space Calibrator - {}", k_GRAPHICS_API_STRINGS[(uint32_t)gfxApi]);
    glfwSetWindowTitle(m_glfwWindow, szDebugTitle.c_str());
#endif

    // Minimise the window
    glfwWindowHint(GLFW_VISIBLE, false);

#if OS_WINDOWS
    HWND windowHwnd = glfwGetWin32Window(m_glfwWindow);
    EnableDarkModeTopBar(windowHwnd);
#endif

    // Load icon and set it in the window
    GLFWimage images[1] = {};
    std::string iconPath = (platform::getExeDir() / "taskbar_icon.png").string();
    images[0].pixels = stbi_load(iconPath.c_str(), &images[0].width, &images[0].height, 0, 4);
    glfwSetWindowIcon(m_glfwWindow, 1, images);
    stbi_image_free(images[0].pixels);

    ImGui::CreateContext();
    ImPlot::CreateContext();
    ImPlot3D::CreateContext();
    ImGuiIO& io = ImGui::GetIO();
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableGamepad;
    io.IniFilename = nullptr;

    // load resources
    ImFontConfig cfg;
    // io.Fonts->AddFontFromMemoryCompressedTTF(DroidSans_compressed_data, DroidSans_compressed_size, 24.0f, &cfg);
    // @FIXME: embed fonts later

    static const ImWchar k_RANGE_CORE_LATIN[] = {
        0x0020,
        0x00FF,
        0,
    };
    static const ImWchar k_RANGE_ICONS[] = {
        ICON_MIN_MS,
        ICON_MAX_16_MS,
        0,
    };

    // default font
    std::string fontPath = (util::getSpaceCalibratorInstallDir() / "assets" / "fonts" / "Poppins-Regular.ttf").string();
    cfg.GlyphExcludeRanges = NULL;
    cfg.GlyphOffset.y = 0.0f;
    cfg.MergeMode = false;
    cfg.PixelSnapH = false;
    ImGui::fonts::pDefault = io.Fonts->AddFontFromFileTTF(fontPath.c_str(), 24.0f, &cfg);

    fontPath = (util::getSpaceCalibratorInstallDir() / "assets" / "fonts" / "MPLUS1p-Regular.ttf").string();
    cfg.GlyphExcludeRanges = k_RANGE_CORE_LATIN;
    cfg.GlyphOffset.y = 0.0f;
    cfg.MergeMode = true;
    cfg.PixelSnapH = false;
    io.Fonts->AddFontFromFileTTF(fontPath.c_str(), 24.0f, &cfg);

    fontPath = (util::getSpaceCalibratorInstallDir() / "assets" / "fonts" / "NotoSansSC-Regular.ttf").string();
    cfg.GlyphExcludeRanges = k_RANGE_CORE_LATIN;
    cfg.GlyphOffset.y = 0.0f;
    cfg.MergeMode = true;
    cfg.PixelSnapH = false;
    io.Fonts->AddFontFromFileTTF(fontPath.c_str(), 24.0f, &cfg);

    fontPath = (util::getSpaceCalibratorInstallDir() / "assets" / "fonts" / "NotoSansTC-Regular.ttf").string();
    cfg.GlyphExcludeRanges = k_RANGE_CORE_LATIN;
    cfg.GlyphOffset.y = 0.0f;
    cfg.MergeMode = true;
    cfg.PixelSnapH = false;
    io.Fonts->AddFontFromFileTTF(fontPath.c_str(), 24.0f, &cfg);

    fontPath = (util::getSpaceCalibratorInstallDir() / "assets" / "fonts" / "NotoSansKR-Regular.ttf").string();
    cfg.GlyphExcludeRanges = k_RANGE_CORE_LATIN;
    cfg.GlyphOffset.y = 0.0f;
    cfg.MergeMode = true;
    cfg.PixelSnapH = false;
    io.Fonts->AddFontFromFileTTF(fontPath.c_str(), 24.0f, &cfg);

    fontPath = (util::getSpaceCalibratorInstallDir() / "assets" / "fonts" / "MaterialSymbolsSharp_FILL.ttf").string();
    cfg.GlyphExcludeRanges = k_RANGE_CORE_LATIN;
    cfg.GlyphOffset.y = 4.0f;
    cfg.MergeMode = true;
    cfg.PixelSnapH = true;
    io.Fonts->AddFontFromFileTTF(fontPath.c_str(), 24.0f, &cfg);

    // bold font
    fontPath = (util::getSpaceCalibratorInstallDir() / "assets" / "fonts" / "Poppins-Bold.ttf").string();
    cfg.GlyphExcludeRanges = NULL;
    cfg.GlyphOffset.y = 0.0f;
    cfg.MergeMode = false;
    cfg.PixelSnapH = false;
    ImGui::fonts::pHeading = io.Fonts->AddFontFromFileTTF(fontPath.c_str(), 24.0f, &cfg);

    fontPath = (util::getSpaceCalibratorInstallDir() / "assets" / "fonts" / "MPLUS1p-Bold.ttf").string();
    cfg.GlyphExcludeRanges = k_RANGE_CORE_LATIN;
    cfg.GlyphOffset.y = 0.0f;
    cfg.MergeMode = true;
    cfg.PixelSnapH = false;
    io.Fonts->AddFontFromFileTTF(fontPath.c_str(), 24.0f, &cfg);

    fontPath = (util::getSpaceCalibratorInstallDir() / "assets" / "fonts" / "NotoSansSC-Bold.ttf").string();
    cfg.GlyphExcludeRanges = k_RANGE_CORE_LATIN;
    cfg.GlyphOffset.y = 0.0f;
    cfg.MergeMode = true;
    cfg.PixelSnapH = false;
    io.Fonts->AddFontFromFileTTF(fontPath.c_str(), 24.0f, &cfg);

    fontPath = (util::getSpaceCalibratorInstallDir() / "assets" / "fonts" / "NotoSansTC-Bold.ttf").string();
    cfg.GlyphExcludeRanges = k_RANGE_CORE_LATIN;
    cfg.GlyphOffset.y = 0.0f;
    cfg.MergeMode = true;
    cfg.PixelSnapH = false;
    io.Fonts->AddFontFromFileTTF(fontPath.c_str(), 24.0f, &cfg);

    fontPath = (util::getSpaceCalibratorInstallDir() / "assets" / "fonts" / "NotoSansKR-Bold.ttf").string();
    cfg.GlyphExcludeRanges = k_RANGE_CORE_LATIN;
    cfg.GlyphOffset.y = 0.0f;
    cfg.MergeMode = true;
    cfg.PixelSnapH = false;
    io.Fonts->AddFontFromFileTTF(fontPath.c_str(), 24.0f, &cfg);

    fontPath = (util::getSpaceCalibratorInstallDir() / "assets" / "fonts" / "MaterialSymbolsSharp_FILL.ttf").string();
    cfg.GlyphExcludeRanges = k_RANGE_CORE_LATIN;
    cfg.GlyphOffset.y = 4.0f;
    cfg.MergeMode = true;
    cfg.PixelSnapH = true;
    io.Fonts->AddFontFromFileTTF(fontPath.c_str(), 24.0f, &cfg);

    ImGui_ImplGlfw_InitForOther(m_glfwWindow, true);
    m_graphicsContext->initialiseWindow(m_glfwWindow, m_windowWidth, m_windowHeight);

    ImGui::StyleColorsDark();
    SetupImGuiStyle();

    return true;
}

void Window::SetupImGuiStyle()
{
    // https://github.com/ocornut/imgui/issues/707#issuecomment-4995755614
    ImGuiStyle& style = ImGui::GetStyle();
    ImVec4* colors = style.Colors;

    // Backgrounds
    colors[ImGuiCol_WindowBg] = ImVec4(0.12f, 0.13f, 0.15f, 1.00f); // Dark grey base
    // colors[ImGuiCol_ChildBg] = ImVec4(0.14f, 0.15f, 0.17f, 1.00f);
    colors[ImGuiCol_PopupBg] = ImVec4(0.10f, 0.10f, 0.12f, 0.95f);
    colors[ImGuiCol_Border] = ImVec4(0.30f, 0.33f, 0.42f, 0.40f);

    // Text
    colors[ImGuiCol_Text] = ImVec4(0.90f, 0.93f, 0.95f, 1.00f);
    colors[ImGuiCol_TextDisabled] = ImVec4(0.60f, 0.65f, 0.70f, 1.00f);

    // Headers
    colors[ImGuiCol_Header] = ImVec4(0.36f, 0.42f, 0.55f, 0.60f);
    colors[ImGuiCol_HeaderHovered] = ImVec4(0.44f, 0.50f, 0.68f, 0.80f);
    colors[ImGuiCol_HeaderActive] = ImVec4(0.46f, 0.55f, 0.75f, 1.00f);

    // Buttons
    colors[ImGuiCol_Button] = ImVec4(0.28f, 0.34f, 0.48f, 0.70f);
    colors[ImGuiCol_ButtonHovered] = ImVec4(0.36f, 0.45f, 0.65f, 0.85f);
    colors[ImGuiCol_ButtonActive] = ImVec4(0.40f, 0.50f, 0.70f, 1.00f);

    // Frames
    colors[ImGuiCol_FrameBg] = ImVec4(0.20f, 0.22f, 0.28f, 1.00f);
    colors[ImGuiCol_FrameBgHovered] = ImVec4(0.28f, 0.32f, 0.42f, 1.00f);
    colors[ImGuiCol_FrameBgActive] = ImVec4(0.32f, 0.38f, 0.50f, 1.00f);

    // Tabs
    colors[ImGuiCol_Tab] = ImVec4(0.26f, 0.30f, 0.42f, 0.80f);
    colors[ImGuiCol_TabHovered] = ImVec4(0.36f, 0.42f, 0.58f, 1.00f);
    colors[ImGuiCol_TabActive] = ImVec4(0.42f, 0.50f, 0.68f, 1.00f);
    colors[ImGuiCol_TabUnfocused] = ImVec4(0.20f, 0.24f, 0.32f, 0.80f);
    colors[ImGuiCol_TabUnfocusedActive] = ImVec4(0.30f, 0.36f, 0.50f, 1.00f);

    // Titles
    colors[ImGuiCol_TitleBg] = ImVec4(0.20f, 0.25f, 0.30f, 1.00f);
    colors[ImGuiCol_TitleBgActive] = ImVec4(0.25f, 0.30f, 0.40f, 1.00f);
    colors[ImGuiCol_TitleBgCollapsed] = ImVec4(0.10f, 0.12f, 0.15f, 0.75f);

    // Scrollbars
    colors[ImGuiCol_ScrollbarBg] = ImVec4(0.13f, 0.14f, 0.18f, 1.00f);
    colors[ImGuiCol_ScrollbarGrab] = ImVec4(0.25f, 0.30f, 0.38f, 0.60f);
    colors[ImGuiCol_ScrollbarGrabHovered] = ImVec4(0.35f, 0.40f, 0.50f, 0.80f);
    colors[ImGuiCol_ScrollbarGrabActive] = ImVec4(0.45f, 0.50f, 0.65f, 1.00f);

    // Checkboxes / Radios
    colors[ImGuiCol_CheckMark] = ImVec4(0.80f, 0.85f, 1.00f, 1.00f);

    // Sliders
    colors[ImGuiCol_SliderGrab] = ImVec4(0.50f, 0.65f, 0.90f, 1.00f);
    colors[ImGuiCol_SliderGrabActive] = ImVec4(0.60f, 0.75f, 1.00f, 1.00f);

    // Resize Grip
    colors[ImGuiCol_ResizeGrip] = ImVec4(0.30f, 0.40f, 0.50f, 0.60f);
    colors[ImGuiCol_ResizeGripHovered] = ImVec4(0.40f, 0.50f, 0.60f, 0.80f);
    colors[ImGuiCol_ResizeGripActive] = ImVec4(0.50f, 0.60f, 0.80f, 1.00f);

    // Separator
    colors[ImGuiCol_Separator] = ImVec4(0.35f, 0.40f, 0.48f, 0.7f);
    colors[ImGuiCol_SeparatorHovered] = ImVec4(0.50f, 0.60f, 0.72f, 0.9f);
    colors[ImGuiCol_SeparatorActive] = ImVec4(0.65f, 0.70f, 0.85f, 1.0f);

    // Menus and Tooltips
    colors[ImGuiCol_MenuBarBg] = ImVec4(0.14f, 0.15f, 0.17f, 1.00f);
    // colors[ImGuiCol_TooltipBg]          = ImVec4(0.18f, 0.20f, 0.25f, 0.95f);

    // Drag & Drop
    colors[ImGuiCol_DragDropTarget] = ImVec4(0.50f, 0.85f, 1.00f, 0.90f);

    // Style Metrics
    style.ChildRounding = 6.0f;
    style.FrameRounding = 5.0f;
    style.PopupRounding = 6.0f;
    style.GrabRounding = 4.0f;
    style.TabRounding = 5.0f;

    style.WindowBorderSize = 0.0f;
    style.FrameBorderSize = 0.0f;
    style.PopupBorderSize = 1.0f;

    style.FramePadding = ImVec2(10, 6);
    style.ItemSpacing = ImVec2(10, 10);
    style.ItemInnerSpacing = ImVec2(6, 4);
    style.IndentSpacing = 20.0f;

    style.ScrollbarSize = 20.0f;
    style.ScrollbarRounding = 12.0f;
    style.ScrollbarPadding = 3.0f;
}

void Window::Shutdown()
{
    m_graphicsContext->shutdown();
    ImGui_ImplGlfw_Shutdown();

    ImPlot::DestroyContext();
    ImPlot3D::DestroyContext();
    ImGui::DestroyContext();

    if (m_glfwWindow)
        glfwDestroyWindow(m_glfwWindow);

    glfwTerminate();
}

void Window::RunLoop()
{
    VRState::getInstance()->updateVrState();
    CalibrationManager::getInstance()->init();

    double lastFrameStartTime = glfwGetTime();
    bool bKeyboardOpen = false;
    bool bKeyboardJustClosed = false;
    char textBuf[0x400] = {};

    while (!glfwWindowShouldClose(m_glfwWindow)) {
        double time = glfwGetTime();
        VRState::getInstance()->updateVrState();
        CalibrationManager::getInstance()->calibrationTick(time);

        bool dashboardVisible = false;
        int width = 0, height = 0;
        glfwGetFramebufferSize(m_glfwWindow, &width, &height);
        const bool windowVisible = (width > 0 && height > 0);

        if (VRState::getInstance()->getOverlayHandle()) {
            vr::VROverlayHandle_t hOverlayHandle = VRState::getInstance()->getOverlayHandle();
            // @TODO: SteamVR overlay handling
            auto& io = ImGui::GetIO();
            dashboardVisible = vr::VROverlay()->IsActiveDashboardOverlay(hOverlayHandle);

            // After closing the keyboard, this code waits one frame for ImGui to pick up the new text from SetActiveText
            // before clearing the active widget. Then it waits another frame before allowing the keyboard to open again,
            // otherwise it will do so instantly since WantTextInput is still true on the second frame.
            if (bKeyboardJustClosed && bKeyboardOpen) {
                ImGui::ClearActiveID();
                bKeyboardOpen = false;
            } else if (bKeyboardJustClosed) {
                bKeyboardJustClosed = false;
            } else if (!io.WantTextInput) {
                // User might close the keyboard without hitting Done, so we unset the flag to allow it to open again.
                bKeyboardOpen = false;
            } else if (io.WantTextInput && !bKeyboardOpen && !bKeyboardJustClosed) {
                int id = ImGui::GetActiveID();
                auto textInfo = ImGui::GetInputTextState(id);

                if (textInfo != nullptr) {
                    memset(textBuf, 0, sizeof(textBuf));
                    size_t dwTextLength = std::min(static_cast<size_t>(textInfo->TextLen), sizeof(textBuf) - 1);
                    memcpy(textBuf, textInfo->TextA.Data, dwTextLength);

                    uint32_t unFlags = vr::EKeyboardFlags::KeyboardFlag_Minimal | vr::EKeyboardFlags::KeyboardFlag_ShowArrowKeys; // EKeyboardFlags

                    vr::EVROverlayError err = vr::VROverlay()->ShowKeyboardForOverlay(
                        hOverlayHandle, vr::k_EGamepadTextInputModeNormal, vr::k_EGamepadTextInputLineModeSingleLine,
                        unFlags, "Space Calibrator Overlay", sizeof(textBuf), textBuf, 0);
                    bKeyboardOpen = err == vr::EVROverlayError::VROverlayError_None;
                }
            }

            vr::VREvent_t vrEvent;
            while (vr::VROverlay()->PollNextOverlayEvent(hOverlayHandle, &vrEvent, sizeof(vrEvent))) {
                switch (vrEvent.eventType) {
                case vr::VREvent_MouseMove:
                    io.AddMousePosEvent(vrEvent.data.mouse.x, m_isYFlipped ? m_windowHeight - vrEvent.data.mouse.y : vrEvent.data.mouse.y);
                    break;
                case vr::VREvent_MouseButtonDown:
                    io.AddMouseButtonEvent((vrEvent.data.mouse.button & vr::VRMouseButton_Left) == vr::VRMouseButton_Left ? 0 : 1, true);
                    break;
                case vr::VREvent_MouseButtonUp:
                    io.AddMouseButtonEvent((vrEvent.data.mouse.button & vr::VRMouseButton_Left) == vr::VRMouseButton_Left ? 0 : 1, false);
                    break;
                case vr::VREvent_ScrollDiscrete: {
                    float x = vrEvent.data.scroll.xdelta * 360.0f * 8.0f;
                    float y = vrEvent.data.scroll.ydelta * 360.0f * 8.0f;
                    io.AddMouseWheelEvent(x, m_isYFlipped ? y : y);
                    break;
                }
                case vr::VREvent_KeyboardCharInput: {
                    switch (vrEvent.data.keyboard.cNewInput[0]) {
                    case '\b': {
                        io.AddKeyEvent(ImGuiKey_Backspace, true);
                        io.AddKeyEvent(ImGuiKey_Backspace, false);
                        break;
                    }
                    case '\n': {
                        io.AddKeyEvent(ImGuiKey_Enter, true);
                        io.AddKeyEvent(ImGuiKey_Enter, false);
                        vr::VROverlay()->HideKeyboard();
                        break;
                    }
                    case 27: {
                        if (vrEvent.data.keyboard.cNewInput[1] == 91) {
                            switch (vrEvent.data.keyboard.cNewInput[2]) {
                            case 65: {
                                io.AddKeyEvent(ImGuiKey_UpArrow, true);
                                io.AddKeyEvent(ImGuiKey_UpArrow, false);
                                break;
                            }
                            case 66: {
                                io.AddKeyEvent(ImGuiKey_DownArrow, true);
                                io.AddKeyEvent(ImGuiKey_DownArrow, false);
                                break;
                            }
                            case 67: {
                                io.AddKeyEvent(ImGuiKey_RightArrow, true);
                                io.AddKeyEvent(ImGuiKey_RightArrow, false);
                                break;
                            }
                            case 68: {
                                io.AddKeyEvent(ImGuiKey_LeftArrow, true);
                                io.AddKeyEvent(ImGuiKey_LeftArrow, false);
                                break;
                            }
                            }
                        }
                        break;
                    }
                    default: {
                        io.AddInputCharactersUTF8(vrEvent.data.keyboard.cNewInput);
                        break;
                    }
                    }

                    break;
                }
                case vr::VREvent_KeyboardDone: {
                    // @TODO: fallthrough?
                    if (bKeyboardOpen) {
                        bKeyboardJustClosed = true;
                        vr::VROverlay()->HideKeyboard();
                    }
                    break;
                }
                case vr::VREvent_KeyboardClosed_Global: {
                    if (vrEvent.data.keyboard.overlayHandle == hOverlayHandle) {
                        bKeyboardOpen = false;
                        vr::VROverlay()->HideKeyboard();
                    }
                    break;
                }

                case vr::VREvent_Quit:
                    return;
                }
            }
        }

        if (windowVisible || dashboardVisible) {
            auto& io = ImGui::GetIO();

            // These change state now, so we must execute these before doing our own modifications to the io state for VR
            m_graphicsContext->newFrame();
            ImGui_ImplGlfw_NewFrame();

            io.DisplaySize = ImVec2((float)m_windowWidth, (float)m_windowHeight);
            io.DisplayFramebufferScale = ImVec2(1.0f, 1.0f);

            io.ConfigFlags = io.ConfigFlags & ~ImGuiConfigFlags_NoMouseCursorChange;
            if (dashboardVisible) {
                io.ConfigFlags = io.ConfigFlags | ImGuiConfigFlags_NoMouseCursorChange;
            }

            ImGui::NewFrame();

            spacecal::drawInterface(dashboardVisible, time);

            ImGui::EndFrame();
            ImGui::Render();

            m_graphicsContext->renderDrawData(ImGui::GetDrawData(), ImVec4(0.0f, 0.0f, 0.0f, 1.0f));
            m_graphicsContext->present(m_windowWidth, m_windowHeight);

            // update vr dashboard if viisble
            if (dashboardVisible) {
                vr::Texture_t vrTexture = m_graphicsContext->getVRTexture();

                vr::HmdVector2_t mouseScale = { .v = { (float)m_windowWidth, (float)m_windowHeight } };

                vr::VROverlay()->SetOverlayTexture(VRState::getInstance()->getOverlayHandle(), &vrTexture);
                vr::VROverlay()->SetOverlayMouseScale(VRState::getInstance()->getOverlayHandle(), &mouseScale);
            }
        }

        constexpr double dashboardInterval = 1.0 / 90.0; // fps
        double waitEventsTimeout = std::max(CalibrationManager::getInstance()->getWantedUpdateInterval(), dashboardInterval);

        if (dashboardVisible && waitEventsTimeout > dashboardInterval)
            waitEventsTimeout = dashboardInterval;

        glfwWaitEventsTimeout(waitEventsTimeout);

        // If we're minimized rendering won't limit our frame rate so we need to do it ourselves.
        if (glfwGetWindowAttrib(m_glfwWindow, GLFW_ICONIFIED)) {
            double targetFrameTime = 1 / k_MINIMIZED_MAX_FPS;
            double waitTime = targetFrameTime - (glfwGetTime() - lastFrameStartTime);
            if (waitTime > 0) {
                std::this_thread::sleep_for(std::chrono::duration<double>(waitTime));
            }

            lastFrameStartTime += targetFrameTime;
        }
    }

    spacecal::cleanupInterface();
    CalibrationManager::getInstance()->shutdown();
}
}