/**
 * Copyright (c) 2026, Ouster, Inc.
 * All rights reserved.
 */

#pragma once

// macOS-only helpers for branding the viz application. macOS has no per-window
// icon and derives the menu-bar / Dock name from the process or app bundle;
// when ouster-cli runs under the bare Python interpreter there is no bundle, so
// these set the icon and name programmatically. All are implemented in
// macos_icon.mm and compiled only on Apple platforms.

namespace ouster {
namespace sdk {
namespace viz {

// Override the program name to "Ouster Viz" so GLFW builds its menu items
// ("About/Quit Ouster Viz"). MUST be called BEFORE glfwInit().
void set_macos_app_name();

// Set the bold application name shown in the menu bar to "Ouster Viz" via a
// guarded private LaunchServices call. Call AFTER glfwInit(). Does not affect
// the Dock tile or Cmd-Tab switcher (those are fixed to the executable name at
// launch and need a real .app bundle to change).
void set_macos_menu_name();

// Set the application (Dock) icon to the procedurally-rendered Ouster logo.
// Must be called after glfwInit() (which creates the shared NSApplication).
void set_macos_app_icon();

}  // namespace viz
}  // namespace sdk
}  // namespace ouster
