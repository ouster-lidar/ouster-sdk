/**
 * Copyright (c) 2026, Ouster, Inc.
 * All rights reserved.
 */

#include "macos_icon.h"

#import <AppKit/AppKit.h>

#include <crt_externs.h>

#include <cstring>
#include <vector>

#include "ouster_icon.h"

// Private LaunchServices SPI used to set the app's session display name (what
// the menu bar shows) for a process with no .app bundle. Weak-linked: if a
// future macOS drops these, the symbols resolve to NULL and set_macos_menu_name()
// simply leaves the default name. Note this does NOT affect the Dock tile or
// Cmd-Tab switcher, which are bound to the executable name at launch.
extern "C" {
CFTypeRef _LSGetCurrentApplicationASN(void) __attribute__((weak_import));
OSStatus _LSSetApplicationInformationItem(int, CFTypeRef, CFStringRef, CFStringRef,
                                          CFDictionaryRef*) __attribute__((weak_import));
extern CFStringRef _kLSDisplayNameKey __attribute__((weak_import));
}

namespace ouster {
namespace sdk {
namespace viz {

void set_macos_app_name() {
    // GLFW reads the program name to build its menu items ("About Ouster Viz" /
    // "Quit Ouster Viz") when there is no app bundle. Must happen before
    // glfwInit(). A static buffer gives the pointer a permanent lifetime.
    static char app_name[] = "Ouster Viz";
    char** progname = _NSGetProgname();
    if (progname != NULL) {
        *progname = app_name;
    }
}

void set_macos_menu_name() {
    // Set the bold application name shown in the menu bar. There is no public
    // API for a bundle-less process, so this uses a weak-linked private
    // LaunchServices call, fully guarded. Must run after glfwInit() (the app is
    // registered by then). Best-effort: if unavailable, the menu bar keeps the
    // interpreter name. Does not affect the Dock tile / Cmd-Tab.
    if (_LSGetCurrentApplicationASN == NULL || _LSSetApplicationInformationItem == NULL) {
        return;
    }
    CFTypeRef asn = _LSGetCurrentApplicationASN();
    if (asn == NULL) {
        return;
    }
    CFStringRef key = (&_kLSDisplayNameKey != NULL && _kLSDisplayNameKey != NULL)
                          ? _kLSDisplayNameKey
                          : CFSTR("LSDisplayName");
    const int kLSDefaultSessionID = -2;
    _LSSetApplicationInformationItem(kLSDefaultSessionID, asn, key, CFSTR("Ouster Viz"), NULL);
}

void set_macos_app_icon() {
    // Render the Ouster logo procedurally, then inset it with rounded corners
    // for a native Dock look (a rounded rectangle -- circular corner arcs, not a
    // true superellipse squircle; Cocoa anti-aliases it).
    const int dim = 256;
    std::vector<unsigned char> rgba = impl::render_ouster_icon(dim);
    NSBitmapImageRep* rep =
        [[NSBitmapImageRep alloc] initWithBitmapDataPlanes:NULL
                                                pixelsWide:dim
                                                pixelsHigh:dim
                                             bitsPerSample:8
                                           samplesPerPixel:4
                                                  hasAlpha:YES
                                                  isPlanar:NO
                                            colorSpaceName:NSDeviceRGBColorSpace
                                               bytesPerRow:dim * 4
                                              bitsPerPixel:32];
    if (rep == nil) {
        return;
    }
    std::memcpy([rep bitmapData], rgba.data(), rgba.size());
    NSImage* src = [[NSImage alloc] initWithSize:NSMakeSize(dim, dim)];
    [src addRepresentation:rep];

    const CGFloat size = dim;
    const CGFloat margin = size * 0.08;   // transparent margin, like a native tile
    const CGFloat side = size - 2 * margin;
    const CGFloat radius = side * 0.225;  // ~ Apple icon corner radius
    NSImage* rounded = [NSImage imageWithSize:NSMakeSize(size, size)
                                      flipped:NO
                               drawingHandler:^BOOL(NSRect /*dstRect*/) {
                                   NSRect rect = NSMakeRect(margin, margin, side, side);
                                   [[NSBezierPath bezierPathWithRoundedRect:rect
                                                                    xRadius:radius
                                                                    yRadius:radius] addClip];
                                   [src drawInRect:rect
                                          fromRect:NSZeroRect
                                         operation:NSCompositingOperationSourceOver
                                          fraction:1.0];
                                   return YES;
                               }];
    [NSApp setApplicationIconImage:rounded];
}

}  // namespace viz
}  // namespace sdk
}  // namespace ouster
