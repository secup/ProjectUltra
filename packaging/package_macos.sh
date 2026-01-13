#!/bin/bash
# Package ProjectUltra for macOS as a .app bundle

set -e

APP_NAME="ProjectUltra"
VERSION="0.1.0"
BUILD_DIR="../build"
OUTPUT_DIR="dist/macos"

echo "=== Packaging ProjectUltra for macOS ==="

# Build release version
echo "Building release..."
cd ..
mkdir -p build-release
cd build-release
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j$(sysctl -n hw.ncpu) ultra_gui
cd ../packaging

# Create app bundle structure
echo "Creating app bundle..."
rm -rf "$OUTPUT_DIR"
mkdir -p "$OUTPUT_DIR/${APP_NAME}.app/Contents/MacOS"
mkdir -p "$OUTPUT_DIR/${APP_NAME}.app/Contents/Resources"
mkdir -p "$OUTPUT_DIR/${APP_NAME}.app/Contents/Frameworks"

# Copy executable
cp "../build-release/ultra_gui" "$OUTPUT_DIR/${APP_NAME}.app/Contents/MacOS/${APP_NAME}"

# Create Info.plist
cat > "$OUTPUT_DIR/${APP_NAME}.app/Contents/Info.plist" << EOF
<?xml version="1.0" encoding="UTF-8"?>
<!DOCTYPE plist PUBLIC "-//Apple//DTD PLIST 1.0//EN" "http://www.apple.com/DTDs/PropertyList-1.0.dtd">
<plist version="1.0">
<dict>
    <key>CFBundleExecutable</key>
    <string>${APP_NAME}</string>
    <key>CFBundleIdentifier</key>
    <string>com.projectultra.modem</string>
    <key>CFBundleName</key>
    <string>${APP_NAME}</string>
    <key>CFBundleVersion</key>
    <string>${VERSION}</string>
    <key>CFBundleShortVersionString</key>
    <string>${VERSION}</string>
    <key>CFBundlePackageType</key>
    <string>APPL</string>
    <key>CFBundleSignature</key>
    <string>????</string>
    <key>NSHighResolutionCapable</key>
    <true/>
    <key>NSMicrophoneUsageDescription</key>
    <string>ProjectUltra needs microphone access for receiving audio from your radio.</string>
    <key>LSMinimumSystemVersion</key>
    <string>10.13</string>
</dict>
</plist>
EOF

# Copy SDL2 framework (if using framework version)
if [ -d "/opt/homebrew/opt/sdl2/lib" ]; then
    echo "Bundling SDL2..."
    cp -R /opt/homebrew/opt/sdl2/lib/libSDL2*.dylib "$OUTPUT_DIR/${APP_NAME}.app/Contents/Frameworks/" 2>/dev/null || true

    # Fix library paths
    EXECUTABLE="$OUTPUT_DIR/${APP_NAME}.app/Contents/MacOS/${APP_NAME}"
    for lib in "$OUTPUT_DIR/${APP_NAME}.app/Contents/Frameworks/"*.dylib; do
        if [ -f "$lib" ]; then
            libname=$(basename "$lib")
            install_name_tool -change "/opt/homebrew/opt/sdl2/lib/$libname" "@executable_path/../Frameworks/$libname" "$EXECUTABLE" 2>/dev/null || true
        fi
    done
fi

# Create DMG
echo "Creating DMG..."
DMG_NAME="${APP_NAME}-${VERSION}-macOS.dmg"
rm -f "$OUTPUT_DIR/$DMG_NAME"

# Create a temporary directory for DMG contents
DMG_TEMP="$OUTPUT_DIR/dmg_temp"
rm -rf "$DMG_TEMP"
mkdir -p "$DMG_TEMP"
cp -R "$OUTPUT_DIR/${APP_NAME}.app" "$DMG_TEMP/"
ln -s /Applications "$DMG_TEMP/Applications"

# Create DMG
hdiutil create -volname "$APP_NAME" -srcfolder "$DMG_TEMP" -ov -format UDZO "$OUTPUT_DIR/$DMG_NAME"
rm -rf "$DMG_TEMP"

echo ""
echo "=== macOS Package Complete ==="
echo "App Bundle: $OUTPUT_DIR/${APP_NAME}.app"
echo "DMG: $OUTPUT_DIR/$DMG_NAME"
echo ""
echo "To distribute:"
echo "1. Test the .app on a clean Mac"
echo "2. For wider distribution, consider code signing with:"
echo "   codesign --deep --force --sign \"Developer ID\" ${APP_NAME}.app"
