#!/bin/bash
# Package ProjectUltra for Linux as AppImage

set -e

APP_NAME="ProjectUltra"
VERSION="0.1.0"
OUTPUT_DIR="dist/linux"
APPDIR="$OUTPUT_DIR/${APP_NAME}.AppDir"

echo "=== Packaging ProjectUltra for Linux ==="

# Build release version
echo "Building release..."
cd ..
mkdir -p build-release
cd build-release
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j$(nproc) ultra_gui
cd ../packaging

# Create AppDir structure
echo "Creating AppDir..."
rm -rf "$OUTPUT_DIR"
mkdir -p "$APPDIR/usr/bin"
mkdir -p "$APPDIR/usr/lib"
mkdir -p "$APPDIR/usr/share/applications"
mkdir -p "$APPDIR/usr/share/icons/hicolor/256x256/apps"

# Copy executable
cp "../build-release/ultra_gui" "$APPDIR/usr/bin/${APP_NAME}"

# Copy shared libraries
echo "Bundling libraries..."
for lib in libSDL2 libGL; do
    ldd "../build-release/ultra_gui" | grep "$lib" | awk '{print $3}' | while read libpath; do
        if [ -f "$libpath" ]; then
            cp "$libpath" "$APPDIR/usr/lib/" 2>/dev/null || true
        fi
    done
done

# Create desktop file
cat > "$APPDIR/${APP_NAME}.desktop" << EOF
[Desktop Entry]
Type=Application
Name=ProjectUltra
Comment=High-Speed HF Modem
Exec=ProjectUltra
Icon=projectultra
Categories=HamRadio;Audio;
Terminal=false
EOF

cp "$APPDIR/${APP_NAME}.desktop" "$APPDIR/usr/share/applications/"

# Create simple icon (placeholder - replace with real icon)
cat > "$APPDIR/usr/share/icons/hicolor/256x256/apps/projectultra.svg" << 'EOF'
<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 100 100">
  <circle cx="50" cy="50" r="45" fill="#2a5298"/>
  <text x="50" y="60" text-anchor="middle" fill="white" font-size="24" font-family="sans-serif">Ultra</text>
</svg>
EOF

# Create AppRun
cat > "$APPDIR/AppRun" << 'EOF'
#!/bin/bash
SELF=$(readlink -f "$0")
HERE=${SELF%/*}
export LD_LIBRARY_PATH="${HERE}/usr/lib:${LD_LIBRARY_PATH}"
exec "${HERE}/usr/bin/ProjectUltra" "$@"
EOF
chmod +x "$APPDIR/AppRun"

# Create symlinks for AppImage
ln -sf usr/share/icons/hicolor/256x256/apps/projectultra.svg "$APPDIR/projectultra.svg"
ln -sf usr/share/icons/hicolor/256x256/apps/projectultra.svg "$APPDIR/.DirIcon"

# Download appimagetool if not present
if [ ! -f "appimagetool-x86_64.AppImage" ]; then
    echo "Downloading appimagetool..."
    wget -q "https://github.com/AppImage/AppImageKit/releases/download/continuous/appimagetool-x86_64.AppImage"
    chmod +x appimagetool-x86_64.AppImage
fi

# Create AppImage
echo "Creating AppImage..."
ARCH=x86_64 ./appimagetool-x86_64.AppImage "$APPDIR" "$OUTPUT_DIR/${APP_NAME}-${VERSION}-x86_64.AppImage"

echo ""
echo "=== Linux Package Complete ==="
echo "AppImage: $OUTPUT_DIR/${APP_NAME}-${VERSION}-x86_64.AppImage"
echo ""
echo "To distribute:"
echo "1. Test on different Linux distros"
echo "2. Upload to GitHub Releases"
echo ""
echo "Users can run with:"
echo "  chmod +x ${APP_NAME}-${VERSION}-x86_64.AppImage"
echo "  ./${APP_NAME}-${VERSION}-x86_64.AppImage"
