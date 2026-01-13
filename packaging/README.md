# ProjectUltra Packaging Guide

## Quick Package Commands

### macOS
```bash
cd packaging
./package_macos.sh
```
Output: `dist/macos/ProjectUltra.app` and `ProjectUltra-0.1.0-macOS.dmg`

### Windows
```batch
cd packaging
package_windows.bat
```
Output: `dist/windows/ProjectUltra/` folder and `.zip`

### Linux
```bash
cd packaging
./package_linux.sh
```
Output: `dist/linux/ProjectUltra-0.1.0-x86_64.AppImage`

---

## Prerequisites

### macOS
- Xcode Command Line Tools
- Homebrew
- SDL2: `brew install sdl2`

### Windows
- Visual Studio 2019+
- CMake
- SDL2 (via vcpkg or manual install)
  ```
  vcpkg install sdl2:x64-windows
  ```

### Linux
- GCC/G++
- CMake
- SDL2: `sudo apt install libsdl2-dev`
- For AppImage: wget

---

## Distribution Checklist

### Before Release
- [ ] Update version in `CMakeLists.txt`
- [ ] Update version in packaging scripts
- [ ] Test on clean machines
- [ ] Run full simulation test suite

### macOS
- [ ] Test on macOS 10.13+ (High Sierra)
- [ ] Consider code signing for Gatekeeper
- [ ] Consider notarization for macOS 10.15+

### Windows
- [ ] Test on Windows 7, 10, 11
- [ ] Include VC++ Redistributable or link statically
- [ ] Test with Windows Defender

### Linux
- [ ] Test AppImage on Ubuntu, Fedora, Debian
- [ ] Verify audio device detection
- [ ] Check OpenGL compatibility

---

## GitHub Release

1. Create a new release tag:
   ```bash
   git tag -a v0.1.0 -m "Release 0.1.0"
   git push origin v0.1.0
   ```

2. Build packages on each platform

3. Upload to GitHub Releases:
   - ProjectUltra-0.1.0-macOS.dmg
   - ProjectUltra-0.1.0-Windows.zip
   - ProjectUltra-0.1.0-x86_64.AppImage

---

## File Sizes (Approximate)

| Platform | Size |
|----------|------|
| macOS .app | ~5 MB |
| macOS .dmg | ~3 MB |
| Windows .zip | ~4 MB |
| Linux AppImage | ~8 MB |
