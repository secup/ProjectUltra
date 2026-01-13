@echo off
REM Package ProjectUltra for Windows
REM Run this from a Visual Studio Developer Command Prompt

set APP_NAME=ProjectUltra
set VERSION=0.1.0
set OUTPUT_DIR=dist\windows

echo === Packaging ProjectUltra for Windows ===

REM Build release version
echo Building release...
cd ..
if not exist build-release mkdir build-release
cd build-release
cmake .. -DCMAKE_BUILD_TYPE=Release
cmake --build . --config Release --target ultra_gui
cd ..\packaging

REM Create output directory
echo Creating package...
if exist "%OUTPUT_DIR%" rmdir /s /q "%OUTPUT_DIR%"
mkdir "%OUTPUT_DIR%\%APP_NAME%"

REM Copy executable
copy "..\build-release\Release\ultra_gui.exe" "%OUTPUT_DIR%\%APP_NAME%\%APP_NAME%.exe"

REM Copy SDL2 DLL (adjust path as needed)
if exist "C:\SDL2\lib\x64\SDL2.dll" (
    copy "C:\SDL2\lib\x64\SDL2.dll" "%OUTPUT_DIR%\%APP_NAME%\"
) else if exist "%VCPKG_ROOT%\installed\x64-windows\bin\SDL2.dll" (
    copy "%VCPKG_ROOT%\installed\x64-windows\bin\SDL2.dll" "%OUTPUT_DIR%\%APP_NAME%\"
) else (
    echo WARNING: SDL2.dll not found - you need to copy it manually
)

REM Copy Visual C++ Runtime (if not using static linking)
REM Users may need to install VC++ Redistributable

REM Create README
echo ProjectUltra v%VERSION% > "%OUTPUT_DIR%\%APP_NAME%\README.txt"
echo. >> "%OUTPUT_DIR%\%APP_NAME%\README.txt"
echo High-Speed HF Modem - Open Source >> "%OUTPUT_DIR%\%APP_NAME%\README.txt"
echo. >> "%OUTPUT_DIR%\%APP_NAME%\README.txt"
echo To run: Double-click %APP_NAME%.exe >> "%OUTPUT_DIR%\%APP_NAME%\README.txt"
echo. >> "%OUTPUT_DIR%\%APP_NAME%\README.txt"
echo Requirements: >> "%OUTPUT_DIR%\%APP_NAME%\README.txt"
echo - Windows 7 or later >> "%OUTPUT_DIR%\%APP_NAME%\README.txt"
echo - Visual C++ Redistributable 2019 or later >> "%OUTPUT_DIR%\%APP_NAME%\README.txt"

REM Create ZIP
echo Creating ZIP archive...
cd "%OUTPUT_DIR%"
powershell -Command "Compress-Archive -Path '%APP_NAME%' -DestinationPath '%APP_NAME%-%VERSION%-Windows.zip' -Force"
cd ..\..

echo.
echo === Windows Package Complete ===
echo Folder: %OUTPUT_DIR%\%APP_NAME%
echo ZIP: %OUTPUT_DIR%\%APP_NAME%-%VERSION%-Windows.zip
echo.
echo To distribute:
echo 1. Test on a clean Windows machine
echo 2. Include VC++ Redistributable or link statically
