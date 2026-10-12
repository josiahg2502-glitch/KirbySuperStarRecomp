@echo off
setlocal
cd /d "%~dp0"
where gcc >nul 2>nul
if not errorlevel 1 goto gcc
where clang >nul 2>nul
if not errorlevel 1 goto clang
where zig >nul 2>nul
if not errorlevel 1 goto zig
where cl >nul 2>nul
if not errorlevel 1 goto msvc
rem Visual Studio Build Tools need their compiler environment loaded first.
set "VSWHERE=%ProgramFiles(x86)%\Microsoft Visual Studio\Installer\vswhere.exe"
if not exist "%VSWHERE%" goto try_pyzig
for /f "usebackq delims=" %%i in (`"%VSWHERE%" -latest -products * -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -property installationPath`) do set "VSROOT=%%i"
if not defined VSROOT goto try_pyzig
call "%VSROOT%\VC\Auxiliary\Build\vcvars64.bat"
if errorlevel 1 goto try_pyzig
goto msvc
:try_pyzig
python -m ziglang version >nul 2>nul
if not errorlevel 1 goto pyzig
py -m ziglang version >nul 2>nul
if not errorlevel 1 goto pyzig_launcher
goto missing
:gcc
gcc -O2 -std=c11 -Isrc src\game_recomp.c src\snes_runtime.c src\snes_main.c -mwindows -lgdi32 -luser32 -lwinmm -o game.exe
exit /b %errorlevel%
:clang
clang -O2 -std=c11 -Isrc src\game_recomp.c src\snes_runtime.c src\snes_main.c -Wl,/subsystem:windows -lgdi32 -luser32 -lwinmm -o game.exe
exit /b %errorlevel%
:zig
zig cc -target x86_64-windows-gnu -O2 -std=c11 -Isrc src\game_recomp.c src\snes_runtime.c src\snes_main.c -Wl,--subsystem,windows -lgdi32 -luser32 -lwinmm -o game.exe
exit /b %errorlevel%
:pyzig
python -m ziglang cc -target x86_64-windows-gnu -O2 -std=c11 -Isrc src\game_recomp.c src\snes_runtime.c src\snes_main.c -Wl,--subsystem,windows -lgdi32 -luser32 -lwinmm -o game.exe
exit /b %errorlevel%
:pyzig_launcher
py -m ziglang cc -target x86_64-windows-gnu -O2 -std=c11 -Isrc src\game_recomp.c src\snes_runtime.c src\snes_main.c -Wl,--subsystem,windows -lgdi32 -luser32 -lwinmm -o game.exe
exit /b %errorlevel%
:msvc
if exist src\game.rc rc /nologo /fo src\game.res src\game.rc
if exist src\game.res (
    cl /nologo /O2 /std:c11 /W3 /Isrc src\game_recomp.c src\snes_runtime.c src\snes_main.c src\game.res user32.lib gdi32.lib winmm.lib /Fe:"Kirby Super Star.exe" /link /SUBSYSTEM:WINDOWS
) else (
    cl /nologo /O2 /std:c11 /W3 /Isrc src\game_recomp.c src\snes_runtime.c src\snes_main.c user32.lib gdi32.lib winmm.lib /Fe:"Kirby Super Star.exe" /link /SUBSYSTEM:WINDOWS
)
copy /y "Kirby Super Star.exe" game.exe >nul
exit /b %errorlevel%
:missing
echo No C compiler found. Install Visual Studio Build Tools with Desktop development with C++, MinGW GCC, or Zig (`pip install ziglang`).
exit /b 1
