@echo off
rem Build the firmware core as a Windows DLL for the digital twin (needs Visual Studio Build Tools).
cd /d "%~dp0"
call "C:\Program Files (x86)\Microsoft Visual Studio\2022\BuildTools\VC\Auxiliary\Build\vcvars64.bat" >nul || exit /b 1
cl /nologo /O2 /EHsc /D_CRT_SECURE_NO_WARNINGS /LD ..\sma_rig\rig_core.cpp rig_core_capi.cpp /Fe:rig_core.dll
