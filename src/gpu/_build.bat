@echo off
setlocal ENABLEEXTENSIONS ENABLEDELAYEDEXPANSION

set _FXC_="%ProgramFiles(x86)%\Windows Kits\8.1\bin\x86\fxc.exe" /nologo
goto :start

:build_cs
set NAME=%1
%_FXC_% /O3 /Gfp /Tcs_5_0 /Vn%NAME% /Fh%NAME%.h %NAME%.hlsl
goto :EOF

:start
call :build_cs cs_shapply
