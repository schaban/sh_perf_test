@echo off
setlocal ENABLEEXTENSIONS ENABLEDELAYEDEXPANSION

set BASE_PATH=%ProgramFiles%
if exist "%SystemRoot%\SysWOW64" set BASE_PATH=%ProgramW6432%
set HPATH=%BASE_PATH%\Side Effects Software

for /f "delims=!" %%i in ('dir /b "%HPATH%"') do (
	set PY_PATH=%HPATH%\%%i\python27\python.exe
	if exist !PY_PATH! (
		"!PY_PATH!" shgen.py
		goto :EOF
	)
)