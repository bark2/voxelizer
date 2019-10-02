@echo off
call "C:\Program Files (x86)\Microsoft Visual Studio\2017\Community\VC\Auxiliary\Build\vcvarsall" x86 
set compilerflags /I".\include" /O2 /Zi /EHsc
set linkerflags=/OUT:".\voxelizer.exe" /MACHINE:X86 /SUBSYSTEM:CONSOLE /ERRORREPORT:PROMPT /NOLOGO /DYNAMICBASE "irit.lib"
cl.exe %compilerflags% voxelizer.cpp common.cpp iritSkel.cpp main.cpp obj.cpp /link %linkerflags%
pause