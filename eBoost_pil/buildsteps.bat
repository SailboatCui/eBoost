@echo off

if not defined PILPREPTOOL (
   for /f "tokens=2*" %%A in ('reg query HKCU\Software\Plexim\PLECS /v PILFrameworkPath 2^>NUL ^| findstr "REG_SZ"') do (
      @set PILROOT=%%~B%
   )
   @set "PILPREPTOOL=!PILROOT!\Tools\1.0\win32\pilpreptool.exe"
) 

if defined ProgramFiles(x86) (
   set C2PROG_EXE="%ProgramFiles(x86)%\C2Prog\C2ProgShell.exe"
) else (
   set C2PROG_EXE="%ProgramFiles%\C2Prog\C2ProgShell.exe"
)

set BUILDSTEP=step_%1
shift

set COMPILERPATH=%~1
shift

set PROJECTNAME=%~1
shift

goto %BUILDSTEP%

:step_0 
    setlocal EnableDelayedExpansion
   
    rem   Reconstruct the CFLAGS argument, which may be split into multiple
    rem   arguments if there are spaces in include_paths.
    rem   NOTE: This will still break if there are other argument separators
    rem         (comma, semicolon, equal sign, tab character) in include_paths.
    set CFLAGS=
:cflag_start
    if -%1-==-- goto cflag_end
    if %1==PLECS_PIL_SEPARATOR goto cflag_end
    set CFLAG=%1
    set CFLAGS=%CFLAGS% %CFLAG:\"="%
    shift
    goto cflag_start
:cflag_end
    rem   chop first 2 characters ( ") and last character (")
    set CFLAGS=%CFLAGS:~2,-1%

    rem   shift out separator token
    if %1==PLECS_PIL_SEPARATOR shift

    rem   Reconstruct the C_SRCS argument, which may be split into multiple
    rem   arguments if there are spaces in file names.
    set C_SRCS=
:csrc_start
    if -%1-==-- goto csrc_end
    if %C_SRC:~-1%==\ (
        rem   If the previous filename ended with a backslash,
        rem   extend it instead of beginning a new filename 
        set C_SRC=%C_SRC:~0,-1% %1
    ) else (
        set C_SRC=%1
    )
    if not %C_SRC:~-1%==\ (
        rem   If the current filename does not end with a backslash,
        rem   enclose it in quotes and append it
        set C_SRCS=%C_SRCS% "%C_SRC%"
    )
    shift
    goto csrc_start
:csrc_end

    rem   Run C preprocessor on all source files to inline included files
    rem   and expand macros
    set PILPREPFILES=
    for %%i in (%C_SRCS%) do (
       "%COMPILERPATH%/bin/cl2000" %CFLAGS% -DPIL_PREP_TOOL --preproc_only %%i
       move "%%~pni.pp" "%%~ni.pilpp" >NUL
       set PILPREPFILES=!PILPREPFILES! "%%~ni.pilpp"
    )
    
    rem   Run pilpreptool on all preprocessed files and delete them afterwards
    "%PILPREPTOOL%" %PILPREPFILES% -k "cregister __cregister interrupt __interrupt __inline far" -postfix -o ..\pil_symbols
    del %PILPREPFILES%
    goto end_step
    
:step_1
   set C2PROGTARGET=%1
   rem trim \" surrounding target name, e.g. for \"28069,67,66_JTAG\"
   set C2PROGTARGET=%C2PROGTARGET:"=%
   set C2PROGTARGET=%C2PROGTARGET:\=%  
   if exist %C2PROG_EXE% (
      "%COMPILERPATH%/bin/hex2000" -romwidth 16 -memwidth 16 -i -o %PROJECTNAME%.hex %PROJECTNAME%.out
      if  errorlevel  1  goto  error
      %C2PROG_EXE% -hex=%PROJECTNAME%.hex -ehx=%PROJECTNAME%.ehx -target=%C2PROGTARGET%
      if  errorlevel  1  goto  error
   )
:end_step

if  errorlevel  1  goto  error
exit 0

:error
exit 1