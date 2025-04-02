@REM @echo off
set version=GraphExecuter1.10
pyinstaller -w main.py -i settings\myicon.png  -n %version% -y
cd .\docs\userguide\
mkdocs build
cd ../../
@REM echo D|xcopy res dist\%version%\res /s /e /h /k /y
echo D|xcopy settings dist\%version%\settings /s /e /h /k /y
echo D|xcopy src\GraphFlow dist\%version%\_internal\src\GraphFlow /s /e /h /k /y
echo D|xcopy docs\userguide\site dist\%version%\docs\userguide\site /s /e /h /k /y
del %version%.spec

