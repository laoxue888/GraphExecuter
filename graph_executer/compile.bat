@REM @echo off
set version=GraphExecuter_1.10
@REM pyinstaller -w --exclude PyQt6 main.py -i settings\myicon.png  -n %version% -y
pyinstaller -w main.py -i settings\myicon.png  -n %version% -y --exclude PyQt6
@REM cd .\docs\userguide\
@REM mkdocs build
@REM cd ../../
echo D|xcopy res dist\%version%\res /s /e /h /k /y
echo D|xcopy settings dist\%version%\_internal\settings /s /e /h /k /y
echo D|xcopy src\GraphFlow dist\%version%\_internal\src\GraphFlow /s /e /h /k /y
echo D|xcopy nodes dist\%version%\_internal\nodes /s /e /h /k /y
@REM @REM echo D|xcopy utils dist\%version%\utils /s /e /h /k /y
@REM @REM echo D|xcopy logs dist\%version%\logs /s /e /h /k /y
@REM echo D|xcopy docs\userguide\site dist\%version%\docs\userguide\site /s /e /h /k /y
@REM echo D|xcopy docs\examples_graph dist\%version%\docs\examples_graph /s /e /h /k /y
del %version%.spec

