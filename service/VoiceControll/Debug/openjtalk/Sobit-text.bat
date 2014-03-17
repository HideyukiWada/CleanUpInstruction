@echo off

setlocal
set VOICE=openjtalk\mei_normal.htsvoice
set DICT_DIR=openjtalk\mecab-naist-jdic

set VOICE=%VOICE:\=/%
set DICT_DIR=%DICT_DIR:\=/%

openjtalk\open_jtalk.exe ^
-x "%DICT_DIR%" ^
-m "%VOICE%" ^
-s 16000 ^
-z 6000 ^
-p 80 ^
-a 0.1 ^
-r 0.8 ^
openjtalk\text.txt
