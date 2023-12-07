@echo on

cd ..\Doc
copy ..\Debug\Application.hex ..\Doc\Application.hex

if exist Application.bin del Application.bin
if exist Resource.bin del Resource.bin

srec_cat.exe Application.hex -Intel ^
 -crop 0xC0020000 0xC0100000 ^
 -offset -0xC0020000 ^
 -o Application.bin -binary

srec_cat.exe Application.hex -Intel ^
 -crop 0x90401020 0x90E00000 ^
 -offset -0x90401020 ^
 -o Resource.bin -binary

 srec_cat Bootloader1.bin -binary -offset 0x08000000 ^
 Application.bin -binary -offset 0x90020000 ^
 Resource.bin -binary -offset 0x90401020 ^
 -o Stm32F750.hex -Intel 

 if exist Application.hex del Application.hex