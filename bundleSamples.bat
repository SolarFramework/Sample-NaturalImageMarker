@echo off
SETLOCAL EnableDelayedExpansion

SET version=0.9.0

SET filename=SolAR_Fiducial_%version%
SET arg1=%1

IF NOT "!arg1!"=="" (SET filename=%arg1%)
echo filename is %filename%



echo "**** Install dependencies locally"
remaken install packagedependencies.txt
remaken install packagedependencies.txt -c debug

echo "**** Bundle dependencies in bin folder"
 FOR /D /R %%d IN (SolARSample*) DO (
    For %%f IN (%%~fd\*_conf.xml) DO (
      echo "** Bundle sample configuration file %%f"
      remaken bundleXpcf "%%f" -d ./bin/Release -s modules
      remaken bundleXpcf "%%f" -d ./bin/Debug -s modules -c debug
   )
)

FOR /D /R %%d IN (SolARPipeline*) DO (
   For %%f IN (%%~fd\*_conf.xml) DO (
      echo "** Bundle sample configuration file %%f"
      remaken bundleXpcf "%%f" -d ./bin/Release -s modules
      remaken bundleXpcf "%%f" -d ./bin/Debug -s modules -c debug
   )
)


echo "**** Zip bundles"
"7z.exe" a -tzip bin\%filename%_debug.zip README.md
"7z.exe" a -tzip bin\%filename%_release.zip README.md
"7z.exe" a -tzip bin\%filename%_debug.zip LICENSE
"7z.exe" a -tzip bin\%filename%_release.zip LICENSE
"7z.exe" a -tzip bin\%filename%_debug.zip bin\Debug
"7z.exe" a -tzip bin\%filename%_release.zip bin\Release

