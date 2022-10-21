@echo off
SETLOCAL EnableDelayedExpansion

SET version=1.0.0

SET filename=SolARSample_Fiducial_%version%
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
      remaken bundleXpcf "%%f" -d ./deploy/bin/x86_64/shared/release -s modules
      remaken bundleXpcf "%%f" -d ./deploy/bin/x86_64/shared/debug -s modules -c debug
   )
)

FOR /D /R %%d IN (SolARPipeline*) DO (
   For %%f IN (%%~fd\*_conf.xml) DO (
      echo "** Bundle sample configuration file %%f"
      remaken bundleXpcf "%%f" -d ./deploy/bin/x86_64/shared/release -s modules
      remaken bundleXpcf "%%f" -d ./deploy/bin/x86_64/shared/debug -s modules -c debug
   )
)


echo "**** Zip bundles"
"7z.exe" a -tzip deploy\%filename%_debug.zip README.md
"7z.exe" a -tzip deploy\%filename%_release.zip README.md
"7z.exe" a -tzip deploy\%filename%_debug.zip LICENSE
"7z.exe" a -tzip deploy\%filename%_release.zip LICENSE
"7z.exe" a -tzip deploy\%filename%_debug.zip deploy\bin\x86_64\shared\debug
"7z.exe" a -tzip deploy\%filename%_release.zip deploy\bin\x86_64\shared\release

