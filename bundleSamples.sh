
Version="0.10.0"

if [ -z "$1" ]
then
   filename="SolAR_NaturalImage_$Version"
else
   filename=$1
fi

# Update configuration files by replacing win-cl-1.1 by linux in module paths
echo "**** Update module path in configuration file (win-cl-14.1 -> linux-gcc)"
sed -i 's/win-cl-14.1/linux-gcc/' $PWD/bin/Debug/*_conf.xml
sed -i 's/win-cl-14.1/linux-gcc/' $PWD/bin/Release/*_conf.xml
sed -i 's/win-cl-14.1/linux-gcc/' $PWD/SolARSample*/*_conf.xml
sed -i 's/win-cl-14.1/linux-gcc/' $PWD/SolARPipeline*/tests/SolARPipelineTest*/*_conf.xml

echo "**** Install dependencies locally"
remaken install packagedependencies.txt
remaken install packagedependencies.txt -c debug

echo "**** Bundle dependencies in bin folder"
for file in $(find ./SolARSample* ./SolARPipeline*/tests/SolARPipelineTest* -path "*_conf.xml")
do
   echo "install dependencies for config file: $file"
   remaken bundleXpcf $file -d ./bin/Release -s modules
   remaken bundleXpcf $file -d ./bin/Debug -s modules -c debug
done

cp ./runFromBundle.sh ./run.sh
mv ./run.sh ./bin/Release/
cp ./runFromBundle.sh ./run.sh
mv ./run.sh ./bin/Debug


zip --symlinks -r "./bin/${filename}_release.zip" ./bin/Release ./README.md ./LICENSE
zip --symlinks -r "./bin/${filename}_debug.zip" ./bin/Debug ./README.md ./LICENSE 
