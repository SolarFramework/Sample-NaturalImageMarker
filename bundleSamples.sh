
Version="0.11.0"

if [ -z "$1" ]
then
   filename="SolARSample_NaturalImage_$Version"
else
   filename=$1
fi

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
