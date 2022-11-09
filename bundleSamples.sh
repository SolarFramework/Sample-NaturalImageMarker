
Version="1.0.0"

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
   remaken bundleXpcf --recurse -d ./deploy/bin/x86_64/shared/release -s modules $file
   remaken bundleXpcf --recurse -d ./deploy/bin/x86_64/shared/debug -s modules -c debug $file
done

cp ./runFromBundle.sh ./run.sh
mv ./run.sh ./deploy/bin/x86_64/shared/release/
cp ./runFromBundle.sh ./run.sh
mv ./run.sh ./deploy/bin/x86_64/shared/debug


zip --symlinks -r "./deploy/${filename}_release.zip" ./deploy/bin/x86_64/shared/release ./README.md ./LICENSE
zip --symlinks -r "./deploy/${filename}_debug.zip" ./deploy/bin/x86_64/shared/debug ./README.md ./LICENSE 
