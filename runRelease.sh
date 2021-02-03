export REMAKEN_PKG_ROOT=~/.remaken

# Update configuration files by replacing win-cl-1.1 by linux in module paths
sed -i 's/win-cl-14.1/linux-gcc/' $PWD/$1_conf.xml

ld_library_path="./"
for modulePath in $(grep -o "\$REMAKEN_PKG_ROOT.*lib" $1_conf.xml)
do
   if ! [[ $ld_library_path =~ "$modulePath/x86_64/shared/release" ]]
   then
      ld_library_path=$ld_library_path:$modulePath/x86_64/shared/release
   fi 
done

LD_LIBRARY_PATH=$ld_library_path $1


