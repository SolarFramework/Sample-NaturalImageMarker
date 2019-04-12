cmake_minimum_required(VERSION 3.7.2)

##################################################
set (VERSION_NUMBER "0.5.2")
project("TestNaturalImageMarkerPipeline")
set (SOURCES 
	main.cpp
)


##################################################

# various macros
include("$ENV{BCOMDEVROOT}/bcomBuild/SolARFramework/solarmacros.cmake")
# config setup
setup()
# process packagedependencies.txt
processPackagedependencies()
# define targets (type library)
defineTargets("executable" "")



