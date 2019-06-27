#TARGET = TestFiducialMarkerPlugin
#VERSION=0.1.0

#CONFIG += c++1z
#CONFIG -= qt
#CONFIG += console

#DEFINES += MYVERSION=$${VERSION}

#CONFIG(debug,debug|release) {
#    DEFINES += _DEBUG=1
#    DEFINES += DEBUG=1
#}

#CONFIG(release,debug|release) {
#    DEFINES += NDEBUG=1
#}

#win32:CONFIG -= static
#win32:CONFIG += shared

#DEPENDENCIESCONFIG = sharedlib recurse
#NOTE : CONFIG as staticlib or sharedlib, DEPENDENCIESCONFIG as staticlib or sharedlib MUST BE DEFINED BEFORE templatelibconfig.pri inclusion
## remove Qt dependencies

QT       -= core gui
CONFIG -= qt

## global defintions : target lib name, version
TARGET = TestNaturalImageMarkerPlugin
FRAMEWORK = $$TARGET
VERSION=0.5.2

DEFINES += MYVERSION=$${VERSION}
DEFINES += TEMPLATE_LIBRARY


CONFIG += c++1z
CONFIG += console


CONFIG(debug,debug|release) {
    DEFINES += _DEBUG=1
    DEFINES += DEBUG=1
}

CONFIG(release,debug|release) {
    DEFINES += _NDEBUG=1
    DEFINES += NDEBUG=1
}

DEPENDENCIESCONFIG = shared recurse

include (../../../../builddefs/qmake/templateappconfig.pri)

HEADERS += \

SOURCES += \
    main.cpp

#unix {
#    LIBS += -ldl
#    QMAKE_CXXFLAGS += -DBOOST_LOG_DYN_LINK
#}

#macx {
#    QMAKE_MAC_SDK= macosx
#    QMAKE_CXXFLAGS += -fasm-blocks -x objective-c++
#}

#win32 {
#    QMAKE_LFLAGS += /MACHINE:X64
#    DEFINES += WIN64 UNICODE _UNICODE
#    QMAKE_COMPILER_DEFINES += _WIN64
#    QMAKE_CXXFLAGS += -wd4250 -wd4251 -wd4244 -wd4275
#}

unix {
}

macx {
    DEFINES += _MACOS_TARGET_
    QMAKE_MAC_SDK= macosx
    QMAKE_CFLAGS += -mmacosx-version-min=10.7 -std=c11 #-x objective-c++
    QMAKE_CXXFLAGS += -mmacosx-version-min=10.7 -std=c11 -std=c++11 -O3 -fPIC#-x objective-c++
    QMAKE_LFLAGS += -mmacosx-version-min=10.7 -v -lstdc++
    LIBS += -lstdc++ -lc -lpthread
}

win32 {

    DEFINES += WIN64 UNICODE _UNICODE
    QMAKE_COMPILER_DEFINES += _WIN64
    QMAKE_CXXFLAGS += -wd4250 -wd4251 -wd4244 -wd4275
}

DISTFILES += \
    PipelineNaturalImageMarker.xml

xpcf_xml_files.path = $$(HOME)/.xpcf
xpcf_xml_files.files=$$files($${PWD}/PipelineNaturalImageMarker.xml)

INSTALLS += xpcf_xml_files
