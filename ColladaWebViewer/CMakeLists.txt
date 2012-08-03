# Define target name and output directory
init_target (ColladaViewerModule OUTPUT plugins)

MocFolder ()

# Define source files
file (GLOB CPP_FILES *.cpp)
file (GLOB H_FILES *.h)
file (GLOB XML_FILES *.xml)

#file (GLOB MOC_FILES ColladaViewerModule.h)


set (SOURCE_FILES ${SOURCE_FILES} ${CPP_FILES} ${H_FILES})
set (FILES_TO_TRANSLATE ${FILES_TO_TRANSLATE} ${H_FILES} ${CPP_FILES} PARENT_SCOPE)

# Qt4 Wrap 
QT4_WRAP_CPP(MOC_SRCS ${H_FILES})

use_app_modules (ColladaViewerModule)
use_core_modules (Framework Math TundraProtocolModule AssetModule)

build_library (${TARGET_NAME} SHARED ${SOURCE_FILES} ${MOC_SRCS} ${UI_SRCS} ${RESOURCE_SRCS})

link_modules (Framework TundraProtocolModule AssetModule)

#seems to be needed if websocketpp is installed as shared
target_link_libraries(${TARGET_NAME} websocketpp)

SetupCompileFlagsWithPCH()

final_target ()

