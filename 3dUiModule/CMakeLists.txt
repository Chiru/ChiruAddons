# Define target name and output directory
init_target(3dUiModule OUTPUT plugins)

# Define source files
file(GLOB CPP_FILES *.cpp)
file(GLOB H_FILES *.h)
file(GLOB MOC_FILES *.h CieMap/*.h)
AddSourceFolder(CieMap)

MocFolder()

set(SOURCE_FILES ${CPP_FILES} ${H_FILES})

QT4_WRAP_CPP(MOC_SRCS ${MOC_FILES})

UseTundraCore()
use_core_modules(TundraCore Math)

use_modules(${PROJECT_SOURCE_DIR}/src/ChiruAddons/RdfModule)
#use_modules(${PROJECT_SOURCE_DIR}/src/ChiruAddons/3dUiModule/CieMap)

build_library(${TARGET_NAME} SHARED ${SOURCE_FILES} ${MOC_SRCS})

link_package(QT4)
link_ogre()
link_modules(TundraCore)

target_link_libraries(${TARGET_NAME} RdfModule ${LIB_FILES})

SetupCompileFlags()

final_target()
