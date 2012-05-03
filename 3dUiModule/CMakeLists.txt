# Define target name and output directory
init_target(3dUiModule OUTPUT plugins)

MocFolder()

# Define source files
file(GLOB CPP_FILES *.cpp)
file(GLOB H_FILES *.h)
file(GLOB MOC_FILES *.h)

set(SOURCE_FILES ${CPP_FILES} ${H_FILES})

QT4_WRAP_CPP(MOC_SRCS ${MOC_FILES})

use_core_modules(Framework)

use_modules(${PROJECT_SOURCE_DIR}/src/Cie/RdfModule)

build_library(${TARGET_NAME} SHARED ${SOURCE_FILES} ${MOC_SRCS})

link_package(QT4)
link_ogre()
link_modules(Framework)

target_link_libraries(${TARGET_NAME} RdfModule ${LIB_FILES})

SetupCompileFlags()

final_target()
