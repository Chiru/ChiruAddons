# Define target name and output directory
init_target(RdfModule OUTPUT plugins)

MocFolder()

# Define source files
file(GLOB CPP_FILES *.cpp)
file(GLOB H_FILES *.h)
file(GLOB MOC_FILES *.h)

set(SOURCE_FILES ${CPP_FILES} ${H_FILES})

QT4_WRAP_CPP(MOC_SRCS ${MOC_FILES})

if (UNIX)
    # Redland RDF lib is named plainly as RDF.
    set(RDF_DIR ${ENV_TUNDRA_DEP_PATH})
else()
    include_directories(redland)
    include_directories(redland/include)

    AddSourceFolder(redland)
    AddSourceFolder(redland/include)
    link_directories(redland/lib)
    set(LIB_FILES libcurl.lib libdb43.lib librdf.lib raptor.lib rasqal.lib)
endif()

UseTundraCore()
use_core_modules(TundraCore)

build_library(${TARGET_NAME} SHARED ${SOURCE_FILES} ${MOC_SRCS})

link_package(QT4)
#link_ogre()
link_modules(TundraCore)

add_definitions(-DRDF_MODULE_EXPORTS)

if (UNIX)
    include_directories(${ENV_TUNDRA_DEP_PATH}/include/raptor2)
    include_directories(${ENV_TUNDRA_DEP_PATH}/include/rasqal)
    target_link_libraries(${TARGET_NAME} optimized rdf)
    target_link_libraries(${TARGET_NAME} debug rdf)
else()
    target_link_libraries(${TARGET_NAME} ${LIB_FILES})
endif()

SetupCompileFlags()

final_target()
