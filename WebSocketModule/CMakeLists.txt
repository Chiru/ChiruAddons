# Define target name and output directory
init_target (WebSocketModule OUTPUT plugins)

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

use_app_modules (WebSocketModule)
use_package_bullet()
use_core_modules (Framework Math Asset Scene TundraProtocolModule Console OgreRenderingModule PhysicsModule)

build_library (${TARGET_NAME} SHARED ${SOURCE_FILES} ${MOC_SRCS} ${UI_SRCS} ${RESOURCE_SRCS})

link_ogre()
link_modules(Framework Math Asset Scene OgreRenderingModule PhysicsModule Console)
link_entity_components(EC_HoveringText EC_TransformGizmo EC_Highlight EC_LaserPointer EC_Sound
                        EC_ParticleSystem EC_PlanarMirror EC_ProximityTrigger EC_Billboard)
link_entity_components(src/ChiruAddons/EC_Portal)
link_entity_components(src/ChiruAddons/EC_Menu)

#seems to be needed if websocketpp is installed as shared
target_link_libraries(${TARGET_NAME} websocketpp jsoncpp)

SetupCompileFlagsWithPCH()

final_target ()

