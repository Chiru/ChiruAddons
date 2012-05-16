Info
====
3dUiModule
- Tundra plugin that implements the interfaces declared in CieMap and ...
RdfModule
-RDF functionality

Usage Instructions
==================
1) Clone github/Chiru/ChiruAddons/ to <Tundra>/ChiruAddons/
2) Append the contents of 3dUiCMakeBuildConfig.txt to Tundra's CMakeBuildConfig.txt
3) Windows: Copy *.dll from Cie/CieModule/redland to <Tundra>/bin
4) Copy ServiceFusionPlugins.xml and ServiceFusion.xml to <Tundra>/bin/ and run Tundra either with
   "--config ServiceFusionPlugins.xml", or alternatively with "--config ServiceFusion.xml", in addition
   to the default Tundra plugin configuration XMLs.

Building Ubuntu 11.10
=====================
1) Do "Usage Instructions" steps 1, 2 and 4.
3) Run ./build-ubuntu-deps.bash script as normal.
   Note! order to get deps to compile under my laptop, I need to remove PythonQt part from the build-ubuntu-deps.bash script.
3) copy "build-rdf-deps.bash" file to <Tundra>/tools folder.
4) Run build-rdf-deps.bash file.
5) Run ./build-ubuntu-deps.bash script again.
6) For testing you can use <Tundra>/bin/Tundra --config 3dUiPlugins.xml --console --file scenes/ObjectMove/

More details can be found at:
http://www.realxtend.org/doxygen/_build_on_ubuntu.html
