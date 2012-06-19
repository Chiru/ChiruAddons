Info
====
3dUiModule
- Tundra plugin that implements the interfaces declared in CieMap and exposes needed functionality to Tundra QtScript
RdfModule
-RDF functionality

Usage Instructions
==================
0) Clone Tundra from LudoCraft/Tundra, realXtend/naali or Chiru/naali
1) Clone github/Chiru/ChiruAddons/ to <Tundra>/ChiruAddons/
2) Append the contents of 3dUiCMakeBuildConfig.txt to Tundra's CMakeBuildConfig.txt
3) Windows: Copy *.dll from Cie/CieModule/redland to <Tundra>/bin
3) Ubuntu: See Building Ubuntu 11.10
4) Copy ServiceFusionPlugins.xml and ServiceFusion.xml to <Tundra>/bin/ and run Tundra either with
   "--config ServiceFusionPlugins.xml", or alternatively with "--config ServiceFusion.xml", in addition
   to the default Tundra plugin configuration XMLs.
5) See Scenes/ServiceFusion/ReadMe.txt

Building Ubuntu 11.10
=====================
1) Do "Usage Instructions" steps 0, 1, 2 and 4.
2) Run <Tundra>/tools/build-ubuntu-deps.bash
   Note! order to get deps to compile under my laptop, I need to remove PythonQt part from the build-ubuntu-deps.bash script.
3) Run build-rdf-deps.bash.
4) Run ./build-ubuntu-deps.bash script again.

More details can be found at:
http://www.realxtend.org/doxygen/_build_on_ubuntu.html
