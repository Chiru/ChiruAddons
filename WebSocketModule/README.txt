### SETUP ###

Dependencies: Boost 1.46.1, WebSocketpp

1. Install Boost 1.46.1
2. Build WebSocketpp library as a shared library
2.1 Make sure that the paths for boost etc. are correct in the WSpp Makefile.
2.2 make SHARED=1
2.3 sudo make SHARED=1 install
3. Install jsoncpp library (if not yet installed automatically by build-ubuntu-deps.bash)
   sudo apt-get install libjsoncpp-dev
4. At $ChiruNaaliRoot$/cmake/ConfigurePackages.cmake, change line:

"find_package(Boost 1.39.0 COMPONENTS thread regex)" to
"find_package(Boost 1.46.1 COMPONENTS thread regex system date_time program_options)"

if not changed already.

5. At $ChiruNaaliRoot$/CMakeBuildConfig.txt, add "AddProject(src/ChiruAddons/WebSocketModule)" into "Configuring Chiru-Addons"-section

6. Run CMAKE and build project

7. Finally, at $ChiruNaaliRoot$/bin/plugins.xml, add "<plugin path="WebSocketModule" />"-line if not added.


## COMMANDLINE PARAMETERS ##

--wsport <Port>  Sets the WebSocket server port, default port is 9002.
