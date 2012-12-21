### SETUP ###

Dependencies: Boost 1.46.1, WebSocketpp

1. Install Boost 1.46.1
2. Build WebSocketpp library as a shared library
3. At $ChiruNaaliRoot$/cmake/ConfigurePackages.cmake, change line:

"find_package(Boost 1.39.0 COMPONENTS thread regex)" to
"find_package(Boost 1.46.1 COMPONENTS thread regex system date_time program_options)"

if not changed already.

4. At $ChiruNaaliRoot$/CMakeBuildConfig.txt, add "AddProject(src/ChiruAddons/WebSocketModule)" into "Configuring Chiru-Addons"-section

5. Run CMAKE and build project

6. Finally, at $ChiruNaaliRoot$/bin/plugins.xml, add "<plugin path="WebSocketModule" />"-line if not added.


## COMMANDLINE PARAMETERS ##

--wsport <Port>  Sets the WebSocket server port
