### README ###

Requirements:
1. ColladadaViewerModule + ObjectCaptureModule running on Tundra server
2. A web browser which supports WebGl and WebSockets
3. Remote storage folder with appropriate settings on a web server

NOTE: If you are using Google Chrome browser on linux, you'll have to use --ignore-gpu-blacklist flag to enable WebGl on Chrome.
However, WebGl on FireFox should work without problems.


## Usage ##

This client can be used locally or it can be used remotely on a server.
The only needed files are index.html and app.min.js. Open index.html with your browser to use the client, if you want to use it locally.

# Connecting to the Tundra server via WebSocket

If you have set up the ColladaViewerModule and ObjectCaptureModule correctly, there should be a WebSocket server running on your Tundra server. (Check out the instructions in ColladaViewerModule folder)
The WebSocket server sends refs to the files stored in the remote COLLADA storage and sends notifications if new files were added to the storage.

You can establish your WebSocket connection like this:
1. If running the client locally, write this into your web browsers address bar:
file:///path/to/the/index.html?port=WEBSOCKET_PORT&ip=TUNDRA_SERVER_IP
2. Or if using the client remotely: http://path.to.your/webclient/?port=WEBSOCKET_PORT&ip=TUNDRA_SERVER_IP

If you don't define the port, it defaults to 9002. If you don't define the ip-address it uses 127.0.0.1 as default.


## Remote storage configuration ##

The web client assumes that all the captured COLLADA-files are stored in a remote storage from which it requests the files using XMLHTTP-requests. The remote storage can be located for example in a Apache-server. Here is the correct Apache settings for a remote storage.

Add following settings into your httpd.conf (located in /etc/apache2/ if you are using linux)

<Directory /path/to/your/storage/folder>
  dav on
  Options None
  AllowOverride None
  Allow from all

  # Forcing the MIME-type to be "text/xml"
  DefaultType None
  AddType text/xml .dae

  # Allowing XMLHttpRequests from all domains (You should define specific domains to increase security)
  Header set Access-Control-Allow-Origin "*"

  <Limit POST PUT PROPFIND>
    Order deny,allow
    Deny from all
    # Add your own ip mask or user authentication here..
    Allow from 130.231.12.0/24
  </Limit>

</Directory>

The following Apache modules are needed for this configuration:
mod_dav, mod_headers, mod_mime


## Building and Compressing ##

If you make changes to the JavaScript files of the client (located in src folder), you'll have to build the compressed js-file (app.min.js) with a build.py script which is located in tools folder.

Run it like this using the command line: python build.py

and it updates the app.min.js file located in the web client root folder. If you made changes to the names of the js-files, you'll have to change them in the build.py also.

The script needs Python installed. Also, it utilizes Google Closure Compiler for compressing and obfuscating the JavaScript files, so if you want to use a different version of the compressor, you'll have to change the path to the compressor jar-file in compress.py file accordingly.