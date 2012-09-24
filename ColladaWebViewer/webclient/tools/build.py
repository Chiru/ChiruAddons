#

from compress import *

scripts = [
    '../src/util/detectmobilebrowser.js', # Has to be the first file in compiled file!
    '../src/libs/three.js', # Libraries must be also placed before the js-files that utilize them
    '../src/gui/dat.gui.js',
    '../src/util/Detector.js',
    '../src/util/ColladaLoader.js',
    '../src/util/TrackballControls.js',
    '../src/util/THREEx.FullScreen.js',
    '../src/util/parseUri.js',
    '../src/util/wsmanager.js',
    '../src/main.js',
    ]

out_uncompressed = '../app.js'
out = '../app.min.js'

def main():
    print 'Compressing JavaScript files into %s' % out
    compress(scripts, out, 'js', False, out_uncompressed)

if __name__ == '__main__':
    main()
