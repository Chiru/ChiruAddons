# This compression script combines all the JavaScript files together and then compresses and optimizes
# the resulted file with Google Closure Compiler


import os

_compressor = 'compressor/compiler.jar'

def compress(in_files, out_file, verbose=False, temp_file='.temp'):
    temp = open(temp_file, 'w')

    for f in in_files:
        fh = open(f)
        data = fh.read() + '\n'
        fh.close()

        temp.write(data)

        print ' + %s, %.2f kB' % (f, (os.path.getsize(f)/1024.0))

    temp.close()

    options = ['--js %s' %temp_file, '--jscomp_off=globalThis', '--language_in=ECMASCRIPT5_STRICT',
               '--jscomp_off=checkTypes', '--js_output_file %s' %out_file]

    if verbose:
        options.append('--warning_level=VERBOSE')
    else:
        options.append('--warning_level=QUIET')

    os.system('java -jar "%s" %s' % (_compressor, ' '.join(options)))


    org_size = os.path.getsize(temp_file)
    new_size = os.path.getsize(out_file)

    print '=> %s' % out_file
    print 'Original size: %.2f kB' % (org_size / 1024.0)
    print 'Size after compression: %.2f kB' % (new_size / 1024.0)
    print 'Compression rate: %.1f%%' % (float(org_size - new_size) / org_size * 100)
    print ''