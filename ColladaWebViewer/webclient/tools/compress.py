#


import os

_compressor = 'compressor/yuicompressor-2.4.7.jar'

def compress(in_files, out_file, in_type='js', verbose=False, temp_file='.temp'):
    temp = open(temp_file, 'w')

    for f in in_files:
        fh = open(f)
        data = fh.read() + '\n'
        fh.close()

        temp.write(data)

        print ' + %s' % f

    temp.close()

    options = ['-o "%s"' % out_file, '--type %s' % in_type]

    if verbose:
        options.append('-v')

    os.system('java -jar "%s" %s "%s"' % (_compressor, ' '.join(options), temp_file))

    org_size = os.path.getsize(temp_file)
    new_size = os.path.getsize(out_file)

    print '=> %s' % out_file
    print 'Original size: %.2f kB' % (org_size / 1024.0)
    print 'Size after compression: %.2f kB' % (new_size / 1024.0)
    print 'Compression rate: %.1f%%' % (float(org_size - new_size) / org_size * 100)
    print ''