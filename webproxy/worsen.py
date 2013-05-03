from PIL import Image
from cStringIO import StringIO
from multiprocessing import Pool
from tempfile import mkstemp

p = Pool(2)

def worsen_jpeg_filenames(filename):
    outf = StringIO()
    im = Image.open(f)
    out_fobj, out_fn = mkstemp(suffix='.jpg', prefix='worsen_')
    im.save(out_fobj, 'JPEG', quality=5)
    out_fobj.close()
    return out_fn

def worsen_jpeg(data):
    inf = StringIO(data)
    outf = StringIO()
    im = Image.open(inf)
    im.save(outf, 'JPEG', quality=5)
    return outf.getvalue()
