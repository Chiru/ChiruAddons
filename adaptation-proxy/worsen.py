from PIL import Image
from cStringIO import StringIO
# from multiprocessing import Pool
# from tempfile import mkstemp

# p = Pool(2)

# def worsen_jpeg_filenames(filename):
#     outf = StringIO()
#     im = Image.open(f)
#     out_fobj, out_fn = mkstemp(suffix='.jpg', prefix='worsen_')
#     im.save(out_fobj, 'JPEG', quality=5)
#     out_fobj.close()
#     return out_fn


def worsen_png_rez(data):
    inf = StringIO(data)
    outf = StringIO()
    im = Image.open(inf)
    oldx, oldy = im.size
    newx = oldx/2
    newy = oldy/2
    im.resize((newx, newy), Image.ANTIALIAS)
    im.save(outf, 'PNG')
    return outf.getvalue()

def worsen_png(data, profile):
    ncolors = profile.get('png-colors')
    if not ncolors:
        return data
    inf = StringIO(data)
    outf = StringIO()
    im = Image.open(inf)
    im2 = im.convert('P', palette=Image.ADAPTIVE, colors=ncolors)  
    im2.save(outf, 'PNG', optimize=True)
    return outf.getvalue()

def worsen_jpeg(data):
    q = profile.get('jpeg-quality')
    if not q:
        return data
    inf = StringIO(data)
    outf = StringIO()
    im = Image.open(inf)
    im.save(outf, 'JPEG', quality=q)
    return outf.getvalue()
