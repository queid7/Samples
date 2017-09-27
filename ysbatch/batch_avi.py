import os

dumppath = '../../dump'

for subname in os.listdir(dumppath):
    subpath = os.path.join(dumppath,subname)

    if not os.path.isdir(subpath):
        continue

    inpath = subpath+'/00001.jpg'
    outpath = dumppath+'/'+subname+'.avi'

    print inpath

    cmd = 'avidemux2_cli --nogui --load %s --fps %d --save %s' \
            %(inpath, 30, outpath)
    os.system(cmd)
    #print cmd

    print '-> '+outpath
