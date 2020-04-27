from building import *
Import('rtconfig')

src   = []
cwd   = GetCurrentDir()

# add adxl345 src files.
if GetDepend('PKG_USING_ADXL345'):
    src += Glob('src/adxl345.c')
if GetDepend('PKG_USING_ADXL345_SAMPLE'):
    src += Glob('samples/adxl345_sample.c')

# add adxl345 include path.
path  = [cwd + '/inc']

# add src and include to group.
group = DefineGroup('adxl345', src, depend = ['PKG_USING_ADXL345'], CPPPATH = path)
Return('group')