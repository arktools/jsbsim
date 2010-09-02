mode(-1)
names=['sci_jsbSimComm'];
files=['sci_jsbSimComm.o'];
libs=[JSBSimLibDir+'/libJSBSim'];
flag='c';
makename='Makelib';
loadername='loader.sce';
libname='JSBSim_sci_gateway';
ldflags='';
cflags='-I../includes -I'+JSBSimIncludeDir;
fflags='';
cc='';
ilib_for_link(names,files,libs,flag,makename,loadername,libname,ldflags,cflags,fflags,cc);

clear names files libs flag makename loadername
clear libname ldflags cflags fflags cc
