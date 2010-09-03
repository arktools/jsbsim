mode(-1)
names=['sci_jsbsimComm'];
files=['sci_jsbsimComm.o'];
libs=[jsbsimLibDir+'/libJSBSim'];
flag='c';
makename='Makelib';
loadername='loader.sce';
libname='jsbsim_sci_gateway';
ldflags='';
cflags='-I../includes -I'+jsbsimIncludeDir;
fflags='';
cc='';
ilib_for_link(names,files,libs,flag,makename,loadername,libname,ldflags,cflags,fflags,cc);

clear names files libs flag makename loadername
clear libname ldflags cflags fflags cc
