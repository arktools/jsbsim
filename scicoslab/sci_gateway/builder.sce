mode(-1)
names=['sci_jsbsimComm','sci_jsbsimTrim','sci_serial'];
files=['sci_jsbsimComm.o','sci_jsbsimTrim.o','sci_serial.o','utilities.o','AsyncSerial.o'];
libs=[jsbsimLibDir+'/libJSBSim'];
flag='c';
makename='Makelib';
loadername='loader.sce';
libname='jsbsim_sci_gateway';
ldflags='-lboost_system-mt -lboost_thread-mt -lboost_date_time-mt';
cflags='-I../includes -I'+jsbsimIncludeDir;
fflags='';
cc='';
ilib_for_link(names,files,libs,flag,makename,loadername,libname,ldflags,cflags,fflags,cc);

clear names files libs flag makename loadername
clear libname ldflags cflags fflags cc
