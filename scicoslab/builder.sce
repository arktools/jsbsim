mode(-1);

jsbsimIncludeDir=getenv("JSBSim")+'/src';
jsbsimLibDir=getenv("JSBSim")+'/src/.libs';

// check version
ierr = execstr("getversion(""scilab"")", "errcatch");
if ierr == 0 then
    disp("JSBSim doesn''t work with Scilab >= 5. Please use ScicosLab.");
    abort;
end

// build subdirectories
CurrentDirectory = pwd();
mainpathB = get_absolute_file_path("builder.sce");
chdir(mainpathB);
if isdir("src") then
    chdir("src");
    exec("builder.sce");
    chdir("..");
end
if isdir("sci_gateway") then
    chdir("sci_gateway");
    exec("builder.sce");
    chdir("..");
end
if isdir("macros") then
    chdir("macros");
    exec("builder.sce");
    chdir("..");
end
if ~MSDOS & isdir("scicos") then
    chdir("scicos");
	exec("builder.sce");
    chdir("..");
end
chdir(CurrentDirectory);

// clean workspace
clear mainpathB get_absolute_file_path isdir CurrentDirectory ierr
