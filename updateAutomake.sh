#!/bin/bash
# run this at the top to generate all data Makefile.am's
# it will also update AC_OUTPUT in the configuration file

dataDirs="aircraft engine systems"
fileRegex="\(.*\.\(xml\|ac\|dcm\|blend\)\|.*INSTALL.*\|.*README.*\)"
configFile="configure.in"
extraConfigFiles="jsbsim.pc"

echo generating data makefiles for: $dataDirs
topDir=$(pwd)
for dataDir in $dataDirs
do
	cd $topDir/$dataDir

	for dir in $(find . -type d)
	do
		cd $topDir/$dataDir/$dir

		# delete old Makefile.am
		rm -rf Makefile.am

		# subdirectories
		if [ $(find . -type d | wc -l) -ne 1 ]
		then
			echo "SUBDIRS = \\" > Makefile.am
			find . -maxdepth 1 -type d | sed -e '/^\.$/d' \
			-e 's:$: \\:g' -e 's:^\./:\t:g' -e '$s:\\::g' >> Makefile.am
		fi

		# data packages
		curdir=$(echo $(pwd) | sed -e "s:${topDir}/::g" -e 's:-:_:g')
		curname=$(echo $curdir | sed -e "s:/:_:g")
		echo ${curname}'dir = ${pkgdatadir}/'${curdir} >> Makefile.am
		echo "${curname}_DATA = \\" >> Makefile.am
		find . -maxdepth 1 -type f -regex $fileRegex | sed -e '/^\.$/d' \
		-e 's:$: \\:g' -e 's:^\./:\t:g' -e '$s:\\::g' >> Makefile.am
		echo 'EXTRA_DIST=$('${curname}'_DATA)' >> Makefile.am

	done

done

# update configuration file
echo updating makefile list in: $configFile
cd $topDir

files=$(find . -name Makefile.am | sed -e '/^\.$/d' \
	-e 's:$: \\:g' -e 's:^\./:\t:g' \
	-e '$s:\\::g' -e '/^\t\..*/d' -e 's/\.am//g')
sed -i "/AC_OUTPUT/,/)/ c AC_OUTPUT( \\
	${extraConfigFiles} \\
	${files})" $configFile
