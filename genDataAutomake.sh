#!/bin/bash
# run this at the top to generate all data Makefile.am's
# the correct make file

dataDirs="aircraft engine systems"
fileRegex="\(.*\)"

topDir=$(pwd)
echo $topDir
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
		curdir=$(echo $(pwd) | sed -e 's:.*/::g')
		lastdir=$(echo $(pwd) | sed -e "s:\/$curdir::g" -e 's:.*/::g')
		echo "${curdir}_datadir = "$\{"${lastdir}"_datadir}/"${curdir}" >> Makefile.am
		echo "${curdir}_data = \\" >> Makefile.am
		find . -maxdepth 1 -type f -regex $fileRegex \
		| sed -e '/^\.$/d' \
		-e 's:$: \\:g' -e 's:^\./:\t:g' -e '$s:\\::g' >> Makefile.am
		echo "EXTRA_DIST=\$("${curdir}"_data)" >> Makefile.am

	done

done

cd $topDir
#find . -name Makefile.am | sed -e '/^\.$/d' \
	#-e 's:$: \\:g' -e 's/^\..*//g' -e 's:^\./:\t:g' -e '$s:\\::g' 
