echo "Try to estimate SVN revision:"

TRUNK_DIR="/home/ubuntu/Desktop/prototype/simd"
USER_VERSION_TXT="$TRUNK_DIR/prj/txt/UserVersion.txt"
FULL_VERSION_TXT="$TRUNK_DIR/prj/txt/FullVersion.txt"
SIMD_VERSION_H="$TRUNK_DIR/src/Simd/SimdVersion.h"
SIMD_VERSION_H_TXT="$TRUNK_DIR/prj/txt/SimdVersion.h.txt"

if [ -e "$FULL_VERSION_TXT" ]
then
	LAST_VERSION=`cat $FULL_VERSION_TXT`
else
	LAST_VERSION="UNKNOWN"
fi

cp $USER_VERSION_TXT $FULL_VERSION_TXT
which svn > /dev/null
if [ $? -eq 0 ];
then
	SVN_INFO=`svn info $TRUNK_DIR`
	if [ $? -eq 0 ] 
	then
		printf . >>$FULL_VERSION_TXT
		svn info $TRUNK_DIR | grep Revision: | cut -c11->>$FULL_VERSION_TXT
	fi
else
	echo "Subversion is not installed!"
fi
FULL_VERSION=`cat $FULL_VERSION_TXT`

NEED_TO_UPDATE="0"
if [ "$LAST_VERSION" = "$FULL_VERSION" ] 
then
	echo "Last project version '$LAST_VERSION' is equal to current version '$FULL_VERSION'."
else
	echo "Last project version '$LAST_VERSION' is not equal to current version '$FULL_VERSION'."
	NEED_TO_UPDATE="1"
fi

if [ -e "$SIMD_VERSION_H" ] 
then
	echo "File '$SIMD_VERSION_H' is already exist."
else
	echo "File '$SIMD_VERSION_H' is not exist."
	NEED_TO_UPDATE="1"
fi

if [ "$NEED_TO_UPDATE" = "0" ] 
then
	echo "Skip updating of '$SIMD_VERSION_H' file."
else
	echo "Create or update '$SIMD_VERSION_H' file."
	cp $SIMD_VERSION_H_TXT $SIMD_VERSION_H
	sed "-i" "s/@VERSION@/$FULL_VERSION/g" $SIMD_VERSION_H
fi

