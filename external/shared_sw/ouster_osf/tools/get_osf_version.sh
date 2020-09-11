#!/bin/bash

# set -x

osfFile=${1}

# dd if=$osfFile bs=1 skip=0 count=100 status=none | od -A x -t x1z
# headerSize=$(dd if=$osfFile bs=1 skip=0 count=4 status=none | od -An -tu4 | awk '{$1=$1;print $1}')
# osfHeaderRoot=$(dd if=$osfFile bs=1 skip=4 count=4 status=none | od -An -tu4 | awk '{$1=$1;print $1}')

declare -A headerVT
headerVT=(["version"]="4" ["status"]="6" ["session_offset"]="8" ["file_length"]="10")

# echo "v = " ${headerVT[version]}

headerSize=$(od -An -tu4 -N 4 -j 0 $osfFile | awk '{$1=$1;print $1}')
headerRootOffset=$(od -An -tu4 -N 4 -j 4 $osfFile | awk '{$1=$1;print $1}')
headerRootOffset=$(($headerRootOffset+4))

echo "Hello OSF!"

echo "Header Size: $headerSize"
echo "Header Root Offset: $headerRootOffset"

chunkOffset=$(($headerSize+4))

echo "chunkOffset = $chunkOffset"

echo "Header (sp):"
od -A x -t x1z -N $(($headerSize+4)) -j 0 $osfFile

headerVTOffset=$(od -An -tu2 -N 4 -j $headerRootOffset $osfFile | awk '{$1=$1;print $1}')
headerVTOffset=$(($headerRootOffset-$headerVTOffset))

echo "Header VTable:"
# od -A x -t x1z -N $((4+4*2)) -j $(($osfHeaderRoot+4-$headerVTOffset)) $osfFile
od -A x -t x1z -N $((4+4*2)) -j $headerVTOffset $osfFile

# versionOffset=$(od -An -tu2 -N 2 -j $(($osfHeaderRoot+4-$headerVTOffset+4)) $osfFile | awk '{$1=$1;print $1}')
versionOffset=$(od -An -tu2 -N 2 -j $(($headerVTOffset+${headerVT[version]})) $osfFile | awk '{$1=$1;print $1}')

# statusOffset=$(od -An -tu2 -N 2 -j $(($osfHeaderRoot+4-$headerVTOffset+4+2)) $osfFile | awk '{$1=$1;print $1}')
statusOffset=$(od -An -tu2 -N 2 -j $(($headerVTOffset+${headerVT[status]})) $osfFile | awk '{$1=$1;print $1}')

# sessionOffOffset=$(od -An -tu2 -N 2 -j $(($osfHeaderRoot+4-$headerVTOffset+4+4)) $osfFile | awk '{$1=$1;print $1}')
sessionOffOffset=$(od -An -tu2 -N 2 -j $(($headerVTOffset+${headerVT[session_offset]})) $osfFile | awk '{$1=$1;print $1}')

# fileLengthOffset=$(od -An -tu2 -N 2 -j $(($osfHeaderRoot+4-$headerVTOffset+4+6)) $osfFile | awk '{$1=$1;print $1}')
fileLengthOffset=$(od -An -tu2 -N 2 -j $(($headerVTOffset+${headerVT[file_length]})) $osfFile | awk '{$1=$1;print $1}')

echo "Version Offset: $versionOffset"
echo "Status Offset: $statusOffset"
echo "Session Off Offset: $sessionOffOffset"
echo "File Length Offset: $fileLengthOffset"

versionVal=$(od -An -tu8 -N 8 -j $(($headerRootOffset+$versionOffset)) $osfFile | awk '{$1=$1;print $1}')
statusVal=$(od -An -tu1 -N 1 -j $(($headerRootOffset+$statusOffset)) $osfFile | awk '{$1=$1;print $1}')
sessionOffVal=$(od -An -tu8 -N 8 -j $(($headerRootOffset+$sessionOffOffset)) $osfFile | awk '{$1=$1;print $1}')
fileLengthVal=$(od -An -tu8 -N 8 -j $(($headerRootOffset+$fileLengthOffset)) $osfFile | awk '{$1=$1;print $1}')

echo "Version Val: $versionVal"
echo "Status Val: $statusVal"
echo "Session Off Val: $sessionOffVal"
echo "File Length Val: $fileLengthVal"


firstChunkSize=$(od -An -tu4 -N 4 -j $(($headerSize+4)) $osfFile | awk '{$1=$1;print $1}')

# echo "After header - first chunk:"
# od -A x -t x1z -N $(($osfChunkSize+4)) -j $(($headerSize+4)) $osfFile

echo "After header - second chunk 100b:"
od -A x -t x1z -N 100 -j $(($headerSize+4+$firstChunkSize+4)) $osfFile

# dd if=$osfFile bs=1 skip=$(($osfHeaderRoot + 4)) count=100 status=none | od -A x -t x1z