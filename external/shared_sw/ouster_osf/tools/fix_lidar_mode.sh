#!/bin/bash

# This patch was made for the web-slam OSF data that was producing by deployed
# record_node OSF writer with a bug that had a hardcoded lidar_mode value
# 1024x10 for osf_sensor.mode field (osfSession.sensors array).

# Web-viz embedding for web-slam handled this via a separate parameter
# for lidar_mode that was stored in a datastore object for the specific
# web-slam job. So everything works well for web-slam results page.

# The bug was fixed at some point, see:
# https://bitbucket.org/ouster_io/shared_sw/pull-requests/79

# But the problem still occurs when we want to use the generated OSFs in other
# pipelines and don't want to make any custom handling for web-slam OSFs
# that has other than 1024x10 lidar_mode.

# Thus this patch.

# Usage: ./fix_lidar_mode.sh <osf_file.osf> <new_lidar_mode>

# NOTE: Script produces more output than needed for the task and has general
#       purpose flatbuffer reading utility functions that can be used for
#       further quick hacks, exploration and messing with OSF files without
#       any dependencies (all commands are POSIX compliant)


set -e

osfFile=${1}
newMode=${2:-"0"}


case "$newMode" in
    "MODE_512x10")
        newModeVal="\x01"
        ;;
    "MODE_512x20")
        newModeVal="\x02"
        ;;
    "MODE_1024x10")
        newModeVal="\x03"
        ;;
    "MODE_1024x20")
        newModeVal="\x04"
        ;;
    "MODE_2048x10")
        newModeVal="\x05"
        ;;
    *)
        echo "ERROR: Not a valid lidar mode specified: $newMode"
        exit 1
        ;;
esac

echo "newMode = $newMode, newModeVal = $newModeVal"

# ===== VTables for our Flatbuffers table ===============
# Flatbuffer stores data in the field via VTable references.
# VTable offsets are generated from *.fbs specs and don't change
# during the regeneration (if the spec doesn't change the order of the field)
# To find the field value locations we need to know the generated VTable
# for Tables of interest.

# osfHeader VTable
declare -A headerVT
headerVT=(["version"]="4" ["status"]="6" ["session_offset"]="8" ["file_length"]="10")

# osfSession VTable
declare -A sessionVT
sessionVT=(["id"]="4" ["session_mode"]="6" ["lidar_frame_mode"]="8" ["range_multiplier"]="10" ["sensors"]="12" ["start_ts"]="14" ["end_ts"]="16" ["map"]="18" ["metadata"]="20")

# osfSensor VTable
declare -A osfSensorVT
osfSensorVT=(["name"]="4" ["id"]="6" ["sn"]="8" ["firmware_rev"]="10" ["mode"]="12" ["extrinsics"]="14" ["beam_azimuths"]="16" ["beam_altitudes"]="18" ["imu_to_sensor_transform"]="20" ["lidar_to_sensor_transform"]="22")

# ===== Helper Functions ===============================================

# Gets the ossfet to the VTable that hold references to the
# table fields
# Params: $1 - offsset to the Table
# Return: offset to the VTable
function get_vt_offset() {
    local vtoff=$(od -An -td4 -N 4 -j $1 $osfFile | awk '{$1=$1;print $1}')
    vtoff=$(($1-$vtoff))
    echo "$vtoff"
}

# Get the byte value from the provided offset
# Params: $1 - offset to the first byte
# Return: unsigned byte value
function get_uint8_val() {
    local val=$(od -An -tu1 -N 1 -j $1 $osfFile | awk '{$1=$1;print $1}')
    echo "$val"
}

# Get the word value (2 bytes) from the provided offset
# Params: $1 - offset to the first byte to read from (2 bytes)
# Return: unsigned word value
function get_uint16_val() {
    local val=$(od -An -tu2 -N 2 -j $1 $osfFile | awk '{$1=$1;print $1}')
    echo "$val"
}

# Get the dword value (4 bytes) from the provided offset
# Params: $1 - offset to the first byte to read from (4 bytes)
# Return: unsigned dword value
function get_uint32_val() {
    local val=$(od -An -tu4 -N 4 -j $1 $osfFile | awk '{$1=$1;print $1}')
    echo "$val"
}

# Get the qword value (8 bytes) from the provided offset
# Params: $1 - offset to the first byte to read from (8 bytes)
# Return: unsigned dword value
function get_uint64_val() {
    local val=$(od -An -tu8 -N 8 -j $1 $osfFile | awk '{$1=$1;print $1}')
    echo "$val"
}

# Get the table field offset where the field value is stored
# Params: $1 - offset to the Table
#         $2 - VTable field offset
# Return: offset to the field value
function get_field_offset() {
    local vtoff=$(get_vt_offset $1)
    local fieldoff=$(get_uint16_val $(($vtoff+$2)) $osfFile | awk '{$1=$1;print $1}')
    echo "$(($1+$fieldoff))"
}

# Get the offset to the value by resolving the reference indirection.
# Flatbuffer stores Tables, Arrays and Strings as a references.
# Params: $1 - offset to the reference value
# Return: resolved offset for the current reference
function get_ind_offset() {
    local val=$(get_uint32_val $1)
    echo "$(($1+$val))"
}

# Get the offset to the field value stored by reference. (Table, Array
# or String)
# Params: $1 - offset to the Table
#         $2 - VTable field offset
# Return: resolved offset for the field value (which was stored by reference)
function get_ind_field_offset() {
    local fieldoff=$(get_field_offset $1 $2)
    echo "$(get_ind_offset $fieldoff)"
}

# Get the qword table field value
# Params: $1 - offset to the Table
#         $2 - VTable field offset
# Return: unsigned qword value for the field
function get_uint64_field_val() {
    local fieldoff=$(get_field_offset $1 $2)
    local fieldval=$(get_uint64_val $fieldoff)
    echo "$fieldval"
}

# Get the dword table field value
# Params: $1 - offset to the Table
#         $2 - VTable field offset
# Return: unsigned dword value for the field
function get_uint32_field_val() {
    local fieldoff=$(get_field_offset $1 $2)
    local fieldval=$(get_uint32_val $fieldoff)
    echo "$fieldval"
}

# Get byte table field value
# Params: $1 - offset to the Table
#         $2 - VTable field offset
# Return: unsigned byte value for the field
function get_uint8_field_val() {
    local fieldoff=$(get_field_offset $1 $2)
    local fieldval=$(get_uint8_val $fieldoff)
    echo "$fieldval"
}


# ===== Get Header Info =====================================

# OSF file starts with the osfHeader Table which stored
# with a prefixed size object, so the first 4 bytes is the size
# of the osfHeader buffer.
headerSize=$(get_uint32_val 0)

# Next 4 bytes contains the offset to the root table, which in our case
# is a osfHeader Table
headerRootOffset=$(get_ind_offset 4)

echo "Hello OSF!"
echo "Header Size: $headerSize"
echo "Header Root Offset: $headerRootOffset"

# Show the osfHeader buffer bytes
echo "---------- Header Buffer Bytes (size prefixed):"
od -A x -t x1z -N $(($headerSize+4)) -j 0 $osfFile


# Show the header VTable bytes
echo "---------- Header VTable:"
headerVTOffset=$(get_vt_offset $headerRootOffset)

# Length of the VTable = number of VTable fields X 2 bytes + 4 bytes
headerVTLength=$((4+${#headerVT[@]}*2))
od -A x -t x1z -N $headerVTLength -j $headerVTOffset $osfFile

# Read the osfHeader values using the offset to the osfHeader Table
# and the VTable field offset from headerVT dict.
# The field type and thus the number of bytes to read determined by
# osfHeader.fbs schema spec.
versionVal=$(get_uint64_field_val $headerRootOffset ${headerVT[version]})
statusVal=$(get_uint8_field_val $headerRootOffset ${headerVT[status]})
sessionOffVal=$(get_uint64_field_val $headerRootOffset ${headerVT[session_offset]})
fileLengthVal=$(get_uint64_field_val $headerRootOffset ${headerVT[file_length]})

echo "Version Val: $versionVal"
echo "Status Val: $statusVal"
echo "Session Off Val: $sessionOffVal"
echo "File Length Val: $fileLengthVal"


# ===== Get Session Info ========================================

# osfSession buffer saved with the prefixed size, so first 4 bytes
# is the the size of the osfSession buffer
sessionSize=$(get_uint32_val $sessionOffVal)

# next 4 bytes is the reference to the beginnnig of the osfSession Table
# that we read via ref indirection
sessionRootOffset=$(get_ind_offset $(($sessionOffVal+4)))

echo "sessionSize = $sessionSize"
echo "sessionRootOffset = $sessionRootOffset"

# osfSession.sensors fiels is an Array so it's stored by reference
# which we need to read through the indirection
sensorsOffset=$(get_ind_field_offset $sessionRootOffset ${sessionVT[sensors]})
echo "sensors off = $sensorsOffset"

# first 4 bytes of the array it's the length of an array (in the number of
# elements, not bytes)
sensorsLen=$(get_uint32_val $sensorsOffset)
echo "sensors len = $sensorsLen"

# Validate sensors len == 1, we don't handle the 3 sensors case
# because web-slam data output always has 1 sensor
if [[ ! "$sensorsLen" -eq "1" ]]; then
    echo "ERROR: Script works only for 1 sensor, but found '$sensorsLen'"
    exit 1
fi

# ======== Get first sensor from osfSession.sensors array =========


# next 4 bytes after the array length is the reference to the
# osf_sensor Table, because tables always stored by references and not inlined
sensor1RootOffset=$(get_ind_offset $(($sensorsOffset+4)))
echo "sensor1RootOffset = $sensor1RootOffset"

# Show first bytes of the sensor Table
echo "---------- Sensor1 (100 bytes)....:"
od -A x -t x1z -N 100 -j $sensor1RootOffset $osfFile

# Show the VTable of first osf_sensor
sensor1VTOffset=$(get_vt_offset $sensor1RootOffset)
echo "sensor1VTOffset = $sensor1VTOffset"
echo "---------- Sensor1 VTable:"
od -A x -t x1z -N $((4+${#osfSensorVT[@]}*2)) -j $sensor1VTOffset $osfFile


# Get the offset to the field value of osf_sensor.mode, which we want to
# modify
sensor1ModeFieldOff=$(get_field_offset $sensor1RootOffset ${osfSensorVT[mode]})
echo "sensor1.mode Field Offset = $sensor1ModeFieldOff"

# Read the mode field value with our function
sensor1ModeVal=$(get_uint8_field_val $sensor1RootOffset ${osfSensorVT[mode]})
echo "sensor1.mode Value = $sensor1ModeVal"

# Show the neighborhoods of the mode field value
echo "---------- Sensor1 Mode (+/- 16 bytes):"
od -A x -t x1z -N 16 -j $(($sensor1ModeFieldOff-16)) $osfFile
od -A x -t x1z -N 1 -j $(($sensor1ModeFieldOff)) $osfFile
od -A x -t x1z -N 16 -j $(($sensor1ModeFieldOff+1)) $osfFile

# Another way of reading the field value is via direct byte value read function
# it's just for a verification that we read everything correctly
sensor1ModeVal0=$(get_uint8_val $sensor1ModeFieldOff)
echo "sensor1.mode Value = $sensor1ModeVal0 (check)"

if [[ ! "$sensor1ModeVal" -eq "$sensor1ModeVal0" ]]; then
    echo "ERROR: In mode value reading, something is off '$sensor1ModeVal' != '$sensor1ModeVal0'"
    exit 1
fi

# ========= Ready to Patch Mode field Value =========

echo
echo "Patching .... to new mode value: $newModeVal"

# Write new mode value to the field value offset (1 byte)
printf $newModeVal | dd of=$osfFile bs=1 seek=$sensor1ModeFieldOff count=1 conv=notrunc

echo
echo "DONE!"

# Show the neighborhood of the modified mode value
echo "---------- Sensor1 New Mode (+/- 16 bytes):"
od -A x -t x1z -N 16 -j $(($sensor1ModeFieldOff-16)) $osfFile
od -A x -t x1z -N 1 -j $(($sensor1ModeFieldOff)) $osfFile
od -A x -t x1z -N 16 -j $(($sensor1ModeFieldOff+1)) $osfFile


echo
echo "success"
