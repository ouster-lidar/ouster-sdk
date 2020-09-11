# Preclude false positive calls (-e), report all errors (-x)
set -e -x

BASE=$(dirname $(readlink -f "$0"))
DOCK=/shared_sw

gs_source="gs://artifacts.ouster-build.appspot.com/jenkins/test-files/ouster_pyclient"
gs_target="$BASE/test-files"

mkdir -p $gs_target

gsutil -q rsync -d $gs_source $gs_target
docker run --rm\
       -v /${BASE}/build:/${DOCK}/build/ \
       -v /${BASE}/ouster_example:/${DOCK}/ouster_example/:ro \
       -v /${BASE}/sensor_utils:/${DOCK}/sensor_utils/:ro \
       -v /${BASE}/test-files:/${DOCK}/test-files:ro \
       shared_sw /bin/bash -c  " \
           set -e -x &&\
           pip3 install \
               \$(ls ${DOCK}/build/ouster_client*.whl) &&\
           pip3 install \
               \$(ls ${DOCK}/build/ouster_pcap*.whl) &&\
           cp -r ${DOCK}/sensor_utils/test ${DOCK}/test &&\
           cp -r ${DOCK}/test-files ${DOCK}/test/ &&\
           cd ${DOCK}/test &&\
           python3 -m pytest
           "
