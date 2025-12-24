# ============================== BASE CONTAINER ==============================
FROM ubuntu:22.04 AS BASE_IMAGE
ENV INSTALL_DIR="/usr/local"
ENV WORKSPACE=/root
ARG APT_PROXY=""

RUN /bin/sh -c "if ! [ -z \"$APT_PROXY\" ]; then \
        echo 'Using Proxy for APT'; \
        echo 'Acquire::http::Proxy \"$APT_PROXY\";' > /etc/apt/apt.conf.d/01proxy; \
    fi"

RUN apt-get update \
    && apt-get install -y python3 python3-venv

COPY ./scripts/dev.py ./scripts/requirements.txt \
    $WORKSPACE/scripts/
COPY ./scripts/dev.py ./scripts/requirements.txt \
    ./scripts/dev_script_library/build_libs.py \
    ./scripts/dev_script_library/context.py \
    ./scripts/dev_script_library/dev_dependencies.py \
    $WORKSPACE/scripts/dev_script_library/

RUN python3 -m venv $WORKSPACE/venv \
    && . $WORKSPACE/venv/bin/activate \
    && python3 -m pip install -r $WORKSPACE/scripts/requirements.txt \
    && cd $WORKSPACE \
    && python3 ./scripts/dev.py utils install-system-packages


# ============================== BUILD WITH DEV SCRIPTS STAGE ==============================
FROM BASE_IMAGE AS BUILD_W_DEV_SCRIPTS
COPY . $WORKSPACE/
RUN cd $WORKSPACE \
    && rm -rf build \
    && python3 -m venv $WORKSPACE/venv \
    && . $WORKSPACE/venv/bin/activate \
    && $WORKSPACE/scripts/dev.sh build cpp --install-dir $INSTALL_DIR --no-examples \
    --build-type Release --threads 4 --use-system-libs


# ============================== BUILD WITHOUT DEV SCRIPTS STAGE ==============================
FROM BASE_IMAGE AS BUILD_WO_DEV_SCRIPTS

COPY . $WORKSPACE/

RUN python3 -m venv $WORKSPACE/venv \
    && . $WORKSPACE/venv/bin/activate \
    && cd $WORKSPACE


ENV INSTALL_DIR="/usr/local"

COPY . $WORKSPACE/

RUN cd $WORKSPACE \
    && cmake -DCMAKE_INSTALL_PREFIX=$INSTALL_DIR -DBUILD_EXAMPLES=OFF \
        -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF . \
    && cmake --build . --parallel 4 --target install


# ============================== BUILD TESTER (WITH DEV SCRIPTS) ==============================
FROM BASE_IMAGE AS BUILD_TESTER_W_DEV
ENV INSTALL_DIR="/usr/local"
COPY --from=BUILD_W_DEV_SCRIPTS $INSTALL_DIR $INSTALL_DIR
COPY tests/pcaps/OS-2-32-U0_v2.0.0_1024x10.pcap examples/static_linking_example/CMakeLists.txt \
     examples/static_linking_example/main.cpp  $WORKSPACE/

RUN export CMAKE_PREFIX_PATH="$INSTALL_DIR" \
    && mkdir -p $WORKSPACE/build \
    && cd $WORKSPACE/build \
    && cmake $WORKSPACE -DCMAKE_BUILD_TYPE=Release \
    && cmake --build . --parallel 4


# ============================== BUILD TESTER (WITHOUT DEV SCRIPTS) ==============================
FROM BASE_IMAGE AS BUILD_TESTER_WO_DEV
ENV INSTALL_DIR="/usr/local"
COPY --from=BUILD_WO_DEV_SCRIPTS $INSTALL_DIR $INSTALL_DIR
COPY tests/pcaps/OS-2-32-U0_v2.0.0_1024x10.pcap examples/static_linking_example/CMakeLists.txt \
     examples/static_linking_example/main.cpp  $WORKSPACE/

RUN export CMAKE_PREFIX_PATH="$INSTALL_DIR" \
    && mkdir -p $WORKSPACE/build \
    && cd $WORKSPACE/build \
    && cmake $WORKSPACE -DCMAKE_BUILD_TYPE=Release \
    && cmake --build . --parallel 4


# ============================== FINAL RUNNER STAGE ==============================
FROM BASE_IMAGE AS FINAL_RUNNER
WORKDIR /test

COPY tests/pcaps/OS-2-32-U0_v2.0.0_1024x10.pcap .

COPY --from=BUILD_TESTER_W_DEV /root/build/pcap_test ./pcap_test_with_dev_scripts

COPY --from=BUILD_TESTER_WO_DEV /root/build/pcap_test ./pcap_test_without_dev_scripts

CMD echo "--- RUNNING TEST BUILT WITH DEV SCRIPTS ---" \
    && ./pcap_test_with_dev_scripts ./OS-2-32-U0_v2.0.0_1024x10.pcap \
    && echo "\n--- RUNNING TEST BUILT WITHOUT DEV SCRIPTS ---" \
    && ./pcap_test_without_dev_scripts ./OS-2-32-U0_v2.0.0_1024x10.pcap
