FROM ghcr.io/odin-detector/odin-data:latest AS build

# Root of excalibur-detector
COPY . /tmp/excalibur-detector

# C++
WORKDIR /tmp/excalibur-detector
RUN mkdir -p build && cd build && \
    cmake -DCMAKE_INSTALL_PREFIX=/odin -DODINDATA_ROOT_DIR=/odin ../cpp && \
    make -j8 VERBOSE=1 && \
    make install

# Python
WORKDIR /tmp/excalibur-detector/python
RUN python -m pip install .

# Final image
FROM fedora:37
COPY --from=build /odin /odin
ENV PATH=/odin/bin:/odin/venv/bin:$PATH
WORKDIR /odin

RUN dnf update -y && dnf install -y \
    # odin-data C++ runtime dependencies
    blosc boost hdf5 log4cxx libpcap zeromq && \
    # tidy up
    dnf -y clean all
