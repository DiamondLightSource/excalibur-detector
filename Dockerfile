ARG ODIN_DATA_VERSION=1.12.0-xspress-dev4

FROM ghcr.io/odin-detector/odin-data-build:${ODIN_DATA_VERSION} AS developer

FROM developer AS build

# Copy this repo into /odin/excalibur-detector
COPY . /odin/excalibur-detector

# Build the C++ part of excalibur-detector
WORKDIR /odin/excalibur-detector
RUN mkdir -p build && cd build && \
    cmake -DCMAKE_INSTALL_PREFIX=/odin -DODINDATA_ROOT_DIR=/odin ../cpp && \
    make -j8 VERBOSE=1 && \
    make install

# Build the Python part of excalibur-detector
WORKDIR /odin/excalibur-detector/python
RUN python -m pip install .[sim]

FROM ghcr.io/odin-detector/odin-data-runtime:${ODIN_DATA_VERSION} AS runtime

# Copy full install tree
COPY --from=build /odin /odin

# Copy the venv
COPY --from=build /venv /venv

# Remove the excalibur-detector source tree now it has been built
RUN rm -rf /odin/excalibur-detector

# Add binaries and Python venv to the image PATH
ENV PATH=/odin/bin:/odin/venv/bin:$PATH

# Use /odin as the landing directory
WORKDIR /odin
