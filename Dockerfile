### BASE STAGE ###

# Build from sawyer ompl dev box
FROM jgkawell/cairo-lfd:simple AS base

# Needed for NLP audio packages
RUN apt -y update && apt -y install \
        python-catkin-tools \
        portaudio19-dev \
        gcc \
        g++

# Clone and install KDL for MoveIt plan listener
RUN cd ~/catkin_ws/src \
        && git clone https://github.com/gt-ros-pkg/hrl-kdl.git \
        && cd hrl-kdl/pykdl_utils/ \
        && python setup.py install \
        && cd ../hrl_geom \
        && python setup.py install

# Clone NLP package
RUN cd ~/catkin_ws/src && git clone https://github.com/cairo-robotics/cairo-nlp.git

# Load feedback CC-LfD
COPY . /root/catkin_ws/src/feedback_cclfd

# Install Python requirements
RUN cd ~/catkin_ws/src/feedback_cclfd && pip install -r requirements.txt
RUN cd ~/catkin_ws/src/cairo-nlp && pip install -r requirements.txt

# Current workaround since Google Cloud installation is broken
RUN pip install --upgrade pip
RUN pip install --upgrade 'setuptools<45.0.0'
RUN pip install --upgrade 'cachetools<5.0'
RUN pip install --upgrade cryptography
RUN python -m easy_install --upgrade pyOpenSSL

# Finally build the workspace
RUN cd ~/catkin_ws && catkin build

# Clean up apt
RUN rm -rf /var/lib/apt/lists/*


### NVIDIA STAGE ###

# Extra needed setup for Nvidia-based graphics
FROM base AS nvidia

# Copy over needed OpenGL files from Nvidia's image
COPY --from=nvidia/opengl:1.0-glvnd-runtime-ubuntu16.04 /usr/local /usr/local
COPY --from=nvidia/opengl:1.0-glvnd-runtime-ubuntu16.04 /etc/ld.so.conf.d/glvnd.conf /etc/ld.so.conf.d/glvnd.conf