# 1. Wir starten mit deinem gewohnten JetPack 6 Basis-Image
FROM dustynv/ros:humble-desktop-l4t-r36.2.0

# --- NEU: Den abgelaufenen ROS 2 GPG-Schlüssel aktualisieren ---
RUN apt-get update || true \
    && apt-get install -y curl \
    && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

# 2. Die nötigen Werkzeuge installieren (ganz wichtig: 'swig' für den Python-Wrapper)
RUN apt-get update && apt-get install -y \
    cmake \
    build-essential \
    python3-dev \
    python3-pip \
    swig \
    git \
    && rm -rf /var/lib/apt/lists/*

# 3. Das YDLidar C++ SDK herunterladen und global installieren
RUN cd /tmp && \
    git clone https://github.com/YDLIDAR/YDLidar-SDK.git && \
    cd YDLidar-SDK && \
    mkdir build && cd build && \
    cmake .. && \
    make -j$(nproc) && \
    make install

# 4. Das Python-Paket aus dem SDK generieren und über pip installieren
RUN cd /tmp/YDLidar-SDK && \
    pip3 install .

# Alle neuen Python Packages hier installieren
RUN pip3 install --no-cache-dir smbus2

# 5. Startverzeichnis festlegen (passend zu deinem Volume-Mount)
WORKDIR /root/ros2
