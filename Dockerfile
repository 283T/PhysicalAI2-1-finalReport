FROM ros:jazzy-ros-base@sha256:066420e07f60aa18262f2479981def87ebcfcec42eefb0c0c57c4a46098348ca AS dev

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        ros-jazzy-ros-gz \
        ros-jazzy-gz-tools-vendor \
        ros-jazzy-gz-sim-vendor \
        python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*

RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        ros-jazzy-ur-description ros-jazzy-xacro \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /workspace
CMD ["bash"]

FROM dev AS gui
RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        xvfb x11vnc novnc websockify openbox xterm x11-utils \
        mesa-utils scrot ffmpeg \
    && rm -rf /var/lib/apt/lists/*
ENV DISPLAY=:99 \
    LIBGL_ALWAYS_SOFTWARE=1 \
    QT_X11_NO_MITSHM=1
CMD ["python3", "scripts/start_desktop.py"]
