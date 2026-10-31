FROM ros:jazzy-ros-base-noble

ENV DEBIAN_FRONTEND=noninteractive \
    DISPLAY=:99 \
    QT_QPA_PLATFORM=xcb \
    QT_X11_NO_MITSHM=1 \
    LIBGL_ALWAYS_SOFTWARE=1 \
    LIBGL_DRI3_DISABLE=1 \
    XDG_RUNTIME_DIR=/tmp/xdg-runtime-root \
    ROS_DOMAIN_ID=18

RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-jazzy-ros-gz \
    python3-sklearn \
    python3-joblib \
    python3-numpy \
    xvfb \
    x11vnc \
    novnc \
    websockify \
    openbox \
    xterm \
    mesa-utils \
    libgl1-mesa-dri \
    dbus-x11 \
    procps \
    && rm -rf /var/lib/apt/lists/* \
    && mkdir -p /tmp/xdg-runtime-root \
    && chmod 0700 /tmp/xdg-runtime-root

COPY scripts/start-desktop.sh /usr/local/bin/start-desktop.sh
RUN chmod +x /usr/local/bin/start-desktop.sh
WORKDIR /workspace
CMD ["/usr/local/bin/start-desktop.sh"]
