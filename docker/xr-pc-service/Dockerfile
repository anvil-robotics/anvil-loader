FROM ubuntu:22.04

ENV DEBIAN_FRONTEND=noninteractive

# Runtime deps for RoboticsServiceProcess
RUN apt-get update && apt-get install -y --no-install-recommends \
    libgl1 libegl1 libgles2 libxkbcommon0 libdbus-1-3 libfontconfig1 libfreetype6 \
    libglib2.0-0 \
  && rm -rf /var/lib/apt/lists/*

# Stub desktop integration utilities called by the deb post-install (not needed headless)
RUN for cmd in xdg-icon-resource gtk-update-icon-cache update-mime-database update-desktop-database; do \
      ln -sf /bin/true /usr/local/bin/$cmd; \
    done

# Pre-built Ubuntu 22.04 deb — self-contained (Qt 6.6.3 + gRPC + OpenSSL bundled, no APT deps)
ADD https://github.com/XR-Robotics/XRoboToolkit-PC-Service/releases/download/v1.0.0/XRoboToolkit_PC_Service_1.0.0_ubuntu_22.04_amd64.deb /tmp/xr-pc-service.deb
RUN dpkg -i /tmp/xr-pc-service.deb && rm /tmp/xr-pc-service.deb

# Run without a physical display
ENV QT_QPA_PLATFORM=offscreen

# runService.sh launches the binary in the background then exits — not suitable for Docker.
# Run RoboticsServiceProcess directly in the foreground with the same env the script sets.
WORKDIR /opt/apps/roboticsservice
ENTRYPOINT ["bash", "-c", \
  "export LD_LIBRARY_PATH=:/opt/apps/roboticsservice:/opt/apps/roboticsservice/lib:/opt/apps/roboticsservice/SDK/x64 && \
   export QT_PLUGIN_PATH=/opt/apps/roboticsservice/plugins/ && \
   export QT_QML_PATH=/opt/apps/roboticsservice/qml/ && \
   exec ./RoboticsServiceProcess"]
