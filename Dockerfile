FROM nvcr.io/nvidia/isaac-sim:5.1.0

# Accept NVIDIA EULA
ENV ACCEPT_EULA=Y
ENV PRIVACY_CONSENT=Y

# Copy the project into the container
COPY simulation/ /workspace/simulation/
COPY scripts/ /workspace/scripts/
COPY Kemabots-Robotics-Sim-Assignment/ /workspace/Kemabots-Robotics-Sim-Assignment/

WORKDIR /workspace

# Default: run the warehouse scene script in headless mode
CMD ["bash", "-c", "./isaac-sim/python.sh /workspace/scripts/create_warehouse_scene.py --headless"]
