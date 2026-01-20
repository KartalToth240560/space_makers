FROM nvcr.io/nvidia/isaac-sim:2023.1.1

# Install Isaac Lab
WORKDIR /workspace
RUN git clone https://github.com/isaac-sim/IsaacLab.git
WORKDIR /workspace/IsaacLab

# Do not Install dependencies
# RUN ./isaaclab.sh --install

# Set up Python environment
ENV PYTHONPATH="/workspace/IsaacLab:${PYTHONPATH}"

# Create directories for outputs
RUN mkdir -p /workspace/outputs/models /workspace/outputs/logs /workspace/outputs/videos

WORKDIR /workspace