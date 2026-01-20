FROM nvcr.io/nvidia/isaac-sim:4.2.0

# Metadata
LABEL maintainer="kartal"
LABEL description="IsaacLab for RL training"

# Set working directory
WORKDIR /workspace

# Install system dependencies
RUN apt-get update && apt-get install -y \
    git \
    wget \
    curl \
    vim \
    htop \
    tmux \
    build-essential \
    && rm -rf /var/lib/apt/lists/*

# Clone IsaacLab
RUN git clone https://github.com/isaac-sim/IsaacLab.git /workspace/IsaacLab

WORKDIR /workspace/IsaacLab

# Install IsaacLab and dependencies
RUN /isaac-sim/python.sh -m pip install --upgrade pip && \
    /isaac-sim/python.sh -m pip install -e . && \
    /isaac-sim/python.sh -m pip install \
    tensorboard \
    wandb \
    gymnasium \
    stable-baselines3 \
    matplotlib \
    pandas

# Create output directories
RUN mkdir -p /workspace/logs /workspace/checkpoints /workspace/results /workspace/configs

# Environment variables
ENV ISAACSIM_PATH="/isaac-sim"
ENV ISAACLAB_PATH="/workspace/IsaacLab"
ENV ACCEPT_EULA=Y

# Set up convenience aliases
RUN echo 'alias python="/isaac-sim/python.sh"' >> ~/.bashrc && \
    echo 'alias isaaclab="/isaac-sim/python.sh -m isaaclab"' >> ~/.bashrc

WORKDIR /workspace/IsaacLab

# Keep container running
CMD ["/bin/bash"]