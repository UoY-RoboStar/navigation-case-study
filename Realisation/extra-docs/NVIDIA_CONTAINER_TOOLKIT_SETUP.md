# NVIDIA Container Toolkit Setup Guide

## Overview

The NVIDIA Container Toolkit allows Docker containers to access NVIDIA GPUs for GPU-accelerated workloads. This is the modern replacement for the deprecated `nvidia-docker2`.

## Prerequisites

- NVIDIA GPU installed
- NVIDIA drivers installed (version 450.80.02 or later)
- Docker installed and running

## Verify NVIDIA Driver

```bash
nvidia-smi
```

You should see your GPU information and driver version.

## Installation Steps

### Step 1: Add Repository

```bash
distribution=$(. /etc/os-release;echo $ID$VERSION_ID)
curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | \
  sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg
curl -s -L https://nvidia.github.io/libnvidia-container/$distribution/libnvidia-container.list | \
  sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' | \
  sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list
```

### Step 2: Install Package

```bash
sudo apt-get update
sudo apt-get install -y nvidia-container-toolkit
```

### Step 3: Configure Docker Runtime

```bash
sudo nvidia-ctk runtime configure --runtime=docker
```

This will update `/etc/docker/daemon.json` with the NVIDIA runtime configuration.

### Step 4: Restart Docker

```bash
sudo systemctl restart docker
```

## Verification

Test that Docker can access the GPU:

```bash
docker run --rm --gpus all nvidia/cuda:12.0.0-base-ubuntu22.04 nvidia-smi
```

You should see the same GPU information as running `nvidia-smi` on the host.

## Usage with Waypoint Navigator

### Start with GPU Acceleration

```bash
./run_waypoint_navigator_docker.sh start --gpu nvidia
```

### If Toolkit Not Installed

Use software rendering:

```bash
./run_waypoint_navigator_docker.sh start --gpu none
```

## Troubleshooting

### Issue: "could not select device driver"

**Solution:** Restart Docker daemon
```bash
sudo systemctl restart docker
```

### Issue: "NVIDIA-SMI has failed"

**Solution:** Update NVIDIA drivers
```bash
sudo apt-get install --reinstall nvidia-driver-XXX
# Replace XXX with your driver version
```

### Issue: "no NVIDIA GPU device"

**Check if GPU is detected:**
```bash
lspci | grep -i nvidia
```

**Check driver status:**
```bash
nvidia-smi
```

### Issue: Permission denied

**Solution:** Add user to docker group
```bash
sudo usermod -aG docker $USER
# Log out and back in
```

## Docker Compose Configuration

The waypoint navigator Docker Compose file already includes GPU configuration:

```yaml
deploy:
  resources:
    reservations:
      devices:
        - driver: nvidia
          count: 1
          capabilities: [gpu, compute, utility]
environment:
  - NVIDIA_DRIVER_CAPABILITIES=all
  - NVIDIA_VISIBLE_DEVICES=all
```

## Alternative: Software Rendering

If you don't want to install the toolkit or don't have an NVIDIA GPU:

```bash
./run_waypoint_navigator_docker.sh start --gpu none
```

This uses CPU-based software rendering (slower but works everywhere).

## References

- Official NVIDIA Container Toolkit docs: https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/
- Installation guide: https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html
- Docker GPU support: https://docs.docker.com/config/containers/resource_constraints/#gpu

## Quick Reference

```bash
# Install
sudo apt-get install -y nvidia-container-toolkit
sudo nvidia-ctk runtime configure --runtime=docker
sudo systemctl restart docker

# Test
docker run --rm --gpus all nvidia/cuda:12.0.0-base-ubuntu22.04 nvidia-smi

# Use with waypoint navigator
./run_waypoint_navigator_docker.sh start --gpu nvidia
```
