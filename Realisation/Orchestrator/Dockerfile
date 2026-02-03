# orchestrator/Dockerfile

FROM python:3.11-slim

# Accept Docker GID as build argument
ARG DOCKER_GID=142

# Install git and docker CLI for potential Git repo builds and Docker operations
RUN apt-get update && apt-get install -y git docker.io && apt-get clean

# Create docker group with the same GID as host
RUN groupadd -g ${DOCKER_GID} docker || groupmod -g ${DOCKER_GID} docker

# Set working directory
WORKDIR /app



# Copy orchestrator source
COPY requirements.txt .
COPY orchestrator.py .


ENV PYTHONUNBUFFERED=1

# Install Python dependencies
RUN pip install --no-cache-dir -r requirements.txt

# Create a non-root user and add to docker group
RUN useradd -m -s /bin/bash orchestrator && usermod -aG docker orchestrator

# Switch to non-root user
USER orchestrator

# Entrypoint
CMD ["python", "-u", "orchestrator.py"]
