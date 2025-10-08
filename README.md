# PubSub - ROS2 Publisher-Subscriber System

A containerized ROS2 Publisher-Subscriber communication system with Docker and Kubernetes deployment support.

## Overview

This project demonstrates a ROS2 publish-subscribe messaging pattern implementation, containerized for easy deployment across different environments. It's designed for distributed robotics applications where multiple nodes need to communicate asynchronously.

## Features

- ROS2-based publisher and subscriber nodes
- Dockerized deployment for consistency across environments
- Kubernetes orchestration support for scalable deployments
- Message-based asynchronous communication
- Easy integration with existing ROS2 ecosystems

## Tech Stack

- **ROS2** - Robot Operating System 2 (middleware framework)
- **Docker** - Containerization platform
- **Kubernetes** - Container orchestration
- **Python/C++** - Primary development languages

## Project Structure

```
.
├── Dockerfile              # Docker image definition
├── docker-compose.yml      # Multi-container orchestration
├── kubernetes/deployments  # Kubernetes manifests
│   ├── pubsub.yaml         # K8s deployment configuration
├── src/                    # Source code
│   ├── publisher/          # Publisher node implementation
│   └── subscriber/         # Subscriber node implementation
└── launch/                 # ROS2 launch files
```

## Prerequisites

- ROS2 (Humble/Iron/Jazzy or compatible distribution)
- Docker (v20.10 or higher)
- Docker Compose (v2.0 or higher)
- Kubernetes cluster (optional, for K8s deployment)
- kubectl (for Kubernetes deployment)

## Getting Started

### Local ROS2 Development

1. Clone the repository:
```bash
git clone https://github.com/BM-45/PubSub.git
cd PubSub
```

2. Build the ROS2 workspace:
```bash
colcon build
source install/setup.bash
```

3. Launch the publisher and subscriber:
```bash
ros2 launch launch/pubsub.launch.py
```

### Docker Deployment

1. Build the Docker image:
```bash
docker build -t image-name .
```

### Kubernetes Deployment

1. Apply the Kubernetes manifests:
```bash
kubectl apply -f kubernetes/deployment/pubsub.yaml
```

2. Verify the deployment:
```bash
kubectl get pods
kubectl get services
```
