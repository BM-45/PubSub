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
├── kubernetes/             # Kubernetes manifests
│   ├── deployment.yaml    # K8s deployment configuration
│   └── service.yaml       # K8s service definitions
├── src/                    # Source code
│   ├── publisher/         # Publisher node implementation
│   └── subscriber/        # Subscriber node implementation
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
docker-compose build
```

2. Run the containers:
```bash
docker-compose up
```

### Kubernetes Deployment

1. Apply the Kubernetes manifests:
```bash
kubectl apply -f kubernetes/deployment.yaml
kubectl apply -f kubernetes/service.yaml
```

2. Verify the deployment:
```bash
kubectl get pods
kubectl get services
```

## Usage

### Publisher Node
The publisher node broadcasts messages on a defined topic at regular intervals.

### Subscriber Node
The subscriber node listens to the topic and processes incoming messages.

### Topics
- Default topic: `/pubsub_topic`
- Message type: `std_msgs/String` (or custom message type)

## Configuration

Edit the configuration files to customize:
- Topic names
- Message types
- Publishing frequency
- QoS (Quality of Service) settings

## Monitoring

Monitor the communication using ROS2 tools:
```bash
# List active topics
ros2 topic list

# Echo messages on a topic
ros2 topic echo /pubsub_topic

# Check node information
ros2 node info /publisher_node
ros2 node info /subscriber_node
```

## Troubleshooting

- Ensure ROS2 environment is sourced: `source /opt/ros/<distro>/setup.bash`
- Check if nodes are running: `ros2 node list`
- Verify topic connections: `ros2 topic info /pubsub_topic`
- For Docker issues, check container logs: `docker-compose logs`

## Contributing

Contributions are welcome! Please:
1. Fork the repository
2. Create a feature branch
3. Commit your changes
4. Submit a pull request

## License

This project is licensed under the MIT License.

## References

- [ROS2 Documentation](https://docs.ros.org/)
- [Docker Documentation](https://docs.docker.com/)
- [Kubernetes Documentation](https://kubernetes.io/docs/)
