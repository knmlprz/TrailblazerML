# Docker - how to build and run the container

To build and run the container, you need to have Docker installed on your machine.

## Build the container

To build the container, run the following command:

```bash
sudo docker build --build-arg PYTHON_VERSION=<python_version - default=3.11.9> -t <name_of_the_image> .
```
To insert into the container run the following command:

```bash
sudo docker run -it <name_of_the_image> /bin/bash
```

## What is working now?

- [x] Ubuntu 22.04
- [x] Pyenv with python version management
- [x] ROS2 Humble Hawksbill installed
- [ ] concon installed
