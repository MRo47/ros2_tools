# ROS2 dev container

This package is a guide on how to create dev containers for ROS2.
Dev containers, are Docker containers that are specifically configured to provide a fully featured development environment.

## Setup

Check the official instructions: [Install Docker Engine on Ubuntu](https://docs.docker.com/engine/install/ubuntu/) first, and follow the (post-installation steps)[https://docs.docker.com/engine/install/linux-postinstall/].

To setup a dev container, three different types of files need to be configured.

### 1. Dockerfile

This file will build the docker images and multistage builds are used to separate container functionality.
Multistage builds are useful when extending the functionality of the container for different development or production environments.

```Dockerfile
ARG ROS_DISTRO=jazzy

FROM ros:${ROS_DISTRO} AS ros2_base
ENV ROS_DISTRO=${ROS_DISTRO}

...

## Computer vision stage
FROM ros2_base AS ros2_cv

...

## Development stage
FROM ros2_cv AS ros2_dev

...
```

In the Dockerfile snippet above different stages/targets exist (ros2_base, ros2_cv, ros2_dev). Each stage builds over the top one.
This pattern can be changed for example to use the development (ros2_dev) directly from base (ros2_base) or have a navigation module in the mid section swapping out the computer vision module.

To keep dependencies clear from the builds they are listed in separate files with names `apt-*-packages`.
The following Dockerfile command parses and installs the dependencies.

```Dockerfile
RUN apt-get update && \
    apt-get install -y $(cut -d# -f1 </tmp/apt-base-packages | envsubst) \
    && rm -rf /var/lib/apt/lists/* /tmp/apt-base-packages
```

### 2. docker-compose.*.yml

These files provide the configuration to build, run and combine the images built using `Dockerfile`.

```yaml
services:
  dev: # name of this service, will be used in devcontainer.json
    image: ros2_dev # thw image will have this name once built
    extends:
      file: docker-compose.cv.yml # extend the below service from this file
      service: computer_vision
    build:
      context: .
      dockerfile: Dockerfile # the only dockerfile
      args:
        USER_NAME: $USER_NAME # will be taken from .env file, run export_env.sh first
        ...
      target: ros2_dev # must match the stage name in Dockerfile
    ...
    user: $USER_NAME # will login as your usename
    volumes:
      - /tmp/.X11-unix:/tmp/.X11-unix:rw # for gui applications
      - /home/$USER_NAME/.ssh:/home/$USER_NAME/.ssh:ro # git credentials
      - /home/$USER_NAME/data:/home/$USER_NAME/data:rw # i like to keep data this way
      - /dev/bus/usb:/dev/bus/usb # for usb access, depends where its located
      - ../workspaces:/home/$USER_NAME/workspaces # mount workspace directory inside the container
```

To extend any service from a dockerfile have the `extend:` field and specify the docker-compose*.yml `file:` which has the service and the name of the service in that file as `service:`

### 3. devcontainer.json

This file uses the `docker-compose.yml` files to build the services and also provides configuration for vscode to work with the target service.

```json
{
  "name": "ros2", // this will show up as container name in vscode
  "dockerComposeFile": [ // files appear in order they extend base > cv > dev
    "docker-compose.base.yml",
    "docker-compose.cv.yml",
    "docker-compose.dev.yml"
  ],
  "service": "dev", // the service you want to attach to
  "workspaceFolder": "/home/${localEnv:USER}/workspaces/ros2_ws" // workspace dir to open by default inside your service
}
```

For more info see [this](https://code.visualstudio.com/docs/devcontainers/create-dev-container).

## Build

### 1. Generate .env file

Before building the containers a `.env` file must be configured. This contains the users UID, GID, name and group name which will be used to create a non-root user inside the dev container. This will also ensure the user has permissions to the mounted files and directories.

To generate the `.env` file in `.devcontainer`

```bash
# from ros2_dev_container
bash export_env.sh
```

### 2. Build the container in vscode

Make sure you have the [Dev Containers](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-containers) extension installed in vscode.

Open `ros2_dev_container` directory in vscode > press `F1` > `Dev Containers: Rebuild and Reopen in Container`.

Usually once built vscode will ask you to open this directory in container, which you can say yes.
Or from command palette `F1` and run `Reopen in Container`.

> **NOTE** if the build fails, reopen in local folder option will show you logs. also [see](https://code.visualstudio.com/docs/devcontainers/create-dev-container#_full-configuration-edit-loop)

### 3. For non vscode users

The initial setup remains the same. Generate the `.env` file in `.devcontainer`.

```bash
# from ros2_dev_container
bash export_env.sh
```

#### 3.a. Using docker-compose

- To build or run the containers

  ```bash
  # from ros2_dev_container
  docker-compose -f .devcontainer/docker-compose.base.yml -f .devcontainer/docker-compose.cv.yml -f .devcontainer/docker-compose.dev.yml up -d
  ```

- To get shell access into the container while its running. Get the name of the dev container from command above and run the following from another terminal.

  ```bash
  docker exec -it <container_name> /bin/bash
  ```

- To stop the containers.

  ```bash
  # from ros2_dev_container
  docker compose -f .devcontainer/docker-compose.base.yml -f .devcontainer/docker-compose.cv.yml -f .devcontainer/docker-compose.dev.yml down
  ```

#### 3.b. Using devcontainers-cli

The [cli](https://github.com/devcontainers/cli) version of the vscode extension can also be used.

- To build and start the containe.

  ```bash
  # from ros2_dev_container
  devcontainer up --workspace-folder .
  ``` 

- To get shell access into the container while its running.

  ```bash
  # from ros2_dev_container
  devcontainer exec --workspace-folder . bash
  ``` 

- To stop the container

  ```bash
  # To list all running containers
  docker ps
  
  # To stop the container by its ID
  docker stop <container_id>
  
  # To remove the container entirely
  docker rm <container_id>
  ```
