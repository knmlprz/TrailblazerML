# TrailblazerML

this is a project to create an autonomous control rover for the rover “legnedary rover PRz” the goal is to win the
competition one of the tasks is autonomy

# What hardware we are using

- Nvidia Jetson xavier nx
- Camera Oak-D-pro-w
- GPS NEO-6M-0U-BLOX

# What software we are using (main libraries)

- DepthAI
- OpenCV
- Open3d

## How to test project (tested on linux ubuntu graphic env X11)

#### build the docker image

```bash
 sudo docker build -t trb_1 .
```

#### open docker with graphic env X11
    ```bash
    sudo xhost +local:docker 
    sudo docker run -it --privileged \
    --network=host \
    --ipc=host \
    -v /tmp/.x11-unix:/tmp/.X11-unix:rw \
    --env=DISPLAY \
    trb_1
    ```
#### In docker terminal launch the Gazebo simulator with the robot:
    ```bash
    ros2 launch gazebo_viz launch_sim.launch.py
    ```
#### in next terminal

##### open docker with graphic env X11
    ```bash
    sudo xhost +local:docker 
    sudo docker run -it --privileged \
    --network=host \
    --ipc=host \
    -v /tmp/.x11-unix:/tmp/.X11-unix:rw \
    --env=DISPLAY \
    trb_1
    ```
#### in docker terminal test robot movement
    ```
    ros2 topic pub /diff_drive_controller_right/cmd_vel_unstamped geometry_msgs/msg/Twist "{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
    ```

# Docker

## How to build the docker image

```bash
 sudo docker build -t trb_1 .
```

## How to run the docker image

### Linux

1. X11
    ```bash
    sudo xhost +local:docker 
    sudo docker run -it --privileged \
    --network=host \
    --ipc=host \
    -v /tmp/.x11-unix:/tmp/.X11-unix:rw \
    --env=DISPLAY \
    trb_1
    ```
2. Wayland

   Check wayland socket
    ```bash
        echo $WAYLAND_DISPLAY
    ```
   Check user UID
    ```bash
        echo $XDG_RUNTIME_DIR
    ```

    ```bash
      docker run -it \
                 --network=host \
                 --ipc=host \
                 -v /run/user/<echo $XDG_RUNTIME_DIR UID>/<echo $WAYLAND_DISPLAY>:/run/user/<echo $XDG_RUNTIME_DIR UID>/<echo $WAYLAND_DISPLAY> \
                 -e WAYLAND_DISPLAY=<echo $WAYLAND_DISPLAY> \
                 -e XDG_RUNTIME_DIR=/run/user/<echo $XDG_RUNTIME_DIR UID> \
                 -e GDK_BACKEND=wayland \
                 <image_name> \
                 <terminal view (bash - /bin/bash)>
    ```

# Saving Docker State with `docker commit`

When making modifications within a running Docker container (such as installing new dependencies or making system
changes), you can save these changes as a new Docker image using `docker commit`. This allows you to preserve your
modifications without needing to rebuild the container from scratch.

#### **Steps to Save and Reuse a Modified Docker Container**

1. **Check running containers**
   ```bash
   docker ps
   ```
   Find the `CONTAINER ID` or `NAME` of the running container you want to save.

2. **Create a snapshot of the current container state**
   ```bash
   docker commit <container_id> trb_snapshot
   ```
   Replace `<container_id>` with the actual ID of your container.

3. **Run a new container from the saved snapshot**
   ```bash
   sudo xhost +local:docker
   sudo docker run -it --privileged \
       --network=host \
       --ipc=host \
       -v /tmp/.x11-unix:/tmp/.X11-unix:rw \
       --env=DISPLAY \
       trb_snapshot
   ```

#### **Notes:**

- The snapshot (`trb_snapshot`) can be used as a base for further development.
- This method allows you to keep changes without modifying the original `Dockerfile`.
- If you want to make the changes permanent, consider updating the `Dockerfile` and rebuilding the image
  using `docker build`.

# Workflow

## Branches

- **main**
    - main branch, should be always stable - every pull request to this branch should be reviewed by at least 2-3
      persons.
    - merge to this branch can be done only by the project manager.
    - merge to this branch can be done from **dev** branch.
    - every merge means new version of the project.

- **dev**
    - development branch
    - merge to this branch can be done only if all feature tests are passed.
    - merge to this branch can be only done by feature reviewer
    - acceptance criteria for merge to this branch:
        - have all functionality tests passed.
    - merge to this branch means end of milestone.

- **feature**
    - branches for specific features
    - use to develop new features

- **bug**
    - branches for specific bugs
    - use to fix bugs from feature branch

- **test**
    - branches for specific features
    - use only to test specific feature - do not add new features to this branch

```markdown
├── **main**
│ └── **dev**
│ ├── **feature**
│ │ ├── **bug**
│ │ └── **test**          
│ │
│ ├── **feature**
│ │ ├── **bug**
│ │ └── **test**
│ │
```

## Create Branches

- Create a new branch for each feature or bug fix.
- Branch names should be descriptive:
    ```markdown
        [feature/bug/test]/[milestone]/[short-description]
    ```
  Example:
    ```markdown
        feature/ros_introduction/start_with_ros
    ```

## Commit Messages

- Commit messages should be descriptive - **obligatory use
  of [Conventional Commits](https://www.conventionalcommits.org/en/v1.0.0/)**

## Issues

- Issues have to be created with templates provided in the repository
    - Issue template - create issue ticket
    - Bug report - report a bug for specific issue
- **Every issue have to have assignees and reviewer**

## Pull Requests

- Pull requests should be descriptive and have annotation to issue

