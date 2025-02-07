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

# Docker 

## How to build the docker image

```bash
 sudo docker build -t <image_name e.g: trailblazerml> .
```

## How to run the docker image

### Linux 
1. X11
    ```bash
        sudo docker run -it \
                        --network=host \
                        --ipc=host \
                        -v /tmp/.x11-unix:/tmp/.X11-unix:rw \
                        --env=DISPLAY <image_name> \ 
                        <terminal view (bash - /bin/bash)>
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
│     └── **dev**
│           ├── **feature** 
│           │      ├── **bug**
│           │      └── **test**          
│           │
│           ├── **feature** 
│           │      ├── **bug**
│           │      └── **test**
│           │
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

