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
   
# Main engineers 

- [Daniel Kleczyński](https://github.com/Kleczyk)
- [Filip Walkowicz](https://github.com/FWalkowicz)
- [Jakub Bartecki](https://github.com/kubabartecki)
- [Mateusz Sygnator](https://github.com/Sygnator)
- [Damian Kobylinski](https://github.com/DamianKobylinski)
