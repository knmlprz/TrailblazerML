# Use an official Ubuntu base image
FROM ubuntu:22.04

ENV DEBIAN_FRONTEND=noninteractive
ARG PYTHON_VERSION=3.10.0

RUN apt-get update && apt-get install -y \
    curl locales software-properties-common git build-essential libssl-dev zlib1g-dev libbz2-dev libreadline-dev \
    libsqlite3-dev wget llvm libncursesw5-dev xz-utils tk-dev libxml2-dev \
    libxmlsec1-dev libffi-dev ca-certificates && \
    apt-get clean && rm -rf /var/lib/apt/lists/*

RUN locale-gen en_US.UTF-8 && \
    update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

ENV PYENV_ROOT="/root/.pyenv"
ENV PATH="$PYENV_ROOT/bin:$PYENV_ROOT/shims:$PATH"
ENV LANG=en_US.UTF-8

RUN curl -L https://github.com/pyenv/pyenv-installer/raw/master/bin/pyenv-installer | bash

RUN bash -l -c "pyenv install $PYTHON_VERSION && pyenv global $PYTHON_VERSION"

RUN add-apt-repository universe && \
    rm -rf /var/lib/apt/lists/*

RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

RUN sh -c 'echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" > /etc/apt/sources.list.d/ros2.list'

RUN apt-get update && apt-get install -y \
  python3-flake8-docstrings \
  python3-pip python3-pytest-cov \
  python3-flake8-blind-except \
  python3-flake8-builtins \
  python3-flake8-class-newline \
  python3-flake8-comprehensions \
  python3-flake8-deprecated \
  python3-flake8-import-order \
  python3-flake8-quotes \
  python3-pytest-repeat \
  python3-colcon-common-extensions \
  python3-pytest-rerunfailures \
  ros-dev-tools \
  ros-humble-desktop  \
  python3-colcon-common-extensions \
  ros-humble-gazebo-ros-pkgs \
  ros-humble-ros2-control \
  ros-humble-ros2-controllers \
  ros-humble-gazebo-ros2-control \
  ros-humble-position-controllers \
#  ros-humble-controller-manager-spawner \
  ros-humble-xacro \
  joystick \
  jstest-gtk \
  evtest \
  ros-humble-twist-mux \
  ros-humble-rviz2 \
  ros-humble-gazebo-ros2-control && \
  rm -rf /var/lib/apt/lists/*

COPY . /Traiblazer/

RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
RUN echo "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" >> ~/.bashrc

RUN bash -c "source ~/.bashrc"

COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

#ENTRYPOINT ["/entrypoint.sh"]

CMD ["bash"]
