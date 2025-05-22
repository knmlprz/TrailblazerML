# TrailblazerML Docker Setup 🚀

---

Ten dokument zawiera przegląd konfiguracji Dockera dla projektu TrailblazerML, w tym instrukcje dotyczące budowania, uruchamiania i zarządzania kontenerem Docker na architekturach ARM (ze wsparciem do testowania na x86_64).

# 📋 Spis treści

- [Przegląd](#-przegląd)
- [Wymagania wstępne](#️-wymagania-wstępne)
- [Budowanie obrazu Dockera](#️-budowanie-obrazu-dockera)
  - [Architektura ARM](#architektura-arm)
  - [Architektura x86_64 (Testowanie)](#architektura-x86_64-testowanie)

- [Uruchamianie kontenera Docker](#-uruchamianie-kontenera-docker)
    - [Architektura ARM](#architektura-arm-1)
    - [Architektura x86_64 (Testowanie)](#architektura-x86_64-testowanie-1)

- [Uruchamianie istniejącego kontenera](#-uruchamianie-istniejącego-kontenera)
- [Zapisywanie stanu kontenera Docker](#-zapisywanie-stanu-kontenera-docker)
- [Wyjaśnienie Dockerfile](#-wyjaśnienie-dockerfile)
  - [Obraz bazowy](#obraz-bazowy)
  - [Zmienne środowiskowe](#zmienne-środowiskowe)
  - [Zależności systemowe](#zależności-systemowe)
  - [Zależności Pythona](#zależności-pythona)
  - [Konfiguracja ROS 2](#konfiguracja-ros-2)
  - [Konfiguracja projektu](#konfiguracja-projektu)
  - [Entrypoint i polecenie](#entrypoint-i-polecenie)

# 📖 Przegląd
Projekt TrailblazerML wykorzystuje Dockera do stworzenia spójnego środowiska deweloperskiego do pracy z ROS 2 (Humble) na architekturach ARM. Dostarczony Dockerfile konfiguruje środowisko ze wszystkimi niezbędnymi zależnościami, w tym ROS 2, Python 3.10 oraz narzędziami takimi jak Terminator. To zapewnia przenośność i powtarzalność na różnych systemach, z obsługą testowania na x86_64 przy użyciu emulacji.

# ⚠️ Wymagania wstępne
Przed użyciem konfiguracji Dockera upewnij się, że:

1. Docker jest zainstalowany na Twoim systemie.
2. Do testowania na x86_64 zainstaluj QEMU do emulacji ARM: sudo apt-get install qemu-user-static
    - Skonfiguruj przekazywanie X11 dla aplikacji graficznych (np. RViz2, Terminator):
      Dla Bash: echo 'xhost +local:docker' >> ~/.bashrc
    To pozwala kontenerowi Docker na dostęp do wyświetlacza hosta dla aplikacji graficznych.


# 🛠️ Budowanie obrazu Dockera

## Architektura ARM

---

Zbudowanie Docker obrazu na systemie ARM:

```bash
  sudo docker build -t trb_1 .
```

# x86_64 Architecture (Testing)

Zbudowanie obrazu Docker na systemie x86_64 do testowania na emulacji ARM:

```bash
    sudo docker run --rm --privileged multiarch/qemu-user-static --reset -p yes
    sudo docker buildx create --use
    sudo docker buildx build --platform linux/arm64 -t trb_1 --load .
```

Pierwsze polecenie ustawia QEMU do emulacji ARM64.  
Drugie polecenie inicjalizuje buildx do budowania na wiele architektur.  
Trzecie polecenie buduje obraz dla architektury ARM64.

# 🚀 Uruchamianie kontenera Docker

---

## Architektura ARM

Aby uruchomić kontener na systemie opartym o ARM:

```
sudo docker run -it --platform linux/arm64 --name trb_1_arm --privileged --network=host --ipc=host -v /dev:/dev trb_1
```

- --platform linux/arm64: Określa platformę ARM64.
- --name trb_1_arm: Nadaje kontenerowi nazwę trb_1_arm.
- --privileged: Przyznaje rozszerzone uprawnienia (wymagane do dostępu do urządzeń).
- --network=host: Używa stosu sieciowego hosta.
- --ipc=host: Udostępnia przestrzeń IPC hosta.
- -v /dev:/dev: Montuje katalog /dev z hosta, umożliwiając dostęp do sprzętu.

## Architektura x86_64 (Testowanie)

Aby uruchomić kontener na systemie x86_64 do testowania:

```bash
  sudo docker run -it --platform linux/arm64 --name trb_1_arm --privileged --network=host --ipc=host -e DISPLAY=$DISPLAY -e WAYLAND_DISPLAY=$WAYLAND_DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix -v /run/user/$(id -u)/wayland-0:/run/user/$(id -u)/wayland-0 -v /dev:/dev trb_1
```

Dodatkowe flagi (-e DISPLAY, -v /tmp/.X11-unix, itd.) umożliwiają obsługę GUI dla narzędzi takich jak RViz2 i Terminator. To polecenie zapewnia kompatybilność z X11 i Wayland dla aplikacji graficznych.

# 🔄 Uruchamianie istniejącego kontenera

Aby ponownie wejść do wcześniej utworzonego kontenera (`trb_1_arm`) w tym samym stanie:


```bash
  sudo docker start -ai trb_1_arm
```

To polecenie uruchamia kontener w trybie interaktywnym, zachowując jego poprzedni stan.

# 💾 Zapisywanie stanu kontenera Docker
Aby zapisać zmiany wprowadzone wewnątrz działającego kontenera (np. nowe zależności lub konfiguracje):

Sprawdź uruchomione kontenery, aby znaleźć ID lub nazwę kontenera:
```
docker ps -a
```

Utwórz migawkę aktualnego stanu kontenera:
```
docker commit trb_1_arm trb_snapshot
```

Uruchom nowy kontener z zapisanej migawki:
```
sudo docker run -it --name trb_new --privileged --network=host --ipc=host --env=DISPLAY trb_snapshot
```

# 📜 Wyjaśnienie pliku Dockerfile

Plik Dockerfile konfiguruje środowisko deweloperskie dla TrailblazerML z ROS 2 Humble i powiązanymi zależnościami. Poniżej znajduje się szczegółowy opis jego elementów.

## Obraz bazowy: 
- FROM arm64v8/ubuntu:22.04

Używa Ubuntu 22.04 dla architektury ARM64 jako obrazu bazowego.

## Zmienne środowiskowe
- ENV DEBIAN_FRONTEND=noninteractive
- ENV LANG=en_US.UTF-8
- ENV LC_ALL=en_US.UTF-8
- ENV ROS_PYTHON_VERSION=3
- ENV ROS_DISTRO=humble

## Dodatkowe zmienne środowiskowe
- DEBIAN_FRONTEND=noninteractive: Zapobiega interaktywnym zapytaniom podczas instalacji pakietów.
- LANG i LC_ALL: Ustawia lokalizację na UTF-8 dla spójnego przetwarzania tekstu.
- ROS_PYTHON_VERSION=3: Określa użycie Pythona 3 dla ROS 2.
- ROS_DISTRO=humble: Ustawia dystrybucję ROS 2 na Humble.

## Zależności systemowe
 curl \
 locales \
 software-properties-common \
 git \
 build-essential \
 ...
 terminator \
 && locale-gen en_US.UTF-8 \
 && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 \
 && apt-get clean && rm -rf /var/lib/apt/lists/*

Instaluje podstawowe narzędzia i biblioteki (np. curl, git, python3.10, terminator).
Konfiguruje lokalizację na en_US.UTF-8.
Czyści pamięć podręczną apt, aby zmniejszyć rozmiar obrazu.

## Zależności Pythona
RUN pip install --no-cache-dir numpy lxml

Instaluje pakiety Python numpy i lxml bez buforowania, aby zminimalizować rozmiar obrazu.

## Konfiguracja ROS 2
RUN add-apt-repository universe \
    && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu jammy main" > /etc/apt/sources.list.d/ros2.list

RUN apt-get update && apt-get install -y \
    python3-flake8-docstrings \
    python3-pytest-cov \
    ...
    ros-humble-desktop \
    ros-dev-tools \
    ...
    && apt-get clean && rm -rf /var/lib/apt/lists/*

Dodaje repozytorium i klucz ROS 2.
Instaluje ROS 2 Humble (ros-humble-desktop), narzędzia deweloperskie (ros-dev-tools) oraz wybrane pakiety ROS (np. ros-humble-rviz2, ros-humble-xacro).
Instaluje dodatkowe narzędzia do lintowania i testowania kodu w Pythonie.

## Konfiguracja projektu
COPY . /TrailblazerML/

RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc \
    && echo "source /usr/share/colcon-argcomplete/hook/colcon-argcomplete.bash" >> ~/.bashrc \
    && bash -c "source ~/.bashrc"

RUN apt-get update

RUN rosdep init && rosdep update

WORKDIR /TrailblazerML
RUN bash -c "rosdep install --from-paths src --ignore-src -r -y"

Kopiuje pliki projektu do katalogu /TrailblazerML/.
Konfiguruje środowisko przez dodanie do ~/.bashrc źródłowania ROS 2 i Colcon.
Inicjuje i aktualizuje rosdep, aby zainstalować zależności dla workspace ROS 2.
Ustawia katalog roboczy na /TrailblazerML i instaluje zależności specyficzne dla projektu.

## Entrypoint i polecenie
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/bin/bash", "/entrypoint.sh"]
CMD ["bash"]


Kopiuje skrypt entrypoint.sh i nadaje mu uprawnienia do uruchamiania.
Ustawia entrypoint na uruchomienie entrypoint.sh z /bin/bash.
Domyślnie uruchamia interaktywną powłokę Bash.