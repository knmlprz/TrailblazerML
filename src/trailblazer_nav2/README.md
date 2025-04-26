# Trailblazer nav2

## 📚 Spis treści

- [📁 Struktura katalogów](#-struktura-katalogów)
- [🗺️ GPS i GNNS](#gps-i-gnss)
- [🧭 Lokalizacja robota](#lokalizacja-robota)
- [📦 Konfiguracja lokalizacji](#konfiguracja-lokalizacji)
- [🛠️ Jak używać](#️-jak-używać)

## 📁 Struktura katalogów
    src/trailblazer_nav2
    ├── config              # Pliki konfiguracyjne
    ├── launch              # Pliki startowe
    ├── maps                # Mapy
    ├── package.xml         # Zależności, opis, wersja
    ├── setup.py            # Plik instalacyjny
    └── trailblazer_nav2    # Moduł paczki

## 🗺️ GPS i GNNS
### Co to jest?
GPS/GNSS to systemy, które używają satelit, by określić Twoje położenie (szerokość, długość i wysokość geograficzną).Standardowo pozycję podaje się w układzie ***WGS84*** – to taki "globalny" system XYZ, którego środek jest w środku Ziemi.
<div align="center">
  <img src="images/image.png" width="500" height="500">
</div>

Nawigowanie robota w takim systemie byłoby niewygodne — lepiej mówić mu "jedź 100 metrów na północ", a nie przesuwaj się o 0.001 stopnia szerokości geograficznej.

### Co to UTM?
Aby rozwiązać ten problem, używa się systemu UTM (Universal Transverse Mercator).
UTM dzieli Ziemię na strefy i tworzy lokalne układy współrzędnych w metrach, co znacznie ułatwia nawigację dla robotów bo zamiast globalnych współrzędnych mamy lokalne X i Y w metrach.
<div align="center">
  <img src="images/image-1.png" width="500" height="600">
</div>

## 🧭 Lokalizacja robota
Robot musi wiedzieć gdzie jest i w którą stronę patrzy. GPS daje tylko pozycję, ale nie orientację (czyli "w którą stronę przód robota jest skierowany"). Rozwiązaniem tego jest zastosowanie IMU (Inertial Measurement Unit).


## 🛠️ Jak używać
### ✅ Zbudowanie paczki
```bash
cd ~/TrailblazerML
colcon build --packages-select trailblazer_nav2
source install/setup.bash
```
### 🚀 Uruchomienie symulacji samodzielnie
```bash

```

### 🧩 Integracja z innymi paczkami
```python

```