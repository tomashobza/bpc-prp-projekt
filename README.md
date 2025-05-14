# Robotický projekt BPC-PRP

Repozitář obsahuje implementaci modulárního řídícího systému pro autonomního robota, který je schopen navigace v koridorech, detekce křižovatek a rozhodování na základě ArUco markerů.

## Autoři

- **Tomáš Hobza** (xhobza03)
- **Daniel Jacobs** (xjacob00)

## Struktura repozitáře

```
.
├── Dockerfile             # Konfigurace pro Docker
├── Makefile               # Build automatizace
├── paper                  # Dokumentace
│   └── bpc_prp_paper.pdf
├── setup.bash             # Skript pro nastavení prostředí
└── src
    └── robots
        ├── CMakeLists.txt
        ├── include        # Hlavičkové soubory
        │   ├── algorithms
        │   │   ├── aruco_detector.hpp
        │   │   ├── kinematics.hpp
        │   │   ├── pid.h
        │   │   ├── planar_imu_integrator.hpp
        │   │   └── turns.hpp
        │   ├── CameraNode.h
        │   ├── ControlNode.h
        │   ├── EncoderNode.h
        │   ├── ImuNode.h
        │   ├── LidarNode.h
        │   ├── LineEstimator.h
        │   ├── LineNode.h
        │   ├── MotorNode.h
        │   └── robots
        │       └── io_node.hpp
        ├── package.xml    # Informace o ROS 2 balíčku
        ├── README.md
        ├── src            # Zdrojové kódy
        │   ├── io_node.cpp
        │   └── main.cpp   # Hlavní vstupní bod programu
        └── test           # Testovací soubory
            ├── CMakeLists.txt
            └── my_test.cpp
```

## Požadavky

- ROS 2 (testováno s Humble Hawksbill)
- OpenCV s ArUco modulem
- Kompilátor podporující C++17
- Docker (volitelné, pro kontejnerizované nasazení)

## Sestavení projektu

Pomocí poskytnutého Makefile:

```bash
# Klonování repozitáře
git clone https://github.com/vaše-uživatelské-jméno/bpc-prp-projekt.git
cd bpc-prp-projekt

# Sestavení projektu
make build
```

Nebo pomocí Dockeru:

```bash
# Sestavení Docker image
make docker-build

# Spuštění Docker kontejneru
make docker-run
```

## Spuštění řídícího systému robota

```bash
# Načtení nastavení prostředí
source setup.bash

# Spuštění hlavního uzlu
ros2 run robots robot_controller
```

## Testování

Testy lze spustit pomocí:

```bash
make test
```

## Licence

Projekt je určen pouze pro studijní účely kurzu BPC-PRP na VUT v Brně.
