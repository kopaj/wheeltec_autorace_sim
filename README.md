# Wheeltec AutoRace Simulation

ROS 2 Humble és Gazebo Fortress alapú szimulációs környezet Wheeltec / Roboworks robot autonóm sávtartásának fejlesztéséhez.

A projekt célja egy kameraalapú pályakövető rendszer megvalósítása, amely egy 2D-s, fehér vonalakkal határolt tesztpályán képes a robotot autonóm módon végigvezetni. A jelenlegi verzióban a lane perception és a PID vezérlés már egy teljes kört sikeresen teljesített a generált tesztpályán.

## Fejlesztési környezet

- Ubuntu 22.04 LTS
- ROS 2 Humble
- Gazebo Fortress
- `ros_gz`
- OpenCV
- `cv_bridge`
- `rqt_image_view`

## ROS 2 csomagok

### `wheeltec_autorace_description`

A Wheeltec / Roboworks robotmodell, szenzorok, mesh-ek és a pályamodell erőforrásai.

### `wheeltec_autorace_gazebo`

A Gazebo világfájlok és a szimulációs környezet.

### `wheeltec_autorace_bringup`

A szimuláció indításához szükséges ROS 2 launch fájlok és a ROS-Gazebo bridge-ek.

A `simulation.launch.py` jelenleg elindítja:

- a Gazebo Fortress szimulációt,
- a Wheeltec / Roboworks modellt,
- a szükséges ROS-Gazebo bridge-eket,
- a `lane_perception` node-ot a hozzá tartozó YAML konfigurációval,
- az `rqt_image_view` alkalmazást.

A PID controller egyelőre külön indul, hogy a vezérlő hangolása a szimulációtól függetlenül végezhető legyen.

### `wheeltec_autorace_application`

A kamera feldolgozásáért, sávfelismerésért és a pályakövető vezérlésért felelős node-ok.

Jelenleg:

- `lane_perception`
- `pid_lane_controller`

## Lane perception pipeline

A jelenlegi vezérléshez használt sávfelismerés nem kizárólag Hough-egyenesekből dolgozik. A Hough-transzformáció megmaradt diagnosztikai célra, a tényleges pályakövetés viszont bird's-eye nézetben történik.

A fő feldolgozási lánc:

```text
/camera/image_raw
        |
        v
HSV alapú fehér sávszegmentálás
        |
        v
ROI + perspektívatranszformáció
        |
        v
Bird's-eye view
        |
        v
Hisztogram alapú kezdőpont-keresés
        |
        v
Sliding window sávkövetés
        |
        v
Bal és jobb sáv görbeillesztése
        |
        v
Sávközép + előretekintési pont
        |
        v
control_error_normalized
        |
        v
PID controller
        |
        v
/cmd_vel
        |
        v
Ackermann steering
```

A perception külön debug képeket is publikál, így futás közben ellenőrizhető a szegmentálás, a bird's-eye transzformáció, a sliding window keresés és a becsült középvonal.

## Generált tesztpálya

A pálya már nem kézzel rajzolt textúrából készül. A projektben található Python script reprodukálható módon generálja a tesztpályát.

A jelenlegi tesztpálya főbb adatai:

- fizikai méret: `9.1 x 9.1 m`
- sávszélesség: kb. `0.74 m`
- fehér vonalvastagság: `0.05 m`
- kanyarsugár: `0.85 m`
- több, egymástól egyenes szakasszal elválasztott 90 fokos irányváltás

A pálya tervezésénél figyelembe lett véve a Wheeltec Ackermann modell kormányzási korlátja:

```text
steering_limit = 0.4 rad
wheel_base     = 0.262 m
```

A generált pálya célja, hogy a robotnak ne kelljen minden kanyarban a fizikai kormányzási limit közelében haladnia.

### Pálya újragenerálása

A repository gyökeréből:

```bash
cd ~/wheeltec_autorace_ws/src/wheeltec_autorace_sim

python3 tools/generate_multicorner_track.py \
  --output \
  wheeltec_autorace_description/models/racetrack/materials/textures/course.png \
  --debug-output \
  /tmp/course_multicorner_debug.png
```

A debug változat megnyitható:

```bash
xdg-open /tmp/course_multicorner_debug.png
```

A Gazebo a `course.png` fájlt használja.

## Build

Lépj a workspace gyökerébe:

```bash
cd ~/wheeltec_autorace_ws
```

Töltsd be a ROS 2 Humble környezetet:

```bash
source /opt/ros/humble/setup.bash
```

A függőségek telepítése:

```bash
rosdep install \
  --from-paths src \
  --ignore-src \
  -r \
  -y \
  --rosdistro humble
```

Build:

```bash
colcon build --symlink-install
```

A workspace betöltése:

```bash
source ~/wheeltec_autorace_ws/install/setup.bash
```

Minden új terminálban szükséges:

```bash
source /opt/ros/humble/setup.bash
source ~/wheeltec_autorace_ws/install/setup.bash
```

## Szimuláció indítása

A teljes szimuláció:

```bash
ros2 launch \
  wheeltec_autorace_bringup \
  simulation.launch.py
```

A launch jelenleg automatikusan elindítja a Gazebót, a lane perception node-ot és az `rqt_image_view`-t is.

Az `rqt_image_view` listájából fejlesztés közben főleg ezeket érdemes figyelni:

```text
/camera/image_raw
/lane_detection/debug_image
/lane_detection/white_mask
/lane_detection/edges
/lane_detection/birdseye_mask
/lane_detection/birdseye_debug
```

## PID controller indítása

A PID controller jelenleg külön terminálból indul.

Új terminál:

```bash
source /opt/ros/humble/setup.bash
source ~/wheeltec_autorace_ws/install/setup.bash
```

Controller:

```bash
ros2 run \
  wheeltec_autorace_application \
  pid_lane_controller \
  --ros-args \
  --params-file \
  ~/wheeltec_autorace_ws/install/wheeltec_autorace_application/share/wheeltec_autorace_application/config/pid_lane_controller.yaml
```

Engedélyezés:

```bash
ros2 service call \
  /pid_lane_controller/enable \
  std_srvs/srv/SetBool \
  "{data: true}"
```

Leállítás:

```bash
ros2 service call \
  /pid_lane_controller/enable \
  std_srvs/srv/SetBool \
  "{data: false}"
```

A letiltás nullás sebességparancsot küld a robotnak.

## Hasznos topicok

Perception:

```text
/camera/image_raw
/lane_detection/valid
/lane_detection/degraded
/lane_detection/cross_track_error_normalized
/lane_detection/control_error_normalized
/lane_detection/heading_error
/lane_detection/debug_image
/lane_detection/birdseye_debug
```

Vezérlés:

```text
/control/pid/output
/cmd_vel
```

Például:

```bash
ros2 topic echo /lane_detection/control_error_normalized
```

```bash
ros2 topic echo /control/pid/output
```

```bash
ros2 topic echo /cmd_vel
```

## Jelenlegi állapot

A projekt jelenlegi mérföldkövei:

- Wheeltec / Roboworks robotmodell Gazebo Fortress alatt
- Ackermann steering és `/cmd_vel` vezérlés
- RGB kamera ROS 2 bridge
- generált, kinematikai korlátokhoz igazított 2D tesztpálya
- HSV alapú sávszegmentálás
- Canny + Hough diagnosztikai feldolgozás
- bird's-eye nézet
- sliding window alapú sávkövetés
- görbealapú sávközép-becslés
- PID alapú autonóm sávtartás
- sikeres teljes autonóm kör a generált tesztpályán

A következő fejlesztési lépések között szerepel a PID további kiértékelése, valamint a Pure Pursuit vezérlő implementálása és összehasonlítása.
