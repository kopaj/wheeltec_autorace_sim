# Wheeltec AutoRace Simulation

A projekt célja egy ROS 2 Humble és Gazebo Fortress alapú szimulációs környezet létrehozása a Wheeltec (Roboworks) robothoz. A robot egy kamerával érzékeli a pálya két fehér szélét, ezekből megbecsüli a sáv középvonalát, majd egy PID szabályozó segítségével generálja a haladási és kormányzási parancsokat.

A fejlesztés jelenleg szimulációban fut, de a csomagstruktúra és a fontosabb paraméterek úgy vannak kialakítva, hogy később a vezérlés a fizikai robotra is átvihető legyen.

## Környezet

A projekt az alábbi környezetre készült:

- Ubuntu 22.04 LTS
- ROS 2 Humble
- Gazebo Fortress
- `ros_gz`
- OpenCV
- `cv_bridge`
- `rqt_image_view`

## Csomagok

### `wheeltec_autorace_description`

A Wheeltec robot modelljei, szenzorai, mesh fájljai és a pálya modellje található itt.

### `wheeltec_autorace_gazebo`

A Gazebo world fájlokat tartalmazza. A projektben használt versenypálya innen kerül betöltésre.

### `wheeltec_autorace_bringup`

A szimuláció indításához szükséges ROS 2 launch fájlokat és a ROS–Gazebo bridge konfigurációját tartalmazza.

### `wheeltec_autorace_application`

A kameraképet feldolgozó sávfelismerés és a pályakövetéshez használt vezérlési algoritmusok találhatók ebben a csomagban.

Jelenleg két fontos node tartozik ide:

- `lane_perception`
- `pid_lane_controller`

---

## Lane perception pipeline

A sávfelismerés bemenete a robot RGB kamerájának képe:

```text
/camera/image_raw
```

A jelenlegi feldolgozási lánc röviden:

```text
RGB kamerakép
    |
    v
HSV színtér + fehér sávok maszkolása
    |
    v
morfológiai szűrés
    |
    v
perspektíva-transzformáció (bird's-eye view)
    |
    v
alsó képrész hisztogramja
    |
    v
sliding-window sávkeresés
    |
    v
2. fokú polinom illesztése a bal és jobb sávra
    |
    v
sávközép + előretekintési pont
    |
    v
normalizált vezérlési hibajel
    |
    v
PID vezérlő
    |
    v
/cmd_vel
```

A Canny élkeresés és a Hough-transzformáció továbbra is része a perception node-nak, de jelenleg főleg diagnosztikai és vizualizációs célra használjuk. A vezérléshez a bird's-eye nézetből, sliding-window módszerrel követett görbült sávhatárok adják a középvonalat. Ez az élesebb kanyaroknál stabilabb, mint amikor a teljes sávhatárt egyetlen egyenessel próbáljuk közelíteni.

Ha az egyik sávhatár rövid időre nem használható, a perception az utoljára ismert sávgeometria alapján becslést készít. Ha a sávkövetés teljesen elveszik, a PID vezérlő recovery módba tud váltani, majd ha a sáv újra felismerhető, visszatér a normál követéshez.

A jelenlegi verzió az egyenes és egyszerűbb kanyargós szakaszokat már végig tudja követni. Az összetettebb, egymást gyorsan követő kanyarok robusztus kezelése még további finomhangolást igényel.

---

# Telepítés és build

## 1. Workspace létrehozása

Ha még nincs workspace:

```bash
mkdir -p ~/wheeltec_autorace_ws/src
cd ~/wheeltec_autorace_ws/src
```

A repository klónozása:

```bash
git clone https://github.com/kopaj/wheeltec_autorace_sim.git
```

Ha a repository már megvan, ez a lépés kihagyható.

## 2. ROS 2 környezet betöltése

```bash
source /opt/ros/humble/setup.bash
```

## 3. Függőségek telepítése

A workspace gyökeréből:

```bash
cd ~/wheeltec_autorace_ws

rosdep install \
  --from-paths src \
  --ignore-src \
  -r \
  -y \
  --rosdistro humble
```

Ha szükséges, az alapvető képfeldolgozó és megjelenítő csomagok külön is telepíthetők:

```bash
sudo apt update
sudo apt install -y \
  ros-humble-cv-bridge \
  ros-humble-rqt-image-view \
  libopencv-dev
```

## 4. Build

```bash
cd ~/wheeltec_autorace_ws

source /opt/ros/humble/setup.bash

colcon build --symlink-install
```

Fejlesztés közben, ha csak az application és bringup csomag változott, gyorsabb lehet:

```bash
colcon build \
  --symlink-install \
  --packages-select \
  wheeltec_autorace_application \
  wheeltec_autorace_bringup
```

## 5. Workspace source-olása

Minden új terminálban szükséges:

```bash
source /opt/ros/humble/setup.bash
source ~/wheeltec_autorace_ws/install/setup.bash
```

Ha ez kimarad, a saját csomagok vagy executable-ök nem biztos, hogy elérhetők lesznek.

---

# A projekt indítása lépésről lépésre

Fejlesztés közben érdemes a fő komponenseket külön terminálokban futtatni. Így könnyebben látható, hogy melyik node mit ír ki és hol jelenik meg egy esetleges hiba.

> Ha a saját `simulation.launch.py` verziód már automatikusan elindítja a `lane_perception` vagy `pid_lane_controller` node-ot is, az adott node-ot ne indítsd el még egyszer külön terminálban. Ezt a `ros2 node list` paranccsal tudod ellenőrizni.

## Terminál 1 – Gazebo szimuláció

```bash
source /opt/ros/humble/setup.bash
source ~/wheeltec_autorace_ws/install/setup.bash

ros2 launch wheeltec_autorace_bringup simulation.launch.py
```

A launch elindítja a Gazebo környezetet, betölti a Wheeltec modellt és létrehozza a szükséges ROS–Gazebo kapcsolatokat.

A robot vezérlési parancsa ROS oldalon:

```text
/cmd_vel
```

A kamera fő ROS topicja:

```text
/camera/image_raw
```

## Terminál 2 – Lane perception

Először ellenőrizhető, hogy már fut-e:

```bash
ros2 node list | grep lane_perception
```

Ha nincs találat, indítsd el külön:

```bash
source /opt/ros/humble/setup.bash
source ~/wheeltec_autorace_ws/install/setup.bash

ros2 run \
  wheeltec_autorace_application \
  lane_perception \
  --ros-args \
  --params-file \
  $(ros2 pkg prefix wheeltec_autorace_application)/share/wheeltec_autorace_application/config/lane_perception.yaml
```

Néhány fontos perception topic:

```text
/lane_detection/debug_image
/lane_detection/white_mask
/lane_detection/birdseye_mask
/lane_detection/birdseye_debug
/lane_detection/valid
/lane_detection/degraded
/lane_detection/cross_track_error_normalized
/lane_detection/control_error_normalized
/lane_detection/heading_error
```

A vezérlés szempontjából a legfontosabb jel jelenleg:

```text
/lane_detection/control_error_normalized
```

## Terminál 3 – `rqt_image_view`

```bash
source /opt/ros/humble/setup.bash
source ~/wheeltec_autorace_ws/install/setup.bash

rqt_image_view
```

Első ellenőrzéshez érdemes ezeket a topicokat végignézni:

```text
/camera/image_raw
/lane_detection/white_mask
/lane_detection/debug_image
/lane_detection/birdseye_debug
```

A `birdseye_debug` mutatja a sliding-window keresést és a vezérléshez használt sávgeometriát, ezért PID hangolásnál ez a leghasznosabb nézet.

## Terminál 4 – PID sávtartó vezérlő

Először ellenőrizd, hogy már fut-e:

```bash
ros2 node list | grep pid_lane_controller
```

Ha nem fut:

```bash
source /opt/ros/humble/setup.bash
source ~/wheeltec_autorace_ws/install/setup.bash

ros2 run \
  wheeltec_autorace_application \
  pid_lane_controller \
  --ros-args \
  --params-file \
  $(ros2 pkg prefix wheeltec_autorace_application)/share/wheeltec_autorace_application/config/pid_lane_controller.yaml
```

A PID biztonsági okból alapértelmezetten nem indítja el rögtön a robotot.

Bekapcsolás:

```bash
ros2 service call \
  /pid_lane_controller/enable \
  std_srvs/srv/SetBool \
  "{data: true}"
```

Kikapcsolás és megállítás:

```bash
ros2 service call \
  /pid_lane_controller/enable \
  std_srvs/srv/SetBool \
  "{data: false}"
```

A PID fontosabb debug topicjai:

```text
/control/pid/p_term
/control/pid/i_term
/control/pid/d_term
/control/pid/output
/control/pid/active
/control/pid/recovery_active
```

A ténylegesen kiküldött sebesség- és kormányparancs megfigyelhető:

```bash
ros2 topic echo /cmd_vel
```

---

# Gyors indítás

Ha a projekt már egyszer le lett fordítva, a szokásos fejlesztői indítás röviden:

### 1. Szimuláció

```bash
source /opt/ros/humble/setup.bash
source ~/wheeltec_autorace_ws/install/setup.bash
ros2 launch wheeltec_autorace_bringup simulation.launch.py
```

### 2. Lane perception, ha a launch nem indította el

```bash
source /opt/ros/humble/setup.bash
source ~/wheeltec_autorace_ws/install/setup.bash
ros2 run wheeltec_autorace_application lane_perception --ros-args \
  --params-file $(ros2 pkg prefix wheeltec_autorace_application)/share/wheeltec_autorace_application/config/lane_perception.yaml
```

### 3. Képmegjelenítés

```bash
rqt_image_view
```

### 4. PID controller, ha a launch nem indította el

```bash
source /opt/ros/humble/setup.bash
source ~/wheeltec_autorace_ws/install/setup.bash
ros2 run wheeltec_autorace_application pid_lane_controller --ros-args \
  --params-file $(ros2 pkg prefix wheeltec_autorace_application)/share/wheeltec_autorace_application/config/pid_lane_controller.yaml
```

### 5. Autonóm követés engedélyezése

```bash
ros2 service call /pid_lane_controller/enable \
  std_srvs/srv/SetBool "{data: true}"
```

---

# Hasznos ellenőrző parancsok

Futó node-ok:

```bash
ros2 node list
```

Lane perception topicok:

```bash
ros2 topic list | grep lane_detection
```

PID topicok:

```bash
ros2 topic list | grep control/pid
```

A perception érvényességének figyelése:

```bash
ros2 topic echo /lane_detection/valid
```

A vezérlési hibajel figyelése:

```bash
ros2 topic echo /lane_detection/control_error_normalized
```

A kiküldött mozgásparancs:

```bash
ros2 topic echo /cmd_vel
```

---

# Fejlesztési állapot

Jelenleg működik:

- Wheeltec robot Gazebo Fortress környezetben
- egyedi, textúrázott 2D versenypálya
- RGB kamera és ROS 2 image bridge
- fehér sávok HSV alapú szegmentálása
- Canny + Hough diagnosztikai feldolgozás
- bird's-eye perspektíva-transzformáció
- sliding-window alapú sávkövetés
- másodfokú polinom sávmodell
- normalizált sávközép- és lookahead hibajel
- PID alapú sávtartó vezérlés
- dinamikus sebességszabályozás
- részleges sávvesztés kezelése
- rövid idejű recovery mód

A következő fejlesztési lépés a komplexebb kanyarkombinációk robusztusabb kezelése, majd a PID vezérlő mérési eredményeinek rögzítése és összehasonlítása egy Pure Pursuit alapú vezérlővel.
