# Wheeltec AutoRace Simulation

ROS 2 Humble és Gazebo Fortress alapú szimulációs környezet Wheeltec / Roboworks robot kameraalapú sávtartásának fejlesztéséhez.

A robot fehér vonalakkal határolt pályán halad, PID vagy Pure Pursuit vezérléssel. A projekt véletlen pályagenerálást és a két vezérlő összehasonlításához használható, független pályakövetési mérést is tartalmaz.

## Fejlesztési környezet

- Ubuntu 22.04 LTS
- ROS 2 Humble
- Gazebo Fortress
- `ros_gz`, OpenCV, `cv_bridge`, `rqt_image_view`
- Python 3, NumPy és Matplotlib a pályageneráláshoz és a kiértékeléshez

## ROS 2 csomagok

### `wheeltec_autorace_description`

A robotmodell, a kamera, a mesh-ek és a pályamodell erőforrásai.

### `wheeltec_autorace_gazebo`

A Gazebo világfájljai.

### `wheeltec_autorace_bringup`

A szimulációt, a ROS–Gazebo bridge-eket, a sávfelismerést, a kiválasztott vezérlőt és az `rqt_image_view` alkalmazást indító launch fájlok.

### `wheeltec_autorace_application`

A sávfelismerés és a vezérlés node-jai:

- `lane_perception`
- `pid_lane_controller`
- `pure_pursuit_controller`

A beállítások a csomag `config/` mappájában, az azonos nevű YAML-fájlokban találhatók.

### `wheeltec_autorace_evaluation`

A pálya referencia-középvonalának exportálása, a robot tényleges mozgásának mérése és az eredmények ábrázolása.

## Lane perception pipeline

1. A Gazebo kameraképe a bridge-en keresztül a `/camera/image_raw` topicra érkezik.
2. A `lane_perception` HSV-színtérben kiemeli a fehér sávjelöléseket, majd szűrés és perspektívatranszformáció után felülnézeti, azaz bird's-eye képet készít.
3. Hisztogrammal kezdőpontokat keres, sliding window ablakokkal követi a sávhatárokat, majd másodfokú görbéket illeszt rájuk. Egy hiányzó sávhatárt a korábban becsült sávszélesség alapján is képes pótolni.
4. A PID a közeli és előretekintő sávközéppontból képzett, normalizált képtérbeli hibát használja. A Pure Pursuit a kamera belső paraméterei és elhelyezése alapján talajsíkra vetített, méterben megadott célpontból számít görbületet.
5. A kiválasztott vezérlő a `/cmd_vel` topicra küldi a sebességparancsot. A `linear.x` az előremeneti sebesség [m/s], az `angular.z` az elfordulási szögsebesség [rad/s]. Ezeket a Gazebo Ackermann-kormányzási rendszere hajtja végre.

A kanyar-előrejelzés külön ágon, az eredeti kameraképen dolgozik: fehér maszk, Canny-éldetektálás, kijelölt képrészlet és Hough-vonalkeresés alapján állítja elő a `turn_preview` pontszámot. Ezt mindkét vezérlő a kanyar előtti lassításhoz használja. A pontszám nem a kanyar méterben mért távolsága.

A bird's-eye debug képen a kiválasztott vezérlőhöz tartozó pontok és adatok, valamint a parancsolt sebesség jelennek meg.

## Generált tesztpálya

A `tools/generate_multicorner_track.py` egyenesekből és lekerekített kanyarokból álló, zárt pályát készít. A véletlen generálásnál megadható a pályaméret és a kanyarok számának tartománya.

A `random_simulation.launch.py` indításkor új pályát generál, frissíti a pályamodellt és a világot, majd exportálja a hozzá tartozó referenciát. A szimuláció sikeres generálás és export után indul.

Az aktuális fájlok alapértelmezett helye:

| Fájl | Tartalom |
|---|---|
| `~/wheeltec_benchmark/track_metadata.json` | A generált pálya adatai |
| `~/wheeltec_benchmark/reference.json` | A méréshez használt referencia |
| `~/wheeltec_benchmark/track_debug.png` | A pálya ellenőrző képe |

Ezek a következő sikeres generáláskor felülíródnak. Azonos pályán végzett összehasonlításhoz a további futásokat a `simulation.launch.py` fájllal indítsd.

## Build

A példákban a projekt helye `~/wheeltec_pp_clean_ws/src/wheeltec_autorace_sim`. Telepített ROS 2 Humble és Gazebo Fortress mellett, új terminálban:

```bash
source /opt/ros/humble/setup.bash
cd ~/wheeltec_pp_clean_ws
rosdep install --from-paths src --ignore-src -r -y --rosdistro humble
colcon build --symlink-install
source install/setup.bash
```

Minden további terminálban töltsd be a környezetet:

```bash
source /opt/ros/humble/setup.bash
source ~/wheeltec_pp_clean_ws/install/setup.bash
```

## Szimuláció indítása

Az alábbi indítások közül egyszerre egyet használj.

### Meglévő pálya

PID vezérlővel:

```bash
ros2 launch wheeltec_autorace_bringup simulation.launch.py controller:=pid
```

Vagy Pure Pursuit vezérlővel:

```bash
ros2 launch wheeltec_autorace_bringup simulation.launch.py controller:=pure_pursuit
```

A `controller` alapértéke `pure_pursuit`. A vezérlő node elindul, de a robot mozgását külön kell engedélyezni.

### Új véletlen pálya

```bash
ros2 launch wheeltec_autorace_bringup random_simulation.launch.py \
  controller:=pid \
  min_track_size:=10.0 max_track_size:=14.0 \
  min_turns:=8 max_turns:=12 variation:=0.65
```

Itt is választható a `controller:=pure_pursuit`. Más workspace használatakor a `generator_script` argumentummal add meg a `tools/generate_multicorner_track.py` teljes elérési útját.

### Vezérlő nélkül

```bash
ros2 launch wheeltec_autorace_bringup simulation.launch.py controller:=none
```

A szimuláció, a bridge-ek és a sávfelismerés így is elindulnak.

## Vezérlők engedélyezése

A robot nyugalomba kerülése után, másik terminálban engedélyezd a launch-nál kiválasztott vezérlőt.

### PID

```bash
ros2 service call /pid_lane_controller/enable std_srvs/srv/SetBool '{data: true}'
```

### Pure Pursuit

```bash
ros2 service call /pure_pursuit_controller/enable std_srvs/srv/SetBool '{data: true}'
```

Megállításhoz ugyanazt a service-t hívd `'{data: false}'` értékkel. A letiltás nullás sebességparancsot küld. Ezután a launch `Ctrl+C`-vel zárható be.

## Hasznos topicok

| Topic | Tartalom |
|---|---|
| `/camera/image_raw` | Eredeti kamerakép |
| `/lane_detection/birdseye_debug` | Sávhatárok, keresőablakok és a vezérlő adatai |
| `/lane_detection/turn_preview_debug` | A kanyar-előrejelzés képe |
| `/lane_detection/control_error_normalized` | A PID normalizált hibajele |
| `/lane_detection/target` | A Pure Pursuit célpontja |
| `/lane_detection/target_status` | A célpont becslésének állapota |
| `/lane_detection/turn_preview` | Kanyar-előrejelzési pontszám |
| `/control/pid/status` | A PID állapota |
| `/control/pure_pursuit/status` | A Pure Pursuit állapota |
| `/cmd_vel` | Parancsolt sebesség és elfordulási szögsebesség |

A képtopicok az automatikusan megnyíló `rqt_image_view` ablakban választhatók ki. Szöveges ellenőrzés például:

```bash
ros2 topic echo /control/pid/status
```

## Pályakövetési mérés

A mérés a Gazebo tényleges robotpózát hasonlítja a pálya referencia-középvonalához, a vezérlők saját hibajelétől függetlenül. Többek között méterben mért oldalhibát, irányhibát, átlagsebességet és megtett utat számít.

Futó szimuláció mellett, a vezérlő engedélyezése előtt indítsd el külön terminálban:

```bash
ros2 launch wheeltec_autorace_evaluation evaluate.launch.py \
  reference:=$HOME/wheeltec_benchmark/reference.json \
  label:=pid_trial01 duration:=60.0
```

A referencia az aktuális pályához tartozzon. A random launch ezt automatikusan elkészíti. Pure Pursuit mérésénél adj másik címkét, például `label:=pp_trial01`.

Másik terminálban élesítsd a mérést:

```bash
ros2 service call /evaluation/start std_srvs/srv/Trigger '{}'
```

Sikeres válasz után engedélyezd a vezérlőt. Az `ARMED` állapotból a robot mért elindulásakor vált `RECORDING` állapotba. A fenti beállítással 60 szimulációs másodperc után lezárja a mérést; a robot ettől tovább halad.

Korábbi lezárás:

```bash
ros2 service call /evaluation/stop std_srvs/srv/Trigger '{}'
```

Az eredmények a `~/wheeltec_results/<időpont>_<label>/` mappába kerülnek: `trajectory.csv`, `summary.json` és a méréshez használt `reference.json` másolata. A pontos mappa az `/evaluation/status` üzenet `output` mezőjében is látható.

Az elkészült mérésből ábra készíthető; az alábbi útvonalat cseréld a mérési mappára:

```bash
ros2 run wheeltec_autorace_evaluation plot_tracking \
  ~/wheeltec_results/IDE_A_MERES_MAPPAJA
```

A parancs ugyanoda menti a `tracking.png` fájlt.

A külső modellek és összetevők forrásai: [THIRD_PARTY.md](THIRD_PARTY.md).
