# Evaluation – telepítés és futtatás

## 1. Telepítés – egyszer

A `wheeltec_autorace_evaluation` mappát ide másold:

```text
/home/kopaj/wheeltec_pp_clean_ws/src/wheeltec_autorace_sim/wheeltec_autorace_evaluation/
```

```bash
source /opt/ros/humble/setup.bash
sudo apt install libignition-transport11-dev libignition-msgs8-dev python3-numpy python3-opencv python3-matplotlib ros-humble-ament-cmake-python
cd ~/wheeltec_pp_clean_ws
colcon build --symlink-install --packages-select wheeltec_autorace_evaluation
```

**Minden új terminálban:**

```bash
source /opt/ros/humble/setup.bash
source ~/wheeltec_pp_clean_ws/install/setup.bash
```

## 2. Pályareferencia – pályánként egyszer

Ha már megvan a megfelelő `~/wheeltec_benchmark/reference.json`, ugorj a 3. pontra.

**Csak új pálya készítéséhez:** a futó szimulációt állítsd le, majd indítsd ezt.
Ez lecseréli a jelenlegi pályát. Ha már megvan a jelenlegi pályához tartozó metadata, ezt hagyd ki.

```bash
ros2 launch wheeltec_autorace_bringup random_simulation.launch.py \
  controller:=pid variation:=0.65 \
  generator_script:=$HOME/wheeltec_pp_clean_ws/src/wheeltec_autorace_sim/tools/generate_multicorner_track.py
```

A szimuláció elindulása után **másik terminálban** mentsd a metadata fájlt, majd exportáld a referenciát:

```bash
mkdir -p ~/wheeltec_benchmark
cp /tmp/wheeltec_random_track.json ~/wheeltec_benchmark/track_metadata.json

ros2 run wheeltec_autorace_evaluation export_reference \
  --metadata ~/wheeltec_benchmark/track_metadata.json \
  --generator ~/wheeltec_pp_clean_ws/src/wheeltec_autorace_sim/tools/generate_multicorner_track.py \
  --variation 0.65 \
  --output ~/wheeltec_benchmark/reference.json
```

A `--variation` egyezzen a pályageneráláskor használttal. Meglévő `reference.json` fájlt az export nem ír felül; új pályához adj új fájlnevet, és azt használd a mérésnél is.

## 3. PID mérés

**1. terminál – szimuláció:** ha a 2. pontból már fut, ne indíts másodikat. Új próbához ugyanarról a kezdőpózról indítsd:

```bash
ros2 launch wheeltec_autorace_bringup simulation.launch.py controller:=pid
```

**2. terminál – mérő:**

```bash
ros2 launch wheeltec_autorace_evaluation evaluate.launch.py \
  reference:=$HOME/wheeltec_benchmark/reference.json \
  label:=pid_trial01 duration:=60.0
```

**3. terminál – ellenőrzés, mérés élesítése, vezérlő indítása:**

```bash
ros2 topic echo /evaluation/ground_truth_pose --once
ros2 service call /evaluation/start std_srvs/srv/Trigger '{}'
# Csak a success: true válasz után:
ros2 service call /pid_lane_controller/enable std_srvs/srv/SetBool '{data: true}'
```

A mérés az elinduláskor kezdődik, és 60 szimulációs másodperc után automatikusan ment. A robot ettől nem áll meg.

**Állapot, kézi mentés és robotmegállítás:**

```bash
ros2 topic echo /evaluation/status --once
# Kézi lezárás, ha a mérés még fut:
ros2 service call /evaluation/stop std_srvs/srv/Trigger '{}'
ros2 service call /pid_lane_controller/enable std_srvs/srv/SetBool '{data: false}'
```

Csak kézzel lezárandó méréshez: `duration:=0.0`.

## 4. Pure Pursuit mérés – ugyanazon a pályán

Állítsd le az előző szimulációt és mérő launchot. Új terminálokban, ugyanabban a sorrendben:

```bash
# 1. terminál
ros2 launch wheeltec_autorace_bringup simulation.launch.py controller:=pure_pursuit

# 2. terminál
ros2 launch wheeltec_autorace_evaluation evaluate.launch.py \
  reference:=$HOME/wheeltec_benchmark/reference.json \
  label:=pp_trial01 duration:=60.0

# 3. terminál
ros2 topic echo /evaluation/ground_truth_pose --once
ros2 service call /evaluation/start std_srvs/srv/Trigger '{}'
# Csak a success: true válasz után:
ros2 service call /pure_pursuit_controller/enable std_srvs/srv/SetBool '{data: true}'

# A próba végén a robot megállítása:
ros2 service call /pure_pursuit_controller/enable std_srvs/srv/SetBool '{data: false}'
```

## 5. Generált fájlok és ábra

| Fájl | Hely |
|---|---|
| Mentett pályaadatok | `~/wheeltec_benchmark/track_metadata.json` |
| Pályareferencia | `~/wheeltec_benchmark/reference.json` |
| Mérési összegzés | `~/wheeltec_results/<időpont>_<label>/summary.json` |
| Nyers mérési adatok | `~/wheeltec_results/<időpont>_<label>/trajectory.csv` |
| Referencia másolata | `~/wheeltec_results/<időpont>_<label>/reference.json` |
| Ábra, az alábbi parancs után | `~/wheeltec_results/<időpont>_<label>/tracking.png` |

Az eredménymappa pontos útvonalát a mérő kiírja; az `/evaluation/status` üzenet `output` mezőjében is szerepel.

```bash
# A helyőrzőt cseréld a konkrét mérési mappa nevére:
ros2 run wheeltec_autorace_evaluation plot_tracking \
  ~/wheeltec_results/IDE_A_MERES_MAPPAJA
```
