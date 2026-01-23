# Pi4_ros2_ws

Workspace **ROS 2 Humble** pour piloter et superviser la voiture depuis un **Raspberry Pi 4** (Ubuntu 22.04).
Il regroupe les nœuds nécessaires pour :
- convertir des commandes (joystick / commandes distantes) en ordres moteurs,
- échanger avec l’électronique embarquée via **SocketCAN** (`can0`),
- publier la télémétrie capteurs et appliquer des sécurités en temps réel.

### Fonctionnalités principales
- **Pilotage**
  - Joystick -> commande propulsion + direction
  - Modes de conduite (manuel / autonome selon implémentation)
- **Communication**
  - CAN via `can0` (TX/RX des trames vers l’électronique)
- **Sécurité**
  - Anti-collision ultrason (ralentissement puis arrêt)
  - Watchdog de commande (fail-safe si perte de données)
- **Capteurs / Télémétrie**
  - Ultrasons, IMU, GNSS, batterie, retours moteurs, etc.
- **Supervision**
  - Logging (HTTP) + commandes distantes (WebSocket)
  - Envoi d’images caméra (upload)



## Arborescence

- `src/` : paquets ROS 2
  - `can/` : RX/TX CAN (SocketCAN)
  - `car_control/` : génération des commandes moteurs (à partir joystick / mode / logique)
  - `safety_stop/` : filtre sécurité ultrasons + watchdog commande
  - `interfaces/` : messages ROS personnalisés (`.msg`)
  - `online_logger/` : logs web + WebSocket + upload images
  - `system_check/` : check communications + report
  - `watchdog/` : surveillance (selon implémentation)
  - `simulation/` : nœud(s) de simulation (selon implémentation)
  - `geicar_start/` : launch principal
  - `sllidar_ros2/`, `imu_tools/`, `audio_common/`, `carla_msgs/`, `joystick/` : dépendances/paquets tiers vendorisés



## Démarrage rapide

### 1) Dépendances
- ROS 2 installé et sourcé (`/opt/ros/$ROS_DISTRO`)
- Outils build : `colcon`, `rosdep`
- Python deps (pour `online_logger`) :
  - `requests`
  - `websockets`

### 2) Build
```bash
cd Pi4_ros2_ws
source /opt/ros/humble/setup.bash

rosdep update
rosdep install --from-paths src -i -y

colcon build --symlink-install
source install/setup.bash
```

### 3) Lancer le système
Le launch principal est dans `geicar_start` :
```bash
ros2 launch geicar_start geicar.launch.py
```


## Pipeline fonctionnel

```
(joystick) --> car_control_node --> /motors_order_raw --> safety_stop_node --> /motors_order --> can_tx_node --> CAN
```

Dans le launch `geicar.launch.py`, `car_control_node` remappe `motors_order` vers `motors_order_raw` :
- `car_control_node` publie “motors_order”
- le remap l’envoie sur `motors_order_raw`
- `safety_stop_node` s’abonne à `motors_order_raw` et republie une commande sûre sur `motors_order`
- `can_tx_node` s’abonne à `motors_order` et l’envoie sur le bus CAN



## Paquets clés

### `can`
#### `can_rx_node`
- Pub :
  - `us_data` (`interfaces/Ultrasonic`)
  - `imu/data_raw` (`sensor_msgs/Imu`)
  - `imu/mag` (`sensor_msgs/MagneticField`)
  - `gnss_data` (`interfaces/Gnss`)
  - `motors_feedback` (`interfaces/MotorsFeedback`)
  - `general_data` (`interfaces/GeneralData`)
  - `steering_calibration` (`interfaces/SteeringCalibration`)
  - `system_check` (`interfaces/SystemCheck`)

#### `can_tx_node`
- Sub :
  - `motors_order` (`interfaces/MotorsOrder`)
  - `steering_calibration` (`interfaces/SteeringCalibration`)
  - `system_check` (`interfaces/SystemCheck`)

> Convention PWM : `50` = neutre (stop).


### `safety_stop`
#### `safety_stop_node`
Rôle : **sécuriser** la commande moteur en fonction des ultrasons + watchdog.
- Sub:
  - `us_data` (`interfaces/Ultrasonic`) (QoS sensor)
  - `motors_order_raw` (`interfaces/MotorsOrder`)
- Pub:
  - `motors_order` (`interfaces/MotorsOrder`) (commande filtrée)
  - `/logger` (`interfaces/LogEntry`) (évènements stop/timeout)

Fonctions principales :
- **Fail-safe** : si les ultrasons sont trop vieux → STOP immédiat (PWM=50, sans rampe)
- **Ralentissement linéaire** entre `slow_dist_*` et `stop_dist_*`
- **STOP dur** si obstacle trop proche
- **Watchdog commande** : si plus de commande reçue → STOP

Paramètres (avec valeurs du launch `geicar.launch.py`) :
- `stop_dist_front_cm` / `stop_dist_rear_cm`
- `slow_dist_front_cm` / `slow_dist_rear_cm`
- `us_timeout_ms`
- `cmd_timeout_ms`
- `log_actions`
- `max_delta_pwm_up_per_msg`, `max_delta_pwm_down_per_msg` (rampe)



### `interfaces` (messages personnalisés)
Les `.msg` sont dans `src/interfaces/msg/`.

Résumé des principaux messages :

- `GeneralData.msg`
  - `float32 battery_level`
  - `float32 temperature`
  - `int16 pressure`
  - `int8 humidity`

- `Gnss.msg`
  - `float64 latitude, longitude, altitude` (défaut `-1`)
  - `int8 quality` (0=no fix, 1=GNSS, 2=DGPS, 4=RTK fixed, 5=RTK float, 6=dead reckoning)
  - `float64 hacc, vacc` (mm)

- `MotorsOrder.msg`
  - `int8 left_rear_pwm` (0..100, défaut 50)
  - `int8 right_rear_pwm` (0..100, défaut 50)
  - `int8 steering_angle` ([-128..127])

- `MotorsFeedback.msg`
  - `int8 left_rear_odometry, right_rear_odometry` (pulses, 36 = 1 tour)
  - `float32 left_rear_speed, right_rear_speed` (RPM)

- `Ultrasonic.msg`
  - `int16 front_left, front_center, front_right`
  - `int16 rear_left, rear_center, rear_right`

- `LogEntry.msg`
  - niveaux : `ERROR/WARN/TRACE/DEBUG/MS/DASHBOARD`
  - `uint8 level`
  - `string sender`
  - `string message`

- `SystemCheck.msg`
  - flags `request/response/report/print`
  - booléens `jetson/l476/f103`
  - report texte : `comm_jetson`, `comm_l476`, `comm_f103`, `battery`, `ultrasonics[6]`, `gps`, `imu`, `lidar`, `camera`



### `online_logger`
- `LoggerSubscriber` : s’abonne à `/logger` et envoie les logs sur un serveur HTTP.
- `ImageSender` :
  - Sub : `/image_raw/compressed` (`sensor_msgs/CompressedImage`)
  - POST périodique vers serveur (`min_interval_sec`)
  - Publie un statut sur `/image_sender/status`
  - Lit un token secret dans `/home/pi/.bin/sek` (à créer sur la cible)
- `SocketListener` :
  - Client WebSocket (ex: `wss://magictintin.fr/ws`)
  - Gère des commandes distantes (ex: `gotoX|Y|Yaw`, `horn`, `tts>...`, `stopgoto`, confirm/deny)
  - Publie :
    - `/car_arrived_to_fall` (Bool)
    - `/image_verified` (Bool)
    - `/someone_fell` (Bool)
  - Intègre Nav2 (`/navigate_to_pose`) + audio (`/music_play`, action `/say`)
