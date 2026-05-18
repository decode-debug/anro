# ANRO - Sterowanie Robotem Dobot Magician

## Autorzy
- **Mikołaj Wróbel**
- **Kacper Maciejko**

## Opis projektu
Repozytorium zawiera pakiety ROS 2 dla dystrybucji **Jazzy** związane z modelowaniem, wizualizacją i sterowaniem robotem Dobot Magician oraz z zadaniami wykonywanymi w Turtlesim. W projekcie zaimplementowano:
- kinematykę prostą i odwrotną ramienia,
- model URDF/Xacro robota,
- wizualizację w RViz2,
- zadanie z laboratorium 6 z kamerą, markerami i automatycznym pick-and-place,
- skrypty sterujące dla pakietu `lab2`.

## Struktura pakietów

### `dobot_magician`
Pakiet odpowiedzialny za model robota i wizualizację.

Najważniejsze elementy:
- `urdf/dobot.urdf.xacro` - podstawowy model robota.
- `urdf/dobot_with_camera.urdf.xacro` - model rozszerzony o kamerę Intel RealSense D435i.
- `config/params.yaml` - wymiary członów robota i limity złączy.
- `forwardkin` - publikuje pozycję efektora końcowego na `/end_effector_pose`.
- `inversekin` - przelicza cele w przestrzeni na zmienne złączowe i publikuje `/joint_states`.
- `marker_publisher` - publikuje markery kostki i kartki do RViz2.
- `random_scene_publisher` - losuje pozycje kostki i kartki względem `camera_link`.
- `grasp_controller` - transformuje pozycje z kamery do `base_link`, planuje chwyt i odkładanie kostki.
- `launch/display.launch.py` - tryb podglądu modelu i ręcznego sterowania.
- `launch/lab6.launch.py` - pełny scenariusz laboratorium 6.

### `lab2`
Pakiet ze skryptami do wcześniejszych laboratoriów.

Najważniejsze elementy:
- `dobot_move` - podstawowa sekwencja pick-and-place.
- `dobot_tower` - automatyczne budowanie wieży z klocków.
- `dobot_print_position` - diagnostyka pozycji robota.
- `turtle_controller` - sterowanie Turtlesim.
- `launch/dobot_launch.py` - uruchamianie sekwencji dla Dobota.
- `launch/turtles_launch.py` - uruchamianie Turtlesim i kontrolera.

## Wymagania
- ROS 2 Jazzy
- `rclpy`
- `geometry_msgs`
- `sensor_msgs`
- `std_msgs`
- `visualization_msgs`
- `tf2_ros`
- `robot_state_publisher`
- `joint_state_publisher_gui`
- `rviz2`
- `xacro`
- `turtlesim`

## Budowanie workspace
Projekt należy budować z katalogu workspace, czyli z folderu `anro`.

```bash
cd /home/mikolajwrobel/anro_lab_6_github/anro
source /opt/ros/jazzy/setup.bash
colcon build --packages-select dobot_magician lab2
source install/setup.bash
```

Jeśli chcesz zbudować tylko pakiet z modelem robota:

```bash
cd /home/mikolajwrobel/anro_lab_6_github/anro
source /opt/ros/jazzy/setup.bash
colcon build --packages-select dobot_magician
source install/setup.bash
```

Nie uruchamiaj `colcon build` wewnątrz katalogu `anro/dobot_magician`, bo utworzone lokalnie katalogi `build/`, `install/` i `log/` mogą psuć testy `ament_flake8`.

## Uruchamianie

### Podgląd modelu i sterowanie punktami z RViz2
Tryb domyślny uruchamia RViz2, `robot_state_publisher`, `forwardkin`, `marker_publisher` oraz `inversekin`. Klikanie narzędziem `PublishPoint` publikuje cele na `/clicked_point`.

```bash
ros2 launch dobot_magician display.launch.py
```

### Podgląd modelu z `joint_state_publisher_gui`
Jeśli chcesz poruszać robotem ręcznie suwakami, uruchom tryb GUI bez `inversekin`:

```bash
ros2 launch dobot_magician display.launch.py use_gui:=true use_inversekin:=false
```

### Laboratorium 6 - kamera, markery i pick-and-place
Pełny scenariusz uruchamia model robota z kamerą, losowanie obiektów w `camera_link`, transformację do `base_link` i sekwencję przenoszenia kostki na środek kartki.

```bash
ros2 launch dobot_magician lab6.launch.py
```

W tym scenariuszu:
- kamera Intel RealSense D435i jest wymodelowana jako prostopadłościan `90 x 25 x 25 mm`,
- kostka ma wymiar `2 cm`,
- kartka ma wymiar `5 x 10 cm` i grubość `1 mm`,
- pozycje kostki i kartki są publikowane wyłącznie względem `camera_link`,
- `grasp_controller` transformuje je do `base_link` i wysyła cele do `inversekin`.

### `lab2` - automatyka Dobota

```bash
ros2 launch lab2 dobot_launch.py height:=3
```

Parametr `height` określa liczbę klocków w wieży.

### `lab2` - Turtlesim

```bash
ros2 launch lab2 turtles_launch.py t2_name:='zolw_alfa' t3_name:='zolw_beta'
```

## Testy
Sprawdzenie pakietu `dobot_magician`:

```bash
cd /home/mikolajwrobel/anro_lab_6_github/anro
source /opt/ros/jazzy/setup.bash
colcon test --packages-select dobot_magician
colcon test-result --verbose --all
```

## Licencja
Projekt jest udostępniany na licencji **Apache-2.0**.