# ANRO - Labolatoria -  Sterowanie Robotem Dobot Magician

## Autorzy
* **Mikołaj Wróbel**
* **Kacper Maciejko**

## Opis projektu
Projekt zawiera pakiety dla systemu ROS 2 (dystrybucja **Jazzy**) służące do modelowania, wizualizacji oraz sterowania ramieniem robotycznym Dobot Magician oraz symulatorem Turtlesim. Projekt implementuje algorytmy kinematyki prostą i odwrotną, a także sekwencje automatycznego manipulowania obiektami.

## Struktura pakietów

### 1. dobot_magician - folder zawierający pracę z 3. i 4. Labolatoriów
Symulacja robota:
* **Modelowanie**: Wykorzystanie plików Xacro do generowania struktury URDF robota, uwzględniającej wymiary fizyczne z pliku `params.yaml`.
* **Kinematyka Prosta (`forwardkin`)**: Przelicza kąty złączy na pozycję (x, y, z) efektora końcowego, publikując dane na temat `/end_effector_pose`.
* **Kinematyka Odwrotna (`inversekin`)**: Oblicza wymagane kąty złączy na podstawie kliknięcia punktu w przestrzeni (`/clicked_point`), biorąc pod uwagę limity fizyczne robota.
* **Konfiguracja**: Plik `params.yaml` definiuje wymiary ramion (rear arm: 0.135m, fore arm: 0.147m) oraz limity ruchu dla poszczególnych jointów.

### 2. lab2
Skrypty operacyjne i sterujące robotem:
* **`dobot_tower`**: Skrypt automatyzujący budowę wieży z klocków o zadanej wysokości, wykorzystujący akcje `PTP_action` oraz serwis chwytaka.
* **`dobot_move`**: Podstawowa sekwencja pick-and-place.
* **`turtle_controller`**: Zaawansowany kontroler dla Turtlesim, realizujący sekwencję obrotów i tworzenia nowych żółwi (`spawn`).
* **`dobot_print_position`**: Narzędzie diagnostyczne monitorujące pozycję robota na topiku `/dobot_pose`.

## Wymagania i instalacja

### Wymagania
* **ROS 2 Jazzy**
* Zainstalowane pakiety: `rclpy`, `std_msgs`, `geometry_msgs`, `sensor_msgs`, `turtlesim` oraz customowe wiadomości `dobot_msgs`.

### Budowanie projektu
```bash
cd ~/ros2_ws
colcon build --packages-select dobot_magician lab2
source install/setup.bash
```

## Instrukcja uruchamiania

### Wizualizacja robota i kinematyka
Uruchomienie modelu w RViz2 wraz z węzłami kinematyki:
```bash
ros2 launch dobot_magician display.launch.py
```
Możesz sterować robotem poprzez publikowanie punktów docelowych lub zmianę stanów złączy.

### Budowanie wieży (Dobot)
Uruchomienie automatycznej sekwencji układania wieży:
```bash
ros2 launch lab2 dobot_launch.py height:=3
```
*Parametr `height` określa liczbę klocków w wieży.*

### Kontrola Turtlesim
Uruchomienie symulatora i kontrolera sekwencyjnego:
```bash
ros2 launch lab2 turtles_launch.py t2_name:="zolw_alfa" t3_name:="zolw_beta"
```
*Skrypt obróci żółwia o 180 stopni, stworzy nowego, a następnie wykona kolejne manewry.*

## Licencja
Projekt udostępniany na licencji **Apache-2.0**.
