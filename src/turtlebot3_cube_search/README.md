# Autonomiczne poszukiwanie pięciu kostek ArUco

## Zakres i założenia

Projekt zawiera trzy node'y Python/rclpy oraz paczkę interfejsów. Robot zna
statyczną mapę i jest już zlokalizowany. Nawigacja należy do Nav2, pomiary kostek
pochodzą wyłącznie z kamery RGB. Aplikacja nie subskrybuje lidaru.

Jedno ID oznacza jedną nieruchomą kostkę. Pięć różnych ID kończy zadanie.
Na scenie nie powinno być innych znaczników z tego słownika; jeśli ID są znane,
ustaw `allowed_ids` w detektorze. Kilka ścian tej samej kostki z tym samym ID
wymaga znanej geometrii każdej ściany; obecny pojedynczy offset tego nie rozwiązuje.

**Domyślna pozycja kostki to środek markera**, w metrach, XYZ w `map`.
Parametr `marker_to_cube_offset` pozwala wskazać środek kostki w osiach markera.
Przykład: marker leży na powierzchni sześcianu o boku 0.10 m, a jego +Z wskazuje
na zewnątrz: offset `[0.0, 0.0, -0.05]`. Rozmiar markera nie musi być bokiem kostki.

Zasięg lidaru 3.5 m należy skonfigurować w istniejącym systemie nawigacyjnym,
zgodnie z parametrami sensora (zasięg raytracingu nie może go przekraczać).
Nie jest to ograniczenie zasięgu kamery ani podstawa pozycjonowania markerów.
Kostki w symulacji powinny mieć geometrię wizualną bez geometrii kolizyjnej
widocznej dla lidaru, zgodnie z założeniem zadania.

## Środowisko i zależności

Podczas analizy host miał Ubuntu 26.04.1, Python 3.14, bez aktywnego ROS i colcon.
Katalog `~/ros2_kilted/install` zawierał brakujące pliki instalacji i nie stanowił
działającego underlay. W lokalnym obrazie `arm/lab05` potwierdzono:

- Ubuntu 24.04, ROS 2 Jazzy, Python 3.12.3;
- Nav2 (`nav2_msgs` 1.3.13), TF2, cv_bridge 4.1.0, colcon i pytest;
- OpenCV 4.6.0 z `cv2.aruco`, NumPy;
- pakiety `ros_gz` i biblioteki Gazebo; nie potwierdzono uruchomionej symulacji;
- brak zarejestrowanych paczek `turtlebot3_*` w tym obrazie.

Docelowo wymagane są: `rclpy`, `nav2_msgs`, `tf2_ros_py`, `tf2_geometry_msgs`,
`cv_bridge`, `sensor_msgs`, `nav_msgs`, `geometry_msgs`, `action_msgs`, `std_srvs`,
OpenCV z modułem ArUco i NumPy, a do budowania `ament_cmake`, `ament_python`,
`rosidl_default_generators`, colcon. Nav2, sterowniki/kamera i pakiety TurtleBot3
muszą być zainstalowane oraz skonfigurowane w środowisku robota.

W działającym, wspieranym środowisku ROS można zainstalować zależności manifestów:

```bash
source /opt/ros/<dystrybucja>/setup.bash
rosdep install --from-paths src --ignore-src -r -y
```

Nie instaluj pakietów Jazzy dla Ubuntu 24.04 bezpośrednio na hoście 26.04.
Weryfikowana jest zgodność z Jazzy; inne dystrybucje wymagają osobnego budowania
i testów, szczególnie definicji akcji Nav2. Detektor obsługuje starsze
`aruco.detectMarkers` oraz nowsze `aruco.ArucoDetector`.

Alternatywa: [`aruco_opencv`](https://index.ros.org/p/aruco_opencv/) wykorzystuje
OpenCV i może zastąpić detektor po sprawdzeniu dystrybucji, słownika, rozmiaru,
znaczników czasu i układu optycznego. Potrzebny będzie adapter do
`ArucoDetectionArray`; nie są to zamienne interfejsy. `aruco_markers` również
wymaga sprawdzenia konkretnej implementacji i formatu wiadomości.
Nie dodano tych alternatyw jako zależności obowiązkowych.

## Budowanie i testy

Z katalogu głównego workspace:

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select cube_search_interfaces turtlebot3_cube_search
source install/setup.bash
colcon test --packages-select cube_search_interfaces turtlebot3_cube_search --event-handlers console_direct+
colcon test-result --verbose
```

W lokalnym obrazie Docker, bez instalacji na hoście, można wykonać:

```bash
docker run --rm --network none \
  --user "$(id -u):$(id -g)" -e HOME=/tmp \
  -v "$PWD:/ws" -w /ws --entrypoint /bin/bash arm/lab05 -c '
    source /opt/ros/jazzy/setup.bash
    colcon build --symlink-install --packages-select cube_search_interfaces turtlebot3_cube_search &&
    source install/setup.bash &&
    colcon test --packages-select cube_search_interfaces turtlebot3_cube_search --event-handlers console_direct+ &&
    colcon test-result --verbose
  '
```

Testy obejmują syntetyczne obrazy ArUco i metryczną estymację pozy, filtrację
pozycji, geometrię mapy, obsługę nieznanych komórek, osiągalność, wyścigi anulowania
oraz integrację ROS z atrapami akcji Nav2. Dodatkowy test uruchamia trzy procesy
przez launch i sprawdza ich zamknięcie po SIGINT. Atrapy nie testują fizyki ani Nav2.
Końcowy przebieg: **48 testów, zero błędów, zero pominiętych**. Testy sprawdzają
również oznaczenie pozy i orientacji układem optycznym, zachowanie timestampu,
zgłoszony pomiar przy podłodze, przekazanie parametru `optical_frame` przez launch
oraz skalę pozycji przy rozmiarze wzoru około 7 cm na kostce 9 cm. Szczegóły
wykonanych sprawdzeń są zapisane w `VALIDATION.md` w workspace.

## Wymagania uruchomieniowe

Najpierw uruchom właściwy dla swojej instalacji bringup TurtleBot3 Waffle,
kamerę, map server, lokalizację (np. AMCL) oraz Nav2. Ta paczka nie dostarcza świata,
modelu Waffle, mapy ani konfiguracji AMCL. Wybór launchy Gazebo wymaga dopasowania
do faktycznie zainstalowanej wersji pakietów TurtleBot3 i symulatora.

Sprawdź:

```bash
ros2 topic list
ros2 action list -t
ros2 topic echo /camera/camera_info --once
ros2 run tf2_ros tf2_echo map base_footprint
ros2 run tf2_ros tf2_echo base_footprint camera_rgb_optical_frame
```

Ostatnia nazwa musi odpowiadać parametrowi detektora `optical_frame`, domyślnie
`camera_rgb_optical_frame`. Wymagany łańcuch:
`map -> odom -> base_footprint -> ... -> camera_rgb_optical_frame`.
Oś optyczna kamery: X w prawo, Y w dół, Z w przód. `frame_id` obrazu lub
CameraInfo może wskazywać mechaniczny `camera_rgb_frame` i nie musi być nazwą
układu optycznego. Nie wybieraj go jako `optical_frame`.

### Układ optyczny i poprawka lokalizacji

OpenCV `solvePnP` zwraca pozycję **i orientację** markera w osiach optycznych.
Detektor publikuje te wartości bez dodatkowego obrotu, z
`header.frame_id=optical_frame` i oryginalnym czasem obrazu. Nagłówek wejściowego
obrazu nie jest modyfikowany. Obraz i CameraInfo nadal muszą mieć zgodne
wejściowe `frame_id` oraz rozmiar, aby nie mieszać różnych kamer.

W badanym środowisku Jazzy/Harmonic obraz i CameraInfo są oznaczone
`camera_rgb_frame`. Kopiowanie tego oznaczenia do pozy OpenCV było błędem:
TF do tej ramki ma rotację jednostkową, więc głębokość optyczna stawała się
wysokością w układzie robota. Dla pomiaru `[-0.23184, 0.10913, 2.77726]` i
translacji kamery `[0.069, -0.047, 0.117]` otrzymywano błędnie
`[-0.16284, 0.06213, 2.89426]`.

Istniejący TF do `camera_rgb_optical_frame`, z kwaternionem
`[-0.5, 0.5, -0.5, 0.5]`, zmienia osie według
`[x, y, z]_optical -> [z, -x, -y]_base`. Poprawna pozycja tego pomiaru w
`base_footprint` wynosi `[2.84626, 0.18484, 0.00787]` m. Pozycja w `map`
uwzględnia jeszcze położenie i orientację robota w chwili wykonania zdjęcia.

`cube_localizer` korzysta z istniejącego TF2 dla czasu obrazu; nie dodaje obrotu
optycznego ręcznie. **Nie dodawaj kolejnej statycznej transformacji**:
potrzebny TF już dostarcza model robota. Przy innej kamerze ustaw jej istniejący
układ optyczny przez parametr/argument `optical_frame`.

Po wdrożeniu poprawki anuluj aktywne zadanie i uruchom ponownie lokalizator albo
wywołaj `/cube_localizer/reset` podczas bezczynności. Wcześniej potwierdzone,
błędne pozycje są przechowywane w pamięci i nie zostaną automatycznie usunięte.

Wymagane aktywne akcje Nav2:

- `/compute_path_to_pose` (`nav2_msgs/action/ComputePathToPose`);
- `/navigate_to_pose` (`nav2_msgs/action/NavigateToPose`);
- `/spin` (`nav2_msgs/action/Spin`), z Behavior Server.

W Jazzy konfiguracja Behavior Server powinna zawierać plugin Spin:

```yaml
behavior_server:
  ros__parameters:
    behavior_plugins: [spin, backup, drive_on_heading, wait]
    spin:
      plugin: nav2_behaviors::Spin
```

To fragment istniejącej konfiguracji Nav2, nie kompletny plik bringup.
Zachowaj definicje pozostałych pluginów i konfigurację costmap, TF oraz prędkości.
Zobacz [dokumentację Behavior Server](https://docs.nav2.org/jazzy/configuration_and_development/configuration_guide/core_servers/configuring_behavior_server/).
Obrót 360° wymaga miejsca na cały obrys robota; Spin może przerwać ruch ze względu
na kolizję i wtedy punkt zostanie pominięty. Przy małej prędkości zwiększ `spin_timeout`.

## Uruchomienie zadania

Zmierz bok czarnego kwadratu markera (bez białego marginesu). W modelach
`arm05_sim` kostka ma bok 9 cm, ale czarny wzór zajmuje 588 z 755 pikseli
tekstury, więc jego bok wynosi około 7 cm. Dla tych modeli:

```bash
source install/setup.bash
ros2 launch turtlebot3_cube_search cube_search.launch.py \
  marker_size:=0.07 optical_frame:=camera_rgb_optical_frame use_sim_time:=true
```

Na fizycznym robocie użyj `use_sim_time:=false`. Cały system musi mieć zgodny czas;
w symulacji wymagany jest `/clock`. Nazwy tematów i akcji można przekazać argumentami
`image_topic`, `camera_info_topic`, `map_topic`, `navigate_action`, `planner_action`,
`spin_action`; nazwy TF argumentami `map_frame`, `base_frame` i `optical_frame`.
`optical_frame` ma domyślnie wartość `camera_rgb_optical_frame` zarówno w node,
YAML, jak i launch. Wybór tej nazwy nie tworzy nowego TF.
Argument launch `marker_size` nadal jest obowiązkowy. W dostarczonym YAML
ustawiono `0.07` dla `arm05_sim`; inne modele wymagają własnego pomiaru.
Sam detektor bez parametrów ma wartość 0 i zgłosi błąd startu.
Po zmianie rozmiaru anuluj aktywne zadanie i zrestartuj lokalizator lub wywołaj
`/cube_localizer/reset` podczas bezczynności, aby usunąć stare potwierdzenia.
Parametry wczytywane są przy starcie; zmiany wymagają ponownego uruchomienia node'ów.

W drugim terminalu z tym samym underlay i overlay:

```bash
ros2 action send_goal /find_cubes cube_search_interfaces/action/FindCubes \
  '{timeout_sec: 600.0}' --feedback
ros2 topic echo /cubes --qos-durability transient_local
```

`timeout_sec: 0.0` wybiera domyślny limit. Limit jest czasem rzeczywistym,
więc obowiązuje również przy zatrzymanym `/clock`; sprzątanie może dodatkowo
trwać do dwóch `stop_timeout`. Jedno zadanie naraz; drugie zostanie odrzucone.
Istniejące potwierdzenia mogą zakończyć kolejne zadanie natychmiast.

Anulowanie z CLI (pusty UUID i zerowy czas oznaczają wszystkie cele tego serwera):

```bash
ros2 service call /find_cubes/_action/cancel_goal action_msgs/srv/CancelGoal \
  '{goal_info: {goal_id: {uuid: [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]}, stamp: {sec: 0, nanosec: 0}}}'
```

Potwierdzone kostki są trwałe w pamięci procesu. Przed zmianą rozmieszczenia
anuluj zadanie i po otrzymaniu jego wyniku wyczyść lokalizator:

```bash
ros2 service call /cube_localizer/reset std_srvs/srv/Trigger '{}'
```

Poczekaj na pustą listę `/cubes` przed kolejną misją. Nie resetuj lokalizatora
podczas zadania. Restart lokalizatora również czyści stan. Zmiana mapy podczas
misji przerywa zadanie; po zmianie układu odniesienia wyczyść obserwacje.

## Działanie i kontrakty

1. `aruco_detector` czyta obraz i kalibrację z QoS sensor data. Wymaga pełnej
   rozdzielczości pinhole, bez ROI/binningu, zgodnego rozmiaru i wejściowego `frame_id`.
   Modele `plumb_bob` i `rational_polynomial` są obsługiwane; fisheye wymaga
   wcześniejszej rektyfikacji i odpowiedniego CameraInfo. Publikuje pomiary
   z oryginalnym czasem obrazu, pozą w układzie `optical_frame` i błędem reprojekcji.
2. `cube_localizer` transformuje pozę przez TF2 dla czasu pomiaru. Krótko czeka
   na spóźniony TF, odrzuca przeterminowane pomiary, błędne pozy i duplikaty.
   Dla ID szuka największego skupienia w oknie, wyznacza medianę i sprawdza rozrzut,
   liczbę oraz rozpiętość czasową pomiarów. Potwierdzone kostki nie wygasają.
   Cofnięcie zegara czyści lokalne obserwacje. `/cubes` ma QoS reliable,
   transient local, depth 1, także dla pustej listy po resecie.
3. `cube_search_server` powiększa przeszkody o promień robota i zapas; nieznane
   komórki i granice mapy są niedostępne. Flood fill od robota odrzuca oddzielne
   obszary. Dla każdego najbliższego nieodwiedzonego punktu Nav2 sprawdza ścieżkę,
   wykonuje przejazd i Spin o `2π`. Detekcja trwa w czasie jazdy i obrotu.
   Jeśli brak przecięcia regularnej siatki z obszarem, próbuje bieżącej komórki.

`FindCubes` zwraca `CubeArray` (wspólny `header.frame_id=map`), kod i opis.
Sukces ROS Action oznacza dokładnie pięć ID; abort zwraca częściową listę.
Feedback zawiera liczbę ID, liczbę **podjętych prób punktów** (także pominiętych),
liczbę punktów i fazę. Wynikowe kody są stałymi w `FindCubes.action`.

Serwer anuluje wyłącznie własne cele Nav2. Obsługuje anulowanie także przed
potwierdzeniem przyjęcia celu. Czeka na terminalny wynik celu, nie tylko odpowiedź
na prośbę o anulowanie. Jeśli nie może potwierdzić zakończenia, zwraca `STOP_FAILED`
i blokuje nowe misje do restartu. Ten kod **nie potwierdza fizycznego zatrzymania**.
Po awarii komunikacji sprawdź robota i Nav2 przed restartem. Przy prawidłowym
działaniu Nav2 zakończenie jego celu zatrzymuje sterowanie ruchem. Nie uruchamiaj
równocześnie innego klienta wysyłającego cele lub prędkości do tego samego robota.
SIGINT i SIGTERM inicjują to samo sprzątanie; powtórne sygnały nie przerywają go.
Launch daje serwerowi 15 s przed eskalacją do SIGTERM. Jeśli zwiększasz
`stop_timeout` powyżej domyślnych 5 s, dopasuj także limity zamykania w launch.

## Dobór parametrów i ograniczenia

- `grid_spacing` dobierz do FOV, rozmiaru markera, oświetlenia i zasłonięć.
  Siatka i obrót nie gwarantują widoczności pięciu kostek w dowolnym środowisku.
- `robot_radius=0.30` i zapas 0.10 m są konserwatywnym punktem startowym,
  wymagającym sprawdzenia rzeczywistego obrysu Waffle i konfiguracji Nav2.
  Jeśli bieżąca pozycja nie mieści się w powiększonej wolnej przestrzeni, misja
  zakończy się `MAP_ERROR`; popraw parametry/mapę lub przestaw robota.
- `max_range` dotyczy jakości kamery i nie wynika z zasięgu lidaru.
- Monokularna estymacja kwadratowego markera może być niejednoznaczna,
  szczególnie z daleka i pod małym kątem. Filtr redukuje szum, ale nie usuwa
  systematycznych błędów kalibracji, TF, rozmiaru markera ani lokalizacji AMCL.
- Potwierdzenie z wielu klatek nie jest pełną fuzją probabilistyczną.
  Nie ma wymagania obserwacji z różnych pozycji robota ani estymacji kowariancji.
- Robot i mapa powinny pozostawać w tej samej osi czasu i układzie odniesienia.
  Po resetach świata uruchom ponownie misję i wyczyść stan przed jej startem.
- Algorytm siatki działa na całej mapie w pamięci; przeznaczony jest do map
  laboratoryjnych. Nie implementuje eksploracji nieznanej przestrzeni.

## Procedura testu całego systemu

1. Uruchom właściwą symulację Waffle z RGB, TF, kalibracją, lidar 3.5 m,
   mapą, lokalizacją i Nav2. Umieść pięć wizualnych kostek z różnymi ID
   DICT_ARUCO_ORIGINAL. Zapisz ich rzeczywiste pozycje i geometrię markerów.
2. Potwierdź tematy, TF, czas, akcje Nav2 i działający samodzielny przejazd.
3. Uruchom aplikację ze zmierzonym `marker_size`. Sprawdź
   `/aruco/detections`: ID, optyczny frame, czas obrazu i metryczną głębokość.
4. Obserwuj `/cubes`; jedno ID musi wystąpić tylko raz. Porównaj XYZ z prawdą
   symulacji, uwzględniając offset marker–kostka. Zapisz błędy i rosbag.
5. Wyślij `/find_cubes`, potwierdź przejazdy, obrót i sukces z pięcioma ID.
6. Zasłoń jedną kostkę: oczekuj `EXHAUSTED` lub `TIMEOUT` z częściową listą.
7. Anuluj podczas jazdy i obrotu; sprawdź wyniki akcji Nav2 oraz brak ruchu.
8. Sprawdź niedostępny punkt, utratę TF, zatrzymany zegar, błędną kalibrację
   i odrzucenie drugiego zadania. Po każdej zmianie sceny resetuj stan.

Ta procedura wymaga wykonania w rzeczywistej symulacji. Syntetyczny test ROS
z atrapami akcji nie jest potwierdzeniem działania TurtleBot3 w Gazebo.
