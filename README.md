# turtlebot3-aruco-search

Projekt akademicki ROS 2: autonomiczne wyszukiwanie pięciu kostek ArUco przez
TurtleBot3 Waffle na znanej mapie. Kamera RGB służy do wykrywania i lokalizacji
znaczników, a lidar do lokalizacji robota i nawigacji przez Nav2.

## Zabezpieczona wersja bazowa

Pierwszy commit `6548530` zachowuje kod **sprzed poprawki rozmiaru markera**.
Użytkownik potwierdził w ROS 2 Jazzy + Gazebo Harmonic znalezienie pięciu ID
i zakończenie `/find_cubes` statusem `SUCCEEDED`, kodem `0` oraz komunikatem
`Confirmed five distinct marker IDs`.

W tym przebiegu użyto `marker_size:=0.09`. Współrzędne miały średni błąd XY
około 1.07 m. Analiza modeli wykazała, że 0.09 m to bok kostki, a wzór ArUco
ma bok około 0.07 m. Korektę konfiguracji i instrukcji na `marker_size:=0.07`
wprowadzono w osobnym commicie po publikacji wersji bazowej. Zabezpieczony
pierwszy commit pozostaje bez zmian. Po korekcie nie wykonano jeszcze ponownie
pełnej misji w Gazebo ani pomiaru końcowej dokładności.
Szczegóły przebiegu i ograniczenia: [BASELINE.md](BASELINE.md).

## Architektura

| Paczka | Zawartość |
|---|---|
| `cube_search_interfaces` | Wiadomości detekcji i kostek oraz akcja `FindCubes` |
| `turtlebot3_cube_search` | Trzy node'y Python/rclpy, launch, YAML i testy |

- `aruco_detector`: OpenCV ArUco (`DICT_ARUCO_ORIGINAL`), poza markera w
  `camera_rgb_optical_frame`, oryginalny timestamp obrazu.
- `cube_localizer`: TF2 dla czasu pomiaru, filtrowanie obserwacji i jedna pozycja
  dla każdego potwierdzonego ID na `/cubes`.
- `cube_search_server`: siatka punktów obserwacyjnych, nawigacja Nav2, obrót
  przez Spin, obsługa timeoutów i anulowania. Akcja `/find_cubes` zwraca pięć
  unikalnych ID i ich pozycje w `map`.

## Wymagania

- ROS 2 Jazzy na zgodnym systemie, np. Ubuntu 24.04; Python 3.12.
- Nav2, TF2, OpenCV z modułem ArUco, cv_bridge, NumPy i colcon.
- Działający TurtleBot3 Waffle z kamerą RGB, kalibracją i lidarem.
- Statyczna mapa, lokalizacja robota oraz aktywne akcje Nav2:
  `/compute_path_to_pose`, `/navigate_to_pose`, `/spin`.
- Dla symulacji: Gazebo Harmonic, `ros_gz`, świat i modele robota oraz kostek.

Świat laboratoryjny `arm05_sim`, jego mapa i modele są zewnętrzną zależnością
i nie są zawarte w tym repozytorium. Dokładne odtworzenie przebiegu wymaga
również tych zasobów. Wymagany jest istniejący łańcuch TF od `map` do
`camera_rgb_optical_frame`.

To zwykły workspace ROS 2. **Docker nie jest wymagany**; był używany jako
środowisko budowania i testów. Lokalny obraz `arm/lab05` nie jest publikowany
razem z projektem. Podczas przygotowywania projektu host Ubuntu 26.04 nie miał
aktywnej instalacji ROS, dlatego sprawdzenia wykonano w kontenerze Jazzy.

## Budowanie

Po sklonowaniu repozytorium przejdź do jego katalogu głównego:

```bash
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --packages-select cube_search_interfaces turtlebot3_cube_search
source install/setup.bash
```

Polecenie `rosdep` wymaga wcześniej skonfigurowanego narzędzia i aktualnej bazy
zależności. Pakiety TurtleBot3, konfigurację kamery i bringup Nav2 przygotuj
zgodnie z własnym środowiskiem.

## Uruchomienie z poprawionym rozmiarem markera

Najpierw uruchom symulację, kamerę, mapę, lokalizację i Nav2. Następnie:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch turtlebot3_cube_search cube_search.launch.py \
  marker_size:=0.07 optical_frame:=camera_rgb_optical_frame use_sim_time:=true
```

W drugim terminalu, po wczytaniu tego samego środowiska:

```bash
ros2 action send_goal /find_cubes cube_search_interfaces/action/FindCubes \
  '{timeout_sec: 600.0}' --feedback
ros2 topic echo /cubes --qos-durability transient_local
```

`0.07` odpowiada wzorowi w modelach `arm05_sim`, bez białego marginesu.
Przed ponownym wyszukiwaniem anuluj poprzednią misję i uruchom ponownie
lokalizator lub wyczyść jego obserwacje podczas bezczynności:

```bash
ros2 service call /cube_localizer/reset std_srvs/srv/Trigger '{}'
```

W przeciwnym razie wcześniej potwierdzone, błędne pozycje pozostaną w pamięci.
Odtworzenie wersji bazowej z `0.09` opisuje [BASELINE.md](BASELINE.md).
Dla innych modeli należy zmierzyć bok samego wzoru z czarnym obramowaniem,
bez białego marginesu. Na fizycznym robocie użyj `use_sim_time:=false`.
Domyślnie raportowany jest środek markera; offset do środka kostki jest
parametrem lokalizatora. Nie dodawaj dodatkowego obrotu ani statycznego TF,
jeżeli model robota udostępnia już poprawny układ optyczny.

## Testy i wyniki

```bash
colcon test --packages-select cube_search_interfaces turtlebot3_cube_search --event-handlers console_direct+
colcon test-result --verbose
```

Ostatni wykonany przebieg automatyczny: **48 testów, zero błędów i pominiętych**.
Obejmuje detekcję, filtrację, transformacje optyczne i czasowe, siatkę,
anulowanie celów, atrapy Nav2 i uruchamianie przez launch. Regresja rozmiaru
sprawdza metryczną pozycję na podstawie proporcji wzoru do tekstury `arm05_sim`.
[VALIDATION.md](VALIDATION.md) opisuje środowisko i granice tych testów.

Sukces pełnej misji w Gazebo zgłosił użytkownik; nie jest on równoznaczny
z potwierdzeniem dokładności współrzędnych. Nie sprawdzono fizycznego robota.
Pełne instrukcje diagnostyki, resetowania obserwacji, anulowania i konfiguracji:
[README paczki](src/turtlebot3_cube_search/README.md).

## Pliki repozytorium i licencja

Repozytorium obejmuje źródła obu paczek, konfigurację, launch, testy oraz
dokumentację. `build/`, `install/`, `log/`, cache, lokalne ustawienia IDE
i pliki poświadczeń są pomijane przez `.gitignore`.

Licencja Apache-2.0, zgodnie z manifestami paczek: [LICENSE](LICENSE).
