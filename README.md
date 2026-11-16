# turtlebot3-aruco-search

Projekt akademicki ROS 2: autonomiczne wyszukiwanie pięciu kostek ArUco przez
TurtleBot3 Waffle na znanej mapie. Kamera RGB służy do wykrywania i lokalizacji
znaczników, a lidar do lokalizacji robota i nawigacji przez Nav2.

## Wyniki końcowego eksperymentu w Gazebo

W pełnym teście w **ROS 2 Jazzy + Gazebo Harmonic** TurtleBot3 Waffle
**autonomicznie odnalazł wszystkie pięć znaczników ArUco (ID 0–4)**.
Po poprawieniu rozmiaru wzoru na `marker_size:=0.07` akcja `/find_cubes`
ponownie zakończyła się sukcesem:

```text
Goal finished with status: SUCCEEDED
code: 0
```

Błąd XY jest odległością euklidesową między pozycją oszacowaną a pozycją
referencyjną: `sqrt((x_est - x_ref)^2 + (y_est - y_ref)^2)`.
Średnia jest średnią arytmetyczną błędów pięciu kostek. Współrzędne oszacowane
pochodzą z wyniku zakończonej akcji `/find_cubes`, przekazanego przez autora
eksperymentu; nie użyto późniejszych aktualizacji `/cubes`.

| ID | Gazebo X [m] | Gazebo Y [m] | Wynik akcji X [m] | Wynik akcji Y [m] | Błąd XY [cm] |
|---|---:|---:|---:|---:|---:|
| 0 | -0.274347664 | 0.329628692 | -0.201268829 | 0.306422823 | 7,667 |
| 1 | -4.582670572 | 0.069896309 | -4.602684275 | 0.000272130 | 7,244 |
| 2 | -2.967552403 | -4.601375238 | -3.006144825 | -4.574235166 | 4,718 |
| 3 | -1.017191552 | -4.449580775 | -1.024153840 | -4.408878549 | 4,129 |
| 4 | -1.030826303 | 1.219633894 | -0.992684770 | 1.182179738 | 5,346 |
| **Średnia** | — | — | — | — | **5,821 ≈ 5,8** |

Średnią obliczono z niezaokrąglonych błędów; wynosi `0.058209663089 m`.

| Przebieg | `marker_size` | Znalezione ID | Status akcji | Średni błąd XY |
|---|---:|---:|---|---:|
| Przed poprawką skali | 0.09 m | 5/5 | `SUCCEEDED` | około 107 cm |
| Po poprawce skali | 0.07 m | 5/5 | `SUCCEEDED` | **około 5,8 cm** |

Średni błąd zmalał około **18-krotnie**. Wyniki opisują ten eksperyment
symulacyjny; nie są gwarancją tej samej dokładności w każdym środowisku.
Nie wykonano testu na fizycznym robocie.

## Rozmiar markera i układ odniesienia

W modelach `arm05_sim` kostka ma bok **9 cm**, ale wzór ArUco z czarnym
obramowaniem zajmuje około **588 × 588 px** w teksturze **755 × 755 px**.
Rozmiar do solvePnP to bok wzoru bez białego marginesu:

```text
0.09 * 588 / 755 ≈ 0.07009 m
```

Podanie 0.09 m zamiast około 0.07 m zawyżało odległości o około 28,4%.
W idealnym teście numerycznym dla głębokości 3.8 m powodowało ponad metr błędu,
mimo praktycznie zerowego błędu reprojekcji. Korekta dotyczy skali pomiaru
wizualnego; nie wymaga zmiany Nav2 ani strategii wyszukiwania.

OpenCV zwraca pozycję i orientację w osiach optycznych: X w prawo, Y w dół,
Z w przód. Detekcje są oznaczone `camera_rgb_optical_frame`, zachowują czas
obrazu, a TF2 przelicza je do `map` dla tego czasu. Kamera może oznaczać obrazy
i CameraInfo jako `camera_rgb_frame`; nie wolno przenosić tej nazwy na pozę
optyczną. Nie dodawaj ręcznego obrotu ani nowego statycznego TF, jeśli właściwa
transformacja już istnieje w modelu robota.

Log spawnera Gazebo podaje pozycje początku modeli w układzie świata.
Ich XY odpowiada środkom kostek; aplikacja domyślnie raportuje środki markerów
w `map`. Marker na bocznej ścianie może być przesunięty względem środka kostki
o około 4,5 cm. Ocena dokładności wymaga zgodności układów odniesienia
i uwzględnienia definicji raportowanego punktu. Wynik końcowy akcji stanowi
migawkę potwierdzonych pozycji; późniejsze `/cubes` może zawierać zaktualizowane
oszacowania, dlatego nie należy mieszać tych pomiarów w jednej tabeli.

Wersję sprzed korekty zachowuje commit `6548530`, a samą korektę commit
`c61b749`. Historyczne raporty pozostają dostępne w Git:
[BASELINE.md](https://github.com/Mikens44/turtlebot3-aruco-search/blob/6548530/BASELINE.md)
oraz [VALIDATION.md](https://github.com/Mikens44/turtlebot3-aruco-search/blob/c61b749/VALIDATION.md).

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
Dla innych modeli należy zmierzyć bok samego wzoru z czarnym obramowaniem,
bez białego marginesu. Na fizycznym robocie użyj `use_sim_time:=false`.
Domyślnie raportowany jest środek markera; offset do środka kostki jest
parametrem lokalizatora. Nie dodawaj dodatkowego obrotu ani statycznego TF,
jeżeli model robota udostępnia już poprawny układ optyczny.

## Testy automatyczne

```bash
colcon test --packages-select cube_search_interfaces turtlebot3_cube_search --event-handlers console_direct+
colcon test-result --verbose
```

Ostatni wykonany przebieg automatyczny: **48 testów, zero błędów i pominiętych**.
Obejmuje detekcję, filtrację, transformacje optyczne i czasowe, siatkę,
anulowanie celów, atrapy Nav2 i uruchamianie przez launch. Regresja rozmiaru
sprawdza metryczną pozycję na podstawie proporcji wzoru do tekstury `arm05_sim`.
Budowanie obu paczek przez `colcon build` zakończyło się poprawnie.

| Grupa testów | Liczba |
|---|---:|
| Detekcja, metryczna pozycja i skala wzoru | 8 |
| Ramka optyczna i timestamp detekcji | 3 |
| Filtracja i potwierdzanie pozycji | 6 |
| Siatka i osiągalność punktów | 4 |
| Asynchroniczne cele i anulowanie | 4 |
| Integracja ROS z TF2 i atrapami Nav2 | 21 |
| Launch, parametry i zamykanie procesów | 2 |
| **Razem** | **48** |

Sprawdzono m.in. sukces i anulowanie podczas planowania, jazdy i obrotu,
odrzucenie równoległego zadania, timeout z częściowym wynikiem, brak lub zmianę
mapy, wyczerpanie siatki, opóźniony TF, transformację dla czasu obrazu,
zatrzymany zegar symulacji oraz `STOP_FAILED`. Regresja skali wymaga błędu
poniżej 1 cm na idealnych narożnikach markera z modelu; nie jest to próg
dokładności całego robota w Gazebo.

Automatyczne sprawdzenia wykonano w środowisku Ubuntu 24.04, ROS 2 Jazzy,
Python 3.12.3, OpenCV 4.6.0, cv_bridge 4.1.0, nav2_msgs 1.3.13 i pytest 7.4.4.
Testy integracyjne izolują domeny DDS między przypadkami. Atrapy Nav2 weryfikują
protokół i logikę, ale nie fizykę, regulator ani rzeczywiste zatrzymanie robota.
Pełny eksperyment Gazebo opisano oddzielnie powyżej.

Raport automatyczny po lokalnym uruchomieniu znajduje się w
`build/turtlebot3_cube_search/pytest.xml`, a logi w `log/latest_test/`
i `log/latest_build/`. Artefakty budowania nie należą do repozytorium;
po przeniesieniu workspace lub zmianie środowiska należy zbudować paczki ponownie.

## Ograniczenia

- Wyniki dotyczą jednej zakończonej misji w Gazebo; nie potwierdzają działania
  na fizycznym TurtleBot3 ani zgodności z innymi dystrybucjami ROS.
- Dokładność zależy od rozmiaru wzoru, kalibracji, TF i lokalizacji robota.
  Wielokrotne obserwacje ograniczają szum, lecz nie usuwają błędu systematycznego.
- Siatka i obrót nie gwarantują widoczności wszystkich markerów na dowolnej
  mapie. Zasłonięcia, FOV i odstęp punktów wymagają dopasowania do środowiska.
- Nie badano kompletności pokrycia dowolnych map ani rzeczywistego zasięgu
  detekcji na fizycznej kamerze. Świat, modele i mapa `arm05_sim` są zewnętrzne.

Pełne instrukcje diagnostyki, resetowania obserwacji, anulowania i konfiguracji:
[README paczki](src/turtlebot3_cube_search/README.md).

## Pliki repozytorium i licencja

Repozytorium obejmuje źródła obu paczek, konfigurację, launch, testy oraz
dokumentację. `build/`, `install/`, `log/`, cache, lokalne ustawienia IDE
i pliki poświadczeń są pomijane przez `.gitignore`.

Licencja Apache-2.0, zgodnie z manifestami paczek: [LICENSE](LICENSE).
