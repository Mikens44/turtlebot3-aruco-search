# Wyniki walidacji

Weryfikację, w tym poprawki układu optycznego i rozmiaru wzoru z 2026-11-16,
wykonano w tymczasowych kontenerach z istniejącego,
lokalnego obrazu `arm/lab05`. Nie instalowano ROS ani bibliotek na hoście.
Kontenery miały wyłączoną sieć zewnętrzną, workspace zamontowany jako `/ws`
i `ROS_DOMAIN_ID=93` dla launch. Przypadki integracyjne używają oddzielnych
domen DDS od 100, aby ponowne używanie nazw atrap Nav2 nie mieszało discovery
pomiędzy kolejnymi testami.

## Potwierdzone środowisko

| Element | Wersja / stan |
|---|---|
| System kontenera | Ubuntu 24.04, x86_64 |
| ROS 2 | Jazzy |
| Python | 3.12.3 |
| OpenCV | 4.6.0, moduł ArUco dostępny, starsze API `detectMarkers` |
| cv_bridge | 4.1.0 |
| nav2_msgs | 1.3.13 |
| pytest | 7.4.4 |
| TF2, Nav2, colcon, NumPy | Dostępne |
| TurtleBot3 | Brak paczek `turtlebot3_*` na liście `ros2 pkg list` |
| Gazebo | Dostępne pakiety `ros_gz` i zależności vendor; nie uruchamiano symulatora |

Host: Ubuntu 26.04.1, Python 3.14.4; brak aktywnego ROS, colcon, OpenCV i NumPy.
Próba wczytania lokalnego `~/ros2_kilted/install/setup.bash` wykazała brakujące
pliki i nie dostarczyła działającego środowiska.

## Budowanie i testy

W kontenerze, z katalogu `/ws`:

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select cube_search_interfaces turtlebot3_cube_search
source install/setup.bash
colcon test --packages-select cube_search_interfaces turtlebot3_cube_search --event-handlers console_direct+
colcon test-result --verbose
```

Końcowe wyniki:

```text
Summary: 2 packages finished
48 passed in 21.41s
Summary: 48 tests, 0 errors, 0 failures, 0 skipped
```

| Grupa | Liczba | Zakres |
|---|---:|---|
| Detektor | 8 | Syntetyczny marker, ID, głębokość metryczna, zakres, kwaterniony, walidacja rozmiaru i regresja skali arm05_sim |
| Układ detekcji | 3 | Domyślny i własny optical_frame, zachowanie timestampu i nagłówków źródłowych, brak dodatkowego obrotu pozy i orientacji |
| Estymator | 6 | Szum, odstające pomiary, duplikaty, czas, rozrzut, reset i brak skoków pozycji |
| Siatka | 4 | Obrócony początek mapy, przeszkody, nieznane komórki, margines, rozłączność i narożniki |
| Cele asynchroniczne | 4 | Anulowanie przed przyjęciem celu, odrzucenie, wynik, deadline i błąd transportu |
| Integracja ROS | 21 | Wiadomości, TF optyczny, detektor–lokalizator, atrapy Nav2 i cykl misji |
| Launch | 2 | Zainstalowane executables, YAML, trzy node'y, akcja, domyślny/własny optical_frame i czyste zamknięcie po SIGINT |

Integracja ROS sprawdza między innymi sukces oraz anulowanie w każdej z trzech
faz (planowanie, jazda, obrót), odrzucenie równoległego zadania, częściowy wynik
po timeout, pomijanie nieosiągalnych punktów, wyczerpanie siatki, zmianę i brak
mapy, timeout pojedynczego przejazdu, opóźniony TF, offset markera, transformację
według **czasu pomiaru**, timeout przy zatrzymanym zegarze symulacji oraz
`STOP_FAILED` z blokadą następnych misji.

Pierwotna walidacja obejmowała także `ros2 pkg executables` oraz
`ros2 launch ... --show-args`. Testy launch nadal sprawdzają uruchamianie procesów
i ich zamykanie po powtórzonym SIGINT.

## Regresja: współrzędne optyczne kamery

Potwierdzono błąd w detektorze: nagłówek obrazu `camera_rgb_frame` był kopiowany
do pozy OpenCV, mimo że jest ona wyrażona w osiach optycznych. Lokalizator
poprawnie stosował TF dla podanej ramki i czasu, ale ramka była błędna.
Detektor ma teraz parametr `optical_frame` (domyślnie `camera_rgb_optical_frame`)
i zachowuje oryginalny czas obrazu, bez modyfikowania wejściowego nagłówka.
Poza OpenCV nie jest ręcznie obracana. Nie dodano żadnego publikatora TF do aplikacji.

Regresja odtwarza podany pomiar `[-0.23184, 0.10913, 2.77726]` m, translację
kamery `[0.069, -0.047, 0.117]` m i kwaternion `[-0.5, 0.5, -0.5, 0.5]`.
Uzyskana pozycja wynosi `[2.84626, 0.18484, 0.00787]` m, gdy `map` i
`base_footprint` pokrywają się. Drugi przypadek obraca robota o 90 stopni
i przesuwa go w `map` o `[1, 2, 0]`, oczekując `[0.81516, 4.84626, 0.00787]` m.
Oba przypadki wykorzystują domyślne progi lokalizatora: pięć pomiarów
rozłożonych na co najmniej 0.8 s. TF jest dostarczany tylko przez atrapę modelu
robota w teście. Zachowano też test transformacji w czasie pomiaru, zamiast
używania najnowszego TF, oraz test całego łańcucha obraz–detekcja–lokalizacja
z mechanicznym frame_id na wejściu i obróconym TF optycznym.

W samej poprawce ramki optycznej algorytm eksploracji, warunki zakończenia
akcji, rozmiar markera i progi potwierdzania pozostały bez zmian.

## Osobna poprawka: rozmiar wzoru arm05_sim

Po publikacji wersji bazowej `6548530` ustawiono w YAML `marker_size: 0.07`
i zaktualizowano instrukcje uruchomienia do `marker_size:=0.07`.
Nie zmieniono kodu node'ów, launch, interfejsów ani progów potwierdzania.

Nowy test wyznacza narożniki dla wzoru o boku `0.09 * 588 / 755` m,
umieszczonego na głębokości 3.8 m. Rozmiar PnP odczytuje z rzeczywistego YAML.
Przy ustawieniu 0.07 m błąd pozycji w idealnych danych wynosi około 5 mm
(test wymaga mniej niż 1 cm). Dawne 0.09 m daje ponad 1 m błędu dla tych samych
narożników. Test nie wymaga dołączania zewnętrznych modeli ani tekstur.

Wyniki te dotyczą testu numerycznego i zestawu automatycznego. Po poprawce
rozmiaru nie wykonano ponownie pełnej misji w Gazebo ani pomiaru dokładności
pięciu kostek. Sukces misji z rozmiarem 0.09 m zgłosił użytkownik i zapisano go
w `BASELINE.md`; ten historyczny dokument nie został zmieniony.

Raport pytest: `build/turtlebot3_cube_search/pytest.xml`.
Logi ostatniego przebiegu colcon: `log/latest_test/` i `log/latest_build/`.
Artefakty budowania odnoszą się do ścieżki `/ws` w kontenerze; po przeniesieniu
workspace lub zmianie środowiska należy ponownie zbudować paczki.

## Czego nie potwierdzono

- Nie wykonano testu z fizycznym TurtleBot3 ani w Gazebo.
- Atrapy akcji nie weryfikują planera, regulatora, kolizji ani rzeczywistego
  zatrzymania robota. Przeprowadzone testy sprawdzają protokół i logikę aplikacji.
- Nie dostarczono świata symulacji, mapy, modelu Waffle z kamerą, kalibracji
  konkretnej kamery ani konfiguracji AMCL/Nav2 dla konkretnego środowiska.
- Nie sprawdzono działania na innych dystrybucjach ROS ani nowszym API OpenCV.
- Nie zmierzono dokładności pozycji na prawdziwych obrazach, rzeczywistego zasięgu
  detekcji ani kompletności pokrycia konkretnej mapy.

Procedura przeprowadzenia tych prób znajduje się w README paczki.
