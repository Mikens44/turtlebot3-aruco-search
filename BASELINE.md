# Wersja bazowa: pięć ID, akcja SUCCEEDED, przed poprawką marker_size

Ten dokument opisuje stan zachowany w pierwszym commicie projektu
`turtlebot3-aruco-search`. Kod lokalizacji i wyszukiwania nie został zmieniony
podczas przygotowywania repozytorium.

## Potwierdzony wynik

Użytkownik zgłosił zakończenie pełnego wyszukiwania w ROS 2 Jazzy i Gazebo Harmonic:

```text
code: 0
message: Confirmed five distinct marker IDs
Goal finished with status: SUCCEEDED
```

Parametry uruchomienia:

```text
marker_size:=0.09
optical_frame:=camera_rgb_optical_frame
use_sim_time:=true
```

Rozmiar jest obowiązkowym argumentem launch. W `config/search.yaml` pozostaje
wartość `0.0`, która celowo wymusza podanie rzeczywistej wartości przy starcie;
wartość użyta podczas opisanej misji pochodziła z argumentu `0.09`.

Wersja zawiera poprawkę oznaczania pozy OpenCV układem optycznym. Nie wolno
wracać do oznaczania tych współrzędnych jako `camera_rgb_frame`.

## Pozycje zgłoszone dla zakończonej misji

| ID | Gazebo X [m] | Gazebo Y [m] | Wynik X [m] | Wynik Y [m] |
|---|---:|---:|---:|---:|
| 0 | 9.344612 | -6.413588 | 10.268304 | -7.067797 |
| 1 | -5.491882 | 0.115610 | -5.890120 | -0.732686 |
| 2 | 8.872643 | 3.348599 | 8.456218 | 4.226258 |
| 3 | 7.232773 | -2.532455 | 8.313903 | -2.940656 |
| 4 | 5.648271 | -2.809170 | 6.746625 | -2.450956 |

Średnia odległość XY dla powyższych par: około **1.0703 m**.
Wynik potwierdza działanie wyszukiwania i zakończenia akcji, ale pozycje
w tej wersji nie są jeszcze wystarczająco dokładne.

## Znana przyczyna błędu skali — poprawka jeszcze niewprowadzona

W zewnętrznych modelach `arm05_sim` kostka ma bok 0.09 m. Tekstura ma
755 × 755 pikseli, a czarny kwadrat ArUco z obramowaniem zajmuje około
588 × 588 pikseli. Bok wzoru wynosi zatem około:

```text
0.09 * 588 / 755 = 0.07009 m
```

Podanie 0.09 m do solvePnP zawyża odległość o około 28.4%. Planowana korekta
`marker_size:=0.07` ma zostać wprowadzona dopiero po utworzeniu repozytorium,
w **osobnym commicie**. Po zmianie konieczne będzie wyczyszczenie wcześniej
potwierdzonych obserwacji oraz powtórzenie testu dokładności.

Nie jest to korekta algorytmu eksploracji, Nav2 ani interfejsów ROS.
Log symulacji zawiera pozycje początku modeli w świecie Gazebo, a aplikacja
domyślnie raportuje środki markerów w `map`; podczas oceny dokładności trzeba
uwzględnić tę różnicę i zgodność układów odniesienia.

## Zakres zabezpieczenia

Commit zachowuje obie paczki ROS 2, wiadomości i akcję, wszystkie node'y,
konfigurację, launch, testy i dokumentację. Nie obejmuje wyników budowania,
logów, obrazu Docker ani zewnętrznego świata i modeli `arm05_sim`.
Dokładne odtworzenie misji wymaga posiadania tych zewnętrznych zasobów.

Ostatni wykonany wcześniej zestaw automatyczny obejmował 47 testów zakończonych
powodzeniem; szczegóły w `VALIDATION.md`. Przy przygotowaniu tej wersji bazowej
nie deklaruje się ponownego wykonania pełnej misji w Gazebo.
