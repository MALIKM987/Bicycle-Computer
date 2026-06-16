# UART i CSV

## Cel

UART jest kanałem diagnostycznym. Pozwala obserwować pracę firmware'u, wykrywać błędy i zapisywać dane do późniejszej analizy.

## Ustawienia

Zalecana konfiguracja terminala:

- 115200 b/s,
- 8 bitów danych,
- brak parzystości,
- 1 bit stopu,
- brak kontroli przepływu.

## Tryby pracy

### Tryb diagnostyczny

Tryb diagnostyczny powinien być czytelny dla człowieka. Może pokazywać inicjalizację czujników, błędy i aktualne wartości pomiarowe.

### Tryb CSV

Tryb CSV powinien wysyłać dane w stałej kolejności kolumn. Dzięki temu można je zapisać do pliku i analizować w arkuszu kalkulacyjnym albo Pythonie.

## Proponowane kolumny CSV

| Kolumna | Znaczenie |
|---|---|
| `time_ms` | czas od startu programu |
| `speed_kmh` | prędkość |
| `trip_m` | dystans przejazdu |
| `moving_time_s` | czas ruchu |
| `avg_kmh` | prędkość średnia |
| `max_kmh` | prędkość maksymalna |
| `temp_c` | temperatura |
| `pressure_hpa` | ciśnienie |
| `humidity_pct` | wilgotność |
| `altitude_m` | wysokość |
| `grade_pct` | nachylenie |
| `vam_mh` | VAM |
| `roll_deg` | roll |
| `pitch_deg` | pitch |
| `power_w` | szacowana moc |
| `kcal` | szacowana energia |

## Częstotliwość logowania

Dla zwykłej diagnostyki wystarczy 1 Hz. Do testów IMU można chwilowo zwiększyć częstotliwość, ale nie należy przeciążać UART-a niepotrzebnymi danymi.
