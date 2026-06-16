# Testowanie i walidacja

## Cel testow

Testy maja potwierdzic, ze firmware dziala poprawnie na biurku oraz podczas rzeczywistej jazdy.

## Testy stanowiskowe

- start programu bez zawieszenia,
- poprawna inicjalizacja OLED,
- poprawny odczyt BME280,
- reakcja IMU na ruch plytki,
- odbior danych przez UART,
- reakcja przycisku,
- poprawne zliczanie impulsow z czujnika Halla.

## Testy terenowe

W terenie nalezy porownac predkosc i dystans z innym urzadzeniem, sprawdzic stabilnosc wskazan OLED oraz zapisac przykladowy log UART/CSV.

## Walidacja algorytmow

Najwazniejsze elementy do sprawdzenia to filtr predkosci, timeout zatrzymania, estymacja wysokosci, nachylenie, VAM oraz szacowanie mocy.
