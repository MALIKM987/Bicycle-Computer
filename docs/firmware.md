# Projekt firmware'u

## Zadania firmware'u

Firmware odpowiada za inicjalizację peryferiów, odczyt czujników, obsługę przerwań, obliczanie parametrów jazdy, aktualizację interfejsu OLED oraz wysyłanie danych diagnostycznych przez UART.

## Proponowany podział

Projekt warto podzielić na moduły odpowiedzialne za pomiar prędkości, statystyki jazdy, dane środowiskowe, dane z IMU, metryki podjazdu, model mocy, ekran OLED oraz logowanie UART.

## Pętla główna

Pętla główna powinna być nieblokująca. Zadania okresowe powinny korzystać ze znaczników czasu zamiast długich opóźnień.

## Obsługa przerwań

Przerwanie od czujnika Halla powinno tylko zapisać czas impulsu i ustawić flagę. Cięższe obliczenia powinny być wykonywane poza przerwaniem.

## Zasady jakości kodu

- ograniczać ilość kodu w pliku głównym,
- oddzielać logikę aplikacji od kodu wygenerowanego przez CubeMX,
- stosować jasne nazwy zmiennych z jednostkami,
- unikać blokujących opóźnień,
- dodawać obsługę błędów czujników i magistrali.
