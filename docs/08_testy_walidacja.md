# Testy i walidacja

## Cel

Testy mają potwierdzić, że system działa poprawnie zarówno na biurku, jak i podczas jazdy.

## Testy stanowiskowe

| Test | Oczekiwany wynik |
|---|---|
| start programu | brak zawieszenia i poprawna inicjalizacja |
| OLED | ekran pokazuje stronę startową lub dane |
| BME280 | wartości temperatury, ciśnienia i wilgotności są realistyczne |
| IMU | wartości zmieniają się przy ruchu płytką |
| UART | terminal odbiera dane |
| przycisk | zmienia ekran OLED |
| czujnik Halla | impulsy zwiększają dystans i prędkość |

## Testy terenowe

Podczas jazdy należy sprawdzić:

- stabilność wskazań prędkości,
- poprawność dystansu,
- reakcję na zatrzymanie,
- czytelność OLED,
- jakość danych CSV,
- zachowanie wysokości i nachylenia.

## Walidacja danych

Dane z UART/CSV warto porównać z telefonem, GPS-em lub komercyjnym licznikiem rowerowym. Różnice należy opisać, ponieważ część z nich będzie wynikała z innej metody pomiaru.

## Testy regresyjne

W przyszłości warto wydzielić algorytmy do funkcji możliwych do testowania na PC. Szczególnie dobrze nadają się do tego:

- przeliczanie impulsów na prędkość,
- filtr prędkości,
- dystans,
- wysokość,
- VAM,
- model mocy,
- formatowanie CSV.
