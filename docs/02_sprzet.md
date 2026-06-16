# Sprzęt i czujniki

## Spis treści

1. [Platforma prototypowa](#platforma-prototypowa)
2. [Lista elementów](#lista-elementów)
3. [Czujnik Halla](#czujnik-halla)
4. [BME280](#bme280)
5. [IMU](#imu)
6. [OLED SSD1306](#oled-ssd1306)
7. [UART](#uart)
8. [Zasilanie](#zasilanie)
9. [Kierunek rozwoju sprzętu](#kierunek-rozwoju-sprzętu)

## Platforma prototypowa

Prototyp zakłada użycie mikrokontrolera STM32L432KC na płytce NUCLEO-L432KC. Płytka jest wygodna do prac rozwojowych, ponieważ posiada programator/debugger ST-LINK, łatwy dostęp do pinów i obsługę w STM32CubeIDE.

## Lista elementów

| Element | Interfejs | Zadanie |
|---|---|---|
| STM32L432KC | MCU | główna jednostka sterująca |
| Czujnik Halla | GPIO / EXTI | pomiar obrotów koła |
| BME280 | I2C | temperatura, ciśnienie, wilgotność |
| IMU, np. BMI160 | I2C | orientacja i drgania |
| OLED SSD1306 128x64 | I2C | wyświetlanie danych |
| Przycisk | GPIO | zmiana ekranu |
| UART | USART | diagnostyka i CSV |

## Czujnik Halla

Czujnik Halla wykrywa pole magnetyczne magnesu zamontowanego na kole. Gdy magnes przechodzi obok czujnika, na wejściu mikrokontrolera pojawia się impuls.

Najważniejsze problemy praktyczne:

- drgania i zakłócenia mogą generować fałszywe impulsy,
- przewód od koła może zbierać zakłócenia,
- wejście powinno mieć stabilny poziom logiczny,
- w firmware potrzebna jest eliminacja zbyt szybkich impulsów.

## BME280

BME280 mierzy temperaturę, ciśnienie i wilgotność. W projekcie jego najważniejszym zastosowaniem jest pomiar ciśnienia, z którego można oszacować wysokość.

Ograniczenia:

- wysokość z ciśnienia zależy od warunków pogodowych,
- do dokładniejszych wyników potrzebne jest ciśnienie odniesienia,
- czujnik powinien być umieszczony tak, aby nie był ogrzewany bezpośrednio przez elektronikę.

## IMU

IMU dostarcza dane z akcelerometru. Przy spokojnym ruchu można oszacować pochylenie płytki względem grawitacji. Dane z akcelerometru mogą też służyć do prostego wskaźnika drgań.

W projekcie IMU może realizować:

- roll,
- pitch,
- wskaźnik drgań,
- diagnostykę sposobu montażu urządzenia.

## OLED SSD1306

OLED 128x64 umożliwia lokalne wyświetlanie danych. Ze względu na małą rozdzielczość interfejs powinien być podzielony na strony.

Przykładowe strony:

- RIDE - prędkość i dystans,
- CLIMB - wysokość i podjazd,
- ENV - temperatura, ciśnienie, wilgotność,
- IMU - orientacja i drgania,
- STATS - statystyki jazdy.

## UART

UART jest używany do diagnostyki. Pozwala sprawdzić dane bez patrzenia na OLED i zapisywać logi CSV do późniejszej analizy.

Zalecane ustawienia:

- 115200 b/s,
- 8 bitów danych,
- brak parzystości,
- 1 bit stopu.

## Zasilanie

W prototypie zasilanie może pochodzić z USB. W wersji docelowej potrzebne będzie zasilanie bateryjne, stabilizator napięcia i analiza poboru prądu.

## Kierunek rozwoju sprzętu

Docelowo projekt może zostać przeniesiony na własną płytkę PCB. Wtedy warto przewidzieć:

- złącze programujące SWD,
- złącze czujnika Halla,
- złącze OLED,
- złącza I2C,
- układ zasilania bateryjnego,
- zabezpieczenia wejść,
- wygodne punkty testowe.
