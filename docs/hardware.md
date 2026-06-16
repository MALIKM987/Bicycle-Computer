# Opis sprzętu

## Platforma prototypowa

Aktualna wersja projektu jest przygotowana dla mikrokontrolera STM32L432KC, używanego na płytce NUCLEO-L432KC.

## Główne elementy

| Element | Interfejs | Rola |
|---|---|---|
| STM32L432KC | MCU | sterowanie całym systemem |
| czujnik Halla | GPIO / przerwanie | pomiar obrotów koła |
| BME280 | I2C | temperatura, ciśnienie, wilgotność |
| IMU | I2C | orientacja i drgania |
| OLED SSD1306 | I2C | prezentacja danych |
| przycisk | GPIO | zmiana ekranów |
| UART | USART | diagnostyka i eksport CSV |

## Uwagi sprzętowe

Czujnik Halla wymaga filtracji programowej, ponieważ zakłócenia mogą powodować fałszywe impulsy. Magistrala I2C powinna mieć poprawne rezystory podciągające. W wersji docelowej należy przewidzieć wygodne złącze programujące oraz złącza dla czujników.

## Możliwy rozwój sprzętu

- własna płytka PCB,
- zasilanie bateryjne,
- układ ładowania i zabezpieczeń,
- obudowa odporna na warunki zewnętrzne,
- złącza dla czujników i wyświetlacza,
- wyprowadzenie interfejsu programowania.
