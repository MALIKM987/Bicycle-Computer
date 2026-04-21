# Bicycle Computer

## Opis projektu
Bicycle Computer to projekt systemu embedded zrealizowany w języku C dla mikrokontrolera STM32. Celem projektu jest budowa komputera rowerowego, który może stanowić bazę do pomiaru i prezentacji parametrów jazdy, takich jak prędkość, dystans czy czas przejazdu.

Projekt został przygotowany w środowisku typowym dla mikrokontrolerów STM32, z wykorzystaniem wygenerowanej struktury projektu oraz sterowników niskopoziomowych.

## Główne założenia
- implementacja aplikacji embedded w języku C,
- praca na platformie STM32,
- integracja kodu użytkownika ze sterownikami i konfiguracją mikrokontrolera,
- rozwój projektu w strukturze typowej dla STM32CubeIDE / STM32CubeMX.

## Wykorzystane technologie
- C
- STM32
- STM32CubeIDE / STM32CubeMX
- Embedded systems

## Struktura projektu
- `Core/` – kod aplikacyjny i pliki inicjalizacyjne,
- `Drivers/` – sterowniki i biblioteki mikrokontrolera,
- `Debug/`, `Release/` – pliki buildów,
- `*.ioc` – konfiguracja mikrokontrolera i peryferiów,
- `*.ld` – skrypt linkera.

## Zakres mojej pracy
W projekcie zajmowałem się:
- konfiguracją i organizacją projektu dla mikrokontrolera STM32,
- rozwojem logiki aplikacji w języku C,
- integracją kodu użytkownika z wygenerowaną strukturą projektu,
- pracą z typowym workflow dla systemów wbudowanych.

## Status projektu
Projekt jest rozwijany jako praktyczna realizacja z zakresu systemów embedded i elektroniki.

## Możliwe kierunki rozwoju
- obsługa czujników prędkości i dystansu,
- wyświetlanie danych na ekranie,
- zapis historii przejazdów,
- optymalizacja poboru mocy,
- rozbudowa o dodatkowe interfejsy komunikacyjne.

## Autor
Maciej Molik
