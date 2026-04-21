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


# Bicycle Computer

## Project Overview
Bicycle Computer is an embedded systems project developed in C for an STM32 microcontroller. The goal of the project is to build a bicycle computer that can serve as a platform for measuring and displaying ride-related parameters such as speed, distance, and ride time.

The project follows a typical STM32 development workflow, using a generated project structure and low-level drivers.

## Main Objectives
- develop an embedded application in C,
- work on an STM32-based platform,
- integrate user code with drivers and MCU configuration,
- maintain a project structure typical for STM32CubeIDE / STM32CubeMX.

## Technologies Used
- C
- STM32
- STM32CubeIDE / STM32CubeMX
- Embedded systems

## Project Structure
- `Core/` – application code and initialization files,
- `Drivers/` – MCU drivers and libraries,
- `Debug/`, `Release/` – build output files,
- `*.ioc` – microcontroller and peripheral configuration,
- `*.ld` – linker script.

## My Contribution
In this project, I was responsible for:
- configuring and organizing the STM32 project,
- developing application logic in C,
- integrating user code with the generated project structure,
- working within a typical embedded systems development workflow.

## Project Status
The project is being developed as a practical embedded systems and electronics implementation.

## Possible Future Improvements
- support for speed and distance sensors,
- data visualization on a display,
- ride history logging,
- power consumption optimization,
- extension with additional communication interfaces.

## Author
Maciej Molik
