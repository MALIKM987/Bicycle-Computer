# Plan rozwoju

## Etap 1 - dokumentacja

- pełny README po polsku,
- opis architektury,
- opis sprzętu,
- opis firmware'u,
- opis algorytmów,
- diagramy Mermaid,
- opis kalibracji i testów.

## Etap 2 - porządkowanie firmware'u

- ograniczenie logiki w `main.c`,
- wydzielenie modułów,
- konfiguracja w osobnym pliku,
- spójne nazwy zmiennych z jednostkami,
- lepsza obsługa błędów.

## Etap 3 - walidacja terenowa

- test prędkości z GPS,
- test dystansu na znanej trasie,
- test wysokości i przewyższeń,
- log CSV z przejazdu,
- analiza danych po jeździe.

## Etap 4 - rozbudowa funkcji

- zapis historii przejazdów,
- tryb niskiego poboru mocy,
- ustawienia użytkownika,
- lepsza obsługa przycisku,
- dodatkowe ekrany OLED.

## Etap 5 - własna płytka PCB

- schemat,
- projekt PCB,
- złącza czujników,
- zasilanie bateryjne,
- SWD do programowania,
- obudowa.
