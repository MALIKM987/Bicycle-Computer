# Algorytmy obliczeniowe

## Spis treści

1. [Prędkość](#prędkość)
2. [Dystans](#dystans)
3. [Filtrowanie prędkości](#filtrowanie-prędkości)
4. [Wykrywanie zatrzymania](#wykrywanie-zatrzymania)
5. [Wysokość z ciśnienia](#wysokość-z-ciśnienia)
6. [Nachylenie](#nachylenie)
7. [VAM](#vam)
8. [Roll i pitch](#roll-i-pitch)
9. [Wskaźnik drgań](#wskaźnik-drgań)
10. [Szacowanie mocy](#szacowanie-mocy)

## Prędkość

Prędkość jest obliczana z czasu pomiędzy impulsami czujnika Halla. Jeżeli jeden impuls odpowiada jednemu obrotowi koła, to prędkość chwilowa wynika z obwodu koła i okresu między impulsami.

W praktyce należy pamiętać, że:

- przy bardzo małych prędkościach impulsy pojawiają się rzadko,
- przy zatrzymaniu roweru ostatnia obliczona prędkość nie może zostać na ekranie na stałe,
- fałszywe impulsy mogą powodować chwilowe skoki prędkości.

## Dystans

Dystans jest sumą zaakceptowanych impulsów pomnożonych przez obwód koła. Najważniejszym parametrem kalibracyjnym jest więc rzeczywisty obwód koła roweru.

## Filtrowanie prędkości

Do wygładzenia wskazań można użyć filtru EMA. Filtr zmniejsza skoki wskazań, ale nie powinien być zbyt wolny, aby użytkownik widział realną zmianę prędkości.

Parametr filtru powinien być dobrany eksperymentalnie podczas jazdy.

## Wykrywanie zatrzymania

Jeżeli od ostatniego impulsu minął określony czas, firmware powinien ustawić prędkość na zero. Dzięki temu licznik poprawnie reaguje na zatrzymanie roweru.

## Wysokość z ciśnienia

Wysokość można oszacować na podstawie ciśnienia atmosferycznego. Jest to wygodne, ale wynik zależy od pogody i ciśnienia odniesienia.

Wnioski praktyczne:

- wysokość bez kalibracji jest przybliżona,
- krótkotrwałe zmiany ciśnienia mogą powodować szum,
- do nachylenia lepiej używać zmian wysokości z okna czasowego, a nie pojedynczych próbek.

## Nachylenie

Nachylenie trasy jest obliczane z różnicy wysokości i przebytego dystansu. Dla stabilnego wyniku należy liczyć je na oknie pomiarowym, a nie z dwóch sąsiednich próbek.

## VAM

VAM oznacza tempo zdobywania wysokości w metrach na godzinę. Jest przydatny przy analizie podjazdów.

## Roll i pitch

Roll i pitch można oszacować na podstawie wektora przyspieszenia, gdy układ nie wykonuje gwałtownych ruchów. Przy dynamicznej jeździe wynik może być zaburzony przez drgania i przyspieszenia inne niż grawitacja.

## Wskaźnik drgań

Wskaźnik drgań można wyznaczać z danych akcelerometru. Nie musi to być pomiar laboratoryjny - wystarczy orientacyjna wartość pokazująca, czy rower jedzie po gładkiej czy nierównej nawierzchni.

## Szacowanie mocy

Uproszczony model mocy może uwzględniać:

- moc potrzebną do pokonywania podjazdu,
- opory toczenia,
- opór aerodynamiczny.

Model nie uwzględnia wielu czynników, takich jak wiatr, straty napędu, dokładna pozycja rowerzysty czy zmiany nawierzchni. Wynik należy traktować jako estymację.
