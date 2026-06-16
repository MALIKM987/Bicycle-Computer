# Algorytmy obliczeniowe

## Spis treści

1. [Założenia i oznaczenia](#założenia-i-oznaczenia)
2. [Pomiar prędkości z czujnika Halla](#pomiar-prędkości-z-czujnika-halla)
3. [Dystans i statystyki jazdy](#dystans-i-statystyki-jazdy)
4. [Filtrowanie prędkości](#filtrowanie-prędkości)
5. [Eliminacja fałszywych impulsów](#eliminacja-fałszywych-impulsów)
6. [Wykrywanie zatrzymania](#wykrywanie-zatrzymania)
7. [Wysokość z ciśnienia](#wysokość-z-ciśnienia)
8. [Nachylenie trasy](#nachylenie-trasy)
9. [VAM](#vam)
10. [Roll i pitch z akcelerometru](#roll-i-pitch-z-akcelerometru)
11. [Wskaźnik drgań](#wskaźnik-drgań)
12. [Szacowanie mocy rowerzysty](#szacowanie-mocy-rowerzysty)
13. [Energia i kilokalorie](#energia-i-kilokalorie)
14. [Ograniczenia algorytmów](#ograniczenia-algorytmów)

## Założenia i oznaczenia

W projekcie komputer rowerowy wyznacza parametry jazdy na podstawie impulsów z czujnika Halla, danych środowiskowych z BME280 oraz danych ruchu z IMU.

| Symbol | Znaczenie | Jednostka |
|---|---|---|
| `C` | obwód koła | m |
| `N` | liczba zaakceptowanych impulsów | - |
| `PPR` | liczba impulsów na jeden obrót koła | impuls/obrót |
| `T` | czas między impulsami | s |
| `v` | prędkość liniowa | m/s |
| `v_kmh` | prędkość w kilometrach na godzinę | km/h |
| `d` | dystans | m |
| `h` | wysokość | m |
| `p` | ciśnienie zmierzone | hPa |
| `p0` | ciśnienie odniesienia na poziomie morza | hPa |
| `m` | masa rowerzysty, roweru i bagażu | kg |
| `g` | przyspieszenie ziemskie | m/s² |
| `Crr` | współczynnik oporu toczenia | - |
| `CdA` | efektywna powierzchnia oporu aerodynamicznego | m² |
| `rho` | gęstość powietrza | kg/m³ |

## Pomiar prędkości z czujnika Halla

Czujnik Halla generuje impuls przy wykryciu magnesu na kole. Jeżeli jeden impuls odpowiada jednemu pełnemu obrotowi koła, wtedy prędkość można wyznaczyć z obwodu koła i czasu pomiędzy impulsami.

Dla ogólnego przypadku, gdy na jeden obrót przypada `PPR` impulsów:

$$
v = \frac{C}{T \cdot PPR}
$$

gdzie:

- `v` - prędkość w m/s,
- `C` - obwód koła w metrach,
- `T` - czas między dwoma zaakceptowanymi impulsami w sekundach,
- `PPR` - liczba impulsów na jeden obrót koła.

Przeliczenie na km/h:

$$
v_{kmh} = v \cdot 3.6
$$

Dla jednego magnesu na kole najczęściej:

$$
PPR = 1
$$

czyli:

$$
v = \frac{C}{T}
$$

Przykład: dla obwodu koła `C = 2.096 m` i czasu między impulsami `T = 0.3 s`:

$$
v = \frac{2.096}{0.3} = 6.986 \text{ m/s}
$$

$$
v_{kmh} = 6.986 \cdot 3.6 = 25.15 \text{ km/h}
$$

## Dystans i statystyki jazdy

Dystans jest liczony z liczby zaakceptowanych impulsów.

Jeżeli jeden impuls odpowiada jednemu obrotowi koła:

$$
d = N \cdot C
$$

Dla ogólnego przypadku:

$$
d = \frac{N}{PPR} \cdot C
$$

Średnia prędkość z całego przejazdu:

$$
v_{avg} = \frac{d}{t_{move}}
$$

gdzie `t_move` to czas jazdy bez postojów.

Po przeliczeniu na km/h:

$$
v_{avg,kmh} = \frac{d}{t_{move}} \cdot 3.6
$$

Maksymalna prędkość jest aktualizowana, gdy aktualna filtrowana prędkość jest większa od poprzedniego maksimum:

$$
v_{max} = \max(v_{max}, v_{filtered})
$$

## Filtrowanie prędkości

Pomiar prędkości z czujnika Halla może mieć skoki, szczególnie przy małych prędkościach lub pojedynczych zakłóceniach. Do wygładzenia wskazań można użyć filtru EMA, czyli wykładniczej średniej kroczącej.

$$
v_f[n] = \alpha \cdot v[n] + (1 - \alpha) \cdot v_f[n-1]
$$

gdzie:

- `v[n]` - nowy pomiar prędkości,
- `v_f[n]` - przefiltrowana prędkość,
- `alpha` - współczynnik filtru z zakresu 0...1.

Interpretacja:

- większe `alpha` - szybsza reakcja, ale większe skoki,
- mniejsze `alpha` - stabilniejszy odczyt, ale większe opóźnienie.

Jeżeli filtr ma być powiązany ze stałą czasową `tau`, można użyć:

$$
\alpha = \frac{\Delta t}{\tau + \Delta t}
$$

gdzie:

- `Delta t` - czas od poprzedniej aktualizacji,
- `tau` - stała czasowa filtru.

## Eliminacja fałszywych impulsów

Czujnik Halla i przewody mogą generować zakłócenia. Dlatego impulsy pojawiające się zbyt szybko po poprzednim impulsie należy odrzucać.

Warunek akceptacji impulsu:

$$
t_{now} - t_{last} > t_{debounce}
$$

gdzie:

- `t_now` - aktualny czas,
- `t_last` - czas poprzedniego zaakceptowanego impulsu,
- `t_debounce` - minimalny odstęp między impulsami.

Jeżeli warunek nie jest spełniony, impuls jest traktowany jako zakłócenie.

## Wykrywanie zatrzymania

Gdy rower się zatrzyma, impulsy z czujnika Halla przestają przychodzić. Bez dodatkowej logiki na ekranie mogłaby pozostać ostatnia prędkość.

Warunek zatrzymania:

$$
t_{now} - t_{last\_pulse} > t_{stop}
$$

Jeżeli warunek jest spełniony:

$$
v = 0
$$

oraz:

$$
v_f = 0
$$

gdzie `t_stop` to czas po którym system uznaje, że rower stoi.

## Wysokość z ciśnienia

Wysokość można szacować na podstawie ciśnienia atmosferycznego. W praktyce używa się wzoru barometrycznego:

$$
h = 44330 \cdot \left(1 - \left(\frac{p}{p_0}\right)^{0.1903}\right)
$$

gdzie:

- `h` - wysokość w metrach,
- `p` - zmierzone ciśnienie w hPa,
- `p0` - ciśnienie odniesienia w hPa.

Typowo jako wartość startową można przyjąć:

$$
p_0 = 1013.25 \text{ hPa}
$$

W praktyce dokładność wysokości zależy od pogody. Dlatego najlepszym rozwiązaniem jest kalibracja `p0` albo ustawienie znanej wysokości początkowej.

## Nachylenie trasy

Nachylenie trasy można obliczyć z różnicy wysokości i przebytego dystansu.

$$
grade = \frac{\Delta h}{\Delta d} \cdot 100\%
$$

gdzie:

- `Delta h` - zmiana wysokości,
- `Delta d` - dystans pokonany w analizowanym oknie.

Przykład:

$$
\Delta h = 5 \text{ m}
$$

$$
\Delta d = 100 \text{ m}
$$

$$
grade = \frac{5}{100} \cdot 100\% = 5\%
$$

Nachylenia nie powinno się liczyć z dwóch sąsiednich próbek, bo wysokość z barometru może być zaszumiona. Lepsze jest okno pomiarowe, np. kilka sekund jazdy albo określony dystans.

## VAM

VAM oznacza prędkość zdobywania wysokości w metrach na godzinę.

$$
VAM = \frac{\Delta h}{\Delta t} \cdot 3600
$$

gdzie:

- `Delta h` - zdobyta wysokość w metrach,
- `Delta t` - czas w sekundach,
- `3600` - przeliczenie sekund na godzinę.

Przykład:

$$
\Delta h = 20 \text{ m}
$$

$$
\Delta t = 120 \text{ s}
$$

$$
VAM = \frac{20}{120} \cdot 3600 = 600 \text{ m/h}
$$

## Roll i pitch z akcelerometru

Gdy rower stoi albo porusza się spokojnie, akcelerometr mierzy głównie składowe przyspieszenia ziemskiego. Można wtedy oszacować kąty przechylenia.

Dla osi akcelerometru `ax`, `ay`, `az`:

$$
roll = atan2(a_y, a_z)
$$

$$
pitch = atan2(-a_x, \sqrt{a_y^2 + a_z^2})
$$

Wynik w radianach można przeliczyć na stopnie:

$$
angle_{deg} = angle_{rad} \cdot \frac{180}{\pi}
$$

Ograniczenie: podczas dynamicznej jazdy wynik może być zaburzony przez drgania i przyspieszenia inne niż grawitacja.

## Wskaźnik drgań

Prosty wskaźnik drgań można policzyć z długości wektora przyspieszenia.

$$
a_{mag} = \sqrt{a_x^2 + a_y^2 + a_z^2}
$$

Następnie można odjąć wartość średnią albo wartość odpowiadającą grawitacji i obliczyć RMS w oknie pomiarowym.

$$
vib_{RMS} = \sqrt{\frac{1}{N}\sum_{i=1}^{N}(a_{mag}[i] - \overline{a}_{mag})^2}
$$

Wartość ta nie jest dokładnym pomiarem drgań laboratoryjnych, ale może pokazać, czy rower jedzie po gładkiej czy nierównej nawierzchni.

## Szacowanie mocy rowerzysty

Uproszczony model mocy może składać się z trzech głównych części:

$$
P_{total} = P_{climb} + P_{roll} + P_{aero}
$$

### Moc na podjazd

Moc potrzebna do zwiększania wysokości:

$$
P_{climb} = m \cdot g \cdot v_{vertical}
$$

gdzie:

$$
v_{vertical} = v \cdot \frac{grade}{100}
$$

czyli:

$$
P_{climb} = m \cdot g \cdot v \cdot \frac{grade}{100}
$$

### Opory toczenia

$$
P_{roll} = C_{rr} \cdot m \cdot g \cdot v
$$

gdzie `Crr` zależy od opon, nawierzchni i ciśnienia w oponach.

### Opór aerodynamiczny

$$
P_{aero} = \frac{1}{2} \cdot \rho \cdot C_dA \cdot v^3
$$

Opór aerodynamiczny rośnie z trzecią potęgą prędkości, dlatego przy szybkiej jeździe zaczyna dominować.

### Całkowita moc

$$
P_{total} = m \cdot g \cdot v \cdot \frac{grade}{100} + C_{rr} \cdot m \cdot g \cdot v + \frac{1}{2} \cdot \rho \cdot C_dA \cdot v^3
$$

Ten model jest przybliżeniem. Nie uwzględnia wiatru, strat napędu, zmian nawierzchni, pozycji rowerzysty ani przyspieszania.

## Energia i kilokalorie

Energia mechaniczna może być liczona z mocy w czasie:

$$
E = P \cdot t
$$

gdzie:

- `E` - energia w dżulach,
- `P` - moc w watach,
- `t` - czas w sekundach.

Przeliczenie dżuli na kilokalorie mechaniczne:

$$
kcal_{mech} = \frac{E}{4184}
$$

Organizm człowieka nie ma sprawności 100%. Jeżeli przyjmiemy sprawność `eta`, energia metaboliczna wynosi:

$$
kcal_{human} = \frac{kcal_{mech}}{\eta}
$$

Przykładowo dla sprawności:

$$
\eta = 0.25
$$

wynik metaboliczny jest około cztery razy większy niż energia mechaniczna.

## Ograniczenia algorytmów

Najważniejsze ograniczenia:

- wysokość z barometru zależy od pogody,
- prędkość z czujnika Halla wymaga poprawnej kalibracji obwodu koła,
- IMU jest wrażliwe na drgania,
- model mocy jest tylko estymacją,
- wiatr i pozycja rowerzysty mogą mocno zmieniać realną moc,
- nachylenie powinno być liczone z okna pomiarowego, a nie z pojedynczych próbek.
