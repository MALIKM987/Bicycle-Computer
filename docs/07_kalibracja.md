# Kalibracja

## Cel kalibracji

Kalibracja jest potrzebna, aby wartości pokazywane przez komputer rowerowy były możliwie bliskie rzeczywistości. Bez kalibracji projekt nadal może działać, ale wyniki będą orientacyjne.

## Obwód koła

Najważniejszy parametr dla prędkości i dystansu.

Procedura:

1. Zaznaczyć punkt na oponie.
2. Ustawić rower na płaskiej powierzchni.
3. Przetoczyć koło o jeden pełny obrót.
4. Zmierzyć dystans między znacznikami.
5. Wpisać wynik jako obwód koła.

## Czujnik Halla

Należy dobrać minimalny odstęp między impulsami. Zbyt mała wartość przepuści zakłócenia, a zbyt duża może odrzucać poprawne impulsy przy dużej prędkości.

## Timeout zatrzymania

Timeout zatrzymania określa, po jakim czasie bez impulsów prędkość zostanie ustawiona na zero. Wartość powinna być dobrana tak, aby rower szybko reagował na zatrzymanie, ale nie zerował prędkości przy bardzo wolnej jeździe.

## Wysokość i ciśnienie

Wysokość z BME280 zależy od ciśnienia odniesienia. Najlepiej skalibrować urządzenie w miejscu o znanej wysokości lub użyć aktualnego ciśnienia z lokalnej stacji pogodowej.

## Model mocy

Do modelu mocy należy dobrać:

- masę rowerzysty i roweru,
- współczynnik oporu toczenia,
- przybliżony parametr oporu aerodynamicznego,
- sprawność człowieka do szacowania energii.

## Walidacja po kalibracji

Po kalibracji warto porównać:

- prędkość z GPS,
- dystans ze znaną trasą,
- wysokość z mapą,
- przewyższenie z aplikacją rowerową,
- moc z subiektywnym odczuciem wysiłku.
