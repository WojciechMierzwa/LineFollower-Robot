#SWIM - Projekt 2024
Opis projektu

Robot klasy LineFollower sterowany autorską aplikacją na mikrokontroler STM32.
Projekt w zamyśle ma mieć 4 tryby:
  1. Śledzenie lini (autonomicznie śledzi linie)
  2. Wykrywanie obiektów (czujnik odległości zamontowany na servie, pojazd porusza się do wykrycia obiektu następnie za pomocą serva czujnik próbuje wykryć drogę bez kolizyjna)
  3. Zdalne kierowanie pojazdem za pomocą modułu bluetooth bez ograniczeń
  4. Zdalne kierowanie pojazdem za pomocą modułu bluetooth, ale pojazd zatrzymuje się przed kolizją

Zakres projektu
Wymagania projektowe:
  1. Budowa pojazdu z wykorzystaniem wybranych technik. √
  2. Opracowanie aplikacji na mikrokontroler STM32 umożliwiającej sterowanie pojazdem. √
  3. Opracowanie algorytmu śledzenia linii. √
  4. Implementacja jednej z dwóch funkcjonalności:
    a) Zdalne sterowanie pojazdem z aplikacji mobilnej. √
    b) Wykrywanie i omijanie przeszkód typu: ściana, cegła. √
