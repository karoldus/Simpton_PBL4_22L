## PBL4 - karol-button-tests
 - wejścia z przycisku ustawione na GPIO_EXTI (0 i 1) + zmiana na Falling edge
 - włączone przerwania EXTI line 0 i 1 w GPIO -> NVIC
 - dodana funkcja obsługi przerwania, która zmienia stan diody (G po zbliżeniu, R po dotknięciu) i wypisuje komunikat