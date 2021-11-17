# InductorTester
Tester is based on tester seen on DiodeGoneWild Youtube channel (http://danyk.cz/avr_ring_en.html).
In short tester charges film capacitor with high Q factor and then connects it in parallel to test inductor. LC circuits resonates and microcontroller counts the number of rings. If the inductor has short turns the number of rings is significantly lower.

The hardware deviation from above mentioned circuit is the use of very fast comparator. With this the controller only counts square pulses which can be simply detected with the use of GPIO edge interrupt. It is crucial to use comparator in ns response range. The period of the ringing can be as low as 10us. With single supply to the comparator a protective diode was also used on comparator input to cut negative cycles. 

The number of rings is displayed on 4 digit display type TM1637. Library from rogerdahl was used with small modifications.

As bonus the timer with 1us increments was used to measure period between first and second pulse. With known capacitor the approximate inductance can be calculated and displayed when button is pressed. That is useful to ball park the inductance as very high inductance inductors can show only few rings as with slow resonance the initial energy in capacitor is spend on resistances over longer time. The inductance is given in uH for up to 9999uH then in mH indicated by last dot.
All zeros with dots indicate that no inductor is connected.
