# SABIAR1
<img width="4000" height="3000" alt="20260918_120404" src="https://github.com/user-attachments/assets/5e4ad0b1-d53e-4866-8e66-b6348d720492" />
<img width="4000" height="3000" alt="20260918_120417" src="https://github.com/user-attachments/assets/9683019d-1334-497a-8caf-af61324176c7" />

Project files for the Sabiá R1 flight computer

These are the kicad files for the ESP32S3 based flight computer I designed over the summer, Sabiá R1. This is intended as a practise project to improve my skills in programming and pcb design. There are, however, known issues with the design that anyone who uses this design should adress:
first, in the BOM file attached, one of the current limiting/voltage divider resistors in the pyro channels is 7.5 $\ohm$, when it should be 7.5k, causing the led to fail upon arming the pyro channel. Further, the battery voltage measurement circuit was mistakenly connected to a pin with no ADC, so it does not work. Testing is still ongoing so I will keep this page updated.

## UPDATE 08/10/2026

I have replaced the faulty resistors with the correct valued ones, and this validated the continuity testing, both at the GPIO pin and the LED. Further, the pyros were tested and shown to work, melting through a thin copper wire at 150ms firing time, drawing a estimated 3 amps.

