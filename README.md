

# SABIAR1
<img width="400" height="300" alt="20260918_120404" src="https://github.com/user-attachments/assets/5e4ad0b1-d53e-4866-8e66-b6348d720492" />
<img width="400" height="300" alt="20260918_120417" src="https://github.com/user-attachments/assets/9683019d-1334-497a-8caf-af61324176c7" />

Project files for the Sabiá R1 flight computer

These are the kicad files for the ESP32S3 based flight computer I designed over the summer, Sabiá R1. This is intended as a practise project to improve my skills in programming and pcb design. There are, however, known issues with the design that anyone who uses this design should adress:
first, in the BOM file attached, one of the current limiting/voltage divider resistors in the pyro channels is 7.5 $\ohm$, when it should be 7.5k, causing the led to fail upon arming the pyro channel. Further, the battery voltage measurement circuit was mistakenly connected to a pin with no ADC, so it does not work. Testing is still ongoing so I will keep this page updated.

## UPDATE 08/10/2026

I have replaced the faulty resistors with the correct valued ones, and this validated the continuity testing, both at the GPIO pin and the LED. Further, the pyros were tested and shown to work, melting through a thin copper wire at 150ms firing time, drawing a estimated 3 amps. No tests have been conducted on a live pyro charge.

Regarding flight software, the attitude kalman filter has been updated to use the magnetometer for up-down facing disambiguation, allowing for accurate logging throughout the flight envelope. Also, the magnetometer can now take both soft and hard iron calibration. The relevant paremeters were calculated using MagMaster. Furthermore, dead-reckoning was added for the y and z axes. This is very much experimental and is not recomended to be used in any flight logic (eg. deploy parachutes early if too far down range).
