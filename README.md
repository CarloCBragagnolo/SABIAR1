# SABIAR1
Project files for the Sabiá R1 flight computer

These are the kicad files for the ESP32S3 based flight computer I designed over the summer, Sabiá R1. This is intended as a practise project to improve my skills in programming and pcb design. There are, however, known issues with the design that anyone who uses this design should adress:
first, in the BOM file attached, one of the current limiting/voltage divider resistors in the pyro channels is 7.5 $\ohm$, when it should be 7.5k, causing the led to fail upon arming the pyro channel. Further, the battery voltage measurement circuit was mistakenly connected to a pin with no ADC, so it does not work. Testing is still ongoing so I will keep this page updated.
