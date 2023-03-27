# Mazda-6-GJ-GL-DPF
# #DPF regeneration indicator // Wskaźnik wypalania DPF

This is a simple program which give a oportunity to get an view to DPF filling and regeneration. Program is based on PM_GEN which linnerary increase together with used fuel. In most cases PM_GEN with value more than 5.7-5.9 activate DPF regeneration if filter is working properly. 

Circuit is based on ESP32 DevKit V1 30 pin version and 10 x LED bar. LEDs from 1 to 9 indicate filling of filter, LED 10 indicate BT status during startup and active regeneration during normal operation. 

There are two type of code:
One is based on MAC address - need to be updated in code, second is based on name, standard name is "V-LINK", if is different need to be updated.

Instruction is in polish language.

Project can't be copy and sell.

## Neccessary libraries
https://github.com/PowerBroker2/ELMduino

https://youtube.com/shorts/jTqBH3UMz80?feature=share
