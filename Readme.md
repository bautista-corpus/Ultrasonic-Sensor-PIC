# Sensor de distancia
## Sensor Ultrasonico + PIC + UART

Este programa implementa:

- Una forma de reportar el funcionamiento del PIC.
    - *Se utiliza **U1TXInterrupt*** 

- La medicion de distancia utilizando el sensor ultrasonico SR-04

    - *Se utiliza **Timer2** junto **T2Interrupt** con el modo **GATE** configurado

### Materiales
- dsPIC30F4013
- Modulo Bluetooth HC-06
- Bluetooth terminal (App de Android)
- Sensor Ultrasonico SR-04

### Información adicional

- Compilador: xc16
- IDE: MPLAB X 5.25

