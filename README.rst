

I2C LM75A
################

Descripción
********

Este es un ejemplo para el sensor CJMCU-75 LM75A, via protocolo I2C, provisto por el sistema operativo Zephyr.

El slave address del LM75A es 0x48, donde A0, A1, y A2 están conectados a tierra (GND).

Pruebas del ejemplo
********************


Comando a correr::
    west build -p auto -b esp32


