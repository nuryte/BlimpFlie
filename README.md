# Blimp Flie Framework


## Main functions

## Examples
* BicopterControl.ino
  - contains base code
  - flags can be set from ModTranseiver.py
* BicopterControlTransiever.ino
  - Same as BicopterControl but can send data back if enabled from ModTranseiver.py
* Transeiver.ino
  - contains the code in the antenna ESP32 which sends commands through interfacing with ModTranseiver.py
  - make sure that the serial monitor is not open when running ModTranseiver.py to prevent blocking on serial
  - the serial monitor will print out the MAC address if opened, which can be used as the MASTER_MAC in ModTranseiver.py if feedback is needed

* ModTranseiver.py
  - When sending flags through the send function, there are many flags which do different tasks on the arduino side
  - 0-10 do nothing
  - 11-16 send important values to save temporarily
  - 17 activate send back
  - 20 use 4 motor/servo commands
  - 21 use 6 dof commands
  - 22 use 4 motor/servo commands
  - 23 use 6 dof commands
