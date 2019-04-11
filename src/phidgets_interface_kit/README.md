# phidgets_interface_kit

scarica i driver
entra dentro la cartella esegui
 ./configure
  make
  sudo make install

sempre nell cartella
 sudo cp udev/99-phidgets.rules /etc/udev/rules.d

This package provides support for using the Phidgets InterfaceKit.

## Published Topics

 * `analog_in` (phidgets_interface_kit/AnalogArray)
 * `digital_in` (phidgets_interface_kit/DigitalArray)
 * `digital_out` (phidgets_interface_kit/DigitalArray)

## Subscribed Topics

 * `cmd_digital_out` (phidgets_interface_kit/DigitalArray)
