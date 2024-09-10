An example for a `no_std` build of the dynamixel2 crate using a rp2040 microcontroller.

I had an **Adafruit KB2040** on hand so I used that, but any rp2040 board should work (changing the bsp in the `Cargo.toml`).

### Pins
I used an **MAX3078** RS485 transceiver.  

| Pin | Function   |
| --- |------------|
| 4 | RS485 TX     |
| 5 | RS485 RX     |
| 6 | RS485 Enable |