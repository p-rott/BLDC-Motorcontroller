# BLDC-Motorcontroller
This is a very simple sketch to control two BLDC motors using the STEVAL-GMBL02V1, SimpleFOC library and Arduino IDE.   
Motors are iPower GM3506 with internal AS5048A encoders.   

I'm using Tourqe as MotionControlType and Voltage as TourqeControlType. This is because the STEVAL-GMBL02V1 sadly only offers a single shunt per motor connection, making Current control impossible (using SimpleFOC v2.2.2 at least).    

The controller is managed via I2C connection to a Raspberry Pi and receives relevant speed and steering data.

# Findings regarding the STEVAL-GMBL02V1   
I didn't find much information about the STEVAL-GMBL02V1 online, so i collected everything here in case anyone else might be interested in it.   
- The documentation notes "6V to 8.4VDC bus voltage range, which you can increase to 11V". It isn't mentioned again, and should be taken with a grain of salt, as the STSPIN233 driver has an operating voltage between 1.8 and 10V, with 11V being the absolute maximum rating for it.
- I2C can be wired via PB9/SDA and PB8/SCL, or if absolutely neccessary there is another I2C pin in the 10 pin header for the STLink connection. You will need to overwrite the SWD pins for this, so take care.
- BOOT0 is hardwired to GND, so you'll need a small probe to pull it high.
