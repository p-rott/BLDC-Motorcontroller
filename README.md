# b.ad-Motorcontroller
This is a very simple sketch to control two BLDC motors using the STEVAL-GMBL02V1, SimpleFOC library and Arduino IDE.   
Motors are iPower GM3506 with internal AS5048A encoders.   

I'm using Tourqe as MotionControlType and Voltage as TourqeControlType. This is because the STEVAL-GMBL02V1 sadly only offers a single shunt per motor connection, making Current control impossible (using SimpleFOC v2.2.2 at least).    

The controller is managed via I2C connection to a Raspberry Pi and receives relevant speed and steering data.
