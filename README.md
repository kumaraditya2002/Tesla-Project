# Tesla-Project:Self Balancing Bot

# Materials Required

1. Microcontroller(Arduino UNO/Nano)-1

2. Dc geared Motors-2

3. Motor Driver(L298N)-1

4. MPU-6050 6 Axis DOF Gyroscope

5. Wheels-2

6. Chassis

7. 9volt battery-2

8. Breadboard 

# Things you will learn
PID Control

Using MPU-6050 for measuring angles

I2C Communication protocol

Implementing complementry filters for getting accurate results from sensors
# Description
# 1. How was it done :
It will be prevented from falling by giving acceleration
to the wheels according to its inclination from the
vertical. If the bot gets tilts by an angle, than in the
frame of the wheels, the centre of mass of the bot will
experience a pseudo force which will apply a torque
opposite to the direction of tilt.
# 2. PID Control System:
The control algorithm that was used to maintain it
balance on the autonomous self-balancing two wheel
robot was the PID controller. The proportional,
integral, and derivative (PID) controller, is well known
as a three term controller.
The input to the controller is the error from the
system. The Kp, Ki, and Kd are referred as the
proportional, integral, and derivative constants (the
three terms get multiplied by these constants
respectively). The closed loop control system is also referred to as a negative feedback system. The basic
idea of a negative feedback system is that it measures
the process output y from a sensor. The measured
process output gets subtracted from the reference setpoint value to produce an error. The error is then fed
into the PID controller, where the error gets managed
in three ways. The error will be used on the PID
controller to execute the proportional term, integral
term for reduction of steady state errors, and the
derivative term to handle overshoots. After the PID
algorithm processes the error, the controller produces
a control signal u. The PID control signal then gets fed
into the process under control.
The process under PID control is the two wheeled
robot. The PID control signal will try to drive the
process to the desired reference setpoint value. In the
case of the two wheel robot, the desired set-point
value is the zero degree vertical position. The PID
control algorithm can be modelled in a mathematical
representation.
PID is used to calculate the ‘correction’ term :
Correction = Kp*error + Ki* ∫error + Kd*
d/dt(error);
Kp , Ki and Kd are constants which are set
experimentally.
# 3.I2C Interface:
Inter-Integrated Circuit (I²C pronounced I- Squared- C)
is a 2 wire serial bus typically used to communicate
with sensors and other small components.
The two lines of the I2C bus are SDA (Data) and SLC
(clock) which can be run in parallel to communicate
with several devices at once. I2C allows up to 112
"slave" (such as a sensor) devices to be controlled by a
single "master" (such as Arduino, in our case). Each
slave device on the bus must have it's own unique
address so the master can communicate directly with
the intended device. These addresses are typically hardcoded into the slave device, but often allow it to be
changed by simply pulling one of the pins of the sensor
high or low. This allows more than one of the same
device to be on the same bus without conflicting
addresses.
I2C is often referred to as TWI or 2-wire-serial.

I2C Tutorial : I2C basic command sequence.

 1. Send the START bit (S).

 2. Send the slave address (ADDR).

 3. Send the Read(R)-1 / Write(W)-0 bit.

 4. Wait for/Send an acknowledge bit (A).

 5. Send/Receive the data byte (8 bits) (DATA).

 6. Expect/Send acknowledge bit (A).

 7. Send the STOP bit (P).
