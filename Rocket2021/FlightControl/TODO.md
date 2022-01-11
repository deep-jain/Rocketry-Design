# Things to add to FlightControl.ino
With most important things at the top

## (DONE) Ability to control flaps via DC motor
Maybe make a wrapper class? Like motor.set(ON), motor.set(OFF). Hopefully as simple as possible.

## (DONE) Detection/progression through the stages of flight
### The stages of flight are:
### 1) On Launchpad. (accel ~= 0)
### 2) Motor Burn. (accel >> 0)
### 3) Burnout. (accel << 0)
Almost all of our calculations will occur during stage 3, with most other operations occurring during stage 2.

## (DONE) Kalman filter partial implementation
This entails passing the filter one variable at a time and receiving a less noisy, more accurate version of that variable.

## (DONE) Kalman filter full implementation
This means passing the filter several variables and getting a more comprehensive model of the state of the vehicle.
