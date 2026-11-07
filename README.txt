ARM-E is our mecanum-wheel robot with a robotic arm on top.
We went with three 3.7V Li-ion cells in series, giving about 11.1V.
That 11.1V gets split into different rails for different parts.
One LM2596 buck converter brings it down to 5V for the brain.
The brain is an ESP32-S3 N16R8.
A second step-down gives 6V for the servo side.
The servos hang off a PCA9685 16-channel I2C driver.
we initially planned for three MG996R servos and three MG90S servos.
For driving, there are two TB6612FNG motor drivers.
They run four 12V 130RPM 25GA gear motors, one per mecanum wheel.
Made a browser page you want to control each servo’s angle individually.
MG996R strips and we need to get it replaced asap
WE got MG995 as the other out of stock
The battery drains too fast and, BMS turns off the circuit too much and too fast
Works with direct battery supply for limited time

