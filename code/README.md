# Code Readme

The C files included are each in a folder that identify their purpose. 

car_PID --> Is the main code for controlling the steering, speed, and all other peripheral sensors that control the car. It also implements PID to proportionally control the speed and steering. 

remote --> This is the IR sensor code which is run on a seperate esp to remotely stop and start the car.

speed_monitor_control --> This code is used to read the speed of the car using the pulse counter via optical encoder, and also displays this speed on the LCD backpack. 
