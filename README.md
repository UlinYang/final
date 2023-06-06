# final
program setup: import all the library and cpp file into mbed and, put BB car on the rail, remember to change the IP address in main.cpp and .py file.

expected result: The BB car will follow the black rail, If the Laser ping detected that there is an obstacle in front of the car the car would turn to the right and detect if there is any obstacle at the right side of the road. If not, the car will go for that direction. If there is still an obstacle there, the car will turn to the left side of the road and do the same detection, if no available road again, the car will follow the original path back to the circle. 0b0111 or 0b1110: Decides whether the car branch right or left when it encounter a crossroad.
