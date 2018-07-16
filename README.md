# Prepoznavanje prometnih znakova                                                                                                                                                                               
Autor: Sandi Šegota
JMBAG: 0069064326
DSS Računarstva
  
## Dependancies

* Ubuntu 16.04 - Host PC
* ROS Kinetic 
	* OpenCV3
	* roscpp
	* catkin
	* std_msgs
	* genmsg
	* cv_bridge
* NodeJs v8.11.2
* Target PC: linux sa ROS kinetic. Predlažem preuzimanje gotovog imagea sa: https://downloads.ubiquityrobotics.com/pi.html


## Upute za instalaciju

1. Instalirati sve potrebne pakete
2. Kreirati direktorij `~/catkin_ws`  
3. Kopirati traffic_sign_recognition direktorij u `catkin_ws/src/` direktorij
4. Podesiti `$(ROS_ROOT)/rostoolchain.cmake` sa opcijama
```
set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_C_COMPILER arm-linux-gnueabihf-gcc)
set(CMAKE_CXX_COMPILER arm-linux-gnueabihf-g++)
set(CMAKE_FIND_ROOT_PATH /path/to/cross/compile/build/environment)
```
5. U `catkin_ws` direktoriju pokrenuti build naredbom `catkin_make`
6. Nakon izvršavanja builda iz direktorija `catkin_ws/devel/lib/tsr` kopirati izvršne datoteke na TARGET sustav
7. Kopirati `/tsr/iot` na target device

## Upute za pokretanje

Program bi se trebao pokrenuti automatski pri paljenju Raspberry Pi uređaja. Ukoliko se ne pokrene potrebno je slijediti slijedeće upute:

1. Spojiti kameru sa Raspberry Pi
2. Pokrenuti ROS Master naredbom `roscore`
3. `cd catkin_ws`
4. `source devel/setup.bash`
5. Pokrenuti program naredbom `rosrun traffic_sign_recognition try.launch`
6. Ako je potrebna IoT komunikacija pokrenuti Node.js skriptu sa `node src/traffic_sign_recognition/iot/scripts/listener.js`

