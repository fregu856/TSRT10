# TSRT10

******
Setup of LIDAR (scanse sweep, https://github.com/scanse/sweep-ros):
- Install the libsweep library from sweep-sdk (https://github.com/scanse/sweep-sdk): 
- - $ cd ~/TSRT10
- - $ git clone https://github.com/scanse/sweep-sdk
- - $ cd sweep-sdk/libsweep
- - $ mkdir -p build
- - $ cd build
- - $ cmake .. -DCMAKE_BUILD_TYPE=Release
- - $ cmake --build .
- - $ sudo cmake --build . --target install
- - $ sudo ldconfig

Create a catkin workspace:
- $ cd TSRT10
- $ mkdir catkin_ws
- $ cd catkin_ws
- $ mkdir src
- $ catkin_make
- Add "source ~/TSRT10/catkin_ws/devel/setup.bash" to the bottom of ~/.bashrc ($ sudo nano ~/.bashrc to edit, we do this for this line to be run everytime we open the terminal, otherwise we'd have to do it manually)  
- $ source ~/TSRT10/catkin_ws/devel/setup.bash
 
You should also build the package:
- $ cd ~/TSRT10_sim/catkin_ws  
- $ catkin_make 

