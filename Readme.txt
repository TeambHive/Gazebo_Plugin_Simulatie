Instructions to make a plugin in gazebo and configure the gazebo libraries: http://gazebosim.org/tutorials?tut=plugins_hello_world&cat=write_plugin

ctrl + alt + t
sudo apt-get install libgazebo<YOUR_GAZEBO_VERSION_NUMBER>-dev    %example: "libgazebo7-dev"
cd <PATH_TO_THIS_DIRECTORY>
mkdir build
cd build
cmake ../
make
cd ..
export GAZEBO_PLUGIN_PATH=$<PATH_TO_THIS_DIRECTORY>/build:$GAZEBO_PLUGIN_PATH   %export GAZEBO_PLUGIN_PATH=$HOME/Gazebo_Plugin_Simulatie/build:$GAZEBO_PLUGIN_PATH
gzserver -u model_push.world

- in another terminal -
gzclient

ENJOY!
