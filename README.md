# Cart Pole ROS package

This is a ROS package for pole on a cart. 

## Software Dependencies

- Python 2.7
- ROS Kinetic
- Gazeebo

## Outline of the project

This project is to create a model of a pole on a cart and control the acceleration of the cart to collect data from the odometer of the cart and the angle of the pole.

## How to Compile the repository

1. Change your woking directory to your catkin work space `$ cd ~/your_catkin_ws/src`
2. Clone this repository to the current folder `$ git clone -b kinetic-devel https://github.com/Rakshith-2905/smart_car_basic`
3. Make this to add this to the ROS packages to do that run `$cd ..` to go back to the catkin workspace and then run `$ catkin_make` to compile the new repository.

## How to run the project

1. Start a new terminal and change the working directory to your catkin work space.
2. Sour the setup.*sh file by the running the command `$ source devel/setup.bash` in the command line
3. Start a master node by running the command `$roscore`
2. On a new terminal follow step 1 and 2 and then run the command `$ roslaunch cart_pole mybot_world.launch` this will launch the gazebo environment
3. open another terminal follow step 1 and 2. Then change the working directory to `$ cd ~/your_catkin_ws/src/cart_pole/scripts
4. Run the python file using the command `$python train.py` this will give a force of 10 N and display the angle of the pole
