# BotROS - The Joy of Painting

BotROS - The Joy of Painting is a project completed by 
Alex Overman <ayres036@umn.edu>, Sam Williams <will6673@umn.edu>, and Nadya Postolaki <posto018@umn.edu>. 
The purpose of this project was to create a robot that could draw any image that was given to it. 
BotROS can now 'draw' any black and white .bmp given to it.

# Setup
Before anything else, please run the following code from the root folder.

`chmod +x runThisFirst.sh`
`./runThisFirst.sh`
Due to some *weirdness*, we could not include turtlebot files in this repo.
Before continuing, please make sure you complete the following steps.

Open a terminal and run the following commands:

`cd /project_ws/src/`

`git clone https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git`

`git clone https://github.com/ROBOTIS-GIT/turtlebot3.git`

`git clone https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git`

`cd /project_ws && catkin_make`

`gedit ~/.bashrc`

This last command will open a text editor.

Please add this line at the bottom of the file.

`export TURTLEBOT3_MODEL=burger`

After you are done, make sure to reload the bashrc file.

`source ~/.bashrc`

# Running the code
In order to run BotROS, you'll want to run the following commands.

In one terminal:

`roscore`

In another:

`cd project_ws`

`catkin_make`

`source devel/setup.bash`

`roslaunch turtlebot3_fake turtlebot3_fake.launch`

This opens up the RViz simulation environment to see TurtleBot.

In order to use the required visualization setup, go to File-->Open Config and select the botros.rviz file supplied with the project.

In another terminal:

`cd project_ws`

`source devel/setup.bash`

`rosrun botros botros.py`

This runs the actual BotROS script.

# How to Use
In order to properly use BotROS, you'll need to add the .bmp image that
you'd like to have drawn in the images project_ws/src/botros/images folder.

You may also need to travel to this folder and type

`chmod +x FILENAME`

in order to allow BotROS to access your file.

Upon running BotROS, you'll be asked for the image name.
The following images are included in this build of BotROS:
- box
- cross
- bigcross
- circle
- spiral
- triangle
- smile

# Sketchify
In reference to the studies of simplifying images for BotROS to easily replicate, python script written by [HarshitRoys](https://github.com/harshitroy2605/imag-to-sketch/blob/master/1.py) was implemented and edited for this project. 

To run:
`python3 sketchify.py`

# TODO
- Add a prismatic joint that contains a pencil of some sort. Physically this would be easy enough, but digitally it has proven quite difficult.
- Add colors.
- Improve BotROS's path-finding.
