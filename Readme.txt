Web app:
Note- all index.wsgi files contain metadata used for these various applications.
 
ROSE/webapp/CustomerUI/app.py: Implements the customer-side web application that allows customers to place orders for various drinks and posts them to the database.

ROSE/webapp/CustomerUI/templates/index.html: HTML file to display the homepage for the customer interface.

ROSE/webapp/CustomerUI/static/img: This directory contains the images used for the customer interface of the web application.

ROSE/webapp/CustomerUI/static/js: This directory contains the javascript files that were present in the template that we used to create the customer interface.

ROSE/webapp/CustomerUI/static/css: This directory contains the css files that were present in the template that we used to create the customer interface. 

ROSE/webapp/ROSE_ManagerSide/app.py: Implements the manager-side web application, displaying the currently placed orders from the database for the manager’s review.

ROSE/webapp/ROSE_ManagerSide/templates/managerHome.html: HTML file to display the homepage for the manager interface.

ROSE/webapp/ROSE_ManagerSide/templates/override.html: HTML file to display the manager robot override screen.

ROSE/webapp/app.py: Implements the robot override which lets the user control the robot with keys for movement, rotation and speed.

ROSE/webapp/Date_and_Voltage.py: Implemented to push in a time stamp into the database and another method for getting/reading in voltage.Note that this was a test file for these features, but were not used in the Demos due to time constraint and the need to complete our main goal.

ROSE/webapp/templates/index.html: Sets up the functionality and layout of the user interface. Also using the css file in the same folder.

ROSE/webapp/templates/style.css: Sets up the definition of the layouts which are used in index.html which is located in the same folder.


Database:

ROSE/server/HelloWorldPyMongo.py: Test script for storing and retrieving a “hello world” message from a MongoDB database using python.

ROSE/server/conntest.py: Test script for connecting to a remote server’s MongoDB database using python.

ROSE/server/mongodb/helloworld.cpp: Test script for storing and retrieving a “hello world” message from a MongoDB database using C++.

ROSE/server/mongodb/practicedb1.cpp: Script used for practicing MongoDB coding in C++.

ROSE/server/mongodb/simulations/armcoord.cpp: Simulation for storing the coordinates of the arm to the database.

ROSE/server/mongodb/simulations/ locxvel.cpp: Simulation for push the encoder values to the database.

ROSE/robot/dbconn.cpp: Used in demo 1. contains the dbconn class, which is used to connect ROSE with the database, modified for reading control inputs for the robot

ROSE/robot/dbconn.h: Used in demo 1. Method definitions for the methods in the dbconn class of demo 1.

ROSE/robot/dbconntwo.h: Used in demo 2. contains the dbconn class, which is used to connect ROSE with the database, modified for reading and storing order information sent from the web app. Also contains a simulation fuction called dbsim

ROSE/robot/dbconntwo.cpp: Used in demo 2. Method definitions for the methods in the class “dbconn” of demo 2. Includes: recv_data(db), send_data(db), clear_data(), init_rose_status(db), and db.update()

ROSE/robot/dbsim.cpp: Contains the definition of the “dbsim” function defined in dbconntwo.h and a main function to run a simulation of reading orders from the database using the dbconn class defined in dbconntwo.h.

Robot:
Arduino Code:
code/Rose_Demo_2/arduino/DEV0001-base/DEV0001-base.ino : This controls the base of the robot. Helps with movement by sending values to motors.

code/Rose_Demo_2/arduino/DEV1001-arm_upper/DEV1001-arm_upper.ino : This controls the upper 5 joints of the arm on the robot. Sends values to motors.

code/Rose_Demo_2/arduino/DEV1002-arm_lower/DEV1002-arm_lower.ino : This controls the last joint on the arm, which basically allows the arm to rotate.

code/Rose_Demo_2/arduino/DEV2001-screen/DEV2001-screen.ino : This displays a text message on the robot.

C Code:
code/Rose_Demo_2/robot/serial.c : Allows you to communicate with arduinos using C++ code.

C++ Code:
code/Rose_Demo_2/robot/actions.cpp : Creates a linked list of actions to go from one action to another. 

code/Rose_Demo_2/robot/astar.cpp : Maze solving algorithm to get to a localized location in the shortest time.

code/Rose_Demo_2/robot/chili_landmarks.cpp : Recognizes chili-tags and places them in a globally recognized array.

code/Rose_Demo_2/robot/dbconntwo.cpp : Sends and receives messages from the database . Allows communication with the webapp.

code/Rose_Demo_2/robot/draw.cpp: Lets you draw shapes 

code/Rose_Demo_2/robot/heap.cpp : Creates a heap for astar

code/Rose_Demo_2/robot/highgui.cpp : Has lots of openCV functions that are necessary for our object detection.

code/Rose_Demo_2/robot/mathfun.cpp : Has some useful math functions that are used for various calculations such as IK and movement stuff as well as particle filter.

code/Rose_Demo_2/robot/pfilter.cpp : Creates a particle filter for localization using chilitags around the room.

code/Rose_Demo_2/robot/Rose.cpp: Makes an instance of the robot and contains all the robot functions. It makes connection with the robot and takes all the data sent from robot (arduino, encoders and potentiometers) and parses the info.

code/Rose_Demo_2/robot/runrobot.cpp: Runs the robot

code/Rose_Demo_2/robot/sim_landmark.cpp: Creates the landmarks used in the particle filter.
code/Rose_Demo_2/robot/sim_map.cpp: Creates a map using the landmarks for the particle filter.

code/Rose_Demo_2/robot/sim_robot.cpp: Simulate a robot for localization using a particle filter.

code/Rose_Demo_2/robot/arm/arm.cpp : Arm test file

code/Rose_Demo_2/robot/arm/test.cpp : Unit test to test the arm. Moves the arm to various positions

code/Rose_Demo_2/robot/baserobot/baserobot.cpp : Unit test for moving the robot using PID.

code/Rose_Demo_2/robot/chilitags/chili_landmarks.cpp : Tests whether the robot recognizes landmark and calculates position or not.

code/Rose_Demo_2/robot/chilitags/test.cpp : Tests detection of various chilitags.

code/Rose_Demo_2/robot/serial/seric.c : Tests whether you are able to communicate between the jetson tk1 and the arduinos on the robot.


How to run Unit Tests:
Database:
Unit tests can be compiled for the database connection of the robot using the following on the command line:

c++ --std=c++11 dbconntwo.cpp -o db $(pkg-config --cflags --libs libmongocxx)

Where “db” is the executable file created.

*NOTE*, the main function of dbconntwo.cpp is located at the very bottom of the script and must be uncommented so that dbconntwo.cpp may be compiled.

To run simulations under the “server” folder for the database, just use:

c++ --std=c++11 <file name>.cpp -o db $(pkg-config --cflags --libs libmongocxx)


Webapp:

CustomerUI: In order to run the customer interface, would we first have to be in the CustomerUI directory, located in the webapp folder. Then, inside the terminal, type the following on the command line:
python app.py
app.py is the file which implements all of the code in the CustomerUI folder. This includes creation of the aesthetic aspect of the customer side of the web application and its functionality.

ROSE_ManagerSide: In order to run the manager interface we would first have to be in the ROSE_ManagerSide directory, located in the webapp folder. Then, inside the terminal, type the same thing on the command line as mentioned in the CustomerUI part.
Here app.py implements all aspects of the ROSE_ManagerSide folder. These aspects include pulling orders from the database and displaying them for the user.

Override: In order to run the override for the robot, we would first have to be in the webapp directory, located in the ROSE folder.. Then, as stated in the previous two parts. We would run the same command on the command line.
Here app.py implements the direction, rotation and the speed override for the robot. 

Robot:

The ROSE project requires the following to run:

Linux --> Ubuntu 15.10
Python 2.7 (apt-get -- should already have with ubuntu)
Festival (apt-get)
SDL & SDL2 (apt-get)
SDL_TTF (apt-get)
Armadillo (apt-get)
MongoDB (apt-get)

OpenCV 3.0 (from SOURCE - NOT from apt-get) get it from itseez on github
OpenCV_contrib (from SOURCE - install with opencv) again from itseez
Chilitags (from SOURCE)
Arduino (1.6.7 or later) (from SOURCE - NOT from apt-get)
Adafruit motorshieldv2 libraries for arduino
MongoDB C / C++ Driver

After all dependencies are satisfied, the robot application can be run by running the makefile in the robot folder, and then running the executable:

make
./rose

The particle filter, although not completely integrated yet, can be run by running the makfile in the particle/particle_current folder, and then running the executable:

make
./sim

The arm test application can be run by going into the robot/arm directory and running the makefile, and then running the executable:
make
./arm

