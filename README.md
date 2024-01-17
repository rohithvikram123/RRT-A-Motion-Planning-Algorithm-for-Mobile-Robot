# RRT-A-Motion-Planning-Algorithm-for-Mobile-Robot

-----------------------------------------------------------------------------------------------------------------
ENPM 661- Planning for Autonomous Robots (Spring 23)

RRT-A* Motion Planning Algorithm for Mobile Robot 
-----------------------------------------------------------------------------------------------------------------

Team Members:

Name: Rohith Vikram Saravanan 
UID: 119474198

Name: Poojan Desai
UID: 119455760

------------------------------------------------------------------------------------------------------------------


User Dependencies:
-> Python 3.9
-> IDE to run the program (I used VSCode)
-> Libraries : numpy, math, matplotlib.pyplot, time, 


Instructions to run: 
-> Open an IDE
-> Navigate to the folder where the .py file exists
-> Upon running the code, the user will be prompted to enter parameter such as x and y coordinates of start node, x and y coordinates of goal node
	For example,
	- X coordinate of start node: 10
	- Y coordinate of start node: 10
	- X coordinate of goal node: 900
	- y coordinate of goal node: 900
-> The code intergrats 2 obstacel space, therefore in the code

	FOR RRT-A* 
	- If user wants to test algorith in less dense environment uncomment lines from 175 to 195 and comment lines from 138 to 170
	- If you want to test the algorith in Dense environment uncomment lines from 138 to 170 and comment lines from 175 to 195
	FOR RRT 
	- If user wants to test algorith in less dense environment uncomment lines from 175 to 195 and comment lines from 138 to 170
	- If you want to test the algorith in Dense environment uncomment lines from 138 to 170 and comment lines from 175 to 195
	
