# HumanMachineCollaboration-3Dvision
The main task of the project was to implement and realise human-machine collaboration using a Kinect 3D camera.

3D cameras can be used as NUI (natural user interface). NUI means that the person uses its natural environment to control the application, for example: using its own body as a controller and thus controlling is a direct and intuitive environment. Kinect version 1 was used for this project and for programming the Kinect for Windows Software Development Kit was used.
Colour camera provides RGB colour stream. The Kinect depth sensor consists of an infrared emitter and an infrared depth sensor. The two images (picture 1) obtained by the depth sensor are then composed using the Stereo Triangulation method, which is an analytical algorithm for calculating the depth information from points in an image. The sensor then provides each pixel of a depth image with a number that is the distance of an object in front of the sensor in millimetres.

<div align='center'>
<img src="https://user-images.githubusercontent.com/81230042/119347399-62e01880-bc93-11eb-9c58-1cf49849e2bd.png" />
	
Picture 1: 	Depth sensing area 
</div> 

Skeleton tracking is the most crucial function of Kinect in this matter. It detects a human body and provides the position information about its joints (picture 2).

<div align='center'>
<img src="https://user-images.githubusercontent.com/81230042/119347513-8905b880-bc93-11eb-9ef4-91109425df91.png" />
	
Picture 2: 	Tracked joints by Kinect
</div> 

For body recognition, Microsoft used an extensive training database to estimate different body parts in different poses, body shapes, clothing, and more. A deep randomised decision tree was used as a learning algorithm, where they achieved accuracy and speed for their purpose. To put it differently, it compares learnt parts of a body with those it sees. This process is done in three steps.
In the first step, Kinect identifies an object of the human body. In the second step, the sensor starts comparing trained segments of a body with received depth image of the human body in front of a sensor. This comparison is performed inside a sensor with very high processing speed. Finally, when all parts are recognised, it places the points representing joints and extracts information about their position in space.

The goal of this project was to create the Natural User Interface (NUI) application for controlling the robotic arm through the Kinect 3D camera. As a result, the robot mimics the movements of the right hand and responds to gestures from the left hand. The camera communicates with the robotic arm via a TCP/IP protocol. 
The testing in the simulation was run on multiple robotic arms in RobotStudio, and for the testing on the real robotic arm, IRB 120 by ABB company was used.
This technology could provide natural cooperation between humans and robots for a broad range of potential future applications, such as moving heavy objects in production, controlling a pyrotechnic robot, space missions and many others.


<div align='center'>
<img src="https://user-images.githubusercontent.com/81230042/119347581-9d49b580-bc93-11eb-8cea-6686506778e2.png" />
	
Picture 3: 	Placement of the sensor and the robotic arm
</div>

<div align='center'>
<img src="https://user-images.githubusercontent.com/81230042/119347625-ab97d180-bc93-11eb-9c67-9b989ba52d04.png" />
	
Picture 4: 	GUI of the application
</div>
 
