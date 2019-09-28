Tutorial for working with novelty detection and automatical extraction of novelty region. For any questions please contact me: shengzhi.wang@tum.de
------
------
There are two main folders. **Simulations** is the folder containing everthing about the novelty detection on simulated robot. **Real_Nao_Control** is consist of everthing on real robot. You can have a try on the simulated robot to get familiar with. Then switch to real robot. Before you get started, please have a check on the "Set up steps for mirror project.txt" and make sure you have installed all the desired stuffs. 

# Simulated_Nao
1. To get the training dataset, you have to follow the tutorial by [this site](https://github.com/matejhof/nao-mirror-self-recog), then save the recorded images into the folder "Simulated_Nao". 

2. Run "Novelty_Region_Detector.py". Inside the code you can adjust some hyperparameters. And you can also use different filters to filter the substraction of reconstruction image by original image. 

# Real_Nao_Control
1. Go to the folder: cd to Robot_Control/
2. Catkin build the workspace: catkin build
3. Source the setup.bash: source devel/setup.bash
4. Record training data: Before you start it, please put the robot before a mirror, mark this position for future experiments. Moveover, you should put a white blanket behind the robot as a background. If all are ready, then run the code: **rosrun nao_control Real_Recorder.py**
5. Save the images into folder "Nao_Mirror_Training_Dataset". 
6. Train the network for image reconstruction: **rosrun nao_control Decoder_Naojoint.py** (**Caution**: There are actually two parts inside the code. The first part above is to train the model, meanwhile the second part below is to test the model performance. So make sure which part you want and comment the other part. For testing performance, you should choose your own training dataset and put some post-it on the face by photoshop (or opencv). After that, the model will be saved into "Trained_Networks"
7. If you want to see the real time performance(without the arm touching), just run: **rosrun nao_control Real_Time.py**
8. (Optional) If you want to train the robot to touch the post-it, you need to record data firstly. Run the file: **rosrun nao_control Nao_Touching_recorder.py**. We use two buttons on the head to record data. The first front button is to record the centroid position of novelty region. The middle button is to record the joint angles of HeadYaw, HeadPitch, Lshoulderpitchs, Lshoulderroll, LElbowRoll. Firstly, you should be very careful to record the centroid position, because since you want to press the button, your hand will block the light, which leads to the distortion of novelty region. The way to solve this problem is to create a csv file, print the centroid position and record by hand. After recording the centroid point, move the left arm and head to move robot's hand to reach the post-it. Press the middle button, the head will automatically go back to neutral position. So is a period of recording. After recording the data, run: **rosrun nao_control Nao_Touching.py** to see the performance.
	










