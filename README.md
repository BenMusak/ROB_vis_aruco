# ROB_vis_aruco
Tracking of aruco markers using ROS 2 galactic and opencv contrib.

1) Check if the current default Camera on your PC or laptop is set as the one you want to use for tracking. 

    If it is not, either change the part in the code that says "cap = cv2.VideoCapture(0)" to "cap = cv2.VideoCapture(n)" where 'n' is any real number that represents which camera to pull images from.

2) Open a terminal

    Navigate to the PATH where the reposetory is:

    a)

    ``` bash
    cd PATH/
    ```

    b)

    run the python script

    ``` bash
    python3 robVis_pkg/robVis_node.py
    ```
    
# Dependensies      
    sudo apt-get install ros-galactic-turtle-tf2-py ros-galactic-tf2-tools ros-galactic-tf-transformations
    pip3 install transforms3d
    pip3 unintall opencv-python
    pip3 install opencv-contrib-python
    pip3 install glob
    pip3 install numpy
    pip3 install scipy
