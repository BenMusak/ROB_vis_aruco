# ROB_vis_aruco
Tracking of aruco markers

1) Clone the reposetory

2) Setup the correct video capture and calibrate camera

    a. Setup the correct video capture
    
    Check if the current default camera on your PC or laptop is set as the one you want to use for tracking. If it is not, either change the part in the code that says "cap = cv2.VideoCapture(0)" to "cap = cv2.VideoCapture(n)" where 'n' is any real number that represents which camera to pull images from.
    
    b. Start calibration

    Go into the file "robVis_node.py" and change 'calibration_d' to TRUE and follow instructions
    Use a calibration board (6*9)

3) Open a terminal

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
