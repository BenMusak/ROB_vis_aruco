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
    
Error Handling
    
1) If an error occurs while the placing the ArUco marker in front of the camera, that most likely means the calibration file "cali.yml" is either corrupted or not found. Try re-calibrating the camera or placing the "cali.yml" file in different folders inside the workspace, as the problem could also occur if the program is unable to locate that file.
