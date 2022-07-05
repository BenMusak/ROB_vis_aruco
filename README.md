# ROB_vis_aruco

Tracking of aruco markers

## Installation

1. In a terminal

    Clone the reposetory into a **PATH** of your choice

    ``` bash
    cd PATH/
    ```

    ``` bash
    git clone https://github.com/BenMusak/ROB_vis_aruco.git
    ```

## Dependensies

1. In a terminal

    Install packages:

    ``` bash
    sudo apt-get install ros-galactic-turtle-tf2-py ros-galactic-tf2-tools ros-galactic-tf-transformations
    pip3 install transforms3d
    pip3 unintall opencv-python
    pip3 install opencv-contrib-python
    pip3 install glob
    pip3 install numpy
    pip3 install scipy
    ```

## Setup the correct video capture and calibrate camera

1. Setup video capture

    Check if the current default camera on your PC or laptop is set as the one you want to use for tracking. If it is not. Go into the file "robVis_node.py". Change the part in the code that says "`cap = cv2.VideoCapture(0)`" to "`cap = cv2.VideoCapture(n)`" where 'n' is any real number that represents which camera to pull images from.

2. Start calibration mode

    a)

    Go into the file "robVis_node.py" and change "`calibration_d`" to `TRUE`

    b)

    Open a new terminal

    Navigate to the **PATH** where the reposetory is:

    ``` bash
    cd PATH/
    ```

    c)

    run the python script

    ``` bash
    python3 robVis_pkg/robVis_node.py
    ```

    follow instructions
    Use a calibration board (6*9)

3. Start tracking mode

    Go into the file "robVis_node.py" and change "`calibration_d`" to `FALSE`.

    The script will now use the newly created calibration file to track ArUco markers with. The calibration is only neccesary to perform once, or when a new camera is used. A window showing the tracking will appear if everything is working

## Error Handling

1. If an error occurs while the placing the ArUco marker in front of the camera, that most likely means the calibration file "cali.yml" is either corrupted or not found. Try re-calibrating the camera or placing the "cali.yml" file in different folders inside the workspace, as the problem could also occur if the program is unable to locate that file.

2. Try different integers "`n`" in "`cap = cv2.VideoCapture(n)`"
