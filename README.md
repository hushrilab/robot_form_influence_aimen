# Robot_Form_Influence_Aimen
Edited files of the julie/talos_public_ws/src/talos_moveit_config/scripts directory

### Run the expirements in simulation - Pick and place or Handover scenerio
1. First terminal: `cd robohub/julie/talos_public_ws`

2. First terminal: `source devel/setup.bash`
3. First terminal: `roslaunch talos_moveit_config custom_moveit_rviz.launch` #wait for 'You can start planning now' in terminal
4. Second terminal: `cd robohub/julie/talos_public_ws`
5. Second terminal: `source devel/setup.bash`
6. Second terminal: `rosrun talos_moveit_config talos_pickplace.py` or `rosrun talos_moveit_config talos_handover.py`
### Run the experiments IRL - Pick and place or Handover scenario
#### On Windows
1. Connect shimmer sensor, make firmware 'logging and streaming' and configure sensor

#### On Ubuntu
1. Connect to RoboHub wifi

2. First terminal: `hcitool scan` #get Shimmer3-D382 ID (00:06:66:BA:D3:82)

3. First terminal: `sudo rfcomm bind 0 00:06:66:BA:D3:82`

4. First terminal: `sudo chmod 666 /dev/rfcomm0`

5. First terminal: `ssh pal@talos-4c` (Password: pal)

6. First terminal: `cd aimen_scripts`

7. First terminal: `./fixit.sh` (Password: palroot)

8. Second terminal: `ssh pal@talos-4c` (Password: pal)

9. Second terminal: `./restart_deployer.sh; sleep 10; timeout 10 roslaunch talos_controller_configuration position_controllers.launch; roslaunch talos_controller_configuration default_controllers.launch`

10. Third terminal: `./mqtt_subscriber.py 127.0.0.1 /dev/video#`

11. Fourth terminal: `./mqtt_subscriber.py 127.0.0.1 /dev/video#`

12. First terminal: `python talos_pickplace.py` OR `python talos_handover.py`
