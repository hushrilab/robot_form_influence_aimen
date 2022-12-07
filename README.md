# Robot_Form_Influence_Aimen
Edited files of the julie/talos_public_ws/src/talos_moveit_config/scripts directory
### Run the experiments IRL - Pick and place or Handover scenario
#### On Windows
1. Connect shimmer sensor, make firmware 'logging and streaming' and configure sensor

#### On Ubuntu
1. Connect to RoboHub wifi

2. First terminal: `hcitool scan` #get Shimmer3-D382 ID (00:06:66:BA:D3:82)

3. First terminal: `sudo rfcomm 0 00:06:66:BA:D3:82`

4. First terminal: `sudo chmod 666 /dev/rfcomm0`

5. First terminal: `ssh pal@talos-4c` (Password: pal)

6. First terminal: `cd aimen_scripts`

7. First terminal: `./fixit.sh` (Password: palroot)

8. Second terminal: `ssh pal@talos-4c` (Password: pal)

9. Second terminal: `./restart_deployer.sh; sleep 10; timeout 10 roslaunch talos_controller_configuration position_controllers.launch; roslaunch talos_controller_configuration default_controllers.launch`

10. Third terminal: `./mqtt_subscriber.py 127.0.0.1 /dev/video#`

11. Fourth terminal: `./mqtt_subscriber.py 127.0.0.1 /dev/video#`

12. First terminal: `python talos_pickplace.py` OR `python talos_handover.py`
