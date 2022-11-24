# Robot_Form_Influence_Aimen
Edited files of the julie/talos_public_ws/src/talos_moveit_config/scripts directory
### Run the experiments IRL - Pick and place or Handover scenario
1. Connect to RoboHub wifi

2. First terminal: `ssh pal@talos-4c` (Password: pal)

3. First terminal: `cd aimen_scripts`

4. First terminal: `./fixit.sh` (Password: palroot)

5. Second terminal: `ssh pal@talos-4c` (Password: pal)

6. Second terminal: `./restart_deployer.sh; sleep 10; timeout 10 roslaunch talos_controller_configuration position_controllers.launch; roslaunch talos_controller_configuration default_controllers.launch`

7. Third terminal: `./mqtt_subscriber.py 127.0.0.1 /dev/video#`

8. Fourth terminal: `./mqtt_subscriber.py 127.0.0.1 /dev/video#`

9. First terminal: `python talos_pickplace.py` OR `python talos_handover.py`
