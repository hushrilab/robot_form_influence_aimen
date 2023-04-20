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
1. Connect shimmer sensor, with Consensys, make firmware 'logging and streaming' and configure sensor, then undock. This step is necessary to align the timestamp of the sensor (otherwise it is not properly timestamped).

#### On Labwork6
1. Plug 2 cameras into computer using USB-3 cords and ports

2. Plug bluetooth dongle into Talos
3. `v4l2-ctl --list-devices` #will give multiple /dev/video# options to find the right ones use next command
4. `v4l2-ctl -d /dev/video# --list-formats-ext` #go through all options looking for 'YUYV' for both cameras
5. First terminal: `./mqtt_subscriber.py mqtt.eclipseprojects.io /dev/video#`
6. Second terminal: `./mqtt_subscriber.py mqtt.eclipseprojects.io /dev/video##`
7. Lift Talos 1ft off the ground, flip the switch, wait a couple seconds and press top button.
8. Third terminal: `ssh pal@talos-4c` (Password: pal)
9. Third terminal: `cd aimen_scripts`
10. Third terminal: `./fixit.sh` (Password: palroot)
11. Fourth terminal: `ssh pal@talos-4c` (Password: pal)
12. Fourth terminal: `./restart_deployer.sh; sleep 10; timeout 10 roslaunch talos_controller_configuration position_controllers.launch; roslaunch talos_controller_configuration default_controllers.launch`
13. Fifth terminal: `ssh pal@talos-4c` (Password: pal)
14. Fifth terminal:`sudo bash` (Password: palroot)
15. Fifth terminal: `sudo apt intall bluez`
16. Fifth terminal: `bluetoothctl`
17. Fifth terminal: `power on`
18. Fifth terminal: `agent on`
19. Firth terminal: `scan on` and verify that`00:06:66:BA:D3:82` is detected
20. Fifth terminal: `pair 00:06:66:BA:D3:82` (Password: 1234)
21. Fifth terminal: Ctrl+D
22. Fifth terminal: `rfcomm bind 0 00:06:66:BA:D3:82` #to check if device binded use `rfcomm`, and after running the python scripts each time `rfcomm release 0` and repeat step 22
23. Third terminal:`python talos_pickplace.py` OR `python talos_handover.py` (Once initialized i.e Talos is in starting position then place Talos on the floor and make sure the crane rope is loose)
