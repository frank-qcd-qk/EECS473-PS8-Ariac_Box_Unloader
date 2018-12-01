echo "Primary Launch: Competition simulation and start competition!"
gnome-terminal -e 'roslaunch cxq41_ps8_box_unloader cxq41_ps8_main.launch'
sleep 20
echo "Secondary Launch: Conveyor action server and Kuka action server!"
gnome-terminal -e 'roslaunch cxq41_ps8_box_unloader cxq41_ps8_secondary.launch'
