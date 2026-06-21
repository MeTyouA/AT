gnome-terminal -- bash -c "source install/setup.bash; ros2 launch rmu_gazebo_simulator bringup_sim.launch.py ; exec bash"

gnome-terminal -- bash -c "source install/setup.bash; ros2 launch pb2025_nav_bringup rm_navigation_simulation_launch.py ; exec bash" 