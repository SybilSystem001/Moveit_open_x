# This package was made for Ros2 Jazzy and the template was designed for the open manipulator x
#to install the package clone the repository on your workspace folder example "/${workspace}/src"
```bash
git clone -b jazzy https://github.com/SybilSystem001/Moveit_open_x.git
```
#to build only this package run the following command
```bash
colcon build --packages-select hello_moveit
```
#runs the pick and place program, you can edit it according to your needs
```bash
ros2 run hello_moveit hello_moveit
```
