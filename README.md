# Instalacja

Instrukcja instalacji zakłada wykorzystanie systemu Ubuntu 14.04 oraz ROS-a w wersji Indigo.
(Możliwe jest wykonanie na Ubuntu 16.04 i ROS Kinetic).

Instalacja pakietów z repozytorium (zakładam, że jest zainstalowany ROS)

``` sudo apt-get install -y ros-indigo-map-server ros-indigo-stage-ros ```

Przygotowanie przestrzeni roboczej:

```
source /opt/ros/indigo/setup.bash
cd ~
mkdir -p sipr/sipr_lab4_ws/src
cd sipr/sipr_lab4_ws/src
catkin_init_workspace
git clone https://bitbucket.org/rrgwut/swrm_ros.git
git clone https://bitbucket.org/maciej_przybylski/sipr_lab3_template.git
cd ..
catkin_make
```
