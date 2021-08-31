# Projet robot autonome : DIMR-kuka
Répertoire du projet 3A robotique autonome (promotion 2019/2020) : DIMR KUKA

Ce répertoire contient l'ensemble des fichiers nécessaires à l'utilisation du projet ROS, ainsi que les différentes documentations du bras KUKA KR6 R900 et de son contrôleur KRC4.



To begin, turn the green on/off switch on the controller to the on position. The kuka will initialise for a few minutes.

Once initialized you will have access to the kuka control interface.

### Manual control

To control the robot in manual mode start by placing it in **T1** mode. To do this, turn the key on the control panel, press **T1** and then turn the key over.

<center>
<img src="/img/kukat1.jpg" width="50%"></img>
</center>

Once in T1 mode, unlock the emergency stops. Click *confirm all* on the control panel

<center>
<img src="/img/kukaconfirmall.jpg" width="50%"/>
</center>

then click on the *operator acknolegment* button on the back of the controller (without going through the kuka work area!)

<center>
<img src="/img/kukacontrolleur.png" width="50%"/>
</center>

At this point the kuka is ready to be controlled manually. Simply hold one of the white buttons on the back of the tablet with your finger and then move the various joints with the 6 buttons on the side of the tablet.

### Automatic control

To run a program on the kuka it is necessary to switch it to **automatic** mode.

⚠️ Automatic mode means that you do not need to hold down a button to make the kuka move as in **T1** mode, when using this mode always have an emergency stop button in your hands. To switch to automatic mode turn the key, click *aut* and then turn the key back.

<center>
<img src="/img/kukaaut.jpg" width="50%"></img>
</center>

Once in automatic mode, unlock the emergency stops, set the *operator safety* switch to 1 and click on *operator acknowledgement* (without going through the kuka work area!). Then start the controller by clicking on the I (or O) at the top of the tablet and then clicking on the I button

<center>
<img src="/img/kukaautlaunch.jpg" width="50%"></img>
</center>

Then you have to select a program. For example, for the program *kuka_eki* you just have to click on it and then press the *select* button. Finally, to start the program, press the *play* button on the left of the tablet.

<center>
<img src="/img/kukalaunch.jpg" width="50%"></img>
</center>

⚠️ The programme is now running independently. Please be aware of the kuka's movements and do not hesitate to press the emergency stop.


## ROS

### Installation

```bash
sudo apt-get install python-imaging-tk
sudo apt-get install python-tk
cd ~/catkin_ws/src/
git clone https://github.com/ros-industrial/kuka_experimental
git clone https://github.com/Bordeaux-INP/dimr-kuka
cd ..
rosdep install --from-paths src --ignore-src
catkin_make
source ~/.bashrc
```

If you also want to use the wsg50 effector mounted on the kuka :
```bash
cd ~/catkin_ws/src/
git clone https://github.com/nalt/wsg50-ros-pkg
cd ..
catkin_make
source ~/.bashrc
```

### Simulation

To start the DIMR-KUKA project in simulation, type the following command. An RSI simulator is started in the background to simulate the true response of the KRC4 via the RSI module.

```bash
roslaunch dimr_kuka dimr_kuka.launch sim:=true
```

### Real

Once the kuka has been put into emergency mode, connect the ethernet cable from the switch on the bottom of the kuka to your PC. Also connect the power supply to the end effector if you wish to use it. Once everything is connected you can raise the emergency stop on the kuka and run the *kuka_eki* program.

On your pc it is necessary to manually provide DHCP on the address `192.168.250.21/24`. Remember to disable the proxy server if you have one. The kuka ip address will be `192.168.250.20` and the effector will be `192.168.250.22`.

<center>
<img src="/img/kukadhcp.png" width="60%"></img>
</center>

You can check the connection of all items by running the command `nmap -sP 192.168.250.0/24` if everything is connected the output should be similar to this:

```bash
Starting Nmap 7.80 ( https://nmap.org ) at 2021-08-31 10:57 CEST
Nmap scan report for 192.168.250.20
Host is up (0.0016s latency).
Nmap scan report for morhost-iscsi2.iscsi.ipb.fr (192.168.250.21)
Host is up (0.00027s latency).
Nmap scan report for hangar-iscsi2a.iscsi.ipb.fr (192.168.250.22)
Host is up (0.0025s latency).
Nmap done: 256 IP addresses (3 hosts up) scanned in 16.12 seconds
```

You can then run the command for the kuka. You will then have access to rviz to control it.
```bash
roslaunch dimr_kuka dimr_kuka.launch sim:=false mode:=eki 
```

In order to do this it is first necessary to change the connection ip in the ros package. Open with your favorite editor the file: `~/catkin_ws/src/wsg50-ros-pkg/wsg_50_driver/launch/wsg_50_tcp.launch` and replace line 5 with the line below

```xml
	<param name="ip" type="string" value="192.168.250.22"/>
```

Once registered you can run the following command to take control of the effector. Then open a browser and go to `192.168.250.22`.

```bash
roslaunch wsg_50_driver wsg_50_tcp.launch     
```

### Ronoco

To use the kuka (and the effector) with ronoco you just have to [install the project](https://sdelpeuch.github.io/ronoco/) and then launch it with the following command.

```roslaunch
roslaunch ronoco manipulator.launch commander:=manipulator compliant_mode:=None end_effector:=wsg_50_driver/move
```

<center>
<img src="/img/kuka.gif"></img>
</center>





## TODO

* Documentation sur la calibration (vidéos à upload) ;
* Documentation sur la procédure de restauration du KRC4 en cas de pépin ;
* Investiguer les problèmes avec le module RSI (workspace KRC4 à setup via Kuka WorkVisual ?) ;
* Problèmes avec les points cartésiens sur le robot lors de l'utilisation réelle avec EKI ;
* Améliorer le connecteur X11 en faisant une vrai boîte (voir documentation start-up) ;
* Avoir KUKA au téléphone pour qu'ils viennent mettre à jour le KRC4 (et fournir une maj de RSI au passage)