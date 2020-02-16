# Projet robot autonome : DIMR-kuka
Répertoire du projet 3A robotique autonome (promotion 2019/2020) : DIMR KUKA

Ce répertoire contient l'ensemble des fichiers nécessaires à l'utilisation du projet ROS, ainsi que les différentes documentations du bras KUKA KR6 R900 et de son contrôleur KRC4.



## Mise en garde

Attention, le bras KUKA KR6 R900 n'est pas un cobot, vous pouvez très facilement blesser ou tuer quelqu'un. Faites très attention lors de son utilisation, idéalement une deuxième personne est assignée au bouton d'arrêt d'urgence externe lors des manipulations.



## Prérequis

Avant de construire le workspace, vous devez posséder sur votre machine linux :

* [Python 2.7](https://stackoverflow.com/a/59632121)
* [ROS Melodic **Desktop-Full**](https://wiki.ros.org/melodic/Installation/Ubuntu)
* [ROS Moveit](https://moveit.ros.org/install/)

Vérifiez que votre installation soit propre avant de continuer :

```
sudo apt update
sudo apt upgrade
sudo apt autoremove
```



## Installation

Installez Tkinter sur votre pc :
```
sudo apt-get install python-imaging-tk
sudo apt-get install python-tk
```


Créez un nouvel espace de travail ROS :

```
mkdir -p ~/dimr_kuka_ws/src
cd ~/dimr_kuka_ws/
catkin_make
```

Allez dans le dossier src et téléchargez kuka experimental pour obtenir le modèle du KUKA KR6 R900 :

```
cd ~/dimr_kuka_ws/src/
git clone https://github.com/ros-industrial/kuka_experimental
```

Téléchargez ensuite le dépôt actuel pour obtenir les packages nécessaire au projet :

```
git clone https://github.com/Copper-Bot/dimr-kuka.git
```

Remontez au dossier principal, vérifier les dépendances, et lancer une compilation :

```
cd ~/dimr_kuka_ws/
rosdep install --from-paths src --ignore-src
catkin_make
```

Editez votre bashrc pour sourcer automatiquement l'espace de travail :

```
echo "source ~/dimr_kuka_ws/devel/setup.bash" >> ~/.bashrc
```

>  Dans le cas où vous avez déjà sourcé plusieurs espaces de travail ROS, tapez plutôt :
>
> ```
> echo "source ~/dimr_kuka_ws/devel/setup.bash --extend" >> ~/.bashrc
> ```

Pour vérifier l'installation, redémarrer votre terminal, et tapez la commande suivante :

```
roslaunch dimr_kuka moveit_rviz_planning_execution.launch sim:=true
```



## Utilisation en simulation

Pour lancer le projet DIMR-KUKA en simulation, branchez si possible une manette, et tapez la commande suivante :

```
roslaunch dimr_kuka dimr_kuka.launch sim:=true
```

Un simulateur RSI est lancé en arrière plan pour simuler la vrai réponse du KRC4 via le module RSI.

## Utilisation réelle

Pour lancer le projet DIMR-KUKA sur le robot réel, le contrôleur KRC4 doit posséder :

* soit le module Kuka RSI (Robot Sensor Interface)
* soit le module Kuka EKI (Ethernet KRL Interface)

Dans l'état actuel d'avancement du projet, le module RSI produit un "boot failed" au démarrage, donc seul le module EKI est opérationnelle pour le moment.

Avec une très grande prudence sur les mouvements du bras, vous pouvez lancer le projet DIMR-KUKA et prendre le contrôle du bras manuellement (à l'aide d'une manette) avec la commande suivante :

```
roslaunch dimr_kuka dimr_kuka.launch sim:=false mode:=eki
```

Si vous voulez uniquement l'interface MOVEIT+RVIZ, tapez la commande suivante :

```
roslaunch dimr_kuka moveit_rviz_planning_execution.launch sim:=false mode:=eki
```

Enfin, si vous voulez uniquement tester la liaison du bras avec le module EKI, tapez la commande suivante :

```
roslaunch dimr_kuka test_EKI.launch
```



## TODO

* Documentation sur la calibration (vidéos à upload) ;
* Documentation sur la procédure de restauration du KRC4 en cas de pépin ;
* Investiguer les problèmes avec le module RSI (workspace KRC4 à setup via Kuka WorkVisual ?) ;
* Problèmes avec les points cartésiens sur le robot lors de l'utilisation réelle avec EKI ;
* Améliorer le connecteur X11 en faisant une vrai boîte (voir documentation start-up) ;
* Avoir KUKA au téléphone pour qu'ils viennent mettre à jour le KRC4 (et fournir une maj de RSI au passage)