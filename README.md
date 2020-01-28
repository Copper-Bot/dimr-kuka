# Projet robot autonome : DIMR-kuka
Répertoire du projet 3A robotique autonome (promotion 2019/2020) : DIMR KUKA

Ce répertoire contient l'ensemble des fichiers nécessaire à l'utilisation du projet ROS, ainsi que les différentes documentations du bras KUKA KR6 R900 et de son controlleur KRC4.



## Prérequis

Avant de construire le workspace, vous devez poseder sur votre machine linux :

* Python 2.7
* ROS Melodic



## Installation

Créer un nouvel espace de travail ROS :

```
mkdir -p ~/dimr_kuka_ws/src
cd ~/dimr_kuka_ws/
catkin_make
```

Allez dans le dossier src et télechargez kuka experimental pour obtenir le modèle du KUKA KR6 R900 :

```
cd ~/dimr_kuka_ws/src/
git clone https://github.com/ros-industrial/kuka_experimental
```

Téléchargez ensuite le répo actuel pour obtenir les packages nécessaire au projet :

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
roslaunch dimr-kuka test_kr6r900sixx.launch
```

