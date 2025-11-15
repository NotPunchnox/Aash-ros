# Quadruped Description - ROS2

Ce workspace ROS2 contient le package `quadruped_description` qui permet de visualiser un robot quadrupède dans RViz2.

## Prérequis

- ROS2 (Humble ou version supérieure)
- `xacro` pour le traitement des fichiers URDF
- `rviz2` pour la visualisation 3D

## Installation

### 1. Compiler le workspace

Depuis la racine du workspace (`/home/user/ros2_ws`) :

```bash
colcon build --packages-select quadruped_description
```

### 2. Sourcer l'environnement

```bash
source install/setup.bash
```

## Lancement

### Option 1 : Lancer le robot et RViz séparément

**Terminal 1** - Démarrer le robot state publisher :
```bash
ros2 launch quadruped_description quadruped.launch.py
```

**Terminal 2** - Démarrer RViz2 :
```bash
ros2 launch quadruped_description start_rviz.launch.py
```

### Option 2 : Tout lancer en une seule commande

```bash
ros2 launch quadruped_description quadruped.launch.py & ros2 launch quadruped_description start_rviz.launch.py
```

## Structure du package

```
quadruped_description/
├── launch/                  # Fichiers de lancement
│   ├── quadruped.launch.py        # Lance le robot state publisher
│   └── start_rviz.launch.py       # Lance RViz2 avec la config
├── quadruped/              # Fichiers URDF/Xacro du robot
├── rviz/                   # Configuration RViz2
└── package.xml             # Métadonnées du package
```

## Résolution de problèmes

### Erreur de compilation
Si vous rencontrez des erreurs lors du build :
```bash
# Nettoyer le workspace
rm -rf build/ install/ log/
# Reconstruire
colcon build --packages-select quadruped_description

# Lancer le service
source install/setup.bash
ros2 launch quadruped_description quadruped.launch.py

# Lancer la simulation
source install/setup.bash
ros2 launch quadruped_description start_rviz.launch.py

# Lancer le panel de commande (articulations)
source install/setup.bash
ros2 run joint_state_publisher_gui joint_state_publisher_gui
```

### Le robot ne s'affiche pas dans RViz
1. Vérifiez que les deux launches sont actifs
2. Dans RViz, ajoutez un display "RobotModel" si nécessaire
3. Vérifiez que le Fixed Frame est correctement configuré