# Quadruped Description - ROS2

Ce workspace ROS2 contient le package `quadruped_description` qui permet de visualiser et simuler un robot quadrupède dans RViz2 et Gazebo.

## Caractéristiques

- **Modèle URDF complet** avec visuals, collisions et inertias
- **12 joints articulés** (3 par patte : hip, knee, ankle)
- **Support ros2_control** avec interfaces position/velocity
- **Simulation Gazebo** avec plugin gazebo_ros2_control
- **Visualisation RViz2** avec configuration pré-définie
- **Dummy root link** pour résoudre les warnings KDL

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

### Option 1 : Visualisation simple dans RViz

**Terminal 1** - Démarrer le robot state publisher :
```bash
ros2 launch quadruped_description quadruped.launch.py
```

**Terminal 2** - Démarrer RViz2 :
```bash
ros2 launch quadruped_description start_rviz.launch.py
```

### Option 2 : Simulation Gazebo avec ros2_control

**⚠️ Note pour environnement SSH/Headless :** Si vous n'avez pas d'affichage graphique, utilisez l'option `gui:=false`

**Lancer la simulation Gazebo avec GUI** :
```bash
ros2 launch quadruped_description gazebo.launch.py
```

**Lancer la simulation Gazebo sans GUI (mode headless)** :
```bash
ros2 launch quadruped_description gazebo.launch.py gui:=false
```

Cela démarre :
- Gazebo avec le robot quadrupède (serveur seulement si gui:=false)
- Les contrôleurs ros2_control (joint_state_broadcaster, quadruped_controller)
- La publication des états des joints

**Contrôler les joints** :
```bash
# Publier une commande de position pour un joint
ros2 topic pub /quadruped_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory "
joint_names: ['front_left_hip', 'front_left_knee', 'front_left_ankle']
points:
  - positions: [0.5, 0.3, -0.3]
    time_from_start:
      sec: 1
      nanosec: 0
"
```

**Visualiser les joints actifs** :
```bash
ros2 control list_hardware_interfaces
```

### Option 3 : Tout lancer en une seule commande (RViz uniquement)

```bash
ros2 launch quadruped_description quadruped.launch.py & ros2 launch quadruped_description start_rviz.launch.py
```

## Structure du package

```
quadruped_description/
├── launch/                      # Fichiers de lancement
│   ├── quadruped.launch.py           # Lance le robot state publisher
│   ├── start_rviz.launch.py          # Lance RViz2 avec la config
│   └── gazebo.launch.py              # Lance Gazebo avec ros2_control
├── config/                      # Configuration
│   └── quadruped_control.yaml        # Configuration ros2_control
├── quadruped/                   # Fichiers URDF/Xacro du robot
│   ├── robot.urdf                    # Description URDF avec ros2_control
│   └── assets/                       # Meshes STL
├── rviz/                        # Configuration RViz2
└── package.xml                  # Métadonnées du package
```

## Résolution de problèmes

### Erreur de compilation
Si vous rencontrez des erreurs lors du build :
```bash
# Nettoyer le workspace
rm -rf build/ install/ log/
# Reconstruire
colcon build --packages-select quadruped_description
```

### Gazebo : "Address already in use"
Si vous obtenez l'erreur "Unable to start server[bind: Address already in use]" :
```bash
# Tuer tous les processus Gazebo
killall -9 gzserver gzclient 2>/dev/null; sleep 1; echo "Gazebo arrêté"
# Attendre 2 secondes
sleep 2
# Relancer
ros2 launch quadruped_description gazebo.launch.py
```

### Gazebo : Problème d'affichage (headless/sans GUI)
Si vous êtes en SSH ou sans serveur X :
```bash
# Lancer Gazebo en mode headless (sans GUI)
DISPLAY= ros2 launch quadruped_description gazebo.launch.py gui:=false
```

### Gazebo : Plugin ros2_control non trouvé
Si le controller_manager ne démarre pas :
```bash
# Vérifier l'installation des packages
sudo apt install ros-humble-gazebo-ros2-control \
                 ros-humble-ros2-control \
                 ros-humble-ros2-controllers
                 
# Vérifier que le plugin existe
ls /opt/ros/humble/lib/libgazebo_ros2_control.so
```

### Le robot ne s'affiche pas dans RViz
1. Vérifiez que les deux launches sont actifs
2. Dans RViz, ajoutez un display "RobotModel" si nécessaire
3. Vérifiez que le Fixed Frame est correctement configuré (utilisez `dummy_root` ou `body`)

### Contrôler manuellement les joints
Pour tester les joints sans Gazebo :
```bash
# Terminal 1 : Lancer le robot
ros2 launch quadruped_description quadruped.launch.py

# Terminal 2 : Lancer RViz
ros2 launch quadruped_description start_rviz.launch.py

# Terminal 3 : GUI pour contrôler les joints
ros2 run joint_state_publisher_gui joint_state_publisher_gui
```