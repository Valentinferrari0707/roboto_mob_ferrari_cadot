CADOT FERRARI
README LOCAL_PLANNER


BASIC RUN (RVIZ + LOCAL + GLOBAL) 
roslaunch navigation_stage_student_tp navigation_tp.launch
 

1/ INITIALISATION 
- Des constantes
- Du node appelé "localPlanner_student"
- Déclaration publisher pour commander en vitesse
- Subscriptions à /odom et /scan
- Déclaration des services /move_to/singleGoal et /move_to/pathGoal 


2/DESCRIPTION GENERALE
Ce programme calcule et envoie des commandes en vitesse au robot pour atteindre chaque point d'une liste "pathPoses". Une fois la liste pathPoses construite il passe par une suite de traitements pour chaque point à atteindre:
- Calcul de l'angle et de la distance entre le point actuel et le prochain à atteindre
- Commande en vitesse tant jusqu'à une valeur proche du point à atteindre
Ce programme permet notamment au robot de s'arrêter lorsqu'il est trop proche des murs


3/CONSTRUCTION DE PATHPOSES: GOAL POINT METHOD

DESCRIPTION: Il est possible de commander le robot de manière à ce qu'il atteigne un unique goal point.
Pour cela il suffit d'appeler le service  /move_to/singleGoal avec en paramètre un vecteur [x,y,angle], angle = orientation finale
La "requête" est traitée dans la fonction goalService. Elle construit un objet de type PoseStamped complété avec les données du vecteur position finale, qu'on ajoutera dans la liste pathPoses.  

RUN:rosservice call /move_to/singleGoal '[1.0, 3.0, 3.0]

4/CONSTRUCTION DE PATHPOSES: PATH TO GOAL METHOD 

DESCRIPTION: Il est possible de construire une liste complète de goal points. On utilise pour cela le service /move_to/pathGoal. On lui envoie un objet de type Path constitué lui même d'autant d'objets PoseStamped complété avec les coordonnées de chaque goal point. 

RUN: rosrun local_planner_student testPathGenerator.py