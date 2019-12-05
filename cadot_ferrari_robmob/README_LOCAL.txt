CADOT FERRARI
README LOCAL_PLANNER


BASIC RUN (RVIZ + LOCAL + GLOBAL) 
roslaunch navigation_stage_student_tp navigation_tp.launch
 

1/ INITIALISATION 
- Des constantes
- Du node appel� "localPlanner_student"
- D�claration publisher pour commander en vitesse
- Subscriptions � /odom et /scan
- D�claration des services /move_to/singleGoal et /move_to/pathGoal 


2/DESCRIPTION GENERALE
Ce programme calcule et envoie des commandes en vitesse au robot pour atteindre chaque point d'une liste "pathPoses". Une fois la liste pathPoses construite il passe par une suite de traitements pour chaque point � atteindre:
- Calcul de l'angle et de la distance entre le point actuel et le prochain � atteindre
- Commande en vitesse tant jusqu'� une valeur proche du point � atteindre
Ce programme permet notamment au robot de s'arr�ter lorsqu'il est trop proche des murs


3/CONSTRUCTION DE PATHPOSES: GOAL POINT METHOD

DESCRIPTION: Il est possible de commander le robot de mani�re � ce qu'il atteigne un unique goal point.
Pour cela il suffit d'appeler le service  /move_to/singleGoal avec en param�tre un vecteur [x,y,angle], angle = orientation finale
La "requ�te" est trait�e dans la fonction goalService. Elle construit un objet de type PoseStamped compl�t� avec les donn�es du vecteur position finale, qu'on ajoutera dans la liste pathPoses.  

RUN:rosservice call /move_to/singleGoal '[1.0, 3.0, 3.0]

4/CONSTRUCTION DE PATHPOSES: PATH TO GOAL METHOD 

DESCRIPTION: Il est possible de construire une liste compl�te de goal points. On utilise pour cela le service /move_to/pathGoal. On lui envoie un objet de type Path constitu� lui m�me d'autant d'objets PoseStamped compl�t� avec les coordonn�es de chaque goal point. 

RUN: rosrun local_planner_student testPathGenerator.py