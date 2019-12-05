CADOT FERRARI
README GLOBAL_PLANNER


BASIC RUN (RVIZ + LOCAL + GLOBAL) 
roslaunch navigation_stage_student_tp navigation_tp.launch


1/ INITIALISATION 
- Des constantes
- Du node appelé "ShortPathMng"
- Déclaration publisher dans /process_algo et //move_base_simple/goal
- Subscriptions à /map et /clicked
- Ecoute le services : /move_to/pathGoal 


2/DESCRIPTION GENERALE
Ce programme va permettre de gérer notre global_planner mais également faire le lien avec le local_planner. Concernant le global_planner, le robot reçoit une map prédéfinis que nous modifions en augmentant le inflate radius de chaque obstacles afin d'eviter tout choc entre le robot et les obstacles. Ensuite il est possible de choisir notre algorithme de path planning , WAVEFRONT est celui par défaut, mais nous devions implémenter celui de DIJKSTRA et A*. 
Suite à cela nous devions faire le lien entre le global_planner et le local_planner qui se fait via le service implémenté dans le local_planner qui nous permet ainsi de gérer les déplacements du robot et donc de construire le path pour qu'il puisse atteindre son point final.


3/CONSTRUCTION DES PROGRAMMES DES PATH PLANNING : 

L'élaboration de ces programmes a été fait à l'aide du cours et une simple traduction de ce dernier. Nous avons réutilisé certaines fonctions implémentées dans l'exemple donné celui du WAVEFRONT.Nous avons pu ainsi testé ces 2 programmes et pu observé le bon fonctionnements de ces derniers.
 

RUN: rosrun navigation_stage_student_tp ShortPathMng.py
