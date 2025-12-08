Projet MOGPL 2025 – La balade du robot
--------------------------------------

Auteurs : Yanis Labiadh & David Gu
Année : 2025-2026
Cours : Modélisation et Optimisation des Graphes et PL (MOGPL)


DESCRIPTION DU PROGRAMME
------------------------

Le fichier projet.py contient :

1. Génération des intersections valides (prise en compte du diamètre du robot)
   build_valid_positions()

2. Un algorithme BFS optimisé pour trouver la plus courte séquence de commandes
   bfs()

3. Un moteur de génération d’instances aléatoires (question c et d)
   generate_grid(), test_temps_taille(), test_temps_obstacle()

4. Un programme linéaire en nombres entiers (PLNE) pour générer automatiquement
   une grille avec contraintes (question e) :
   plne()

5. Une interface utilisateur complète permettant :
   - de générer une grille via le PLNE
   - d’entrer un point de départ, une orientation, un objectif
   - de calculer la trajectoire optimale via BFS
   interface()

6. Un main() lançant l’interface quand le fichier est exécuté directement.


PRÉREQUIS
---------

1. Python 3.9+
2. Modules Python :
   - gurobipy
   - time
   - random
   - sys
   - collections


Sans Gurobi, seules les fonctions BFS et tests de taille fonctionneront.


UTILISATION
-----------

1. Exécuter une instance contenue dans un fichier :
   
   python3 projet.py < grille.txt

2. Utiliser l’interface pour générer une grille optimisée par PLNE :

   python3 projet.py

   puis entrer :
       - Nombre de lignes
       - Nombre de colonnes
       - Nombre d'obstacles P
       - Position de départ (i j)
       - Orientation (nord/sud/est/ouest)
       - Position d’arrivée (i j)

3. Lancer les tests expérimentaux (questions c et d)
   → dans projet.py, appeler depuis un interpréteur Python :

   test_temps_taille()
   test_temps_obstacle()

   Ces fonctions génèrent automatiquement :
       grids_taille.txt
       temps_taille.txt
       res_taille.txt
       grids_obstacle.txt
       temps_obstacle.txt
       res_obstacle.txt


FORMAT DES ENTRÉES / SORTIES
----------------------------

• Une grille est donnée sous le format :
    M N
    M lignes de N valeurs (0 = libre, 1 = obstacle)
    D1 D2 F1 F2 orientation
    0 0   (fin)

• La sortie du BFS est :
    T cmd1 cmd2 ... cmdT
  où T est la longueur minimale en nombre de commandes.


EXEMPLE
-------

Input :
    9 10
    (grille...)
    7 2 2 7 sud
    0 0

Output :
    12 D a1 D a3 a3 D a3 a1 D a1 G a2




