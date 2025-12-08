from collections import deque
import sys
import random
import time
import gurobipy as gp
import math

# Encodage des orientations
# 0 = nord, 1 = est, 2 = sud, 3 = ouest
DIR_MAP = {
    "nord": 0,
    "est": 1,
    "sud": 2,
    "ouest": 3
}

INV_DIR_MAP = {
    0 : "nord",
    1 : "est",
    2 : "sud",
    3 : "ouest"
}
# Vecteurs de déplacement associés aux directions
# 0=N, 1=E, 2=S, 3=O
DR = [-1, 0, 1, 0]  # variation sur la ligne
DC = [ 0, 1, 0,-1]  # variation sur la colonne


# =========================
#  Fonctions utilitaires
# =========================

def turn_left(d):
    """Retourne la direction obtenue après un quart de tour à gauche."""
    return (d + 3) % 4  # équivalent à d - 1 mod 4


def turn_right(d):
    """Retourne la direction obtenue après un quart de tour à droite."""
    return (d + 1) % 4


def advance_n(r, c, d, n):
    """Renvoie la position (nr, nc) après un déplacement de n cases dans la direction d."""
    return r + DR[d] * n, c + DC[d] * n


def can_advance(valid, M, N, r, c, d, n):
    """
    Vérifie si le robot peut avancer de n positions d'intersection depuis (r,c) dans la direction d.
    On vérifie toutes les intersections intermédiaires (nr, nc) sont valides.
    """
    for step in range(1, n + 1):
        nr = r + DR[d] * step
        nc = c + DC[d] * step

        # On doit rester dans la zone 1..M-1, 1..N-1
        if nr <= 0 or nr >= M or nc <= 0 or nc >= N:
            return False

        if not valid[nr][nc]:
            return False

    return True


def build_valid_positions(grid, M, N):
    """
    valid[r][c] = True si le centre du robot peut se trouver à l'intersection (r,c).
    On utilise les cases :
      (r-1,c-1), (r-1,c),
      (r,  c-1), (r,  c)
    qui doivent toutes être libres (0).
    r va de 1 à M-1, c de 1 à N-1.
    """
    valid = [[False] * N for _ in range(M)]
    for r in range(1, M):
        for c in range(1, N):
            if (grid[r-1][c-1] == 0 and
                grid[r-1][c]   == 0 and
                grid[r][c-1]   == 0 and
                grid[r][c]     == 0):
                valid[r][c] = True
    return valid

def generate_grid(M, N, nb_obs):
    """Renvoie une grid sous forme liste de listes"""
    grid = [[0 for _ in range(M)] for _ in range(N)]
    obstacles = set()

    while len(obstacles) < nb_obs:
        x = random.randint(0, N-1)
        y = random.randint(0, M-1)
        if (x,y) not in obstacles :
            obstacles.add((x, y))

    for (x, y) in obstacles:
        grid[x][y] = 1

    ligne_fin = []

    while True :
        x_rob = random.randint(0, N-1)
        y_rob = random.randint(0, M-1)
        x_obs = random.randint(0, N-1)
        y_obs = random.randint(0, M-1)
        if (x_rob,y_rob) != (x_obs,y_obs) and (x_rob,y_rob) not in obstacles and (x_obs,y_obs) not in obstacles:
            dir = random.randint(0, 3)
            ligne_fin = [x_rob, y_rob, x_obs, y_obs, INV_DIR_MAP[dir]]
            break
        
    final = []
    final.append([M,N])
    final.extend(grid)
    final.append(ligne_fin)
    final.append([0,0])
    return final

def read_grid(filename):
    grid = []
    with open(filename, "r") as f:
        lines = [line.strip() for line in f.readlines() if line.strip()]

    N, M = map(int, lines[0].split())
    grid.append([M, N])

    for i in range(1,M):
        ligne = list(map(int, lines[i].split()))
        grid.append(ligne)

    data = lines[i+1].split()
    D1 = int(data[0])
    D2 = int(data[1])
    F1 = int(data[2])
    F2 = int(data[3])
    orient = data[4]  
    grid.append([D1, D2, F1, F2, orient])

    grid.append([0,0])

    return grid


# =========================
#  BFS sur l'espace d'états
# =========================

def bfs(valid, M, N, start_r, start_c, start_dir, goal_r, goal_c):
    """
    BFS sur les intersections valides (r,c,d).
    Les indices r,c sont des intersections, pas des cases.
    """

    # Si départ ou arrivée non valides -> impossible
    if not (0 < start_r < M and 0 < start_c < N and valid[start_r][start_c]):
        return -1, []
    if not (0 < goal_r < M and 0 < goal_c < N and valid[goal_r][goal_c]):
        return -1, []

    visited = [[[False] * 4 for _ in range(N)] for __ in range(M)]
    parent = {}

    from collections import deque
    q = deque()

    start = (start_r, start_c, start_dir)
    visited[start_r][start_c][start_dir] = True
    parent[start] = (None, None)
    q.append(start)

    end_state = None

    while q:
        r, c, d = q.popleft()

        if r == goal_r and c == goal_c:
            end_state = (r, c, d)
            break

        # Tourner gauche
        nd = turn_left(d)
        if not visited[r][c][nd]:
            visited[r][c][nd] = True
            parent[(r, c, nd)] = ((r, c, d), 'G')
            q.append((r, c, nd))

        # Tourner droite
        nd = turn_right(d)
        if not visited[r][c][nd]:
            visited[r][c][nd] = True
            parent[(r, c, nd)] = ((r, c, d), 'D')
            q.append((r, c, nd))

        # Avancer de 1, 2, 3
        for step in range(1, 4):
            if not can_advance(valid, M, N, r, c, d, step):
                break

            nr, nc = advance_n(r, c, d, step)

            if not visited[nr][nc][d]:
                visited[nr][nc][d] = True
                parent[(nr, nc, d)] = ((r, c, d), str(step))
                q.append((nr, nc, d))

    if end_state is None:
        return -1, []

    # Reconstruction
    actions = []
    state = end_state
    while parent[state][0] is not None:
        prev_state, act = parent[state]
        actions.append(act)
        state = prev_state

    actions.reverse()

    commands = []
    for a in actions:
        if a == 'G':
            commands.append('G')
        elif a == 'D':
            commands.append('D')
        else:  # '1','2','3'
            commands.append('a' + a)

    return len(actions), commands


def exec(grid, grid_file="grid.txt", res_file="res.txt"):
    """
    Execute l'algorithme de parcours en largeur sur la grille mis en argument.
    Ecrit l'instance et les résultats dans les fichiers.
    """
    with open(grid_file, "w") as f_grid, open(res_file, "w") as f_res:

        N, M  = grid[0]

        print(grid)

        for ligne in grid:
            f_grid.write(" ".join(map(str, ligne)) + "\n")
                
        D1, D2, F1, F2, orient_str = grid[-2] # Obtention des positions objectifs et de départ
        start_dir = DIR_MAP[orient_str]

        # Construire les positions valides pour le centre du robot
        valid = build_valid_positions(grid[1:-2], M, N)

        # Ici D1,D2,F1,F2 sont des coordonnées d'intersections (coins nord-ouest)
        dist, cmds = bfs(valid, M, N, D1, D2, start_dir, F1, F2)

        if dist == -1:
            print(-1)
        else:
            if cmds:
                print(dist, *cmds)
            else:
                print(0)
        f_res.write(str(dist) +" "+ " ".join(map(str,cmds)) + "\n")

# =========================
#  Fonctions de tests
# =========================

def test_temps_taille():
    """
    Test numérique de temps de calcul sur plusieurs tailles NxN différentes avec N obstacles.
    """
    tailles = [10, 20, 30, 40, 50]
    with open("grids_taille.txt", "w") as f_grids, open("temps_taille.txt", "w") as f_temps, open("res_taille.txt", "w") as f_res:

        for N in tailles:
            nb_obstacles = N 
            temps_exec = []

            for _ in range(10):
                grid = generate_grid(N, N, nb_obstacles)

                f_grids.write(f"{N} {N}\n") # 1ère ligne
                for ligne in grid:
                    f_grids.write(" ".join(map(str, ligne)) + "\n") #Ecrit chaque ligne de la grid

                t0 = time.perf_counter()

                valid = build_valid_positions(grid[1:-2], N, N)
                D1, D2, F1, F2, orient_str = grid[-2]
                start_dir = DIR_MAP[orient_str]
                long, act = bfs(valid, N, N, D1, D2, start_dir, F1, F2)

                t1 = time.perf_counter()
                f_res.write(str(long) +" "+ " ".join(map(str,act)) + "\n")
                temps_exec.append(t1 - t0)

            moyenne = sum(temps_exec) / len(temps_exec)
            f_temps.write(f"{N}\t{moyenne:.6f}\n")
                
def test_temps_obstacle():
    """
    Test numérique de temps de calcul sur plusieurs nombre d'obstacles différents pour une grille 20x20.
    """
    obs = [10, 20, 30, 40, 50]
    with open("grids_obstacle.txt", "w") as f_grids, open("temps_obstacle.txt", "w") as f_temps, open("res_obstacle.txt", "w") as f_res:

        for N in obs:
            nb_obstacles = N 
            temps_exec = []

            for _ in range(10):
                grid = generate_grid(20, 20, nb_obstacles)

                f_grids.write(f"{20} {20}\n") # 1ère ligne
                for ligne in grid:
                    f_grids.write(" ".join(map(str, ligne)) + "\n") #Ecrit chaque ligne de la grid

                t0 = time.perf_counter()

                valid = build_valid_positions(grid[1:-2], 20, 20)
                
                D1, D2, F1, F2, orient_str = grid[-2]
                start_dir = DIR_MAP[orient_str]
                long, act = bfs(valid, 20, 20, D1, D2, start_dir, F1, F2)

                t1 = time.perf_counter()
                f_res.write(str(long) +" "+ " ".join(map(str,act)) + "\n")
                temps_exec.append(t1 - t0)

            moyenne = sum(temps_exec) / len(temps_exec)
            f_temps.write(f"{N}\t{moyenne:.6f}\n")

# =========================
#  Fonctions pour l'interface du programme linéaire
# =========================

def generate_value_grid(M, N):
    """
    Crée la grille de poids pour le PLNE
    """
    return [[random.randint(0,1000) for _ in range(M)] for _ in range(N)]

def plne(M, N, P, grid):
    """
    Programme linéaire permettant d'attribuer P obstacles de manière a minimiser la somme des poids,
    en respectant les contraintes données.
    """
    m = gp.Model()
    x = {}
    
    #Création des variables binaires
    for i in range(N):
        for j in range(M):
            x[i,j] = m.addVar(vtype=gp.GRB.BINARY)
    #Contrainte sur le nb d'obstacles dans une colonne
    for i in range(N):
        m.addConstr(gp.quicksum(x[i,j] for j in range(M)) <= math.ceil((2*P)/M))
    #Contrainte sur le nb d'obstacles dans une ligne
    for j in range(M) :
        m.addConstr(gp.quicksum(x[i,j] for i in range(N)) <= math.ceil((2*P)/N))
    #Contrainte 101 dans les lignes
    if M >= 3:
        for i in range(N):
            for j in range(M-2):
                m.addConstr(x[i,j] + x[i,j+2] <= 1 + x[i,j+1])
    #Contrainte 101 dans les colonnes
    if N >= 3:
        for j in range(M):
            for i in range(N-2):
                m.addConstr(x[i,j] + x[i+2,j] <= 1 + x[i+1,j])
    # Contrainte finale, il faut p obstacles et on met l'objectif de minimization
    m.addConstr(gp.quicksum(x[i,j] for i in range(N) for j in range(M)) == P)
    m.setObjective(gp.quicksum(grid[i][j] * x[i,j] for i in range(N) for j in range(M)), gp.GRB.MINIMIZE)

    m.optimize()
    
    return [(i, j) for (i, j), var in x.items() if var.X > 0.5]

def interfacePLNE():
    """
    Interface permettant à l'utilisateur de fournir les données pour une instance,
    laissant le programme linéaire placer les obstacles.
    """
    N = int(input("Nombre de lignes = "))
    M = int(input("Nombre de colonnes = "))
    P = int(input("Nombre d'obstacles = "))

    grid = [[0 for _ in range(M)] for _ in range(N)]
    weight = generate_value_grid(M, N)

    obstacles = plne(M, N, P, weight)
    print(f"Position des obstacles :\n{obstacles}")
    for i in obstacles :
        grid[i[0]][i[1]] = 1
    while True :
        si, sj = map(int, input("\n Position de départ (m n): ").split())
        if (si,sj) in obstacles :
            print("Cette position est dans un obstacle. Choisissez à nouveau \n")
        else : 
            break
    while True :
        orientation = input("Orientation (nord/sud/est/ouest): ")
        if orientation not in ["sud", "nord", "est", "ouest"] :
            print("Cette orientation n'est pas bonne")
        else : 
            break
    while True :
        oi, oj = map(int, input("Position objectif (m n): ").split())
        if (oi,oj) in obstacles :
            print("Cette position est dans un obstacle. Choisissez à nouveau \n")
        else : 
            break
    
    valid = build_valid_positions(grid, M, N)
    dist, act = bfs(valid, M, N, si, sj, orientation, oi, oj) 

    print(f"{dist} {act}")

def interfaceExec():
    name = input("Nom du fichier contenant l'instance : ")
    exec(read_grid(name))
    
def main():
    Choix = int(input("1 - Execution depuis une instance dans un fichier\n2 - Utilisation du PLNE \n"))
    if Choix == 2 :
        interfacePLNE()
    if Choix == 1 :
        interfaceExec()

if __name__ == "__main__":
    main()