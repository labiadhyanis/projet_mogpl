from collections import deque
import sys

# Encodage des orientations
# 0 = nord, 1 = est, 2 = sud, 3 = ouest
DIR_MAP = {
    "nord": 0,
    "est": 1,
    "sud": 2,
    "ouest": 3
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


def main():
    data = sys.stdin.read().strip().splitlines()
    idx = 0

    while idx < len(data):
        line = data[idx].strip()
        idx += 1
        if not line:
            continue

        M, N = map(int, line.split())
        if M == 0 and N == 0:
            break

        grid = []
        for _ in range(M):
            row = list(map(int, data[idx].split()))
            idx += 1
            grid.append(row)

        parts = data[idx].split()
        idx += 1

        D1 = int(parts[0])
        D2 = int(parts[1])
        F1 = int(parts[2])
        F2 = int(parts[3])
        orient_str = parts[4].lower()
        start_dir = DIR_MAP[orient_str]

        # Construire les positions valides pour le centre du robot
        valid = build_valid_positions(grid, M, N)

        # Ici D1,D2,F1,F2 sont des coordonnées d'intersections (coins nord-ouest)
        dist, cmds = bfs(valid, M, N, D1, D2, start_dir, F1, F2)

        if dist == -1:
            print(-1)
        else:
            if cmds:
                print(dist, *cmds)
            else:
                print(0)


if __name__ == "__main__":
    main()