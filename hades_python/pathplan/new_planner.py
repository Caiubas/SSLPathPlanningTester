import math
from typing import Optional, List


# ============================================================================
# NODE
# ============================================================================

class Node:
    """
    Nó da árvore de busca.
    """
    def __init__(self, point, parent=None):
        self.point = point
        self.parent = parent
        self.children: list["Node"] = []

        self.generated_children = False   # evita loops / regeneração
        self.dead = False                # nó sem saída

    def add_child(self, child: "Node"):
        self.children.append(child)

    def alive_children(self):
        return [c for c in self.children if not c.dead]

    def path_cost(self):
        """
        custo acumulado até raiz
        """
        cost = 0.0
        n = self
        while n.parent is not None:
            cost += n.point.distance_to(n.parent.point)
            n = n.parent
        return cost

    def extract_path(self):
        pts = []
        n = self
        while n is not None:
            pts.append(n.point)
            n = n.parent
        return list(reversed(pts))

    def __repr__(self):
        return f"Node({self.point})"


# ============================================================================
# TREE
# ============================================================================

class Tree:
    def __init__(self, start):
        self.root = Node(start)
        self.nodes = [self.root]

    def add_node(self, point, parent):
        n = Node(point, parent)
        parent.add_child(n)
        self.nodes.append(n)
        return n


# ============================================================================
# PATH PLANNER
# ============================================================================

class PathPlanner:
    """
    Busca determinística por tangentes com backtracking.
    """

    def __init__(
        self,
        world,
        min_step: float = 5.0,
        clearance: float = 1.0
    ):
        self.world = world
        self.min_step = min_step
        self.clearance = clearance

        self.visited_points = set()

    # ----------------------------------------------------------------------
    # heurística
    # ----------------------------------------------------------------------
    def heuristic(self, node, goal):
        """
        menor = melhor
        custo acumulado + distância ao alvo
        """
        return node.path_cost() + node.point.distance_to(goal)

    # ----------------------------------------------------------------------
    # remove pontos ruins
    # ----------------------------------------------------------------------
    def valid_point(self, p):
        if not self.world.do_contain_point(p):
            return False

        if self.world.inside_obstacle(p) is not None:
            return False

        if p in self.visited_points:
            return False

        return True

    # ----------------------------------------------------------------------
    # gera filhos tangentes
    # ----------------------------------------------------------------------
    def generate_children(self, tree, node, obstacle, goal):
        """
        Gera pontos tangentes ao obstáculo.
        """

        if node.generated_children:
            return

        node.generated_children = True

        tangents = obstacle.get_tangent_points(node.point)

        candidates = []

        for p in tangents:

            # passo mínimo
            if node.point.distance_to(p) < self.min_step:
                continue

            if not self.valid_point(p):
                continue

            # caminho node -> tangente precisa estar livre
            if not self.world.is_free_path(node.point, p):
                # se colidir, descartado
                continue

            candidates.append(p)

        # cria nós
        for p in candidates:
            self.visited_points.add(p)
            tree.add_node(p, node)

    # ----------------------------------------------------------------------
    # mata nó e sobe recursivamente
    # ----------------------------------------------------------------------
    def kill_if_needed(self, node):
        while node is not None:

            if node.alive_children():
                return node

            node.dead = True
            node = node.parent

        return None

    # ----------------------------------------------------------------------
    # seleciona melhor filho
    # ----------------------------------------------------------------------
    def best_child(self, node, goal):
        alive = node.alive_children()

        if not alive:
            return None

        alive.sort(key=lambda c: self.heuristic(c, goal))
        return alive[0]

    # ----------------------------------------------------------------------
    # planner principal
    # ----------------------------------------------------------------------
    def plan(self, start, goal):

        tree = Tree(start)
        current = tree.root
        self.visited_points.add(start)

        while current is not None:

            # --------------------------------------------------------------
            # tenta conexão direta ao objetivo
            # --------------------------------------------------------------
            if self.world.is_free_path(current.point, goal):
                goal_node = tree.add_node(goal, current)
                return goal_node.extract_path()

            # --------------------------------------------------------------
            # encontra obstáculo bloqueando linha reta
            # --------------------------------------------------------------
            obs = self.world.obstacle_hit(current.point, goal)

            if obs is None:
                # redundância defensiva
                goal_node = tree.add_node(goal, current)
                return goal_node.extract_path()

            # --------------------------------------------------------------
            # gera filhos UMA vez
            # --------------------------------------------------------------
            self.generate_children(tree, current, obs, goal)

            # --------------------------------------------------------------
            # escolhe melhor filho
            # --------------------------------------------------------------
            nxt = self.best_child(current, goal)

            if nxt is not None:
                current = nxt
                continue

            # --------------------------------------------------------------
            # nó morreu -> backtrack
            # --------------------------------------------------------------
            current.dead = True
            current = self.kill_if_needed(current.parent)

        return None


# ============================================================================
# USO
# ============================================================================

# planner = TangentTreePlanner(world, min_step=10.0)
# path = planner.plan(start_point, goal_point)

# if path:
#     print("Caminho encontrado:")
#     for p in path:
#         print(p)
# else:
#     print("Sem solução")