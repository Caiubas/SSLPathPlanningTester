import random
import math
from pathplan.main import *


class Node:
    def __init__(self, point: Point, parent=None):
        self.point = point
        self.parent = parent


class RRT:
    def __init__(self, world: World, step_size=0.5, max_iter=5000):
        self.world = world
        self.step_size = step_size
        self.max_iter = max_iter
        self.nodes = []

    # --------------------------------------------------
    # Utilidades
    # --------------------------------------------------

    def sample(self, goal: Point, goal_bias=0.1) -> Point:
        """Amostragem com bias para o goal"""
        if random.random() < goal_bias:
            return goal

        # bounding box simples
        if self.world.boundaries:
            xs = [v.x for v in self.world.boundaries.vertices]
            ys = [v.y for v in self.world.boundaries.vertices]
            return Point(random.uniform(min(xs), max(xs)),
                         random.uniform(min(ys), max(ys)))

        return Point(random.uniform(0, 10), random.uniform(0, 10))

    def nearest(self, point: Point) -> Node:
        return min(self.nodes, key=lambda n: n.point.distance_to(point))

    def steer(self, from_p: Point, to_p: Point) -> Point:
        """Avança do from_p em direção ao to_p limitando a distância ao step_size"""
        dist = from_p.distance_to(to_p)
        if dist < self.step_size:
            return to_p

        # Cálculo convencional de steering (direção e magnitude)
        theta = math.atan2(to_p.y - from_p.y, to_p.x - from_p.x)
        new_x = from_p.x + self.step_size * math.cos(theta)
        new_y = from_p.y + self.step_size * math.sin(theta)
        return Point(new_x, new_y)

    def backtrack(self, node: Node):
        path = []
        while node:
            path.append(node.point)
            node = node.parent
        return path[::-1]

    # --------------------------------------------------
    # CORE
    # --------------------------------------------------

    def plan(self, start: Point, goal: Point):
        self.nodes = [Node(start)]

        for _ in range(self.max_iter):

            rand = self.sample(goal)
            nearest = self.nearest(rand)

            new_point = self.steer(nearest.point, rand)

            # -----------------------------------
            # RRT Convencional: Apenas caminho livre
            # -----------------------------------
            # Se houver colisão, o bloco 'if' falha e o ponto é ignorado (loop continua).
            if self.world.is_free_path(nearest.point, new_point):
                new_node = Node(new_point, nearest)
                self.nodes.append(new_node)

                # Verifica se está próximo o suficiente do objetivo para conectar
                if new_point.distance_to(goal) <= self.step_size:
                    if self.world.is_free_path(new_point, goal):
                        goal_node = Node(goal, new_node)
                        self.nodes.append(goal_node)
                        return self.backtrack(goal_node)

        return None