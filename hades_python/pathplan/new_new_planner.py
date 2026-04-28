import math
from typing import Optional, List, Set

from main import (
    Point, Vector, Quadrilateral, Circle, Stadium, generate_random_world, World, Obstacle
)

class Node:
    def __init__(self, point, parent=None):
        self.point = point
        self.parent = parent
        self.children: List[Node] = []
        self.expanded = False
        self.dead = False

    def path(self):
        node = self
        p = []
        while node:
            p.append(node.point)
            node = node.parent
        return list(reversed(p))


class Tree:
    def __init__(self, start):
        self.root = Node(start)
        self.current = self.root


class PathPlanner:
    def __init__(
        self,
        world,
        step_size: float = 1.5,
        min_step: float = 0.0,
        max_iterations: int = 5000,
        super_position_margin: float = 0.1,
    ):
        self.world = world
        self.step_size = step_size
        self.min_step = min_step
        self.max_iterations = max_iterations
        self.super_position_margin = super_position_margin

        self.visited: Set[tuple] = set()

    # ---------------------------
    # Utils
    # ---------------------------

    def _hash_point(self, p):
        return (round(p.x, 3), round(p.y, 3))

    def _is_new_point(self, p):
        key = self._hash_point(p)
        if key in self.visited:
            return False
        self.visited.add(key)
        return True

    def _heuristic(self, p, goal):
        return p.distance_to(goal)

    def _step_towards(self, origin, target):
        v = Vector.from_points(origin, target)
        d = v.get_norm()

        if d < 1e-9:
            return target

        v = v.get_normalized()
        step = min(self.step_size, d)

        return Point(
            origin.x + v.x * step,
            origin.y + v.y * step
        )

    def _validate_point(self, p):
        if not self.world.do_contain_point(p):
            return False
        if self.world.inside_obstacle(p):
            return False
        return True

    # ---------------------------
    # Core: gerar filhos
    # ---------------------------

    def _is_close_to_branch(self, candidate, node):
        """
        Sobe a árvore a partir do nó atual verificando se o candidato
        já está perigosamente perto de algum ancestral do próprio ramo.
        """
        current = node
        while current:
            if current.point.distance_to(candidate) <= self.super_position_margin:
                return True
            current = current.parent
        return False

    def _generate_children(self, node, goal):
        generated = []

        # mesma lógica de cascata do main.py (_c_point)
        unchecked = [goal]

        # evita expandir o mesmo obstáculo infinitamente
        visited_obstacles = set()

        # evita filhos duplicados
        generated_keys = set()

        while unchecked:
            candidate = unchecked.pop(0)
            collided = False

            for obstacle in self.world.obstacles:

                if obstacle.is_intercepted_by(node.point, candidate):
                    collided = True

                    obs_id = id(obstacle)

                    # expande tangentes uma vez por nó
                    if obs_id not in visited_obstacles:
                        visited_obstacles.add(obs_id)

                        tangents = obstacle.get_tangent_points(node.point)

                        for p in tangents:
                            unchecked.append(p)

            # fora dos limites = inválido
            if not self.world.do_contain_point(candidate):
                collided = True

            # candidato livre vira filho
            if not collided:

                if node.point.distance_to(candidate) < self.min_step:
                    continue

                if not self._validate_point(candidate):
                    continue

                if self._is_close_to_branch(candidate, node):
                    continue

                key = (round(candidate.x, 3), round(candidate.y, 3))
                if key in generated_keys:
                    continue

                generated_keys.add(key)

                child = Node(candidate, parent=node)
                generated.append(child)

        return generated

    # ---------------------------
    # Backtracking
    # ---------------------------

    def _backtrack(self, tree):
        node = tree.current
        node.dead = True

        while node.parent:
            parent = node.parent
            alive = [c for c in parent.children if not c.dead]

            if alive:
                tree.current = alive[0]
                return True

            node = parent
            node.dead = True

        return False

    # ---------------------------
    # Planejamento principal
    # ---------------------------

    def plan(self, start, goal):
        self.visited.clear()

        # se start dentro de obstáculo → sair
        obs = self.world.inside_obstacle(start)
        if obs:
            start = self._get_exit_point(obs, start)

        obs = self.world.inside_obstacle(goal)
        if obs:
            goal = self._get_exit_point(obs, goal)

        tree = Tree(start)
        self._is_new_point(start)

        iterations = 0

        while iterations < self.max_iterations:
            iterations += 1

            node = tree.current

            # ---------------------------
            # 1. conexão direta
            # ---------------------------
            if self.world.is_free_path(node.point, goal):
                return self._smooth_path(node.path() + [goal])

            # ---------------------------
            # 2. expansão
            # ---------------------------
            if not node.expanded:
                children = self._generate_children(node, goal)

                node.children = children
                node.expanded = True

                if not children:
                    node.dead = True
                else:
                    node.children.sort(
                        key=lambda c: self._heuristic(c.point, goal)
                    )

            # ---------------------------
            # 3. escolher melhor filho
            # ---------------------------
            alive_children = [c for c in node.children if not c.dead]

            if alive_children:
                tree.current = alive_children[0]
                continue

            # ---------------------------
            # 4. backtrack
            # ---------------------------
            if not self._backtrack(tree):
                return None


        print("max iter")
        return None

    def _get_exit_point(self, obstacle: Obstacle, point: Point) -> Point:
        """Obtém o ponto de saída de um obstáculo (equivale a interference() do C++)."""
        if isinstance(obstacle, (Circle, Quadrilateral, Stadium)):
            return obstacle.get_exit_point(point)
        # fallback genérico
        candidates = obstacle.get_tangent_points(point)
        if candidates:
            return min(candidates, key=lambda p: p.distance_to(point))
        return point

    def _smooth_path(self, path):
        i = 0
        while i < len(path) - 2:
            if self.world.is_free_path(path[i], path[i + 2]):
                path.pop(i + 1)
            else:
                i += 1
        return path

    def plan_debug(self, start, goal):
        self.visited.clear()

        obs = self.world.inside_obstacle(start)
        if obs:
            start = self._get_exit_point(obs, start)

        obs = self.world.inside_obstacle(goal)
        if obs:
            goal = self._get_exit_point(obs, goal)

        tree = Tree(start)
        self._is_new_point(start)

        iterations = 0

        while iterations < self.max_iterations:
            iterations += 1
            node = tree.current

            new_children = []
            dead_nodes = []

            # ---------------------------
            # conexão direta
            # ---------------------------
            if self.world.is_free_path(node.point, goal):
                yield DebugStep(node, [], [])
                return node.path() + [goal]

            # ---------------------------
            # expansão
            # ---------------------------
            if not node.expanded:
                children = self._generate_children(node, goal)

                node.children = children
                node.expanded = True
                new_children = children

                if not children:
                    node.dead = True
                    dead_nodes.append(node)
                else:
                    node.children.sort(
                        key=lambda c: self._heuristic(c.point, goal)
                    )

            # ---------------------------
            # escolher filho
            # ---------------------------
            alive_children = [c for c in node.children if not c.dead]

            yield DebugStep(node, new_children, dead_nodes)

            if alive_children:
                tree.current = alive_children[0]
                continue

            # ---------------------------
            # backtrack
            # ---------------------------
            node.dead = True
            dead_nodes.append(node)

            if not self._backtrack(tree):
                yield DebugStep(node, [], dead_nodes)
                return None







class DebugStep:
    def __init__(self, current, new_children, dead_nodes):
        self.current = current
        self.new_children = new_children
        self.dead_nodes = dead_nodes

import matplotlib.pyplot as plt


import matplotlib.pyplot as plt


class PlannerDebugger:
    def __init__(self, world):
        self.world = world

        self.fig, self.ax = plt.subplots()
        plt.ion()

    def draw_world(self):
        for obs in self.world.obstacles:
            if isinstance(obs, Circle):
                circle = plt.Circle(
                    (obs.center.x, obs.center.y),
                    obs.radius,
                    fill=False
                )
                self.ax.add_patch(circle)

            elif isinstance(obs, Quadrilateral):
                xs = [v.x for v in obs.vertices] + [obs.vertices[0].x]
                ys = [v.y for v in obs.vertices] + [obs.vertices[0].y]
                self.ax.plot(xs, ys)

            elif isinstance(obs, Stadium):
                a, b = obs.vertices
                self.ax.plot([a.x, b.x], [a.y, b.y])

        self.ax.set_aspect('equal')

    def _print_node_info(self, node):
        print("\n--- NODE INFO ---")
        print(f"Point: {node.point}")
        print(f"Children: {len(node.children)}")
        print(f"Expanded: {node.expanded}")
        print(f"Dead: {node.dead}")
        print("-----------------\n")

    def run(self, planner, start, goal):
        self.draw_world()

        gen = planner.plan_debug(start, goal)

        step_count = 0
        final_path = None

        while True:
            try:
                step = next(gen)
            except StopIteration as e:
                final_path = e.value  # ← AQUI está o segredo
                break

            step_count += 1
            node = step.current

            # ---------------------------
            # DESENHO
            # ---------------------------

            if node.parent:
                self.ax.plot(
                    [node.parent.point.x, node.point.x],
                    [node.parent.point.y, node.point.y],
                )

            for child in step.new_children:
                self.ax.plot(
                    [node.point.x, child.point.x],
                    [node.point.y, child.point.y],
                    linestyle='dashed'
                )

            for dead in step.dead_nodes:
                self.ax.plot(
                    dead.point.x,
                    dead.point.y,
                    marker='x'
                )

            self.ax.plot(node.point.x, node.point.y, marker='o')

            self.ax.plot(start.x, start.y, marker='s')
            self.ax.plot(goal.x, goal.y, marker='*')

            self.ax.set_title(f"Step {step_count}")

            # atualização correta
            self.fig.canvas.draw()
            self.fig.canvas.flush_events()
            plt.pause(0.001)

            # controle
            cmd = input("[ENTER]=next | b=burst 10 | p=info | q=quit > ").strip()

            if cmd == "q":
                print("Encerrado pelo usuário.")
                return

            elif cmd == "p":
                print("\n--- NODE INFO ---")
                print(f"Point: {node.point}")
                print(f"Children: {len(node.children)}")
                print(f"Expanded: {node.expanded}")
                print(f"Dead: {node.dead}")
                print("-----------------\n")
                input("Pressione ENTER...")

            elif cmd == "b":
                for _ in range(10):
                    try:
                        step = next(gen)
                    except StopIteration as e:
                        final_path = e.value
                        break

                    node = step.current

                    if node.parent:
                        self.ax.plot(
                            [node.parent.point.x, node.point.x],
                            [node.parent.point.y, node.point.y],
                        )

                self.fig.canvas.draw()
                self.fig.canvas.flush_events()
                plt.pause(0.001)

        # ---------------------------
        # DESENHAR PATH FINAL
        # ---------------------------
        if final_path:
            print("Path encontrado!")

            xs = [p.x for p in final_path]
            ys = [p.y for p in final_path]

            self.ax.plot(xs, ys, linewidth=3)

            self.fig.canvas.draw()
            self.fig.canvas.flush_events()
            plt.pause(0.001)

        else:
            print("Nenhum caminho encontrado.")

        plt.ioff()
        plt.show()



if __name__ == '__main__':
    boundaries = Quadrilateral(vertices=[
        Point(0.0, 0.0),
        Point(24.0, 0.0),
        Point(24.0, 20.0),
        Point(0.0, 20.0),
    ])
    point_a, point_b, obstacles = generate_random_world(
        boundaries,
        n_circles=32,
        n_quads=13,
        n_stadiums=0
    )
    obstacles = [Circle(center=Point(1.551, 3.444), radius=1.4580930399059089),
     Circle(center=Point(17.586, 0.156), radius=0.7275764680218594),
     Circle(center=Point(9.893, 16.039), radius=0.8235058250285312),
     Circle(center=Point(15.638, 10.680), radius=0.45983807087419404),
     Circle(center=Point(22.714, 15.553), radius=1.2479849204766449),
     Circle(center=Point(7.816, 3.806), radius=0.9524006536322488),
     Circle(center=Point(20.015, 5.000), radius=0.86269113718211),
     Circle(center=Point(12.555, 5.078), radius=0.724093123655689),
     Circle(center=Point(19.098, 14.361), radius=1.1255217641665338),
     Circle(center=Point(21.749, 19.570), radius=0.7425404035021971),
     Circle(center=Point(15.440, 17.664), radius=1.0237577467820111),
     Circle(center=Point(10.397, 2.893), radius=0.9053348418410287),
     Circle(center=Point(14.317, 2.296), radius=1.0268631050525996),
     Circle(center=Point(7.453, 18.656), radius=0.508244658588918),
     Circle(center=Point(20.666, 6.026), radius=0.9429429895554325),
     Circle(center=Point(10.045, 16.478), radius=1.4696954778132079),
     Circle(center=Point(17.741, 0.568), radius=0.40686655987492654),
     Circle(center=Point(5.914, 14.375), radius=0.9129847516468483),
     Circle(center=Point(16.806, 1.196), radius=0.7297595213411222),
     Circle(center=Point(11.495, 13.624), radius=1.0284800599576833),
     Circle(center=Point(19.734, 14.586), radius=0.5919332390853216),
     Circle(center=Point(1.936, 0.892), radius=1.3450854079180932),
     Circle(center=Point(21.430, 10.442), radius=1.5166472799887576),
     Circle(center=Point(11.413, 1.666), radius=1.2118059715102445),
     Circle(center=Point(13.204, 5.590), radius=0.6233267099031933),
     Circle(center=Point(2.240, 17.167), radius=0.6347573696328166),
     Circle(center=Point(9.322, 16.759), radius=0.7896204064752922),
     Circle(center=Point(14.902, 15.278), radius=0.7760100907566589),
     Circle(center=Point(9.597, 0.076), radius=1.287811701141421),
     Circle(center=Point(15.985, 17.564), radius=0.7101208208248333),
     Circle(center=Point(5.926, 8.762), radius=1.3073909483766286),
     Circle(center=Point(11.998, 6.073), radius=0.5138472202630816),
     Quadrilateral(vertices=[Point(-0.280, -0.837), Point(1.637, -0.837), Point(1.637, 1.335), Point(-0.280, 1.335)]),
     Quadrilateral(vertices=[Point(6.752, 17.873), Point(10.204, 17.873), Point(10.204, 20.577), Point(6.752, 20.577)]),
     Quadrilateral(vertices=[Point(20.044, 6.993), Point(23.153, 6.993), Point(23.153, 9.912), Point(20.044, 9.912)]),
     Quadrilateral(vertices=[Point(14.190, 7.460), Point(16.686, 7.460), Point(16.686, 8.962), Point(14.190, 8.962)]),
     Quadrilateral(vertices=[Point(18.551, 3.070), Point(21.709, 3.070), Point(21.709, 4.480), Point(18.551, 4.480)]),
     Quadrilateral(vertices=[Point(8.708, 16.765), Point(10.706, 16.765), Point(10.706, 19.057), Point(8.708, 19.057)]),
     Quadrilateral(vertices=[Point(3.137, 18.495), Point(4.597, 18.495), Point(4.597, 21.424), Point(3.137, 21.424)]),
     Quadrilateral(
         vertices=[Point(16.266, 11.719), Point(18.093, 11.719), Point(18.093, 14.173), Point(16.266, 14.173)]),
     Quadrilateral(vertices=[Point(-0.323, -0.190), Point(1.902, -0.190), Point(1.902, 1.084), Point(-0.323, 1.084)]),
     Quadrilateral(
         vertices=[Point(17.829, 19.207), Point(19.968, 19.207), Point(19.968, 20.658), Point(17.829, 20.658)]),
     Quadrilateral(
         vertices=[Point(18.224, 17.704), Point(20.199, 17.704), Point(20.199, 19.970), Point(18.224, 19.970)]),
     Quadrilateral(
         vertices=[Point(17.270, 15.224), Point(18.874, 15.224), Point(18.874, 17.093), Point(17.270, 17.093)]),
     Quadrilateral(vertices=[Point(17.562, 9.621), Point(19.606, 9.621), Point(19.606, 12.536), Point(17.562, 12.536)])]
    point_a = Point(1.309, 17.874)
    point_b = Point(22.639, 7.697)
    world = World(obstacles=obstacles, boundaries=boundaries)
    debugger = PlannerDebugger(world)
    planner = PathPlanner(world)

    debugger.run(planner, point_a, point_b)