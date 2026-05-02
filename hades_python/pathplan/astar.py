import heapq
import math
from pathplan.main import World, Point


# ---------------------------------------------------------------------------
# Nó interno do A*
# ---------------------------------------------------------------------------
class _Node:
    """Nó da grelha usada pelo A*."""

    __slots__ = ("point", "parent", "g", "h", "f")

    def __init__(self, point: Point, parent=None, g: float = 0.0, h: float = 0.0):
        self.point  = point
        self.parent = parent
        self.g      = g          # custo acumulado desde o início
        self.h      = h          # heurística até ao goal
        self.f      = g + h      # custo total estimado

    # Necessário para o heap do Python (compara por f)
    def __lt__(self, other):
        return self.f < other.f

    def __eq__(self, other):
        return _grid_key(self.point) == _grid_key(other.point)

    def __hash__(self):
        return hash(_grid_key(self.point))


def _grid_key(point: Point, resolution: float = 0.5) -> tuple:
    """Converte um Point em chave de grelha discreta."""
    return (round(point.x / resolution), round(point.y / resolution))


def _heuristic(a: Point, b: Point) -> float:
    """Distância euclidiana como heurística admissível."""
    dx = a.x - b.x
    dy = a.y - b.y
    return math.hypot(dx, dy)


def _neighbors(point: Point, resolution: float) -> list[Point]:
    """
    Gera os 8 vizinhos (4 cardeais + 4 diagonais) de um ponto na grelha.
    As diagonais têm custo √2 × resolution.
    """
    r = resolution
    offsets = [
        ( r,  0),   (-r,  0),   (0,  r),   (0, -r),   # cardeais
        ( r,  r),   (-r,  r),   ( r, -r),  (-r, -r),  # diagonais
    ]
    return [Point(point.x + dx, point.y + dy) for dx, dy in offsets]


# ---------------------------------------------------------------------------
# Planner A* — interface idêntica ao RRT
# ---------------------------------------------------------------------------
class AStar:
    """
    Planeador de caminhos baseado em A* sobre uma grelha discreta.

    Interface compatível com RRT:
        planner = AStar(world=world)
        path    = planner.plan(start, goal)   # → list[Point] | None

    Parâmetros
    ----------
    world       : World  — ambiente com obstáculos e fronteiras
    resolution  : float  — tamanho da célula da grelha (m)
    max_nodes   : int    — limite de nós expandidos (evita loops infinitos)
    """

    def __init__(self, world: World, resolution: float = 0.5, max_nodes: int = 50_000):
        self.world      = world
        self.resolution = resolution
        self.max_nodes  = max_nodes

    # ------------------------------------------------------------------
    # Utilidades internas
    # ------------------------------------------------------------------
    def _in_bounds(self, point: Point) -> bool:
        """Verifica se o ponto está dentro das fronteiras do world."""
        if self.world.boundaries is None:
            return True
        return self.world.do_contain_point(point)

    def _is_free(self, point: Point) -> bool:
        """Ponto livre de obstáculos e dentro das fronteiras."""
        if not self._in_bounds(point):
            return False
        # Verifica colisão com cada obstáculo individualmente via segmento nulo
        return self.world.is_free_path(point, point)

    def _is_free_segment(self, a: Point, b: Point) -> bool:
        """Segmento entre dois pontos sem colisões."""
        return self.world.is_free_path(a, b)

    @staticmethod
    def _step_cost(a: Point, b: Point) -> float:
        return math.hypot(b.x - a.x, b.y - a.y)

    def _backtrack(self, node: _Node) -> list[Point]:
        path = []
        while node:
            path.append(node.point)
            node = node.parent
        return path[::-1]

    def _snap_to_grid(self, point: Point) -> Point:
        """Encaixa um ponto arbitrário no centro da célula de grelha mais próxima."""
        r = self.resolution
        return Point(round(point.x / r) * r, round(point.y / r) * r)

    # ------------------------------------------------------------------
    # CORE — interface pública idêntica ao RRT
    # ------------------------------------------------------------------
    def plan(self, start: Point, goal: Point) -> list[Point] | None:
        """
        Planeia um caminho de *start* até *goal*.

        Devolve uma lista de Point ou None se não existir caminho.
        """
        r = self.resolution

        # Encaixa início e fim na grelha
        start_snapped = self._snap_to_grid(start)
        goal_snapped  = self._snap_to_grid(goal)

        # Verifica se início/fim são acessíveis
        if not self._is_free(start_snapped):
            start_snapped = start   # tenta com o ponto original
        if not self._is_free(goal_snapped):
            goal_snapped = goal

        open_heap: list[_Node] = []
        open_dict: dict[tuple, _Node]  = {}   # chave → melhor nó aberto
        closed:    set[tuple]          = set()

        start_node = _Node(
            start_snapped,
            parent=None,
            g=0.0,
            h=_heuristic(start_snapped, goal_snapped),
        )
        heapq.heappush(open_heap, start_node)
        open_dict[_grid_key(start_snapped, r)] = start_node

        nodes_expanded = 0

        while open_heap:
            if nodes_expanded >= self.max_nodes:
                return None   # limite de expansão atingido

            current = heapq.heappop(open_heap)
            cur_key = _grid_key(current.point, r)

            if cur_key in closed:
                continue
            closed.add(cur_key)
            nodes_expanded += 1

            # ── Goal reached? ──────────────────────────────────────────────
            if _heuristic(current.point, goal_snapped) <= r * 1.5:
                # Tenta ligar directamente ao goal real
                if self._is_free_segment(current.point, goal):
                    goal_node = _Node(goal, parent=current, g=current.g, h=0.0)
                    raw_path  = self._backtrack(goal_node)
                    # Garante que o ponto de início é o original
                    if raw_path and raw_path[0] != start:
                        raw_path[0] = start
                    return raw_path

            # ── Expandir vizinhos ──────────────────────────────────────────
            for nb_point in _neighbors(current.point, r):
                nb_key = _grid_key(nb_point, r)

                if nb_key in closed:
                    continue
                if not self._in_bounds(nb_point):
                    continue
                if not self._is_free_segment(current.point, nb_point):
                    continue

                tentative_g = current.g + self._step_cost(current.point, nb_point)

                # Se já existe um nó aberto com custo menor, ignorar
                existing = open_dict.get(nb_key)
                if existing is not None and existing.g <= tentative_g:
                    continue

                nb_node = _Node(
                    nb_point,
                    parent=current,
                    g=tentative_g,
                    h=_heuristic(nb_point, goal_snapped),
                )
                heapq.heappush(open_heap, nb_node)
                open_dict[nb_key] = nb_node

        return None   # sem caminho