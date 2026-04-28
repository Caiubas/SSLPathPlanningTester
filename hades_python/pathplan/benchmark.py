from time import perf_counter
import statistics as stats

import numpy as np

import plot
from main import no_collision, World, generate_random_world, Point, Quadrilateral, PathPlanner
from new_new_planner import PathPlanner as RRT

import matplotlib.pyplot as plt

from new_bboptimizer import *
from pipeline import new_no_collision

#from rrt import RRT

def plot_benchmark_results(times):
    """
    times: dict retornado pelo benchmark_pipeline
    """
    # -------------------------------
    # SUCCESS RATE (TAXA DE SUCESSO)
    # -------------------------------
    N = times.get("N", 0)
    success_c = times.get("success_c", 0)
    success_rrt = times.get("success_rrt", 0)

    if N > 0:
        rate_c = (success_c / N) * 100
        rate_rrt = (success_rrt / N) * 100

        print("\n==================================")
        print("📊 BENCHMARK SUCCESS RATE")
        print("==================================")
        print(f"Total de Cenários (N): {N}")
        print(f"Planner C:   {success_c}/{N} ({rate_c:.1f}%)")
        print(f"Planner RRT: {success_rrt}/{N} ({rate_rrt:.1f}%)")
        print("==================================\n")

        # Gráfico da Taxa de Sucesso
        plt.figure()
        bars = plt.bar(["Planner C", "Planner RRT"], [rate_c, rate_rrt], color=['#4C72B0', '#DD8452'])
        plt.title("Taxa de Sucesso dos Planejadores")
        plt.ylabel("Sucesso (%)")
        plt.ylim(0, 110)  # Margem extra para o texto

        # Adiciona a porcentagem em cima das barras
        for bar in bars:
            yval = bar.get_height()
            plt.text(bar.get_x() + bar.get_width() / 2.0, yval + 2, f"{yval:.1f}%", ha='center', va='bottom',
                     fontweight='bold')

        plt.tight_layout()
        plt.show()

    # Remove chaves que não são listas (como N, success_c, success_rrt) e listas vazias
    clean_times = {k: v for k, v in times.items() if isinstance(v, list) and len(v) > 0}

    if not clean_times:
        print("Sem dados válidos de tempo para plotar.")
        return

    # -------------------------------
    # BOXPLOT (distribuição)
    # -------------------------------
    plt.figure()
    plt.boxplot(clean_times.values(), labels=clean_times.keys())
    plt.title("Distribuição dos Tempos por Etapa")
    plt.ylabel("Tempo (s)")
    plt.xticks(rotation=30)
    plt.tight_layout()
    plt.show()

    # -------------------------------
    # MÉDIA POR ETAPA
    # -------------------------------
    means = [sum(v) / len(v) for v in clean_times.values()]

    plt.figure()
    plt.bar(clean_times.keys(), means)
    plt.title("Tempo Médio por Etapa")
    plt.ylabel("Tempo (s)")
    plt.xticks(rotation=30)
    plt.tight_layout()
    plt.show()

    # -------------------------------
    # HISTOGRAMA (TOTAL)
    # -------------------------------
    if "total_c" in clean_times or "total_rrt" in clean_times:
        plt.figure()
        if "total_c" in clean_times:
            plt.hist(clean_times["total_c"], bins=20, alpha=0.5, label='Total C')
        if "total_rrt" in clean_times:
            plt.hist(clean_times["total_rrt"], bins=20, alpha=0.5, label='Total RRT')

        plt.title("Distribuição do Tempo Total")
        plt.xlabel("Tempo (s)")
        plt.ylabel("Frequência")
        plt.legend()
        plt.tight_layout()
        plt.show()


def run_pipeline(planner_class, world, point_a, point_b, vi, vmax, umin, umax):
    t_total_start = perf_counter()

    # ---------------- PLANNER ----------------
    t0 = perf_counter()
    planner = planner_class(world=world)
    path = planner.plan(point_a, point_b)
    t1 = perf_counter()

    if not path:
        return None

    t_planner = t1 - t0

    # ---------------- STEER ----------------
    t0 = perf_counter()
    x0 = State2D(PhaseState(path[0].x, vi.x), PhaseState(path[0].y, vi.y))
    limits = AccelLimits(umin, umax, vmax=vmax)
    steer = Steer2D(limits)

    seq = steer.steer_list(path, vi)
    steer_path = seq.integrate_list(x0)
    t1 = perf_counter()

    t_steer = t1 - t0

    # ---------------- OTIMIZAÇÃO ----------------
    t0 = perf_counter()
    opt = BangBangOptimizer(steer, new_no_collision, world)
    result = opt.optimize(x0, seq)
    t1 = perf_counter()

    t_opt = t1 - t0

    # ---------------- INTEGRAÇÃO ----------------
    t0 = perf_counter()
    optimized_path = result.integrate_list(x0)
    t1 = perf_counter()

    t_integration = t1 - t0

    t_total = perf_counter() - t_total_start

    return {
        "path": path,
        "optimized_path": [Point(p.x.q, p.y.q) for p in optimized_path],
        "times": (t_planner, t_steer, t_opt, t_integration, t_total)
    }


def benchmark_pipeline(N):
    vmax = 3
    umax = Vector(0.1, 0.1)
    umin = Vector(-0.1, -0.1)
    vi = Vector(0, 0)

    boundaries = Quadrilateral(vertices=[
        Point(0.0, 0.0),
        Point(24.0, 0.0),
        Point(24.0, 20.0),
        Point(0.0, 20.0),
    ])

    worlds = []
    for n in range(N):
        point_a, point_b, obstacles = generate_random_world(
            boundaries,
            n_circles=32,
            n_quads=13,
            n_stadiums=0
        )
        world = World(obstacles=obstacles, boundaries=boundaries)
        worlds.append((point_a, point_b, world))

    times = {
        "N": N,  # Guarda o total de simulações
        "success_c": 0,  # Contador de sucesso C
        "success_rrt": 0,  # Contador de sucesso RRT

        "planner_c": [],
        "planner_rrt": [],

        "steer_c": [],
        "steer_rrt": [],

        "optimizer_c": [],
        "optimized_rrt": [],

        "integration_c": [],
        "integration_rrt": [],

        "total_c": [],
        "total_rrt": []
    }

    results = []

    for i in range(N):
        print(f"Executando pipeline {i + 1}/{N}...")
        point_a, point_b, world = worlds[i]

        res_c = run_pipeline(
            PathPlanner, world, point_a, point_b, vi, vmax, umin, umax
        )

        if res_c:
            times["success_c"] += 1  # Incrementa em caso de sucesso
            t_planner, t_steer, t_opt, t_int, t_total = res_c["times"]

            times["planner_c"].append(t_planner)
            times["steer_c"].append(t_steer)
            times["optimizer_c"].append(t_opt)
            times["integration_c"].append(t_int)
            times["total_c"].append(t_total)
        else:
            print("Falha no Planner C para os obstáculos e pontos:")
            print([obs for obs in world.obstacles])
            print(point_a, point_b)

        # =========================
        # RRT
        # =========================
        res_rrt = run_pipeline(
            RRT, world, point_a, point_b, vi, vmax, umin, umax
        )

        if res_rrt:
            times["success_rrt"] += 1  # Incrementa em caso de sucesso
            t_planner, t_steer, t_opt, t_int, t_total = res_rrt["times"]

            times["planner_rrt"].append(t_planner)
            times["steer_rrt"].append(t_steer)
            times["optimized_rrt"].append(t_opt)
            times["integration_rrt"].append(t_int)
            times["total_rrt"].append(t_total)
        else:
            print("Falha no Planner RRT para os obstáculos e pontos:")
            print([obs for obs in world.obstacles])
            print(point_a, point_b)

        # =========================
        # SALVAR RESULTADOS
        # =========================
        results.append({
            "world": world,

            "path_c": res_c["path"] if res_c else None,
            "opt_c": res_c["optimized_path"] if res_c else None,

            "path_rrt": res_rrt["path"] if res_rrt else None,
            "opt_rrt": res_rrt["optimized_path"] if res_rrt else None,
        })

    return times, results

def plot_paths_comparison(results, max_plots=None):

    if not results:
        print("Sem resultados para plotar.")
        return

    plotted = 0

    for i, data in enumerate(results):
        if max_plots is not None and plotted >= max_plots:
            break

        world = data["world"]
        #opt_path = data["optimized_path"]

        fig, ax = plt.subplots()

        # ---------------------------
        # Boundaries
        # ---------------------------
        if world.boundaries:
            verts = world.boundaries.vertices
            xs = [p.x for p in verts] + [verts[0].x]
            ys = [p.y for p in verts] + [verts[0].y]
            ax.plot(xs, ys, linestyle='--')

        # ---------------------------
        # Obstáculos
        # ---------------------------
        for obs in world.obstacles:

            if obs.__class__.__name__ == "Circle":
                circle = plt.Circle((obs.center.x, obs.center.y),
                                    obs.radius,
                                    fill=False)
                ax.add_patch(circle)

            elif obs.__class__.__name__ == "Quadrilateral":
                verts = obs.vertices
                xs = [p.x for p in verts] + [verts[0].x]
                ys = [p.y for p in verts] + [verts[0].y]
                ax.plot(xs, ys)

                # --- Stadium ---
            elif obs.__class__.__name__ == "Stadium":
                p1, p2 = obs.vertices
                r = obs.radius

                # vetor direção
                dx = p2.x - p1.x
                dy = p2.y - p1.y
                length = np.hypot(dx, dy)

                if length == 0:
                    continue

                ux, uy = dx / length, dy / length
                # vetor perpendicular
                px, py = -uy, ux

                # retângulo central
                corners = [
                    (p1.x + px * r, p1.y + py * r),
                    (p2.x + px * r, p2.y + py * r),
                    (p2.x - px * r, p2.y - py * r),
                    (p1.x - px * r, p1.y - py * r),
                    (p1.x + px * r, p1.y + py * r),
                ]
                xs, ys = zip(*corners)
                ax.plot(xs, ys)

                # semicirculos
                theta = np.linspace(0, np.pi, 50)

                # lado p1
                angle = np.arctan2(uy, ux)
                x1 = p1.x + r * np.cos(theta + angle + np.pi / 2)
                y1 = p1.y + r * np.sin(theta + angle + np.pi / 2)
                ax.plot(x1, y1)

                # lado p2
                x2 = p2.x + r * np.cos(theta + angle - np.pi / 2)
                y2 = p2.y + r * np.sin(theta + angle - np.pi / 2)
                ax.plot(x2, y2)


        if data["path_c"]:
            xs = [p.x for p in data["path_c"]]
            ys = [p.y for p in data["path_c"]]
            ax.plot(xs, ys, '--', label="A*")

        # RRT
        if data["path_rrt"]:
            xs = [p.x for p in data["path_rrt"]]
            ys = [p.y for p in data["path_rrt"]]
            ax.plot(xs, ys, ':', label="RRT")

        # OTIMIZADOS
        if data["opt_c"]:
            xs = [p.x for p in data["opt_c"]]
            ys = [p.y for p in data["opt_c"]]
            ax.plot(xs, ys, label="A* opt")

        if data["opt_rrt"]:
            xs = [p.x for p in data["opt_rrt"]]
            ys = [p.y for p in data["opt_rrt"]]
            ax.plot(xs, ys, label="RRT opt")

        ax.legend()

        # start/goal
        #ax.scatter(path[0].x, path[0].y, s=100)
        #ax.scatter(path[-1].x, path[-1].y, s=100)
        try:
            ax.set_title(f"Execução {i} | path_c={len(data["opt_c"])} pts | opt={len(data["opt_rrt"])} pts")
        except:
            pass
        ax.set_aspect('equal', adjustable='box')
        ax.grid()

        plt.show()

        plotted += 1

    print(f"\nPlots gerados: {plotted}")

if __name__ == "__main__":
    times, results = benchmark_pipeline(30)
    #print(len(times["planner"]))
    #print(np.mean(times["planner"]))
    print(times)
    plot_benchmark_results(times)
    plot_paths_comparison(results)