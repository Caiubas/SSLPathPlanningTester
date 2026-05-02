import json
import math
import os
import random
import statistics
import statistics as stats
from datetime import datetime, timezone
from time import perf_counter

import numpy as np

import plot
from main import no_collision, World, generate_random_world, Point, Quadrilateral
from c_path import PathPlanner

import matplotlib.pyplot as plt

from new_bboptimizer import *
from pipeline import new_no_collision

from rrt import RRT
from astar import AStar  # <-- A* adicionado


def path_length(path) -> float:
    """
    Calcula o comprimento euclidiano total de um caminho.

    Aceita tanto uma lista de Point(x, y) (caminho do planner)
    como uma lista de State2D (caminho integrado),
    extraindo as coordenadas posicionais em ambos os casos.
    """
    if len(path) < 2:
        return 0.0

    first = path[0]
    if hasattr(first, 'x') and hasattr(first.x, 'q'):
        coords = [(s.x.q, s.y.q) for s in path]
    else:
        coords = [(p.x, p.y) for p in path]

    total = 0.0
    for i in range(1, len(coords)):
        dx = coords[i][0] - coords[i - 1][0]
        dy = coords[i][1] - coords[i - 1][1]
        total += (dx * dx + dy * dy) ** 0.5
    return total


# ---------------------------------------------------------------------------
# Helpers de formatação
# ---------------------------------------------------------------------------

PLANNER_COLORS = {
    "C":    "#4C72B0",
    "RRT":  "#DD8452",
    "AStar":"#55A868",
}

def _mean(lst):
    return sum(lst) / len(lst) if lst else 0.0

def _bar_labels(ax, bars, fmt="{:.2f}"):
    for bar in bars:
        h = bar.get_height()
        ax.text(
            bar.get_x() + bar.get_width() / 2.0,
            h * 1.01 if h > 0 else 0.01,
            fmt.format(h),
            ha='center', va='bottom', fontweight='bold', fontsize=9,
        )


# ---------------------------------------------------------------------------
# _savefig  — guarda e opcionalmente mostra um plot
# ---------------------------------------------------------------------------

# Directório global para guardar os plots (pode ser sobrescrito pelo caller)
_PLOT_OUTPUT_DIR: str = "benchmark_plots"
_PLOT_COUNTER: int = 0          # garante ordem nos nomes dos ficheiros
_PLOT_SHOW: bool = True          # se False, só guarda (útil em modo headless)


def _savefig(fig: plt.Figure, name: str, *, subdir: str = "") -> str:
    """
    Guarda *fig* em PNG com nome sugestivo dentro de _PLOT_OUTPUT_DIR.

    Parâmetros
    ----------
    fig    : figura matplotlib
    name   : nome base (sem extensão); espaços → underscore, barras removidas
    subdir : sub-directório opcional dentro de _PLOT_OUTPUT_DIR

    Devolve
    -------
    str — caminho absoluto do ficheiro gravado
    """
    global _PLOT_COUNTER
    _PLOT_COUNTER += 1

    safe_name = name.replace(" ", "_").replace("/", "-").replace("\\", "-")
    out_dir   = os.path.join(_PLOT_OUTPUT_DIR, subdir) if subdir else _PLOT_OUTPUT_DIR
    os.makedirs(out_dir, exist_ok=True)

    filename  = f"{_PLOT_COUNTER:03d}_{safe_name}.png"
    filepath  = os.path.join(out_dir, filename)

    fig.savefig(filepath, dpi=150, bbox_inches="tight")
    print(f"  📸  Plot guardado: {filepath}")

    if _PLOT_SHOW:
        plt.show()
    else:
        plt.close(fig)

    return os.path.abspath(filepath)


# ---------------------------------------------------------------------------
# plot_benchmark_results  — versão ampliada
# ---------------------------------------------------------------------------

def plot_benchmark_results(times, planners=("C", "RRT", "AStar"), *, plot_subdir: str = ""):
    """
    times : dict retornado por benchmark_pipeline.
    planners : quais planners foram incluídos no benchmark.
    plot_subdir : sub-directório dentro de _PLOT_OUTPUT_DIR para este benchmark.
    """

    N = times.get("N", 0)

    # ── 1. TAXA DE SUCESSO ──────────────────────────────────────────────────
    success_vals = [times.get(f"success_{p.lower()}", 0) for p in planners]
    rate_vals    = [(s / N * 100) if N > 0 else 0 for s in success_vals]

    print("\n==================================")
    print("📊 BENCHMARK SUCCESS RATE")
    print("==================================")
    print(f"Total de Cenários (N): {N}")
    for p, s, r in zip(planners, success_vals, rate_vals):
        print(f"Planner {p:5s}: {s}/{N} ({r:.1f}%)")
    print("==================================\n")

    fig, ax = plt.subplots(figsize=(7, 4))
    colors = [PLANNER_COLORS.get(p, "#999") for p in planners]
    bars = ax.bar(planners, rate_vals, color=colors)
    ax.set_title("Taxa de Sucesso dos Planejadores")
    ax.set_ylabel("Sucesso (%)")
    ax.set_ylim(0, 115)
    _bar_labels(ax, bars, "{:.1f}%")
    fig.tight_layout()
    _savefig(fig, "taxa_de_sucesso", subdir=plot_subdir)

    # ── 2. COMPRIMENTO MÉDIO DO CAMINHO ────────────────────────────────────
    len_data  = {p: times.get(f"path_length_{p.lower()}", []) for p in planners}
    len_means = [_mean(len_data[p]) for p in planners]

    for p in planners:
        d = len_data[p]
        if d:
            print(f"Comprimento do caminho {p}: soma={sum(d):.3f}  média={_mean(d):.3f}")

    fig, ax = plt.subplots(figsize=(7, 4))
    bars = ax.bar(
        [p for p in planners if len_data[p]],
        [_mean(len_data[p]) for p in planners if len_data[p]],
        color=[PLANNER_COLORS.get(p, "#999") for p in planners if len_data[p]],
    )
    ax.set_title("Comprimento Médio do Caminho (Waypoints)")
    ax.set_ylabel("Comprimento (m)")
    _bar_labels(ax, bars, "{:.2f} m")
    fig.tight_layout()
    _savefig(fig, "comprimento_medio_caminho", subdir=plot_subdir)

    # ── 3. TEMPO DE GERAÇÃO DE TRAJETÓRIA ──────────────────────────────────

    pipeline_keys = {
        "C":     ("planner_c",   "steer_c",   "optimizer_c",  "integration_c",  "total_c"),
        "RRT":   ("planner_rrt", "steer_rrt", "optimized_rrt","integration_rrt","total_rrt"),
        "AStar": ("planner_astar","steer_astar","optimizer_astar","integration_astar","total_astar"),
    }

    # 3a. Tempo médio de GERAÇÃO (total do pipeline) por planner — barras
    gen_means = []
    gen_labels = []
    for p in planners:
        key = pipeline_keys[p][4]
        d = times.get(key, [])
        if d:
            gen_labels.append(p)
            gen_means.append(_mean(d))

    if gen_means:
        fig, ax = plt.subplots(figsize=(7, 4))
        bars = ax.bar(gen_labels, gen_means,
                      color=[PLANNER_COLORS.get(p, "#999") for p in gen_labels])
        ax.set_title("Tempo Médio de Geração de Trajetória (pipeline completo)")
        ax.set_ylabel("Tempo (s)")
        _bar_labels(ax, bars, "{:.3f} s")
        fig.tight_layout()
        _savefig(fig, "tempo_medio_geracao_pipeline", subdir=plot_subdir)

    # 3b. Detalhe das etapas de geração (stacked bar)
    etapa_labels = ["Planner", "Steer", "Optimizer", "Integração"]
    etapa_idxs   = [0, 1, 2, 3]

    fig, ax = plt.subplots(figsize=(8, 4))
    x = np.arange(len(planners))
    bottoms = np.zeros(len(planners))
    etapa_colors = ["#4C72B0", "#DD8452", "#55A868", "#C44E52"]

    for ei, (elabel, eidx) in enumerate(zip(etapa_labels, etapa_idxs)):
        vals = []
        for p in planners:
            key = pipeline_keys[p][eidx]
            d = times.get(key, [])
            vals.append(_mean(d) if d else 0.0)
        bars = ax.bar(x, vals, bottom=bottoms, label=elabel, color=etapa_colors[ei])
        bottoms += np.array(vals)

    ax.set_xticks(x)
    ax.set_xticklabels(planners)
    ax.set_title("Decomposição do Tempo de Geração por Etapa")
    ax.set_ylabel("Tempo (s)")
    ax.legend()
    fig.tight_layout()
    _savefig(fig, "decomposicao_tempo_geracao_etapas", subdir=plot_subdir)

    # 3c. Boxplot do tempo total de geração
    gen_box_data   = []
    gen_box_labels = []
    for p in planners:
        key = pipeline_keys[p][4]
        d = times.get(key, [])
        if d:
            gen_box_data.append(d)
            gen_box_labels.append(p)

    if gen_box_data:
        fig, ax = plt.subplots(figsize=(7, 4))
        bp = ax.boxplot(gen_box_data, labels=gen_box_labels,
                        patch_artist=True, notch=False)
        for patch, p in zip(bp['boxes'], gen_box_labels):
            patch.set_facecolor(PLANNER_COLORS.get(p, "#999"))
            patch.set_alpha(0.6)
        ax.set_title("Distribuição do Tempo de Geração de Trajetória")
        ax.set_ylabel("Tempo (s)")
        fig.tight_layout()
        _savefig(fig, "boxplot_tempo_geracao", subdir=plot_subdir)

    # ── 4. TEMPO DA PRÓPRIA TRAJETÓRIA ──────────────────────────────────────

    traj_keys = {
        "C":     ("traj_time_pre_c",     "traj_time_post_c"),
        "RRT":   ("traj_time_pre_rrt",   "traj_time_post_rrt"),
        "AStar": ("traj_time_pre_astar", "traj_time_post_astar"),
    }

    # 4a. Duração média da trajetória antes vs depois — barras agrupadas
    pre_means  = []
    post_means = []
    traj_labels = []
    for p in planners:
        pre  = times.get(traj_keys[p][0], [])
        post = times.get(traj_keys[p][1], [])
        if pre and post:
            traj_labels.append(p)
            pre_means.append(_mean(pre))
            post_means.append(_mean(post))

    if traj_labels:
        x = np.arange(len(traj_labels))
        w = 0.35
        fig, ax = plt.subplots(figsize=(8, 4))
        bars_pre  = ax.bar(x - w/2, pre_means,  w, label="Antes da opt",  alpha=0.85,
                           color=[PLANNER_COLORS.get(p, "#999") for p in traj_labels])
        bars_post = ax.bar(x + w/2, post_means, w, label="Depois da opt", alpha=0.55,
                           color=[PLANNER_COLORS.get(p, "#999") for p in traj_labels],
                           hatch='//')
        _bar_labels(ax, bars_pre,  "{:.3f} s")
        _bar_labels(ax, bars_post, "{:.3f} s")
        ax.set_xticks(x)
        ax.set_xticklabels(traj_labels)
        ax.set_title("Duração Média da Trajetória — Antes vs Depois da Optimização")
        ax.set_ylabel("Duração (s)")
        ax.legend()
        fig.tight_layout()
        _savefig(fig, "duracao_trajetoria_pre_vs_post_opt", subdir=plot_subdir)

    # 4b. Redução média da duração da trajetória
    reduction_labels, reductions = [], []
    for p in planners:
        pre  = times.get(traj_keys[p][0], [])
        post = times.get(traj_keys[p][1], [])
        if pre and post:
            reduction_labels.append(p)
            reductions.append(_mean([a - b for a, b in zip(pre, post)]))

    if reductions:
        fig, ax = plt.subplots(figsize=(7, 4))
        bars = ax.bar(reduction_labels, reductions,
                      color=[PLANNER_COLORS.get(p, "#999") for p in reduction_labels])
        ax.set_title("Redução Média da Duração da Trajetória pelo Optimizador")
        ax.set_ylabel("Redução (s)")
        _bar_labels(ax, bars, "{:.3f} s")
        fig.tight_layout()
        _savefig(fig, "reducao_media_duracao_trajetoria", subdir=plot_subdir)

    # 4c. Série temporal da duração da trajetória antes vs depois — um gráfico por planner
    for p in planners:
        pre  = times.get(traj_keys[p][0], [])
        post = times.get(traj_keys[p][1], [])
        if not pre or not post:
            continue
        print(f"\nDuração de trajetória {p}:")
        print(f"  Antes  da opt: soma={sum(pre):.3f} s  média={_mean(pre):.3f} s")
        print(f"  Depois da opt: soma={sum(post):.3f} s  média={_mean(post):.3f} s")
        diff = [a - b for a, b in zip(pre, post)]
        print(f"  Redução:       média={_mean(diff):.3f} s  ({100*sum(diff)/sum(pre):.1f}% de ganho)")

        x = range(len(pre))
        fig, ax = plt.subplots(figsize=(9, 4))
        ax.plot(x, pre,  label="Antes da opt",  linestyle='--', marker='o', markersize=3,
                color=PLANNER_COLORS.get(p, "#999"))
        ax.plot(x, post, label="Depois da opt", linestyle='-',  marker='s', markersize=3,
                color=PLANNER_COLORS.get(p, "#999"), alpha=0.6)
        ax.fill_between(x, post, pre, alpha=0.12, label="Redução",
                        color=PLANNER_COLORS.get(p, "#999"))
        ax.set_title(f"Duração da Trajetória por Cenário — Planner {p}")
        ax.set_xlabel("Cenário")
        ax.set_ylabel("Duração (s)")
        ax.legend()
        fig.tight_layout()
        _savefig(fig, f"serie_duracao_trajetoria_{p.lower()}", subdir=plot_subdir)

    # 4d. Boxplot da duração da trajetória (antes e depois, todos os planners)
    box_data_pre  = []
    box_data_post = []
    box_labels_pre  = []
    box_labels_post = []
    for p in planners:
        pre  = times.get(traj_keys[p][0], [])
        post = times.get(traj_keys[p][1], [])
        if pre:
            box_data_pre.append(pre);   box_labels_pre.append(f"{p}\npré")
        if post:
            box_data_post.append(post); box_labels_post.append(f"{p}\npós")

    if box_data_pre:
        all_data   = box_data_pre + box_data_post
        all_labels = box_labels_pre + box_labels_post
        fig, ax = plt.subplots(figsize=(10, 4))
        bp = ax.boxplot(all_data, labels=all_labels, patch_artist=True, notch=False)
        n_pre = len(box_data_pre)
        for i, (patch, lbl) in enumerate(zip(bp['boxes'], all_labels)):
            p_name = lbl.split('\n')[0]
            patch.set_facecolor(PLANNER_COLORS.get(p_name, "#999"))
            patch.set_alpha(0.7 if i < n_pre else 0.35)
            if i >= n_pre:
                patch.set_hatch('//')
        ax.set_title("Distribuição da Duração da Trajetória — Antes e Depois da Optimização")
        ax.set_ylabel("Duração (s)")
        fig.tight_layout()
        _savefig(fig, "boxplot_duracao_trajetoria_pre_pos", subdir=plot_subdir)

    # ── 5. BOXPLOT GERAL de todas as métricas de tempo de pipeline ──────────
    clean_times = {k: v for k, v in times.items()
                   if isinstance(v, list) and len(v) > 0
                   and not k.startswith("traj_time")
                   and not k.startswith("path_length")}

    if clean_times:
        fig, ax = plt.subplots(figsize=(max(8, len(clean_times) * 0.8), 5))
        ax.boxplot(clean_times.values(), labels=clean_times.keys())
        ax.set_title("Distribuição dos Tempos de Pipeline por Etapa")
        ax.set_ylabel("Tempo (s)")
        ax.tick_params(axis='x', rotation=35)
        fig.tight_layout()
        _savefig(fig, "boxplot_geral_tempos_pipeline", subdir=plot_subdir)


# ---------------------------------------------------------------------------
# save_benchmark_json  — serializa métricas + configuração para ficheiro JSON
# ---------------------------------------------------------------------------

def _list_stats(lst: list) -> dict:
    """Devolve um sub-dicionário com estatísticas descritivas de uma lista."""
    if not lst:
        return {"n": 0, "mean": None, "std": None, "min": None,
                "max": None, "median": None, "sum": None}
    return {
        "n":      len(lst),
        "mean":   statistics.mean(lst),
        "std":    statistics.pstdev(lst),
        "min":    min(lst),
        "max":    max(lst),
        "median": statistics.median(lst),
        "sum":    sum(lst),
    }


def save_benchmark_json(
    times: dict,
    *,
    n_scenarios: int,
    n_circles: int,
    n_quads: int,
    n_stadiums: int,
    vmax: float,
    umax_x: float,
    umax_y: float,
    umin_x: float,
    umin_y: float,
    vi_x: float,
    vi_y: float,
    only_shared_success: bool,
    random_seed: int | None,
    planners: tuple | list = ("C", "RRT", "AStar"),
    output_path: str = "benchmark_results.json",
) -> str:
    suffix_map = {
        "C":     "c",
        "RRT":   "rrt",
        "AStar": "astar",
    }
    opt_key_map = {
        "c":     "optimizer_c",
        "rrt":   "optimized_rrt",
        "astar": "optimizer_astar",
    }

    config = {
        "timestamp_utc":       datetime.now(timezone.utc).isoformat(),
        "random_seed":         random_seed,
        "n_scenarios_requested": n_scenarios,
        "n_scenarios_counted":   times.get("N", 0),
        "only_shared_success": only_shared_success,
        "planners":            list(planners),
        "obstacles_per_scenario": {
            "circles":  n_circles,
            "quads":    n_quads,
            "stadiums": n_stadiums,
            "total":    n_circles + n_quads + n_stadiums,
        },
        "kinematics": {
            "vmax":  vmax,
            "umax":  {"x": umax_x, "y": umax_y},
            "umin":  {"x": umin_x, "y": umin_y},
            "v_initial": {"x": vi_x, "y": vi_y},
        },
    }

    planner_results = {}
    for p in planners:
        s = suffix_map.get(p, p.lower())

        n_success = times.get(f"success_{s}", 0)
        n_counted = times.get("N", 0)

        planner_results[p] = {
            "success_count":      n_success,
            "success_rate_pct":   round(n_success / n_counted * 100, 2) if n_counted else None,
            "path_length_m":      _list_stats(times.get(f"path_length_{s}", [])),
            "generation_time_s": {
                "planner":     _list_stats(times.get(f"planner_{s}", [])),
                "steer":       _list_stats(times.get(f"steer_{s}", [])),
                "optimizer":   _list_stats(times.get(opt_key_map.get(s, f"optimizer_{s}"), [])),
                "integration": _list_stats(times.get(f"integration_{s}", [])),
                "total":       _list_stats(times.get(f"total_{s}", [])),
            },
            "trajectory_duration_s": {
                "pre_optimization":  _list_stats(times.get(f"traj_time_pre_{s}",  [])),
                "post_optimization": _list_stats(times.get(f"traj_time_post_{s}", [])),
                "reduction": _list_stats(
                    [a - b for a, b in zip(
                        times.get(f"traj_time_pre_{s}",  []),
                        times.get(f"traj_time_post_{s}", []),
                    )]
                ),
            },
            "raw_series": {
                "path_length_m":          times.get(f"path_length_{s}", []),
                "generation_total_s":     times.get(f"total_{s}", []),
                "generation_planner_s":   times.get(f"planner_{s}", []),
                "generation_steer_s":     times.get(f"steer_{s}", []),
                "generation_optimizer_s": times.get(opt_key_map.get(s, f"optimizer_{s}"), []),
                "generation_integration_s": times.get(f"integration_{s}", []),
                "traj_duration_pre_s":    times.get(f"traj_time_pre_{s}",  []),
                "traj_duration_post_s":   times.get(f"traj_time_post_{s}", []),
            },
        }

    comparison = {}
    planner_list = list(planners)
    for i in range(len(planner_list)):
        for j in range(i + 1, len(planner_list)):
            pa, pb = planner_list[i], planner_list[j]
            key = f"{pa}_vs_{pb}"

            def _ratio(a, b):
                return round(a / b, 4) if b else None

            def _diff(a, b):
                return round(a - b, 6) if (a is not None and b is not None) else None

            ma_gen  = planner_results[pa]["generation_time_s"]["total"]["mean"]
            mb_gen  = planner_results[pb]["generation_time_s"]["total"]["mean"]
            ma_traj = planner_results[pa]["trajectory_duration_s"]["post_optimization"]["mean"]
            mb_traj = planner_results[pb]["trajectory_duration_s"]["post_optimization"]["mean"]
            ma_len  = planner_results[pa]["path_length_m"]["mean"]
            mb_len  = planner_results[pb]["path_length_m"]["mean"]

            comparison[key] = {
                "generation_time_ratio":   _ratio(ma_gen,  mb_gen),
                "generation_time_diff_s":  _diff(ma_gen,   mb_gen),
                "traj_duration_ratio":     _ratio(ma_traj, mb_traj),
                "traj_duration_diff_s":    _diff(ma_traj,  mb_traj),
                "path_length_ratio":       _ratio(ma_len,  mb_len),
                "path_length_diff_m":      _diff(ma_len,   mb_len),
                "success_rate_diff_pct":   _diff(
                    planner_results[pa]["success_rate_pct"],
                    planner_results[pb]["success_rate_pct"],
                ),
            }

    document = {
        "benchmark_config":   config,
        "planner_results":    planner_results,
        "planner_comparison": comparison,
    }

    abs_path = os.path.abspath(output_path)
    with open(abs_path, "w", encoding="utf-8") as f:
        json.dump(document, f, indent=2, ensure_ascii=False)

    print(f"\n✅  Resultados gravados em: {abs_path}")
    return abs_path


# ---------------------------------------------------------------------------
# run_pipeline
# ---------------------------------------------------------------------------

def run_pipeline(planner_class, world, point_a, point_b, vi, vmax, umin, umax):
    t_total_start = perf_counter()

    t0 = perf_counter()
    planner = planner_class(world=world)
    path = planner.plan(point_a, point_b)
    t1 = perf_counter()

    if not path:
        return None

    t_planner = t1 - t0

    t0 = perf_counter()
    x0 = State2D(PhaseState(path[0].x, vi.x), PhaseState(path[0].y, vi.y))
    limits = AccelLimits(umin, umax, vmax=vmax)
    steer = Steer2D(limits)

    seq = steer.steer_list(path, vi)
    steer_path = seq.integrate_list(x0)
    t1 = perf_counter()

    t_steer = t1 - t0
    traj_time_pre = seq.total_time()

    t0 = perf_counter()
    opt = BangBangOptimizer(steer, new_no_collision, world)
    result = opt.optimize(x0, seq)
    t1 = perf_counter()

    t_opt = t1 - t0
    traj_time_post = result.total_time()

    t0 = perf_counter()
    optimized_path = result.integrate_list(x0)
    t1 = perf_counter()

    t_integration = t1 - t0
    t_total = perf_counter() - t_total_start
    p_length = path_length(path)

    return {
        "path": path,
        "optimized_path": [Point(p.x.q, p.y.q) for p in optimized_path],
        "times": (t_planner, t_steer, t_opt, t_integration, t_total),
        "traj_time_pre": traj_time_pre,
        "traj_time_post": traj_time_post,
        "path_length": p_length,
    }


# ---------------------------------------------------------------------------
# benchmark_pipeline
# ---------------------------------------------------------------------------

def benchmark_pipeline(N, only_shared_success=False,
                        n_circles=32, n_quads=13, n_stadiums=0,
                        n_scenario_plots: int = 20):
    vmax = 3
    umax = Vector(3, 3)
    umin = Vector(-3, -3)
    vi   = Vector(0, 0)

    _config = dict(
        n_scenarios=N,
        n_circles=n_circles,
        n_quads=n_quads,
        n_stadiums=n_stadiums,
        vmax=vmax,
        umax_x=umax.x, umax_y=umax.y,
        umin_x=umin.x, umin_y=umin.y,
        vi_x=vi.x,     vi_y=vi.y,
        only_shared_success=only_shared_success,
    )

    boundaries = Quadrilateral(vertices=[
        Point(0.0, 0.0),
        Point(13.4, 0.0),
        Point(13.4, 10.4),
        Point(0.0, 10.4),
    ])

    worlds = []
    for n in range(N):
        point_a, point_b, obstacles = generate_random_world(
            boundaries,
            n_circles=n_circles,
            n_quads=n_quads,
            n_stadiums=n_stadiums,
        )
        world = World(obstacles=obstacles, boundaries=boundaries)
        worlds.append((point_a, point_b, world))

    times = {
        "N": N,
        "success_c":     0,
        "success_rrt":   0,
        "success_astar": 0,

        "planner_c":        [], "steer_c":        [], "optimizer_c":       [], "integration_c":       [], "total_c":       [],
        "planner_rrt":      [], "steer_rrt":      [], "optimized_rrt":     [], "integration_rrt":     [], "total_rrt":     [],
        "planner_astar":    [], "steer_astar":    [], "optimizer_astar":   [], "integration_astar":   [], "total_astar":   [],

        "path_length_c":     [],
        "path_length_rrt":   [],
        "path_length_astar": [],

        "traj_time_pre_c":      [], "traj_time_post_c":      [],
        "traj_time_pre_rrt":    [], "traj_time_post_rrt":    [],
        "traj_time_pre_astar":  [], "traj_time_post_astar":  [],
    }

    results = []

    planner_map = [
        ("c",     PathPlanner),
        ("rrt",   RRT),
        ("astar", AStar),
    ]

    for i in range(N):
        print(f"Executando pipeline {i + 1}/{N}...")
        point_a, point_b, world = worlds[i]

        res = {}
        for suffix, cls in planner_map:
            r = run_pipeline(cls, world, point_a, point_b, vi, vmax, umin, umax)
            res[suffix] = r
            if not r:
                print(f"  Falha no Planner {suffix.upper()} — obstáculos: {[obs for obs in world.obstacles]}")
                print(f"  Pontos: {point_a} → {point_b}")

        all_succeeded = all(res[s] is not None for s, _ in planner_map)

        for suffix, _ in planner_map:
            include = bool(res[suffix]) if not only_shared_success else all_succeeded
            if not include:
                continue

            times[f"success_{suffix}"] += 1
            t_planner, t_steer, t_opt, t_int, t_total = res[suffix]["times"]

            opt_key   = "optimizer_c"    if suffix == "c"   else \
                        "optimized_rrt"  if suffix == "rrt" else \
                        "optimizer_astar"

            times[f"planner_{suffix}"].append(t_planner)
            times[f"steer_{suffix}"].append(t_steer)
            times[opt_key].append(t_opt)
            times[f"integration_{suffix}"].append(t_int)
            times[f"total_{suffix}"].append(t_total)
            times[f"path_length_{suffix}"].append(res[suffix]["path_length"])
            times[f"traj_time_pre_{suffix}"].append(res[suffix]["traj_time_pre"])
            times[f"traj_time_post_{suffix}"].append(res[suffix]["traj_time_post"])

        results.append({
            "world":    world,
            "path_c":   res["c"]["path"]           if res["c"]     else None,
            "opt_c":    res["c"]["optimized_path"]  if res["c"]     else None,
            "path_rrt": res["rrt"]["path"]          if res["rrt"]   else None,
            "opt_rrt":  res["rrt"]["optimized_path"] if res["rrt"]  else None,
            "path_astar": res["astar"]["path"]           if res["astar"] else None,
            "opt_astar":  res["astar"]["optimized_path"]  if res["astar"] else None,
            "counted":  all_succeeded if only_shared_success else any(res[s] for s, _ in planner_map),
        })

    if only_shared_success:
        times["N"] = times["success_c"]

    # ── Plots automáticos de cenários ────────────────────────────────────────
    if n_scenario_plots > 0 and results:
        plot_scenarios_sample(
            results,
            n=n_scenario_plots,
            only_counted=only_shared_success,
            plot_subdir="cenarios",
        )

    return times, results, _config


# ---------------------------------------------------------------------------
# _draw_world  — desenha fronteiras e obstáculos num eixo matplotlib
# ---------------------------------------------------------------------------

def _draw_world(ax: plt.Axes, world) -> None:
    """Pinta fronteiras (tracejado cinza) e obstáculos (preenchidos a cinza claro)."""
    # Fronteiras
    if world.boundaries:
        verts = world.boundaries.vertices
        xs = [p.x for p in verts] + [verts[0].x]
        ys = [p.y for p in verts] + [verts[0].y]
        ax.plot(xs, ys, linestyle='--', color='#888888', linewidth=1.2, zorder=1)

    for obs in world.obstacles:
        name = obs.__class__.__name__

        if name == "Circle":
            patch = plt.Circle(
                (obs.center.x, obs.center.y), obs.radius,
                facecolor='#CCCCCC', edgecolor='#666666', linewidth=0.8, zorder=2,
            )
            ax.add_patch(patch)

        elif name == "Quadrilateral":
            verts = obs.vertices
            xs = [p.x for p in verts] + [verts[0].x]
            ys = [p.y for p in verts] + [verts[0].y]
            from matplotlib.patches import Polygon as MplPolygon
            poly = MplPolygon(
                list(zip([p.x for p in verts], [p.y for p in verts])),
                closed=True,
                facecolor='#CCCCCC', edgecolor='#666666', linewidth=0.8, zorder=2,
            )
            ax.add_patch(poly)

        elif name == "Stadium":
            p1, p2 = obs.vertices
            r = obs.radius
            dx = p2.x - p1.x; dy = p2.y - p1.y
            length = np.hypot(dx, dy)
            if length == 0:
                continue
            ux, uy = dx / length, dy / length
            px, py = -uy, ux
            # Corpo rectangular
            corners = [
                (p1.x + px * r, p1.y + py * r), (p2.x + px * r, p2.y + py * r),
                (p2.x - px * r, p2.y - py * r), (p1.x - px * r, p1.y - py * r),
            ]
            from matplotlib.patches import Polygon as MplPolygon
            rect = MplPolygon(corners, closed=True,
                              facecolor='#CCCCCC', edgecolor='#666666',
                              linewidth=0.8, zorder=2)
            ax.add_patch(rect)
            # Semicírculos nas extremidades
            theta = np.linspace(0, np.pi, 60)
            angle = np.arctan2(uy, ux)
            ax.fill(
                p1.x + r * np.cos(theta + angle + np.pi / 2),
                p1.y + r * np.sin(theta + angle + np.pi / 2),
                color='#CCCCCC', zorder=2,
            )
            ax.fill(
                p2.x + r * np.cos(theta + angle - np.pi / 2),
                p2.y + r * np.sin(theta + angle - np.pi / 2),
                color='#CCCCCC', zorder=2,
            )
            # Contornos
            ax.plot(p1.x + r * np.cos(theta + angle + np.pi / 2),
                    p1.y + r * np.sin(theta + angle + np.pi / 2),
                    color='#666666', linewidth=0.8, zorder=3)
            ax.plot(p2.x + r * np.cos(theta + angle - np.pi / 2),
                    p2.y + r * np.sin(theta + angle - np.pi / 2),
                    color='#666666', linewidth=0.8, zorder=3)


# ---------------------------------------------------------------------------
# plot_scenario  — plota um cenário individual com todas as trajetórias
# ---------------------------------------------------------------------------

def plot_scenario(
    data: dict,
    scenario_idx: int,
    *,
    planners: tuple | list = ("C", "RRT", "AStar"),
    show_raw: bool = True,
    plot_subdir: str = "cenarios",
) -> None:
    """
    Gera e guarda um plot detalhado de um único cenário.

    data          : elemento da lista `results` devolvida por benchmark_pipeline
    scenario_idx  : índice do cenário (para o nome do ficheiro e título)
    planners      : quais planners incluir
    show_raw      : se True, desenha também os waypoints crus (tracejado fino)
    plot_subdir   : sub-directório dentro de _PLOT_OUTPUT_DIR
    """
    world = data["world"]

    # Dimensões dinâmicas baseadas no mundo
    if world.boundaries:
        verts = world.boundaries.vertices
        wx = max(p.x for p in verts) - min(p.x for p in verts)
        wy = max(p.y for p in verts) - min(p.y for p in verts)
        fig_w = max(7, wx * 0.65)
        fig_h = max(5, wy * 0.65)
    else:
        fig_w, fig_h = 9, 6

    fig, ax = plt.subplots(figsize=(fig_w, fig_h))

    # ── Mundo ───────────────────────────────────────────────────────────────
    _draw_world(ax, world)

    # ── Trajetórias ─────────────────────────────────────────────────────────
    planner_cfg = {
        "C":     {"raw_key": "path_c",     "opt_key": "opt_c",
                  "raw_ls": "--",  "raw_lw": 0.9, "opt_lw": 2.0,
                  "label_raw": "C (waypoints)", "label_opt": "C (optimizado)"},
        "RRT":   {"raw_key": "path_rrt",   "opt_key": "opt_rrt",
                  "raw_ls": ":",   "raw_lw": 0.9, "opt_lw": 2.0,
                  "label_raw": "RRT (waypoints)", "label_opt": "RRT (optimizado)"},
        "AStar": {"raw_key": "path_astar", "opt_key": "opt_astar",
                  "raw_ls": "-.", "raw_lw": 0.9, "opt_lw": 2.0,
                  "label_raw": "A* (waypoints)", "label_opt": "A* (optimizado)"},
    }

    start_pt = end_pt = None

    for p in planners:
        cfg  = planner_cfg.get(p, {})
        color = PLANNER_COLORS.get(p, "#999999")

        # Waypoints crus
        if show_raw:
            raw = data.get(cfg.get("raw_key", ""))
            if raw:
                ax.plot(
                    [pt.x for pt in raw], [pt.y for pt in raw],
                    linestyle=cfg["raw_ls"], linewidth=cfg["raw_lw"],
                    color=color, alpha=0.45, zorder=4,
                    label=cfg["label_raw"],
                )
                if start_pt is None:
                    start_pt = raw[0]
                    end_pt   = raw[-1]

        # Trajetória optimizada
        opt = data.get(cfg.get("opt_key", ""))
        if opt:
            ax.plot(
                [pt.x for pt in opt], [pt.y for pt in opt],
                linestyle="-", linewidth=cfg["opt_lw"],
                color=color, alpha=0.90, zorder=5,
                label=cfg["label_opt"],
            )
            if start_pt is None:
                start_pt = opt[0]
                end_pt   = opt[-1]

    # ── Marcadores de início / fim ───────────────────────────────────────────
    if start_pt is not None:
        ax.plot(start_pt.x, start_pt.y, 'o', markersize=10,
                color='#2ECC71', markeredgecolor='black', markeredgewidth=0.8,
                zorder=7, label="Início")
        ax.plot(end_pt.x,   end_pt.y,   's', markersize=10,
                color='#E74C3C', markeredgecolor='black', markeredgewidth=0.8,
                zorder=7, label="Fim")
        # Anotações de coordenadas
        ax.annotate(f"({start_pt.x:.1f}, {start_pt.y:.1f})",
                    (start_pt.x, start_pt.y), textcoords="offset points",
                    xytext=(6, 6), fontsize=7, color='#2ECC71', fontweight='bold', zorder=8)
        ax.annotate(f"({end_pt.x:.1f}, {end_pt.y:.1f})",
                    (end_pt.x, end_pt.y), textcoords="offset points",
                    xytext=(6, -12), fontsize=7, color='#E74C3C', fontweight='bold', zorder=8)

    # ── Título com estatísticas por planner ─────────────────────────────────
    stats_parts = []
    for p in planners:
        cfg   = planner_cfg.get(p, {})
        opt   = data.get(cfg.get("opt_key", ""))
        raw   = data.get(cfg.get("raw_key", ""))
        n_opt = len(opt) if opt else 0
        n_raw = len(raw) if raw else 0
        if n_opt:
            stats_parts.append(f"{p}: {n_opt} pts opt / {n_raw} wp")
        else:
            stats_parts.append(f"{p}: ✗")

    n_obs = len(world.obstacles) if world.obstacles else 0
    ax.set_title(
        f"Cenário #{scenario_idx:04d}  |  {n_obs} obstáculos\n" +
        "  |  ".join(stats_parts),
        fontsize=9,
    )

    # ── Acabamentos ─────────────────────────────────────────────────────────
    ax.legend(fontsize=7, loc="upper right", framealpha=0.85)
    ax.set_aspect('equal', adjustable='box')
    ax.grid(True, alpha=0.2, linestyle=':')
    ax.set_xlabel("x (m)", fontsize=8)
    ax.set_ylabel("y (m)", fontsize=8)
    fig.tight_layout()

    _savefig(fig, f"cenario_{scenario_idx:04d}", subdir=plot_subdir)


# ---------------------------------------------------------------------------
# plot_scenarios_sample  — gera N plots a partir da lista results
# ---------------------------------------------------------------------------

def plot_scenarios_sample(
    results: list[dict],
    n: int = 10,
    *,
    planners: tuple | list = ("C", "RRT", "AStar"),
    show_raw: bool = True,
    only_counted: bool = True,
    plot_subdir: str = "cenarios",
    indices: list[int] | None = None,
) -> None:
    """
    Salva plots de cenários com as trajetórias geradas.

    results      : lista devolvida por benchmark_pipeline / _benchmark_pipeline_parametric
    n            : número de cenários a plotar (ignorado se `indices` for fornecido)
    planners     : planners a incluir
    show_raw     : desenhar também os waypoints crus
    only_counted : se True, plota apenas cenários marcados como contabilizados
    plot_subdir  : sub-directório dentro de _PLOT_OUTPUT_DIR
    indices      : lista explícita de índices; se fornecida, ignora `n` e `only_counted`
    """
    if not results:
        print("Sem resultados para plotar.")
        return

    if indices is not None:
        selected = [(idx, results[idx]) for idx in indices if idx < len(results)]
    else:
        pool = [
            (i, d) for i, d in enumerate(results)
            if (not only_counted) or d.get("counted", True)
        ]
        # Distribui os n cenários uniformemente ao longo do pool
        if len(pool) <= n:
            selected = pool
        else:
            step = len(pool) / n
            selected = [pool[int(i * step)] for i in range(n)]

    print(f"\n📸  Gerando {len(selected)} plots de cenários → subdir '{plot_subdir}'")
    for rank, (orig_idx, data) in enumerate(selected, 1):
        print(f"  [{rank}/{len(selected)}] cenário #{orig_idx:04d}", end="\r")
        plot_scenario(
            data,
            scenario_idx=orig_idx,
            planners=planners,
            show_raw=show_raw,
            plot_subdir=plot_subdir,
        )
    print(f"\n✅  {len(selected)} cenários salvos em: "
          f"{os.path.abspath(os.path.join(_PLOT_OUTPUT_DIR, plot_subdir))}")


# ---------------------------------------------------------------------------
# Helpers internos do sweep
# ---------------------------------------------------------------------------

def _extract_metric(times: dict, planner: str, metric: str) -> float | None:
    s = {"C": "c", "RRT": "rrt", "AStar": "astar"}.get(planner, planner.lower())
    opt_key = {"c": "optimizer_c", "rrt": "optimized_rrt", "astar": "optimizer_astar"}.get(s, f"optimizer_{s}")
    N = times.get("N", 0)

    dispatch = {
        "success_rate":           lambda: (times.get(f"success_{s}", 0) / N * 100) if N else None,
        "generation_total":       lambda: _mean(times.get(f"total_{s}", [])) or None,
        "generation_planner":     lambda: _mean(times.get(f"planner_{s}", [])) or None,
        "generation_steer":       lambda: _mean(times.get(f"steer_{s}", [])) or None,
        "generation_optimizer":   lambda: _mean(times.get(opt_key, [])) or None,
        "generation_integration": lambda: _mean(times.get(f"integration_{s}", [])) or None,
        "traj_pre":               lambda: _mean(times.get(f"traj_time_pre_{s}", [])) or None,
        "traj_post":              lambda: _mean(times.get(f"traj_time_post_{s}", [])) or None,
        "traj_reduction":         lambda: _mean(
            [a - b for a, b in zip(times.get(f"traj_time_pre_{s}", []),
                                    times.get(f"traj_time_post_{s}", []))]) or None,
        "path_length":            lambda: _mean(times.get(f"path_length_{s}", [])) or None,
    }

    fn = dispatch.get(metric)
    return fn() if fn else None


_METRIC_LABELS = {
    "success_rate":            ("Taxa de Sucesso",                        "%"),
    "generation_total":        ("Tempo de Geração (total)",               "s"),
    "generation_planner":      ("Tempo de Geração — Planner",             "s"),
    "generation_steer":        ("Tempo de Geração — Steer",               "s"),
    "generation_optimizer":    ("Tempo de Geração — Optimizer",           "s"),
    "generation_integration":  ("Tempo de Geração — Integração",          "s"),
    "traj_pre":                ("Duração da Trajetória (pré-opt.)",        "s"),
    "traj_post":               ("Duração da Trajetória (pós-opt.)",        "s"),
    "traj_reduction":          ("Redução da Duração da Trajetória",        "s"),
    "path_length":             ("Comprimento do Caminho (waypoints)",      "m"),
}

_PARAM_LABELS = {
    "vmax":       "Vmax (m/s)",
    "umax":       "Umax (m/s²)",
    "umin":       "Umin (m/s²)",
    "n_obstacles":"Nº de Obstáculos",
    "n_circles":  "Nº de Círculos",
    "n_quads":    "Nº de Quadriláteros",
    "n_stadiums": "Nº de Stadiums",
}


def _sweep_axis_label(param: str, value) -> str:
    label = _PARAM_LABELS.get(param, param)
    return f"{label}\n= {value}"


# ---------------------------------------------------------------------------
# plot_sweep_results
# ---------------------------------------------------------------------------

def plot_sweep_results(
    sweep_records: list[dict],
    sweep_param: str,
    planners: tuple | list = ("C", "RRT", "AStar"),
    metrics: tuple | list | None = None,
    *,
    plot_subdir: str = "sweep",
):
    if metrics is None:
        metrics = list(_METRIC_LABELS.keys())

    x_vals = [r["param_value"] for r in sweep_records]
    x_label = _PARAM_LABELS.get(sweep_param, sweep_param)

    for metric in metrics:
        m_label, m_unit = _METRIC_LABELS.get(metric, (metric, ""))

        fig, ax = plt.subplots(figsize=(9, 4))
        has_data = False

        for planner in planners:
            y_vals = []
            for rec in sweep_records:
                v = _extract_metric(rec["times"], planner, metric)
                y_vals.append(v)

            if any(v is not None for v in y_vals):
                has_data = True
                y_plot = [v if v is not None else float("nan") for v in y_vals]
                ax.plot(
                    x_vals, y_plot,
                    marker="o", linewidth=2, markersize=6,
                    label=planner,
                    color=PLANNER_COLORS.get(planner, "#999"),
                )
                for xv, yv in zip(x_vals, y_plot):
                    if not (yv != yv):
                        ax.annotate(
                            f"{yv:.2f}",
                            (xv, yv),
                            textcoords="offset points", xytext=(0, 7),
                            ha="center", fontsize=7, color=PLANNER_COLORS.get(planner, "#999"),
                        )

        if not has_data:
            plt.close(fig)
            continue

        ax.set_title(f"{m_label} vs {x_label}")
        ax.set_xlabel(x_label)
        ax.set_ylabel(f"{m_label} ({m_unit})" if m_unit else m_label)
        ax.set_xticks(x_vals)
        ax.legend()
        ax.grid(True, alpha=0.3)
        fig.tight_layout()
        _savefig(fig, f"sweep_linha_{metric}", subdir=plot_subdir)

    # ── Heatmap comparativo ─────────────────────────────────────────────────
    for metric in metrics:
        m_label, m_unit = _METRIC_LABELS.get(metric, (metric, ""))

        matrix = []
        row_labels = []
        for planner in planners:
            row = []
            for rec in sweep_records:
                v = _extract_metric(rec["times"], planner, metric)
                row.append(v if v is not None else float("nan"))
            matrix.append(row)
            row_labels.append(planner)

        flat = [v for row in matrix for v in row if v == v]
        if not flat:
            continue

        matrix_np = np.array(matrix, dtype=float)

        fig, ax = plt.subplots(figsize=(max(6, len(x_vals) * 1.2), max(3, len(planners) * 0.9)))
        im = ax.imshow(matrix_np, aspect="auto", cmap="YlOrRd",
                       vmin=np.nanmin(matrix_np), vmax=np.nanmax(matrix_np))
        plt.colorbar(im, ax=ax, label=f"{m_unit}" if m_unit else "")

        ax.set_xticks(range(len(x_vals)))
        ax.set_xticklabels([str(v) for v in x_vals])
        ax.set_yticks(range(len(row_labels)))
        ax.set_yticklabels(row_labels)
        ax.set_xlabel(x_label)
        ax.set_title(f"Heatmap — {m_label}")

        for ri in range(len(row_labels)):
            for ci in range(len(x_vals)):
                val = matrix_np[ri, ci]
                if val == val:
                    ax.text(ci, ri, f"{val:.2f}", ha="center", va="center",
                            fontsize=8, fontweight="bold",
                            color="white" if val > (np.nanmax(matrix_np) * 0.65) else "black")

        fig.tight_layout()
        _savefig(fig, f"sweep_heatmap_{metric}", subdir=plot_subdir)


# ---------------------------------------------------------------------------
# run_sweep
# ---------------------------------------------------------------------------

def run_sweep(
    sweep_configs: list[dict],
    *,
    n_scenarios: int = 30,
    only_shared_success: bool = True,
    planners: tuple | list = ("C", "RRT", "AStar"),
    random_seed: int | None = 42,
    output_dir: str = "sweep_results",
    plot_individual: bool = False,
    plot_comparison: bool = True,
    metrics_to_plot: list[str] | None = None,
    plots_dir: str | None = None,          # directório raiz dos plots
    show_plots: bool = True,               # False = só guarda, não mostra janelas
    n_scenario_plots: int = 10,            # cenários com trajetórias por run (0 = desligado)
) -> list[dict]:
    """
    Executa uma série de benchmarks com configurações diferentes e agrega
    os resultados num JSON global e em gráficos comparativos.

    Parâmetros
    ----------
    plots_dir         : directório raiz onde os plots serão guardados.
                        Defaults para <output_dir>/plots.
    show_plots        : se False, os plots são apenas guardados em disco
                        (útil em execuções headless / servidor).
    n_scenario_plots  : quantos cenários (com trajetórias) salvar por run.
                        0 desliga esta funcionalidade.
    """
    global _PLOT_OUTPUT_DIR, _PLOT_SHOW, _PLOT_COUNTER

    # Configura directório e comportamento de show
    _PLOT_OUTPUT_DIR = plots_dir if plots_dir else os.path.join(output_dir, "plots")
    _PLOT_SHOW       = show_plots
    _PLOT_COUNTER    = 0   # reinicia contador para este sweep

    os.makedirs(output_dir, exist_ok=True)
    os.makedirs(_PLOT_OUTPUT_DIR, exist_ok=True)

    # Detecta parâmetro variado
    all_keys = set()
    for cfg in sweep_configs:
        all_keys.update(k for k in cfg if k != "label")

    varying = [
        k for k in all_keys
        if len({cfg.get(k) for cfg in sweep_configs}) > 1
    ]
    sweep_param = varying[0] if len(varying) == 1 else "config_index"

    print("=" * 60)
    print(f"🔁  SWEEP  |  {len(sweep_configs)} configurações  |  {n_scenarios} cenários cada")
    print(f"   Parâmetro detectado: {_PARAM_LABELS.get(sweep_param, sweep_param)}")
    print(f"   Planners: {', '.join(planners)}")
    print(f"   Seed: {random_seed}")
    print(f"   Plots → {os.path.abspath(_PLOT_OUTPUT_DIR)}")
    print("=" * 60)

    sweep_records: list[dict] = []
    all_runs_summary: list[dict] = []

    for run_idx, cfg in enumerate(sweep_configs):
        label = cfg.get("label") or f"config_{run_idx:02d}"

        vmax       = cfg.get("vmax",       3.0)
        umax_val   = cfg.get("umax",       3.0)
        umin_val   = cfg.get("umin",      -3.0)
        n_circles  = cfg.get("n_circles",  32)
        n_quads    = cfg.get("n_quads",    13)
        n_stadiums = cfg.get("n_stadiums",  0)

        if sweep_param == "config_index":
            param_value = run_idx
        elif sweep_param in ("umax", "umin"):
            param_value = cfg.get(sweep_param, umax_val if sweep_param == "umax" else umin_val)
        elif sweep_param == "n_obstacles":
            param_value = n_circles + n_quads + n_stadiums
        else:
            param_value = cfg.get(sweep_param)

        print(f"\n{'─'*60}")
        print(f"  Run {run_idx + 1}/{len(sweep_configs)}  —  {label}")
        print(f"  vmax={vmax}  umax=±{umax_val}  circles={n_circles}"
              f"  quads={n_quads}  stadiums={n_stadiums}")
        print(f"{'─'*60}")

        if random_seed is not None:
            random.seed(random_seed + run_idx)

        times, results, pipeline_cfg = _benchmark_pipeline_parametric(
            N=n_scenarios,
            only_shared_success=only_shared_success,
            n_circles=n_circles,
            n_quads=n_quads,
            n_stadiums=n_stadiums,
            vmax=vmax,
            umax_val=umax_val,
            umin_val=umin_val,
        )

        safe_label = label.replace(" ", "_").replace("/", "-")
        json_path = os.path.join(output_dir, f"run_{run_idx:02d}_{safe_label}.json")
        save_benchmark_json(
            times,
            **pipeline_cfg,
            random_seed=random_seed + run_idx if random_seed is not None else None,
            planners=planners,
            output_path=json_path,
        )

        # Gráficos individuais num sub-directório próprio por run
        if plot_individual:
            run_subdir = os.path.join("individual", f"run_{run_idx:02d}_{safe_label}")
            plot_benchmark_results(times, planners=planners, plot_subdir=run_subdir)

        # Plots de cenários com trajetórias
        if n_scenario_plots > 0 and results:
            scenarios_subdir = os.path.join("cenarios", f"run_{run_idx:02d}_{safe_label}")
            plot_scenarios_sample(
                results,
                n=n_scenario_plots,
                planners=planners,
                only_counted=only_shared_success,
                plot_subdir=scenarios_subdir,
            )

        record = {
            "label":       label,
            "config":      {k: v for k, v in cfg.items() if k != "label"},
            "param_value": param_value,
            "times":       times,
            "json_path":   os.path.abspath(json_path),
        }
        sweep_records.append(record)

        summary_entry = {
            "run_index":   run_idx,
            "label":       label,
            "param_value": param_value,
            "config":      record["config"],
            "json_path":   record["json_path"],
            "summary": {},
        }
        for p in planners:
            summary_entry["summary"][p] = {
                m: _extract_metric(times, p, m)
                for m in _METRIC_LABELS
            }
        all_runs_summary.append(summary_entry)

        print(f"\n  Resultados (médias):")
        header = f"  {'Métrica':<35}" + "".join(f"{p:>12}" for p in planners)
        print(header)
        print("  " + "-" * (35 + 12 * len(planners)))
        for m, (mlabel, munit) in _METRIC_LABELS.items():
            row = f"  {mlabel:<35}"
            for p in planners:
                v = _extract_metric(times, p, m)
                row += f"{f'{v:.3f} {munit}':>12}" if v is not None else f"{'—':>12}"
            print(row)

    global_json = {
        "sweep_metadata": {
            "timestamp_utc":      datetime.now(timezone.utc).isoformat(),
            "sweep_param":        sweep_param,
            "sweep_param_label":  _PARAM_LABELS.get(sweep_param, sweep_param),
            "n_configs":          len(sweep_configs),
            "n_scenarios_each":   n_scenarios,
            "only_shared_success": only_shared_success,
            "planners":           list(planners),
            "random_seed":        random_seed,
        },
        "runs": all_runs_summary,
    }
    global_path = os.path.join(output_dir, "sweep_global.json")
    with open(global_path, "w", encoding="utf-8") as f:
        json.dump(global_json, f, indent=2, ensure_ascii=False)
    print(f"\n✅  JSON global do sweep gravado em: {os.path.abspath(global_path)}")

    if plot_comparison and sweep_records:
        print("\n📊  Gerando gráficos comparativos do sweep...")
        plot_sweep_results(
            sweep_records,
            sweep_param=sweep_param,
            planners=planners,
            metrics=metrics_to_plot,
            plot_subdir="comparativo",
        )

    print(f"\n📁  Todos os plots guardados em: {os.path.abspath(_PLOT_OUTPUT_DIR)}")
    return sweep_records


# ---------------------------------------------------------------------------
# _benchmark_pipeline_parametric
# ---------------------------------------------------------------------------

def _benchmark_pipeline_parametric(
    N: int,
    only_shared_success: bool,
    n_circles: int,
    n_quads: int,
    n_stadiums: int,
    vmax: float,
    umax_val: float,
    umin_val: float,
):
    umax = Vector(umax_val,  umax_val)
    umin = Vector(umin_val, umin_val)
    vi   = Vector(0, 0)

    _config = dict(
        n_scenarios=N,
        n_circles=n_circles,
        n_quads=n_quads,
        n_stadiums=n_stadiums,
        vmax=vmax,
        umax_x=umax.x, umax_y=umax.y,
        umin_x=umin.x, umin_y=umin.y,
        vi_x=vi.x,     vi_y=vi.y,
        only_shared_success=only_shared_success,
    )

    boundaries = Quadrilateral(vertices=[
        Point(0.0, 0.0), Point(24.0, 0.0),
        Point(24.0, 20.0), Point(0.0, 20.0),
    ])

    worlds = []
    for _ in range(N):
        point_a, point_b, obstacles = generate_random_world(
            boundaries, n_circles=n_circles, n_quads=n_quads, n_stadiums=n_stadiums,
        )
        worlds.append((point_a, point_b, World(obstacles=obstacles, boundaries=boundaries)))

    times = {
        "N": N,
        "success_c": 0, "success_rrt": 0, "success_astar": 0,
        "planner_c": [], "steer_c": [], "optimizer_c": [], "integration_c": [], "total_c": [],
        "planner_rrt": [], "steer_rrt": [], "optimized_rrt": [], "integration_rrt": [], "total_rrt": [],
        "planner_astar": [], "steer_astar": [], "optimizer_astar": [], "integration_astar": [], "total_astar": [],
        "path_length_c": [], "path_length_rrt": [], "path_length_astar": [],
        "traj_time_pre_c": [], "traj_time_post_c": [],
        "traj_time_pre_rrt": [], "traj_time_post_rrt": [],
        "traj_time_pre_astar": [], "traj_time_post_astar": [],
    }

    planner_map = [("c", PathPlanner), ("rrt", RRT), ("astar", AStar)]
    opt_key_map = {"c": "optimizer_c", "rrt": "optimized_rrt", "astar": "optimizer_astar"}
    results = []

    for i, (point_a, point_b, world) in enumerate(worlds):
        print(f"  Cenário {i + 1}/{N}...", end="\r")
        res = {s: run_pipeline(cls, world, point_a, point_b, vi, vmax, umin, umax)
               for s, cls in planner_map}

        all_ok = all(res[s] is not None for s, _ in planner_map)

        for suffix, _ in planner_map:
            include = bool(res[suffix]) if not only_shared_success else all_ok
            if not include:
                continue
            times[f"success_{suffix}"] += 1
            t_planner, t_steer, t_opt, t_int, t_total = res[suffix]["times"]
            times[f"planner_{suffix}"].append(t_planner)
            times[f"steer_{suffix}"].append(t_steer)
            times[opt_key_map[suffix]].append(t_opt)
            times[f"integration_{suffix}"].append(t_int)
            times[f"total_{suffix}"].append(t_total)
            times[f"path_length_{suffix}"].append(res[suffix]["path_length"])
            times[f"traj_time_pre_{suffix}"].append(res[suffix]["traj_time_pre"])
            times[f"traj_time_post_{suffix}"].append(res[suffix]["traj_time_post"])

        results.append({
            "world": world,
            **{f"path_{s}": res[s]["path"]           if res[s] else None for s, _ in planner_map},
            **{f"opt_{s}":  res[s]["optimized_path"]  if res[s] else None for s, _ in planner_map},
            "counted": all_ok if only_shared_success else any(res[s] for s, _ in planner_map),
        })

    print()
    if only_shared_success:
        times["N"] = times["success_c"]

    return times, results, _config


# ---------------------------------------------------------------------------
# Ponto de entrada
# ---------------------------------------------------------------------------

if __name__ == "__main__":
    SEED = 42

    # ── Exemplo 1: benchmark único ───────────────────────────────────────────
    random.seed(SEED)
    #times, results, cfg = benchmark_pipeline(60, only_shared_success=True)
    #save_benchmark_json(
    #    times, **cfg, random_seed=SEED,
    #    planners=("C", "RRT", "AStar"),
    #    output_path="benchmark_results.json",
    #)
    #plot_benchmark_results(times, planners=("C", "RRT", "AStar"))

    # ── Exemplo 2: sweep variando a densidade de obstáculos ─────────────────
    obstacle_sweep = [
        {"n_circles":  5, "n_quads":  2, "n_stadiums": 0, "label": "~7 obs"},
        {"n_circles": 15, "n_quads":  5, "n_stadiums": 0, "label": "~20 obs"},
        {"n_circles": 21, "n_quads": 7, "n_stadiums": 0, "label": "~28 obs"},
        {"n_circles": 30, "n_quads": 10, "n_stadiums": 0, "label": "~40 obs"},
    ]

    run_sweep(
        obstacle_sweep,
        n_scenarios=1000,
        only_shared_success=False,
        planners=("C", "RRT", "AStar"),
        random_seed=SEED,
        output_dir="sweep_obstacles",
        plot_individual=False,
        plot_comparison=True,
        plots_dir="sweep_obstacles/plots",   # directório raiz para os PNGs
        show_plots=True,                     # False → só guarda, não abre janelas
        n_scenario_plots=10,                 # 10 cenários com trajetórias por configuração
    )