# Archivo: Functions/Observer/Global_Trajectory_Reconstructor.py

import copy
import math


class GlobalTrajectoryReconstructor:

  # ============================================================================
  # VARIABLES DE CONTROL Y LOGGEO
  # ============================================================================
  LOG_TABLA_FRAGMENTOS = (
      True  # 1. true/false: Imprimir o no la tabla de fragmentos recolectados
  )
  LOG_TERMOSTATO = (
      True
      # 2. true/false: Imprimir o no la auto-calibración y el filtro de ruido
  )
  LOG_ARBITRO = (
      2
      # 3. 0: Nada, 1: Resumen ganador, 2: Detalle de puntajes y caminos propuestos
  )
  LOG_TRADUCCION = (
      True
      # 4. true/false: Imprimir o no la traducción física de los patrones finales
  )
  LOG_RECURSION_DEBUG = (
      False  # 5. true/false: Activa impresiones para evitar congelamientos en M2
  )
  ENABLE_DYNAMIC_CLOSURE = (
      True
      # 6. true/false: Activa el cierre inteligente buscando el segmento más cercano al origen
  )
  FORCE_CLOSE_IRREGULAR = (
      False
      # 7. true/false: Activa el cierre forzado añadiendo un segmento final hacia el origen (M1/M2).
  )
  LOG_MAPEO_IDS = (
      True
      # 8. true/false: Imprimir qué IDs locales fueron absorbidos por cada comportamiento ganador.
  )

  # ============================================================================
  # HIPERPARÁMETROS LOCALES DEL SISTEMA
  # ============================================================================

  # ------------------------------------------------------------------------
  # 1. SOLAPES Y TAMAÑO DE ANÁLISIS

  # Tamaño del N-Grama (Cantidad de transiciones consecutivas evaluadas como un bloque).
  # 2 = Pares (Distancia, Ángulo). 3 = Tríos (Distancia, Ángulo, Distancia).
  # Si el sistema de captura descarta fragmentos con menos de 3 datos, subir este valor evita problemas de filtrado.
  NGRAM_SIZE = 3

  # Solape mínimo independiente para el método rápido (Greedy / M1).
  MIN_OVERLAP_M1 = 3

  # Solape mínimo base para el método exhaustivo (Backtracking / M2).
  MIN_OVERLAP_M2 = 3

  # ------------------------------------------------------------------------
  # 2. SEGURIDAD Y RENDIMIENTO (PROTECCIÓN CONTRA CONGELAMIENTOS)

  # Límite máximo de llamadas recursivas permitidas en M2.
  MAX_RECURSIONS_M2 = 5000

  # Interruptor booleano (true/false) para imprimir en consola las rutas exploradas por M2.
  PRINT_RECURSIVE_PATHS = True

  # Límite máximo de fragmentos independientes que M2 está autorizado a procesar.
  MAX_FRAGMENTS_M2 = 9

  # ------------------------------------------------------------------------
  # 3. PONDERACIÓN Y SELECCIÓN DE PATRONES

  # Peso de la frecuencia (Premia patrones repetidos consistentemente).
  W_FREQ = 1

  # Peso de la longitud de la trayectoria.
  # Valores positivos favorecen rutas cortas (restan puntos por tamaño); negativos favorecen rutas largas.
  W_LEN = 0

  # Peso de la calidad de los solapes (Premia uniones robustas y limpias).
  W_OV_QUAL = 2

  # Tolerancia matemática interna utilizada en las comprobaciones geométricas del Geómetra (M3).
  # Mide los grados de error aceptados para que una figura cierre perfectamente en 360°.
  MATH_TOLERANCE = 15

  # ------------------------------------------------------------------------
  # 4. FILTRADO DE RUIDO (TOLERANCIA A LA BASURA DEL SENSOR)

  # Piso mínimo absoluto de filtrado (0.2 equivalentes al 20%).
  RUIDO_BASE = 0.1

  # Margen de ajuste adicional para el ruido (0.15 equivalentes al +15%).
  RUIDO_PENALIZACION_MAX = 0.15

  # ------------------------------------------------------------------------
  # 5. EL GEÓMETRA M3 Y EL ÁRBITRO (LÓGICA DE DECISIÓN)

  # Multiplicador base neutral para las geometrías regulares (polígonos cerrados).
  BONO_M3_BASE = 1.0

  # Premio máximo adicional en la puntuación por simetría perfecta.
  BONO_M3_MAX_EXTRA = 0.5

  # Cobertura mínima absoluta requerida para que el árbitro acepte un patrón (0.25 = 25%).
  SCORE_MIN_BASE = 0.25

  # Blindaje estadístico dinámico (Eleva exigencia en escenarios caóticos).
  SCORE_PENALIZACION_MAX = 0.10

  # Constante de penalización estructural aplicada a trayectorias demasiado largas (Navaja de Ockham).
  PESO_COMPLEJIDAD = 0.05

  ARBITER_CONFIG = {}
  ngram_db = []
  raw_solutions_found = []
  debug_recursions = 0
  m2_aborted = False

  @classmethod
  def log(cls, msg):
    print(f"[PYTHON GESTOR] {msg}", flush=True)

  @classmethod
  def fmt_f(cls, val, decimals=2):
    """Formatea un flotante usando coma decimal igual que string.format en Lua bajo locale es_CO."""
    return f"{val:.{decimals}f}".replace(".", ",")

  @classmethod
  def fmt_struct(cls, s):
    if not s:
      return "nil"
    vals = [cls.fmt_f(v, 2) for v in s.get("value", [])]
    t_str = "Ang" if s.get("type") == 1 else "Dist"
    count = s.get("count", 1)
    count_str = f" [x{count}]" if (count and count > 1) else ""
    return f"({t_str}){count_str} {{{', '.join(vals)}}}"

  @classmethod
  def get_val_type(cls, struct_type, index):
    # En Lua (1-based): index % 2 ~= 0 es impar (1, 3, 5...).
    # En Python (0-based): index % 2 == 0 corresponde a las mismas posiciones (0, 2, 4...).
    is_odd = index % 2 == 0
    if struct_type == 1:
      return "A" if is_odd else "D"
    else:
      return "D" if is_odd else "A"

  @classmethod
  def format_ngram(cls, ngr):
    parts = []
    for k, v in enumerate(ngr["values"]):
      # k es 0-based (k=0 equivale a k=1 impar en Lua)
      is_dist = (
          (k % 2 == 0)
          if (ngr["start_type"] == 0)
          else ((k % 2 != 0) if (ngr["start_type"] == 1) else False)
      )
      lbl = "Dist" if is_dist else "Ang"
      parts.append(f"{lbl}: {cls.fmt_f(v, 2)}")
    return "(" + ", ".join(parts) + ")"

  @classmethod
  def ngrams_match(cls, ng1, ng2, cfg):
    if ng1["start_type"] != ng2["start_type"]:
      return False
    if len(ng1["values"]) != len(ng2["values"]):
      return False
    for k in range(len(ng1["values"])):
      # k es 0-based (k=0 equivale a k=1 impar en Lua)
      is_dist = (
          (k % 2 == 0)
          if (ng1["start_type"] == 0)
          else ((k % 2 != 0) if (ng1["start_type"] == 1) else False)
      )
      tol_cfg = cfg.get("CONFIG", {})
      tol = (
          tol_cfg.get("TOL_DIST", 15) if is_dist else tol_cfg.get("TOL_ANG", 15)
      )
      if abs(ng1["values"][k] - ng2["values"][k]) > tol:
        return False
    return True

  @classmethod
  def extract_ngrams_from_fragment(cls, struct, cfg):
    ngrams = []
    is_dist_first = struct["type"] == 0
    vals = struct["value"]

    if len(vals) < cls.NGRAM_SIZE:
      return [], False

    is_pure = False
    if len(vals) >= 3:
      is_pure = True
      for i in range(len(vals) - 2):
        v1 = vals[i]
        v2 = vals[i + 2]
        v_type = cls.get_val_type(struct["type"], i)
        tol_cfg = cfg.get("CONFIG", {})
        tol = (
            tol_cfg.get("TOL_ANG", 15)
            if v_type == "A"
            else tol_cfg.get("TOL_DIST", 15)
        )
        if abs(v1 - v2) > tol:
          is_pure = False
          break

    for i in range(len(vals) - cls.NGRAM_SIZE + 1):
      ngr_vals = []
      for k in range(cls.NGRAM_SIZE):
        ngr_vals.append(vals[i + k])

      # i es 0-based (i=0 equivale a i=1 impar en Lua)
      is_v1_dist = (i % 2 == 0) if is_dist_first else (i % 2 != 0)
      p_type = 0 if is_v1_dist else 1

      ngrams.append(
          {"values": ngr_vals, "start_type": p_type, "is_pure": is_pure}
      )

    return ngrams, is_pure

  @classmethod
  def build_global_ngram_db(cls, pool, cfg):
    cls.ngram_db = []
    for struct in pool:
      fragment_ngrams, is_pure = cls.extract_ngrams_from_fragment(struct, cfg)
      for ngr in fragment_ngrams:
        found = False
        for db_ngr in cls.ngram_db:
          if cls.ngrams_match(db_ngr, ngr, cfg):
            for k in range(cls.NGRAM_SIZE):
              db_ngr["values"][k] = db_ngr["values"][k] + (
                  ngr["values"][k] - db_ngr["values"][k]
              ) / (db_ngr["merges"] + 1)
            db_ngr["count"] = db_ngr["count"] + (struct.get("count") or 1)
            db_ngr["merges"] = db_ngr["merges"] + 1
            db_ngr["is_pure"] = db_ngr["is_pure"] or is_pure
            found = True
            break
        if not found:
          vals_copy = [v for v in ngr["values"]]
          cls.ngram_db.append({
              "values": vals_copy,
              "start_type": ngr["start_type"],
              "count": struct.get("count") or 1,
              "merges": 1,
              "is_pure": is_pure,
          })

  @classmethod
  def calibrar_hiperparametros(cls, freq_array, current_ngram_db, cfg):
    if len(freq_array) == 0:
      return

    max_frecuencia = 0
    suma_frecuencias = 0
    pureza_total = 0
    ngrams_dominantes = []

    for ngr in current_ngram_db:
      if ngr["count"] > max_frecuencia:
        max_frecuencia = ngr["count"]
        ngrams_dominantes = [ngr]
      elif ngr["count"] == max_frecuencia:
        ngrams_dominantes.append(ngr)

    for f in freq_array:
      suma_frecuencias = suma_frecuencias + f

    for ngr in current_ngram_db:
      if ngr["is_pure"]:
        pureza_total = pureza_total + ngr["count"]

    prom_frecuencia = suma_frecuencias / len(freq_array)

    indice_orden = 1.0
    if suma_frecuencias > 0:
      indice_orden = 1.0 - (len(freq_array) / suma_frecuencias)
      if indice_orden < 0.0:
        indice_orden = 0.0

    cls.ARBITER_CONFIG["INDICE_ORDEN"] = indice_orden

    ratio_pureza = 0
    if suma_frecuencias > 0:
      ratio_pureza = pureza_total / suma_frecuencias

    nivel_caos = 1.0 - indice_orden
    ruido_extra = nivel_caos * cls.RUIDO_PENALIZACION_MAX
    cls.ARBITER_CONFIG["RUIDO_UMBRAL_RELATIVO"] = cls.RUIDO_BASE + ruido_extra

    bono_extra = ratio_pureza * cls.BONO_M3_MAX_EXTRA * indice_orden
    cls.ARBITER_CONFIG["BONO_REGULARIDAD"] = cls.BONO_M3_BASE + bono_extra

    penalizacion_caos = nivel_caos * cls.SCORE_PENALIZACION_MAX
    cls.ARBITER_CONFIG["MIN_SCORE"] = cls.SCORE_MIN_BASE + penalizacion_caos

    repeticiones_minimas = (
        max_frecuencia * cls.ARBITER_CONFIG["RUIDO_UMBRAL_RELATIVO"]
    )

    if cls.LOG_TERMOSTATO:
      cls.log("\n=========================================")
      cls.log(" [TERMOSTATO] AUTO-CALIBRACIÓN DEL ENJAMBRE")
      cls.log("=========================================")
      cls.log("  [1] Extracción de Métricas Base:")
      cls.log(f"Frecuencia Máxima Dominante: {max_frecuencia}")
      for ngr in ngrams_dominantes:
        cls.log(f"        -> Corresponde al N-Grama: {cls.format_ngram(ngr)}")
      cls.log(f"Frecuencia Promedio Global: {cls.fmt_f(prom_frecuencia, 2)}")
      cls.log(f"Volumen Total (Cantidad de N-Gramas): {suma_frecuencias}")
      cls.log(f"N-Gramas Únicos: {len(freq_array)}")
      cls.log(f"N-Gramas con Simetría (Periodicos): {pureza_total}")

      cls.log("\n  [2] Diagnóstico del Entorno (Fórmulas):")
      cls.log(
          "Índice de Orden: Mide qué tan repetitivos son los datos (1.0 ="
          " patrón dominante claro, 0 = caos)."
      )
      cls.log(
          f"        -> 1.0 - (Únicos [{len(freq_array)}] / Volumen"
          f" [{suma_frecuencias}]) = {cls.fmt_f(indice_orden, 2)}"
      )
      cls.log(
          "Ratio de Pureza: Mide la proporción de geometrías perfectas (como"
          " polígonos cerrados)."
      )
      cls.log(
          f"        -> Periodicos [{pureza_total}] / Volumen"
          f" [{suma_frecuencias}] = {cls.fmt_f(ratio_pureza, 2)}"
      )

      cls.log("\n  [3] Fórmulas y Resultados Aplicados:")
      cls.log(
          "Umbral de Ruido: Porcentaje mínimo de repetición para que un dato NO"
          " sea descartado."
      )
      cls.log(
          f"        > FÓRMULA UMBRAL : Base({cls.fmt_f(cls.RUIDO_BASE, 2)}) +"
          f" [Caos({cls.fmt_f(nivel_caos, 2)}) *"
          f" Max_Pen({cls.fmt_f(cls.RUIDO_PENALIZACION_MAX, 2)})] ="
          f" {cls.fmt_f(cls.ARBITER_CONFIG['RUIDO_UMBRAL_RELATIVO'] * 100, 1)}%."
      )
      cls.log(
          "        > LÍMITE ABSOLUTO:"
          f" {cls.fmt_f(cls.ARBITER_CONFIG['RUIDO_UMBRAL_RELATIVO'] * 100, 1)}%"
          f" de {max_frecuencia} repeticiones (frecuencia dominante)."
      )
      cls.log(
          "        > RESULTADO      : Cualquier fragmento con MENOS de"
          f" {cls.fmt_f(repeticiones_minimas, 2)} repeticiones es RUIDO."
      )
      cls.log("=========================================")

  @classmethod
  def calcular_limite_ruido(cls, frecuencias):
    if len(frecuencias) == 0:
      return 0
    max_freq = 0
    for f in frecuencias:
      if f > max_freq:
        max_freq = f
    return max_freq * cls.ARBITER_CONFIG.get("RUIDO_UMBRAL_RELATIVO", 0)

  @classmethod
  def check_subset_match(
      cls, long_val, short_val, start_idx, long_type, short_type, cfg
  ):
    if cls.get_val_type(long_type, start_idx) != cls.get_val_type(
        short_type, 0
    ):
      return False
    for k in range(len(short_val)):
      l_idx = start_idx + k
      v_type = cls.get_val_type(short_type, k)
      diff = abs(long_val[l_idx] - short_val[k])
      tol_cfg = cfg.get("CONFIG", {})
      tol = (
          tol_cfg.get("TOL_ANG", 15)
          if v_type == "A"
          else tol_cfg.get("TOL_DIST", 15)
      )
      if diff > tol:
        return False
    return True

  @classmethod
  def reduce_fragments_pool(cls, pool, cfg):
    reduced_pool = []
    group_logs = []
    skip = {}

    pool.sort(key=lambda a: len(a["value"]), reverse=True)

    for i in range(len(pool)):
      if not skip.get(i):
        current_target = pool[i]
        absorbed_ids = [current_target["id"]]
        merged_any = False

        for j in range(i + 1, len(pool)):
          if not skip.get(j):
            candidate = pool[j]

            if len(current_target["value"]) >= len(candidate["value"]):
              for start_idx in range(
                  len(current_target["value"]) - len(candidate["value"]) + 1
              ):
                if cls.check_subset_match(
                    current_target["value"],
                    candidate["value"],
                    start_idx,
                    current_target["type"],
                    candidate["type"],
                    cfg,
                ):
                  c1 = current_target.get("count") or 1
                  c2 = candidate.get("count") or 1
                  divisor = c1 + c2

                  for k in range(len(candidate["value"])):
                    t_idx = start_idx + k
                    sum_val = (current_target["value"][t_idx] * c1) + (
                        candidate["value"][k] * c2
                    )
                    current_target["value"][t_idx] = sum_val / divisor

                  current_target["count"] = divisor
                  if "consumed_ids" not in current_target:
                    current_target["consumed_ids"] = {}
                  for cid in candidate.get("consumed_ids") or {}:
                    current_target["consumed_ids"][cid] = True

                  absorbed_ids.append(candidate["id"])
                  merged_any = True
                  skip[j] = True
                  break

        if merged_any:
          group_logs.append(
              {"ids": absorbed_ids, "result_str": cls.fmt_struct(current_target)}
          )

        reduced_pool.append(current_target)

    return reduced_pool, group_logs

  @classmethod
  def calculate_overlap(cls, s1, s2, min_ov, cfg):
    len1, len2 = len(s1["value"]), len(s2["value"])
    max_ov = min(len1, len2)
    for ov in range(max_ov, min_ov - 1, -1):
      match = True
      for k in range(ov):
        idx1, idx2 = len1 - ov + k, k
        t1 = cls.get_val_type(s1["type"], idx1)
        t2 = cls.get_val_type(s2["type"], idx2)
        if t1 != t2:
          match = False
          break
        diff = abs(s1["value"][idx1] - s2["value"][idx2])
        tol_cfg = cfg.get("CONFIG", {})
        tol = (
            tol_cfg.get("TOL_ANG", 15)
            if t1 == "A"
            else tol_cfg.get("TOL_DIST", 15)
        )
        if diff > tol:
          match = False
          break
      if match:
        return ov
    return 0

  @classmethod
  def merge_fragments(cls, s1, s2, overlap):
    new_vals = []
    for k in range(len(s1["value"]) - overlap):
      new_vals.append(s1["value"][k])

    for k in range(overlap):
      v1 = s1["value"][len(s1["value"]) - overlap + k]
      v2 = s2["value"][k]
      new_vals.append((v1 + v2) / 2.0)

    for k in range(overlap, len(s2["value"])):
      new_vals.append(s2["value"][k])

    combined_id = s1["id"] + "+" + s2["id"]
    new_count = (s1.get("count") or 1) + (s2.get("count") or 1)
    new_total_ov = (s1.get("total_ov") or 0) + (s2.get("total_ov") or 0) + overlap

    new_consumed = {}
    for k in s1.get("consumed_ids") or {}:
      new_consumed[k] = True
    for k in s2.get("consumed_ids") or {}:
      new_consumed[k] = True

    return {
        "value": new_vals,
        "type": s1["type"],
        "id": combined_id,
        "count": new_count,
        "total_ov": new_total_ov,
        "consumed_ids": new_consumed,
    }

  @classmethod
  def clone_pool(cls, pool):
    new_pool = []
    for v in pool:
      nv = [val for val in v["value"]]
      c_ids = {id_k: id_v for id_k, id_v in (v.get("consumed_ids") or {}).items()}
      new_pool.append({
          "value": nv,
          "type": v["type"],
          "id": v["id"],
          "count": v.get("count"),
          "total_ov": v.get("total_ov") or 0,
          "consumed_ids": c_ids,
      })
    return new_pool

  @classmethod
  def solve_greedy(cls, initial_pool, cfg):
    pool = cls.clone_pool(initial_pool)
    while len(pool) > 1:
      best = {"i": -1, "j": -1, "ov": -1, "struct": None}
      for i in range(len(pool)):
        for j in range(len(pool)):
          if i != j:
            ov = cls.calculate_overlap(pool[i], pool[j], cls.MIN_OVERLAP_M1, cfg)
            if ov > best["ov"]:
              best = {
                  "i": i,
                  "j": j,
                  "ov": ov,
                  "struct": cls.merge_fragments(pool[i], pool[j], ov),
              }
      if best["ov"] > 0:
        pool[best["i"]] = best["struct"]
        del pool[best["j"]]
      else:
        break
    return pool

  @classmethod
  def recursive_finder(cls, current_pool, history_log, cfg, dynamic_min_overlap):
    if cls.m2_aborted:
      return

    cls.debug_recursions += 1

    if cls.debug_recursions > cls.MAX_RECURSIONS_M2:
      if cls.LOG_RECURSION_DEBUG:
        print(
            f"[PYTHON GESTOR] [ALERTA CRÍTICA M2] Límite de {cls.MAX_RECURSIONS_M2}"
            " iteraciones superado. Abortando árbol de búsqueda para evitar"
            " congelamiento...",
            flush=True,
        )
      cls.m2_aborted = True
      return

    if cls.LOG_RECURSION_DEBUG and (cls.debug_recursions % 500 == 0):
      print(
          "[PYTHON GESTOR] [ALERTA RECURSIVIDAD] recursive_finder iteración:"
          f" {cls.debug_recursions} | Tamaño del pool: {len(current_pool)}",
          flush=True,
      )

    made_merge = False
    candidates = []
    for i in range(len(current_pool)):
      for j in range(len(current_pool)):
        if i != j:
          ov = cls.calculate_overlap(
              current_pool[i], current_pool[j], dynamic_min_overlap, cfg
          )
          if ov > 0:
            candidates.append({"i": i, "j": j, "ov": ov})

    if len(candidates) > 0:
      made_merge = True
      candidates.sort(key=lambda a: a["ov"], reverse=True)
      for cand in candidates:
        if cls.m2_aborted:
          break

        next_pool = []
        merged = cls.merge_fragments(
            current_pool[cand["i"]], current_pool[cand["j"]], cand["ov"]
        )
        next_pool.append(merged)
        for k in range(len(current_pool)):
          if k != cand["i"] and k != cand["j"]:
            next_pool.append(current_pool[k])
        next_history = [h for h in history_log]
        next_history.append(
            f"Merge {current_pool[cand['i']]['id']}+{current_pool[cand['j']]['id']}"
            f" (Ov:{cand['ov']})"
        )

        cls.recursive_finder(
            next_pool, next_history, cfg, dynamic_min_overlap
        )

    if not made_merge and not cls.m2_aborted:
      cls.raw_solutions_found.append(
          {"pool_result": current_pool, "history": history_log}
      )

      if cls.PRINT_RECURSIVE_PATHS:
        path_str = " -> ".join(history_log)
        if path_str == "":
          path_str = "Directo sin merges"
        cls.log("[M2 RUTA ENCONTRADA] Pasos: " + path_str)

  @classmethod
  def select_best_solution_m2(cls, solutions, cfg):
    if len(solutions) == 0:
      return None
    unique_groups = {}
    for sol in solutions:
      hashes = []
      total_len = 0
      total_ov = 0
      for s in sol["pool_result"]:
        hashes.append(cls.fmt_struct(s))
        total_len = total_len + len(s["value"])
        total_ov = total_ov + (s.get("total_ov") or 0)
      hashes.sort()
      key = " || ".join(hashes)
      if key not in unique_groups:
        unique_groups[key] = {
            "pool_result": sol["pool_result"],
            "count": 1,
            "history": sol["history"],
            "total_len": total_len,
            "total_ov": total_ov,
        }
      else:
        unique_groups[key]["count"] = unique_groups[key]["count"] + 1

    unique_list = []
    for key, data in unique_groups.items():
      score = (
          (data["count"] * cls.W_FREQ)
          + (data["total_ov"] * cls.W_OV_QUAL)
          - (data["total_len"] * cls.W_LEN)
      )
      data["score"] = score
      unique_list.append(data)

    unique_list.sort(key=lambda x: (x["score"], x["count"]), reverse=True)
    return unique_list[0]

  @classmethod
  def is_fragment_closed(cls, struct, cfg):
    values = struct["value"]
    current_type = struct["type"]

    temp_dist = 0
    temp_ang = 0
    expecting_dist = current_type == 0
    params = []

    for val in values:
      if expecting_dist:
        temp_dist = val
        params.append([temp_dist, temp_ang])
        temp_ang = 0
        expecting_dist = False
      else:
        temp_ang = val
        expecting_dist = True

    if len(params) < 3:
      return False

    x, y = 0, 0
    theta_deg = 0
    for i in range(len(params)):
      d = params[i][0]
      a = params[i][1]
      theta_deg = theta_deg + a
      theta_rad = math.radians(theta_deg)
      x = x + d * math.cos(theta_rad)
      y = y + d * math.sin(theta_rad)

    dist_to_origin = math.sqrt(x**2 + y**2)
    tol = (
        cfg["CONFIG"]["TOL_DIST"]
        if (cfg and "CONFIG" in cfg and "TOL_DIST" in cfg["CONFIG"])
        else 15.0
    )
    return dist_to_origin <= tol

  @classmethod
  def force_close_fragment(cls, struct, cfg):
    values = struct["value"]
    current_type = struct["type"]

    temp_dist = 0
    temp_ang = 0
    expecting_dist = current_type == 0
    params = []

    for val in values:
      if expecting_dist:
        temp_dist = val
        params.append([temp_dist, temp_ang])
        temp_ang = 0
        expecting_dist = False
      else:
        temp_ang = val
        expecting_dist = True

    x, y = 0, 0
    theta_deg = 0
    for i in range(len(params)):
      d = params[i][0]
      a = params[i][1]
      theta_deg = theta_deg + a
      theta_rad = math.radians(theta_deg)
      x = x + d * math.cos(theta_rad)
      y = y + d * math.sin(theta_rad)

    dist_to_origin = math.sqrt(x**2 + y**2)
    tol = (
        cfg["CONFIG"]["TOL_DIST"]
        if (cfg and "CONFIG" in cfg and "TOL_DIST" in cfg["CONFIG"])
        else 15.0
    )
    if dist_to_origin <= tol:
      return

    target_angle_rad = math.atan2(-y, -x)
    target_angle_deg = math.degrees(target_angle_rad)

    turn_angle = target_angle_deg - theta_deg
    turn_angle = (turn_angle + 180) % 360 - 180

    if expecting_dist:
      values[-1] = turn_angle
      values.append(dist_to_origin)
    else:
      values.append(turn_angle)
      values.append(dist_to_origin)

  @classmethod
  def solve_smart_backtracking_all(cls, initial_pool, cfg, dynamic_min_overlap):
    if cls.LOG_RECURSION_DEBUG:
      print(
          "[PYTHON GESTOR] [DEBUG M2] -> Iniciando Backtracking M2. Pool:"
          f" {len(initial_pool)} | Solape Requerido: {dynamic_min_overlap}",
          flush=True,
      )

    cls.raw_solutions_found = []
    sorted_pool = cls.clone_pool(initial_pool)
    sorted_pool.sort(key=lambda a: len(a["value"]), reverse=True)

    cls.debug_recursions = 0
    cls.m2_aborted = False

    cls.recursive_finder(sorted_pool, [], cfg, dynamic_min_overlap)

    if cls.m2_aborted:
      return None

    if cls.LOG_RECURSION_DEBUG:
      print(
          "[PYTHON GESTOR] [DEBUG M2] -> Finalizado con éxito. Llamadas"
          f" recursivas totales: {cls.debug_recursions}",
          flush=True,
      )

    return cls.select_best_solution_m2(cls.raw_solutions_found, cfg)

  @classmethod
  def evaluar_hipotesis_m3(
      cls, clean_pool, current_ngram_db, total_data_volume, iteration, cfg
  ):
    m3_score = -1
    m3_hypothesis = None
    m3_covered_ids = {}

    best_pure_ngram = None
    for ngr in current_ngram_db:
      if ngr["is_pure"]:
        if not best_pure_ngram or ngr["count"] > best_pure_ngram["count"]:
          best_pure_ngram = ngr

    if best_pure_ngram:
      coverage_count = 0
      source_fragments_list = []

      best_d, best_a = 0, 0
      if best_pure_ngram["start_type"] == 0:
        best_d = best_pure_ngram["values"][0]
        best_a = best_pure_ngram["values"][1]
      else:
        best_a = best_pure_ngram["values"][0]
        best_d = best_pure_ngram["values"][1]

      if best_a is None or best_d is None:
        return -1, None, {}

      for struct in clean_pool:
        matches_perfectly = True
        expecting_dist = struct["type"] == 0
        vals = struct["value"]

        for k in range(len(vals)):
          tol_cfg = cfg.get("CONFIG", {})
          tol = (
              tol_cfg.get("TOL_DIST", 15)
              if expecting_dist
              else tol_cfg.get("TOL_ANG", 15)
          )
          ref_val = best_d if expecting_dist else best_a

          if abs(vals[k] - ref_val) > tol:
            matches_perfectly = False
            break
          expecting_dist = not expecting_dist

        if matches_perfectly:
          coverage_count = coverage_count + (struct.get("count") or 1)
          for cid in struct.get("consumed_ids") or {}:
            m3_covered_ids[cid] = True
          source_fragments_list.append(struct["id"])

      coverage_norm_m3 = coverage_count / total_data_volume
      lados = 4
      if best_a != 0:
        lados = math.floor((360 / abs(best_a)) + 0.5)

      error_cierre = abs((lados * abs(best_a)) - 360)
      if error_cierre > cls.MATH_TOLERANCE:
        return -1, None, {}

      parametros_m3 = lados * 2
      complexity_norm_m3 = parametros_m3 / total_data_volume
      m3_score = (
          coverage_norm_m3 * cls.ARBITER_CONFIG.get("BONO_REGULARIDAD", 1.0)
      ) - (complexity_norm_m3 * cls.PESO_COMPLEJIDAD)

      synthetic_vals = []
      for _ in range(lados):
        synthetic_vals.append(best_d)
        synthetic_vals.append(best_a)
      m3_hypothesis = {
          "type": 0,
          "value": synthetic_vals,
          "id": f"Poligono_Iter{iteration}",
          "count": coverage_count,
      }

    return m3_score, m3_hypothesis, m3_covered_ids

  @classmethod
  def process_numeric_result(cls, result_struct, patron_id, cfg):
    current_pattern_params = []
    values = result_struct["value"]
    current_type = result_struct["type"]

    temp_dist = 0
    temp_ang = 0
    expecting_dist = current_type == 0

    for val in values:
      if expecting_dist:
        temp_dist = val
        current_pattern_params.append([temp_dist, temp_ang])
        temp_ang = 0
        expecting_dist = False
      else:
        temp_ang = val
        expecting_dist = True

    dropped_segment_flag = False
    removed_count = 0

    if cls.ENABLE_DYNAMIC_CLOSURE and len(current_pattern_params) >= 3:
      x, y = 0, 0
      theta_deg = 0
      min_dist = float("inf")
      best_closing_index = -1

      tol_cierre = cfg.get("CONFIG", {}).get("TOL_DIST") or 5.0

      for i in range(len(current_pattern_params)):
        d = current_pattern_params[i][0]
        a = current_pattern_params[i][1]
        theta_deg = theta_deg + a
        theta_rad = math.radians(theta_deg)
        x = x + d * math.cos(theta_rad)
        y = y + d * math.sin(theta_rad)

        dist = math.sqrt(x**2 + y**2)

        # i >= 2 en 0-based equivale a i >= 3 en 1-based de Lua
        if i >= 2:
          if dist < min_dist:
            min_dist = dist
            best_closing_index = i

      if (
          best_closing_index != -1
          and min_dist <= tol_cierre
          and best_closing_index < len(current_pattern_params) - 1
      ):
        removed_count = len(current_pattern_params) - 1 - best_closing_index
        for _ in range(removed_count):
          current_pattern_params.pop()
        dropped_segment_flag = True

    if cls.LOG_TRADUCCION:
      cls.log(f"\n[TRADUCCIÓN FÍSICA - COMPORTAMIENTO {patron_id}]")

      if dropped_segment_flag:
        kept_vals = []
        elements_to_keep = len(current_pattern_params) * 2
        elements_to_keep = min(elements_to_keep, len(values))

        for k in range(elements_to_keep):
          kept_vals.append(cls.fmt_f(values[k], 2))

        t_str = "Ang" if current_type == 1 else "Dist"
        count = result_struct.get("count")
        count_str = f" [x{count}]" if (count and count > 1) else ""
        new_chain_str = f"({t_str}){count_str} {{{', '.join(kept_vals)}}}"

        cls.log(
            "   >> LÓGICA DE CIERRE INTELIGENTE: Se eliminaron los últimos"
            f" {removed_count} segmento(s) porque el segmento"
            f" {len(current_pattern_params)} dejaba al robot más cerca del"
            " origen."
        )
        cls.log("   >> CADENA ACTUALIZADA: " + new_chain_str)

      for idx, seg in enumerate(current_pattern_params):
        cls.log(
            f"   Seg {idx + 1}: (Ang {cls.fmt_f(seg[1], 2)}, Dist"
            f" {cls.fmt_f(seg[0], 2)})"
        )

    return current_pattern_params

  @classmethod
  def run(cls, robot_id, stored_data, cfg, offset_comportamientos):
    if cls.LOG_TABLA_FRAGMENTOS:
      cls.log("\n=========================================")
      cls.log(" TABLA DE FRAGMENTOS RECOLECTADOS")
      cls.log("=========================================")
      for i, v in enumerate(stored_data):
        cls.log(f" [{i + 1}] {cls.fmt_struct(v)}")

    raw_pool = []
    for i, v in enumerate(stored_data):
      preserved_consumed_ids = {}
      if v.get("consumed_ids"):
        for cid in v["consumed_ids"]:
          preserved_consumed_ids[cid] = True
      else:
        preserved_consumed_ids[str(i + 1)] = True

      raw_pool.append({
          "value": list(v["value"]),
          "type": v["type"],
          "count": v.get("count", 1),
          "id": v.get("id") or str(i + 1),
          "consumed_ids": preserved_consumed_ids,
      })

    if len(raw_pool) == 0:
      return False, "", [], {}

    reduced_pool, group_logs = cls.reduce_fragments_pool(raw_pool, cfg)

    if cls.LOG_TERMOSTATO:
      cls.log("\n=========================================")
      cls.log(" [REDUCCIÓN DE FRAGMENTOS (PRE-PROCESAMIENTO)]")
      if len(group_logs) > 0:
        for num, g_log in enumerate(group_logs):
          ids_str = ", ".join(g_log["ids"])
          cls.log(
              f" promedio {num + 1} = fragmentos: {ids_str} ="
              f" {g_log['result_str']}"
          )
      else:
        cls.log(
            " >> No se encontraron sub-cadenas para reducir. Todos los"
            " fragmentos son únicos."
        )

    cls.build_global_ngram_db(reduced_pool, cfg)

    freq_array = []
    for ngr in cls.ngram_db:
      freq_array.append(ngr["count"])

    cls.calibrar_hiperparametros(freq_array, cls.ngram_db, cfg)

    limite_ruido = cls.calcular_limite_ruido(freq_array)

    clean_pool = []
    discarded_fragments = []

    for struct in reduced_pool:
      fragment_ngrams, _ = cls.extract_ngrams_from_fragment(struct, cfg)
      is_noisy = False
      failed_ngram_count = 0
      failed_ngram_str = ""
      ngrams_info = []

      for ngr in fragment_ngrams:
        current_count = 0
        for db_ngr in cls.ngram_db:
          if cls.ngrams_match(db_ngr, ngr, cfg):
            current_count = db_ngr["count"]
            break

        ngrams_info.append(
            {"str": cls.format_ngram(ngr), "count": current_count}
        )

        if current_count < limite_ruido and not is_noisy:
          is_noisy = True
          failed_ngram_count = current_count
          failed_ngram_str = cls.format_ngram(ngr)

      if not is_noisy:
        clean_pool.append(struct)
      else:
        discarded_fragments.append({
            "id": struct["id"],
            "str": cls.fmt_struct(struct),
            "failed_count": failed_ngram_count,
            "failed_str": failed_ngram_str,
            "min": limite_ruido,
            "ngrams": ngrams_info,
        })

    if cls.LOG_TERMOSTATO:
      cls.log("\n=========================================")
      cls.log(" [FILTRO RUIDO]")

      if len(discarded_fragments) > 0:
        for info in discarded_fragments:
          cls.log("")
          cls.log(f" Fragmento {info['id']} -> {info['str']}")
          cls.log("   Frecuencia de sus secuencias (N-Gramas):")
          for ngr_info in info["ngrams"]:
            cls.log(
                f"   {ngr_info['str']} -> Frecuencia: {ngr_info['count']}"
            )
          cls.log(
              "   >> DESCARTADO: Debido a que la secuencia"
              f" {info['failed_str']} tenía {info['failed_count']} repeticiones"
              f" (frecuencia mínima debió ser {cls.fmt_f(info['min'], 2)})."
          )
      else:
        cls.log(
            "\n >> Ningún fragmento fue eliminado. Todos superaron el umbral"
            f" de ruido mínimo ({cls.fmt_f(limite_ruido, 2)})."
        )
      cls.log("=========================================")

      cls.log("\n=========================================")
      cls.log(" [POOL FINAL POST-REDUCCIÓN Y FILTRO]")
      if len(clean_pool) > 0:
        for i, v in enumerate(clean_pool):
          cls.log(f" [{i + 1}] {cls.fmt_struct(v)}")
      else:
        cls.log(" >> El pool quedó vacío.")
      cls.log("=========================================\n")

    all_final_results = []
    iteration = 1

    while len(clean_pool) > 0:
      total_data_volume = 0
      for v in clean_pool:
        total_data_volume = total_data_volume + (v.get("count") or 1)
      if total_data_volume == 0:
        break

      winner_struct = None
      ids_to_remove = {}
      is_m3_winner = False
      is_single_closed = False

      if len(clean_pool) == 1:
        is_single_closed = cls.is_fragment_closed(clean_pool[0], cfg)
        if is_single_closed:
          winner_struct = clean_pool[0]
          ids_to_remove = clean_pool[0].get("consumed_ids") or {}
          if cls.LOG_ARBITRO >= 1:
            cls.log(
                "\n [ÁRBITRO] >>> ¡Gana Único Fragmento (Cierre Natural)! ->"
                f" {cls.fmt_struct(winner_struct)}"
            )

      if not is_single_closed:
        cls.build_global_ngram_db(clean_pool, cfg)
        m3_score, m3_hypothesis, m3_covered_ids = cls.evaluar_hipotesis_m3(
            clean_pool, cls.ngram_db, total_data_volume, iteration, cfg
        )

        explorer_score = -1
        explorer_hypothesis = None
        explorer_name = "M1"
        explorer_pool_result = []
        m1_fallback_reason = ""

        idx_orden = cls.ARBITER_CONFIG.get("INDICE_ORDEN") or 1.0
        dynamic_min_overlap = cls.MIN_OVERLAP_M2
        dynamic_max_fragments = cls.MAX_FRAGMENTS_M2

        if idx_orden <= 0.4:
          dynamic_min_overlap = max(dynamic_min_overlap, 3)
          dynamic_max_fragments = min(dynamic_max_fragments, 5)
        elif idx_orden <= 0.8:
          dynamic_min_overlap = max(dynamic_min_overlap, 2)
          dynamic_max_fragments = min(dynamic_max_fragments, 7)

        if cls.LOG_RECURSION_DEBUG:
          print(
              "\n[DEBUG ADAPTATIVO] Indice de Orden:"
              f" {cls.fmt_f(idx_orden, 2)} | M2 ajustado a Solape >="
              f" {dynamic_min_overlap}, Límite Fragmentos <="
              f" {dynamic_max_fragments}",
              flush=True,
          )

        if len(clean_pool) <= dynamic_max_fragments:
          m2_solution = cls.solve_smart_backtracking_all(
              clean_pool, cfg, dynamic_min_overlap
          )

          if m2_solution and len(m2_solution["pool_result"]) > 0:
            explorer_pool_result = m2_solution["pool_result"]
            explorer_name = "M2"
          else:
            m1_fallback_reason = (
                " [Forzado a M1: M2 abortó al superar"
                f" {cls.MAX_RECURSIONS_M2} llamadas recursivas]"
            )
            explorer_pool_result = cls.solve_greedy(clean_pool, cfg)
        else:
          m1_fallback_reason = (
              f" [Forzado a M1: {len(clean_pool)} fragmentos superan el límite"
              f" dinámico de {dynamic_max_fragments} fragmentos]"
          )
          explorer_pool_result = cls.solve_greedy(clean_pool, cfg)

        if len(explorer_pool_result) > 0:
          explorer_pool_result.sort(
              key=lambda a: (a.get("count") or 1), reverse=True
          )
          explorer_hypothesis = explorer_pool_result[0]
          coverage_norm_exp = (
              explorer_hypothesis.get("count") or 1
          ) / total_data_volume
          complexity_norm_exp = (
              len(explorer_hypothesis["value"]) / total_data_volume
          )
          explorer_score = (coverage_norm_exp * 1.0) - (
              complexity_norm_exp * cls.PESO_COMPLEJIDAD
          )

        min_score = cls.ARBITER_CONFIG.get("MIN_SCORE", cls.SCORE_MIN_BASE)

        if m3_score > explorer_score and m3_score >= min_score:
          m3_hypothesis["consumed_ids"] = m3_covered_ids
          winner_struct = m3_hypothesis
          ids_to_remove = m3_covered_ids
          is_m3_winner = True

          if cls.LOG_ARBITRO == 2:
            cls.log(
                f"\n [ÁRBITRO DETALLE] Evaluación de la Iteración {iteration}"
            )
            cls.log(f"Puntaje M3 (Regular): {cls.fmt_f(m3_score, 4)}")
            cls.log(
                f"Puntaje {explorer_name} (Irregular):"
                f" {cls.fmt_f(explorer_score, 4)}"
            )
            cls.log(f"Mínimo Requerido: {cls.fmt_f(min_score, 4)}")
            cls.log(f"Camino propuesto M3: {cls.fmt_struct(m3_hypothesis)}")
            if explorer_hypothesis:
              cls.log(
                  f"Camino propuesto {explorer_name}:"
                  f" {cls.fmt_struct(explorer_hypothesis)}"
              )

          if cls.LOG_ARBITRO >= 1:
            cls.log(
                "\n [ÁRBITRO] >>> ¡Gana M3 (Geometría Regular)! ->"
                f" {cls.fmt_struct(winner_struct)}"
            )

        elif explorer_score >= m3_score and explorer_score >= min_score:
          winner_struct = explorer_hypothesis
          ids_to_remove = explorer_hypothesis.get("consumed_ids") or {}

          if cls.LOG_ARBITRO == 2:
            cls.log(
                f"\n [ÁRBITRO DETALLE] Evaluación de la Iteración {iteration}"
            )
            cls.log(f"Puntaje M3 (Regular): {cls.fmt_f(m3_score, 4)}")
            cls.log(
                f"Puntaje {explorer_name} (Irregular):"
                f" {cls.fmt_f(explorer_score, 4)}"
            )
            cls.log(f"Mínimo Requerido: {cls.fmt_f(min_score, 4)}")
            if m3_hypothesis:
              cls.log(f"Camino propuesto M3: {cls.fmt_struct(m3_hypothesis)}")
            cls.log(
                f"Camino propuesto {explorer_name}:"
                f" {cls.fmt_struct(explorer_hypothesis)}"
            )

          if cls.LOG_ARBITRO >= 1:
            reason_str = m1_fallback_reason if explorer_name == "M1" else ""
            cls.log(
                f"\n [ÁRBITRO] >>> ¡Gana {explorer_name} (Ruta"
                f" Irregular){reason_str}! -> {cls.fmt_struct(winner_struct)}"
            )
        else:
          if cls.LOG_ARBITRO == 2:
            cls.log(
                "\n [ÁRBITRO DETALLE] Fin - Ninguno superó el umbral mínimo"
                f" ({cls.fmt_f(min_score, 4)})"
            )
            cls.log(
                f"Puntaje M3: {cls.fmt_f(m3_score, 4)} | Puntaje"
                f" {explorer_name}: {cls.fmt_f(explorer_score, 4)}"
            )
          break

      if (
          winner_struct
          and cls.FORCE_CLOSE_IRREGULAR
          and not is_m3_winner
          and not is_single_closed
      ):
        cls.force_close_fragment(winner_struct, cfg)
        if cls.LOG_ARBITRO >= 1:
          cls.log(
              " [CIERRE FORZADO] Ruta irregular actualizada (Regresa al"
              f" origen) -> {cls.fmt_struct(winner_struct)}"
          )

      all_final_results.append(winner_struct)
      next_pool = []
      for struct in clean_pool:
        if not ids_to_remove.get(struct["id"]):
          next_pool.append(struct)
      clean_pool = next_pool
      iteration = iteration + 1

    linea_resumen = robot_id + " & Imitado&"
    nuevos_patrones = []
    winner_ids_map = {}

    offset_c = offset_comportamientos or 0

    if cls.LOG_MAPEO_IDS:
      cls.log("\n [MAPEO DE IDs AL FINALIZAR ÁRBITRO]")

    for i, res in enumerate(all_final_results):
      if not res.get("consumed_ids"):
        res["consumed_ids"] = {res["id"]: True}

      res_ids_str = ""
      for cid in res["consumed_ids"]:
        winner_ids_map[cid] = True
        res_ids_str = res_ids_str + str(cid) + ", "

      # ¡CORRECCIÓN APLICADA AQUÍ! Ahora el mapeo muestra el mismo índice que la traducción
      if cls.LOG_MAPEO_IDS:
        cls.log(
            f"   >> Comportamiento {offset_c + i + 1} absorbió los fragmentos"
            f" locales con ID: {{{res_ids_str}}}"
        )

      pattern_params = cls.process_numeric_result(res, offset_c + i + 1, cfg)
      linea_resumen = linea_resumen + f"p{offset_c + i + 1}"
      for idx, seg in enumerate(pattern_params):
        if idx > 0:
          linea_resumen = linea_resumen + ","
        linea_resumen = (
            linea_resumen + f"({cls.fmt_f(seg[1], 2)},{cls.fmt_f(seg[0], 2)})"
        )
      nuevos_patrones.append(pattern_params)

    return True, linea_resumen, nuevos_patrones, winner_ids_map