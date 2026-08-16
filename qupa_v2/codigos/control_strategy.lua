-- scripts/control_strategy.lua
-- Estrategia Selectiva con Memoria, Aprendizaje y Olvido 
-- Este archivo implementa una FSM (máquina de estados) para controlar un robot Qupa
-- que explora, selecciona blobs (objetivos por color), decide estocásticamente si
-- acepta la tarea y entra a un parche negro para ejecutar una acción (servicio).
-- Estados principales: EXPLORE -> APPROACH -> ENTERING -> EXECUTE -> SPIN_180 -> EXIT_PATCH -> EXPLORE
-- "Blindaje fuerte": durante ENTERING, EXECUTE y SPIN_180 (y parte de EXIT_PATCH)
-- se reduce o ignora la reacción a sensores de proximidad para garantizar que la
-- maniobra de entrada/servicio no sea interrumpida por pequeñas detecciones.
-- Forzar escritura con punto decimal para el CSV y evitar confusión con separación de coma ","
pcall(function() os.setlocale("C", "numeric") end)
------------------------ PARÁMETROS DE LOCOMOCIÓN Y ARBITRAJE ------------------------
local BASE_SPEED = 8.0
local TURN_GAIN  = 4.0
local THRESHOLD  = 0.035      -- Umbral de proximidad para evasión (Qupa)
local VMAX       = 8.0
local avoid_counter = 0      
local avoid_vL      = 0      
local avoid_vR      = 0      
local AVOID_TICKS   = 15
-- Pesos de score 
local W_DIST     = 0.4
local W_ALIGN    = 0.4
-- Entrada / servicio / salida (TIEMPO DE SERVICIO SE RECALCULA SEGÚN EL MODELO)
local ENTER_OFFSET_T   = 30     -- Empuje dentro del TAM
local ENTER_BLACK_N    = 4
local SPIN_SPEED       = 8.0
local SPIN_TICKS_180   = 40
local EXIT_UNBLACK_N   = 3
local EXIT_MAX_T       = 250
local LOST_TOL_T       = 40
local EXIT_AVOID_DELAY = 200      -- ticks sin evasión al inicio de EXIT_PATCH
-- Aproximación al blob
local ANG_GAIN         = 10.0
local ANG_DEADBAND     = 0.03
-- Compromiso de entrada
local NEAR_ANG_TH      = 20.0    -- |ángulo| pequeño -> centrado 0.20 10.
local NEAR_DIST_TH     = 0.30    -- si hay .distance 0.65 0.15
local NEAR_AREA_TH     = 1500    -- si no hay .distance, usar área aprox
local HARD_AVOID_TH    = 0.10    -- solo evasión extrema puede frenar entrada
-- Seleccion de tarea 
local commit_ok = false -- Bandera: solo entrar si hubo aceptación explícita
-- Controla que solo haya un intento de decisión tardía por encuentro de tareas
local late_tried = false
------------------------ MODELO DE ESPECIALIZACIÓN PAPER ------------------------
local T_TICKS_PER_SEC  = 10.0     -- Ticks por segundo en el simulador (ajustar si es diferente)
local GAMMA          = 1.0        -- Parámetro de la función logística
local N_MAX          = 12.0       -- Contador para aprendizaje máximo
local W_STD          = 120.0      -- Tiempo de servicio base (segundos) 
local W_MIN          = 24.0       -- Tiempo de servicio mínimo (segundos) 
local DF_DECAY_DIST  = 300.0      -- Distancia para decaimiento (cm) 300 cm papaer
local CROSS_FORGET_FACTOR = 1.0   -- Factor de olvido cruzado 
local K_GAIN = 1.25               -- Ganancia para curva sigmoide de servicio 

local POS_UNITS_IN_METERS = true  -- ARGoS expone robot.position en metros
local CM_PER_POS_UNIT     = POS_UNITS_IN_METERS and 100.0 or 1.0
local GREEDY_MODE         = false -- true => baseline codicioso, false => selectivo
local LOG_TELEMETRY       = false -- true => imprime telemetría clave (compacta)
-- Variables de Estado del Modelo
local memory_m       = 0.0        -- Contador de memoria 'm' (rango: [-10, 10])
local learn_count_b  = 0.0        -- Contador de aprendizaje 'n_b' (BLUE/TAREA 1)
local learn_count_g  = 0.0        -- Contador de aprendizaje 'n_g' (GREEN/TAREA 2)
local last_position  = {x=0.0, y=0.0} -- Para calcular la distancia recorrida (olvido)
local accumulated_dist = 0.0      -- Acumulador para olvido discreto
-- Variables de Tiempo de Búsqueda
local current_search_ticks = 0
local last_search_ticks    = 0 -- Snapshot para el CSV al momento de aceptar
-- Resumen de eventos 
metrics = {
  accept = 0, reject = 0,
  accept_late = 0, reject_late = 0,
  exec_green = 0, exec_blue = 0,
  wticks_sum_g = 0, wticks_sum_b = 0
}
-- ====== Sidecar CSV enriquecido para EXECUTE_START ======
local SIM_TICK = 0
local _exec_csv = nil
local _exec_header_written = false
local last_accept_kind = "NA"      -- "ACCEPT" | "ACCEPT_LATE" | "NA"
local last_p_g_decision = -1       -- p_g del instante de decisión (temprana o tardía)
local last_task_kind = "NONE"      -- task latched en el momento de aceptar
-- === Cabecera única global por archivo ===
local function _open_exec_csv_once()
  if _exec_csv ~= nil then return end

  local csv_path     = "arena_execstart.csv"
  local header_path  = "arena_execstart.header"  -- marcador simple

  -- 1) Ver si el CSV ya tiene contenido
  local need_header = true
  do
    local fr = io.open(csv_path, "r")
    if fr then
      local first = fr:read(1)
      if first ~= nil then need_header = false end
      fr:close()
    end
  end

  -- 2) Intento de “marcar” que alguien ya escribió cabecera (mitiga duplicado)
  if need_header then
    local mr = io.open(header_path, "r")
    if mr then
      mr:close()
      need_header = false
    else
      -- Creamos el marcador
      local mw = io.open(header_path, "w")
      if mw then mw:write("1"); mw:close() end
      -- seguimos con need_header=true => escribiremos la cabecera
    end
  end

  -- 3) Abrimos en append y, si aún procede, escribimos la cabecera
  _exec_csv = io.open(csv_path, "a")
  if not _exec_csv then
    log("ERROR: no pude abrir " .. csv_path .. " para append")
    return
  end

  if need_header and not _exec_header_written then
    _exec_csv:write("tick,greedy,robot,accept_kind,m,p_g,planned_wticks,task,search_ticks\n")
    _exec_csv:flush()
    _exec_header_written = true
  end
end

local function write_exec_start_row(planned_wticks, p_g)
  _open_exec_csv_once()
  if not _exec_csv then return end
  
  local rid = robot.id or "unknown"
  
  -- TRUCO: Formatear manualmente para asegurar punto decimal
  -- math.floor para enteros, y string.format con %.2f forzado
  local m_str   = string.format("%.2f", memory_m):gsub(",", ".")
  local pg_str  = (type(p_g)=="number") and string.format("%.2f", p_g):gsub(",", ".") or "-1"
  local wt_str  = string.format("%d", math.floor((planned_wticks or 0)+0.5))
  
  local row = string.format("%d,%s,%s,%s,%s,%s,%s,%s,%d\n",
                            SIM_TICK, tostring(GREEDY_MODE), rid, last_accept_kind, 
                            m_str, pg_str, wt_str, tostring(last_task_kind),
                            (last_search_ticks or 0))
  _exec_csv:write(row)
  _exec_csv:flush()
end
-- ====== Fin sidecar CSV ======
------------------------ FSM & HELPERS ------------------------
local STATE = "EXPLORE"
local target_kind = "NONE"
local lost_ticks   = 0
local enter_black_streak = 0

local entering_ticks = 0
local service_ticks  = 0
local spin_ticks     = 0
local exit_ticks     = 0
local unblack_streak = 0
local exit_avoid_delay = 0
local entering_prelock = false -- Se usa para un bloqueo de movimiento forzado
local reject_turn_ticks = 0 -- variable de giro tras rechazo de tarea
-- VARIABLES DEL WATCHDOG (ANTI-ATASCO)
local wd_last_pos = {x=0.0, y=0.0}
local wd_check_timer = 0
local WD_CHECK_INTERVAL = 50  -- Chequear cada 5 segundos (50 ticks)
local WD_MOVE_TH = 0.10       -- Debe moverse al menos 5cm
local RECOVERY_TICKS = 0      -- Tiempo restante en modo recuperación
-- Función auxiliar para limitar valores 
local function clamp(x, min_val, max_val) 
  if x < min_val then 
    return min_val 
  elseif x > max_val then 
    return max_val  -- retorna el valor máximo si se excede
  end 
  return x 
end
local function has(tbl, key) return (tbl and tbl[key] ~= nil) end

local function apply_and_return(vL, vR)
  if robot.wheels and robot.wheels.set_velocity then
    vL = clamp(vL, -VMAX, VMAX); vR = clamp(vR, -VMAX, VMAX)
    robot.wheels.set_velocity(vL, vR)
  end
  return true
end
-- apply_and_return: aplica velocidades a las ruedas y devuelve true.
-- Esto centraliza el límite de velocidad (clamp) y evita repetir comprobaciones en todo el código.
local function cam_blobs()
  local cam = robot.colored_blob_omnidirectional_camera
  if not cam then return {} end
  return cam.readings or cam
end
-- cam_blobs: devuelve la lista de blobs detectados por la cámara omnidireccional
-- en el formato esperado (algunos entornos exponen readings, otros devuelven la tabla directa).
local function task_type_of_blob(b)
  if not b or not b.color then return "NONE" end
  local r = b.color.red   or 0
  local g = b.color.green or 0
  local bl= b.color.blue  or 0
  -- Tarea 1 es AZUL (BLUE), Tarea 2 es VERDE (GREEN).
  if bl > r and bl > g and bl > 20 then return "BLUE" end
  if g  > r and g  > bl and g  > 20 then return "GREEN" end
  -- Si el escenario usa RED para Tarea 1, descomentar la siguiente línea:
  -- if r > bl and r > g and r > 20 then return "RED" end
  return "NONE"
end
-- task_type_of_blob: clasifica un blob según su color dominante.
-- Retorna "BLUE", "GREEN" o "NONE". (Hay una nota para habilitar RED si se usa.)
local function score_blob(b)
  local s = 0.0
  if b.distance then s = s + W_DIST  * (1.0 / math.max(0.05, b.distance)) end
  if b.angle    then s = s + W_ALIGN * math.cos(b.angle) end
  return s
end
-- score_blob: heurística para priorizar blobs. Considera distancia (inversa)
-- y alineamiento (coseno del ángulo). Se usan W_DIST y W_ALIGN como pesos.
local function choose_target_blob()
  local best, best_score = nil, -1
  for _, b in ipairs(cam_blobs()) do
    local t = task_type_of_blob(b)
    if t == "BLUE" or t == "GREEN" or t == "RED" then
      local s = score_blob(b)
      if s > best_score then best, best_score = b, s end
    end
  end
  return best
end
-- choose_target_blob: recorre los blobs visibles y selecciona el de mayor "score".
-- Filtra por tareas reconocidas (BLUE/GREEN/RED).

-- RANDOM-WALK / EVASIÓN EXACTA
local function drive_avoidance_exact()
    -- 1. BLINDAJE TOTAL
    if STATE == "ENTERING" or STATE == "EXECUTE" or STATE == "SPIN_180" or (STATE == "EXIT_PATCH" and exit_avoid_delay > 0) then
        return BASE_SPEED, BASE_SPEED, 0.0
    end
    if avoid_counter > 0 then
        avoid_counter = avoid_counter - 1
        return avoid_vL, avoid_vR, 1.0 -- Retornamos magnitud alta para mantener estado
    end
    -- 2. Obtener lecturas
    local prox = robot.qupa_proximity or robot.proximity
    local readings = prox and (prox.readings or prox) or {}
    -- Si no hay lecturas, seguimos recto
    if #readings == 0 then return BASE_SPEED, BASE_SPEED, 0.0 end

    -- 3. Mapeo de Sensores (Lua inicia índices en 1)
    -- Sensor 0: Frontal           -> readings[1]
    -- Sensor 1: Frontal derecha   -> readings[2]
    -- Sensor 2: Frontal izquierda -> readings[3]
    
    local val_f  = readings[1] and readings[1].value or 0.0
    local val_fr = readings[2] and readings[2].value or 0.0
    local val_fl = readings[3] and readings[3].value or 0.0

    -- Detectamos "obstáculo"
    local det_f  = val_f  > THRESHOLD
    local det_fr = val_fr > THRESHOLD
    local det_fl = val_fl > THRESHOLD

    -- Calculamos la "magnitud" máxima para devolverla a la FSM
    local mag = math.max(val_f, val_fr, val_fl)
    local vL, vR = BASE_SPEED, BASE_SPEED
    local trigger_avoid = false

    -- 4. LÓGICA DE DECISIÓN (Random Walk con Sensores)
    if mag >= THRESHOLD then
        trigger_avoid = true
        -- CASO A: Detectan los 3 sensores (Bloqueo total) -> Giro Aleatorio
        if det_f and det_fr and det_fl then
            if robot.random.uniform(0.0, 1.0) > 0.5 then
                -- Girar Derecha
                vL = BASE_SPEED + TURN_GAIN
                vR = BASE_SPEED - TURN_GAIN
            else
                -- Girar Izquierda
                vL = BASE_SPEED - TURN_GAIN
                vR = BASE_SPEED + TURN_GAIN
            end

        -- CASO B: Detecta Frontal + Derecha (o solo Derecha) -> Girar Izquierda (Huir del obstáculo)
        elseif (det_f and det_fr) or det_fr then
            vL = BASE_SPEED - TURN_GAIN
            vR = BASE_SPEED + TURN_GAIN

        -- CASO C: Detecta Frontal + Izquierda (o solo Izquierda) -> Girar Derecha (Huir del obstáculo)
        elseif (det_f and det_fl) or det_fl then
            vL = BASE_SPEED + TURN_GAIN
            vR = BASE_SPEED - TURN_GAIN

        -- CASO D: Solo Frontal (sin laterales) -> Decisión aleatoria (Safety)
        -- Si no ponemos esto, el robot chocaría de frente contra un poste fino.
        elseif det_f then
             if robot.random.uniform(0.0, 1.0) > 0.5 then
                vL = BASE_SPEED + TURN_GAIN
                vR = BASE_SPEED - TURN_GAIN
            else
                vL = BASE_SPEED - TURN_GAIN
                vR = BASE_SPEED + TURN_GAIN
            end
        end
    end

    -- 5. GUARDAR EN MEMORIA
    if trigger_avoid then
        avoid_counter = AVOID_TICKS -- "Recordar" esta decisión por 15 ticks
        avoid_vL = clamp(vL, -VMAX, VMAX)
        avoid_vR = clamp(vR, -VMAX, VMAX)
        return avoid_vL, avoid_vR, mag
    end

    return BASE_SPEED, BASE_SPEED, mag
end
-- drive_avoidance_exact: algoritmo de evasión basado en lecturas de proximidad.
-- Suma vectores de sensores, calcula magnitud y ángulo, y gira en función de eso.
-- IMPORTANTE: respeta el "blindaje" en estados críticos, devolviendo movimiento
-- recto para no interrumpir la entrada/servicio.
-- En el robot QUPA se lee el único sensor de tierra
local function ground_avg_gray()
  local g = robot.qupa_ground or robot.ground or robot.base_ground or robot.motor_ground
  if not g then return nil end
  local readings = g.readings or g
  -- Asume que el Qupa tiene UN SOLO sensor (la primera lectura)
  local v = nil
  if type(readings) == "table" and #readings > 0 then
    local r1 = readings[1]
    if type(r1) == "number" then v = r1
    elseif type(r1) == "table" then
      if r1.value ~= nil then v = r1.value
      elseif r1.gray  ~= nil then v = r1.gray end
    end
  elseif type(readings) == "number" then
    v = readings
  end

  if v ~= nil then
    if v > 2.0 then v = v / 255.0 end -- Normalizar si no lo está
    return v
  end
  return nil
end
-- ground_avg_gray: devuelve el valor normalizado del sensor de suelo (gris).
-- Maneja varias formas en que el sensor puede estar expuesto (número, tabla, campos).
local function on_black_patch()
  local avg = ground_avg_gray()
  return (avg ~= nil) and (avg < 0.15)
end
-- on_black_patch: detecta si el robot está sobre un parche negro usando el umbral 0.15.
local function set_led_red(on)
  if has(robot,"leds") and robot.leds.set_all_colors then
    if on then robot.leds.set_all_colors(255,0,0) else robot.leds.set_all_colors(0,0,0) end
  end
end
-- set_led_red: enciende/apaga todos los leds en rojo para indicar estado de servicio.

-- FUNCIÓN CRÍTICA DEL MODELO: DECISIÓN ESTOCÁSTICA 
local function calculate_prob_green()
  -- Función logística: p_g = 1 / (1 + exp(-gamma * m))
  return 1.0 / (1.0 + math.exp(-GAMMA * memory_m))
end

-- calculate_prob_green: implementación de la probabilidad estocástica de aceptar
-- la tarea GREEN basada en la memoria 'm' usando una función logística.

-- FUNCIÓN APRENDIZAJE DEL MODELO: TIEMPO DE SERVICIO VARIABLE 
-- calculate_service_time: calcula el tiempo de servicio en ticks en función
-- del contador de aprendizaje n_x usando curva sigmoide.

local function calculate_service_time(n_count)
  
  -- 1. Condición estricta: Si es novato (n=0), tiempo estándar 120seg
  if n_count <= 0 then
    return math.ceil(W_STD * T_TICKS_PER_SEC)
  end

  -- 2. Definimos el centro 'c' para la simetría 
  local c = N_MAX / 2.0 
  
  -- 3. Cálculo de la Sigmoide (factor de progreso de 0 a 1)
  local sigmoid = 1.0 / (1.0 + math.exp(-(n_count - c)))
  
  -- 4. Cálculo del tiempo usando 'k' 
  -- La ganancia máxima (tiempo ahorrado) es W_STD / k.
  -- Con k=1.25, el ahorro es 96s, resultando en w_min=24s.
  local time_saved = (W_STD / K_GAIN) * sigmoid
  local t_sec = W_STD - time_saved
  
  -- Retornamos ticks enteros (techo)
  return math.ceil(t_sec * T_TICKS_PER_SEC) 
end

-- Telemetría ligera para auditoría (controlada por LOG_TELEMETRY)

local function _num(x) return (type(x) == "number") and x or 0 end

function log_evt(evt, extra)
  extra = extra or {}

  if LOG_TELEMETRY then
    local msg = string.format(
      "[%s] S=%s tk=%s m=%.2f nb=%.2f ng=%.2f search=%d",
      tostring(evt), STATE, tostring(target_kind), memory_m, learn_count_b, learn_count_g, current_search_ticks
    )
    if extra.note      then msg = msg .. " note=" .. tostring(extra.note) end
    if extra.p_g       then msg = msg .. string.format(" p_g=%.2f", extra.p_g) end
    if extra.w_ticks or extra.wticks or extra.w or extra.service_ticks then
      local show_w = extra.w_ticks or extra.wticks or extra.w or extra.service_ticks
      msg = msg .. string.format(" w_ticks=%d", show_w)
    end
    print(msg)
  end

  -- Contadores de decisiones
  if evt == "ACCEPT"      then metrics.accept      = metrics.accept + 1 end
  if evt == "REJECT"      then metrics.reject      = metrics.reject + 1 end
  if evt == "ACCEPT_LATE" then metrics.accept_late = metrics.accept_late + 1 end
  if evt == "REJECT_LATE" then metrics.reject_late = metrics.reject_late + 1 end

  -- Inicio de servicio: sumar w_ticks a las métricas por color
  if evt == "EXECUTE_START" then
    local wt = _num(extra.w_ticks or extra.wticks or extra.w or extra.service_ticks or 0)
    if target_kind == "GREEN" then
      metrics.exec_green   = metrics.exec_green + 1
      metrics.wticks_sum_g = metrics.wticks_sum_g + wt
    else
      metrics.exec_blue    = metrics.exec_blue + 1
      metrics.wticks_sum_b = metrics.wticks_sum_b + wt
    end
  end
end

-- Entra de forma atómica a ENTERING
local function start_entering()
  STATE                = "ENTERING"
  entering_ticks       = ENTER_OFFSET_T
  enter_black_streak   = 0
  lost_ticks           = 0
  entering_prelock     = true -- Evitar doble trigger por fallback
  
  accumulated_dist     = 0.0 -- RESET: La distancia acumulada de la búsqueda infructuosa ha terminado.
  last_position        = robot.position or last_position -- Resetear la posición base para la siguiente distancia.
  
  -- empuje inicial, sin evasión. El blindaje se aplica en drive_avoidance_exact.
  return apply_and_return(BASE_SPEED, BASE_SPEED)
end

-- start_entering: pasa el robot al estado ENTERING y aplica un empuje inicial.
-- El "blindaje" posterior evita que la evasión detenga la entrada.

-- ESTRATEGIA ESPECIAL PARA SALIR DE CAJAS ESTRECHAS
-- Ignora todo excepto obstáculos frontales críticos
-- ESTRATEGIA "OJOS DE CABALLO" PARA SALIR
-- Ignora TOTALMENTE las paredes laterales. Solo reacciona si choca de frente.
local function drive_exit_tunnel()
    local prox = robot.qupa_proximity or robot.proximity
    if not prox then return BASE_SPEED, BASE_SPEED end
    
    local readings = prox.readings or prox
    -- Si no hay lecturas, aceleramos a tope (asumimos libertad)
    if #readings == 0 then return VMAX, VMAX end

    -- Mapeo directo de hardware
    -- readings[1] es Sensor 0 (0° Frontal)
    -- readings[2] es Sensor 1 (-45° Derecha)
    -- readings[3] es Sensor 2 (+45° Izquierda)
    
    local val_f = readings[1] and readings[1].value or 0.0
    
    -- PARAMETRO: Qué tan cerca debe estar la pared para considerar "Bloqueo Frontal"
    local BLOCK_THRESHOLD = 0.5 

    -- LÓGICA SIMPLIFICADA "OJOS DE CABALLO":
    -- Solo nos importa si tenemos una pared INMEDIATAMENTE enfrente (Sensor 0).
    -- Ignoramos totalmente los sensores 1 y 2 para que el robot roce las paredes laterales al salir.

    if val_f > BLOCK_THRESHOLD then
        -- CASO 1: Bloqueo frontal real (chocó contra el fondo)
        -- Giramos fuerte sobre el eje.
        
        -- Truco: Si también tenemos lectura en la derecha (sensor 2), giramos izquierda.
        local val_fr = readings[2] and readings[2].value or 0.0
        local val_fl = readings[3] and readings[3].value or 0.0
        
        local turn_speed = BASE_SPEED
        
        if val_fr > val_fl then
             -- Más obstáculo a la derecha -> Girar Izquierda
             return -turn_speed, turn_speed 
        else
             -- Más obstáculo a la izquierda (o igual) -> Girar Derecha
             return turn_speed, -turn_speed
        end
    else
        -- CASO 2: Frente "libre" (val_f bajo)
        -- ACELERACIÓN MÁXIMA PARA VENCER FRICCIÓN
        -- Aunque los sensores diagonales (1 y 2) estén tocando pared, empujamos recto.
        return VMAX, VMAX 
    end
end
------------------------ LIFECYCLE -------------------------
function init()
  if robot.colored_blob_omnidirectional_camera then
    robot.colored_blob_omnidirectional_camera.enable()
  end
  STATE, target_kind = "EXPLORE", "NONE"
  commit_ok = false
  late_tried = false
  set_led_red(false)
  
  -- Inicialización del estado del modelo
  memory_m      = 0.0
  learn_count_b = 0.0
  learn_count_g = 0.0
  accumulated_dist = 0.0
  
  -- Inicializar posición para el olvido por distancia
  if robot.position then
    last_position = robot.position
  else
    last_position = {x=0.0, y=0.0}
  end

  -- variables para evitar atascos
  wd_last_pos = {x=0.0, y=0.0}
  wd_check_timer = WD_CHECK_INTERVAL
  RECOVERY_TICKS = 0
  if robot.position then wd_last_pos = robot.position end

  -- Mostrar evento de inicio
  log_evt("INIT")
end

-- init: inicializa sensores y variables del modelo al arrancar la simulación.

function reset()
  STATE, target_kind = "EXPLORE", "NONE"
  lost_ticks, enter_black_streak = 0, 0
  entering_ticks, service_ticks  = 0, 0
  spin_ticks, exit_ticks, unblack_streak = 0, 0, 0
  exit_avoid_delay = 0
  entering_prelock = false
  set_led_red(false)
  if has(robot, "wheels") and robot.wheels.set_velocity then robot.wheels.set_velocity(0,0) end
  
  -- Resetear variables del modelo
  memory_m      = 0.0
  learn_count_b = 0.0
  learn_count_g = 0.0
  accumulated_dist = 0.0 
  late_tried = false

  -- Reset sidecar y búsqueda
  last_accept_kind = "NA"
  last_p_g_decision = -1
  last_task_kind = "NONE" 
  current_search_ticks = 0
  last_search_ticks = 0

  -- Reset atascos
  wd_last_pos = {x=0.0, y=0.0}
  wd_check_timer = WD_CHECK_INTERVAL
  RECOVERY_TICKS = 0
end

-- reset: limpia estados y variables cuando se resetea la simulación.

-- FUNCIÓN CRÍTICA DEL MODELO: OLVIDO POR DISTANCIA  
local function apply_distance_decay()
  local current_pos = robot.position
  if not current_pos then return end 

  -- Calcular delta distancia
  local dx = current_pos.x - last_position.x
  local dy = current_pos.y - last_position.y
  local dist_units = math.sqrt(dx*dx + dy*dy)
  local dist_cm = dist_units * CM_PER_POS_UNIT
  
  last_position = current_pos
  accumulated_dist = accumulated_dist + dist_cm

  -- Evento discreto por distancia:
  -- "upon traveling for a distance df... counters are decremented" 
  while accumulated_dist >= DF_DECAY_DIST do
    accumulated_dist = accumulated_dist - DF_DECAY_DIST
    
    -- 1. Olvido de Habilidad (Learning)
    learn_count_b = clamp(learn_count_b - 1.0, 0.0, N_MAX)
    learn_count_g = clamp(learn_count_g - 1.0, 0.0, N_MAX)

    -- 2. Olvido de Especialización (Memory 'm') 
    -- "m is decremented... while searching... to prevent dead-locks"
    -- Relaja 'm' hacia 0.
    if memory_m > 0.0 then
      memory_m = clamp(memory_m - 1.0, 0.0, 10.0) -- No cruzar 0
    elseif memory_m < 0.0 then
      memory_m = clamp(memory_m + 1.0, -10.0, 0.0) -- No cruzar 0
    end
    
    if LOG_TELEMETRY then
       print(string.format("[FORGET] Dist trigger! m=%.1f nb=%.1f ng=%.1f", memory_m, learn_count_b, learn_count_g))
    end
  end
end

-- apply_distance_decay: implementa el olvido discreto cada DF_DECAY_DIST.
-- Reduce n_b, n_g y relaja 'm' hacia 0 para evitar deadlocks en entornos cambiantes.

function step()
  
-- [MEJORADO] CONSTANTES ANTI-ATASCO LOCALES
  local WD_INTERVAL = 30   -- Chequear cada 30 ticks (3 segundos) -> Más rápido
  local WD_MIN_DIST = 0.08 -- Debe moverse al menos 8cm (Si se mueve menos, está atascado)
  
  -- LÓGICA ANTI-ATASCO (WATCHDOG)
  if STATE == "EXPLORE" or STATE == "APPROACH" or STATE == "TURNING_AWAY" then
      -- Inicialización segura
      if not wd_check_timer then wd_check_timer = WD_INTERVAL end
      if not wd_last_pos then wd_last_pos = robot.position or {x=0,y=0} end
      
      wd_check_timer = wd_check_timer - 1
      if wd_check_timer <= 0 then
          local curr = robot.position
          if curr then
             local dx = curr.x - wd_last_pos.x
             local dy = curr.y - wd_last_pos.y
             local dist = math.sqrt(dx*dx + dy*dy)
             
             -- Si no se movió lo suficiente, activamos ESCAPE
             if dist < WD_MIN_DIST then
                 STATE = "RECOVERY"
                 RECOVERY_TICKS = 100 -- 2.0 segundos de maniobra total
                 log_evt("STUCK_DETECTED", { note="Force Escape" })
             end
             wd_last_pos = curr
          end
          wd_check_timer = WD_INTERVAL
      end
  end

  -- [MEJORADO] ESTADO DE RECUPERACIÓN (MANIOBRA DE 2 FASES)
  if STATE == "RECOVERY" then
      if not RECOVERY_TICKS then RECOVERY_TICKS = 0 end
      RECOVERY_TICKS = RECOVERY_TICKS - 1
      
      -- Fase 1 (Primeros 10 ticks): RETROCESO PURO
      -- Sacamos al robot del problema alejándolo recto
      if RECOVERY_TICKS > 10 then
          apply_and_return(-BASE_SPEED, -BASE_SPEED)
      
      -- Fase 2 (Últimos 10 ticks): GIRO BRUSCO
      -- Cambiamos la orientación para no volver a mirar el obstáculo
      else
          -- Giro aleatorio a izquierda o derecha para no caer en bucles
          -- Usamos el ID del robot o algo aleatorio para decidir el lado
          local turn_dir = (os.time() % 2 == 0) and 1 or -1
          apply_and_return(turn_dir * BASE_SPEED, -turn_dir * BASE_SPEED)
      end
      
      if RECOVERY_TICKS <= 0 then
          STATE = "EXPLORE"
          target_kind = "NONE"
          -- Reset de variables de búsqueda para dar chance
          wd_check_timer = WD_INTERVAL 
          if robot.position then wd_last_pos = robot.position end
      end
      SIM_TICK = SIM_TICK + 1
      return
  end

  -- Contabilizar tiempo de búsqueda en estados activos
  if STATE == "EXPLORE" or STATE == "APPROACH" or STATE == "TURNING_AWAY" then
     current_search_ticks = current_search_ticks + 1
  end

  -- *** GESTIÓN DE ESTADOS DE COMPROMISO (SIN EVASIÓN) ***

  if STATE == "ENTERING" then
    -- El BLINDAJE ESTÁ EN drive_avoidance_exact
    local vL, vR = BASE_SPEED, BASE_SPEED
    entering_ticks = entering_ticks - 1
    if apply_and_return(vL, vR) then
      if entering_ticks <= 0 then
        STATE = "EXECUTE"
        -- Recalculamos el tiempo de servicio
        if target_kind == "GREEN" then
          service_ticks = calculate_service_time(learn_count_g)
        else 
          service_ticks = calculate_service_time(learn_count_b)
        end
        set_led_red(true)
        log_evt("EXECUTE_START", { wticks = service_ticks })
        write_exec_start_row(service_ticks, last_p_g_decision)
        
        -- Reset tiempo de búsqueda nueva tarea (Exito)
        current_search_ticks = 0 
      end
      SIM_TICK = SIM_TICK + 1  
      return
    end
  end

if STATE == "EXECUTE" then
    -- BLINDAJE ACTIVO: Quieto
    service_ticks = service_ticks - 1
    if apply_and_return(0, 0) then
      if service_ticks <= 0 then
        
        -- LÓGICA DE ACTUALIZACIÓN DE APRENDIZAJE Y MEMORIA (CORREGIDA)
        if target_kind == "GREEN" then
          -- Aprendizaje (n) limitado a N_MAX
          learn_count_g = clamp(learn_count_g + 1.0, 0.0, N_MAX)
          learn_count_b = clamp(learn_count_b - CROSS_FORGET_FACTOR, 0.0, N_MAX) 

          memory_m = clamp(memory_m + 1.0, -10.0, 10.0)

        elseif target_kind == "BLUE" or target_kind == "RED" then
          learn_count_b = clamp(learn_count_b + 1.0, 0.0, N_MAX)
          learn_count_g = clamp(learn_count_g - CROSS_FORGET_FACTOR, 0.0, N_MAX)
          
          memory_m = clamp(memory_m - 1.0, -10.0, 10.0)
        end

        set_led_red(false)
        STATE = "SPIN_180"
        -- Tiempo de espera para fase de retroceso previa al giro
        spin_ticks = SPIN_TICKS_180 + 120 
        log_evt("EXECUTE_END")
      end
      SIM_TICK = SIM_TICK + 1  
      return
    end
  end

  if STATE == "SPIN_180" then
    -- Lógica: 1. Retroceso -> 2. Giro -> 3. Validación
    spin_ticks = spin_ticks - 1

    -- FASE 1: Retroceso previo 
    -- Si faltan más ticks que los necesarios para girar, significa que estamos en tiempo extra de retroceso
    if spin_ticks > SPIN_TICKS_180 then
        apply_and_return(-BASE_SPEED, -BASE_SPEED) -- Ir hacia atrás recto
        SIM_TICK = SIM_TICK + 1
        return
    end

    if apply_and_return(-SPIN_SPEED, SPIN_SPEED) then
      if spin_ticks <= 0 then
        STATE = "EXIT_PATCH"
        exit_ticks, unblack_streak = 0, 0
        exit_avoid_delay = EXIT_AVOID_DELAY
      end
      SIM_TICK = SIM_TICK + 1  
      return
    end
  end

  -- RECHAZO Y HUÍDA
  -- Este estado fuerza al robot a girar para perder de vista la tarea rechazada
  if STATE == "TURNING_AWAY" then
    reject_turn_ticks = reject_turn_ticks - 1
    -- Giro en su propio eje para desorientarse del objetivo actual
    apply_and_return(BASE_SPEED, -BASE_SPEED) 
    
    if reject_turn_ticks <= 0 then
      STATE = "EXPLORE"
      target_kind = "NONE"
    end
    SIM_TICK = SIM_TICK + 1
    return
  end

  if STATE == "EXIT_PATCH" then
    exit_ticks = exit_ticks + 1

    -- 1. Movimiento: Usamos la estrategia de "Ojos de Caballo"
    local vL, vR = drive_exit_tunnel()

    -- 2. Lógica de Salida con TIEMPO MÍNIMO FORZADO
    local EXIT_MIN_T = 200  -- 4 segundos ciegos obligatorios
    local EXIT_CONFIRM_STREAK = 5 
    local finished = false

    if exit_ticks > EXIT_MIN_T then
        local avg = ground_avg_gray()
        if avg and avg >= 0.40 then 
          unblack_streak = unblack_streak + 1
        else
          unblack_streak = 0
        end

        if unblack_streak >= EXIT_CONFIRM_STREAK or exit_ticks >= EXIT_MAX_T then
            finished = true
        end
    else
        unblack_streak = 0
    end

    if finished then
      STATE, target_kind = "EXPLORE", "NONE"
      enter_black_streak = 0
      entering_prelock = false
      commit_ok = false   
      log_evt("EXIT_DONE")
      if robot.wheels then robot.wheels.set_velocity(BASE_SPEED, BASE_SPEED) end
      
      -- Reset búsqueda para una nueva tarea a realizar (ciclo completo)
      current_search_ticks = 0 
      
      SIM_TICK = SIM_TICK + 1
      return
    end

    if apply_and_return(vL, vR) then SIM_TICK = SIM_TICK + 1; return end
  end

  -- *** GESTIÓN DE ESTADOS DE LOCOMOCIÓN (CON EVASIÓN) ***

  --------------- EXPLORE ---------------
  if STATE == "EXPLORE" then
    local target = choose_target_blob()

    apply_distance_decay()

    if target then
      target_kind = task_type_of_blob(target)
      lost_ticks = 0
      commit_ok = false
      late_tried = false
      STATE = "APPROACH"
    else
      local vL, vR = select(1, drive_avoidance_exact())
      if apply_and_return(vL, vR) then SIM_TICK = SIM_TICK + 1; return end
    end
  end

  --------------- APPROACH --------------
  if STATE == "APPROACH" then
    local target_blob = nil
    for _, b in ipairs(cam_blobs()) do
      if task_type_of_blob(b) == target_kind then
        target_blob = b
        break
      end
    end

    if not target_blob then
      -- Búsqueda breve
      lost_ticks = lost_ticks + 1
      local t = robot.random.uniform(-TURN_GAIN, TURN_GAIN)
      if apply_and_return(-t, t) then
        if lost_ticks > LOST_TOL_T then
          STATE, target_kind = "EXPLORE", "NONE"
          commit_ok = false
        end
        SIM_TICK = SIM_TICK + 1  
        return
      end
    else
      local ang, dist, area = target_blob.angle or 0.0, target_blob.distance, target_blob.area

      -- 1) Criterio de "compromiso" (CON UMBRALES YA AJUSTADOS ARRIBA)
      local near_by_dist = (dist ~= nil) and (dist <= NEAR_DIST_TH)
      local near_by_area = (dist == nil)  and (area ~= nil) and (area >= NEAR_AREA_TH)
      local centered     = (math.abs(ang) <= NEAR_ANG_TH)
      local closeish     = centered and (near_by_dist or near_by_area)

      local turn_cmd = (math.abs(ang) < ANG_DEADBAND) and 0 or clamp(ang*ANG_GAIN, -TURN_GAIN, TURN_GAIN)
      local vL_app, vR_app = BASE_SPEED - turn_cmd, BASE_SPEED + turn_cmd
      local avg_edge = ground_avg_gray()
      if avg_edge and (avg_edge < 0.22) and not commit_ok then
        local slow = 0.5 * BASE_SPEED
        vL_app = slow - turn_cmd
        vR_app = slow + turn_cmd
      end

      if closeish then
        -- 2) LÓGICA DE DECISIÓN ESTOCÁSTICA
        local p_g = calculate_prob_green()
        local p_target
        if GREEDY_MODE then
          p_target = 1.0   
        else
          p_target = (target_kind == "GREEN") and p_g or (1.0 - p_g)
        end

        local u = robot.random.uniform(0.0, 1.0)
        if u <= p_target then
          -- ACEPTO LA TAREA
          commit_ok = true
          start_entering() 
          last_accept_kind  = "ACCEPT"
          
          -- Registrar decisión real 1.0 si es greedy
          if GREEDY_MODE then
             last_p_g_decision = 1.0
          else
             last_p_g_decision = p_g
          end
          
          last_task_kind    = target_kind
          
          -- Tiempo de búsqueda snapshot
          last_search_ticks = current_search_ticks

          log_evt("ACCEPT", { p_g = p_g })
          SIM_TICK = SIM_TICK + 1  
          return 
        else
          -- RECHAZO LA TAREA: GIRAR Y HUIR
          -- En lugar de volver a EXPLORE y ver la misma caja, 
          -- giramos forzosamente para romper contacto visual.
          STATE = "TURNING_AWAY"
          reject_turn_ticks = 25 -- ~90 grados aprox (ajustable)
          
          commit_ok = false
          log_evt("REJECT", { p_g = p_g })
          SIM_TICK = SIM_TICK + 1  
          return
        end
      end

      -- 3) Arbitraje con evasión
      local vL, vR
      local prox_vL, prox_vR, pmag = drive_avoidance_exact()

      if closeish then
        if pmag >= HARD_AVOID_TH then vL, vR = prox_vL, prox_vR else vL, vR = vL_app, vR_app end
      else
        if pmag >= THRESHOLD then vL, vR = prox_vL, prox_vR else vL, vR = vL_app, vR_app end
      end

      -- 4) Último chance en el borde
      if apply_and_return(vL, vR) then
        if on_black_patch() and not entering_prelock then
          enter_black_streak = enter_black_streak + 1
          if enter_black_streak >= ENTER_BLACK_N then
            if not commit_ok then
              if not late_tried then
                late_tried = true
                local p_g_edge = calculate_prob_green()
                local p_t_edge = GREEDY_MODE and 1.0 or ((target_kind == "GREEN") and p_g_edge or (1.0 - p_g_edge))
                local u_edge   = robot.random.uniform(0.0, 1.0)
                if u_edge <= p_t_edge then
                  commit_ok = true
                  last_accept_kind  = "ACCEPT_LATE"
                  
                  -- Registrar decisión real 1.0 si es greedy
                  if GREEDY_MODE then
                    last_p_g_decision = 1.0
                  else
                    last_p_g_decision = p_g_edge
                  end
                  
                  last_task_kind    = target_kind
                  
                  -- Varaible tiempo de búsqueda snapshot (late)
                  last_search_ticks = current_search_ticks

                  log_evt("ACCEPT_LATE", { p_g = p_g_edge })
                  if start_entering() then SIM_TICK = SIM_TICK + 1; return end
                else
                  log_evt("REJECT_LATE", { p_g = p_g_edge })
                  apply_and_return(-BASE_SPEED, -BASE_SPEED)
                  STATE, target_kind = "EXPLORE", "NONE"
                  commit_ok = false
                  enter_black_streak = 0
                  SIM_TICK = SIM_TICK + 1  
                  return
                end
              else
                apply_and_return(-BASE_SPEED, -BASE_SPEED)
                STATE, target_kind = "EXPLORE", "NONE"
                commit_ok = false
                enter_black_streak = 0
                SIM_TICK = SIM_TICK + 1  
                return
              end
            else
              if start_entering() then SIM_TICK = SIM_TICK + 1; return end
            end
          end
        else
          enter_black_streak = 0
        end
        SIM_TICK = SIM_TICK + 1  
        return
      end
    end
  end

  SIM_TICK = SIM_TICK + 1 
end

function destroy()
  if robot.colored_blob_omnidirectional_camera then
    robot.colored_blob_omnidirectional_camera.disable()
  end

  local ag = metrics.exec_green
  local ab = metrics.exec_blue
  local avg_g = (ag > 0) and (metrics.wticks_sum_g / ag) or 0
  local avg_b = (ab > 0) and (metrics.wticks_sum_b / ab) or 0

  log("=== SUMMARY ===")
  log(string.format("ACCEPT: %d | REJECT: %d | ACCEPT_LATE: %d | REJECT_LATE: %d",
      metrics.accept, metrics.reject, metrics.accept_late, metrics.reject_late))
  log(string.format("EXEC G: %d (avg wticks=%.1f) | EXEC B: %d (avg wticks=%.1f)",
      ag, avg_g, ab, avg_b))

  -- cerrar sidecar
  if _exec_csv then _exec_csv:flush(); _exec_csv:close(); _exec_csv = nil end
end

-- destroy: limpia/deshabilita sensores al final de la simulación.