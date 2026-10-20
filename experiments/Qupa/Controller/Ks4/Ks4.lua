--=============================================
--   MEJORAS :  -- Modulos de reconstrucción y guardado de fragmentos optimizado.
                -- Nuevo experimento presentado.
--=============================================
-------------------------------------------------------------------------------------------------------------------------------------
-------------------------------------------------------------------------------------------------------------------------------------
-------------------------------------------------------------------------------------------------------------------------------------
--                                                  VARIABLES GLOBALES                                                             --
-------------------------------------------------------------------------------------------------------------------------------------
-------------------------------------------------------------------------------------------------------------------------------------
-------------------------------------------------------------------------------------------------------------------------------------

--================== ACTIVAR PARA QUPA ==================
-----------------------------
--NOTAS: 
    -- Formula para calcular la dstancia de detección de observación: d = tan(angulo_de_apertura)*altura_de_la_camara -> Qupa: d = tan(70.71°) * 0.245 = 0.7 m.
        -- El angulo se modifica en el archivo .argos:   <qupa id="Qupa6" omnidirectional_camera_aperture="70.71">

-----------------------------
--Parámetro de simulación externos

-- 187014    semilla 1er exp. FOOT
math.randomseed(os.time())

ticks_per_second                    = 4                         -- Número de ticks que representa un segundo en la simulación (ajusta según la configuración de .argos)
max_time                            = 5000 * ticks_per_second   -- Duración máxima de la simulación en ticks
Distribucion_sensores_de_proximidad = {0,-45,45,-90, 90,180}    -- Ángulos de los sensores de proximidad a considerar para la detección de obstáculos (en grados) --> {} o "uniformes"
--Distribucion_sensores_de_proximidad = "uniform"               -- Ángulos de los sensores de proximidad a considerar para la detección de obstáculos (en grados) --> {} o "uniformes"
Num_cantidad_leds                   = 2
FOOTBOT_INTERWHEEL_DISTANCE         = 14                        -- Distancia entre ruedas del Qupa (en metros)
-- Importamos 
math_helper = require("Functions.math_helper")

------------------------------
--Parámetro de simulación internos

factor_vel  = 1                   -- Factor para ajustar la velocidad del master al ejecutar el patrón
speed_dem   = 10*factor_vel       -- 
speed_dem_g = (speed_dem/2)       -- 

d_speed     = 30                  -- velocidad en linea recta del apprentice
g_speed     = 80                  -- Velocidad de giro del apprentice

------------------------------
-- Modelos

-- Métricas del modelo de absevación
distance_treshold       = 20         -- Sensibilidad del apprentice a caambios en los puntos obtenidos del maestro durante la etapa de observación.

-- Métricas del modelo de paciencia
MAX_PATIENCE            = 160 * ticks_per_second       -- Tiempo máximo de espera sin recibir datos útiles en segundos
BASE_RESTORE            = 20  * ticks_per_second       -- Paciencia base a restaurar con datos refinados/actualizados en segundos

-- Métricas del modelo de ActiveLearning
back_threshold          = 10      -- Distancia que retrocede el apprentice para evitar colisión. cm.
front_threshold         = 30      -- Limite de distancia minima entre un objeto detectado y el apprentice para entrar a evadirlo
max_tracking_distance   = 10      -- Máxima distancia a la que se puede alejar el centroide antes de que el apprentce se acerque para no perder de vista. cm
porcentaje              = 0.75    -- 1 < porcentaje < 0.33 , indica que tanto el apprentice busca alejarse del maestro cuando este cruza el max_tracking_distance.

------------------------------
-- Módulos

-- Métricas del módulo de ruido en camara de blobs del aprendiz: noise_manager.lua
noise_cfg = {
    ENABLE_NOISE            = true ,    -- Si es false, el robot ve perfecto (Ground Truth)
    USE_DISTANCE_DEPENDENT  = true  ,    -- El ruido aumenta si el objeto está lejos
    USE_OUTLIERS            = false ,    -- Activar lecturas "fantasma" o fallos graves
    USE_BIAS                = true  ,    -- Error sistemático (ej. sensor mal calibrado +2cm siempre)
    -- 
    NOISE_BASE_RADIUS       = 0.000 ,    -- Ruido constante base (ej. 1 cm de error mínimo)
    NOISE_SIGMAS            = 2.0   ,    -- 2 sigmas = 95% de probabilidad de caer dentro del 'Radio Actual'
    NOISE_FACTOR            = 0.1   ,    -- Crece un 10% respecto a la distancia (ej. a 100cm, suma 5cm extra al radio si es 0.05)
    OUTLIER_PROB            = 0.01  ,    -- % de probabilidad de fallo catastrófico
    BIAS_OFFSET             = 0.0   ,    -- Cm extra que agrega el sensor por defecto (si USE_BIAS es true)
}

-- Métricas del módulo: Point_Cloud_Segmenter.lua y Segmenter_Parameterizer.lua 
seg_cfg = {
    distance_treshold       = 15,                  -- Menor que la longitud de lado más pequeña del/los comportamiento a aprender (cm)
    limite_maximo_ventana   = (30)/factor_vel,     --
    tamano_subsampling      = 15,                  -- 
    use_subsampling         = false,               --
}

-- Métricas del módulo de reconstrucción de comportamientos de movimiento: Global_Trajectory_Reconstructor.lua y Parameter_Storage.lua
-- Esta tabla centraliza los hiperparámetros de tolerancia geométrica, pesos de evaluación, filtrado de ruido y protecciones de rendimiento en tiempo real para el sistema de reconstrucción.
CONFIG = {
    -- ------------------------------------------------------------------------
    -- 1. TOLERANCIAS GEOMÉTRICAS Y SOLAPES
    
    -- Margen de error permitido en distancia (en centímetros).
    -- Implicación: Define cuán estrictos somos para considerar que dos distancias detectadas corresponden al mismo segmento. 
    -- Si es muy bajo, el ruido real romperá los datos verdaderos; si es muy alto, unirá cosas dispares.
    TOL_DIST = 15,
    
    -- Margen de error permitido en ángulos (en grados).
    -- Implicación: Tolerancia angular aceptada para determinar si dos rectas, tramos son colineales.
    TOL_ANG = 15,
}

------------------------------
-- Parámetros de patrónes predefinidos a ejecutar por los master
distance_factor = 1  -- Factor de escala para ajustar el tamaño de las figuras predefinidas (ajusta según tu espacio de simulación)

if robot.id == "Qupa4" then

elseif robot.id == "Qupa0" then
    figura = "triangulo_irregular_1" -- "triangulo_regular" -- "octagono" -- "triangulo_irregular_1" --"Pentágono_irregular"  -- "Heptágono"-- "" -- "trayectoria_1" 
    distance_factor = 1
elseif robot.id == "Qupa1" then
    figura = "triangulo_regular" -- "triangulo_regular" -- "octagono" -- "triangulo_irregular_1" --"Pentágono_irregular"  -- "Heptágono"-- "" -- "trayectoria_1" 
    distance_factor = 1
elseif robot.id == "Qupa2" or robot.id == "Qupa3" or robot.id == "Qupa5" then
    figura = "Pentágono_irregular" -- "triangulo_regular" -- "octagono" -- "triangulo_irregular_1" --"Pentágono_irregular"  -- "Heptágono"-- "" -- "trayectoria_1" 
    distance_factor = 1
elseif robot.id == "Qupa6" or robot.id == "Qupa7" then
    figura = "octagono" -- "triangulo_regular" -- "octagono" -- "triangulo_irregular_1" --"Pentágono_irregular"  -- "Heptágono"-- "" -- "trayectoria_1" 
    distance_factor = 1
end

------------------------------
-- Banderas de loggeo
SRP_LOG_MODE        = 0 -- 0: Sin logs, 1: Parámetros obtenidos tras finalizar la observación de un maestro. 2. Paso a paso de la actualización de maestros, 3: Imprime todo.
UP_LOG_MODE         = 2 -- 0: Sin logs, 1: Delimitador de ciclos de aprendizaje 2: Información sobre la paciencia (Tiempo que el apprentice actualiza para continuar o deter su aprendizaje)
AL_LOG_MODE         = 0 -- 0: Sin logs, 1: Información sobre el Active Learning, 2: Información detallada del Active Learning, 3: Estados del active learning
DEBUG_EVASION       = 0 -- 0: Nada, 1: Cambios de estado, 2: Sensores y estados continuos

-------------------------------------------------------------------------------------------------------------------------------------
-------------------------------------------------------------------------------------------------------------------------------------
-------------------------------------------------------------------------------------------------------------------------------------
--                                                      MAIN FUNCTIONS                                                             --
-------------------------------------------------------------------------------------------------------------------------------------
-------------------------------------------------------------------------------------------------------------------------------------
-------------------------------------------------------------------------------------------------------------------------------------

-------------------------------------------------------------------------------------------------------------------------------------
-- Funciones para los trackers 
    -- Estas funciones se mantienen comentadas por defecto. Si se requieren se llaman en arun_sim.py y este las descomenta para que se ejecuten automaticamente.
-------------------------------------------------------------------------------------------------------------------------------------

-- function Print_tracker_paciencia(current_patience_seg)

--     print("[PACIENCIA](" .. current_patience_seg.. ") segundos.") 
-- end

-- function Print_tracker_trayectory_data(p1, p2, p3, p4, p5, p6, p7, p8)
--     -- Retrocompatibilidad: Si el primer parámetro no es "trajectory" ni "Active_learning"
--     if type(p1) ~= "string" or (p1 ~= "trajectory" and p1 ~= "Active_learning") then
--         local k, x, y, logging = p1, p2, p3, p4
--         if type(k) == "number" then
--             print("Master" .. k .. " & " .. x .. " , " .. y)
--             if logging then log("Master" .. k .. " & " .. x .. " , " .. y) end
--         elseif type(k) == "string" then
--             local rx = robot.positioning.position.x
--             local ry = robot.positioning.position.y
--             if k == "Qupa4" then
--                 print("Apprentice & " .. rx .. " , " .. ry)
--             elseif k == "App" then
--                 print("Patrón".. (master_indice or "") .." & " .. rx .. " , " .. ry)
--             else
--                 print(k .. " & " .. rx .. " , " .. ry)
--             end
--         end
--         return
--     end

--     -- [NUEVO FORMATO] Usamos PUNTO Y COMA (;) para evitar choques con decimales
--     if p1 == "trajectory" then
--         local k, x, y, logging = p2, p3, p4, p5
--         if type(k) == "number" then
--             print("[trajectory];Master" .. k .. ";" .. x .. ";" .. y)
--             logging = false
--             if logging then log("[trajectory];Master" .. k .. ";" .. x .. ";" .. y) end
--         elseif type(k) == "string" then
--             local rx = robot.positioning.position.x
--             local ry = robot.positioning.position.y
--             if k == "Qupa4" then
--                 print("[trajectory];Apprentice;" .. rx .. ";" .. ry)
--             elseif k == "App" then
--                 print("[trajectory];Patrón".. (master_indice or "") ..";" .. rx .. ";" .. ry)
--             else
--                 print("[trajectory];" .. k .. ";" .. rx .. ";" .. ry)
--             end
--         end
        
--     -- [NUEVO FORMATO] Datos enriquecidos de Active Learning
--     elseif p1 == "Active_learning" then
--         local al_mode, name, x, y, extra1, extra2, extra3 = p2, p3, p4, p5, p6, p7, p8
--         -- El aprendiz usa siempre su posición global real para anclar los vectores
--         if type(name) == "string" and name == "Qupa4" then
--             x = robot.positioning.position.x
--             y = robot.positioning.position.y
--             name = "Apprentice"
--         end
--         local out = "[active_learning];" .. al_mode .. ";" .. name .. ";" .. x .. ";" .. y
--         if extra1 ~= nil then out = out .. ";" .. extra1 end
--         if extra2 ~= nil then out = out .. ";" .. extra2 end
--         if extra3 ~= nil then out = out .. ";" .. extra3 end
--         print(out)
--     end
-- end

-- function Print_tracker_AL_PID(target, actual, control_vars)
--     local str = string.format("[PID];%.4f;%.4f", target, actual)
--     -- Recorremos la tabla para agregar cada variable con su nombre (ej: ;out_speed_d=30.0)
--     for name, value in pairs(control_vars) do
--         str = str .. string.format(";%s=%.4f", name, value)
--     end
--     print(str)
-- end

-- function Print_tracker_calidad_imitacion_data(linea_resumen, is_master)

--     if is_master == 0 then

--         print(linea_resumen)
--     else

--         -- Función auxiliar para separ   ar la parte entera y los dos decimales de un número
--         local function split_num(val)
--             local sign = val < 0 and "-" or ""
--             local abs_val = math.abs(val)
--             local int_part = math.floor(abs_val)
--             local dec_part = math.floor((abs_val - int_part) * 100 + 0.5)
            
--             -- Ajuste por si el redondeo sube el decimal a 100
--             if dec_part >= 100 then
--                 int_part = int_part + 1
--                 dec_part = dec_part - 100
--             end
            
--             -- Retorna dos valores: la parte entera (con su signo) y la parte decimal a 2 dígitos
--             return string.format("%s%d", sign, int_part), string.format("%02d", dec_part)
--         end
        
--         -- Obtenemos el patrón correspondiente a la variable global 'figura' actual
--         local data = original_params()
        
--         -- Verificamos que se haya encontrado un patrón válido
--         if not data or not data.params then 
--             log(robot.id .. " & Error: No se encontró un patrón asignado.")
--             return 
--         end
        
--         local nombre_patron = data.name or "Desconocido"
--         -- Iniciamos la cadena con el formato solicitado
--         local resultado = robot.id .. " & Original&" .. nombre_patron
        
--         local params = data.params
--         for j = 1, #params do
--             -- En original_params, la tabla es {distancia, angulo}
--             local dist = params[j][1]
--             local ang = params[j][2]
            
--             -- Separamos enteros y decimales de la distancia y el ángulo
--             local dist_i, dist_d = split_num(dist)
--             local ang_i, ang_d = split_num(ang)
            
--             -- Concatenamos el paso con el formato (angI,angD,distI,distD)
--             resultado = resultado .. string.format("(%s,%s,%s,%s)", ang_i, ang_d, dist_i, dist_d)
            
--             -- Agregamos la coma separadora entre pasos (excepto en el último)
--             if j < #params then
--                 resultado = resultado .. ","
--             end
--         end
--         -- Imprimimos el resultado final en la terminal de ARGoS
--         log(resultado)
--     end
-- end

-- function Print_tracker_varianza(k, var)
--     print( "Maestro"..k.." & " .. var)             -- Datos para el anális de varianza en los datos obtenidos por el observador. Script -> sun_sim.py
-- end

-- function evaluator_otsu(separacion, avance)
--     print( "Otsu & " .. separacion.." , "..avance) -- Datos para el anális del método de otsu a distintos valores de velocidad del demostrador: speed_dem  -> Script: master_evluator.py
-- end

-- function evaluator_ruido (raw_angle, raw_dist, noisy_angle, noisy_dist)
--     print("[EVALUATOR_RUIDO]Blob : angulo = " .. raw_angle .. ", distancia = " .. raw_dist)
--     --log("\n[EVALUATOR]Blob perfecto: angulo = " .. raw_angle .. ", distancia = " .. raw_dist)
--     print("[EVALUATOR_RUIDO]Blob ruidoso : angulo = " .. noisy_angle .. ", distancia = " .. noisy_dist)
-- end

-------------------------------------------------------------------------------------------------------------------------------------
-- Función que se ejecuta al inicio de la simulación
-------------------------------------------------------------------------------------------------------------------------------------

function init()
    
    robot.colored_blob_omnidirectional_camera.enable()

    determinar_rol()
    
    if Print_tracker_calidad_imitacion_data then Print_tracker_calidad_imitacion_data(robot.id, is_master) end

    if is_master == 0 then
        local current_x     = robot.positioning.position.x
        local current_y     = robot.positioning.position.y
        local orientation   = robot.positioning.orientation
        local ang           = math_helper.vec2_angle(orientation)

        if Print_tracker_trayectory_data then print("[trajectory];" .. "[offset];" .. current_x .. ";" .. current_y .. ";" .. ang) end

        if Print_tracker_paciencia then 
            print("[PACIENCIA]("..(MAX_PATIENCE/ticks_per_second)..")&" .. ticks_per_second)
        end

        if Print_tracker_varianza then
            if seg_cfg.use_subsampling then
                print("[varianza](tamano_subsampling="..seg_cfg.tamano_subsampling)
            end
        end
    end
end
-------------------------------------------------------------------------------------------------------------------------------------
-- Función principal que se ejecuta en cada tick
-------------------------------------------------------------------------------------------------------------------------------------

timer = 1
function step()

    if robot.id == "Qupa4" and 
        ( evaluator_ruido or Print_tracker_trayectory_data or Print_tracker_paciencia or Print_tracker_AL_PID) then 
            print("Timer="..timer) 
    end

    if is_master == 1 then
        master() -- Ejecutar si es master
    else
        apprentice()
    end

    if robot.id == "Qupa4" and timer == max_time then
        print("End_simulation")
    end

    timer = timer + 1
end

-------------------------------------------------------------------------------------------------------------------------------------
-- Funciones para cada rol
-------------------------------------------------------------------------------------------------------------------------------------

all_params              = {}    -- Almacenará todos los patrones descubiertos
parametros_elegidos     = {}    -- El patrón específico seleccionado para ejecutar

d1                      = 2
master_avg_dist         = 0
master_indice           = 0
function master()
    --d1 = 2 -- ESTAMOS PROVISIONALMENTE FORZANDO AL DEMOSTRADOR A EJECUTAR EL COMPORTAMIENTO DIRECTAMENTE

    if d1 == 0 then
        detectar_zona_demostracion()
        --log("detectar_zona_demostracion")

    elseif d1 == 1 then   
        espera_antes_de_ejecutar_patron()
        --log("espera_antes_de_ejecutar_patron")

    elseif d1 == 2 then
        -- =========================================================
        -- Selección aleatoria del comportamiento a demostrar
        -- =========================================================

        -- Verificamos que existan comportamientos guardados en la tabla
        if #all_params > 0 then

            if master_indice == #all_params then
                d1 = 4
                return
            end
            
            master_indice = master_indice + 1
            parametros_elegidos = all_params[master_indice]
            
        else
            if robot.id == "Qupa4" then
                log("No se detectaron comportamientos en la etapa de observación, "..robot.id)
                d1 = 4
                return
            end
            -- Respaldo de seguridad por si la tabla aún está vacía
            parametros_elegidos = original_params()
            parametros_elegidos = parametros_elegidos.params
        end

        d1 = 3

    elseif d1 == 3 then
        Ejecutar_comportamiento(parametros_elegidos)

        if PI_end then
            resetDEM()
            d1 = 2
        end
    elseif d1 == 4 then
        robot.wheels.set_velocity(0, 0)
        setleds("black")
        print("End_simulation")
    end
end

c4      = 0
AP_1    = 1
function apprentice()

    -- === ESTADO DE APRENDIZAJE ===
    if c4 == 0 then 
        store_relative_positions()
        
    -- === ESTADOS DE EXPLORACIÓN (RANDOM WALK) ===
    elseif c4 == 1 then

        AP_1 = 1
        rw_timer = rw_timer + 1
        if Print_tracker_trayectory_data then Print_tracker_trayectory_data("trajectory", "Qupa4", 0, 0) end

        RandomWalking()

        if rw_timer >= TIEMPO_RW_TICKS then
            robot.wheels.set_velocity(0, 0)
            r_move()                 
            r_Temporizador_1()
            r_move()
            rw_timer = 0             
            c4 = 2                   
        end

    elseif c4 == 2 then
        -- Limpieza selectiva de memoria: Manda los IDs ganadores para que el global buffer descarte los inútiles
        local current_cfg = { CONFIG = CONFIG, LOG_MODE = SRP_LOG_MODE }
        ParameterStorage.clear(current_cfg, current_winning_ids)
        current_winning_ids = {} -- Vaciamos tras usar

        soft_reset_observer()

        local current_x = robot.positioning.position.x
        local current_y = robot.positioning.position.y        
        local orientati = robot.positioning.orientation
        local ang       = (math_helper.vec2_angle(orientati)/ 180) * math.pi

        if Print_tracker_trayectory_data then print("[trajectory];" .. "[offset];" .. current_x .. ";" .. current_y .. ";" .. ang) end

        c4 = 0 -- Regresa a observar

    -- === ESTADOS DE EJECUCUCION DE COMPORTAMIENTOS APRENDIDOS ===
    elseif c4 == 3 then
        local NAC = navegar_a_coordenadas(0,0,0.10)
        if NAC then
            c4 = 4
        end 
        
    elseif c4 == 4 then
        resetOBS()
        is_master = 1
    end
end

-------------------------------------------------------------------------------------------------------------------------------------
-------------------------------------------------------------------------------------------------------------------------------------
-------------------------------------------------------------------------------------------------------------------------------------
--                                          Funciones de comportamiento de los Qupa                                               --
-------------------------------------------------------------------------------------------------------------------------------------
-------------------------------------------------------------------------------------------------------------------------------------
-------------------------------------------------------------------------------------------------------------------------------------

-------------------------------------------------------------------------------------------------------------------------------------
-------------------------------------------------------------------------------------------------------------------------------------
-- MODULO DEL MAESTRO
-------------------------------------------------------------------------------------------------------------------------------------
-------------------------------------------------------------------------------------------------------------------------------------

function detectar_zona_demostracion()
    RandomWalking()
    setleds("black")
    local G = ground{
        range = {
            {0.21, 0.3 },   -- gris 2
            }                
        , print_all_data        = false
        , print_detected_ranges = false
        -- base_ground_active = false
    }

    if #G.Gr > 0 then
        d1 = 1
        robot.wheels.set_velocity(0, 0)
        r_Temporizador_1()
        r_move()

        -- if robot.id == "Qupa3" then 
        --     setleds("red")
        --     log("Zona de demostración detectada, "..robot.id)
        -- end
        --setleds("red")
    end
end

EAEP_c1 = 0
function espera_antes_de_ejecutar_patron()
    if EAEP_c1 == 0 then
        local blobs = robot.colored_blob_omnidirectional_camera
        local total_distance = 0
        local count = 0  
        local vel_g = 10
        master_avg_dist = 0

        Temporizador_1(15)

        --setleds("yellow")
        if temp == false then
            EAEP_c1 = 1
            --setleds("yellow")
        end
        
    else
        d1 = 2
        r_move()
        r_Temporizador_1()
        -- log("Inicio de demostración por " .. robot.id)
    end
end

-------------------------------------------------------------------------------------------------------------------------------------
-- función para realizar el comportamiento de movimiento (trayectoria)
-------------------------------------------------------------------------------------------------------------------------------------
SMART_MASTERS = true -- true: Se detienen si hay obstáculo. false: Son ciegos.

indice          = 1
PI_end          = false
PI_counter_seg  = 0

function Ejecutar_comportamiento(params_P)
    local params = params_P
    local prox = proximidad()
    local master_paused = false

    -- =========================================================================
    -- LÓGICA DE PAUSA INTELIGENTE
    -- =========================================================================
    if SMART_MASTERS and robot.id ~= "Qupa4" then
        -- Si hay un obstáculo cerca en el cono frontal
        if prox[1] == 1 and ( math.abs(prox[2]) <= 45 ) then 
            master_paused = true
            -- setleds("red") -- Opcional: Indicador de pausa por obstáculo
        end
    end
    -- =========================================================================

    if not PI_end then
        if params and #params >= 1 then
            if indice <= #params then
                local valor1, valor2 = params[indice][1], params[indice][2]
                
                -- ¡CLAVE! Siempre llamamos a move, inyectando la bandera de pausa al final
                move(valor1, valor2, speed_dem, speed_dem_g, false, robot.id, master_paused)

                -- Solo avanzamos la lógica del patrón si NO estamos pausados
                if not master_paused then
                    if robot.id == "Qupa4" then 
                        if Print_tracker_trayectory_data then Print_tracker_trayectory_data("App", 0, 0) end
                    end
                    setleds("yellow")

                    -- c1 == 2 indica que el segmento terminó con éxito dentro de move()
                    if c1 == 2 then
                        PI_counter_seg = PI_counter_seg + 1
                        if indice == #params then
                            indice = indice + 1
                            PI_end = true
                            r_Temporizador_1()
                        else
                            indice = indice + 1
                            c1 = 0
                        end
                    end
                end
            end
        else
            PI_end = true
            robot.leds.set_all_colors("black")
            log( " -------------------------------------------------------------------------" )
            log("Patrón no detectado :( " .. robot.id)
            log( " -------------------------------------------------------------------------" )
        end
    end

    if PI_end then
        robot.wheels.set_velocity(0, 0)   -- Detenerse
        setleds("black")
    end
end

-------------------------------------------------------------------------------------------------------------------------------------
-------------------------------------------------------------------------------------------------------------------------------------
-- MODULO DEL OBSERVADOR (Con Active Learning Integrado)
-------------------------------------------------------------------------------------------------------------------------------------
-------------------------------------------------------------------------------------------------------------------------------------

-- ============================================================================
-- [SECCIÓN 1] IMPORTACIÓN DE SUBMÓDULOS DEL OBSERVADOR
-- ============================================================================
NoiseManager                    = require("Functions.Observer.noise_manager")
PointCloudSegmenter             = require("Functions.Observer.Point_Cloud_Segmenter")
SegmentParameterizer            = require("Functions.Observer.Segment_Parameterizer")
ParameterStorage                = require("Functions.Observer.Parameter_Storage")
GlobalTrajectoryReconstructor   = require("Functions.Observer.Global_Trajectory_Reconstructor")
ImitationQuality                = require("Functions.Observer.ImitationQuality") -- NUEVA IMPORTACIÓN

-- ActiveLearning                  = require("Functions.Observer.Active_learning.Centroide_basic")
-- ActiveLearning                  = require("Functions.Observer.Active_learning.Centroide_threshold")
-- ActiveLearning                  = require("Functions.Observer.Active_learning.Centroide_PID")
ActiveLearning                  = require("Functions.Observer.Active_learning.CamposVectoriales")
-- ActiveLearning                  = require("Functions.Observer.Active_learning.Estatico")
ENABLE_ACTIVE_LEARNING          = true
-- AL_SMOOTH_TICKS                 = 4           -- Cantidad de ticks a recordar (tamaño del buffer)
-- AL_SMOOTH_METHOD                = "median"    -- Puede ser "mean" (media/promedio) o "median" (mediana)

-- ============================================================================
-- [SECCIÓN 2] VARIABLES GLOBALES DE ESTADO Y MEMORIA
-- ============================================================================

-- Variables para lógica de exploración y redundancia (N)
N_MAX_INTENTOS          = 3
intentos_redundantes    = N_MAX_INTENTOS
TIEMPO_RW_TICKS         = 60 * ticks_per_second -- 30 segundos de Random Walk
rw_timer                = 0
K_UMBRAL_SIMILITUD      = 0.85  -- 85% de similitud de Qi (0.85) para considerarlo un patrón "conocido"

current_patience  = MAX_PATIENCE
master_tracking   = {} -- Memoria detallada de cada maestro observado
srt_anterior_data = {} -- Registro temporal para enlazar blobs entre ticks (tracking)

-- Variables globales para el Active Learning (Movimiento del Observador)
motion_active       = false
last_motion_marker  = {0, 0}
srt_ang_mark_ant    = 0
x_mark, y_mark      = 0, 0
x_ant_r, y_ant_r    = 0, 0
current_wait_ticks  = 0
obs_stop_flag       = false
MAX_WAIT_TICKS      = 2*ticks_per_second -- Ajusta este valor según la paciencia deseada frente a un obstáculo

al_current_state    = "IDLE"

SP_CRITICAL_THRESHOLD = 0.10 
function change_al_state(new_state, log_msg, finish_reason)
    if al_current_state ~= new_state then
        -- Si salimos de un estado activo, avisamos su fin con el MOTIVO EXACTO
        if AL_LOG_MODE == 3 and al_current_state ~= "IDLE" then
            local razon = finish_reason or "Razón desconocida"
            log(string.format(">>> [FIN] Estado '%s' - Razón: %s \n", al_current_state, razon))
        end
        
        al_current_state = new_state
        
        -- Si entramos a un estado activo, mostramos la info
        if AL_LOG_MODE == 3 and new_state ~= "IDLE" then
            log(string.format("\n>>> [ALERTA] %s", log_msg))
        end
    end
end
-- ============================================================================
-- [SECCIÓN 3] GESTIÓN DE PACIENCIA Y RECONSTRUCCIÓN
-- ============================================================================

-- Decaimientos diferenciados según la calidad de la información
DECAY_RATES = {
    [1] = 0.15, -- 15% de pérdida por cada vez que ESE fragmento hace Nivel 1
    [2] = 0.10, -- 10% de pérdida por cada vez que ESE fragmento hace Nivel 2
    [3] = 0.0   -- 0% de pérdida para Nivel 3 (Totalmente nuevo, restaura full)
}

-- Para almacenar los IDs ganadores de la última reconstrucción exitosa
current_winning_ids = {}

-- Fíjate que ahora recibe "fragment_history_count"
function update_patience(restore_level, data_count, fragment_history_count)
    if restore_level == 0 then return end -- Nivel 0: Sin restauración
    
    if restore_level == 3 then
        -- Nivel 3: Datos totalmente nuevos (Full). Ahora no decae (0%)
        local decay_factor = 1.0 
        local restore_amount = math.floor(MAX_PATIENCE * decay_factor)
        
        current_patience = math.min(MAX_PATIENCE, current_patience + restore_amount)
        
        if UP_LOG_MODE == 2 or UP_LOG_MODE == 3 then 
            log(string.format("[PACIENCIA] Restaurada (NIVEL 3). Eficacia 100%%. +%.2f seg. Actual: %.2f seg.", 
                restore_amount/ticks_per_second, current_patience/ticks_per_second))
        end

    elseif restore_level == 1 or restore_level == 2 then
        -- ¡CORRECCIÓN! Eliminamos completamente la división por la cantidad de datos.
        -- Ahora la cantidad a restaurar depende única y exclusivamente de la Eficacia del historial.
        
        local decay_factor = math.max(0, 1.0 - (fragment_history_count * DECAY_RATES[restore_level]))
        
        -- Base efectiva = Maximo a restaurar para Nivel 1 (ej: 40seg) * Eficacia
        local base_efectiva = BASE_RESTORE * decay_factor
        
        local restore_amount = math.floor(base_efectiva)
        
        -- Si es Nivel 2, el máximo a restaurar es el doble (ej: 80seg), manteniendo la proporción
        if restore_level == 2 then
            restore_amount = restore_amount * 2
        end
        
        current_patience = math.min(MAX_PATIENCE, current_patience + restore_amount)
        
        if UP_LOG_MODE == 2 or UP_LOG_MODE == 3 then 
            log(string.format("[PACIENCIA] Rest. Parcial (NIVEL %d). Eficacia %.0f%%. +%.2f seg. Actual: %.2f seg.", 
                restore_level, decay_factor * 100, restore_amount/ticks_per_second, current_patience/ticks_per_second)) 
        end
    end
end

function check_patience_and_reconstruct()
    if ParameterStorage.is_done() then return end

    -- El contador se reduce constantemente con el paso del tiempo
    current_patience = current_patience - 1
    if Print_tracker_paciencia then Print_tracker_paciencia(current_patience/ticks_per_second) end

    if current_patience <= 0 then
        
        local datos_guardados = ParameterStorage.get_stored_data()
        local current_cfg = { CONFIG = CONFIG, LOG_MODE = SRP_LOG_MODE }   
        
        local exito, linea_resumen, nuevos_comportamientos, winner_ids = GlobalTrajectoryReconstructor.run(robot.id, datos_guardados, current_cfg, #all_params)
        
        local hay_comportamiento_nuevo = false

        if exito then
            ParameterStorage.set_done()
            if winner_ids then current_winning_ids = winner_ids else current_winning_ids = {} end

            if Print_tracker_calidad_imitacion_data then Print_tracker_calidad_imitacion_data(linea_resumen, is_master) end
            
            -- LÓGICA DE SIMILITUD Y REDUNDANCIA
            if #all_params == 0 then
                hay_comportamiento_nuevo = true
                for _, comportamiento in ipairs(nuevos_comportamientos) do
                    table.insert(all_params, comportamiento)
                end
            else
                for _, nuevo_comportamiento in ipairs(nuevos_comportamientos) do
                    local es_conocido = false
                    for _, comportamiento_conocido in ipairs(all_params) do
                        local qi = ImitationQuality.evaluate(comportamiento_conocido, nuevo_comportamiento)
                        if qi >= K_UMBRAL_SIMILITUD then
                            es_conocido = true
                            if UP_LOG_MODE == 2 then log(string.format(" -> Comportamiento redundante detectado. Similitud Qi = %.2f%%", qi*100)) end
                            
                            -- PROMEDIAR PARÁMETROS DEL COMPORTAMIENTO
                            local min_len = math.min(#comportamiento_conocido, #nuevo_comportamiento)
                            for i = 1, min_len do
                                comportamiento_conocido[i][1] = (comportamiento_conocido[i][1] + nuevo_comportamiento[i][1]) / 2.0
                                comportamiento_conocido[i][2] = (comportamiento_conocido[i][2] + nuevo_comportamiento[i][2]) / 2.0
                            end
                            
                            if #nuevo_comportamiento > #comportamiento_conocido then
                                for i = min_len + 1, #nuevo_comportamiento do
                                    table.insert(comportamiento_conocido, {nuevo_comportamiento[i][1], nuevo_comportamiento[i][2]})
                                end
                            end
                            break
                        end
                    end
                    
                    if not es_conocido then
                        hay_comportamiento_nuevo = true
                        table.insert(all_params, nuevo_comportamiento)
                        if UP_LOG_MODE == 2 then log(" >>> COMPORTAMIENTO NUEVO AÑADIDO AL DICCIONARIO <<<") end
                    end
                end
            end
        else
            current_winning_ids = {}
        end

        -- LÓGICA DE REDUCCIÓN DE INTENTOS Y FIN DE CICLO
        if hay_comportamiento_nuevo then
            intentos_redundantes = N_MAX_INTENTOS
            if UP_LOG_MODE >= 1 or SRP_LOG_MODE >= 1 then
                log("\n===========================================================")
                log(" [FIN CICLO DE APRENDIZAJE] NUEVO COMPORTAMIENTO GUARDADO")
                log(" Iniciando Random Walk para explorar nuevas áreas.")
                log("===========================================================\n\n\n")
            end
            c4 = 1 
        else
            intentos_redundantes = intentos_redundantes - 1
            if intentos_redundantes > 0 then
                if UP_LOG_MODE >= 1 or SRP_LOG_MODE >= 1 then 
                    log("\n===========================================================")
                    log(" [FIN CICLO DE APRENDIZAJE] REDUNDANCIA O FALLO")
                    if exito then
                        log(" -> Todo era redundante (Ya conocido). Intentos restantes N = " .. intentos_redundantes)
                    else
                        log(" -> Reconstrucción fallida (Pura basura). Intentos restantes N = " .. intentos_redundantes)
                    end
                    log(" Iniciando Random Walk para buscar mejor perspectiva.")
                    log("===========================================================\n\n\n")
                end
                c4 = 1 
            else
                if UP_LOG_MODE >= 1 or SRP_LOG_MODE >= 1 then 
                    log("\n\n\n===========================================================")
                    log(" [FIN FASE DE APRENDIZAJE] INTENTOS AGOTADOS (N=0)")
                    log("===========================================================")
                end
                
                log("\n===========================================================")
                log(" RESUMEN GLOBAL: COMPORTAMIENTOS APRENDIDOS (DICCIONARIO)  ")
                log("===========================================================")
                for idx, comportamiento in ipairs(all_params) do
                    log(string.format("\n--> Comportamiento [%d]:", idx))
                    if type(comportamiento) == "table" then
                        local params_str = {}
                        for _, p in ipairs(comportamiento) do
                            table.insert(params_str, string.format("{D:%.2f, A:%.2f}", p[1], p[2]))
                        end
                        log("    Parámetros: " .. table.concat(params_str, ", "))
                    else
                        log("    Parámetros: [Formato desconocido]")
                    end
                end
                log("===========================================================\n")

                intentos_redundantes = N_MAX_INTENTOS 
                c4 = 3
            end
        end
    end
end

-- ============================================================================
-- [SECCIÓN 4] FUNCIONES AUXILIARES DEL OBSERVADOR (TRACKING CRUDO)
-- ============================================================================

function initialize_master_tracking(k, angle_deg, distance, ty)
    local angle_rad = angle_deg * (math.pi / 180) -- Blob viene en grados, se pasa a rad
    
    -- COMPENSACIÓN: offset global del observador (Ya están en Radianes)
    local current_move_angle = motion_active and move_angle or 0
    local obs_ang_offset = srt_ang_mark_ant + current_move_angle 
    
    local ang_total = angle_rad + obs_ang_offset
    local x_rel, y_rel = math_helper.polarACartesianoPAR(distance, ang_total)
    
    local x = x_rel + x_mark
    local y = y_rel + y_mark 

    master_tracking[k] = {
        raw_x = {x}, raw_y = {y}, raw_type = {ty},
        Pattern_start_detected = (ty == 1), 
        End_of_pattern_detected = (ty == 3),
        processed = false, 
        segmentos = { movement = {}, type = {}, valid = {} },
        params = {},
    }
end

function process_master_segmentation(k, track, x_original, y_original, ty)
    table.insert(track.raw_x, x_original)
    table.insert(track.raw_y, y_original)
    table.insert(track.raw_type, ty)

    if ty == 1 then track.Pattern_start_detected = true end
    if ty == 3 then track.End_of_pattern_detected = true end
end

-- ============================================================================
-- [SECCIÓN 5] PIPELINE DE PROCESAMIENTO (OBSERVACIÓN INDIVIDUAL)
-- ============================================================================

function process_single_observation(k, track)
    if SRP_LOG_MODE == 1 or  SRP_LOG_MODE == 3 then
        log(" ........................................................ ")
        log(" PROCESANDO OBSERVACIÓN DEL MAESTRO "..k.." (Datos totales: "..#track.raw_x..")")
        log(" ........................................................ ")
    end

    local clean_mov, clean_type, clean_valid, p_start, p_end = PointCloudSegmenter.process(k, track.raw_x, track.raw_y, track.raw_type, seg_cfg)

    track.segmentos.movement = clean_mov
    track.segmentos.type     = clean_type
    track.segmentos.valid    = clean_valid
    
    if p_start then track.Pattern_start_detected  = true end
    if p_end   then track.End_of_pattern_detected = true end

    if track.segmentos.movement and #track.segmentos.movement > 1 then 
        track.params = SegmentParameterizer.process_segments(track.segmentos.movement, track.segmentos.valid)
        
        local current_cfg = { CONFIG = CONFIG }
        
        -- ¡ATENCIÓN! Capturamos el tercer valor: history_decay_level
        local r_level, r_count, history_decay_level = ParameterStorage.package_and_add(track.params, track.segmentos.valid, current_cfg)
        
        -- Y lo enviamos a update_patience para que lo evalúe de forma aislada
        if r_level and r_level > 0 then update_patience(r_level, r_count, history_decay_level) end
    else
        if SRP_LOG_MODE == 1 or  SRP_LOG_MODE == 3 then log("No hay segmentos suficientes para procesar tras la limpieza.") end
    end
end
-- ============================================================================
-- [SECCIÓN 6] BUCLE PRINCIPAL DEL OBSERVADOR (TICK A TICK)
-- ============================================================================

-- Función para normalizar ángulos entre -PI y PI y evitar que 
-- srt_ang_mark_ant crezca infinitamente tras muchas correcciones
function normalize_angle_rad(ang)
    while ang > math.pi do ang = ang - 2 * math.pi end
    while ang <= -math.pi do ang = ang + 2 * math.pi end
    return ang
end

srt_c1 = 0
function store_relative_positions()

    if AP_1 == 1 then
        if UP_LOG_MODE >= 1 or SRP_LOG_MODE >= 1 then 
            log("\n\n\n===========================================================")
            log(" [INICIO CICLO DE APRENDIZAJE] ")
            log("===========================================================")  
        end
        AP_1 = 0
    end

    if Print_tracker_trayectory_data then Print_tracker_trayectory_data("trajectory", "Qupa4", 0, 0) end
    check_patience_and_reconstruct()

    local proximity = proximidad()
    --log("Proximidad: " .. string.format("%.2f cm, %2f°", proximity[1], proximity[2]).."\n")

    if proximity[1] < 1 then 
        srt_c1 = 0 
    else
        if srt_c1 == 0 then
            log(" CHOQUE!!!!!!! ")
            srt_c1 = srt_c1 + 1
        end
    end

    Temporizador_1(20) 
    if temp == false then robot.leds.set_all_colors("black") end
    
    -- =========================================================================
    -- FASE 0: EJECUCIÓN DE ODOMETRÍA, OBSTÁCULOS Y SUPERVIVENCIA (SP)
    -- =========================================================================

    -- Distribucion_sensores_de_proximidad = {0,-45,45,-90, 90,180}    -- Ángulos de los sensores de proximidad a considerar para la detección de obstáculos (en grados) --> {} o "uniformes"

    if motion_active then
        local prox_data = proximidad() 
        obs_stop_flag = false

        local is_object_critical = (prox_data[1] > 0.9)
        local evadiendo_hacia_atras = (last_motion_marker[1] < 0)

        if is_object_critical then
            if evadiendo_hacia_atras then
                if math.abs(prox_data[2]) >= 135 then obs_stop_flag = true end
            else
                if math.abs(prox_data[2]) <= 45 then obs_stop_flag = true end
            end
        end

        if obs_stop_flag then
            current_wait_ticks = current_wait_ticks + 1
            if current_wait_ticks > MAX_WAIT_TICKS then
                srt_ang_mark_ant = normalize_angle_rad(move_angle + srt_ang_mark_ant)
                x_ant_r = x_mark
                y_ant_r = y_mark
                r_move() 
                robot.wheels.set_velocity(0, 0)
                
                local escape_dir = evadiendo_hacia_atras and 1 or -1
                last_motion_marker = {escape_dir * back_threshold * porcentaje, 0}
                current_wait_ticks = 0
                obs_stop_flag = false
                
                local msg = string.format("EVADIENDO OBJETO FÍSICO | %s | Intensidad SP: %.2f", (escape_dir > 0 and "Avanzando" or "Retrocediendo"), prox_data[1])
                change_al_state("EVASION_SP", msg)
            end
        else
            current_wait_ticks = 0
        end

        -- Extraemos las velocidades dinámicas (si el módulo no las envía, usamos las máximas por defecto)
        local dyn_sd = last_motion_marker[3] or d_speed
        local dyn_sg = last_motion_marker[4] or g_speed

        move(last_motion_marker[1], last_motion_marker[2], dyn_sd, dyn_sg, false, robot.id, obs_stop_flag)

    else
        current_wait_ticks = 0 
    end

    if robot.colored_blob_omnidirectional_camera then               
        local blobs         = robot.colored_blob_omnidirectional_camera
        local yellow_blobs  = {} 
        local green_blobs   = {}  
        local count         = 0   

        -- FASE A: Recolección y Ruido
        for i, blob in ipairs(blobs) do
            local raw_angle = blob.angle * (180 / math.pi)
            local raw_dist = blob.distance
            local noisy_dist, noisy_angle = NoiseManager.aplicar_ruido(raw_dist, raw_angle, noise_cfg) 
            local r, g, b = blob.color.red, blob.color.green, blob.color.blue

            if evaluator_ruido then evaluator_ruido (raw_angle, raw_dist, noisy_angle, noisy_dist) end

            if r > 200 and g > 200 and b < 100 then table.insert(yellow_blobs, {angle = noisy_angle, distance = noisy_dist, type = 2})
            elseif r < 100 and g > 200 and b < 100 then table.insert(green_blobs, {angle = noisy_angle, distance = noisy_dist})
            elseif  r > 200 and g < 100 and b < 100 then table.insert(yellow_blobs, {angle = noisy_angle, distance = noisy_dist, type = 1})
            end 
        end

        local updated_anterior_indices = {}
        local matched_new_indices = {}
        local fallos_maestros = {}

        -- FASE A.5: CORRECCIÓN DE ODOMETRÍA
        if motion_active then
            local ang_total_1_rad = move_angle + srt_ang_mark_ant 
            local x_new_r, y_new_r = math_helper.polarACartesianoPAR(move_distance, ang_total_1_rad)
            
            x_mark = x_new_r + x_ant_r
            y_mark = y_new_r + y_ant_r
            
            if c1 == 2 then 
                srt_ang_mark_ant = normalize_angle_rad(move_angle + srt_ang_mark_ant)
                x_ant_r = x_mark
                y_ant_r = y_mark
                motion_active = false
                current_wait_ticks = 0 
                r_move() 
                robot.wheels.set_velocity(0, 0)
                change_al_state("IDLE", "") 
            end
        else
            x_mark = x_ant_r
            y_mark = y_ant_r
        end
        
        local obs_ang_offset_global = srt_ang_mark_ant + (motion_active and move_angle or 0)

        -- FASE B: ACTUALIZACIÓN DE MAESTROS EXISTENTES
        local candidatos = {}
        for j, new_data in ipairs(yellow_blobs) do
            local ang_total_blob = (new_data.angle * (math.pi/180)) + obs_ang_offset_global
            local bx_rel, by_rel = math_helper.polarACartesianoPAR(new_data.distance, ang_total_blob)
            local bx_global = bx_rel + x_mark
            local by_global = by_rel + y_mark
            
            for k, anterior_data in pairs(srt_anterior_data) do
                if anterior_data.angle and master_tracking[k] and #master_tracking[k].raw_x > 0 then
                    local num_pts = #master_tracking[k].raw_x
                    local mx_global = master_tracking[k].raw_x[num_pts]
                    local my_global = master_tracking[k].raw_y[num_pts]
                    local error_dist = math.sqrt((bx_global - mx_global)^2 + (by_global - my_global)^2)
                    if error_dist <= distance_treshold then
                        table.insert(candidatos, {blob_idx = j, master_idx = k, error = error_dist})
                    end
                end
            end
        end

        table.sort(candidatos, function(a, b) return a.error < b.error end)

        for _, cand in ipairs(candidatos) do
            local j = cand.blob_idx
            local k = cand.master_idx
            if not matched_new_indices[j] and not updated_anterior_indices[k] then
                srt_anterior_data[k].angle = yellow_blobs[j].angle
                srt_anterior_data[k].distance = yellow_blobs[j].distance
                srt_anterior_data[k].type = yellow_blobs[j].type 
                fallos_maestros[k] = nil 
                updated_anterior_indices[k] = true
                matched_new_indices[j] = true
                count = count + 1
            end
        end

        for k, anterior_data in pairs(srt_anterior_data) do
            if anterior_data.angle and not updated_anterior_indices[k] and master_tracking[k] and #master_tracking[k].raw_x > 0 then
                local num_pts = #master_tracking[k].raw_x
                local mx_global = master_tracking[k].raw_x[num_pts]
                local my_global = master_tracking[k].raw_y[num_pts]
                local distancias_sobrantes = {}
                for j, new_data in ipairs(yellow_blobs) do
                    if not matched_new_indices[j] then
                        local ang_total_blob = (new_data.angle * (math.pi/180)) + obs_ang_offset_global
                        local bx_rel, by_rel = math_helper.polarACartesianoPAR(new_data.distance, ang_total_blob)
                        local bx_global = bx_rel + x_mark
                        local by_global = by_rel + y_mark
                        local error_dist = math.sqrt((bx_global - mx_global)^2 + (by_global - my_global)^2)
                        table.insert(distancias_sobrantes, string.format("%.2f cm", error_dist))
                    end
                end
                local msg_razon = string.format("Ningún blob válido a menos de %.2f cm.", distance_treshold)
                if #distancias_sobrantes > 0 then msg_razon = msg_razon .. " Blobs sobrantes a: " .. table.concat(distancias_sobrantes, " | ")
                else msg_razon = msg_razon .. " No se detectaron blobs sobrantes." end
                fallos_maestros[k] = msg_razon
            end
        end

        -- FASE C: CREAR NUEVOS MAESTROS
        for j, new_data in ipairs(yellow_blobs) do
            if not matched_new_indices[j] then
                table.insert(srt_anterior_data, {angle = new_data.angle, distance = new_data.distance, type = new_data.type or 2})
                local new_k = #srt_anterior_data
                updated_anterior_indices[new_k] = true
                initialize_master_tracking(new_k, new_data.angle, new_data.distance, srt_anterior_data[new_k].type)
            end
        end

        -- FASE D: PROCESAR BLOBS VERDES
        for _, green_blob in ipairs(green_blobs) do
            local ang_total_green = (green_blob.angle * (math.pi/180)) + obs_ang_offset_global
            local gbx_rel, gby_rel = math_helper.polarACartesianoPAR(green_blob.distance, ang_total_green)
            local gbx = gbx_rel + x_mark
            local gby = gby_rel + y_mark
            
            for k, master_data in pairs(srt_anterior_data) do
                if master_data.angle and not updated_anterior_indices[k] and master_tracking[k] and #master_tracking[k].raw_x > 0 then 
                    local num_pts = #master_tracking[k].raw_x
                    local mx = master_tracking[k].raw_x[num_pts]
                    local my = master_tracking[k].raw_y[num_pts]
                    local dist_error = math.sqrt((gbx - mx)^2 + (gby - my)^2)
                    if dist_error <= distance_treshold then
                        srt_anterior_data[k].type = 3 
                        updated_anterior_indices[k] = true 
                        master_tracking[k].End_of_pattern_detected = true
                        break 
                    end
                end
            end 
        end

        -- =========================================================================
        -- [INICIO DE GESTOR DE COLA DE EVASIÓN Y PAUSAS]
        -- =========================================================================
        if not al_evasion_queue then al_evasion_queue = {} end
        if not al_pause_timer then al_pause_timer = 0 end

        -- =========================================================================
        -- FASE E: CÁLCULO Y DECISIÓN DE MOVIMIENTO (ACTIVE LEARNING CENTRALIZADO)
        -- =========================================================================
        if ActiveLearning and ActiveLearning.process then
            local al_result = ActiveLearning.process(obs_ang_offset_global, updated_anterior_indices)
            
            if al_result then
                local prox_data = proximidad()
                local is_prox_critical = (prox_data[1] > al_result.prox_threshold)
                local is_cam_critical = (al_result.closest_dist < al_result.front_threshold)
                
                local is_evading = (al_current_state == "EVASION_VISUAL" or al_current_state == "EVASION_CIEGA" or al_current_state == "EVASION_LATERAL")
                local override_normal_move = false

                -- 1. DETENER MOVIMIENTO SI HAY UNA EMERGENCIA NUEVA (Y no estamos ya evadiendo)
                if (is_prox_critical or is_cam_critical) and motion_active and not is_evading then
                    srt_ang_mark_ant = normalize_angle_rad(move_angle + srt_ang_mark_ant)
                    x_ant_r = x_mark; y_ant_r = y_mark
                    motion_active = false
                    r_move()  
                    robot.wheels.set_velocity(0, 0)
                    al_evasion_queue = {} -- Limpiar cola, la nueva emergencia manda
                    al_pause_timer = 0
                end

                -- 2. GENERACIÓN DE RUTAS DE ESCAPE (Si estamos detenidos y no hay nada encolado)
                if (is_prox_critical or is_cam_critical) and #al_evasion_queue == 0 and not motion_active then
                    override_normal_move = true
                    
                    local is_sandwich = false
                    if is_prox_critical and is_cam_critical then
                        local diff = math.abs(al_result.closest_angle - prox_data[2])
                        if diff > 90 and diff < 270 then is_sandwich = true end
                    end
                    
                    if is_sandwich then
                        -- REGLA C: Sándwich -> Evasión Lateral (90° respecto a la cámara)
                        local escape_ang = al_result.closest_angle + 90
                        table.insert(al_evasion_queue, {
                            dist = al_result.back_threshold, ang = escape_ang, 
                            sp_d = d_speed, sp_g = g_speed, msg = "Evasión Lateral (Sándwich)", state = "EVASION_LATERAL"
                        })
                        
                    elseif is_cam_critical then
                        -- REGLA A: Solo Cámara o Mismo Lado -> Evasión Visual
                        local escape_ang = (al_result.closest_angle > 0) and (al_result.closest_angle - 180) or (al_result.closest_angle + 180)
                        table.insert(al_evasion_queue, {
                            dist = al_result.back_threshold, ang = escape_ang, 
                            sp_d = d_speed, sp_g = g_speed, msg = "Evasión Visual (Cámara)", state = "EVASION_VISUAL"
                        })
                        
                    elseif is_prox_critical then
                        -- REGLA B: Solo Proximidad -> Evasión Ciega en 2 Pasos
                        local sign = (prox_data[2] >= 0) and -1 or 1
                        local ang_1 = 45 * sign
                        local ang_2 = -45 * sign
                        
                        -- Si está adelante, la base de escape es hacia atrás (180). Si está atrás, hacia adelante (0).
                        local base_escape = (math.abs(prox_data[2]) <= 90) and 180 or 0
                        
                        table.insert(al_evasion_queue, {
                            dist = al_result.back_threshold, ang = base_escape + ang_1, 
                            sp_d = d_speed, sp_g = g_speed, msg = "Evasión Ciega P1", state = "EVASION_CIEGA"
                        })
                        table.insert(al_evasion_queue, {
                            dist = al_result.back_threshold, ang = ang_2, 
                            sp_d = d_speed, sp_g = g_speed, msg = "Evasión Ciega P2", state = "EVASION_CIEGA"
                        })
                    end
                end

                -- REGLA 4 / 3.a.2: Verificación de Solapamiento con el Centroide
                -- Solo se hace si no hay emergencias (override_normal_move = false)
                if not override_normal_move and al_result.is_centroid and al_result.should_move then
                    local is_overlapping = false
                    for k, data in pairs(srt_anterior_data) do
                        if updated_anterior_indices[k] and data.distance then
                            local r_x, r_y = math_helper.polarACartesianoPAR(data.distance, data.angle * (math.pi/180))
                            local dist_c_to_r = math.sqrt((al_result.target_x - r_x)^2 + (al_result.target_y - r_y)^2)
                            -- Radio de solapamiento: 15cm
                            if dist_c_to_r < 15.0 then is_overlapping = true; break end
                        end
                    end
                    
                    if is_overlapping then
                        change_al_state("ESPERANDO_DESPEJE", "Centroide solapado con robot. Modo estático.")
                        override_normal_move = true -- Nos quedamos quietos
                    end
                end

                -- 3. EJECUTOR DE COLA DE EVASIÓN Y PAUSAS
                if al_pause_timer > 0 and not motion_active then
                    al_pause_timer = al_pause_timer - 1
                
                elseif not motion_active and #al_evasion_queue > 0 then
                    local next_step = table.remove(al_evasion_queue, 1)
                    local ev_ang = next_step.ang
                    local ev_dist = next_step.dist

                    -- Normalizar ángulo a -180 / 180
                    while ev_ang > 180 do ev_ang = ev_ang - 360 end
                    while ev_ang < -180 do ev_ang = ev_ang + 360 end

                    -- OPTIMIZACIÓN REVERSA
                    if ev_ang > 90 then ev_ang = ev_ang - 180; ev_dist = -ev_dist
                    elseif ev_ang < -90 then ev_ang = ev_ang + 180; ev_dist = -ev_dist end
                    
                    motion_active = true
                    last_motion_marker = {ev_dist, ev_ang, next_step.sp_d, next_step.sp_g}
                    
                    -- AQUÍ se marca el estado para proteger la maniobra
                    change_al_state(next_step.state, next_step.msg) 
                    r_move()

                -- 4. MOVIMIENTO NORMAL Y DESVÍOS (Solo si no hay emergencias ni solapamientos)
                elseif not override_normal_move and not motion_active and al_result.should_move then
                    
                    if al_result.pause_ticks > 0 then
                        al_pause_timer = al_result.pause_ticks
                    else
                        local path_blocked = false
                        if al_result.is_centroid then
                            local T_len = al_result.target_dist
                            for k, data in pairs(srt_anterior_data) do
                                if updated_anterior_indices[k] and data.distance and data.distance < T_len then
                                    local R_ang_rad = data.angle * (math.pi/180)
                                    local T_ang_rad = al_result.target_angle * (math.pi/180)
                                    local angle_diff = R_ang_rad - T_ang_rad
                                    local p = data.distance * math.cos(angle_diff)
                                    
                                    if p > 0 and p < T_len then
                                        local dist_to_path = math.abs(data.distance * math.sin(angle_diff))
                                        if dist_to_path < (al_result.front_threshold * 0.8) then
                                            path_blocked = true; break
                                        end
                                    end
                                end
                            end
                        end
                        
                        if path_blocked then
                            -- Desvío preventivo
                            table.insert(al_evasion_queue, {
                                dist = al_result.target_dist / 2, ang = al_result.target_angle + 45, 
                                sp_d = d_speed, sp_g = g_speed, msg = "Parada Extra: Rodeando bloqueo", state = "CALCULANDO_DESVIO"
                            })
                            change_al_state("CALCULANDO_DESVIO", "Ruta bloqueada, encolando desvío.")
                        else
                            local opt_ang = al_result.target_angle
                            local opt_dist = al_result.target_dist * porcentaje
                            
                            -- OPTIMIZACIÓN REVERSA
                            if opt_ang > 90 then opt_ang = opt_ang - 180; opt_dist = -opt_dist
                            elseif opt_ang < -90 then opt_ang = opt_ang + 180; opt_dist = -opt_dist end

                            motion_active = true
                            last_motion_marker = {opt_dist, opt_ang, al_result.speed_d, al_result.speed_g}
                            change_al_state(al_result.state_name, al_result.state_msg)
                            r_move()
                        end
                    end
                end
            end
        end

        -- =========================================================================
        -- FASE G: Gestión de Pérdidas y Procesamiento de Rutas
        -- =========================================================================
        for k = 1, #srt_anterior_data do
            local track = master_tracking[k]
            local is_lost = (srt_anterior_data[k] and srt_anterior_data[k].angle and not updated_anterior_indices[k])
            local is_finished = track and track.End_of_pattern_detected
            if is_lost then
                if SRP_LOG_MODE == 1 or SRP_LOG_MODE == 3 then
                    local razon = fallos_maestros[k] or "Sin blobs amarillos cerca."
                    log("\n[!] Maestro " .. k .. " PERDIDO. Razón: " .. razon)
                end
            end
            if (is_lost or is_finished) and track and not track.processed then
                process_single_observation(k, track)
                track.processed = true
                srt_anterior_data[k].angle = nil
                srt_anterior_data[k].distance = nil
                srt_anterior_data[k].type = nil
            end
        end

        -- =========================================================================
        -- FASE H: Grabación Continua
        -- =========================================================================
        for k, _ in pairs(updated_anterior_indices) do
            local master_data = srt_anterior_data[k]
            local track = master_tracking[k]
            if master_data and track and not track.processed then
                local current_distance = master_data.distance
                local current_angle_rad = master_data.angle * (math.pi / 180) 
                local ty = master_data.type
                
                local ang_total = current_angle_rad + obs_ang_offset_global
                local x_new, y_new = math_helper.polarACartesianoPAR(current_distance, ang_total)
                local x_original = x_new + x_mark
                local y_original = y_new + y_mark

                if Print_tracker_trayectory_data then Print_tracker_trayectory_data("trajectory", k, x_original/100, y_original/100, false) end
                process_master_segmentation(k, track, x_original, y_original, ty)
            end
        end
    end 
end
-------------------------------------------------------------------------------------------------------------------------------------
-------------------------------------------------------------------------------------------------------------------------------------
-------------------------------------------------------------------------------------------------------------------------------------
--                                                      Other functions                                                            --
-------------------------------------------------------------------------------------------------------------------------------------
-------------------------------------------------------------------------------------------------------------------------------------
-------------------------------------------------------------------------------------------------------------------------------------

-------------------------------------------------------------------------------------------------------------------------------------
-------------------------------------------------------------------------------------------------------------------------------------
-- Funcines de sensoores de los Qupa
-------------------------------------------------------------------------------------------------------------------------------------
-------------------------------------------------------------------------------------------------------------------------------------

function ground(opts)
    -- Configuración por defecto
    opts = opts or {}
    local base_ground_active  = opts.base_ground_active         or false
    local print  = opts.print_all_data                          or false
    local print_detected_ranges  = opts.print_detected_ranges   or false
    local ranges = opts.range                                   or {}  -- Tabla de rangos: { {min, max}, {min, max}, ... }

    local Ground = {
        base  = {},
        motor = {},
        Gr    = {}  -- Lista de pares: { {rango_detectado, sensor_detectado}, ... }
    }

    -- Sensores base_ground
    if base_ground_active then 
        if print then
            log(" ------------------ BASE GROUND ------------------ "..robot.id)
        end
        for i, reading in ipairs(robot.base_ground) do
            Ground.base[i] = reading.value
            if print then
                log(string.format("[Base_Ground %d] valor=%.3f", i, reading.value))
            end

            for r_index, bounds in ipairs(ranges) do
                local min_val = bounds[1]
                local max_val = bounds[2]
                if reading.value >= min_val and reading.value <= max_val then
                    table.insert(Ground.Gr, {r_index, i})
                end
            end
        end
    end

    if print then
        log(" ------------------ MOTOR GROUND ----------------- "..robot.id)
    end

    -- Sensores motor_ground
    for i, reading in ipairs(robot.motor_ground) do
        Ground.motor[i] = reading.value
        if print then
            log(string.format("[Motor_Ground %d] valor=%.3f", i, reading.value))
        end

        for r_index, bounds in ipairs(ranges) do
            local min_val = bounds[1]
            local max_val = bounds[2]
            if reading.value >= min_val and reading.value <= max_val then
                local global_index = i + #robot.base_ground  -- Para distinguir sensores
                table.insert(Ground.Gr, {r_index, global_index})
            end
        end
    end

    -- Ordenar por número de rango (primer valor de cada par)
    table.sort(Ground.Gr, function(a, b)
        return a[1] < b[1]
    end)

    if print_detected_ranges then
        log(" -------------------------------------------------- "..robot.id)
    end

    if print_detected_ranges then
        if #Ground.Gr > 0 then
            log("Rangos detectados:")
            for i, pair in ipairs(Ground.Gr) do
                local rango = pair[1]
                local sensor = pair[2]
                log(string.format("  - Rango Detectado: %d por el sensor #%d,", rango, sensor))
            end
        else
            --log("No se detectaron rangos.") 
        end
    end

   return Ground
end

-- Función de detección de proximidad
-- Distribucion_sensores_de_proximidad:
--   "uniform"  -> distribución uniforme
--   tabla      -> distribución explícita por sensor, por ejemplo {0,45,90,180,-90,-45}
proximity = {0, 0}

function obtener_angulo_sensor(i, n_sensores)
    if Distribucion_sensores_de_proximidad == "uniform" or Distribucion_sensores_de_proximidad == nil then
        -- Distribución uniforme como la que ya tenías
        local N = n_sensores / 2
        local step = 180 / N

        if i <= N then
            -- 1 -> 0°, N -> 180°
            return i * step
        else
            -- N+1 -> -180°, final -> 0°
            return -180 + (i - N) * step
        end

    elseif type(Distribucion_sensores_de_proximidad) == "table" then
        -- Verificar tamaño
        if #Distribucion_sensores_de_proximidad ~= n_sensores then
            log(string.format(
                "[ERROR] La tabla de ángulos tiene %d elementos, pero hay %d sensores.",
                #Distribucion_sensores_de_proximidad, n_sensores
            ))
            return nil
        end

        return Distribucion_sensores_de_proximidad[i]

    else
        log("[ERROR] 'Distribucion_sensores_de_proximidad' debe ser 'uniform' o una tabla de ángulos.")
        return nil
    end
end

function proximidad(modo_detallado)
    local sensores = robot.proximity
    local n_sensores = #sensores

    local sum_val = 0
    local count = 0
    local sum_x = 0
    local sum_y = 0
    
    -- Variables para registrar el máximo
    local current_max_val = 0
    local current_max_angle = 0
    
    -- Tabla para guardar el registro de todos los sensores
    local lista_sensores = {}

    -- 1) Recorremos todos los sensores
    for i, s in ipairs(sensores) do
        local v = s.value
        local angle = obtener_angulo_sensor(i, n_sensores)
        
        -- Protección por si no hay ángulo
        if angle == nil then angle = 0 end 
        
        -- Guardamos la medición individual en la lista estructurada
        table.insert(lista_sensores, { id = i, valor = v, angulo = angle })

        -- Buscamos el valor máximo absoluto
        if v > current_max_val then
            current_max_val = v
            current_max_angle = angle
        end

        -- Lógica original de sumas y vectores (filtrando ruido)
        if v > 0.01 then
            local rad = math.rad(angle)
            sum_val = sum_val + v
            count = count + 1

            sum_x = sum_x + v * math.cos(rad)
            sum_y = sum_y + v * math.sin(rad)
        end
    end

    -- 2) Cálculo de promedios
    local prom_v = 0
    local prom_a = 0

    if count > 0 then
        prom_v = sum_val / count
    end

    if sum_x ~= 0 or sum_y ~= 0 then
        local angle_rad
        if math.atan2 ~= nil then
            angle_rad = math.atan2(sum_y, sum_x)
        else
            angle_rad = math.atan(sum_y, sum_x)
        end
        prom_a = math.deg(angle_rad)
    end

    -- 3) Salida según el parámetro recibido
    if modo_detallado then
        -- Retorna la nueva estructura detallada
        return {
            lista = lista_sensores,
            max_val = { valor = current_max_val, angulo = current_max_angle },
            prom_val = { valor = prom_v, angulo = prom_a }
        }
    else
        -- Comportamiento Clásico (Retrocompatibilidad)
        -- Actualizamos el array global tal cual lo tenías
        proximity[1] = prom_v
        proximity[2] = prom_a
        return proximity
    end
end

-------------------------------------------------------------------------------------------------------------------------------------
-------------------------------------------------------------------------------------------------------------------------------------
-- Funcines de acciones de los Qupa
-------------------------------------------------------------------------------------------------------------------------------------
-------------------------------------------------------------------------------------------------------------------------------------

-------------------------------------------------------------------------
-- Random Walking
-------------------------------------------------------------------------
-- Parámetros configurables de sensibilidad (Umbrales)
RW_THRESHOLD_FRONT = 0.4  -- Sensibilidad cono frontal (-45°, 0°, 45°)
RW_THRESHOLD_SIDE  = 0.8  -- Sensibilidad lateral (-90°, 90°)
RW_THRESHOLD_CLEAR = 0.3  -- Umbral para considerar que el camino vuelve a estar libre

-- Variables globales de estado para el Random Walk
RW_b1 = 0
Rw_target_angle = 0

-- Variables de Memoria para la lógica Anti-Atascos
RW_stuck_counter = 0     -- Cuenta cuántas veces seguidas choca en un corto periodo
RW_clear_path_timer = 0  -- Mide cuántos "ticks" ha avanzado sin chocar

function RandomWalking()
    -- Obtenemos la tabla detallada de la nueva función de proximidad
    local datos_prox = proximidad(true)

    if RW_b1 == 0 then
        -- ==========================================================
        -- ESTADO 0: AVANZANDO Y EVALUANDO
        -- ==========================================================
        
        RW_clear_path_timer = RW_clear_path_timer + 1
        
        -- Si avanzamos un buen rato sin obstáculos, reseteamos la memoria de atascos
        if RW_clear_path_timer > 30 and datos_prox.max_val.valor < RW_THRESHOLD_CLEAR then
            RW_stuck_counter = 0
        end

        local max_val = datos_prox.max_val.valor
        local max_ang = datos_prox.max_val.angulo

        -- ----------------------------------------------------------
        -- EVALUACIÓN DE ZONAS Y UMBRALES
        -- ----------------------------------------------------------
        local trigger_evasion = false
        
        -- 1. Zona Frontal (-45°, 0°, 45° inclusive)
        if math.abs(max_ang) <= 45 then
            if max_val > RW_THRESHOLD_FRONT then
                trigger_evasion = true
            end
            
        -- 2. Zona Lateral (90° o -90°)
        elseif math.abs(max_ang) == 90 then
            if max_val > RW_THRESHOLD_SIDE then
                trigger_evasion = true
            end
        end
        
        -- 3. Zona Trasera (180° o -180°): Se ignora.
        -- Como no cae en las condiciones de arriba, trigger_evasion se mantiene en 'false'.

        -- ----------------------------------------------------------
        -- ACCIÓN BASADA EN LA EVALUACIÓN
        -- ----------------------------------------------------------
        if trigger_evasion then
            
            -- 1. Detener motores de inmediato
            robot.wheels.set_velocity(0, 0)
            
            -- 2. Reseteamos el contador de "camino libre" y aumentamos la "frustración"
            RW_clear_path_timer = 0
            RW_stuck_counter = RW_stuck_counter + 1
            
            -- 3. TOMA DE DECISIÓN DEL ÁNGULO DE EVASIÓN
            if RW_stuck_counter >= 3 then
                -- CASO A: Atascado en un callejón o esquina compleja (Anti-Stuck)
                -- Fuerza un giro de entre 160° y 180° para salir por donde vino
                Rw_target_angle = robot.random.uniform_int(160, 180)
                
                -- Giramos hacia el lado más despejado por eficiencia
                if max_ang < 0 then 
                    Rw_target_angle = -Rw_target_angle 
                end
                
                RW_stuck_counter = 0 -- Reseteamos la frustración
                
            else
                -- CASO B: Evasión Inteligente basada en el sensor específico
                if max_ang == 0 then
                    -- Frente exacto: 50% probabilidad izq/der. Giro amplio.
                    if robot.random.uniform_int(0, 1) == 0 then
                        Rw_target_angle = robot.random.uniform_int(90, 160)
                    else
                        Rw_target_angle = robot.random.uniform_int(-160, -90)
                    end
                    
                elseif max_ang == 45 then
                    -- Obstáculo Frente-Izquierda: Girar a la derecha
                    Rw_target_angle = robot.random.uniform_int(-150, -90)
                    
                elseif max_ang == -45 then
                    -- Obstáculo Frente-Derecha: Girar a la izquierda
                    Rw_target_angle = robot.random.uniform_int(90, 150)
                    
                elseif max_ang == 90 then
                    -- Roce Lateral Izquierdo: Giro medio/suave a la derecha para separarse
                    Rw_target_angle = robot.random.uniform_int(-90, -45)
                    
                elseif max_ang == -90 then
                    -- Roce Lateral Derecho: Giro medio/suave a la izquierda para separarse
                    Rw_target_angle = robot.random.uniform_int(45, 90)
                    
                else
                    -- Fallback de seguridad
                    Rw_target_angle = robot.random.uniform_int(90, 180)
                end
            end
            
            -- Cambiamos al estado de giro
            RW_b1 = 1
            
        else
            -- Si no se disparó la evasión (frente libre, lados lejos, o sensor trasero), avanzamos.
            robot.wheels.set_velocity(30, 30)
        end

    elseif RW_b1 == 1 then
        -- ==========================================================
        -- ESTADO 1: GIRANDO PARA EVADIR
        -- ==========================================================
        move(0, Rw_target_angle, 30, 80)

        -- Revisamos las banderas globales de la función move() para saber si ya terminó
        if c1 == 2 then
            RW_b1 = 0
            Rw_target_angle = 0
            c1 = 0
            c2 = 0
        end
    end
end

-- Función para aproximar valores según la resolución física (ticks y velocidad mínima)
function redondear_resolucion(valor, min_speed, tps)
    local res = min_speed / tps
    local abs_val = math.abs(valor)
    local rounded = math.floor((abs_val / res) + 0.5) * res
    return (valor < 0) and -rounded or rounded
end

-- Variables de control originales
c1                  = 0
c2                  = 0
move_controller     = 1
move_distance       = 0 

ticks_needed        = 0
actual_dis_ticks    = 0

angle_ticks_needed  = 0
actual_ang_ticks    = 0

time_needed         = 0
speed               = d_speed

move_angle          = 0

wheel_speed         = 0
omega_rad           = 0

previous_speed_d    = nil

-- variables globales para el control residual exacto
full_ticks_a        = 0
has_rem_a           = false
rem_wheel_speed     = 0
rem_omega_rad       = 0

full_ticks_d        = 0
has_rem_d           = false
rem_speed_d         = 0

min_speed_d, min_speed_g = 5, 1  -- Velocidades mínimas para distancia y giro (en cm/s y °/s respectivamente)

function move(distance, target_angle, speed_d, speed_g, imprimir, name, stop)

    Temporizador_2(0.3)
    local stop = stop or false
    
    if stop then 
        -- Actualizar la odometría del último tick que se ejecutó físicamente
        if c1 == 0 then
            if target_angle > 0 then
                if actual_ang_ticks <= full_ticks_a then
                    move_angle = (1 / ticks_per_second) * actual_ang_ticks * omega_rad
                else
                    move_angle = (1 / ticks_per_second) * full_ticks_a * omega_rad + (1 / ticks_per_second) * (actual_ang_ticks - full_ticks_a) * rem_omega_rad
                end
            else
                if actual_ang_ticks <= full_ticks_a then
                    move_angle = -(1 / ticks_per_second) * actual_ang_ticks * omega_rad
                else
                    move_angle = -((1 / ticks_per_second) * full_ticks_a * omega_rad + (1 / ticks_per_second) * (actual_ang_ticks - full_ticks_a) * rem_omega_rad)
                end
            end
            angle_ticks_needed = angle_ticks_needed + 1

        elseif c1 == 1 then
            if actual_dis_ticks <= full_ticks_d then
                move_distance = (1 / ticks_per_second) * speed_d * actual_dis_ticks * move_controller
            else
                move_distance = ((1 / ticks_per_second) * speed_d * full_ticks_d + (1 / ticks_per_second) * rem_speed_d * (actual_dis_ticks - full_ticks_d)) * move_controller
            end
            ticks_needed = ticks_needed + 1
        end

        robot.wheels.set_velocity(0,0)
    else
        if temp_2 == false then

            local imprimir = imprimir or false
            local name = name or "Unknown808"

            local speed_d = speed_d or d_speed
            local speed_g = speed_g or g_speed
            
            -- Valores por defecto para resoluciones si no se envían
            local min_speed_d = min_speed_d or 5
            local min_speed_g = min_speed_g or 1

            local L = FOOTBOT_INTERWHEEL_DISTANCE --   static const Real FOOTBOT_INTERWHEEL_DISTANCE      = 0.14f;
            local t = timer

            if target_angle > 180 then
                target_angle = target_angle - 360
            end
            if target_angle < -180 then
                target_angle = 360 + target_angle
            end
            if distance < 0 then
                move_controller = -1
            else
                move_controller = 1
            end

            -- Redondear el ángulo y la distancia basados en la resolución de hardware simulada
            target_angle = redondear_resolucion(target_angle, min_speed_g, ticks_per_second)
            distance = redondear_resolucion(distance, min_speed_d, ticks_per_second)

            -- ===============================
            -- Fase 1: Giro 
            -- ===============================
            if c1 == 0 then
                move_distance = 0

                if angle_ticks_needed == 0 then

                    local omega_deg = math.min(math.abs(speed_g), 80)   -- máximo 80°/s
                    omega_rad = math.rad(omega_deg)                     -- radianes por segundo
                    wheel_speed = omega_rad * (L / 2)                   -- velocidad en m/s

                    -- NUEVO: Cálculo de ticks completos y residuales para ángulo
                    local a_tick_full = omega_deg / ticks_per_second
                    full_ticks_a = math.floor(math.abs(target_angle) / a_tick_full)
                    local rem_angle = math.abs(target_angle) - (full_ticks_a * a_tick_full)
                    
                    local res_g = min_speed_g / ticks_per_second
                    has_rem_a = rem_angle > (res_g * 0.1) -- Evitar errores de coma flotante
                    local total_ticks_a = full_ticks_a + (has_rem_a and 1 or 0)
                    
                    local rem_omega_deg = rem_angle * ticks_per_second
                    rem_omega_rad = math.rad(rem_omega_deg)
                    rem_wheel_speed = rem_omega_rad * (L / 2)

                    angle_ticks_needed = total_ticks_a + t 

                    if c2 == 0 then    
                        if name ~= "Unknown808" then
                            Save_position(imprimir, name)
                        end        
                        if imprimir then 
                            log(" ====================================================== "..robot.id)
                            log("Distancia objetivo: "..distance.."cm , Distancia recorrida: "..move_distance.." cm")
                            log("Angulo objetivo: "..target_angle.."° , Angulo recorrido: ".. move_angle*180/math.pi .."°")
                            log(" ====================================================== ")
                        end
                    end
                end

                if t < angle_ticks_needed then
                    -- Inyección de velocidad residual considerando el delay de 1 tick
                    local current_wheel_speed = wheel_speed
                    if actual_ang_ticks >= full_ticks_a and has_rem_a then
                        current_wheel_speed = rem_wheel_speed
                    end

                    if target_angle > 0 then
                        robot.wheels.set_velocity(-current_wheel_speed, current_wheel_speed)
                        if actual_ang_ticks <= full_ticks_a then
                            move_angle = (1 / ticks_per_second) * actual_ang_ticks * omega_rad
                        else
                            move_angle = (1 / ticks_per_second) * full_ticks_a * omega_rad + (1 / ticks_per_second) * (actual_ang_ticks - full_ticks_a) * rem_omega_rad
                        end
                    else
                        robot.wheels.set_velocity(current_wheel_speed, -current_wheel_speed)
                        if actual_ang_ticks <= full_ticks_a then
                            move_angle = -(1 / ticks_per_second) * actual_ang_ticks * omega_rad
                        else
                            move_angle = -((1 / ticks_per_second) * full_ticks_a * omega_rad + (1 / ticks_per_second) * (actual_ang_ticks - full_ticks_a) * rem_omega_rad)
                        end
                    end

                    actual_ang_ticks = actual_ang_ticks + 1

                else  
                    robot.wheels.set_velocity(0, 0)
                    c1 = 1
                    
                    -- NUEVO: Cálculo de ticks completos y residuales para movimiento lineal
                    local d_tick_full = speed_d / ticks_per_second
                    full_ticks_d = math.floor(math.abs(distance) / d_tick_full)
                    local rem_dist = math.abs(distance) - (full_ticks_d * d_tick_full)
                    
                    local res_d = min_speed_d / ticks_per_second
                    has_rem_d = rem_dist > (res_d * 0.1)
                    local total_ticks_d = full_ticks_d + (has_rem_d and 1 or 0)
                    
                    rem_speed_d = rem_dist * ticks_per_second
                    ticks_needed = total_ticks_d + t 

                    -- Dejamos el ángulo cerrado en el valor final exacto
                    if target_angle > 0 then 
                        move_angle = (1 / ticks_per_second) * full_ticks_a * omega_rad + (has_rem_a and (1 / ticks_per_second) * rem_omega_rad or 0)
                    else
                        move_angle = -((1 / ticks_per_second) * full_ticks_a * omega_rad + (has_rem_a and (1 / ticks_per_second) * rem_omega_rad or 0))
                    end
                end
                -- if robot.id == "Qupa4" then
                --     log("Ángulo recorrido: " .. string.format("%.2f", move_angle * (180 / math.pi)) .. "°")
                -- end
            end

            -- ===============================
            -- Fase 2: Movimiento lineal
            -- ===============================
            if c1 == 1 then    

                -- Detectar cambio de velocidad y recalcular con el nuevo modelo residual
                if previous_speed_d == nil then
                    previous_speed_d = speed_d
                elseif previous_speed_d ~= speed_d then
                    
                    local res_d = min_speed_d / ticks_per_second
                    local abs_distance = math.abs(distance)
                    
                    local distancia_recorrida = math.abs(move_distance)
                    local distancia_restante = abs_distance - distancia_recorrida
                    
                    local d_tick_full = speed_d / ticks_per_second
                    local remaining_full_ticks = math.floor(distancia_restante / d_tick_full)
                    local rem_dist = distancia_restante - (remaining_full_ticks * d_tick_full)
                    
                    has_rem_d = rem_dist > (res_d * 0.1)
                    full_ticks_d = actual_dis_ticks + remaining_full_ticks
                    local total_ticks_d = full_ticks_d + (has_rem_d and 1 or 0)
                    
                    rem_speed_d = rem_dist * ticks_per_second
                    ticks_needed = total_ticks_d + t
                    
                    previous_speed_d = speed_d
                end


                if t < ticks_needed then
                    -- Inyección de velocidad residual considerando el delay de 1 tick
                    local current_speed_d = speed_d
                    if actual_dis_ticks >= full_ticks_d and has_rem_d then
                        current_speed_d = rem_speed_d
                    end

                    if actual_dis_ticks <= full_ticks_d then
                        move_distance = (1 / ticks_per_second) * speed_d * actual_dis_ticks * move_controller
                    else
                        move_distance = ((1 / ticks_per_second) * speed_d * full_ticks_d + (1 / ticks_per_second) * rem_speed_d * (actual_dis_ticks - full_ticks_d)) * move_controller
                    end

                    robot.wheels.set_velocity(current_speed_d * move_controller, current_speed_d * move_controller)
                    actual_dis_ticks = actual_dis_ticks + 1
                else
                    -- Asegurar la distancia final exacta al detener
                    if actual_dis_ticks <= full_ticks_d then
                        move_distance = (1 / ticks_per_second) * speed_d * actual_dis_ticks * move_controller
                    else
                        move_distance = ((1 / ticks_per_second) * speed_d * full_ticks_d + (1 / ticks_per_second) * rem_speed_d * (actual_dis_ticks - full_ticks_d)) * move_controller
                    end
                    
                    robot.wheels.set_velocity(0, 0)
                    c1 = 2
                    c2 = c2 + 1    

                    ticks_needed = 0
                    actual_dis_ticks = 0
                    angle_ticks_needed = 0
                    actual_ang_ticks = 0
                    if name ~= "Unknown808" then
                        Save_position(imprimir, name)
                    end

                    if imprimir then 
                        log("\n\n ------------------------------------------------------ "..robot.id)
                        log(" ------------------------------------------------------ ")
                        log("Distancia objetivo: "..distance.."cm , Distancia recorrida: "..move_distance.." cm")
                        log("Angulo objetivo: "..target_angle.."° , Angulo recorrido: ".. move_angle*180/math.pi .."°")
                        log(" ------------------------------------------------------ ")
                        log(" ------------------------------------------------------ \n\n ")
                    end
                end
                -- if robot.id == "Qupa4" then
                --     log("Distancia recorrida: " .. string.format("%.2f", move_distance) .. " cm")
                -- end

            end

            if c1 == 2 then    
                ticks_needed        = 0
                actual_dis_ticks    = 0
                angle_ticks_needed  = 0
                actual_ang_ticks    = 0

                r_Temporizador_2()
            end

            return c2,c1,move_distance,move_angle
        else
            robot.wheels.set_velocity(0, 0)
        end
    end
end

UMBRAL_INTENSIDAD_EVASION = 0.1             -- Sensibilidad para activar la evasión
DURACION_TICKS_ARCO = 5 * ticks_per_second  -- Duración en ticks del arco de evasión

local estado_evasion = {
    modo = "NORMAL",      -- "NORMAL", "GIRANDO", "ARCO"
    ultimo_modo = "",     -- Memoria para imprimir solo 1 vez cuando cambie de estado
    angulo_giro = 90,     
    tiempo_arco = 0,
    alternar_giro = 1     
}

function sign(x)
    if x > 0 then return 1
    elseif x < 0 then return -1
    else return 0 end
end

-- =========================================================================
-- LÓGICA DE NAVEGACIÓN Y EVASIÓN
-- =========================================================================
function navegar_a_coordenadas(target_x, target_y, stop_distance)
    -- --- 1. Calcular Distancias y Ángulo Hacia la Meta ---
    local current_x = robot.positioning.position.x
    local current_y = robot.positioning.position.y
    local dx = target_x - current_x
    local dy = target_y - current_y
    local distance = math.sqrt(dx^2 + dy^2)

    -- Condición de Parada
    if distance <= stop_distance then
        robot.wheels.set_velocity(0, 0)
        estado_evasion.modo = "NORMAL"
        return true 
    end

    -- Calcular orientación exacta respecto a la meta
    local angle_to_center = math.atan2(dy, dx)
    local orientation_quat = robot.positioning.orientation
    local orientation_angle = math.atan2(
        2 * (orientation_quat.w * orientation_quat.z + orientation_quat.x * orientation_quat.y),
        1 - 2 * (orientation_quat.y ^ 2 + orientation_quat.z ^ 2)
    )
    local angle_difference = angle_to_center - orientation_angle
    angle_difference = math.atan2(math.sin(angle_difference), math.cos(angle_difference))

    -- --- 2. Evaluación de Proximidad (USANDO TU FUNCIÓN) ---
    local datos_prox = proximidad()
    local prox_val = datos_prox[1]    -- Intensidad promedio del obstáculo
    local prox_angle = datos_prox[2]  -- Ángulo donde se concentra el obstáculo

    -- Detectamos si el choque es FRONTAL (rango de -45° a 45°)
    local obstaculo_frontal = false
    if prox_val > UMBRAL_INTENSIDAD_EVASION and math.abs(prox_angle) <= 90 then
        obstaculo_frontal = true
    end

    -- --- 3. LOGS DE DEPURACIÓN ---
    if DEBUG_EVASION > 0 then
        if estado_evasion.modo ~= estado_evasion.ultimo_modo then
            if estado_evasion.modo == "NORMAL" then
                log(string.format("[%s] AVANZANDO A COORDENADA: %.2f, %.2f", robot.id, target_x, target_y))
            elseif estado_evasion.modo == "GIRANDO" then
                log(string.format("[%s] GIRANDO %d° para evadir", robot.id, estado_evasion.angulo_giro))
            elseif estado_evasion.modo == "ARCO" then
                log(string.format("[%s] ESQUIVANDO (Haciendo Arco)", robot.id))
            end
            estado_evasion.ultimo_modo = estado_evasion.modo
        end

        if DEBUG_EVASION >= 2 then
            log(string.format("[%s] MODO: %s | ProxPromedio: %.3f a %.1f° | AngMeta: %.1f°", 
                robot.id, estado_evasion.modo, prox_val, prox_angle, math.deg(angle_difference)))
        end
    end

    -- --- 4. Máquina de Estados de Evasión ---
    if estado_evasion.modo == "NORMAL" then
        if obstaculo_frontal then
            -- Decidir dirección de giro basado en el ángulo del obstáculo
            if prox_angle > 5 then
                estado_evasion.angulo_giro = -90
            elseif prox_angle < -5 then
                estado_evasion.angulo_giro = 90
            else
                estado_evasion.angulo_giro = 90 * estado_evasion.alternar_giro
                estado_evasion.alternar_giro = estado_evasion.alternar_giro * -1
            end
            
            r_move()
            estado_evasion.modo = "GIRANDO"
        end

    elseif estado_evasion.modo == "GIRANDO" then
        local rc2, rc1 = move(0, estado_evasion.angulo_giro, 0, 50, false, "Evasion", false)
        
        if rc1 == 2 then
            estado_evasion.modo = "ARCO"
            estado_evasion.tiempo_arco = DURACION_TICKS_ARCO
        end
        return false 

    elseif estado_evasion.modo == "ARCO" then
        if obstaculo_frontal then
            if prox_angle > 5 then
                estado_evasion.angulo_giro = -90
            elseif prox_angle < -5 then
                estado_evasion.angulo_giro = 90
            else
                estado_evasion.angulo_giro = 90 * estado_evasion.alternar_giro
                estado_evasion.alternar_giro = estado_evasion.alternar_giro * -1
            end
            r_move()
            estado_evasion.modo = "GIRANDO"
            return false

        elseif math.abs(angle_difference) < 0.35 and not obstaculo_frontal then
            estado_evasion.modo = "NORMAL"

        else
            estado_evasion.tiempo_arco = estado_evasion.tiempo_arco - 1
            if estado_evasion.tiempo_arco <= 0 then
                estado_evasion.modo = "NORMAL" 
            end
        end
    end

    -- --- 5. Ejecución del Movimiento Físico ---
    if estado_evasion.modo == "ARCO" then
        local velocidad_base = 40
        local diferencial = 20
        
        if estado_evasion.angulo_giro > 0 then
            robot.wheels.set_velocity(velocidad_base + diferencial, velocidad_base - diferencial)
        else
            robot.wheels.set_velocity(velocidad_base - diferencial, velocidad_base + diferencial)
        end
        return false 
        
    elseif estado_evasion.modo == "NORMAL" then
        local speed_straight = 50
        local speed_turn_factor = 10 * ticks_per_second -- Control proporcional dinámico
        local max_wheel_speed = 100   

        if math.abs(angle_difference) < 0.1 then
            robot.wheels.set_velocity(speed_straight, speed_straight)
        else
            local turn_speed = speed_turn_factor * angle_difference
            local vL = -turn_speed
            local vR = turn_speed
            
            if math.abs(vL) > max_wheel_speed then vL = max_wheel_speed * sign(vL) end
            if math.abs(vR) > max_wheel_speed then vR = max_wheel_speed * sign(vR) end

            robot.wheels.set_velocity(vL, vR)
        end
    end
    
    return false 
end
-------------------------------------------------------------------------------------------------------------------------------------
-------------------------------------------------------------------------------------------------------------------------------------
-- 
-------------------------------------------------------------------------------------------------------------------------------------
-------------------------------------------------------------------------------------------------------------------------------------

-- Función para determiaar si un Qupa será apprentice o master
is_master = 0
function determinar_rol()
    -- Selección aleatoria del master

    randoms = robot.random.uniform_int(0, 2)
    if randoms == 1 then
        is_master = 1
    else
        is_master = 0
    end

    if robot.id ~= "Qupa4" then
        is_master = 1
    else
        is_master = 0
    end

    -- Encender LED rojo si es master
    if is_master == 1 then
        --setleds("red")
        log("The " .. robot.id .. " is master = " .. randoms)
        --log("Led rojo encendido por " ..robot.id)
    else
        --robot.leds.set_all_colors("blue")
        log("The " .. robot.id .. " is Observador= " .. randoms)
    end


    return is_master
end

function resetOBS()
    -- ============================  
    srt_c1 = 0
    count  = 0
    srt_ang_mark_ant = 0
    x_mark, y_mark, x_new_r, x_ant_r, y_new_r, y_ant_r = 0, 0, 0, 0, 0 ,0 

    srt_angle_an = 0

    motion_markers = {}           -- Correcciones registradas
    last_motion_marker = {0, 0}   -- Total correccion realizada
    motion_tick = 0               -- Tick actual durante movimiento
    motion_active = false         -- Indica si hay un movimiento en curso
    motion_target_ticks = 0       -- Total de ticks necesarios
    motion_cm_tick = 0   
    motion_dist = 0   

    prev_x_original, prev_y_original = 0, 0       -- guardan la última posición observada
    stationary_count = 0                          -- contador que queremos incrementar (solo 1 vez por grupo)
    stationary_active = false                     -- bandera que indica que ya contamos este grupo igual

    segmentos = {}           -- segmentos[2], segmentos[3], ... contendrán los segmentos
    movement_buffer = {}     -- buffer temporal con puntos mientras el master está en movimiento

    srt_end             = 0

    -- ============================
    DS_c1                 = 0

    equations             = {}

    distancias_a_recorrer = {} --Tabla para almacenar las distancias a recorrer
    intersections         = {}
    angles_a_recorrer     = {}
    -- params                = {}

    -- ============================
    posiciones_guardadas = {}
    
    indice          = 1
    PI_end          = false
    PI_counter_seg  = 0

    -- ============================
    c4              = 0
    -- ============================
    setleds("black")
    r_Temporizador_1()
    r_Temporizador_2()
    r_move()
end

function soft_reset_observer()
    resetOBS() -- Llamamos a tu reset normal de odometría/estados
    
    current_patience = MAX_PATIENCE
    master_tracking = {}
    srt_anterior_data = {}
    
    change_al_state("IDLE", "Fin del Random Walk, volviendo a observar")
end

function resetDEM() 

    -- ============================
    d1 = 0    

    -- ============================
    EAEP_c1 = 0

    -- ============================
    posiciones_guardadas = {}

    -- ============================
    indice          = 1
    PI_end          = false
    PI_counter_seg  = 0

    -- ============================
    --setleds("red")
   
    r_Temporizador_1()
    r_Temporizador_2()
    r_move()
end

-- Función auxiliar para limpiar las variables globales de 'move'  y asegurar que arranque un movimiento en limpio
function r_move() 

    c1                  = 0
    c2                  = 0
    move_controller     = 1
    move_distance       = 0 

    ticks_needed        = 0
    actual_dis_ticks    = 0

    angle_ticks_needed  = 0
    actual_ang_ticks    = 0

    time_needed         = 0
    speed               = d_speed

    move_distance       = 0; --restart distance
    move_angle          = 0; --restart angle

    wheel_speed         = 0
    omega_rad           = 0

    previous_speed_d    = nil
    stop = false

    -- Variables residuales de Store_relative_position
    full_ticks_a        = 0
    has_rem_a           = false
    rem_wheel_speed     = 0
    rem_omega_rad       = 0

    rem_speed_d         = 0
    full_ticks_d        = 0
    has_rem_d           = false
    
end

--Funcion para hacer una espera
temp = true
cr = 1
crono = 0
function Temporizador_1(c)
    c = c * ticks_per_second
    crono = c - cr
    cr = cr + 1

    if crono <= 0 then
        temp = false
        cr = 1          
    end
end
--resetear espera
function r_Temporizador_1()
    temp = true
    cr = 1
    crono = 0
end

--Funcion para hacer una espera
temp_2 = true
cr_2 = 1
crono_2  = 0
function Temporizador_2(d)
    d = d * ticks_per_second
    crono_2  = d - cr_2 
    cr_2  = cr_2  + 1

    if crono_2  <= 0 then
        temp_2  = false
        cr_2  = 1          
    end
end
--resetear espera
function r_Temporizador_2()
    temp_2  = true
    cr_2  = 1
    crono_2  = 0
end

-- Función para setear determinados leds con color personalizado
function setleds(color)
    for i = 1, Num_cantidad_leds do
        if i <= Num_cantidad_leds-1 and i >= 1 then
            robot.leds.set_single_color(i, "black") -- Apagar LED
        else
            robot.leds.set_single_color(i, color) -- Encender LED con el color recibido como parámetro
        end
    end
end

--Funcion para imprimir el contenido de una tabla
function print_table(tbl)
    for key, value in pairs(tbl) do
        if type(value) == "table" then
            log(key .. ":")
            print_table(value) -- Si el valor es una tabla, recursivamente imprimimos sus claves
        else
            log(key .. " = " .. tostring(value))
        end
    end
end

function imprimir_posicion(tabla)    

    -- log(" - Lista de puntos de la trayectoria: ".." " .. robot.id )
    -- for i, punto in ipairs(intersecciones) do
    --     log(string.format("  Intersección %d: (%.2f, %.2f)", i, punto[1], punto[2]))
    -- end
    -- if is_master == 0 then
    --     log(string.format("Total segmentos construidos: %d ", #params)..", "..robot.id)
    -- else
    -- end

    --log(string.format("Total segmentos construidos: %d ", #params)..", "..robot.id)
    for idx, seg in ipairs(full_pattern_params) do
        log(string.format(
            "Segmento %d -> Distancia: %.2f cm, ÁnguloRel: %.2f°",
            idx, seg[1], seg[2]
        ))
    end
    for nombre, posiciones in pairs(tabla) do
        log(nombre .. " -> ")
        for i, pos in ipairs(posiciones) do
            log("  Posición " .. i .. ": x = " .. pos[1] .. ", y = " .. pos[2])
        end
    end
end

posiciones_guardadas = {}
function Save_position(imprimir, name)
    imprimir = imprimir or false
    name = name or "Unknown"

    -- Crear la subtabla si no existe aún
    if posiciones_guardadas[name] == nil then
        posiciones_guardadas[name] = {}
    end

    local posicion = {}
    posicion[1] = robot.positioning.position.x
    posicion[2] = robot.positioning.position.y

    -- Agregar al historial de posiciones
    table.insert(posiciones_guardadas[name], posicion)

    if imprimir then
        log("Guardando: [" .. name .. "] Posición en x: " .. posicion[1] .. ", Posición en y: " .. posicion[2])
    end
end

function original_params()
    local a = {}
    if figura == "triangulo_regular" then
        a = {
            params = {
            { distance_factor*20 ,60  },
            { distance_factor*20 ,120 },
            { distance_factor*20 ,120 },

            { 0, 60}
            },
            name = "triangulo"
        }
    end


    if figura == "octagono"then
        a = {
            params = {
            { distance_factor*20 ,-45 },
            { distance_factor*20 ,-45 },
            { distance_factor*20 ,-45 },
            { distance_factor*20 ,-45 },
            { distance_factor*20 ,-45 },
            { distance_factor*20 ,-45 },
            { distance_factor*20 ,-45 },
            { distance_factor*20 ,-45 },
            },
            name = "octagono"
        }
    end

    if figura == "triangulo_irregular_1" then
        a = {
            params = {
            { 40, 70 },
            { 30, 90 },
            { 50, 127},

            { 0 , 73 }
            },
            name = "Triangulo irregular 1"
        }
    end

    if figura == "Pentágono_irregular" then
        a = {
            params = {
            { 0, -30},
            { 0,  30},

            { distance_factor*33.33 , 0 },
            { distance_factor*33.33 , 90},
            { distance_factor*33.33 , 53},
            { distance_factor*33.33 , 74},
            { distance_factor*38.867, 84},

            { 0, 59},
            },
            name = "Pentágono_irregular"
        }
    end

    if figura == "Heptágono" then
        a = {
            params = {
            { 0, -45},
            { 0,  45},

            { 50,  0},
            { 40, 90},
            { 28, 45},
            { 30, 45},
            { 28, 45},
            { 20, 45},
            { 28, 45},

            { 0 , 45}
            },
            name = "Heptágono"
        }
    end

    if figura == "trayectoria_1" then
        a ={
            params = {
            { 40 , 95 },
            { 50 ,-45 },
            { 30 ,-90 },
            { 20 ,110 },
            { 20 ,-30 },
            { 0 ,-180 },
            },
            name = "Trayectoria 1"
        }
    end
    return a
end

--------------------------------------------------------------------------------------------------------------------------------
-------------------------------------------------------------------------------------------------------------------------------------
-------------------------------------------------------------------------------------------------------------------------------------
--                                                        Functions Finales                                                        --
-------------------------------------------------------------------------------------------------------------------------------------
-------------------------------------------------------------------------------------------------------------------------------------
-- ----------------------------------------------------------------------------------------------------------------------------------

-- Función para limpiar el estado al finalizar
function reset()
end
-- --------------------------------------------------------------------------------------------------
-- Función que se ejecuta al finalizar la simulación
function destroy()
end