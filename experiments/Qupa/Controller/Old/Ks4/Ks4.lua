--=============================================
--   MEJORAS :  -- Active learning con centroide + pid
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

ticks_per_second                    = 10                        -- Número de ticks que representa un segundo en la simulación (ajusta según la configuración de .argos)
max_time                            = 5000 * ticks_per_second    -- Duración máxima de la simulación en ticks
Distribucion_sensores_de_proximidad = {0,-45,45,-90, 90,180}    -- Ángulos de los sensores de proximidad a considerar para la detección de obstáculos (en grados) --> {} o "uniformes"
--Distribucion_sensores_de_proximidad = "uniform"               -- Ángulos de los sensores de proximidad a considerar para la detección de obstáculos (en grados) --> {} o "uniformes"
Num_cantidad_leds                   = 2
FOOTBOT_INTERWHEEL_DISTANCE         = 14                        -- Distancia entre ruedas del Qupa (en metros)
-- Importamos 
math_helper = require("Functions.math_helper")
------------------------------
--Parámetro de simulación internos
ENABLE_ACTIVE_LEARNING = true --Toggle global para activar/desactivar el seguimiento dinámico

speed_dem = 27                  -- Velocidad del master al ejecutar el patrón
d_speed   = 30                  -- velocidad normal
g_speed   = 50                  -- Velocidad de giro

-- Métricas de paciencia
MAX_PATIENCE        = 800       -- Tiempo máximo de espera sin recibir datos útiles
BASE_RESTORE        = 200       -- Paciencia base a restaurar con datos refinados/actualizados

-- 
back_threshold        = 25      -- Distancia que retrocede el apprentice para evitar colisión. cm.
front_threshold       = 25      -- Limite de distancia minima entre un objeto detectado y el apprentice para entrar a evadirlo
max_tracking_distance = 25      -- Máxima distancia a la que se puede alejar el centroide antes de que el apprentice se acerque para no perder de vista. cm
porcentaje            = 0.75     -- 1 < porcentaje < 0.33 , indica que tanto el apprentice busca acercarse al maestro cuando este cruza el max_tracking_distance.

--
distance_treshold   = 3*(speed_dem+d_speed)/ticks_per_second    -- Sensibilidad del apprentice a caambios en los puntos obtenidos del maestro durante la etapa de observación.

--
seg_cfg = {
    distance_treshold       = distance_treshold, -- Función de pint cloud segmentation
}


-- 
-- Esta tabla centraliza los hiperparámetros de tolerancia geométrica, pesos 
-- de evaluación, filtrado de ruido y protecciones de rendimiento en tiempo real 
-- para el sistema de reconstrucción.
-- ============================================================================

CONFIG = {
    -- ------------------------------------------------------------------------
    -- 1. TOLERANCIAS GEOMÉTRICAS Y SOLAPES
    -- ------------------------------------------------------------------------
    
    -- Margen de error permitido en distancia (en centímetros).
    -- Implicación: Define cuán estrictos somos para considerar que dos distancias 
    -- detectadas corresponden al mismo objeto o segmento. Si es muy bajo, el ruido 
    -- real romperá los datos verdaderos; si es muy alto, unirá cosas dispares.
    TOL_DIST = 2 * (speed_dem + d_speed) / ticks_per_second,
    
    -- Margen de error permitido en ángulos (en grados).
    -- Implicación: Tolerancia angular aceptada para determinar si dos rectas, 
    -- tramos o giros pertenecen al mismo patrón o son colineales.
    TOL_ANG = 15,
    
    -- Solape mínimo independiente para el método rápido (Greedy / M1).
    -- Implicación: Número mínimo de puntos consecutivos que deben coincidir 
    -- obligatoriamente para que el algoritmo voraz una dos fragmentos.
    MIN_OVERLAP_M1 = 1,
    
    -- Solape mínimo base para el método exhaustivo (Backtracking / M2).
    -- Implicación: Cuántos segmentoos mínimos deben coincidir en condiciones normales 
    -- para que M2 cree una rama de combinación. (Nota: El sistema ajusta este 
    -- valor de forma dinámica si detecta un entorno caótico con mucho ruido).
    MIN_OVERLAP_M2 = 1,

    -- ------------------------------------------------------------------------
    -- 2. SEGURIDAD Y RENDIMIENTO (PROTECCIÓN CONTRA CONGELAMIENTOS)
    -- ------------------------------------------------------------------------
    
    -- CORTACIRCUITOS: Límite máximo de llamadas recursivas permitidas en M2.
    -- Concepto clave: Una "llamada recursiva" ocurre cuando el algoritmo se llama 
    -- a sí mismo para explorar múltiples caminos y combinaciones de piezas (como 
    -- un laberinto). Cuando se introduce ruido, miles de piezas parecen encajar 
    -- parcialmente, creando una "explosión combinatoria" de millones de opciones 
    -- que saturan la CPU y congelan el PC.
    -- Implicación: Si la búsqueda exhaustiva supera este límite de 5000 llamadas, 
    -- el seguro se activa instantáneamente, aborta M2 y transfiere el control 
    -- al método rápido M1 para garantizar la fluidez en tiempo real de la simulación.
    MAX_RECURSIONS_M2 = 5000,
    
    -- Interruptor booleano (true/false) para imprimir o silenciar en consola 
    -- los pasos detallados que va tomando el método M2 durante la depuración.
    PRINT_RECURSIVE_PATHS = true,
    
    -- Interruptor para forzar la ejecución de M2 incluso cuando el método rápido 
    -- (M1) ya había encontrado una solución perfecta.
    FORCE_M2_ALWAYS = false,
    
    -- Límite máximo de "islas" (fragmentos independientes) que M2 está autorizado 
    -- a procesar mediante búsqueda exhaustiva.
    -- Implicación: Si el número de fragmentos supera este valor, el sistema omite 
    -- M2 automáticamente y recurre a M1 para evitar saturar el procesador.
    MAX_FRAGMENTS_M2 = 9,

    -- ------------------------------------------------------------------------
    -- 3. PONDERACIÓN Y SELECCIÓN DE PATRONES
    -- ------------------------------------------------------------------------
    
    -- Criterio matemático para la selección de rutas finales:
    -- 1 = Prioriza frecuencia, 2 = Prioriza trayectorias largas, 
    -- 3 = Función de Ponderación Global basada en Score.
    selection_mode = 3,
    
    -- Peso de la frecuencia.
    -- Implicación: Qué tanto premia el árbitro a los patrones que se repiten 
    -- consistentemente en el comportamiento del enjambre.
    W_FREQ = 1,
    
    -- Peso de la longitud de la trayectoria.
    -- Implicación: Valores positivos favorecen rutas cortas; negativos favorecen 
    -- rutas largas; 0 ignora por completo la longitud en la decisión.
    W_LEN = -1,
    
    -- Peso de la calidad de los solapes.
    -- Implicación: Premia aquellas soluciones donde los fragmentos encajaban 
    -- de forma limpia, robusta y con alta superposición.
    W_OV_QUAL = 2,

    -- Tolerancia matemática interna utilizada en las comprobaciones geométricas 
    -- del Geómetra (M3).
    MATH_TOLERANCE = 0.5,

    -- ------------------------------------------------------------------------
    -- 4. FILTRADO DE RUIDO (TOLERANCIA A LA BASURA DEL SENSOR)
    -- ------------------------------------------------------------------------
    
    -- Piso mínimo absoluto de filtrado (0.2 equivalentes al 20%).
    -- Implicación: Porcentaje mínimo de repetición obligatorio para que un dato 
    -- no sea descartado como ruido o basura del sensor, incluso en simulaciones ideales.
    RUIDO_BASE = 0.2,
    
    -- Margen de ajuste adicional para el ruido (0.15 equivalentes al +15%).
    -- Implicación: Define cuán agresivamente se endurece el filtro de ruido 
    -- cuando el termostato interno detecta un entorno de alto caos o dispersión.
    RUIDO_PENALIZACION_MAX = 0.15,

    -- ------------------------------------------------------------------------
    -- 5. EL GEÓMETRA M3 Y EL ÁRBITRO (LÓGICA DE DECISIÓN)
    -- ------------------------------------------------------------------------
    
    -- Multiplicador base neutral para las geometrías regulares (polígonos cerrados).
    BONO_M3_BASE = 1.0,
    
    -- Premio máximo adicional en la puntuación si el termostato detecta que el enjambre 
    -- está formando figuras simétricas perfectas (como polígonos inspirados en abejas).
    BONO_M3_MAX_EXTRA = 0.5,
    
    -- Cobertura mínima absoluta requerida para que el árbitro acepte un patrón 
    -- (0.25 equivalentes al 25% del volumen total de datos del enjambre).
    SCORE_MIN_BASE = 0.25,
    
    -- Blindaje estadístico dinámico. Eleva la exigencia de puntuación en escenarios 
    -- muy fragmentados para evitar que el sistema "alucine" patrones por mera casualidad.
    SCORE_PENALIZACION_MAX = 0.10,
    
    -- Constante de penalización estructural aplicada a trayectorias demasiado largas.
    -- Implicación: Aplica un principio de parsimonia (Navaja de Ockham), evitando 
    -- que una ruta irregular gigantesca (M2) supere injustamente a una geometría 
    -- corta y elegante (M3) solo por abarcar mayor volumen bruto de datos.
    PESO_COMPLEJIDAD = 0.25
}

-- CONFIGURACIÓN DEL MODELO DE RUIDO (SENSOR NOISE CONFIG)
noise_cfg = {
    ENABLE_NOISE            = true ,    -- Si es false, el robot ve perfecto (Ground Truth)
    USE_DISTANCE_DEPENDENT  = true  ,    -- El ruido aumenta si el objeto está lejos
    USE_OUTLIERS            = false ,    -- Activar lecturas "fantasma" o fallos graves
    USE_BIAS                = true  ,    -- Error sistemático (ej. sensor mal calibrado +2cm siempre)
    -- 
    NOISE_BASE_RADIUS       = 5.000,    -- Ruido constante base (ej. 1 cm de error mínimo)
    NOISE_SIGMAS            = 2.0   ,    -- 2 sigmas = 95% de probabilidad de caer dentro del 'Radio Actual'
    NOISE_FACTOR            = 0.05  ,    -- Crece un 5% respecto a la distancia (ej. a 100cm, suma 5cm extra al radio)
    OUTLIER_PROB            = 0.01  ,    -- % de probabilidad de fallo catastrófico
    BIAS_OFFSET             = 1.5   ,    -- Cm extra que agrega el sensor por defecto (si USE_BIAS es true)
}

--Parametros de patrónes predefinidos
bigger = 1  -- Factor de escala para ajustar el tamaño de las figuras predefinidas (ajusta según tu espacio de simulación)

if robot.id == "Qupa4" then

elseif robot.id == "Qupa1" or robot.id == "Qupa2" or robot.id == "Qupa3" or robot.id == "Qupa5" then
    figura = "triangulo_regular" -- "triangulo_regular" -- "octagono" -- "triangulo_irregular_1" --"Pentágono_irregular"  -- "Heptágono"-- "" -- "trayectoria_1" 
    bigger = 1
else
    figura = "triangulo_regular" -- "triangulo_regular" -- "octagono" -- "triangulo_irregular_1" --"Pentágono_irregular"  -- "Heptágono"-- "" -- "trayectoria_1" 
    bigger = 1  -- Factor de escala para ajustar el tamaño de las figuras predefinidas (ajusta según tu espacio de simulación)
end

-- Banderas de loggeo

-- Loggeo de la información obtenida por el apprentice durante la etapa de Observación
SRP_LOG_MODE        = 0 -- 0: Sin logs, 1: Parámetros obtenidos tras finalizar la observación de un maestro. 2. Paso a paso de la actualización de maestros, 3: Imprime todo.
UP_LOG_MODE         = 1 -- 0: Sin logs, 1: Información sobre la paciencia (Tiempo que el apprentice actualiza para continuar o deter su aprendizaje) 
AL_LOG_MODE         = 0 -- 0: Sin logs, 1: Información sobre el Active Learning, 2: Información detallada del Active Learning, 3: Estados del active learning
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

-- function Print_tracker_AL_PID(target, actual, output)
--     print(string.format("[PID];%.4f;%.4f;%.4f", target, actual, output))
-- end

-- function Print_tracker_calidad_imitacion_data(linea_resumen, is_master)

--     if is_master == 0 then

--         print(linea_resumen)
--     else

--         -- Función auxiliar para separar la parte entera y los dos decimales de un número
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
--             local dist = params[j][1]
--             local ang = params[j][2]
            
--             -- Separamos enteros y decimales de la distancia y el ángulo
--             local dist_i, dist_d = split_num(dist)
--             local ang_i, ang_d = split_num(ang)
            
--             -- Concatenamos el paso con el formato (distI,distD,angI,angD)
--             resultado = resultado .. string.format("(%s,%s,%s,%s)", dist_i, dist_d, ang_i, ang_d)
            
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
--     log("timer = "..timer)
--     log("Blob detectado: angulo = " .. raw_angle .. ", distancia = " .. raw_dist)
--     log("Blob ruidoso detectado: angulo = " .. noisy_angle .. ", distancia = " .. noisy_dist)
-- end

-------------------------------------------------------------------------------------------------------------------------------------
-- Función que se ejecuta al inicio de la simulación
-------------------------------------------------------------------------------------------------------------------------------------

function init()
    
    robot.colored_blob_omnidirectional_camera.enable()
    determinar_rol()
    if Print_tracker_calidad_imitacion_data then Print_tracker_calidad_imitacion_data(robot.id, is_master) end
    if Print_tracker_paciencia and robot.id == "Qupa4" then 
        print("[PACIENCIA]("..(MAX_PATIENCE/ticks_per_second)..")&" .. ticks_per_second)

    end
end

-------------------------------------------------------------------------------------------------------------------------------------
-- Función principal que se ejecuta en cada tick
-------------------------------------------------------------------------------------------------------------------------------------

timer = 1
function step()

    if robot.id == "Qupa4" and 
        (Print_tracker_trayectory_data or Print_tracker_paciencia) then 
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


    --d1 = 2 -- ESTAMOS PROVISIONALMENTE FORZANDO AL DEMOSTRADOR A EJECUTAR EL PATRON DIRECTAMENTE PARA PODER VER EL FUNCIONAMIENTO DEL OBSERVADOR

    if d1 == 0 then
        detectar_zona_demostracion()
        --log("detectar_zona_demostracion")

    elseif d1 == 1 then   
        espera_antes_de_ejecutar_patron()
        --log("espera_antes_de_ejecutar_patron")

    elseif d1 == 2 then
        -- =========================================================
        -- Selección aleatoria del patrón a demostrar
        -- =========================================================

        -- Verificamos que existan patrones guardados en la tabla
        if #all_params > 0 then

            if master_indice == #all_params then
                d1 = 4
                return
            end
            -- Escoge un índice aleatorio entre 1 y la cantidad de patrones guardados
            --local indice_aleatorio = robot.random.uniform_int(1, #all_params)
            --local indice_aleatorio = math.random(1, #all_params)
            master_indice = master_indice + 1
            parametros_elegidos = all_params[master_indice]
            --log(" ========= Ejecutando patrón aleatorio (1,"..#all_params..") , -> "..indice_aleatorio..", "..robot.id.." ========= ")
            --print_table(parametros_elegidos)
            
        else
            if robot.id == "Qupa4" then
                log("No se detectaron patrones en la etapa de observación, "..robot.id)
                d1 = 4
                return
            end
            -- Respaldo de seguridad por si la tabla aún está vacía
            parametros_elegidos = original_params()
            parametros_elegidos = parametros_elegidos.params
            --log(" ========= Ejecutando patrón original: " .. robot.id.." ========= ")
            --print_table(parametros_elegidos)

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

c4 = 0
first_time_apprentice = true
function apprentice()

    if c4 == 0 then 
        store_relative_positions()
    elseif c4 == 1 then
        local NAC = navegar_a_coordenadas(0,0,0.10)
        if NAC then
            c4 = 2
        end 
    elseif c4 == 2 then
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
        if prox[1] > 0.8 then 
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
                move(valor1, valor2, speed_dem, 10, false, robot.id, master_paused)

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

-- ============================================================================
-- [SECCIÓN 2] VARIABLES GLOBALES DE ESTADO Y MEMORIA
-- ============================================================================
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

SP_CRITICAL_THRESHOLD = 0.80 
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
function update_patience(restore_type, count)
    if restore_type == "full" then
        current_patience = MAX_PATIENCE
        if UP_LOG_MODE == 1 then 
            log("[PACIENCIA] Restaurada completamente (" .. MAX_PATIENCE/ticks_per_second .. ") segundos.") 
        end
    elseif restore_type == "partial" then
        local divisor = (count > 1) and (count - 1) or 1
        local restore_amount = math.floor(BASE_RESTORE / divisor)
        current_patience = math.min(MAX_PATIENCE, current_patience + restore_amount)
        if UP_LOG_MODE == 1 then 
            log("[PACIENCIA] Restauración parcial: +" .. restore_amount/ticks_per_second .. " segundos. Actual: " .. current_patience/ticks_per_second.." segundos") 
        end
    end
end

function check_patience_and_reconstruct()
    if ParameterStorage.is_done() then return end

    current_patience = current_patience - 1
    if Print_tracker_paciencia then Print_tracker_paciencia(current_patience/ticks_per_second) end

    if current_patience <= 0 then

        if UP_LOG_MODE == 1 then 
            log("\n\n\n\n [PACIENCIA] AGOTADA: Ejecutando Reconstrucción...") 
        end
        local datos_guardados = ParameterStorage.get_stored_data()
        local current_cfg = { CONFIG = CONFIG, LOG_MODE = SRP_LOG_MODE }   
        local exito, linea_resumen, nuevos_patrones = GlobalTrajectoryReconstructor.run(robot.id, datos_guardados, current_cfg)
        
        if exito then
            ParameterStorage.set_done()
            if Print_tracker_calidad_imitacion_data then Print_tracker_calidad_imitacion_data(linea_resumen, is_master) end
            for _, patron in ipairs(nuevos_patrones) do
                table.insert(all_params, patron)
            end

            c4 = 1 
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
        log(" ######################################################## ")
        log(" PROCESANDO OBSERVACIÓN DEL MAESTRO "..k.." (Datos totales: "..#track.raw_x..")")
        log(" ######################################################## ")
    end



    local clean_mov, clean_type, clean_valid, p_start, p_end = PointCloudSegmenter.process(k, track.raw_x, track.raw_y, track.raw_type, seg_cfg)

    track.segmentos.movement = clean_mov
    track.segmentos.type     = clean_type
    track.segmentos.valid    = clean_valid
    
    if p_start then track.Pattern_start_detected  = true end
    if p_end   then track.End_of_pattern_detected = true end

    if #track.segmentos.movement > 1 then 
        -- ¡AQUÍ PASAMOS track.segmentos.valid!
        track.params = SegmentParameterizer.process_segments(track.segmentos.movement, track.segmentos.valid)
        
        local current_cfg = { CONFIG = CONFIG, LOG_MODE = SRP_LOG_MODE }
        local r_type, r_count = ParameterStorage.package_and_add(track.params, track.segmentos.valid, current_cfg)
        
        if r_type then update_patience(r_type, r_count) end
    else
        if SRP_LOG_MODE == 1 or  SRP_LOG_MODE == 3 then log("No hay segmentos suficientes para procesar tras la limpieza.") end
    end
end

-- Función para normalizar ángulos entre -PI y PI y evitar que 
-- srt_ang_mark_ant crezca infinitamente tras muchas correcciones
function normalize_angle_rad(ang)
    while ang > math.pi do ang = ang - 2 * math.pi end
    while ang <= -math.pi do ang = ang + 2 * math.pi end
    return ang
end
-- VARIABLES GLOBALES PARA CONTROL PID DEL ACTIVE LEARNING
Kp = 0.8                   -- Ganancia Proporcional
Ki = 0.01                  -- Ganancia Integral
Kd = 0.1                   -- Ganancia Derivativa

target_centroid_distance = 0.0  -- Referencia: Distancia deseada al centroide (0 = ir al centro exacto)
pid_tolerance = 2.0             -- Zona muerta para evitar oscilación minúscula
max_pid_output = 100.0           -- Límite máximo del vector de movimiento
min_pid_output = 10.0            -- Límite mínimo de movimiento

pid_error_integral = 0          -- Memoria del término integral
pid_last_error = 0              -- Memoria del error anterior para la derivada

-- ============================================================================
-- [SECCIÓN 6] BUCLE PRINCIPAL DEL OBSERVADOR (TICK A TICK)
-- ============================================================================
srt_c1 = 0
function store_relative_positions()

    if Print_tracker_trayectory_data then Print_tracker_trayectory_data("trajectory", "Qupa4", 0, 0) end
    check_patience_and_reconstruct()

    local proximity = proximidad()
    if proximity[1] < 1 then srt_c1 = 0 else
        if srt_c1 == 5 * ticks_per_second then resetOBS() return end
        srt_c1 = srt_c1 + 1
    end

    Temporizador_1(20) 
    if temp == false then robot.leds.set_all_colors("black") end
    
    -- =========================================================================
    -- FASE 0: EJECUCIÓN DE ODOMETRÍA, OBSTÁCULOS Y SUPERVIVENCIA (SP)
    -- =========================================================================
    if motion_active then
        local prox_data = proximidad() 
        obs_stop_flag = false

        local is_object_critical = (prox_data[1] > 0.8)
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

        move(last_motion_marker[1], last_motion_marker[2], d_speed, 80, false, robot.id, obs_stop_flag)
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
        -- FASE E: CÁLCULO DEL CENTROIDE (ACTIVE LEARNING - PID - MODO 3)
        -- =========================================================================
        local active_count = 0
        local sum_x, sum_y = 0, 0
        local closest_master_dist = 999999 
        local angle_of_closest = 0

        for k, data in pairs(srt_anterior_data) do
            if updated_anterior_indices[k] and data.angle and data.distance then
                local mx, my = math_helper.polarACartesianoPAR(data.distance, data.angle * (math.pi/180))
                sum_x = sum_x + mx
                sum_y = sum_y + my
                active_count = active_count + 1

                if data.distance < closest_master_dist then
                    closest_master_dist = data.distance
                    angle_of_closest = data.angle
                end
            end
        end

        if active_count > 0 and ENABLE_ACTIVE_LEARNING then
            local centroid_x = sum_x / active_count
            local centroid_y = sum_y / active_count
            local centroid_dist, centroid_angle_rad = math_helper.cartesianoAPolarPAR(centroid_x, centroid_y)
            local centroid_angle = centroid_angle_rad * (180 / math.pi)

            if Print_tracker_trayectory_data then
                local ang_total_cent = centroid_angle_rad + obs_ang_offset_global
                local cgx, cgy = math_helper.polarACartesianoPAR(centroid_dist, ang_total_cent)
                Print_tracker_trayectory_data("Active_learning", 3, "Qupa4", 0, 0, cgx/100, cgy/100)
            end

            if closest_master_dist < front_threshold and motion_active and al_current_state == "PERSIGUIENDO_CENTROIDE" then
                srt_ang_mark_ant = normalize_angle_rad(move_angle + srt_ang_mark_ant)
                x_ant_r = x_mark
                y_ant_r = y_mark
                motion_active = false
                r_move()  
                robot.wheels.set_velocity(0, 0)
                change_al_state("IDLE", "") 
                pid_error_integral = 0      
            end

            if not motion_active then
                if closest_master_dist < front_threshold then
                    motion_active = true
                    pid_error_integral = 0  
                    
                    local diff_ang = centroid_angle - angle_of_closest
                    if diff_ang > 180 then diff_ang = diff_ang - 360 end
                    if diff_ang < -180 then diff_ang = diff_ang + 360 end
                    local safe_angle = 0
                    local msg_evasion = ""
                    if math.abs(diff_ang) < 60 then 
                        if diff_ang >= 0 then safe_angle = angle_of_closest + 60 
                        else safe_angle = angle_of_closest - 60 end
                        msg_evasion = string.format("EVADIENDO MASTER | Rodeando obstáculo hacia %.1f°", safe_angle)
                    else
                        safe_angle = centroid_angle
                        msg_evasion = string.format("EVADIENDO MASTER | Ruta libre hacia %.1f°", safe_angle)
                    end
                    last_motion_marker = {back_threshold * porcentaje, safe_angle}
                    change_al_state("EVADIENDO_MASTER", msg_evasion)
                    r_move()
                else
                    -- CÁLCULOS DEL PID
                    local error_dist = centroid_dist - target_centroid_distance
                    local pid_output = 0

                    if math.abs(error_dist) > pid_tolerance then
                        local p_term = Kp * error_dist
                        pid_error_integral = pid_error_integral + error_dist
                        local i_term = Ki * pid_error_integral
                        local d_term = Kd * (error_dist - pid_last_error)
                        pid_last_error = error_dist
                        
                        pid_output = p_term + i_term + d_term
                        if pid_output > max_pid_output then pid_output = max_pid_output 
                        elseif pid_output < min_pid_output then pid_output = min_pid_output end
                        
                        motion_active = true
                        last_motion_marker = {pid_output, centroid_angle}
                        change_al_state("PERSIGUIENDO_CENTROIDE", "PID")
                        r_move()
                    else
                        pid_error_integral = 0
                    end

                    -- >>> AQUÍ IMPRIMIMOS AL NUEVO TRACKER PID <<<
                    if Print_tracker_AL_PID then 
                        Print_tracker_AL_PID(target_centroid_distance, centroid_dist, pid_output)
                    end
                end
            end
        end

        -- FASE G: Gestión de Pérdidas
        for k = 1, #srt_anterior_data do
            local track = master_tracking[k]
            local is_lost = (srt_anterior_data[k] and srt_anterior_data[k].angle and not updated_anterior_indices[k])
            local is_finished = track and track.End_of_pattern_detected
            if is_lost then
                if SRP_LOG_MODE == 1 or  SRP_LOG_MODE == 3 then
                    log("\n[!] Maestro " .. k .. " PERDIDO.")
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

        -- FASE H: Grabación Continua
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

                -- CORRECCIÓN: Aquí se usa "trajectory" para asegurar compatibilidad
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

function proximidad()
    local sensores = robot.proximity
    local n_sensores = #sensores

    local sum_val = 0
    local count = 0

    local sum_x = 0
    local sum_y = 0

    -- 1) Recorremos todos los sensores
    for i, s in ipairs(sensores) do
        local v = s.value
        --log(string.format("[Proximity Sensor %d] valor=%.3f", i, v))

        if v > 0.01 then
            local angle = obtener_angulo_sensor(i, n_sensores)
            if angle == nil then
                proximity[1] = 0
                proximity[2] = 0
                return proximity
            end

            local rad = math.rad(angle)

            sum_val = sum_val + v
            count = count + 1

            sum_x = sum_x + v * math.cos(rad)
            sum_y = sum_y + v * math.sin(rad)
        end
    end

    -- 2) Promedio de intensidad
    if count > 0 then
        proximity[1] = sum_val / count
    else
        proximity[1] = 0
        proximity[2] = 0
        return proximity
    end

    -- 3) Ángulo promedio ponderado
    if sum_x == 0 and sum_y == 0 then
        proximity[2] = 0
    else
        local angle_rad

        if math.atan2 ~= nil then
            angle_rad = math.atan2(sum_y, sum_x)
        else
            angle_rad = math.atan(sum_y, sum_x)
        end

        proximity[2] = math.deg(angle_rad)
    end

    -- 4) Log para depuración
    -- log(string.format(
    --     "Promedio IR = %.3f, Ángulo promedio ≈ %.1f°",
    --     proximity[1], proximity[2]
    -- ) .. " " .. robot.id)

    return proximity
end

-- función  de caminata aleatoria
RW_Counter = 0
RW_b3 = 0
RW_b2 = 0
RW_b1 = 0

-------------------------------------------------------------------------------------------------------------------------------------
-------------------------------------------------------------------------------------------------------------------------------------
-- Funcines de acciones de los Qupa
-------------------------------------------------------------------------------------------------------------------------------------
-------------------------------------------------------------------------------------------------------------------------------------

function RandomWalking()

    proximidad()
    if proximity[1] > 0.0001 and RW_b1 == 0  then
        RW_b1 = 1
        --log("Obstaculo detectado a "..proximity[1].." En un angulo de: "..proximity[2].." por: "..robot.id)
    elseif RW_b1 == 0 then
        -- Generar un giro aleatorio ocasionalmente
        local random = robot.random.uniform_int(0, 60)
        if random >= 10 and random <= 20 then
            robot.wheels.set_velocity(random, -random)
        elseif random >= 30 then
            robot.wheels.set_velocity(random, random)
        end
    end

    if RW_b1 == 1 then
        if proximity[2] < 90 and  proximity[2] > -90 and proximity[1] > 0.0001 then
            RW_b2 = 1
            --log("detectado en frente: "..proximity[2].." debo retroceder... "..robot.id)

        elseif proximity[2] < -120 or  proximity[2] > 120 then
           -- log("detectado atras: "..proximity[2].." debo avanzar... "..robot.id)
            RW_b2 = 2
        elseif proximity[2] >= 90 and  proximity[2] <= 120  or proximity[2] <= -90 and  proximity[2] >= -120 then
            -- log("---------- No hay problema, objeto ubicado a un lado.  ---------- ... "..robot.id)
            RW_b2 = 1
            -- RW_b1 = 0
            RW_Counter = 1
            -- c1 = 0
            -- c2 = 0
        end  
        --log("Girando..... "..robot.id)

        if RW_b2 == 1 then

            if RW_b3 == 0 then
                RW_Counter = RW_Counter + 1
                RW_b3 = 1
                --log("CAMBIO DE DIRECCION, retrocedi por obstáculo... "..robot.id)
            end

            move(-40, 0, 40)
            if c1 == 2 then
                RW_b1 = 2
                c1 = 0
                c2 = 0
                --log("Retrocedi por obstáculo... "..robot.id)
            end

        elseif RW_b2 == 2 then
            if RW_b3 == 1 then
                RW_Counter = RW_Counter + 1
                RW_b3 = 0
                --log("CAMBIO DE DIRECCION, avance por obstáculo... "..robot.id)
            end
            move(40, 0, 40)
            if c1 == 2 then
                RW_b1 = 2
                c1 = 0
                c2 = 0
                --log("Avance por obstáculo... "..robot.id)
            end
        end
    end

    if RW_b1 == 2 then
        if RW_Counter < 2 and RW_b2 == 1 then  
            move(0,180,30,80)   
        elseif RW_Counter < 2 and RW_b2 == 2 then  
            move(0,0,30,80)   
        else
            move(0,90,30,80)
        end
        if c1 == 2 then
           -- log("----------Colisión eviatada! ---------- ... "..robot.id)
            RW_b2 = 0
            RW_b1 = 0
            RW_Counter = 0
            c1 = 0
            c2 = 0
            r_move()
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

UMBRAL_INTENSIDAD_EVASION = 0.1   -- Sensibilidad para activar la evasión
DURACION_TICKS_ARCO = 50          -- Duración en ticks del arco de evasión
DEBUG_EVASION = 1                 -- 0: Nada, 1: Cambios de estado, 2: Sensores y estados continuos

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

    -- Calcular orientación exacta respecto a la meta desde el inicio (para poder tomar decisiones inteligentes)
    local angle_to_center = math.atan2(dy, dx)
    local orientation_quat = robot.positioning.orientation
    local orientation_angle = math.atan2(
        2 * (orientation_quat.w * orientation_quat.z + orientation_quat.x * orientation_quat.y),
        1 - 2 * (orientation_quat.y ^ 2 + orientation_quat.z ^ 2)
    )
    local angle_difference = angle_to_center - orientation_angle
    angle_difference = math.atan2(math.sin(angle_difference), math.cos(angle_difference))

    -- --- 2. Búsqueda del sensor más crítico ---
    local max_val = 0
    local max_angle = 0
    local n_sensores = #robot.proximity

    for i, s in ipairs(robot.proximity) do
        if s.value > max_val then
            max_val = s.value
            local angulo = obtener_angulo_sensor(i, n_sensores)
            if angulo ~= nil then
                max_angle = angulo
            end
        end
    end

    -- Detectamos si el choque es FRONTAL (rango de -45° a 45°)
    local obstaculo_frontal = false
    if max_val > UMBRAL_INTENSIDAD_EVASION and math.abs(max_angle) <= 45 then
        obstaculo_frontal = true
    end

    -- --- 3. LOGS DE DEPURACIÓN ---
    if DEBUG_EVASION > 0 then
        -- Imprimir solo 1 vez cuando cambia el estado
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

        -- Imprimir continuo si el nivel es 2
        if DEBUG_EVASION >= 2 then
            log(string.format("[%s] MODO: %s | MaxProx: %.3f a %.1f° | AngMeta: %.1f°", 
                robot.id, estado_evasion.modo, max_val, max_angle, math.deg(angle_difference)))
        end
    end

    -- --- 4. Máquina de Estados de Evasión ---
    if estado_evasion.modo == "NORMAL" then
        if obstaculo_frontal then
            if max_angle > 5 then
                estado_evasion.angulo_giro = -90
            elseif max_angle < -5 then
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
            -- Si choca de frente mientras hace el arco, recalcula un nuevo giro
            if max_angle > 5 then
                estado_evasion.angulo_giro = -90
            elseif max_angle < -5 then
                estado_evasion.angulo_giro = 90
            else
                estado_evasion.angulo_giro = 90 * estado_evasion.alternar_giro
                estado_evasion.alternar_giro = estado_evasion.alternar_giro * -1
            end
            r_move()
            estado_evasion.modo = "GIRANDO"
            return false

        -- [NUEVO]: Si está mirando hacia la meta (margen de ~20 grados / 0.35 rad) 
        -- Y no hay obstáculos en su frente, entonces aborta el arco y va directo.
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
        local speed_turn_factor = 100 
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
            { bigger*20 ,60 },
            { bigger*20 ,120 },
            { bigger*20 ,120 },
            { 0, 60}
            },
            name = "triangulo"
        }
    end


    if figura == "octagono"then
        a = {
            params = {
            { bigger*20 ,-45 },
            { bigger*20 ,-45 },
            { bigger*20 ,-45 },
            { bigger*20 ,-45 },
            { bigger*20 ,-45 },
            { bigger*20 ,-45 },
            { bigger*20 ,-45 },
            { bigger*20 ,-45 },
            },
            name = "octagono"
        }
    end

    if figura == "triangulo_irregular_1" then
        a = {
            params = {
            { 40, 10},
            { 30, 90},
            { 50, 127},
            { 0, 133}
            },
            name = "Triangulo irregular 1"
        }
    end

    if figura == "Pentágono_irregular" then
        a = {
            params = {
            { bigger*33.33, 30},
            { bigger*33.33, 90},
            { bigger*33.33, 53},
            { bigger*33.33, 74},
            { bigger*38.867, 84},
            { 0, 29}
            },
            name = "Pentágono_irregular"
        }
    end

    if figura == "Heptágono" then
        a = {
            params = {
            { 50, 10},
            { 40, 90},
            { 28, 45},
            { 30, 45},
            { 28, 45},
            { 20, 45},
            { 28, 45},
            { 0, 35}
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