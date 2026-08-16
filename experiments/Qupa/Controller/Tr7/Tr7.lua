--=============================================
--   MEJORAS :  -- Uso de funciones de usuario.
--=============================================
-------------------------------------------------------------------------------------------------------------------------------------
-------------------------------------------------------------------------------------------------------------------------------------
-------------------------------------------------------------------------------------------------------------------------------------
--                                                  VARIABLES GLOBALES                                                             --
-------------------------------------------------------------------------------------------------------------------------------------
-------------------------------------------------------------------------------------------------------------------------------------
-------------------------------------------------------------------------------------------------------------------------------------

--[[ 
--================== ACTIVAR PARA EPCUK ==================

-- 721549    semilla 1er exp. Qupa

master_threshold = 12 
threshold = 10 

speed = 15
g_speed = 4

perm1c = 20
perm2c = 20
permt = 10
]]

--================== ACTIVAR PARA FOOT-BOT ==================
-----------------------------
--Parámetro de simulación externos

-- 187014    semilla 1er exp. FOOT
math.randomseed(os.time())

ticks_per_second                    = 10                        -- Número de ticks que representa un segundo en la simulación (ajusta según la configuración de .argos)
max_time                            = 500 * ticks_per_second    -- Duración máxima de la simulación en ticks
Distribucion_sensores_de_proximidad = {0,-45,45,-90, 90,180}    -- Ángulos de los sensores de proximidad a considerar para la detección de obstáculos (en grados) --> {} o "uniformes"
--Distribucion_sensores_de_proximidad = "uniform"               -- Ángulos de los sensores de proximidad a considerar para la detección de obstáculos (en grados) --> {} o "uniformes"
Num_cantidad_leds                   = 2
-- Formula para calcular la dstancia de detección de observación: d = tan(angulo_de_apertura)*altura_de_la_camara -> Qupa: d = tan(70.71°) * 0.245 = 0.7 m

-- Importamos 
math_helper = require("Functions.math_helper")
------------------------------
--Parámetro de simulación internos

speed_dem = 30                  -- Velocidad del master al ejecutar el patrón
d_speed   = 15                  -- velocidad normal
g_speed   = 50                  -- Velocidad de giro

-- Métricas de paciencia
MAX_PATIENCE        = 600       -- Tiempo máximo de espera sin recibir datos útiles
BASE_RESTORE        = 200       -- Paciencia base a restaurar con datos refinados/actualizados

-- 
master_threshold      = 41      -- Distancia máxima a la que a la que el maestro espera ver al apprentice
threshold             = 40      -- Distancia a la que se ubica el apprentice del maestro
max_tracking_distance = 165     -- Máxima distancia a la que se puede alejar el maestro antes de que el alumno se acerque para no perderlo de vista. cm
min_tracking_distance = 30      -- Miínima distancia a la que se puede acercar el maestro antes de que el alumno se aleje para no colisionar. cm
porcentaje            = 0.5     -- 1 < porcentaje < 0.33 , indica que tanto el apprentice busca acercarse al maestro cuando este cruza el max_tracking_distance.

--
angulo_treshold     = 15                                        -- Sensibilidad del apprentice a caambios en los datos de angulo obtenidos del maestro, ajuste recomendado: 10 < distance_treshold < 30
distance_treshold   = 3*(speed_dem+d_speed)/ticks_per_second    -- Sensibilidad del apprentice a caambios en los datos de distancia obtenidos del maestro durante la etapa de recolección, ajuste recomendado: 1 < distance_treshold < 2 

--
factor_de_suficiencia   = 0.3     -- Porcetaje ajustable según cuán estrictos queramos ser con la separación mínima de los datos en el método de Otsu. 
HIST_N_ADELANTE         = 5       -- Cantidad de paquetes a mirar hacia adelante (incluyendo el actual)
HIST_M_PORCENTAJE       = 0.8     -- Porcentaje mínimo que debe cumplir (0.8 = 80%)

-- 
CONFIG = {
    TOL_DIST                = distance_treshold/2,  -- (cm)
    TOL_ANG                 = angulo_treshold,      -- (°)
    MIN_OVERLAP_M1          = 1,                    -- Solape mínimo independiente para Greedy (M1)
    MIN_OVERLAP_M2          = 1,                    -- Solape mínimo independiente para Exhaustivo (M2)
    PRINT_RECURSIVE_PATHS   = true,                 -- Imprime los pasos de M2
    FORCE_M2_ALWAYS         = false,                -- Ejecutar M2 incluso si M1 fue perfecto
    MAX_ISLANDS_M2          = 9,                    -- Límite de islas para M2
    
    selection_mode          = 3,                    -- 1=Frecuente, 2=Larga, 3=Función de Ponderación (Score)
    
    W_FREQ                  = 1,                    -- Peso de la frecuencia
    W_LEN                   = -1,                   -- Positivo: Favorece Cortas. Negativo: Favorece Largas. 0: Ignorar.
    W_OV_QUAL               = 2,                    -- Peso de la calidad de solapes

    MATH_TOLERANCE          = 0.5,                  -- Tolerancia matemática M3
    
    
    -- 1. FILTRADO DE RUIDO (Tolerancia a la basura del sensor)
    RUIDO_BASE              = 0.2,  -- (20%) Piso mínimo absoluto. Limpia los errores inevitables de cuantización del sensor, incluso en enjambres perfectos.
    RUIDO_PENALIZACION_MAX  = 0.15, -- (+15%) Tope extra que se suma al ruido base en escenarios de caos total. Evita procesar colisiones masivas como datos válidos.

    -- 2. EL GEÓMETRA M3 (Ventaja para polígonos regulares)
    BONO_M3_BASE            = 1.0,  -- (x1.0) Multiplicador neutral. Las figuras geométricas compiten de igual a igual con las rutas irregulares por defecto.
    BONO_M3_MAX_EXTRA       = 0.5,  -- (+0.5) Premio máximo. Si el Termostato lee mucha simetría pura en la arena, los polígonos ganan hasta un 50% de ventaja para asegurar su victoria por la "Navaja de Ockham".
    
    -- 3. ESCEPTICISMO DEL ÁRBITRO (Score mínimo para validación)
    SCORE_MIN_BASE          = 0.25, -- (5%) Cobertura mínima absoluta requerida. Todo patrón debe explicar al menos al 5% del volumen del enjambre para no ser descartado.
    SCORE_PENALIZACION_MAX  = 0.10, -- (+10%) Blindaje estadístico. En escenarios altamente fragmentados, se sube la exigencia hasta un 15% total para evitar aceptar cadenas formadas por mera casualidad (alucinaciones).
    
    -- 4. CASTIGO ESTRUCTURAL
    PESO_COMPLEJIDAD        = 0.25  -- Constante de penalización para trayectorias largas. Evita que las rutas irregulares inmensas y complejas (M2) le ganen injustamente a las geometrías cortas y elegantes (M3) solo por abarcar más volumen bruto de datos.
}

-- CONFIGURACIÓN DEL MODELO DE RUIDO (SENSOR NOISE CONFIG)
-- Interruptores Maestros (True/False)
ENABLE_NOISE = true            -- Si es false, el robot ve perfecto (Ground Truth)
USE_OUTLIERS = false           -- Activar lecturas "fantasma" o fallos graves
USE_DISTANCE_DEPENDENT = true -- El ruido aumenta si el objeto está lejos
USE_ANGLE_NOISE = true         -- Ruido en la medición del ángulo
USE_BIAS = false                -- Error sistemático (ej. sensor mal calibrado +2cm siempre)

-- Parámetros de Intensidad
NOISE_LEVEL_DIST    = 0.0100       -- % de error proporcional a la distancia
NOISE_LEVEL_ANGLE   = 1.0500       -- ±2 grados de desviación estándar
OUTLIER_PROB        = 0.01           -- 1% de probabilidad de fallo catastrófico
BIAS_OFFSET         = 1.5               -- Cm extra que agrega el sensor por defecto (si USE_BIAS es true)

--Parametros de patrónes predefinidos
bigger = 1  -- Factor de escala para ajustar el tamaño de las figuras predefinidas (ajusta según tu espacio de simulación)

if robot.id == "Qupa4" then

elseif robot.id == "Qupa1" or robot.id == "Qupa2" or robot.id == "Qupa3" or robot.id == "Qupa5" then
    figura = "Pentágono_irregular" -- "triangulo_regular" -- "octagono" -- "triangulo_irregular_1" --"Pentágono_irregular"  -- "Heptágono"-- "" -- "trayectoria_1" 
    bigger = 1
else
    figura = "triangulo_regular" -- "triangulo_regular" -- "octagono" -- "triangulo_irregular_1" --"Pentágono_irregular"  -- "Heptágono"-- "" -- "trayectoria_1" 
    bigger = 1  -- Factor de escala para ajustar el tamaño de las figuras predefinidas (ajusta según tu espacio de simulación)
end

-- Banderas de loggeo

OBSERVER_LOG_MODE   = 1 -- 0: Sin logs, 1: Logs detallados 
DS_LOG_MODE         = 0 -- 0: Sin logs, 1: Logs detallados, 2: Solo resumen final, 3: Resumen + puntos en segmento, 4: Solo parámetros de Otsu
FVS_LOG_MODE        = 0 -- 0: Sin logs, 1: Logs detallados, 2: Solo resumen final, 3: Resumen + detalles de filtrado
-------------------------------------------------------------------------------------------------------------------------------------
-------------------------------------------------------------------------------------------------------------------------------------
-------------------------------------------------------------------------------------------------------------------------------------
--                                                      MAIN FUNCTIONS                                                             --
-------------------------------------------------------------------------------------------------------------------------------------
-------------------------------------------------------------------------------------------------------------------------------------
-------------------------------------------------------------------------------------------------------------------------------------

-------------------------------------------------------------------------------------------------------------------------------------
-- Funciones para los trackers
-------------------------------------------------------------------------------------------------------------------------------------

displayed_data  = 3 --1 Datos obtenidos por el apprentice. 2. Datos globales del movimiento del master, 3: Ambos
function Print_tracker_trayectory_data(k, x, y)

    if type(k) == "number" then

        if displayed_data == 1 or displayed_data == 3 then
            print("Qupa" .. k .. " & " .. x .. " , " .. y)
        end

    elseif type(k) == "string" then

        -- Obtener el vector de posición inicial
        local x = robot.positioning.position.x
        local y = robot.positioning.position.y

        if displayed_data == 2 or displayed_data == 3 then
            if robot.id == "Qupa4" then
    
                print("Qupa400 & " .. x .. " , " .. y)  -- Imprime los datos globales del movimiento del Observador
            else     
                --print(robot.id .. " & " .. x .. " , " .. y)  
            end
        end
    end
end

function Print_tracker_calidad_imitacion_data(linea_resumen, is_master)

    if is_master == 0 then

        print(linea_resumen)
    else

        -- Función auxiliar para separar la parte entera y los dos decimales de un número
        local function split_num(val)
            local sign = val < 0 and "-" or ""
            local abs_val = math.abs(val)
            local int_part = math.floor(abs_val)
            local dec_part = math.floor((abs_val - int_part) * 100 + 0.5)
            
            -- Ajuste por si el redondeo sube el decimal a 100
            if dec_part >= 100 then
                int_part = int_part + 1
                dec_part = dec_part - 100
            end
            
            -- Retorna dos valores: la parte entera (con su signo) y la parte decimal a 2 dígitos
            return string.format("%s%d", sign, int_part), string.format("%02d", dec_part)
        end
        
        -- Obtenemos el patrón correspondiente a la variable global 'figura' actual
        local data = original_params()
        
        -- Verificamos que se haya encontrado un patrón válido
        if not data or not data.params then 
            log(robot.id .. " & Error: No se encontró un patrón asignado.")
            return 
        end
        
        local nombre_patron = data.name or "Desconocido"
        -- Iniciamos la cadena con el formato solicitado
        local resultado = robot.id .. " & Original&" .. nombre_patron
        
        local params = data.params
        for j = 1, #params do
            local dist = params[j][1]
            local ang = params[j][2]
            
            -- Separamos enteros y decimales de la distancia y el ángulo
            local dist_i, dist_d = split_num(dist)
            local ang_i, ang_d = split_num(ang)
            
            -- Concatenamos el paso con el formato (distI,distD,angI,angD)
            resultado = resultado .. string.format("(%s,%s,%s,%s)", dist_i, dist_d, ang_i, ang_d)
            
            -- Agregamos la coma separadora entre pasos (excepto en el último)
            if j < #params then
                resultado = resultado .. ","
            end
        end
        -- Imprimimos el resultado final en la terminal de ARGoS
        log(resultado)
    end
end

function Print_tracker_varianza(k, var, separacion, avance)
    --print( "Maestro"..k.." & " .. var)                -- Datos para el anális de varianza en los datos obtenidos por el apprentice. Script -> run_sim.py
   -- print( "Otsu & " .. separacion.." , "..avance)    -- Datos para el anális del método de otsu a distintos valores de velocidad del master: speed_dem  -> Script: master_evluator.py
end

-------------------------------------------------------------------------------------------------------------------------------------
-- Función que se ejecuta al inicio de la simulación
-------------------------------------------------------------------------------------------------------------------------------------

function init()
    
    robot.colored_blob_omnidirectional_camera.enable()
    determinar_rol()
    Print_tracker_calidad_imitacion_data(robot.id, is_master)
end

-------------------------------------------------------------------------------------------------------------------------------------
-- Función principal que se ejecuta en cada tick
-------------------------------------------------------------------------------------------------------------------------------------

timer = 1
function step()

    if is_master == 1 then
        master() -- Ejecutar si es master
    else
        apprentice()
    end

    if timer == max_time then
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
c5                      = 0
master_avg_angle   = 0
master_avg_dist    = 0
first_time_demonstrator = true
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
            -- Escoge un índice aleatorio entre 1 y la cantidad de patrones guardados
            --local indice_aleatorio = robot.random.uniform_int(1, #all_params)
            local indice_aleatorio = math.random(1, #all_params)
            parametros_elegidos = all_params[indice_aleatorio]
            --log(" ========= Ejecutando patrón aleatorio (1,"..#all_params..") , -> "..indice_aleatorio..", "..robot.id.." ========= ")
            --print_table(parametros_elegidos)
            
        else
            -- Respaldo de seguridad por si la tabla aún está vacía
            parametros_elegidos = original_params()
            parametros_elegidos = parametros_elegidos.params
            --log(" ========= Ejecutando patrón original: " .. robot.id.." ========= ")
            --print_table(parametros_elegidos)

        end

        d1 = 3

    elseif d1 == 3 then
        patron_imitado(parametros_elegidos)
    end
end

c4 = 0
first_time_apprentice = true
function apprentice()

    if c4 == 0 then
        store_relative_positions()
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

master_avg_dist_1 = 0
EAEP_ang_ant = 0
EAEP_c1 = 0
EAEP_c2 = 0
EAEP    = 0
function espera_antes_de_ejecutar_patron()
    if EAEP_c2 == 0 then
        local blobs = robot.colored_blob_omnidirectional_camera
        local total_distance = 0
        local count = 0  
        local vel_g = 10
        master_avg_dist = 0

        Temporizador_1(15)

        --setleds("yellow")
        if temp == false then
            EAEP_c2 = 1
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
-- función para realizar el patron imitado
-------------------------------------------------------------------------------------------------------------------------------------

k               = 0
c5              = 0
indice          = 1
PI_c1           = 0
PI_end          = false
PI_counter_seg  = 0
function patron_imitado(params_P)

    local params = params_P
    local prox = proximidad()

    -- if prox[1] <= 0.5 then
        PI_c1 = 1
    -- elseif prox[1] > 0.5 and  prox[2] >= -90 and  prox[2] <= 90 then 
    --     PI_c1 = 0
    --     k = 1
    --     setleds("black")

    --     if robot.id == "Qupa4" then 
                
    --         log( " -------------------------------------------------------------------------" )
    --         log("patrón imitado: NO finalizado debido a obstaculo en el camino!!, proximidad del ".. proximity[1]*100 .."%, "..robot.id)
    --         log( " -------------------------------------------------------------------------" )
    --     else

    --         log( " -------------------------------------------------------------------------" )
    --         log("patrón demostrado: NO finalizado debido a obstaculo en el camino!!, proximidad del ".. proximity[1]*100 .."%, "..robot.id)
    --         log( " -------------------------------------------------------------------------" )
    --     end
    -- else 
    --     PI_c1 = 1
    -- end

    if PI_c1 == 1 and k == 0 then

        if #params >= 1 then

            if indice <= #params then
                local valor1, valor2 = params[indice][1], params[indice][2]
                move(valor1, valor2, speed_dem, 10, false, robot.id)
                -- log("\nDistancia a recorrer en el segmento "..indice.." = ".. params[indice][1]..", Distancia actual: "..move_distance..", "..robot.id)
                -- log("Angulo del segmento = "..indice.." = ".. params[indice][2]..", Angulo actual: "..move_angle*180/math.pi ..", "..robot.id)
                Print_tracker_trayectory_data(robot.id,0,0)

                setleds("yellow")

                if c1 == 2 then
                    PI_counter_seg = PI_counter_seg + 1
                    if indice == #params then

                        indice = indice + 1
                        PI_end = true
                        r_Temporizador_1()

                                    
                        if c5 == 1 then
                            k = 1
                            -- log(" ========================================================================= ")
                            -- log(" Patrón Imitado: finalizado por: ".. robot.id)                            
                            --imprimir_posicion(posiciones_guardadas)

                        else
                            c5 = 1
                            k = 1
                            --log(" ========= Patrón: finalizado por: ".. robot.id.." ========= ")
                        end
                    else
                        --print_table(params)
                        indice = indice + 1
                        c1 = 0
                    end
                end
            end
        else
            --resetOBS()
            --is_master = 0

            k = 1

            robot.leds.set_all_colors("black")
            log( " -------------------------------------------------------------------------" )
            log("Patrón no detectado :( " .. robot.id)
            log( " -------------------------------------------------------------------------" )
        end
    end

    -- elseif prox[1] >= 0.9 and k == 0 then
    --     k = 1
    --     setleds("black")
    --     log("patrón imitado: NO finalizado debido a obstauclos: " .. robot.id)
    -- end
    if k == 1 then
--        d1 = 3
        robot.wheels.set_velocity(0, 0)   -- Detenerse

        if robot.id == "Qupa4" then 
            k = 1
        else
            k = 2
            setleds("black")
        end

    elseif k==2 then
        resetDEM()
        d1 = 2
        PI_counter_seg = 0
    end
end

-------------------------------------------------------------------------------------------------------------------------------------
-------------------------------------------------------------------------------------------------------------------------------------
-- MODULO DEL OBSERVADOR
-------------------------------------------------------------------------------------------------------------------------------------
-------------------------------------------------------------------------------------------------------------------------------------

-- ============================================================================
-- [SECCIÓN 1] IMPORTACIÓN DE SUBMÓDULOS DEL OBSERVADOR
-- ============================================================================
NoiseManager                    = require("Functions.Observer.noise_manager")                   -- Módulo para simular el ruido de la cámara de blobs
PointCloudSegmenter             = require("Functions.Observer.Point_Cloud_Segmenter")           -- Módulo para segmentar y filtrar la nube de puntos obtenida al observar a cada maestro 
SegmentParameterizer            = require("Functions.Observer.Segment_Parameterizer")           -- Módulo para convertir los segmentos en parámetros reproducibles por el robot (Distancias,Angulos)
ParameterStorage                = require("Functions.Observer.Parameter_Storage")               -- Módulo para almacenar los parámetros de movimiento obtenidos por el apprentice 
GlobalTrajectoryReconstructor   = require("Functions.Observer.Global_Trajectory_Reconstructor") -- Módulo para reconstruir el/las trayectorias a partir de los datos almacenados por el apprentice (cada trayectoria es un comportamiento de movimiento)

-- ============================================================================
-- [SECCIÓN 2] VARIABLES GLOBALES DE ESTADO Y MEMORIA
-- ============================================================================
current_patience  = MAX_PATIENCE
master_tracking   = {} -- Memoria detallada de cada maestro observado
srt_anterior_data = {} -- Registro temporal para enlazar blobs entre ticks (tracking)

-- ============================================================================
-- [SECCIÓN 3] GESTIÓN DE PACIENCIA Y RECONSTRUCCIÓN DEL/LOS COMPORTAMIENTO DE MOVIMIENTO
-- ============================================================================

-- A. Actualizar Paciencia (Suma tiempo cuando se descubren datos útiles)
function update_patience(restore_type, count)
    if restore_type == "full" then
        current_patience = MAX_PATIENCE
        if OBSERVER_LOG_MODE == 1 then log("[PACIENCIA] Restaurada completamente (" .. MAX_PATIENCE .. ").") end
    elseif restore_type == "partial" then
        local divisor = (count > 1) and (count - 1) or 1
        local restore_amount = math.floor(BASE_RESTORE / divisor)
        current_patience = math.min(MAX_PATIENCE, current_patience + restore_amount)
        if OBSERVER_LOG_MODE == 1 then log("[PACIENCIA] Restauración parcial: +" .. restore_amount .. ". Actual: " .. current_patience) end
    end
end

-- B. Verificar Paciencia (Descuenta tiempo y dispara la reconstrucción si llega a 0)
function check_patience_and_reconstruct()
    if ParameterStorage.is_done() then return end

    current_patience = current_patience - 1

    if current_patience <= 0 then
        if _LOG_MODE == 1 then log("\n\n\n\n PACIENCIA AGOTADA: Ejecutando Reconstrucción...") end

        -- PASO 4: Reconstrucción Global. Al agotarse la paciencia (ya no hay datos nuevos útiles), el robot toma todos los parámetros parciales guardados en la memoria y los une como un rompecabezas. 
        -- A partir de los diferentes fragmentos aportados por múltiples maestros observados, reconstruye la trayectoria completa (el comportamiento de movimiento).
        local datos_guardados = ParameterStorage.get_stored_data()
        
        local current_cfg = { CONFIG = CONFIG, LOG_MODE = OBSERVER_LOG_MODE }   
        local exito, linea_resumen, nuevos_patrones = GlobalTrajectoryReconstructor.run(robot.id, datos_guardados, current_cfg)
        
        if exito then
            ParameterStorage.set_done()
            Print_tracker_calidad_imitacion_data(linea_resumen, is_master)
            for _, patron in ipairs(nuevos_patrones) do
                table.insert(all_params, patron)
            end
            -- Cambiar de estado (Se vuelve maestro y termina la observación)
            resetOBS()
            is_master = 1
            c4 = 1 
        end
    end
end

-- ============================================================================
-- [SECCIÓN 4] FUNCIONES AUXILIARES DEL OBSERVADOR (TRACKING CRUDO)
-- ============================================================================

-- A. Inicializa la estructura de memoria para un nuevo robot observado
function initialize_master_tracking(k, angle_deg, distance, ty)
    local angle_rad = angle_deg * (math.pi / 180)
    local x, y = math_helper.polarACartesianoPAR(distance, angle_rad) 

    master_tracking[k] = {
        raw_x = {x}, raw_y = {y}, raw_type = {ty},
        x_ant_r = 0, y_ant_r = 0, srt_ang_mark_ant = 0,
        x_mark = 0, y_mark = 0, 
        Pattern_start_detected = (ty == 1), 
        End_of_pattern_detected = (ty == 3),
        processed = false, 
        segmentos = { movement = {}, type = {}, valid = {} },
        params = {},
    }
end

-- B. Acumula los puntos (X,Y) en cada tick mientras el maestro se mueve
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

-- Transforma el historial de puntos crudos de un robot en parámetros geométricos limpios
function process_single_observation(k, track)
    if OBSERVER_LOG_MODE == 1 then
        log(" ######################################################## ")
        log(" PROCESANDO OBSERVACIÓN DEL MAESTRO "..k.." (Datos totales: "..#track.raw_x..")")
        log(" ######################################################## ")
    end

    local seg_cfg = {
        DS_LOG_MODE             = DS_LOG_MODE,
        FVS_LOG_MODE            = FVS_LOG_MODE,
        factor_de_suficiencia   = factor_de_suficiencia,
        HIST_N_ADELANTE         = HIST_N_ADELANTE,
        HIST_M_PORCENTAJE       = HIST_M_PORCENTAJE,
        distance_treshold       = distance_treshold,
        mostrar_todos_los_puntos= mostrar_todos_los_puntos or false
    }

    -- PASO 1: Toma la nube de puntos obtenida del maestro observado y la segmenta. 
    -- Ejem: La nube de puntos de un triángulo se segmenta en 3 nubes de puntos (una por cada lado). 
    -- Además, aplica filtros para asegurar que el ruido de la cámara no genere segmentos inexistentes.
    local clean_mov, clean_type, clean_valid, p_start, p_end = PointCloudSegmenter.process(k, track.raw_x, track.raw_y, track.raw_type, seg_cfg)

    track.segmentos.movement = clean_mov
    track.segmentos.type     = clean_type
    track.segmentos.valid    = clean_valid
    
    if p_start then track.Pattern_start_detected  = true end
    if p_end   then track.End_of_pattern_detected = true end


    if #track.segmentos.movement > 1 then 
        -- PASO 2: Aplica regresión ortogonal a las nubes de puntos segmentadas para obtener parámetros que el robot puede reproducir. 
        -- Convierte los puntos espaciales en distancias y ángulos que el robot puede llevar a sus motores. 
        -- Ej: El caso del triángulo convertiría los puntos en: params = {{20,120}, {20,120}, {20,120}} -> {distancia, angulo}
        track.params = SegmentParameterizer.process_segments(track.segmentos.movement)

        -- PASO 3: Guarda los parámetros extraídos de este maestro en la memoria. 
        -- De esta forma, cuando se acabe la paciencia, el sistema podrá realizar la reconstrucción global a partir de la información parcial aportada por cada maestro.        
        local current_cfg = { CONFIG = CONFIG, LOG_MODE = OBSERVER_LOG_MODE }
        local r_type, r_count = ParameterStorage.package_and_add(track.params, track.segmentos.valid, current_cfg)
        
        if r_type then update_patience(r_type, r_count) end

    else
        if OBSERVER_LOG_MODE == 1 then log("No hay segmentos suficientes para procesar tras la limpieza.") end
    end
end

-- ============================================================================
-- [SECCIÓN 6] BUCLE PRINCIPAL DEL OBSERVADOR (TICK A TICK)
-- ============================================================================
srt_c1 = 0
function store_relative_positions()
    -- Control de paciencia y reconstrucción global dinámica
    check_patience_and_reconstruct()

    -- Lógica de Timer y Proximidad
    local proximity = proximidad()
    if proximity[1] < 1 then srt_c1 = 0 else
        if srt_c1 == 5 * ticks_per_second then resetOBS() return end
        srt_c1 = srt_c1 + 1
    end

    Temporizador_1(20) 
    if temp == false then robot.leds.set_all_colors("black") end
    
    if robot.colored_blob_omnidirectional_camera then                
        
        local blobs         = robot.colored_blob_omnidirectional_camera
        local yellow_blobs  = {} 
        local green_blobs   = {}  
        local count         = 0   

        -- Empaquetamos la configuración de ruido para el NoiseManager
        local noise_cfg = {
            ENABLE_NOISE           = ENABLE_NOISE,
            USE_OUTLIERS           = USE_OUTLIERS,              OUTLIER_PROB        = OUTLIER_PROB,
            USE_DISTANCE_DEPENDENT = USE_DISTANCE_DEPENDENT,    NOISE_LEVEL_DIST    = NOISE_LEVEL_DIST,
            USE_ANGLE_NOISE        = USE_ANGLE_NOISE,           NOISE_LEVEL_ANGLE   = NOISE_LEVEL_ANGLE,
            USE_BIAS               = USE_BIAS,                  BIAS_OFFSET         = BIAS_OFFSET
        }

        -- FASE A: Recolección y Ruido
        for i, blob in ipairs(blobs) do
            local raw_angle = blob.angle * (180 / math.pi)
            local raw_dist = blob.distance
            
            -- Aplicamos el ruido desde el módulo externo
            local noisy_dist, noisy_angle = NoiseManager.aplicar_ruido(raw_dist, raw_angle, noise_cfg) 
            
            local r, g, b = blob.color.red, blob.color.green, blob.color.blue

            if r > 200 and g > 200 and b < 100 then table.insert(yellow_blobs, {angle = noisy_angle, distance = noisy_dist, type = 2})
            elseif r < 100 and g > 200 and b < 100 then table.insert(green_blobs, {angle = noisy_angle, distance = noisy_dist})
            elseif  r > 200 and g < 100 and b < 100 then table.insert(yellow_blobs, {angle = noisy_angle, distance = noisy_dist, type = 1})
            end 
        end

        local srt_anterior_angle_positive = {}
        local srt_anterior_angle_negative = {}
        local updated_anterior_indices = {}
        local matched_new_indices = {}
        local fallos_maestros = {}

        -- Preparar índices anteriores
        for k, data in pairs(srt_anterior_data) do
            if data and data.angle then 
                if data.angle >= 0 then table.insert(srt_anterior_angle_positive, {angle = data.angle, original_index = k})
                else table.insert(srt_anterior_angle_negative, {angle = data.angle, original_index = k}) end
            end
        end

        -- FASE B: Actualización de Maestros Existentes (Yellow Blobs)
        for j, new_data in ipairs(yellow_blobs) do
            local target_list = (new_data.angle >= 0) and srt_anterior_angle_positive or srt_anterior_angle_negative
            for _, anterior_info in ipairs(target_list) do
                local k = anterior_info.original_index
                if not updated_anterior_indices[k] then
                    local anterior_data = srt_anterior_data[k]
                    if anterior_data.angle then 
                        local max_ang = math.abs(new_data.angle) + angulo_treshold
                        local min_ang = math.abs(new_data.angle) - angulo_treshold
                        
                        if math.abs(anterior_data.angle) < max_ang and math.abs(anterior_data.angle) > min_ang then
                            local max_dist = anterior_data.distance + distance_treshold
                            local min_dist = anterior_data.distance - distance_treshold
                            
                            if new_data.distance < max_dist and new_data.distance > min_dist then
                                srt_anterior_data[k].angle = new_data.angle
                                srt_anterior_data[k].distance = new_data.distance
                                srt_anterior_data[k].type = new_data.type 
                                
                                local x,y = math_helper.polarACartesianoPAR(srt_anterior_data[k].distance /100, srt_anterior_data[k].angle * math.pi / 180)
                                Print_tracker_trayectory_data(k, x, y)
                                
                                fallos_maestros[k] = nil 
                                updated_anterior_indices[k] = true
                                matched_new_indices[j] = true
                                count = count + 1
                                break 
                            else fallos_maestros[k] = string.format("Fuera de rango de distancia (Vio: %.2f)", new_data.distance) end
                        else fallos_maestros[k] = fallos_maestros[k] or "No coincidió con su ángulo." end
                    else fallos_maestros[k] = "Sin datos previos." end 
                end 
            end 
        end 

        -- FASE C: Crear Nuevos Maestros
        for j, new_data in ipairs(yellow_blobs) do
            if not matched_new_indices[j] then
                table.insert(srt_anterior_data, {angle = new_data.angle, distance = new_data.distance, type = new_data.type or 2})
                local new_k = #srt_anterior_data
                updated_anterior_indices[new_k] = true
                initialize_master_tracking(new_k, new_data.angle, new_data.distance, srt_anterior_data[new_k].type)
            end
        end

        -- FASE D: Procesar Blobs Verdes (Fin de Patrón Visual)
        for _, green_blob in ipairs(green_blobs) do
            for k, master_data in pairs(srt_anterior_data) do
                if master_data.angle and not updated_anterior_indices[k] then 
                     srt_anterior_data[k].type = 3 
                     updated_anterior_indices[k] = true 
                     master_tracking[k].End_of_pattern_detected = true 
                end
            end 
        end 

        -- FASE E: Gestión de Pérdidas y Procesamiento de Rutas
        for k = 1, #srt_anterior_data do
            local track = master_tracking[k]
            local is_lost = (srt_anterior_data[k] and srt_anterior_data[k].angle and not updated_anterior_indices[k])
            local is_finished = track and track.End_of_pattern_detected
            
            if is_lost then
                local razon = fallos_maestros[k] or "Sin blobs amarillos cerca."
                log("\n[!] Maestro " .. k .. " PERDIDO. Razón: " .. razon)
            end
            
            if (is_lost or is_finished) and track and not track.processed then
                -- ENVIAR AL PIPELINE DEL OBSERVADOR
                process_single_observation(k, track)
                
                track.processed = true
                srt_anterior_data[k].angle = nil
                srt_anterior_data[k].distance = nil
                srt_anterior_data[k].type = nil
            end
        end

        -- FASE F: Grabación Continua de Datos (Maestros Activos)
        for k, _ in pairs(updated_anterior_indices) do
            local master_data = srt_anterior_data[k]
            local track = master_tracking[k]

            if master_data and track and not track.processed then
                local current_distance = master_data.distance
                local current_angle_rad = master_data.angle * (math.pi / 180)
                local ty = master_data.type
                
                local ang_total = current_angle_rad + track.srt_ang_mark_ant
                local x_new, y_new = math_helper.polarACartesianoPAR(current_distance, ang_total)
                local x_original = x_new + track.x_mark
                local y_original = y_new + track.y_mark

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
-- Funcines de acciones/decisiones/sensores de los Qupa
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
t1                  = 0

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
        angle_ticks_needed = angle_ticks_needed + 1
        ticks_needed = ticks_needed + 1
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

            local L = 14 --   static const Real FOOTBOT_INTERWHEEL_DISTANCE      = 0.14f;
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

    posiciones_guardadas = {}
    
    k                       = 0
    indice                  = 1
    PI_c1                   = 0
    PI_end                  = false

    -- ============================
    c4            = 0
    -- ============================
    setleds("black")
    r_Temporizador_1()
    r_Temporizador_2()
    r_move()
end

function resetDEM() 

    k = 0

    -- ============================¿
    d1 = 0

    master_avg_angle = 0

    posiciones_guardadas = {}

    indice = 1

    EAEP_c1 = 0
    EAEP_c2 = 0
    master_avg_dist_1 = 0
    EAEP_ang_ant = 0
    -- ============================

    --setleds("red")
   
    r_Temporizador_1()
    r_Temporizador_2()
    r_move()

end


function r_move() 

    c1                  = 0
    c2                  = 0
    move_controller     = 1
    move_distance       = 0 

    ticks_needed        = 0
    actual_dis_ticks    = 0

    angle_ticks_needed  = 0
    actual_ang_ticks    = 0

    t1                  = 0

    time_needed         = 0
    speed               = d_speed

    move_distance       = 0; --restart distance
    move_angle          = 0; --restart angle

    wheel_speed         = 0
    omega_rad           = 0

    previous_speed_d    = nil
    stop = false
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
            { bigger*20 ,120 },
            { bigger*20 ,120 },
            { bigger*20 ,120 },
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
            { bigger*33.33, 10},
            { bigger*33.33, 90},
            { bigger*33.33, 53},
            { bigger*33.33, 74},
            { bigger*38.867, 84},
            { 0, 49}
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