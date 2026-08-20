--=============================================
--   MEJORAS :  -- Mejoras en la función move.
--              -- Mejoras en la función de selección y detección de patrones (Multiples patrones).
                -- Movimmiento dinámico del observador, siguiendo el centroide de los datos del maestros activos.
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

-- 721549    semilla 1er exp. epuck

demostrador_threshold = 12 
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
max_time                            = 600 * ticks_per_second    -- Duración máxima de la simulación en ticks
Num_sensores_de_proximidad          = 24                        -- Numero de sensores de proximidad del robot
------------------------------
--Parámetro de simulación internos

speed_dem = 30                   -- Velocidad del demostrador al ejecutar el patrón
d_speed   = 15                  -- velocidad normal
g_speed   = 50                  -- Velocidad de giro
 
-- Métricas de paciencia
MAX_PATIENCE        = 600       -- Tiempo máximo de espera sin recibir datos útiles
BASE_RESTORE        = 200       -- Paciencia base a restaurar con datos refinados/actualizados

-- 
demostrador_threshold = 41      -- Distancia máxima a la que a la que el maestro espera ver al observador
threshold             = 40      -- Distancia a la que se ubica el observador del maestro
max_tracking_distance = 60      -- Máxima distancia a la que se puede alejar el maestro antes de que el alumno se acerque para no perderlo de vista. cm
min_tracking_distance = 30      -- Miínima distancia a la que se puede acercar el maestro antes de que el alumno se aleje para no colisionar. cm
porcentaje            = 0.5     -- 1 < porcentaje < 0.33 , indica que tanto el observador busca acercarse al maestro cuando este cruza el max_tracking_distance.

MAX_WAIT_TICKS        = 100*ticks_per_second    -- Número máximo de ticks que el observador esperará hasta que el obstaculo se despeje del camino.
--
angulo_treshold     = 15                                        -- Sensibilidad del observador a caambios en los datos de angulo obtenidos del maestro, ajuste recomendado: 10 < distance_treshold < 50
distance_treshold   = 3*(speed_dem+d_speed)/ticks_per_second    -- Sensibilidad del observador a caambios en los datos de distancia obtenidos del maestro, ajuste recomendado: 1 < distance_treshold < 2 durante la etapa de recolección

--
factor_de_suficiencia = 0.3     -- Porcetaje ajustable según cuán estrictos queramos ser con la separación mínima de los datos en el método de Otsu. 
HIST_N_ADELANTE = 5             -- Cantidad de paquetes a mirar hacia adelante (incluyendo el actual)
HIST_M_PORCENTAJE = 0.8         -- Porcentaje mínimo que debe cumplir (0.8 = 80%)

-- 
CONFIG = {
    TOL_DIST                = distance_treshold/2,  -- (cm)
    TOL_ANG                 = angulo_treshold/2,      -- (°)
    MIN_OVERLAP_M1          = 1,                    -- Solape mínimo independiente para Greedy (M1)
    MIN_OVERLAP_M2          = 1,                    -- Solape mínimo independiente para Exhaustivo (M2)
    PRINT_RECURSIVE_PATHS   = true,                 -- Imprime los pasos de M2
    FORCE_M2_ALWAYS         = false,                -- Ejecutar M2 incluso si M1 fue perfecto
    MAX_ISLANDS_M2          = 9,                    -- Límite de islas para M2
    
    selection_mode          = 3,                    -- 1=Frecuente, 2=Larga, 3=Función de Ponderación (Score)
    
    W_FREQ                  = 1,                    -- Peso de la frecuencia
    W_LEN                   = -1,                    -- Positivo: Favorece Cortas. Negativo: Favorece Largas. 0: Ignorar.
    W_OV_QUAL               = 2,                    -- Peso de la calidad de solapes

    MATH_TOLERANCE          = 0.5,                  -- Tolerancia matemática M3
    
    
    -- 1. FILTRADO DE RUIDO (Tolerancia a la basura del sensor)
    RUIDO_BASE              = 0.2, -- (20%) Piso mínimo absoluto. Limpia los errores inevitables de cuantización del sensor, incluso en enjambres perfectos.
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
USE_DISTANCE_DEPENDENT = false -- El ruido aumenta si el objeto está lejos
USE_ANGLE_NOISE = true         -- Ruido en la medición del ángulo
USE_BIAS = true                -- Error sistemático (ej. sensor mal calibrado +2cm siempre)

-- Parámetros de Intensidad
NOISE_LEVEL_DIST = 0.01       -- % de error proporcional a la distancia
NOISE_LEVEL_ANGLE = 0.5       -- ±2 grados de desviación estándar
OUTLIER_PROB = 0.01           -- 1% de probabilidad de fallo catastrófico
BIAS_OFFSET = 2               -- Cm extra que agrega el sensor por defecto (si USE_BIAS es true)

-- Banderas de loggeo
DS_LOG_MODE  = 0 -- 0: Sin logs, 1: Logs detallados, 2: Solo resumen final, 3: Resumen + puntos en segmento, 4: Solo parámetros de Otsu
FVS_LOG_MODE = 0 -- 0: Sin logs, 1: Logs detallados, 2: Solo resumen final, 3: Resumen + detalles de filtrado

displayed_data   = 2 --1 Datos obtenidos por el observador. 2. Datos globales del movimiento de demostradores -- 3. Ambas

bigger = 1  -- Factor de escala para ajustar el tamaño de las figuras predefinidas (ajusta según tu espacio de simulación)
--Parametros de patrónes predefinidos
if robot.id == "Footbot1" or robot.id == "Footbot2" or robot.id == "Footbot3" or robot.id == "Footbot5" then
    figura = "Pentágono_irregular" -- "triangulo_regular" -- "octágono" -- "triangulo_irregular_1" --"Pentágono_irregular"  -- "Heptágono"-- "" -- "trayectoria_1" 
else
    figura = "triangulo_regular" -- "triangulo_regular" -- "octágono" -- "triangulo_irregular_1" --"Pentágono_irregular"  -- "Heptágono"-- "" -- "trayectoria_1" 
    --bigger = 1.65  -- Factor de escala para ajustar el tamaño de las figuras predefinidas (ajusta según tu espacio de simulación)
end



-------------------------------------------------------------------------------------------------------------------------------------
-------------------------------------------------------------------------------------------------------------------------------------
-------------------------------------------------------------------------------------------------------------------------------------
--                                                      MAIN FUNCTIONS                                                             --
-------------------------------------------------------------------------------------------------------------------------------------
-------------------------------------------------------------------------------------------------------------------------------------
-------------------------------------------------------------------------------------------------------------------------------------

-------------------------------------------------------------------------------------------------------------------------------------
-- Función que se ejecuta al inicio de la simulación
-------------------------------------------------------------------------------------------------------------------------------------

function init()

    --[[
    for key, _ in pairs(robot) do
    	  log("Dispositivo detectado: " .. key)
    end

    robot.colored_blob_omnidirectional_camera.enable()
    log("Robot " .. robot.id .. " iniciado.")

    -- Obtener el vector de orientacion inicial
    orientation = robot.positioning.orientation
    vec2_angle(orientation)
    -- Imprimir el ángulo
    --log("Angle: " .. ang)

    ]]
    
    robot.colored_blob_omnidirectional_camera.enable()
    determinar_rol()
end

-------------------------------------------------------------------------------------------------------------------------------------
-- Función principal que se ejecuta en cada tick
-------------------------------------------------------------------------------------------------------------------------------------

timer = 1
function step()

    if is_demonstrator == 1 then
        demostrador() -- Ejecutar si es demostrador
    else
        observador()
    end

    if timer == max_time then
        print("End_simulation")
    end

    timer = timer + 1
end


function Print_displayed_data(k,x,y)
    print( "Footbot"..k.." & " .. x .." , ".. y)
end
   
function Print_variance_data(k,var)
    --print( "Maestro"..k.." & " .. var)
end

function Print_otsu_data(separacion,avance)
    --print( "Otsu & " .. separacion.." , "..avance)
end


-- Función para actualizar la paciencia desde el DataBuffer
current_patience    = MAX_PATIENCE
function update_patience(restore_type, count)
    if restore_type == "full" then
        current_patience = MAX_PATIENCE
        log("[PACIENCIA] Restaurada completamente al máximo (" .. MAX_PATIENCE .. ").")
    elseif restore_type == "partial" then
        -- Restauración parcial con rendimiento decreciente
        local divisor = (count > 1) and (count - 1) or 1
        local restore_amount = math.floor(BASE_RESTORE / divisor)
        
        current_patience = math.min(MAX_PATIENCE, current_patience + restore_amount)
        log("[PACIENCIA] Restauración parcial: +" .. restore_amount .. ". Actual: " .. current_patience)
    end
end

-- Función que evalúa la paciencia en cada tick (Reemplaza el timer == 2000)
function check_patience_and_reconstruct()
    if Global_Reconstruction_Done then return end

    -- Disminuimos la paciencia en cada tick
    current_patience = current_patience - 1

    -- Si la paciencia se agota, ejecutamos la reconstrucción
    if current_patience <= 0 then
        log("\n\n\n\n PACIENCIA AGOTADA: Ejecutando Reconstrucción Global...")
        run_global_reconstruction()
        Global_Reconstruction_Done = true
    end
end

-------------------------------------------------------------------------------------------------------------------------------------
-- Funciones para cada rol
-------------------------------------------------------------------------------------------------------------------------------------

-- ============================================================================
-- VARIABLES GLOBALES PARA LOS PATRONES
-- ============================================================================
all_params = {}             -- Almacenará todos los patrones descubiertos
parametros_elegidos = {}    -- El patrón específico seleccionado para ejecutar


d1 = 2
c5 = 0
demostrador_avg_angle = 0
demostrador_avg_dist = 0
first_time_demonstrator = true
function demostrador()


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


c3 = 0
c4 = 0
first_time_observador = true
function observador()

    if c3 == 0 then

        if c4 == 0 then
            store_relative_positions()
        end
    end

end

-------------------------------------------------------------------------------------------------------------------------------------
-------------------------------------------------------------------------------------------------------------------------------------
-------------------------------------------------------------------------------------------------------------------------------------
--                                          Funciones de comportamiento de los epuck                                               --
-------------------------------------------------------------------------------------------------------------------------------------
-------------------------------------------------------------------------------------------------------------------------------------
-------------------------------------------------------------------------------------------------------------------------------------

-------------------------------------------------------------------------------------------------------------------------------------
-------------------------------------------------------------------------------------------------------------------------------------
-- Funciones del Demostrador
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

        -- if robot.id == "Footbot3" then 
        --     setleds("red")
        --     log("Zona de demostración detectada, "..robot.id)
        -- end
        --setleds("red")
    end
end

demostrador_avg_dist_1 = 0
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
        demostrador_avg_dist = 0

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

function print_positions()

    -- Obtener el vector de posicion inicial
    local posicion_x = robot.positioning.position.x
    local posicion_y = robot.positioning.position.y
    local posicion = { posicion_x, posicion_y }
    --log( robot.id.." & posicion: x=" .. posicion[1] .. ", y=" .. posicion[2])

    print( robot.id.." & " .. posicion[1] .." , ".. posicion[2])
end


PO_c1               = 0
ind                 = 1
move_result         = 0
movimiento_recorrido= 0
PO_counter_seg      = 0
PO_end              = false 
function Patron_Original(p)


    local params    = p.params
    local name      = p.name 
    local prox = proximidad(p)
    --log("prox[1]: "..prox[1]..", prox[2]: "..prox[2]..", "..robot.id)

    if prox[1] < 0.9 and  k == 0 then

        if ind <= #params and k == 0 then
            local s1 = params[ind][1]
            local s2 = params[ind][2]

            move(s1, s2, speed_dem, 10, false, robot.id)
            movimiento_recorrido = move_distance/s1

            -- if robot.id == "Footbot5" then
            --     log("\nPO_counter_seg = "..PO_counter_seg..", movimiento_recorrido = "..movimiento_recorrido)
            --     log("move_distance = "..move_distance..", s1 = "..s1)
            -- end

            figures()

            if c1 == 2 then
                PO_counter_seg = PO_counter_seg + 1
                if ind == #params then

                    --setleds("green")

                    -- log( " -------------------------------------------------------------------------" )
                    -- imprimir_posicion(posiciones_guardadas)
                    -- log(" ========= patrón original: "..name.." finalizado por: " .. robot.id.." ========= ")
                    -- log( " -------------------------------------------------------------------------" )
                    k = 1
                    ind = ind + 1
                    c1 = 0
                    PO_end = true   
                    PO_counter_seg = 0
                elseif ind < #params then
                    ind = ind + 1
                    c1 = 0

                    move_distance = 0
                    move_angle = 0
                end
            end
        end

    elseif prox[1] >= 0.9 and k == 0 then
        d1 = 3
        setleds("black")
        -- log( " -------------------------------------------------------------------------" )
        -- log("patrón original: "..name.." NO finalizado debido a obstauclos: " .. robot.id)
        -- log( " -------------------------------------------------------------------------" )
    end

    if k == 1 then
--        d1 = 3
        robot.wheels.set_velocity(0, 0)   -- Detenerse
        k = 2
    elseif k==2 then
        resetDEM()
    end
end


MT_c1 = 0
function Maestro_triste()
    MT_c1 = MT_c1 + 1

    if MT_c1 >= 5 then 
        RandomWalking()
        Temporizador_1(15)
        if temp== false then
            resetDEM()
        end
    elseif MT_c1 >= 2 then
        --setleds("black")
    else 
        setleds("black")
        robot.wheels.set_velocity(0, 0)   -- Detenerse
    end
end

-------------------------------------------------------------------------------------------------------------------------------------
-- función para realizar el patron imitado
-------------------------------------------------------------------------------------------------------------------------------------

k = 0
indice = 1
c5 = 0
PI_c1 = 0
PI_end = false
function patron_imitado(params_P)

    -- Obtener el vector de posicion inicial
    local x = robot.positioning.position.x
    local y = robot.positioning.position.y

    if robot.id == "Footbot4" or displayed_data == 2 then
        -- string.match busca dígitos (%d+) dentro del texto "Footbot4" y extrae el "4"
        -- tonumber() convierte ese "4" de texto a un número matemático real
        local id_numerico = tonumber(string.match(robot.id, "%d+"))
        
        if id_numerico ~= nil then
            Print_displayed_data(id_numerico, x, y)
        end
    end

    local params = params_P
    local prox = proximidad()

    if prox[1] <= 0.5 then
        PI_c1 = 1
    elseif prox[1] > 0.5 and  prox[2] >= -90 and  prox[2] <= 90 then 
        PI_c1 = 0
        k = 1
        setleds("black")
        log( " -------------------------------------------------------------------------" )
        log("patrón imitado: NO finalizado debido a obstaculo en el camino!!, proximidad del ".. proximity[1]*100 .."%, "..robot.id)
        log( " -------------------------------------------------------------------------" )
    else 
        PI_c1 = 1
    end

    if PI_c1 == 1 and k == 0 then

        if #params >= 1 then

            if indice <= #params then
                local valor1, valor2 = params[indice][1], params[indice][2]
                move(valor1, valor2, speed_dem, 10, false, robot.id)
                -- log("\nDistancia a recorrer en el segmento "..indice.." = ".. params[indice][1]..", Distancia actual: "..move_distance..", "..robot.id)
                -- log("Angulo del segmento = "..indice.." = ".. params[indice][2]..", Angulo actual: "..move_angle*180/math.pi ..", "..robot.id)
                --print_positions()

                figures()

                if c1 == 2 then
                    PO_counter_seg = PO_counter_seg + 1
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
            --is_demonstrator = 0

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

        if robot.id == "Footbot4" then 
            k = 1
        else
            k = 2
        end

    elseif k==2 then
        resetDEM()
        d1 = 2
        PO_counter_seg = 0
    end
end
-------------------------------------------------------------------------------------------------------------------------------------
-------------------------------------------------------------------------------------------------------------------------------------
--Funciones del observador
-------------------------------------------------------------------------------------------------------------------------------------
-------------------------------------------------------------------------------------------------------------------------------------

function Alumno_triste()
    RandomWalking()
    Temporizador_1(10)
    if temp== false then
        r_Temporizador_1()
        resetOBS()
    end
end

-- =========================================================================
-- GLOBALES DE MOVIMIENTO DEL OBSERVADOR
-- =========================================================================
motion_active = false
last_motion_marker = {0, 0}
srt_ang_mark_ant = 0
x_mark, y_mark = 0, 0
x_ant_r, y_ant_r = 0, 0

master_tracking = {}
srt_anterior_data = {}

current_wait_ticks = 0
obs_stop_flag = false

-- A. Generador de Distribución Normal (Algoritmo Box-Muller)
function gaussian(mean, variance)
    local u1 = math.random()
    local u2 = math.random()
    if u1 <= 0 then u1 = 0.000001 end
    local z0 = math.sqrt(-2.0 * math.log(u1)) * math.cos(2.0 * math.pi * u2)
    return z0 * variance + mean
end

-- B. Función Gestora de Ruido
function aplicar_ruido(dist_real, angulo_real)
    if not ENABLE_NOISE then return dist_real, angulo_real end

    if USE_OUTLIERS and math.random() < OUTLIER_PROB then
        return 1000.0, math.random(-180, 180) 
    end

    local dist_final = dist_real
    local angulo_final = angulo_real

    if USE_DISTANCE_DEPENDENT then
        local sigma = 0.5 + (dist_real * NOISE_LEVEL_DIST)
        local error = gaussian(0, sigma)
        dist_final = math.max(0, dist_final + error)
    else
        dist_final = dist_final + gaussian(0, 0.5) 
    end

    if USE_ANGLE_NOISE then
        local error_angular = gaussian(0, NOISE_LEVEL_ANGLE)
        angulo_final = angulo_final + error_angular
    end

    if USE_BIAS then dist_final = dist_final + BIAS_OFFSET end

    return dist_final, angulo_final
end

-- 1. INICIALIZACIÓN (Modificada para usar el offset global del observador)
function initialize_master_tracking(k, angle_deg, distance, ty)
    local angle_rad = angle_deg * (math.pi / 180)
    
    -- Aplicar el offset global actual del observador al nuevo maestro
    local current_move_angle = motion_active and move_angle or 0
    local obs_ang_offset = srt_ang_mark_ant + current_move_angle
    local ang_total = angle_rad + obs_ang_offset
    local x_rel, y_rel = polarACartesianoPAR(distance, ang_total)
    
    -- Guardamos el primer punto absoluto
    local x = x_rel + x_mark
    local y = y_rel + y_mark 

    master_tracking[k] = {
        raw_x = {x},
        raw_y = {y},
        raw_type = {ty},
        Pattern_start_detected = (ty == 1),
        End_of_pattern_detected = (ty == 3),
        processed = false,
        segmentos = { movement = {}, type = {}, valid = {} },
        params = {},
    }
end

-- Funciones Auxiliares (Intactas)
function process_full_trajectory(k, track)
    log("\n######################################################## ")
    log(" PROCESANDO MAESTRO "..k.." (Datos totales: "..#track.raw_x..")")
    log(" ######################################################## ")
    local raw_mov, raw_type = detect_seg(k,track.raw_x, track.raw_y, track.raw_type)
    local clean_mov, clean_type, clean_valid = filter_and_validate_segments(raw_mov, raw_type)

    track.segmentos.movement = clean_mov
    track.segmentos.type = clean_type
    track.segmentos.valid = clean_valid

    if #track.segmentos.type > 0 then
        local first_seg_types = track.segmentos.type[1]
        for _, t in ipairs(first_seg_types) do
            if t == 1 then track.Pattern_start_detected = true break end
        end
        local last_seg_types = track.segmentos.type[#track.segmentos.type]
        for _, t in ipairs(last_seg_types) do
            if t == 3 then track.End_of_pattern_detected = true break end
        end
    end

    if #track.segmentos.movement > 1 then 
        local equations, valid_cols = apply(track.segmentos.movement)
        local intersections = calcular_intersecciones_con_extremos(equations, track.segmentos.movement, valid_cols)
        local distancias, angulos = calcular_distancias_y_angulos(intersections)
        track.params = create_params(distancias, angulos, valid_cols)

        if #track.params > 0 then
            local structure_values = {}
            local structure_type = nil 
            for i, param in ipairs(track.params) do
                local dist = param[1]
                local ang = param[2]
                local col_id = param[3] 
                local is_segment_valid = track.segmentos.valid[col_id]

                if i > 1 then
                    table.insert(structure_values, ang)
                    if structure_type == nil then structure_type = 1 end
                end

                if is_segment_valid then
                    table.insert(structure_values, dist)
                    if structure_type == nil then structure_type = 0 end
                end
            end
            if #structure_values > 0 and structure_type ~= nil then
                local data_packet = { value = structure_values, type = structure_type }
                DataBuffer.add(data_packet)
                log(">> Datos enviados al DataBuffer. (Total values: "..#structure_values..")")
            end
        end
    else
        log("No hay segmentos suficientes para procesar tras la limpieza.")
    end
end

function process_master_segmentation(k, track, x_original, y_original, ty)
    table.insert(track.raw_x, x_original)
    table.insert(track.raw_y, y_original)
    table.insert(track.raw_type, ty)
    if ty == 1 then track.Pattern_start_detected = true end
    if ty == 3 then track.End_of_pattern_detected = true end
end

-- =========================================================================
-- BUCLE PRINCIPAL (Store Relative Positions)
-- =========================================================================
function store_relative_positions()

    -- Obtener el vector de posicion inicial
    local x = robot.positioning.position.x
    local y = robot.positioning.position.y
    
    Print_displayed_data(200, x, y)                

    check_patience_and_reconstruct()

    if Global_Reconstruction_Done then return end

    local proximity = proximidad()
    if proximity[1] < 1 then srt_c5 = 0 else
        if srt_c5 == 5 * ticks_per_second then resetOBS() return end
        srt_c5 = srt_c5 + 1
    end
    
    Temporizador_1(20) 
    if temp == false then robot.leds.set_all_colors("black") end
    
    if robot.colored_blob_omnidirectional_camera then                
        
        local blobs = robot.colored_blob_omnidirectional_camera
        local yellow_blobs = {} 
        local green_blobs = {}  
        local count = 0   

        -- 1. RECOLECCIÓN Y RUIDO
        for i, blob in ipairs(blobs) do
            local raw_angle = blob.angle * (180 / math.pi)
            local raw_dist = blob.distance
            local noisy_dist, noisy_angle = aplicar_ruido(raw_dist, raw_angle)
            local r, g, b = blob.color.red, blob.color.green, blob.color.blue
            
            if r > 200 and g > 200 and b < 100 then 
                table.insert(yellow_blobs, {angle = noisy_angle, distance = noisy_dist, type = 2})
            elseif r < 100 and g > 200 and b < 100 then 
                table.insert(green_blobs, {angle = noisy_angle, distance = noisy_dist})
            elseif  r > 200 and g < 100 and b < 100  then 
                table.insert(yellow_blobs, {angle = noisy_angle, distance = noisy_dist, type = 1})
            end 
        end

local updated_anterior_indices = {}
        local matched_new_indices = {}

        -- 2. ACTUALIZACIÓN DE MAESTROS EXISTENTES (Lógica de coincidencia corregida)
        for j, new_data in ipairs(yellow_blobs) do
            for k, anterior_data in pairs(srt_anterior_data) do
                if anterior_data.angle and not updated_anterior_indices[k] then
                    
                    -- Diferencia angular segura (Maneja el cruce por 0° y el salto en 180°)
                    local diff_ang = math.abs(new_data.angle - anterior_data.angle)
                    if diff_ang > 180 then diff_ang = 360 - diff_ang end
                    
                    if diff_ang <= angulo_treshold then
                        local diff_dist = math.abs(new_data.distance - anterior_data.distance)
                        
                        if diff_dist <= distance_treshold then
                            
                            --log(string.format("[MAESTRO %d] Ant -> Dist: %.2f, Ang: %.2f° || Nuevo -> Dist: %.2f, Ang: %.2f°", k, anterior_data.distance, anterior_data.angle, new_data.distance, new_data.angle))

                            -- ACTUALIZAR DATOS
                            srt_anterior_data[k].angle = new_data.angle
                            srt_anterior_data[k].distance = new_data.distance
                            srt_anterior_data[k].type = new_data.type 

                            updated_anterior_indices[k] = true
                            matched_new_indices[j] = true
                            count = count + 1
                            break -- Ya encontramos su maestro, dejamos de buscar para este blob
                        end 
                    end
                end 
            end 
        end 

        -- 3. CREAR NUEVOS MAESTROS
        for j, new_data in ipairs(yellow_blobs) do
            if not matched_new_indices[j] then
                table.insert(srt_anterior_data, {angle = new_data.angle, distance = new_data.distance, type = new_data.type or 2})
                local new_k = #srt_anterior_data
                updated_anterior_indices[new_k] = true
                initialize_master_tracking(new_k, new_data.angle, new_data.distance, srt_anterior_data[new_k].type)
            end
        end

        -- 4. PROCESAR BLOBS VERDES (FINAL DE PATRÓN)
        for _, green_blob in ipairs(green_blobs) do
            for k, master_data in pairs(srt_anterior_data) do
                if master_data.angle and not updated_anterior_indices[k] then 
                     
                     -- Misma corrección de diferencia angular para los verdes
                     local diff_ang = math.abs(green_blob.angle - master_data.angle)
                     if diff_ang > 180 then diff_ang = 360 - diff_ang end
                     
                     if diff_ang <= angulo_treshold then
                         srt_anterior_data[k].type = 3 
                         updated_anterior_indices[k] = true 
                         master_tracking[k].End_of_pattern_detected = true 
                     end
                end
            end 
        end

        -- =========================================================================
        -- 5. CÁLCULO DEL CENTROIDE Y DECISIÓN DE MOVIMIENTO (HÍBRIDO)
        -- =========================================================================
        local active_count = 0
        local sum_x, sum_y = 0, 0
        local closest_master_dist = 999999 -- Iniciamos con un valor gigante
        local angle_of_closest = 0

        for k, data in pairs(srt_anterior_data) do
            if updated_anterior_indices[k] and data.angle and data.distance then
                -- 1. Acumular para el Centroide
                local mx, my = polarACartesianoPAR(data.distance, data.angle * (math.pi/180))
                sum_x = sum_x + mx
                sum_y = sum_y + my
                active_count = active_count + 1

                -- 2. Rastrear al robot individual más cercano
                if data.distance < closest_master_dist then
                    closest_master_dist = data.distance
                    angle_of_closest = data.angle
                end
            end
        end

        if active_count > 0 then
            local centroid_x = sum_x / active_count
            local centroid_y = sum_y / active_count
            local centroid_dist, centroid_angle_rad = cartesianoAPolarPAR(centroid_x, centroid_y)
            local centroid_angle = centroid_angle_rad * (180 / math.pi)

            --log(string.format("\n\n[CENTROIDE] Activos: %d | Dist_Cent: %.2f | Dist_Mas_Cercano: %.2f \n", active_count, centroid_dist, closest_master_dist))

            -- LÓGICA DE MOVIMIENTO SEPARADA
            
            -- Caso A: Hay un robot DEMASIADO CERCA (Riesgo de colisión)
            if closest_master_dist < min_tracking_distance and not motion_active then
                motion_active = true
                local direction = 1
                if math.abs(angle_of_closest) < 135 then
                    direction = -1 -- Si el robot está adelante, retrocedemos
                else
                    direction = 1 -- Si el robot está atrás, avanzamos (o simplemente nos alejamos)
                end
                last_motion_marker = {direction * threshold * porcentaje, 0} 
                r_move()
                log(">>\n\n EVADIENDO: Robot demasiado cerca a " .. string.format("%.2f", closest_master_dist) .. "cm\n\n")

            -- Caso B: El grupo (centroide) se está alejando mucho (Riesgo de perderlos)
            elseif centroid_dist > max_tracking_distance and not motion_active then
                motion_active = true
                -- Perseguimos al "centro de masa" del grupo
                last_motion_marker = {centroid_dist * porcentaje, centroid_angle}
                r_move()
                log(">> \n\nPERSIGUIENDO: Centroide alejado a " .. string.format("%.2f", centroid_dist) .. "cm\n\n")
            end
        end
        -- =========================================================================
        -- 6. GESTIÓN DEL MOVIMIENTO DEL OBSERVADOR
        -- =========================================================================
        if motion_active then
            
            -- 6.1 ANÁLISIS DE OBSTÁCULOS FRONTALES
            local prox_data = proximidad() -- [1] Distancia normalizada, [2] Ángulo
            obs_stop_flag = false

            -- Si detecta algo MUY cerca (prox > 0 y prox < 0.9) Y está en el cono frontal ([-45°, 45°])
            if prox_data[1] > 0 and prox_data[1] < 0.9 and math.abs(prox_data[2]) <= 45 then
                obs_stop_flag = true
            -- También paramos si hay contacto físico duro (prox == 1) en el frente
            elseif prox_data[1] == 1 and math.abs(prox_data[2]) <= 45 then
                obs_stop_flag = true
            end

            -- 6.2 GESTIÓN DEL TEMPORIZADOR DE PAUSA
            if obs_stop_flag then
                current_wait_ticks = current_wait_ticks + 1
                --log("[PAUSA] Esperando a que el obstáculo se quite... Tick: " .. current_wait_ticks)
                
                if current_wait_ticks > MAX_WAIT_TICKS then
                    log("===============================================================")
                    log("[ALERTA] ¡Tiempo de espera agotado ("..MAX_WAIT_TICKS.." ticks)! Obstáculo bloqueando.")
                    log("===============================================================")
                    -- Aquí, en el futuro, pondremos la lógica para abortar o esquivar
                end
            else
                -- Si el camino está libre, reseteamos el contador de espera
                current_wait_ticks = 0
            end

            -- 6.3 EJECUCIÓN DEL MOVIMIENTO (Pasando el parámetro 'stop')
            -- Asumo la firma: move(dist, ang, vel, margin?, param5?, id?, stop)
            move(last_motion_marker[1], last_motion_marker[2], d_speed, 80, false, robot.id, obs_stop_flag)

            -- 6.4 ACTUALIZACIÓN DEL MARCO DE REFERENCIA (Solo cambia si 'move' realmente avanza)
            local ang_total_1 = move_angle + srt_ang_mark_ant
            local x_new_r, y_new_r = polarACartesianoPAR(move_distance, ang_total_1)
            x_mark = x_new_r + x_ant_r
            y_mark = y_new_r + y_ant_r
            local r, theta = cartesianoAPolarPAR(x_mark, y_mark)

            -- log("Posición: x_new_r: "..x_new_r..", y_new_r: "..y_new_r)
            -- log("Posición: x_ant_r: "..x_ant_r..", y_ant_r: "..y_ant_r)
            -- --log(" ============================================================= ")
            -- log("Posición: x_mark: "..x_mark..", y_mark: "..y_mark .." --> ( "..r.." , "..theta *180/math.pi .." )" )

            if c1 == 2 then -- Finalizó la secuencia de giro y desplazamiento
                srt_ang_mark_ant = move_angle + srt_ang_mark_ant
                x_ant_r = x_mark
                y_ant_r = y_mark
                motion_active = false
                current_wait_ticks = 0 -- Por seguridad, limpiamos al terminar
                c1 = 0
                c2 = 0
                robot.wheels.set_velocity(0, 0)
            end
        else
            -- Si está quieto de forma natural, el marco se mantiene estático 
            x_mark = x_ant_r
            y_mark = y_ant_r
            current_wait_ticks = 0 -- Nos aseguramos de no acumular "esperas" falsas
        end

        -- =========================================================================
        -- 7. GESTIÓN DE MAESTROS PERDIDOS O FINALIZADOS 
        -- =========================================================================
        for k = 1, #srt_anterior_data do
            local track = master_tracking[k]
            local is_lost = (srt_anterior_data[k] and srt_anterior_data[k].angle and not updated_anterior_indices[k])
            local is_finished = track and track.End_of_pattern_detected
            
            if (is_lost or is_finished) and track and not track.processed then
                process_full_trajectory(k, track)
                track.processed = true 
                
                srt_anterior_data[k].angle = nil
                srt_anterior_data[k].distance = nil
                srt_anterior_data[k].type = nil
            end
        end

        -- =========================================================================
        -- 8. GRABACIÓN DE DATOS CORREGIDOS (Para maestros activos)
        -- =========================================================================
        -- El offset angular depende de si nos estamos moviendo AHORA mismo
        local current_move_angle = motion_active and move_angle or 0
        local obs_ang_offset = srt_ang_mark_ant + current_move_angle

        for k, _ in pairs(updated_anterior_indices) do
            local master_data = srt_anterior_data[k]
            local track = master_tracking[k]

            if master_data and track and not track.processed then
                local current_distance = master_data.distance
                local current_angle_rad = master_data.angle * (math.pi / 180)
                local ty = master_data.type
                
                -- Se aplica la matriz de transformación del observador a la lectura local del blob
                local ang_total = current_angle_rad + obs_ang_offset
                local x_new, y_new = polarACartesianoPAR(current_distance, ang_total)
                local x_original = x_new + x_mark
                local y_original = y_new + y_mark

                -- ====== LOG DE COMPENSACIÓN DEFINITIVO ======
                -- local ang_total_deg = ang_total * (180 / math.pi)
                -- local current_move_deg = current_move_angle * (180 / math.pi)
                -- log(string.format("\n[COMPENSACIÓN Maestro %d] Cam Crudo: %.1f° | Obs Giró: %.1f° || ANG GLOBAL GUARDADO: %.1f°", k, master_data.angle, current_move_deg, ang_total_deg))
                -- log(string.format("x_original: %.2f, y_original: %.2f", x_original, y_original))
                -- log(" ------------------------- ")

                -- ============================================     
                
                --log(string.format("[MAESTRO %d] Ant -> Dist: %.2f, Ang: %.2f° || Nuevo -> Dist: %.2f, Ang: %.2f°", k, anterior_data.distance, anterior_data.angle, new_data.distance, new_data.angle))
                
                if displayed_data == 1 then
                    local x,y = x_original/100, y_original/100
                    Print_displayed_data(k, x, y)                
                end 

                process_master_segmentation(k, track, x_original, y_original, ty)
            end
        end
    end 
end

-------------------------------------------------------------------------------------------------------------------------------------
-- FUNCIONES MATEMÁTICAS Y ESTADÍSTICAS
-------------------------------------------------------------------------------------------------------------------------------------

function get_window_stats(start_idx, win_size, xx, yy)
    local sum_x, sum_y = 0, 0
    local n = 0
    
    -- 1. Calcular Centroide (Promedio)
    for j = start_idx, start_idx + win_size - 1 do
        if xx[j] and yy[j] then
            sum_x = sum_x + xx[j]
            sum_y = sum_y + yy[j]
            n = n + 1
        end
    end
    
    if n == 0 then return 0, 0, 0 end
    local avg_x = sum_x / n
    local avg_y = sum_y / n
    
    -- 2. Calcular Dispersión (Desviación Estándar combinada)
    local sum_sq_dist = 0
    for j = start_idx, start_idx + win_size - 1 do
        if xx[j] and yy[j] then
            local dx = xx[j] - avg_x
            local dy = yy[j] - avg_y
            sum_sq_dist = sum_sq_dist + (dx*dx + dy*dy)
        end
    end
    
    -- Retornamos: Desviación Estándar, Centroide X, Centroide Y
    return math.sqrt(sum_sq_dist / n), avg_x, avg_y
end

-- NUEVO: Función para encontrar el umbral óptimo usando el Método de Otsu 1D
function calcular_otsu_1d(varianzas)
    if #varianzas == 0 then return 0, 0 end
    
    -- Encontrar min y max para crear el histograma
    local min_v, max_v = varianzas[1], varianzas[1]
    for i = 1, #varianzas do
        if varianzas[i] < min_v then min_v = varianzas[i] end
        if varianzas[i] > max_v then max_v = varianzas[i] end
    end
    
    if min_v == max_v then return min_v, 0 end
    
    -- Crear histograma aproximado (50 bins)
    local num_bins = 50
    local hist = {}
    for i = 1, num_bins do hist[i] = 0 end
    
    local bin_size = (max_v - min_v) / num_bins
    for i = 1, #varianzas do
        local bin = math.floor((varianzas[i] - min_v) / bin_size) + 1
        if bin > num_bins then bin = num_bins end
        hist[bin] = hist[bin] + 1
    end
    
    local total_datos = #varianzas
    local sum_total = 0
    for i = 1, num_bins do
        local bin_val = min_v + (i - 0.5) * bin_size
        sum_total = sum_total + (bin_val * hist[i])
    end
    
    local weight_bg = 0
    local sum_bg = 0
    local max_varianza_interclase = -1
    local umbral_optimo = 0
    
    -- Evaluar cada posible umbral (Otsu)
    for i = 1, num_bins do
        weight_bg = weight_bg + hist[i]
        
        if weight_bg > 0 then
            local weight_fg = total_datos - weight_bg
            if weight_fg == 0 then break end
            
            local bin_val = min_v + (i - 0.5) * bin_size
            sum_bg = sum_bg + (bin_val * hist[i])
            
            local mean_bg = sum_bg / weight_bg
            local mean_fg = (sum_total - sum_bg) / weight_fg
            
            -- Varianza Inter-clase
            local var_between = weight_bg * weight_fg * (mean_bg - mean_fg) * (mean_bg - mean_fg)
            
            if var_between > max_varianza_interclase then
                max_varianza_interclase = var_between
                umbral_optimo = bin_val
            end
        end
    end
    
    return umbral_optimo, max_varianza_interclase
end

-- ============================================================================= 
-- CONFIGURACIÓN GLOBAL Y LOGS
-- =============================================================================
mostrar_todos_los_puntos = false 

function imprimir_resumen_segmentos(titulo, lista_segmentos, lista_tipos, lista_validos)
    log("\n\n= ====================================================== ")
    log(" " .. titulo)
    log(" ======================================================= ")
    
    if #lista_segmentos == 0 then
        log(" >> No hay segmentos para mostrar.")
    end

    for idx, points in ipairs(lista_segmentos) do
        local num_puntos = #points
        local types_list = lista_tipos[idx]
        
        local etiqueta_validez = ""
        if lista_validos and lista_validos[idx] ~= nil then
            if lista_validos[idx] then etiqueta_validez = " [VALIDO]" else etiqueta_validez = " [INVALIDO]" end
        end

        if num_puntos > 0 then
            local start_pt = points[1]
            local end_pt = points[num_puntos]
            local start_type = (types_list and types_list[1]) or -1
            local end_type = (types_list and types_list[num_puntos]) or -1

            log(string.format("SEGMENTO #%d%s", idx, etiqueta_validez))
            log(string.format(" > Total Puntos: %d", num_puntos))
            log(string.format(" > Inicio: (%.3f, %.3f) [Type: %d]", start_pt[1], start_pt[2], start_type))
            log(string.format(" > Fin:    (%.3f, %.3f) [Type: %d]", end_pt[1], end_pt[2], end_type))
            
            if mostrar_todos_los_puntos then
                local buffer_linea = " > TRAYECTORIA: "
                for p_idx, p in ipairs(points) do
                    buffer_linea = buffer_linea .. string.format("(%.2f, %.2f) ", p[1], p[2])
                    if p_idx % 5 == 0 and p_idx < num_puntos then
                        log(buffer_linea)
                        buffer_linea = "                "
                    end
                end
                log(buffer_linea)
            end
            log("--------------------------------------------------------")
        end
    end
    log(" ======================================================= \n")
end

DS_avance   = 1
Ds_max_av   = 30

dato1    = 0
DS_FIRST = true
-- =============================================================================
-- FUNCIÓN 1: DETECCIÓN DE SEGMENTOS (AUTOMATIZADA Y DINÁMICA)
-- =============================================================================
function detect_seg(s, xx, yy, ttypes) 
    local total_puntos = #xx
    if total_puntos < 3 then return {}, {} end -- Muy pocos datos para procesar

    if DS_LOG_MODE== 1 or DS_LOG_MODE== 4 then
        log("\n>>> INICIANDO FASE 1: EXPLORACIÓN AUTOMÁTICA DE PARÁMETROS")
    end

    -- --- FASE 1: BÚSQUEDA DEL AVANCE ÓPTIMO (OTSU CON SUFICIENCIA) ---
    local max_avance_permitido = math.floor(total_puntos / 3)
    if max_avance_permitido > Ds_max_av then max_avance_permitido = Ds_max_av end
    if max_avance_permitido < DS_avance then max_avance_permitido = DS_avance end

    local resultados_exploracion = {}
    local max_separacion_global = -1

    -- 1. Recopilamos todos los resultados
    for av = DS_avance, max_avance_permitido, DS_avance do
        local temp_vars, temp_cx, temp_cy = {}, {}, {}
        
        for i = 1, total_puntos - av do
            local std_dev, cen_x, cen_y = get_window_stats(i, av, xx, yy)
            table.insert(temp_vars, std_dev)
            table.insert(temp_cx, cen_x)
            table.insert(temp_cy, cen_y)
        end
        
        local umbral, separacion = calcular_otsu_1d(temp_vars)
        
        if DS_LOG_MODE== 1 or DS_LOG_MODE== 4 then
            log(string.format(" Probando Avance=%d | Umbral=%.3f | Separación Otsu=%.3f", av, umbral, separacion))
        end

        table.insert(resultados_exploracion, {
            av = av,
            umbral = umbral,
            separacion = separacion,
            vars = temp_vars,
            cx = temp_cx,
            cy = temp_cy
        })

        if separacion > max_separacion_global then
            max_separacion_global = separacion
        end

        if av == 30 then 
            dato1 = separacion
        end
    end

    -- 2. Seleccionamos el avance más pequeño "suficientemente bueno"
    -- Usamos el factor_de_suficiencia de la separación máxima como criterio de suficiencia
    local umbral_aceptacion = max_separacion_global * factor_de_suficiencia
    local mejor_avance = 5
    local mejor_umbral = 0
    local mejores_varianzas = {}
    local mejores_centroides_x = {}
    local mejores_centroides_y = {}

    for _, res in ipairs(resultados_exploracion) do
        if res.separacion >= umbral_aceptacion then
            mejor_avance = res.av
            mejor_umbral = res.umbral
            mejores_varianzas = res.vars
            mejores_centroides_x = res.cx
            mejores_centroides_y = res.cy
            
            if DS_LOG_MODE== 1 or DS_LOG_MODE== 4 then
                log(string.format(" -> SELECCIONADO: Avance=%d (%.3f supera el umbral del %d%%: %.3f)", 
                    res.av, res.separacion, factor_de_suficiencia*100, umbral_aceptacion))
            end
            break -- Detenemos la búsqueda al encontrar el primero que cumple
        end
    end

    if DS_LOG_MODE== 1 or DS_LOG_MODE== 4 then
        log("\n>>> FASE 2: CONFIGURACIÓN GANADORA")
        log(string.format(" GANADOR -> Avance: %d | Umbral Dinámico: %.3f", mejor_avance, mejor_umbral))


        log("\n>>> FASE 3: SEGMENTACIÓN CON HISTÉRESIS (LOOK-AHEAD)")
    end

    if  DS_FIRST then 
        Print_otsu_data(dato1, mejor_avance) 
        DS_FIRST = false
    end

    -- --- FASE 3: SEGMENTACIÓN BASADA EN VALLES ---
    local segmentos = {}      
    local segmentos_type = {} 
    local k = 1       
    segmentos[k] = {}  
    segmentos_type[k] = {} 

    local ESTADO_MOVIMIENTO = 1
    local ESTADO_BUSCANDO_VALLE = 2
    local estado_actual = ESTADO_MOVIMIENTO

    local valle_min_var = 999999
    local valle_best_idx = -1
    local valle_cen_x, valle_cen_y = 0, 0
    
    local packet_counter = 1 

    for i = 1, #mejores_varianzas do
        local std_dev = mejores_varianzas[i]
        local x, y = xx[i], yy[i]
        local ty = ttypes[i]
        local cx, cy = mejores_centroides_x[i], mejores_centroides_y[i]

        -- LOG RAW SOLICITADO
        if DS_LOG_MODE== 1 or DS_LOG_MODE== 3 then
            local end_idx = i + mejor_avance - 1 
            local end_x, end_y = xx[end_idx], yy[end_idx] 
            log(string.format("Pkt %d | Idx[%d-%d] Pos(%.2f, %.2f)->(%.2f, %.2f) | Var=%.4f", 
                packet_counter, i, end_idx, x, y, end_x, end_y, std_dev))
        end
        
        Print_variance_data(s, std_dev)
        packet_counter = packet_counter + 1

        if estado_actual == ESTADO_MOVIMIENTO then
            if std_dev > mejor_umbral then
                table.insert(segmentos[k], { x, y }) 
                table.insert(segmentos_type[k], ty)
            else
                -- [!] VALIDACIÓN LOOK-AHEAD PARA ENTRAR AL VALLE
                local count_bajo_umbral = 0
                local limite_busqueda = math.min(i + HIST_N_ADELANTE - 1, #mejores_varianzas)
                local puntos_a_revisar = limite_busqueda - i + 1

                for j = i, limite_busqueda do
                    if mejores_varianzas[j] <= mejor_umbral then
                        count_bajo_umbral = count_bajo_umbral + 1
                    end
                end

                local porcentaje_real = count_bajo_umbral / puntos_a_revisar

                if porcentaje_real >= HIST_M_PORCENTAJE then
                    -- Sí se mantiene la caída. Entramos a buscar el valle.
                    estado_actual = ESTADO_BUSCANDO_VALLE
                    valle_min_var = std_dev
                    valle_best_idx = i
                    valle_cen_x, valle_cen_y = cx, cy
                    
                    if DS_LOG_MODE == 1 or DS_LOG_MODE == 3 then
                        log(string.format("  -> [VALIDADO] Caída real detectada. %.0f%% de los prox %d pkts bajo umbral.", porcentaje_real*100, puntos_a_revisar))
                    end
                else
                    -- No se mantiene. Es un caso aislado, lo tratamos como movimiento.
                    table.insert(segmentos[k], { x, y }) 
                    table.insert(segmentos_type[k], ty)
                    
                    if DS_LOG_MODE == 1 or DS_LOG_MODE == 3 then
                        log(string.format("  -> [DESCARTADO] Falso valle en Idx: %d. Solo %.0f%% bajo umbral. Ignorando.", i, porcentaje_real*100))
                    end
                end
            end

        elseif estado_actual == ESTADO_BUSCANDO_VALLE then
            if std_dev <= mejor_umbral then
                -- Seguimos en el valle de forma natural
                if std_dev < valle_min_var then
                    valle_min_var = std_dev
                    valle_best_idx = i
                    valle_cen_x, valle_cen_y = cx, cy
                end
            else
                -- [!] VALIDACIÓN LOOK-AHEAD PARA SALIR DEL VALLE
                local count_sobre_umbral = 0
                local limite_busqueda = math.min(i + HIST_N_ADELANTE - 1, #mejores_varianzas)
                local puntos_a_revisar = limite_busqueda - i + 1

                for j = i, limite_busqueda do
                    if mejores_varianzas[j] > mejor_umbral then
                        count_sobre_umbral = count_sobre_umbral + 1
                    end
                end

                local porcentaje_real = count_sobre_umbral / puntos_a_revisar

                if porcentaje_real >= HIST_M_PORCENTAJE then
                    -- Sí se mantiene la subida de varianza. Realmente salió del valle.
                    local pivot_point = { valle_cen_x, valle_cen_y }
                    local pivot_type = ttypes[valle_best_idx]

                    table.insert(segmentos[k], pivot_point)
                    table.insert(segmentos_type[k], pivot_type)

                    if DS_LOG_MODE== 1 or DS_LOG_MODE== 3 then
                        log(string.format("  -> [VALIDADO] Salida real del valle. %.0f%% de los prox %d pkts sobre umbral.", porcentaje_real*100, puntos_a_revisar))
                        log(string.format("--> [!] AQUÍ SE DIVIDIÓ UN SEGMENTO (Pivote guardado en Idx: %d con Var: %.4f)", valle_best_idx, valle_min_var))
                        log("----------------------------------------------------------------------")
                    end

                    k = k + 1
                    segmentos[k] = {}
                    segmentos_type[k] = {}
                    table.insert(segmentos[k], pivot_point)
                    table.insert(segmentos_type[k], pivot_type)

                    table.insert(segmentos[k], { x, y }) 
                    table.insert(segmentos_type[k], ty)
                    
                    estado_actual = ESTADO_MOVIMIENTO
                else
                    -- No se mantiene la subida. Es un pico aislado de ruido.
                    -- No hacemos nada, permitimos que siga buscando en iteraciones futuras.
                    if DS_LOG_MODE == 1 or DS_LOG_MODE == 3 then
                        log(string.format("  -> [DESCARTADO] Falso movimiento en Idx: %d. Solo %.0f%% sobre umbral. Seguimos en valle.", i, porcentaje_real*100))
                    end
                end
            end
        end
    end 
     
    -- =========================================================================
    -- FORZAR CORTE SI LOS DATOS TERMINARON MIENTRAS EL ROBOT ESTABA QUIETO
    -- =========================================================================
    if estado_actual == ESTADO_BUSCANDO_VALLE then
        local pivot_point = { valle_cen_x, valle_cen_y }
        local pivot_type = ttypes[valle_best_idx]

        table.insert(segmentos[k], pivot_point)
        table.insert(segmentos_type[k], pivot_type)

        if DS_LOG_MODE== 1 or DS_LOG_MODE== 3 then
            log(string.format("--> [!] CORTE DE FIN DE RUTA (Pivote guardado en Idx: %d con Var: %.4f)", valle_best_idx, valle_min_var))
            log("----------------------------------------------------------------------")
        end

        k = k + 1
        segmentos[k] = {}
        segmentos_type[k] = {}
        table.insert(segmentos[k], pivot_point)
        table.insert(segmentos_type[k], pivot_type)
    end
    -- =========================================================================

    for i = #mejores_varianzas + 1, total_puntos do 
        table.insert(segmentos[k], { xx[i], yy[i] }) 
        table.insert(segmentos_type[k], ttypes[i]) 
    end 

    if DS_LOG_MODE== 1 or DS_LOG_MODE== 3 or DS_LOG_MODE== 2 then
        imprimir_resumen_segmentos("RESUMEN RAW (DETECT SEG)", segmentos, segmentos_type, nil)
    end

    return segmentos, segmentos_type 
end

-- =============================================================================
-- HELPER PARA CALCULAR DISTANCIA Y FUNCIÓN DE FILTRADO (INTACTOS)
-- =============================================================================
function get_segment_dist(seg)
    if not seg or #seg < 1 then return 0 end
    local p1 = seg[1]
    local p2 = seg[#seg]
    local dx = p2[1] - p1[1]
    local dy = p2[2] - p1[2]
    return math.sqrt(dx*dx + dy*dy)
end

function filter_and_validate_segments(raw_seg_mov, raw_seg_type)
    -- ... (Tu código original de filtrado, sin modificaciones)
    local n = #raw_seg_mov
    if n == 0 then return {}, {}, {} end

    if FVS_LOG_MODE == 1 then
        log("\n>>> INICIANDO PROCESO DE FILTRADO Y FUSIÓN")
    end

    local raw_start_is_static = false
    local raw_end_is_static = false

    if get_segment_dist(raw_seg_mov[1]) < distance_treshold then raw_start_is_static = true end
    if get_segment_dist(raw_seg_mov[n]) < distance_treshold then raw_end_is_static = true end

    local final_mov = {}
    local final_type = {}
    local final_valid = {} 

    local accum_x, accum_y, accum_count = 0, 0, 0
    
    for i = 1, n do
        local seg = raw_seg_mov[i]
        local types = raw_seg_type[i]
        local dist = get_segment_dist(seg)

        if dist < distance_treshold then
            if FVS_LOG_MODE == 1 or FVS_LOG_MODE == 3 then log(string.format("[X] ELIMINADO Seg Original #%d | Dist: %.3f < Tol: %.3f", i, dist, distance_treshold)) end
            local p_start = seg[1]
            local p_end = seg[#seg]
            accum_x = accum_x + p_start[1] + p_end[1]
            accum_y = accum_y + p_start[2] + p_end[2]
            accum_count = accum_count + 2
        else
            if accum_count > 0 then
                local avg_x = accum_x / accum_count
                local avg_y = accum_y / accum_count
                seg[1] = { avg_x, avg_y } 
                if FVS_LOG_MODE == 1 or FVS_LOG_MODE == 3 then log(string.format(" -> FUSION: Nuevo inicio ajustado a promedio (%.3f, %.3f)", avg_x, avg_y)) end
                accum_x, accum_y, accum_count = 0, 0, 0
            end
            table.insert(final_mov, seg)
            table.insert(final_type, types)
            table.insert(final_valid, true) 
        end
    end

    if accum_count > 0 and (FVS_LOG_MODE == 1 or FVS_LOG_MODE == 3) then log("[!] Los últimos segmentos eran ruido y han sido descartados.") end

    local final_n = #final_mov
    if final_n > 0 then
        if not raw_start_is_static then
            final_valid[1] = false
            if FVS_LOG_MODE == 1 or FVS_LOG_MODE == 3 then log(" -> VALIDACIÓN: Seg #1 marcado INVALIDO (No hubo espera estática al inicio)") end
        end
        if not raw_end_is_static then
            final_valid[final_n] = false
            if FVS_LOG_MODE == 1 or FVS_LOG_MODE == 3 then log(string.format(" -> VALIDACIÓN: Seg #%d marcado INVALIDO (No hubo espera estática al final)", final_n)) end
        end
    end

    if FVS_LOG_MODE == 1 or FVS_LOG_MODE == 2 or FVS_LOG_MODE == 3 then imprimir_resumen_segmentos("RESUMEN FILTRADO (FINAL)", final_mov, final_type, final_valid) end

    return final_mov, final_type, final_valid
end

-- Variables globales 
equations = {}
distancias_a_recorrer = {} -- Tabla para almacenar las distancias a recorrer

-- ============================================================================
-- 1. APPLY: Calcula regresiones ortogonales dinámicas (Normal o Invertida)
-- ============================================================================
function apply(segmentos)
    equations = {}
    local valid_cols = {} 

    local ordered_keys = {}
    for k in pairs(segmentos) do
        table.insert(ordered_keys, k)
    end
    table.sort(ordered_keys)

    for _, col in ipairs(ordered_keys) do
        local points = segmentos[col]

        if #points > 3 then
            local sum_x, sum_y = 0, 0
            local n = #points

            -- 1. Encontrar el centroide (promedio) de los puntos
            for _, point in ipairs(points) do
                sum_x = sum_x + point[1]
                sum_y = sum_y + point[2]
            end
            local mean_x = sum_x / n
            local mean_y = sum_y / n

            -- 2. Calcular los términos de la matriz de covarianza
            local s_xx, s_yy, s_xy = 0, 0, 0
            for _, point in ipairs(points) do
                local dx = point[1] - mean_x
                local dy = point[2] - mean_y
                s_xx = s_xx + (dx * dx)
                s_yy = s_yy + (dy * dy)
                s_xy = s_xy + (dx * dy)
            end

            local m, b
            local is_vertical = false

            -- 3. Decidir la orientación óptima comparando las varianzas
            if s_yy > s_xx then
                -- Eje Y tiene mayor dispersión -> Invertimos (x = my + b)
                is_vertical = true
                if s_yy == 0 then m = 0 else m = s_xy / s_yy end
                b = mean_x - (m * mean_y)
                
                --log("Robot " .. robot.id .. string.format(", Columna %d: INVERTIDA -> x = %.4fy + %.4f", col, m, b))
            else
                -- Eje X tiene mayor dispersión -> Normal (y = mx + b)
                is_vertical = false
                if s_xx == 0 then m = 0 else m = s_xy / s_xx end
                b = mean_y - (m * mean_x)
                
                --log("Robot " .. robot.id .. string.format(", Columna %d: NORMAL -> y = %.4fx + %.4f", col, m, b))
            end

            -- Guardamos como tabla clave-valor para la función de intersecciones
            equations[col] = {m = m, b = b, is_vertical = is_vertical}
            table.insert(valid_cols, col) 
        else
           -- log("Robot " .. robot.id .. string.format(", Columna %d: Ignorada por insuficiencia de puntos (%d puntos)", col, #points))
        end
    end
    
    return equations, valid_cols
end

-- ============================================================================
-- 2. CALCULAR INTERSECCIONES: Cruza las rectas calculadas
-- ============================================================================
function calcular_intersecciones_con_extremos(equations, segmentos, valid_cols)
    local intersections = {} 

    -- Si no hay ecuaciones, no hay intersecciones que calcular
    if #valid_cols == 0 then
        if resetOBS then resetOBS() end 
        return intersections
    end

    local first_key = valid_cols[1]
    local first_eq = equations[first_key]
    local first_segment = segmentos[first_key]

    if type(first_eq) ~= "table" or type(first_segment) ~= "table" or #first_segment == 0 then
        if resetalumno then resetalumno() end
        return intersections
    end

    -- ====================================================
    -- A. Calcular Punto Inicial
    -- ====================================================
    local x_inicio, y_inicio
    if first_eq.is_vertical then
        -- Recta invertida: x = my + b. Usamos la Y del punto para hallar X.
        y_inicio = first_segment[1][2]
        x_inicio = first_eq.m * y_inicio + first_eq.b
    else
        -- Recta normal: y = mx + b. Usamos la X del punto para hallar Y.
        x_inicio = first_segment[1][1]
        y_inicio = first_eq.m * x_inicio + first_eq.b
    end
    table.insert(intersections, {x_inicio, y_inicio})

    -- ====================================================
    -- B. Calcular Intersecciones entre segmentos
    -- ====================================================
    for i = 1, #valid_cols - 1 do
        local eq1 = equations[valid_cols[i]]
        local eq2 = equations[valid_cols[i + 1]]
        local x_int, y_int

        if not eq1.is_vertical and not eq2.is_vertical then
            -- Caso 1: Ambas Normales (y = mx + b)
            local den = eq1.m - eq2.m
            if math.abs(den) > 1e-6 then
                x_int = (eq2.b - eq1.b) / den
                y_int = eq1.m * x_int + eq1.b
            end

        elseif eq1.is_vertical and eq2.is_vertical then
            -- Caso 2: Ambas Invertidas (x = my + b)
            local den = eq1.m - eq2.m
            if math.abs(den) > 1e-6 then
                y_int = (eq2.b - eq1.b) / den
                x_int = eq1.m * y_int + eq1.b
            end

        else
            -- Caso 3: Mixtas (Una Normal y Una Invertida)
            local eq_norm = eq1.is_vertical and eq2 or eq1
            local eq_vert = eq1.is_vertical and eq1 or eq2
            
            -- Sustitución: x = mv * (mn * x + bn) + bv
            local den = 1 - (eq_vert.m * eq_norm.m)
            if math.abs(den) > 1e-6 then
                x_int = (eq_vert.m * eq_norm.b + eq_vert.b) / den
                y_int = eq_norm.m * x_int + eq_norm.b
            end
        end

        -- Solo insertamos si las rectas no eran perfectamente paralelas
        if x_int and y_int then
            table.insert(intersections, {x_int, y_int})           
        end
    end

    -- ====================================================
    -- C. Calcular Punto Final
    -- ====================================================
    local last_key = valid_cols[#valid_cols]
    local last_eq = equations[last_key]
    local last_segment = segmentos[last_key]

    local x_final, y_final
    if last_eq.is_vertical then
        y_final = last_segment[#last_segment][2]
        x_final = last_eq.m * y_final + last_eq.b
    else
        x_final = last_segment[#last_segment][1]
        y_final = last_eq.m * x_final + last_eq.b
    end
    table.insert(intersections, {x_final, y_final}) 

    -- ====================================================
    -- LOG DE INTERSECCIONES
    -- ====================================================
    -- log("--- LOG: Intersecciones Calculadas ---")
    -- log(string.format("Total de puntos: %d", #intersections))
    -- for idx, pt in ipairs(intersections) do
    --     log(string.format("  Punto %d -> X: %.4f, Y: %.4f", idx, pt[1], pt[2]))
    -- end
    -- log("----------------------------------------")

    return intersections
end

-- ============================================================================
-- 3. CALCULAR DISTANCIAS Y ÁNGULOS
-- ============================================================================
angles_a_recorrer = {}
function calcular_distancias_y_angulos(intersecciones)
    angles_a_recorrer = {}
    distancias_a_recorrer = {} 
    
    -- Prevención: Se necesitan al menos 2 puntos para calcular un recorrido
    if #intersecciones < 2 then return distancias_a_recorrer, angles_a_recorrer end

    -- 1) Primer punto
    local x1, y1 = intersecciones[1][1], intersecciones[1][2]

    -- 2) Orientación inicial DEL OBSERVADO (su eje +X)
    local prev_angle = math.atan2(-y1, -x1) * (180/math.pi)

    -- 3) Recorrer todos los segmentos, incluyendo el 1→2
    for i = 1, #intersecciones - 1 do
        local xa, ya = intersecciones[i][1],   intersecciones[i][2]
        local xb, yb = intersecciones[i+1][1], intersecciones[i+1][2]

        -- ======== Calcular distancia Euclidiana entre puntos consecutivos ========
        local distancia = math.sqrt((xb - xa)^2 + (yb - ya)^2) 
        table.insert(distancias_a_recorrer, distancia)

        -- Ángulo absoluto del segmento i→i+1
        local curr_angle = math.atan2(yb - ya, xb - xa) * (180/math.pi)

        -- Giro relativo desde el eje +X del observado o desde el tramo anterior
        local rel_angle = curr_angle - prev_angle

        -- Ajustar a [-180,180]
        if     rel_angle >  180 then rel_angle = rel_angle - 360
        elseif rel_angle < -180 then rel_angle = rel_angle + 360 end

        table.insert(angles_a_recorrer, rel_angle)
        prev_angle = curr_angle
    end

    -- ====================================================
    -- LOG DE DISTANCIAS Y ÁNGULOS
    -- ====================================================
    -- log("--- LOG: Distancias y Ángulos ---")
    -- for i = 1, #distancias_a_recorrer do
    --     log(string.format("  Tramo %d -> Distancia: %.2f, Ángulo Relativo: %.2f°", 
    --         i, distancias_a_recorrer[i], angles_a_recorrer[i]))
    -- end
    -- log("-----------------------------------")
    
    return distancias_a_recorrer, angles_a_recorrer
end

-- ============================================================================
-- 4. CREATE PARAMS: Empaqueta los datos con su ID Original
-- ============================================================================
params = {}
function create_params(distancias, angulos, valid_cols)
    params = {}  
    
    for i = 1, math.min(#distancias, #angulos) do
        local distancia = distancias[i]
        local angulo    = angulos[i]
        
        -- CORRECCIÓN CLAVE: Rescatamos el ID de la columna original que generó este parámetro
        local original_col_id = valid_cols[i] 
        
        -- Guardamos { Distancia, Ángulo, ID_Original }
        table.insert(params, { distancia, angulo, original_col_id })
    end

    -- -- Logging original comentado
    -- log(string.format("Total segmentos construidos: %d ", #params)..", "..robot.id)
    -- for idx, seg in ipairs(params) do
    --     log(string.format(
    --         "Segmento %d -> Distancia: %.2f cm, ÁnguloRel: %.2f°",
    --         idx, seg[1], seg[2]
    --     ))
    -- end
    
    return params
end

-- ============================================================================
-- 5. Datta Buffer: Procesa y Almacena los parámetros generados para su posterior reconstrucción
-- ============================================================================

DataBuffer = { stored = {} }
Global_Reconstruction_Done = false 

-- El contenedor de estado para el ciclo actual (Nace vacío, sin variables fantasma)
local ARBITER_CONFIG = {}

-- ================= HERRAMIENTAS DE TEXTO Y UTILIDAD =================

local function fmt_struct(s)
    if not s then return "nil" end
    local vals = {}
    for _, v in ipairs(s.value) do table.insert(vals, string.format("%.2f", v)) end
    local t_str = (s.type == 1) and "Ang" or "Dist"
    local count_str = (s.count and s.count > 1) and string.format(" [x%d]", s.count) or ""
    return string.format("(%s)%s {%s}", t_str, count_str, table.concat(vals, ", "))
end

local function simple_fmt(val_table)
    local vals = {}
    for _, v in ipairs(val_table) do table.insert(vals, string.format("%.2f", v)) end
    return table.concat(vals, ",")
end

local function get_val_type(struct_type, index)
    local is_odd = (index % 2 ~= 0)
    if struct_type == 1 then return is_odd and "A" or "D"
    else return is_odd and "D" or "A" end
end

local function check_match(long_dat, short_dat, start_idx)
    if get_val_type(long_dat.type, start_idx) ~= get_val_type(short_dat.type, 1) then return false end
    for k = 1, #short_dat.value do
        local l_idx = start_idx + k - 1
        local v_type = get_val_type(short_dat.type, k)
        local diff = math.abs(long_dat.value[l_idx] - short_dat.value[k])
        local tol = (v_type == "A") and CONFIG.TOL_ANG or CONFIG.TOL_DIST
        if diff > tol then return false end
    end
    return true
end

-- ================= ALMACENAMIENTO DE ISLAS =================

function DataBuffer.add(new_dat)
    if Global_Reconstruction_Done then return end 
    local found = false

    for i, exist in ipairs(DataBuffer.stored) do
        if #exist.value == #new_dat.value and exist.type == new_dat.type then
            if check_match(exist, new_dat, 1) then
                found = true
                exist.count = (exist.count or 1) + 1
                for k = 1, #exist.value do 
                    exist.value[k] = exist.value[k] + (new_dat.value[k] - exist.value[k]) / exist.count 
                end
                table.insert(exist.same_size_merges, simple_fmt(new_dat.value))
                log(string.format("[INFO] Dato Refinado (Promedio de %d): %s", exist.count, fmt_struct(exist)))
                update_patience("partial", exist.count) 
                break
            end
        end

        if not found and #new_dat.value >= #exist.value + 2 then
            for start_idx = 2, #new_dat.value - #exist.value do
                if check_match(new_dat, exist, start_idx) then
                    found = true
                    exist.count = (exist.count or 1) + 1
                    local merged_values = {}
                    for k = 1, #new_dat.value do
                        if k >= start_idx and k < start_idx + #exist.value then
                            local e_idx = k - start_idx + 1
                            local avg_val = exist.value[e_idx] + (new_dat.value[k] - exist.value[e_idx]) / exist.count
                            table.insert(merged_values, avg_val)
                        else
                            table.insert(merged_values, new_dat.value[k])
                        end
                    end
                    exist.value = merged_values; exist.type = new_dat.type 
                    table.insert(exist.diff_size_merges, simple_fmt(new_dat.value))
                    log(string.format("[INFO] Dato Actualizado (Extendido a %d val, solape en medio): %s", #exist.value, fmt_struct(exist)))
                    update_patience("partial", exist.count)
                    break
                end
            end
            if found then break end
        end

        if not found and #exist.value >= #new_dat.value + 2 then
            for start_idx = 2, #exist.value - #new_dat.value do
                if check_match(exist, new_dat, start_idx) then
                    found = true
                    local divisor = (exist.count or 1) + 1
                    for k = 1, #new_dat.value do
                        local e_idx = start_idx + k - 1
                        exist.value[e_idx] = exist.value[e_idx] + (new_dat.value[k] - exist.value[e_idx]) / divisor
                    end
                    table.insert(exist.diff_size_merges, simple_fmt(new_dat.value))
                    log(string.format("[INFO] Dato Actualizado (Subcadena promediada - Frec INTACTA %d): %s", exist.count, fmt_struct(exist)))
                    update_patience("partial", exist.count)
                    break
                end
            end
            if found then break end
        end
    end

    if not found then
        local new_v = {}
        for _, v in ipairs(new_dat.value) do table.insert(new_v, v) end
        local entry = { value = new_v, type = new_dat.type, count = 1, same_size_merges = {}, diff_size_merges = {} }
        table.insert(DataBuffer.stored, entry)
        log(string.format("[INFO] Nuevo Dato Registrado: %s", fmt_struct(entry)))
        update_patience("full", 1)
    end
end

-- ================= EXTRACCIÓN DE PARES Y PUREZA =================

local pairs_db = {}

local function extract_pairs_from_island(struct)
    local island_pairs = {}
    local is_dist_first = (struct.type == 0)
    
    if #struct.value < 2 then return {}, false end
    
    local is_pure = false
    if #struct.value >= 3 then
        is_pure = true
        for i = 1, #struct.value - 2 do
            local v1 = struct.value[i]
            local v2 = struct.value[i+2]
            local v_type = get_val_type(struct.type, i)
            local tol = (v_type == "A") and CONFIG.TOL_ANG or CONFIG.TOL_DIST
            if math.abs(v1 - v2) > tol then
                is_pure = false
                break
            end
        end
    end
    
    for i = 1, #struct.value - 1 do
        local v1 = struct.value[i]
        local v2 = struct.value[i+1]
        local is_v1_dist = is_dist_first and (i%2 ~= 0) or (not is_dist_first and i%2 == 0)
        local d, a, p_type
        
        if is_v1_dist then d = v1; a = v2; p_type = 0 
        else a = v1; d = v2; p_type = 1 end
        
        table.insert(island_pairs, {d = d, a = a, type = p_type, is_pure = is_pure})
    end
    
    return island_pairs, is_pure
end

local function build_global_pairs_db(pool)
    pairs_db = {}
    for _, struct in ipairs(pool) do
        local island_pairs, is_pure = extract_pairs_from_island(struct)
        for _, p in ipairs(island_pairs) do
            local found = false
            for _, db_p in ipairs(pairs_db) do
                local values_match = (math.abs(db_p.d - p.d) <= CONFIG.TOL_DIST and math.abs(db_p.a - p.a) <= CONFIG.TOL_ANG)
                if values_match and db_p.type == p.type then
                    db_p.d = db_p.d + (p.d - db_p.d) / (db_p.merges + 1)
                    db_p.a = db_p.a + (p.a - db_p.a) / (db_p.merges + 1)
                    db_p.count = db_p.count + (struct.count or 1)
                    db_p.merges = db_p.merges + 1
                    db_p.is_pure = db_p.is_pure or is_pure 
                    found = true
                    break
                end
            end
            if not found then 
                table.insert(pairs_db, {d = p.d, a = p.a, type = p.type, count = (struct.count or 1), merges = 1, is_pure = is_pure}) 
            end
        end
    end
end

-- ================= EL TERMOSTATO (Auto-Calibración) =================

local function calibrar_hiperparametros(freq_array, current_pairs_db)
    if #freq_array == 0 then return end

    local max_frecuencia = 0
    local suma_frecuencias = 0
    local pureza_total = 0

    for _, f in ipairs(freq_array) do
        if f > max_frecuencia then max_frecuencia = f end
        suma_frecuencias = suma_frecuencias + f
    end
    
    for _, p in ipairs(current_pairs_db) do
        if p.is_pure then pureza_total = pureza_total + p.count end
    end

    local prom_frecuencia = suma_frecuencias / #freq_array
    
-- POR ESTE NUEVO BLOQUE CORREGIDO:
    local prom_frecuencia = suma_frecuencias / #freq_array
    
    -- Cálculos de estado del enjambre (Corregido: Basado en Fragmentación)
    local indice_orden = 1.0
    if suma_frecuencias > 0 then
        -- #freq_array es la cantidad de pares distintos.
        -- Mientras menos pares distintos haya respecto al total de datos, mayor es el orden.
        indice_orden = 1.0 - (#freq_array / suma_frecuencias)
        -- Seguro matemático: evitar valores negativos si algo extraño ocurre
        if indice_orden < 0.0 then indice_orden = 0.0 end
    end
    
    local ratio_pureza = 0
    if suma_frecuencias > 0 then
        ratio_pureza = pureza_total / suma_frecuencias
    end

    log("\n=========================================")
    log("[TERMOSTATO] AUTO-CALIBRACIÓN DEL ENJAMBRE")
    log("=========================================")
    log("  [1] Extracción de Métricas Base:")
    log(string.format("      - Frecuencia Máxima Dominante: %d", max_frecuencia))
    log(string.format("      - Frecuencia Promedio Global: %.2f", prom_frecuencia))
    log(string.format("      - Total Fragmentos (Volumen): %d", suma_frecuencias))
    log(string.format("      - N-Gramas (Pares) Únicos: %d", #freq_array))
    log(string.format("      - Fragmentos con Simetría (Puros): %d", pureza_total))
    
    log("\n  [2] Diagnóstico del Entorno (Fórmulas):")
    log(string.format("      - Índice de Orden: 1.0 - (Únicos [%d] / Volumen [%d]) = %.2f", #freq_array, suma_frecuencias, indice_orden))
    log("          * (1.0 = Dominancia clara, 0.0 = Caos total)")
    log(string.format("      - Ratio de Pureza: Puros [%d] / Volumen [%d] = %.2f", pureza_total, suma_frecuencias, ratio_pureza))
    log("          * (1.0 = Entorno 100% geométrico)")

    -- Aplicación estricta de las variables maestras de CONFIG
    local nivel_caos = 1.0 - indice_orden
    local ruido_extra = nivel_caos * CONFIG.RUIDO_PENALIZACION_MAX
    ARBITER_CONFIG.RUIDO_UMBRAL_RELATIVO = CONFIG.RUIDO_BASE + ruido_extra

    local bono_extra = ratio_pureza * CONFIG.BONO_M3_MAX_EXTRA * indice_orden
    ARBITER_CONFIG.BONO_REGULARIDAD = CONFIG.BONO_M3_BASE + bono_extra

    local penalizacion_caos = nivel_caos * CONFIG.SCORE_PENALIZACION_MAX
    ARBITER_CONFIG.MIN_SCORE = CONFIG.SCORE_MIN_BASE + penalizacion_caos

    log("\n  [3] Fórmulas y Resultados Aplicados (Lectura desde CONFIG):")
    log(string.format("      > UMBRAL RUIDO: Base(%.2f) + [Caos(%.2f) * Max_Penalizacion(%.2f)] = %.1f%% de tolerancia.", 
        CONFIG.RUIDO_BASE, nivel_caos, CONFIG.RUIDO_PENALIZACION_MAX, ARBITER_CONFIG.RUIDO_UMBRAL_RELATIVO * 100))
        
    log(string.format("      > BONO GEOMETRÍA: Base(%.2f) + [Pureza(%.2f) * Orden(%.2f) * Max_Bono(%.2f)] = Multiplicador x%.2f.", 
        CONFIG.BONO_M3_BASE, ratio_pureza, indice_orden, CONFIG.BONO_M3_MAX_EXTRA, ARBITER_CONFIG.BONO_REGULARIDAD))
        
    log(string.format("      > SCORE MÍNIMO: Base(%.2f) + [Caos(%.2f) * Max_Penalizacion(%.2f)] = Exigencia %.3f.", 
        CONFIG.SCORE_MIN_BASE, nivel_caos, CONFIG.SCORE_PENALIZACION_MAX, ARBITER_CONFIG.MIN_SCORE))
    log("=========================================")
end

local function calcular_limite_ruido(frecuencias)
    if #frecuencias == 0 then return 0 end
    local max_freq = 0
    for _, f in ipairs(frecuencias) do
        if f > max_freq then max_freq = f end
    end
    local limite = max_freq * ARBITER_CONFIG.RUIDO_UMBRAL_RELATIVO
    
    log(string.format("\n[FILTRO DE RUIDO] Frec. Max (%d) * Umbral Relativo (%.2f) = Corte Matemático: %.2f", max_freq, ARBITER_CONFIG.RUIDO_UMBRAL_RELATIVO, limite))
    return limite
end

-- ================= ENSAMBLADORES (M1 / M2) =================

local function calculate_overlap(s1, s2, min_ov)
    local len1, len2 = #s1.value, #s2.value
    local max_ov = math.min(len1, len2)
    for ov = max_ov, min_ov, -1 do
        local match = true
        for k = 0, ov-1 do
            local idx1, idx2 = len1-ov+1+k, 1+k
            local t1 = get_val_type(s1.type, idx1)
            local t2 = get_val_type(s2.type, idx2)
            if t1 ~= t2 then match=false; break end
            local diff = math.abs(s1.value[idx1] - s2.value[idx2])
            local tol = (t1=="A") and CONFIG.TOL_ANG or CONFIG.TOL_DIST
            if diff > tol then match=false; break end
        end
        if match then return ov end
    end
    return 0
end

local function merge_fragments(s1, s2, overlap)
    local new_vals = {}
    for k=1, #s1.value - overlap do table.insert(new_vals, s1.value[k]) end
    for k=0, overlap - 1 do 
        local v1 = s1.value[#s1.value - overlap + 1 + k]
        local v2 = s2.value[1 + k]
        table.insert(new_vals, (v1+v2)/2) 
    end
    for k=overlap + 1, #s2.value do table.insert(new_vals, s2.value[k]) end
    
    local combined_id = s1.id .. "+" .. s2.id
    local new_count = (s1.count or 1) + (s2.count or 1)
    local new_total_ov = (s1.total_ov or 0) + (s2.total_ov or 0) + overlap
    
    local new_consumed = {}
    for k, v in pairs(s1.consumed_ids or {}) do new_consumed[k] = true end
    for k, v in pairs(s2.consumed_ids or {}) do new_consumed[k] = true end

    return { value = new_vals, type = s1.type, id = combined_id, count = new_count, total_ov = new_total_ov, consumed_ids = new_consumed }
end

local function clone_pool(pool)
    local new_pool = {}
    for i, v in ipairs(pool) do
        local nv = {}
        for _, val in ipairs(v.value) do table.insert(nv, val) end
        local c_ids = {}
        for id_k, id_v in pairs(v.consumed_ids or {}) do c_ids[id_k] = id_v end
        table.insert(new_pool, { value = nv, type = v.type, id = v.id, count = v.count, total_ov = v.total_ov or 0, consumed_ids = c_ids })
    end
    return new_pool
end

local function solve_greedy(initial_pool)
    local pool = clone_pool(initial_pool)
    while #pool > 1 do
        local best = { i=-1, j=-1, ov=-1, struct=nil }
        for i=1, #pool do
            for j=1, #pool do
                if i ~= j then
                    local ov = calculate_overlap(pool[i], pool[j], CONFIG.MIN_OVERLAP_M1)
                    if ov > best.ov then best = { i=i, j=j, ov=ov, struct=merge_fragments(pool[i], pool[j], ov) } end
                end
            end
        end
        if best.ov > 0 then
            pool[best.i] = best.struct; table.remove(pool, best.j)
        else break end
    end
    return pool
end

local raw_solutions_found = {} 

local function recursive_finder(current_pool, history_log)
    local made_merge = false
    local candidates = {}
    for i=1, #current_pool do
        for j=1, #current_pool do
            if i ~= j then
                local ov = calculate_overlap(current_pool[i], current_pool[j], CONFIG.MIN_OVERLAP_M2)
                if ov > 0 then table.insert(candidates, {i=i, j=j, ov=ov}) end
            end
        end
    end

    if #candidates > 0 then
        made_merge = true
        table.sort(candidates, function(a,b) return a.ov > b.ov end)
        for _, cand in ipairs(candidates) do
            local next_pool = {}
            local merged = merge_fragments(current_pool[cand.i], current_pool[cand.j], cand.ov)
            table.insert(next_pool, merged)
            for k=1, #current_pool do
                if k ~= cand.i and k ~= cand.j then table.insert(next_pool, current_pool[k]) end
            end
            local next_history = {} 
            for _, h in ipairs(history_log) do table.insert(next_history, h) end
            table.insert(next_history, string.format("   Merge %s+%s (Ov:%d)", current_pool[cand.i].id, current_pool[cand.j].id, cand.ov))
            recursive_finder(next_pool, next_history)
        end
    end

    if not made_merge then 
        table.insert(raw_solutions_found, { pool_result = current_pool, history = history_log }) 
    end
end

local function select_best_solution_m2(solutions)
    if #solutions == 0 then return nil end
    local unique_groups = {}
    for _, sol in ipairs(solutions) do
        local hashes = {}
        local total_len = 0; local total_ov = 0
        for _, s in ipairs(sol.pool_result) do 
            table.insert(hashes, fmt_struct(s))
            total_len = total_len + #s.value; total_ov = total_ov + (s.total_ov or 0)
        end
        table.sort(hashes)
        local key = table.concat(hashes, " || ")
        if not unique_groups[key] then 
            unique_groups[key] = { pool_result = sol.pool_result, count = 1, history = sol.history, total_len = total_len, total_ov = total_ov }
        else unique_groups[key].count = unique_groups[key].count + 1 end
    end
    local unique_list = {}
    for key, data in pairs(unique_groups) do
        local score = (data.count * CONFIG.W_FREQ) + (data.total_ov * CONFIG.W_OV_QUAL)
        data.score = score; table.insert(unique_list, data)
    end
    table.sort(unique_list, function(a, b)
        if a.score == b.score then return a.count > b.count end
        return a.score > b.score
    end)
    return unique_list[1]
end

local function solve_smart_backtracking_all(initial_pool)
    raw_solutions_found = {} 
    local sorted_pool = clone_pool(initial_pool)
    table.sort(sorted_pool, function(a,b) return #a.value > #b.value end)
    recursive_finder(sorted_pool, {})
    return select_best_solution_m2(raw_solutions_found)
end


-- =========================================================================
--  MODULO M3: EL GEÓMETRA (Evaluador de Polígonos Regulares)
-- =========================================================================
local function evaluar_hipotesis_m3(clean_pool, current_pairs_db, total_data_volume, iteration)
    local m3_score = -1
    local m3_hypothesis = nil
    local m3_covered_ids = {} 
    
    local best_pure_pair = nil
    for _, p in ipairs(current_pairs_db) do
        if p.is_pure then
            if not best_pure_pair or p.count > best_pure_pair.count then best_pure_pair = p end
        end
    end

    if best_pure_pair then
        local coverage_count = 0
        local source_islands_list = {}

        for _, struct in ipairs(clean_pool) do
            local island_pairs, _ = extract_pairs_from_island(struct)
            local matches_perfectly = true
            for _, p in ipairs(island_pairs) do
                if math.abs(p.d - best_pure_pair.d) > CONFIG.TOL_DIST or math.abs(p.a - best_pure_pair.a) > CONFIG.TOL_ANG then
                    matches_perfectly = false; break
                end
            end
            if matches_perfectly then
                coverage_count = coverage_count + struct.count
                m3_covered_ids[struct.id] = true
                table.insert(source_islands_list, struct.id)
            end
        end
        
        local coverage_norm_m3 = coverage_count / total_data_volume
        local lados = 4 
        if best_pure_pair.a ~= 0 then lados = math.floor((360 / math.abs(best_pure_pair.a)) + 0.5) end
        
        local parametros_m3 = lados * 2
        local complexity_norm_m3 = parametros_m3 / total_data_volume
        m3_score = (coverage_norm_m3 * ARBITER_CONFIG.BONO_REGULARIDAD) - (complexity_norm_m3 * CONFIG.PESO_COMPLEJIDAD)
        
        local synthetic_vals = {}
        for i=1, lados do table.insert(synthetic_vals, best_pure_pair.d); table.insert(synthetic_vals, best_pure_pair.a) end
        m3_hypothesis = { type = 0, value = synthetic_vals, id = "Poligono_Iter"..iteration, count = coverage_count }
        
        local val_str = (best_pure_pair.type == 1) and string.format("Ang: %.2f, Dist: %.2f", best_pure_pair.a, best_pure_pair.d) or string.format("Dist: %.2f, Ang: %.2f", best_pure_pair.d, best_pure_pair.a)
        
        local sources_str = table.concat(source_islands_list, ", ")
        log(string.format("[M3] Analizando Par Puro {%s} de Islas [%s]", val_str, sources_str))
        log(string.format("     -> Propone Polígono (%d lados). Cobertura: %.1f%% | Score Normalizado: %.3f", lados, coverage_norm_m3 * 100, m3_score))
    else
        log("[M3] No hay fragmentos con pureza geométrica demostrable. Se abstiene.")
    end

    return m3_score, m3_hypothesis, m3_covered_ids
end


-- =========================================================================
--  GESTOR PRINCIPAL: EL BUCLE ITERATIVO DE EXTRACCIÓN (ÁRBITRO)
-- =========================================================================
function run_global_reconstruction()
    if Global_Reconstruction_Done then return end
    
    log("\n=========================================")
    log("TABLA DE ISLAS (FRAGMENTOS RECOLECTADOS)")
    log("=========================================")
    for i, v in ipairs(DataBuffer.stored) do 
        log(string.format("[%d] %s", i, fmt_struct(v))) 
    end
        
    local raw_pool = {}
    for i, v in ipairs(DataBuffer.stored) do 
        table.insert(raw_pool, { 
            value = v.value, 
            type = v.type, 
            count = v.count, 
            id = tostring(i),
            consumed_ids = { [tostring(i)] = true } 
        }) 
    end
    if #raw_pool == 0 then return end
    
    build_global_pairs_db(raw_pool)
    
    local freq_array = {}
    for _, p in ipairs(pairs_db) do table.insert(freq_array, p.count) end
    
    calibrar_hiperparametros(freq_array, pairs_db)
    
    log("\n=========================================")
    log("FASE 1: FILTRADO DE RUIDO Y N-GRAMAS")
    log("=========================================")
    for _, p in ipairs(pairs_db) do 
        local val_str = ""
        if p.type == 1 then 
            val_str = string.format("Ang %.2f, Dist %.2f", p.a, p.d)
        else                
            val_str = string.format("Dist %.2f, Ang %.2f", p.d, p.a)
        end
        local t_str = (p.type == 1) and "A-D" or "D-A"
        log(string.format("[PAR %s] %s | Frec: %d (Pureza: %s)", t_str, val_str, p.count, tostring(p.is_pure)))
    end

    local limite_ruido = calcular_limite_ruido(freq_array)
    
    local clean_pool = {}
    local noise_removed_count = 0
    for _, struct in ipairs(raw_pool) do
        local island_pairs, _ = extract_pairs_from_island(struct)
        local is_noisy = false
        for _, p in ipairs(island_pairs) do
            for _, db_p in ipairs(pairs_db) do
                if (math.abs(db_p.d - p.d) <= CONFIG.TOL_DIST and math.abs(db_p.a - p.a) <= CONFIG.TOL_ANG) then
                    if db_p.count < limite_ruido then is_noisy = true end
                    break
                end
            end
            if is_noisy then break end
        end
        
        if is_noisy then
            noise_removed_count = noise_removed_count + 1
            log(string.format("  [-] Eliminado por ruido: %s", fmt_struct(struct)))
        else
            table.insert(clean_pool, struct)
        end
    end

    local all_final_results = {} 
    local iteration = 1

    -- FASE 2: EL BUCLE DEL ÁRBITRO
    log("\n=========================================")
    log("FASE 2: EXTRACCIÓN ITERATIVA DE PATRONES")
    log("=========================================")

    while #clean_pool > 0 do
        log(string.format("\n--- ITERACIÓN %d | Fragmentos en buffer: %d ---", iteration, #clean_pool))
        
        local total_data_volume = 0
        for _, v in ipairs(clean_pool) do total_data_volume = total_data_volume + v.count end
        if total_data_volume == 0 then break end

        build_global_pairs_db(clean_pool)
        local m3_score, m3_hypothesis, m3_covered_ids = evaluar_hipotesis_m3(clean_pool, pairs_db, total_data_volume, iteration)

        local explorer_score = -1
        local explorer_hypothesis = nil
        local explorer_name = "M1"
        local explorer_pool_result = {}
        
        if #clean_pool <= CONFIG.MAX_ISLANDS_M2 then
            log("[EXPLORADORES] Activando M2 (Backtracking Exhaustivo)...")
            local m2_solution = solve_smart_backtracking_all(clean_pool)
            if m2_solution and #m2_solution.pool_result > 0 then
                explorer_pool_result = m2_solution.pool_result
                explorer_name = "M2"
            else
                log("[EXPLORADORES] M2 no encontró combinaciones globales, usando M1 Fallback.")
                explorer_pool_result = solve_greedy(clean_pool)
            end
        else
            log(string.format("[EXPLORADORES] Buffer excede %d islas. Usando M1 (Greedy Voraz) por seguridad...", CONFIG.MAX_ISLANDS_M2))
            explorer_pool_result = solve_greedy(clean_pool)
        end

        if #explorer_pool_result > 0 then
            table.sort(explorer_pool_result, function(a,b) return (a.count or 1) > (b.count or 1) end)
            explorer_hypothesis = explorer_pool_result[1]
            
            local coverage_norm_exp = (explorer_hypothesis.count or 1) / total_data_volume
            local complexity_norm_exp = #explorer_hypothesis.value / total_data_volume
            
            explorer_score = (coverage_norm_exp * 1.0) - (complexity_norm_exp * CONFIG.PESO_COMPLEJIDAD)
            
            log(string.format("[%s] Propone Cadena Irregular (Largo %d). Cobertura: %.1f%% | Score Normalizado: %.3f", 
                explorer_name, #explorer_hypothesis.value, coverage_norm_exp * 100, explorer_score))
        end

        local winner_struct = nil
        local ids_to_remove = {}

        if m3_score > explorer_score and m3_score >= ARBITER_CONFIG.MIN_SCORE then
            winner_struct = m3_hypothesis
            ids_to_remove = m3_covered_ids
            log(string.format("[ÁRBITRO] >>> ¡Gana M3 (Geometría Regular)! -> %s", fmt_struct(winner_struct)))
            
        elseif explorer_score >= m3_score and explorer_score >= ARBITER_CONFIG.MIN_SCORE then
            winner_struct = explorer_hypothesis
            ids_to_remove = explorer_hypothesis.consumed_ids
            log(string.format("[ÁRBITRO] >>> ¡Gana %s (Ruta Irregular)! -> %s", explorer_name, fmt_struct(winner_struct)))
            
        else
            log(string.format("[ÁRBITRO] Ninguna hipótesis supera el MIN_SCORE exigido (%.3f). Fin de la extracción.", ARBITER_CONFIG.MIN_SCORE))
            break
        end

        table.insert(all_final_results, winner_struct)
        local next_pool = {}
        local removed_count = 0
        for _, struct in ipairs(clean_pool) do
            if not ids_to_remove[struct.id] then table.insert(next_pool, struct)
            else removed_count = removed_count + 1 end
        end
        
        log(string.format("  > Se descartaron %d fragmentos base usados por el ganador.", removed_count))
        clean_pool = next_pool
        iteration = iteration + 1
    end

    log("\n---------------- RESULTADOS FINALES ----------------")
    if #all_final_results > 0 then
        log(string.format(">> Se descubrieron %d patrones distintos ejecutados por el enjambre:", #all_final_results))
        for i, res in ipairs(all_final_results) do log(string.format("   Patrón %d: %s", i, fmt_struct(res))) end
    else
        log(">> ALERTA: No se pudo recuperar ninguna estructura sólida (Todo fue filtrado como ruido).")
    end
    log("----------------------------------------------------\n")

    Global_Reconstruction_Done = true
    for i, res in ipairs(all_final_results) do process_numeric_result(res, i) end

        -- ACTIVAR ESTADO FINAL DEL ROBOT
    resetOBS()
    is_demonstrator = 1
end

function process_numeric_result(result_struct, patron_id)
    local current_pattern_params = {} -- Variable local para construir ESTE patrón
    local values = result_struct.value
    local current_type = result_struct.type
    
    local temp_dist = 0; local temp_ang = 0
    local expecting_dist = (current_type == 0)
    
    for i, val in ipairs(values) do
        if expecting_dist then
            temp_dist = val
            table.insert(current_pattern_params, { temp_dist, temp_ang })
            temp_ang = 0
            expecting_dist = false
        else
            temp_ang = val
            expecting_dist = true
        end
    end
    
    log(string.format("\n[TRADUCCIÓN FÍSICA - PATRÓN %d]", patron_id))
    for idx, seg in ipairs(current_pattern_params) do 
        log(string.format("   Seg %d: Dist %.2f, Ang %.2f", idx, seg[1], seg[2])) 
    end
    
    -- Guardamos el patrón procesado en la lista global
    table.insert(all_params, current_pattern_params)
    
    --c4 = 1 -- Señal de que la reconstrucción terminó y hay patrones listos
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
-- Funcines de acciones/decisiones/sensores de los FootBot
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

--función  de detección de proximidad
proximity = {0, 0}
function proximidad()
    local sensores = robot.proximity
    local total   = 0
    local count   = 0
    local max_val = 0
    local max_idx = 0

    -- 1) Recorremos todos los sensores
    for i, s in ipairs(sensores) do
        local v = s.value
        total = total + v
        if v > 0.01 then
            count = count + 1
            if v > max_val then
                max_val = v
                max_idx = i
            end
        end
    end

    -- 2) Calculamos promedio de distancia
    if count > 0 then
        proximity[1] = max_val
    else
        proximity[1] = 0
    end

    -- 3) Estimamos ángulo a partir del sensor más fuerte
    if max_idx > 0 then
        -- Número de sensores por semicírculo
        local N = Num_sensores_de_proximidad / 2  
        -- Paso angular
        local step = 180 / (N )

        local angle = 0
        if max_idx <= N then
            -- Mapea 1→0°, N→180°
            angle = (max_idx) * step
        else
            -- Mapea N+1→-180°, count→0°
            angle = -180 + (max_idx - N ) * step
        end
        proximity[2] = angle
    else
        proximity[2] = 0
    end

    -- 4) Log para depuración
    -- log(string.format(
    --   "Promedio IR = %.3f, Sensor pico = %d (val=%.3f) → Ángulo ≈ %.1f°",
    --   proximity[1], max_idx, max_val, proximity[2]
    -- ).." "..robot.id)

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

-- NUEVAS variables globales para el control residual exacto
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

-- Función para determiaar si un epuck será observador o demostrador
is_demonstrator = 0
function determinar_rol()
    -- Selección aleatoria del demostrador

    randoms = robot.random.uniform_int(0, 2)
    if randoms == 1 then
        is_demonstrator = 1
    else
        is_demonstrator = 0
    end

    if robot.id ~= "Footbot4" then
        is_demonstrator = 1
    else
        is_demonstrator = 0
    end

    -- Encender LED rojo si es demostrador
    if is_demonstrator == 1 then
        --setleds("red")
        log("The " .. robot.id .. " is demostrador = " .. randoms)
        --log("Led rojo encendido por " ..robot.id)
    else
        --robot.leds.set_all_colors("blue")
        log("The " .. robot.id .. " is Observador= " .. randoms)
    end


    return is_demonstrator
end

function resetOBS()
    -- ============================
    distances = {} -- Lista de posiciones relativas (x, y)
    angles = {}    -- Lista de ángulos relativos
    srt_distance_an = 80
    srt_distance_new = 0

    SRT_c1 = 0
    srt_c2 = 0
    srt_c3 = 0
    srt_c4 = 0
    srt_c5 = 0
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
    movement_buffer = {}     -- buffer temporal con puntos mientras el demostrador está en movimiento

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
    c3            = 0
    c4            = 0
    -- ============================
    setleds("black")
    r_Temporizador_1()
    r_Temporizador_2()
    r_move()
end

function resetDEM() 

    ind = 1
    k = 0

    -- ============================¿
    d1 = 0

    demostrador_avg_angle = 0

    posiciones_guardadas = {}

    indice = 1

    EAEP_c1 = 0
    EAEP_c2 = 0
    demostrador_avg_dist_1 = 0
    EAEP_ang_ant = 0
    -- ============================¿

    MT_c1 = 0
    -- ============================¿
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
    for i = 1, 13 do
        if i <= 12 and i >= 1 then
            robot.leds.set_single_color(i, "black") -- Apagar LED
        else
            robot.leds.set_single_color(i, color) -- Encender LED con el color recibido como parámetro
        end
    end
end--187014

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
    -- if is_demonstrator == 0 then
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

function imprimir_tracking_entry(k, data, max_puntos)
    if not data then return end
    -- Si no se especifica max_puntos, por defecto mostramos todos (opcional)
    max_puntos = max_puntos or 999999 

    log("==========================================")
    log("          REPORTE MAESTRO: " .. tostring(k))
    log("==========================================")
    
    -- 1. PATRÓN OBSERVADO Y SECUENCIA
    log("PATRÓN OBSERVADO:")
    local inicio_txt = data.Pattern_start_detected and "Detectado" or "No Detectado"
    local fin_txt = data.End_of_pattern_detected and "Detectado" or "No Detectado"
    log("   Inicio:   " .. inicio_txt)
    log("   Fin:      " .. fin_txt)
    -- Nuevo campo pattern_sequence
    log("   Secuencia: " .. (data.pattern_sequence ~= "" and data.pattern_sequence or "[Vacía]"))
    log("")

    -- 2. PARÁMETROS DE MOVIMIENTO
    if data.params and #data.params > 0 then
        log("PARÁMETROS (Trayectoria Estimada):")
        for idx, seg in ipairs(data.params) do
            log(string.format("   Seg %d -> Distancia: %.2f cm, ÁnguloRel: %.2f°", 
                idx, seg[1], seg[2]))
        end
        log("")
    end

    -- 3. INTERSECCIONES
    if data.intersecciones and #data.intersecciones > 0 then
        log("INTERSECCIONES DETECTADAS:")
        for i, punto in ipairs(data.intersecciones) do
            log(string.format("   #%d: (%.3f, %.3f)", i, punto[1], punto[2]))
        end
        log("")
    end

    -- 4. SEGMENTOS (Con limitador de puntos)
    if data.segmentos and data.segmentos.movement then
        log("DETALLE DE SEGMENTOS (Limite: " .. max_puntos .. " pts):")
        for seg_idx, puntos_en_seg in ipairs(data.segmentos.movement) do
            
            local total_puntos = #puntos_en_seg
            local es_valido = "Inválido"
            if data.segmentos.valid[seg_idx] and data.segmentos.valid[seg_idx][1] == true then
                es_valido = "Válido"
            end

            log(string.format("   --- Segmento %d [%s] (%d puntos totales) ---", seg_idx, es_valido, total_puntos))
            log("   Posición (x, y)          Tipo")

            -- Lógica para limitar puntos
            local mostrar = math.min(total_puntos, max_puntos)
            for pt_idx = 1, mostrar do
                local pt = puntos_en_seg[pt_idx]
                local tipo = data.segmentos.type[seg_idx][pt_idx] or "?"
                log(string.format("   %.3f , %.3f        ->  %s", pt[1], pt[2], tostring(tipo)))
            end

            -- Mensaje de puntos restantes
            if total_puntos > max_puntos then
                local restantes = total_puntos - max_puntos
                log(string.format("   ... [%d puntos restantes no mostrados] ...", restantes))
            end
        end
        log("")
    end

    -- 5. TABLAS DE POSICIÓN ADICIONALES (Se aplica el mismo criterio de max_puntos)
    for nombre, contenido in pairs(data) do
        local reservadas = {segmentos=1, params=1, intersecciones=1, movement_buffer=1, type_buffer=1, pattern_sequence=1}
        if type(contenido) == "table" and not reservadas[nombre] then
            if contenido[1] and type(contenido[1]) == "table" then
                log("OTRA TABLA: " .. nombre)
                local total = #contenido
                local mostrar = math.min(total, max_puntos)
                for i = 1, mostrar do
                    local pos = contenido[i]
                    if pos[1] and pos[2] then
                        log(string.format("   Pos %d: x = %.3f, y = %.3f", i, pos[1], pos[2]))
                    end
                end
                if total > max_puntos then
                    log("   ... [" .. (total - max_puntos) .. " posiciones restantes] ...")
                end
            end
        end
    end

    log("------------------------------------------")
end

FG_first_time   = true
FG_timer        = 0
function figures()

    local yellow_true = false

    --============================== triangulo_regular ==============================
    if figura == "triangulo_regular" then

        if robot.id == "Footbot6" then 

            if PO_counter_seg < 2 then
                yellow_true = true
            end
            if PO_counter_seg == 2 and FG_timer < 25 then
                FG_timer = FG_timer + 1 
                yellow_true = true
            end
        end

        if robot.id == "Footbot7" then 

            if PO_counter_seg < 3 and PO_counter_seg >= 1 then
                yellow_true = true
            end
            if PO_counter_seg == 3 and FG_timer < 25 then
                FG_timer = FG_timer + 1 
                yellow_true = true
            end 
        end

        if robot.id == "Footbot8" then 

            if not FG_first_time and PO_counter_seg == 0 then
                yellow_true = true
            end
            if PO_counter_seg == 1 then
                FG_first_time = false
            end
            if PO_counter_seg > 1 then
                yellow_true = true
            end
        end
    end

    --============================== Octógono ==============================

    if figura == "octágono" then

        if robot.id == "Footbot1" then 
            if PO_counter_seg < 4 then
                yellow_true = true
            end
        end

        if robot.id == "Footbot2" then 

            if PO_counter_seg < 7 and PO_counter_seg >= 3 then
                yellow_true = true
            end
        end

        if robot.id == "Footbot3" then 

            if not FG_first_time and PO_counter_seg < 2 then
                yellow_true = true
            end
            if PO_counter_seg == 3 then
                FG_first_time = false
            end
            if PO_counter_seg >= 6 then
                yellow_true = true
            end
        end
    end

    --============================== triangulo_irregular_1 ==============================
    if figura == "triangulo_irregular_1" then

        if robot.id == "Footbot1" then 

            if PO_counter_seg < 2 then
                FG_timer = 0
                yellow_true = true
            end

            if  PO_counter_seg == 2 and FG_timer < 25 then
                FG_timer = FG_timer + 1 
                yellow_true = true
            end
        end

        if robot.id == "Footbot2" then 

            if PO_counter_seg < 3 and PO_counter_seg >= 1 then
                FG_timer = 0
                yellow_true = true
            end

            if PO_counter_seg == 3 and FG_timer < 25 then
                FG_timer = FG_timer + 1 
                yellow_true = true
            end
        end

        if robot.id == "Footbot3" then 

            if FG_first_time and PO_counter_seg == 1 then
                FG_first_time = false
            end

            if FG_first_time == false then
                if PO_counter_seg == 1  and FG_timer < 25 then
                    FG_timer = FG_timer + 1 
                    yellow_true = true
                end     
                if PO_counter_seg > 1  then
                    yellow_true = true
                end     
                if PO_counter_seg == 0 then
                    yellow_true = true
                    FG_timer = 0
                end    
            end
        end
    end

    --============================== Pentágono_irregular ==============================

    if figura == "Pentágono_irregular" then
    
        --log("\n Ejecutando Pentágono_irregular.."..robot.id)

        if robot.id == "Footbot1" then 

            if PO_counter_seg < 3  then
                yellow_true = true
            end

            if PO_counter_seg == 3 and FG_timer < 15 then
                FG_timer = FG_timer + 1 
                yellow_true = true
            end

            if PO_counter_seg == 4 then
                FG_timer = 0
            end
        end

        if robot.id == "Footbot2"then 

            if     PO_counter_seg >= 1 and PO_counter_seg < 3 then 
                yellow_true = true
            end

            if PO_counter_seg == 3 and movimiento_recorrido <= 0.5 then
                yellow_true = true
            end
        end

        if robot.id == "Footbot3" then 

            if     PO_counter_seg == 1 and movimiento_recorrido >= 0.6 then 
                yellow_true = true
            end

            if PO_counter_seg > 1 and PO_counter_seg < 4  then
                yellow_true = true
            end

            if PO_counter_seg == 4 and movimiento_recorrido <= 0.3 then
                yellow_true = true
            end
        end


        if robot.id == "Footbot5" then

            if  not FG_first_time then 
                if PO_counter_seg < 1 then
                    yellow_true = true
                end
                if PO_counter_seg == 1 and movimiento_recorrido < 0.4 then
                    yellow_true = true
                end
            end

            if  PO_counter_seg >= 4 and PO_counter_seg <= 5  then
                yellow_true = true
                FG_first_time = false
            end
        end
    end

    --============================== Heptágono ==============================

    if figura == "Heptágono" then

        if robot.id == "Footbot1" then 

            if PO_counter_seg < 3  then
                yellow_true = true
            end

            if PO_counter_seg == 3 and FG_timer < 35 then
                FG_timer = FG_timer + 1 
                yellow_true = true
            end
        end

        if robot.id == "Footbot2" then 

            if  PO_counter_seg >= 1 and PO_counter_seg < 4 then 
                yellow_true = true
            end

            if PO_counter_seg == 4 and movimiento_recorrido <= 0.5 then
                yellow_true = true
            end
        end

        if robot.id == "Footbot3"  then 

            if     PO_counter_seg == 2 and movimiento_recorrido >= 0.5 then 
                yellow_true = true
            end

            if PO_counter_seg > 2 and PO_counter_seg < 6  then
                yellow_true = true
            end

            if PO_counter_seg == 6 and movimiento_recorrido <= 0.4 then
                yellow_true = true
            end
        end

        if robot.id == "Footbot5" then 

            if PO_counter_seg < 1 and not FG_first_time and movimiento_recorrido < 0.4 then
                yellow_true = true
            end

            if  PO_counter_seg == 3 and movimiento_recorrido >= 0.4 then 
                yellow_true = true
            end

            if  PO_counter_seg > 3 and PO_counter_seg <= 7  then
                yellow_true = true
                FG_first_time = false
            end
        end        
    end

    --============================== trayectoria_1 ==============================
    if figura == "trayectoria_1" then

        if robot.id == "Footbot1" then 

            if PO_counter_seg < 2  then
                yellow_true = true
            end

            if PO_counter_seg == 2 and movimiento_recorrido <= 0.4 then
                yellow_true = true
            end
        end

        if robot.id == "Footbot2" then 

            if  PO_counter_seg == 0 and movimiento_recorrido > 0.4 then 
                yellow_true = true
            end

            if PO_counter_seg < 4 and PO_counter_seg > 0 then
                yellow_true = true
            end

            if PO_counter_seg == 4 and movimiento_recorrido <= 0.6 then
                yellow_true = true
            end

        end

        if robot.id == "Footbot3" then 

            if     PO_counter_seg == 2 and movimiento_recorrido >= 0.3 then 
                yellow_true = true
            end

            if PO_counter_seg >= 3 then
                yellow_true = true
            end
        end
    end

    if yellow_true == true then
        setleds("yellow")
    else
        setleds("black")
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


    if figura == "octágono"then
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
            name = "octógono"
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

-------------------------------------------------------------------------------------------------------------------------------------
-------------------------------------------------------------------------------------------------------------------------------------
-- Funciones de operaciones y calculos
-------------------------------------------------------------------------------------------------------------------------------------
-------------------------------------------------------------------------------------------------------------------------------------

-- Función para convertir de polares a cartesianas, pares de valores , theta en radianes.
function polarACartesianoPAR(r, theta)
    local x
    local y

    y = r * math.sin(theta)
    x = r * math.cos(theta)

    --log("x = " .. x .. ", y =" .. y)
    return x,y
end

-- Función para convertir de cartesianas a polares, pares de valores
function cartesianoAPolarPAR(x, y)
    local r = math.sqrt(x^2 + y^2)
    local theta = math.atan2(y, x)  -- Ángulo en radianes

    --log("r = " .. r .. ", theta (rad) = " .. theta)
    return r, theta
end

-- Función para calcular el ángulo a partir del vector de orientación
ang = 0
function vec2_angle(v)
    -- Convertir el cuaternión a un ángulo en el plano XY
    ang = math.atan2(2 * (v.w * v.z + v.x * v.y),
        1 - 2 * (v.y ^ 2 + v.z ^ 2))
    -- Convertir el ángulo de radianes a grados
    ang = ang * 180 / math.pi
    return ang
end

-- Función para aproximar valores
function redondear_personalizado(valor)
    local entero = math.floor(valor)
    local decimal = valor - entero

    if decimal > 0.66  then
        return math.ceil(valor)
    elseif decimal < 0.33 then 
        return math.floor(valor)
    else
        return entero + 0.5    
    end
end
-------------------------------------------------------------------------------------------------------------------------------------
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