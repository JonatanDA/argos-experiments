--=============================================
--   MEJORAS :  -- Función de movimiento más robusta, con control de velocidad y corrección de trayectoria
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

ticks_per_second                    = 5                        -- Número de ticks que representa un segundo en la simulación (ajusta según la configuración de .argos)
max_time                            = 5000 * ticks_per_second   -- Duración máxima de la simulación en ticks
Num_sensores_de_proximidad          = 24                        -- Numero de sensores de proximidad del robot

------------------------------
--Parámetro de simulación internos

speed_dem = 5                   -- Velocidad del demostrador al ejecutar el patrón
d_speed   = 15                  -- velocidad normal
g_speed   = 30                  -- Velocidad de giro

-- Métricas de paciencia
MAX_PATIENCE        = 600       -- Tiempo máximo de espera sin recibir datos útiles
BASE_RESTORE        = 200       -- Paciencia base a restaurar con datos refinados/actualizados

-- 
demostrador_threshold = 41      -- Distancia máxima a la que a la que el maestro espera ver al observador
threshold             = 40      -- Distancia a la que se ubica el observador del maestro
max_tracking_distance = 165     -- Máxima distancia a la que se puede alejar el maestro antes de que el alumno se acerque para no perderlo de vista. cm
min_tracking_distance = 30      -- Miínima distancia a la que se puede acercar el maestro antes de que el alumno se aleje para no colisionar. cm
porcentaje            = 0.5     -- 1 < porcentaje < 0.33 , indica que tanto el observador busca acercarse al maestro cuando este cruza el max_tracking_distance.


-- Sensor noise config 
ENABLE_NOISE            = true -- Si es false, el robot ve perfecto (Ground Truth)
USE_OUTLIERS            = false -- Activar lecturas "fantasma" o fallos graves
USE_DISTANCE_DEPENDENT  = true -- El ruido aumenta si el objeto está lejos
USE_ANGLE_NOISE         = true -- Ruido en la medición del ángulo
USE_BIAS                = true -- Error sistemático (ej. sensor mal calibrado +2cm siempre)

    -- Parámetros de Intensidad
BASE_ERROR_DIST     = 1.0   -- Ruido base en distancia (cm)
NOISE_LEVEL_DIST    = 0.01  -- % de error proporcional a la distancia
NOISE_LEVEL_ANGLE   = 0.5   -- ±2 grados de desviación estándar
OUTLIER_PROB        = 0.01  -- 1% de probabilidad de fallo catastrófico
BIAS_OFFSET         = 2     -- Cm extra que agrega el sensor por defecto (si USE_BIAS es true)

------------------------------
--Parámetro automáticos calculados a partir de los anteriores

-- Distancia de referencia para el cálculo (rango de trabajo común)
local DIST_REF = 150  -- cm
local sigma_d = BASE_ERROR_DIST + ( USE_DISTANCE_DEPENDENT and DIST_REF * NOISE_LEVEL_DIST or 0 ) -- Desviación estándar máxima en distancia a 150cm
local err_lin = (3 * sigma_d) + (USE_BIAS and math.abs(BIAS_OFFSET) or 0)
local err_lat = USE_ANGLE_NOISE and (DIST_REF * math.tan(math.rad(3 * NOISE_LEVEL_ANGLE))) or 0

ERROR_MAX_ESTIMADO = math.sqrt(err_lin^2 + err_lat^2) -- Representa la desviación máxima (3-sigma) en cm a una distancia de 150cm.
log(string.format("Error Máximo Estimado (a %dcm): %.2f cm", DIST_REF, ERROR_MAX_ESTIMADO))

-- Parámetros de segmentación y filtrado
min_puntos = 4   -- Cantidad mínima de puntos que debe tener un segmento para ser válido

--
angulo_treshold         = 10                                                            -- Sensibilidad del observador a caambios en los datos de angulo obtenidos del maestro, ajuste recomendado: 10 < distance_treshold < 50
distance_treshold       = ERROR_MAX_ESTIMADO + (speed_dem+d_speed)/ticks_per_second     -- Sensibilidad del observador a caambios en los datos de distancia obtenidos del maestro, ajuste recomendado: 1 < distance_treshold < 2 durante la etapa de recolección
sensitivity             = (speed_dem ) / (2 * ticks_per_second )
sensitivity_2           = (speed_dem ) / ( ticks_per_second )
filter_after_detect_seg = true                                                          -- Si es true, se aplica el filtrado de ruido antes de detectar los segmentos. Si es false, se detectan los segmentos con los datos crudos.-- 

CONFIG = {
    TOL_DIST                = distance_treshold  ,  -- ¿Qué tan "borrosos" pueden ser los datos de distancia para que encajen? (cm)
    TOL_ANG                 = angulo_treshold,      -- ¿Qué tan "borrosos" pueden ser los datos de ángulo para que encajen? (°)
    MIN_OVERLAP             = 1,                    -- ¿Qué tan poco contacto necesitan dos piezas para pegarse?
    PRINT_RECURSIVE_PATHS   = true,                 -- BANDERA: true = Imprime los pasos de la solución elegida
    FORCE_M2_ALWAYS         = false,                -- Ejecutar M2 incluso si M1 fue perfecto
    selection_mode          = 1,                    -- 1 = Seleccionar la solución más repetida, 2 = solución más larga, 3 =  /
    THRESHOLD               = 0.8,                  -- 80% de los pares deben ser similares para considerarlo periódico
    MATH_TOLERANCE          = 0.5                   -- Tolerancia para considerar que n (lados) es un número entero
}

-- Banderas de loggeo
DS_LOG_MODE     = 1 -- 1: Sin logs, 1: Logs detallados, 2: Solo resumen final, 3: Resumen + puntos en segmento, 4: Solo parámetros de Otsu
FVS_LOG_MODE    = 1 -- 1: Sin logs, 1: Logs detallados, 2: Solo resumen final, 3: Resumen + detalles de filtrado

displayed_data  = 1 -- 1 Datos obtenidos por el observador. 2. Datos globales del movimiento del demostrador

-- Parámetros de configuración del experimento
EXPERIMENTO     = 2 -- 1: Memoria de 1 patrón, 2: Memoria de N patrones, 3: Memoria ilimitada
N_PATRONES      = 5 -- Cantidad de patrones a recordar (Solo aplica si EXPERIMENTO == 2)

--Parametros de patrónes predefinidos
figura = "Heptágono" -- "triangulo_regular" -- "octágono" -- "triangulo_irregular_1" --"Pentágono_irregular"  -- "Heptágono"-- "" -- "trayectoria_1" 
bigger = 1.5 -- Factor de escala para los patrones predefinidos (ajusta según el tamaño del entorno y la velocidad del robot)
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
    robot.colored_blob_omnidirectional_camera.enable()
    determinar_rol()
end

-------------------------------------------------------------------------------------------------------------------------------------
-- Función principal que se ejecuta en cada tick
-------------------------------------------------------------------------------------------------------------------------------------

timer       = 1
all_params  = {} -- Tabla para almacenar todos los patrones aprendidos/originales
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
    print( "Maestro"..k.." & " .. var)
end

function Print_pattern_data(k,params,j)

    print( "pattern & "..k.." & " .. j)
    print_table(params)
    print( "end_data_pattern")

end
-------------------------------------------------------------------------------------------------------------------------------------
-- Funciones para cada rol
-------------------------------------------------------------------------------------------------------------------------------------

d1 = 0
c5 = 0
first_time_demonstrator = true
demostrador_avg_angle   = 0
demostrador_avg_dist    = 0
parametros_elegidos     = {}


function demostrador()
    if d1 == 0 then
        log("Buscando zona de demostración... " .. robot.id )
        detectar_zona_demostracion()

        if DZD_c1 == 1 then
            log(" Zona de demostración alcanzada! " .. robot.id)

            d1 = 1
        end

    elseif d1 == 1 then   
        espera_antes_de_ejecutar_patron()

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
            log(" ========= Ejecutando patrón aleatorio (1,"..#all_params..") , -> "..indice_aleatorio..", "..robot.id.." ========= ")
            --print_table(parametros_elegidos)
            
        else
            -- Respaldo de seguridad por si la tabla aún está vacía
            parametros_elegidos = original_params()
            parametros_elegidos = parametros_elegidos.params
            log(" ========= Ejecutando patrón original: " .. robot.id.." ========= ")
            --print_table(parametros_elegidos)

        end

        d1 = 3

    elseif d1 == 3 then
        patron_imitado(parametros_elegidos)

    elseif d1 == 4 then        
        log("Demostración finalizada ".. ", Buscando zona de de observación... " .. robot.id )
        detectar_zona_demostracion()
        if DZD_c1 == 1 then
            log(" Zona de observación alcanzada! " .. robot.id)
            is_demonstrator = 0
            resetOBS()
            resetDEM()
        end
    end
end


c3 = 0
c4 = 0
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

if robot.id == "Footbot200" then
    zona_dem = {x = 0.5, y = 0.0}
elseif robot.id == "Footbot100" then
    zona_dem = {x = -0.5, y = 0.0}
end
radio_tolerancia = 0.05   -- Tolerancia de 10 cm
velocidad_base = 10.0     -- Velocidad lineal de avance (ajusta según tu robot)
Kp = 10.0                 -- Constante proporcional para el giro

DZD_c1 = 0
function detectar_zona_demostracion()
    --log("Detectando zona de demostración... " .. robot.id)
    -- 1. Obtener la posición actual (plano global)
    local x = robot.positioning.position.x
    local y = robot.positioning.position.y
    
    -- 2. Obtener la orientación y extraer el ángulo Z (Yaw)
    local orientacion = robot.positioning.orientation
    
    -- CORRECCIÓN: Usar el método sin guiones bajos
    local yaw, pitch, roll = orientacion:toeulerangles()
    local theta = yaw 
    
    -- 3. Calcular la distancia euclidiana hacia el objetivo
    local dx = zona_dem.x - x
    local dy = zona_dem.y - y
    local distancia = math.sqrt((dx * dx) + (dy * dy))
            
    log(string.format("Posición: (%.2f, %.2f), Distancia al objetivo: %.2f cm", x, y, distancia * 100)..", "..robot.id)

    -- 4. Evaluar si el robot ya está dentro del círculo de tolerancia
    if distancia <= radio_tolerancia then
        -- ¡Objetivo alcanzado!
        DZD_c1 = 1
        -- Detener los motores para no seguir iterando ni pasarse de largo
        robot.wheels.set_velocity(0, 0)
        
    else
        -- 5. Lógica de Navegación hacia el punto
        
        -- Calcular el ángulo absoluto hacia la coordenada objetivo
        local angulo_objetivo = math.atan2(dy, dx)
        
        -- Calcular cuánto tiene que girar el robot
        local error_angulo = angulo_objetivo - theta
        
        -- Normalizar el ángulo (entre -PI y PI) para que gire por el camino más corto
        while error_angulo > math.pi do
            error_angulo = error_angulo - (2 * math.pi)
        end
        while error_angulo < -math.pi do
            error_angulo = error_angulo + (2 * math.pi)
        end
        
        -- Calcular las velocidades para las ruedas
        local vel_giro = Kp * error_angulo
        local vel_avance = velocidad_base
        
        -- Si el robot está muy desalineado (más de 45 grados de error),
        -- frenamos el avance para que rote sobre su propio eje primero.
        if math.abs(error_angulo) > (math.pi / 4) then
            vel_avance = 0.0
        end
        
        -- Asignar velocidades a cada rueda (cinemática diferencial)
        local vel_izq = vel_avance - vel_giro
        local vel_der = vel_avance + vel_giro
        
        -- Aplicar el movimiento
        robot.wheels.set_velocity(vel_izq, vel_der)
    end
end

EAEP_c1 = 0
EAEP_c2 = 0
function espera_antes_de_ejecutar_patron()
    
    if EAEP_c2 == 0 then

        local blobs = robot.colored_blob_omnidirectional_camera
        setleds("black")

        for i, blob in ipairs(blobs) do
            local r, g, b = blob.color.red, blob.color.green, blob.color.blue

            if r < 100 and g < 100 and b > 200 then  -- Si detecta un objeto azul
                EAEP_c2 = 1
                setleds("red")
                break
            end
        end

    elseif EAEP_c2 == 1 then
        Temporizador_1(5)
        if temp == false then
            EAEP_c2 = 2
        end
    else
        d1 = 2
        setleds("yellow")
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

            figures()

            if c1 == 2 then
                PO_counter_seg = PO_counter_seg + 1
                if ind == #params then
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
    
    if displayed_data == 2 then
        if robot.id == "Footbot100" then Print_displayed_data(1, x, y)  end
        if robot.id == "Footbot200" then Print_displayed_data(2, x, y)  end
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

                if c1 == 2 then
                    if indice == #params then

                        indice = indice + 1
                        PI_end = true
                        r_Temporizador_1()

                                    
                        if c5 == 1 then
                            k = 1
                            log(" ========================================================================= ")
                            log(" Patrón Imitado: finalizado por: ".. robot.id)                            
                            --imprimir_posicion(posiciones_guardadas)

                        else
                            c5 = 1
                            k = 1
                            log(" ========= Patrón: finalizado por: ".. robot.id.." ========= ")
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
    DZD_c1 = 0
    if k == 1 and PI_c1 == 1 then
        Temporizador_1(10)
        if temp == false then
            d1 = 4
        end
    elseif k == 1 and PI_c1 == 0 then
        d1 = 4
        log("DEMOSTRACIÓN INTERRUMPIDA, Buscando zona de demostración para observación... " .. robot.id )

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

-- A. Generador de Distribución Normal (Algoritmo Box-Muller)
-- Transforma dos números uniformes (math.random) en uno gaussiano
function gaussian(mean, variance)
    local u1 = math.random()
    local u2 = math.random()
    -- Seguridad para evitar log(0)
    if u1 <= 0 then u1 = 0.000001 end
    
    local z0 = math.sqrt(-2.0 * math.log(u1)) * math.cos(2.0 * math.pi * u2)
    return z0 * variance + mean
end

-- B. Función Gestora de Ruido
function aplicar_ruido(dist_real, angulo_real)
    -- 1. Si el ruido está apagado globalmente, devolver valores reales
    if not ENABLE_NOISE then 
        return dist_real, angulo_real 
    end

    -- 2. OUTLIERS: Fallos catastróficos (Salt & Pepper)
    if USE_OUTLIERS then
        if math.random() < OUTLIER_PROB then
            -- El sensor "alucina": devuelve una distancia enorme y ángulo random
            return 1000.0, math.random(-180, 180) 
        end
    end

    local dist_final = dist_real
    local angulo_final = angulo_real

    -- 3. RUIDO EN DISTANCIA
    if USE_DISTANCE_DEPENDENT then
        -- La desviación estándar (sigma) crece con la distancia
        -- Base de BASE ERROR DIST cm de ruido + el porcentaje definido
        local sigma = BASE_ERROR_DIST + (dist_real * NOISE_LEVEL_DIST)
        local error = gaussian(0, sigma)
        dist_final = dist_final + error
        
        -- Corrección física: la distancia no puede ser negativa
        if dist_final < 0 then dist_final = 0 end
    else
        -- Si está apagado el dependiente, ¿quizás quieras ruido fijo simple?
        -- Si no, lo dejamos igual. Aquí agregamos un ruido fijo pequeño por defecto
        dist_final = dist_final + gaussian(0, BASE_ERROR_DIST) 
    end

    -- 4. RUIDO EN ÁNGULO
    if USE_ANGLE_NOISE then
        -- Ruido gaussiano con media 0 y desviación configurada
        local error_angular = gaussian(0, NOISE_LEVEL_ANGLE)
        angulo_final = angulo_final + error_angular
    end

    -- 5. BIAS (Sesgo sistemático)
    if USE_BIAS then
        dist_final = dist_final + BIAS_OFFSET
    end

    return dist_final, angulo_final
end



master_tracking = {}

-- 1. INICIALIZACIÓN
function initialize_master_tracking(k, angle_deg, distance, ty)
    local angle_rad = angle_deg * (math.pi / 180)
    local x, y = polarACartesianoPAR(distance, angle_rad) 

    master_tracking[k] = {
        -- Datos crudos (Almacén de toda la historia)
        raw_x = {x},
        raw_y = {y},
        raw_type = {ty},

        -- Variables de referencia
        x_ant_r = 0,
        y_ant_r = 0,
        srt_ang_mark_ant = 0,
        x_mark = 0, 
        y_mark = 0, 
        
        -- Banderas de estado
        Pattern_start_detected = (ty == 1), -- Si el primer dato es tipo 1, ya vimos el inicio
        End_of_pattern_detected = (ty == 3), -- Si el primer dato es tipo 3, ya vimos el final (raro pero posible)
        processed = false, -- Para asegurar que solo procesamos una vez al final
        
        -- Estructuras para el procesador (se llenarán al final)

        segmentos = { movement = {}, type = {}, valid = {} },
        params = {},
    }
end

-- =====================================================================
-- CONFIGURACIÓN DE EXPERIMENTOS DE MEMORIA
-- =====================================================================

function process_full_trajectory(k, track)
    log("\n ----------------------------------------------- ")
    log(" PROCESANDO MAESTRO "..k.." (Datos totales: "..#track.raw_x..")")
    log(" ----------------------------------------------- ")

    -- 1. SEGMENTACIÓN (MÉTODO NUEVO)
    local clean_mov = {}
    local clean_x,clean_y = {}, {}

    if filter_after_detect_seg == true then 
        clean_x,clean_y = filtrar_ruido(track.raw_x, track.raw_y, sensitivity_2)
        clean_mov = detect_seg(clean_x, clean_y)
    else
        clean_mov = detect_seg(track.raw_x, track.raw_y)
    end

    -- 2. ASIGNACIÓN AL TRACK
    track.segmentos.movement = clean_mov
    track.segmentos.type = {}
    track.segmentos.valid = {}

    -- 4. CÁLCULOS MATEMÁTICOS
    if #track.segmentos.movement > 1 then 

        -- a) Obtener Ecuaciones y las columnas sobrevivientes
        local equations, valid_cols = apply(track.segmentos.movement)

        -- b) Calcular Intersecciones pasando la nueva variable
        local intersections = calcular_intersecciones_con_extremos(equations, track.segmentos.movement, valid_cols)

        -- c) Calcular Distancias y Ángulos
        local distancias, angulos = calcular_distancias_y_angulos(intersections)

        -- d) Crear Parámetros (pasando valid_cols)
        track.params = create_params(distancias, angulos, valid_cols)

        -- e) PRE-PROCESAMIENTO NUMÉRICO Y DATABUFFER
        if #track.params > 0 then
            
            -- Creamos una tabla nueva para evitar el problema de referencias en Lua
            local copia_parametros = {}
            for i, par in ipairs(track.params) do
                table.insert(copia_parametros, {par[1], par[2]}) 
            end
            
            -- =====================================================================
            -- GESTIÓN DE MEMORIA SEGÚN EL EXPERIMENTO
            -- =====================================================================
            local k_nuevo
            
            if EXPERIMENTO == 1 then
                -- Exp 1: Solo un patrón a la vez (olvida todo lo anterior)
                all_params = {} -- Vaciamos la memoria
                table.insert(all_params, copia_parametros)
                k_nuevo = 1
                
            elseif EXPERIMENTO == 2 then
                -- Exp 2: N patrones guardados (FIFO)
                table.insert(all_params, copia_parametros)
                -- Si excedemos la capacidad N, eliminamos el más antiguo (índice 1)
                if #all_params > N_PATRONES then
                    table.remove(all_params, 1)
                end
                k_nuevo = #all_params
                
            elseif EXPERIMENTO == 3 then
                -- Exp 3: Memoria ilimitada (Comportamiento normal)
                table.insert(all_params, copia_parametros)
                k_nuevo = #all_params
            end

            -- =====================================================================

            if robot.id == "Footbot100" then 
                Print_pattern_data(1, all_params[k_nuevo], k_nuevo)
            elseif robot.id == "Footbot200" then
                Print_pattern_data(2, all_params[k_nuevo], k_nuevo)
            end
            log("[GESTOR] ¡Nuevo patrón aprendido y guardado en all_params[" .. k_nuevo .. "]! (Exp: " .. EXPERIMENTO .. ")\n\n\n")

        else
            log("No se pudieron crear parámetros útiles a partir de los segmentos procesados.")
        end
    else
        log("No hay segmentos suficientes para procesar tras la segmentación.")
    end        

    -- ACTIVAR ESTADO FINAL DEL ROBOT
    resetOBS()
    is_demonstrator = 1
end

-- 2. FUNCIÓN AUXILIAR DE store_relative_positions: FUNCIÓN DE GRABACIÓN 
function process_master_segmentation(k, track, x_original, y_original, ty)
    -- TAREA ÚNICA: ACUMULAR DATOS
    table.insert(track.raw_x, x_original)
    table.insert(track.raw_y, y_original)
    table.insert(track.raw_type, ty)

    --log(string.format("Maestro %d: Nuevo dato agregado - X: %.2f, Y: %.2f, Type: %d", k, x_original, y_original, ty))

    -- Actualizar banderas en tiempo real si aparecen blobs especiales
    if ty == 1 then track.Pattern_start_detected = true end
    if ty == 3 then track.End_of_pattern_detected = true end
end

-- Función para actualizar la paciencia desde el DataBuffer
current_patience    = MAX_PATIENCE
function update_patience(restore_type, count)
    if restore_type == "full" then
        current_patience = MAX_PATIENCE
        --log("[PACIENCIA] Restaurada completamente al máximo (" .. MAX_PATIENCE .. ").")
    elseif restore_type == "partial" then
        -- Restauración parcial con rendimiento decreciente
        local divisor = (count > 1) and (count - 1) or 1
        local restore_amount = math.floor(BASE_RESTORE / divisor)
        
        current_patience = math.min(MAX_PATIENCE, current_patience + restore_amount)
        --log("[PACIENCIA] Restauración parcial: +" .. restore_amount .. ". Actual: " .. current_patience)
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
    
-- =========================================================================
-- 5. BUCLE PRINCIPAL (Store Relative Positions)
-- =========================================================================
srt_anterior_data = {}

function store_relative_positions()

    -- Control de paciencia y reconstrucción global dinámica
    --check_patience_and_reconstruct()

    -- (Lógica de timer y proximidad...)
    local proximity = proximidad()
    if proximity[1] < 1 then srt_c5 = 0 else
        if srt_c5 == 5 * ticks_per_second then resetOBS() return end
        srt_c5 = srt_c5 + 1
    end

    setleds("blue")
    robot.wheels.set_velocity(0, 0)
    
    if robot.colored_blob_omnidirectional_camera then                
        local blobs = robot.colored_blob_omnidirectional_camera
        local yellow_blobs = {} 
        local green_blobs = {}  
        local count = 0   

        -- 1. RECOLECCIÓN Y RUIDO
        for i, blob in ipairs(blobs) do
            local raw_angle = blob.angle * (180 / math.pi) -- Ángulo en grados
            local raw_dist = blob.distance
            local noisy_dist, noisy_angle = aplicar_ruido(raw_dist, raw_angle)

            -- CÁLCULO DE LA DISTANCIA DEL ERROR (Ley de los Cosenos)
            -- math.rad() convierte la diferencia de grados nuevamente a radianes para math.cos
            local diff_angle_rad = math.rad(noisy_angle - raw_angle)
            local error_dist = math.sqrt(raw_dist^2 + noisy_dist^2 - 2 * raw_dist * noisy_dist * math.cos(diff_angle_rad))
            
            local r, g, b = blob.color.red, blob.color.green, blob.color.blue
            
            -- Blob amarillo (Tipo 2)
            if r > 200 and g > 200 and b < 100 then 
                table.insert(yellow_blobs, {angle = noisy_angle, distance = noisy_dist, type = 2})
                -- Imprimimos el error redondeado a 2 decimales
                --log("\n Blob Amarillo (i=" .. i .. ") | Desplazamiento por ruido: " .. string.format("%.2f", error_dist) .. " cm")

            -- Blob verde (Tipo 3)
            elseif r < 100 and g > 200 and b < 100 then 
                table.insert(green_blobs, {angle = noisy_angle, distance = noisy_dist, type = 3})

            -- Blob rojizo guardado como amarillo (Tipo 1)
            elseif  r > 200 and g < 100 and b < 100  then 
                table.insert(yellow_blobs, {angle = noisy_angle, distance = noisy_dist, type = 1})
                --log("Blob Rojo->Amarillo (i=" .. i .. ") | Desplazamiento por ruido: " .. string.format("%.2f", error_dist) .. " cm")
            end 
        end

        local updated_anterior_indices = {}
        local matched_new_indices = {}

        -- 0. LOG DE ENTRADA
        --log(string.format("[DEBUG] Iniciando actualización: Yellow Blobs: %d", #yellow_blobs))

        -- 2. ACTUALIZACIÓN DE MAESTROS EXISTENTES (Yellow)
        for j, new_data in ipairs(yellow_blobs) do
            
            -- Ahora iteramos sobre TODOS los maestros anteriores, sin importar el signo
            for k, anterior_data in pairs(srt_anterior_data) do
                
                -- Verificamos que tenga datos y que no se haya actualizado ya en este tick
                if anterior_data and anterior_data.angle and not updated_anterior_indices[k] then
                    
                    -- A. COMPARACIÓN DE ÁNGULO (Cálculo robusto de diferencia)
                    -- Restamos los ángulos y sacamos el valor absoluto de la DIFERENCIA.
                    -- (Nota: Esta fórmula simple funciona perfecto si los ángulos no saltan repentinamente 
                    -- de 180 a -180. Si laa cámara hace ese salto en la espalda,  añadimos módulo matemático).
                    local diff_angle = math.abs(new_data.angle - anterior_data.angle)

                    -- CORRECCIÓN: Ajuste modular para el cruce de 180 a -180 grados
                    if diff_angle > 180 then
                        diff_angle = 360 - diff_angle
                    end

                    if diff_angle <= angulo_treshold then
                        
                        -- B. COMPARACIÓN DE DISTANCIA
                        local diff_distance = math.abs(new_data.distance - anterior_data.distance)
                        --log("Distancia anterior: "..anterior_data.distance.." cm, Nueva distancia: "..new_data.distance.." cm, Diff: "..string.format("%.2f", diff_distance).." cm")
                        if diff_distance <= distance_treshold then
                            -- ¡MATCH ENCONTRADO! ACTUALIZAR DATOS
                            -- log(string.format("\n[OK] Maestro %d actualizado: Angulo actual -> %.2f, Angulo anterior -> %.2f", k, new_data.angle, srt_anterior_data[k].angle)..", "..robot.id)
                            -- log(string.format("  [OK] Maestro %d actualizado: Distan actual -> %.2f, Distan anterior -> %.2f", k, new_data.distance, srt_anterior_data[k].distance)..", "..robot.id)

                            srt_anterior_data[k].angle = new_data.angle
                            srt_anterior_data[k].distance = new_data.distance
                            srt_anterior_data[k].type = new_data.type 
                            
                            local x, y = polarACartesianoPAR(srt_anterior_data[k].distance / 100, srt_anterior_data[k].angle * math.pi / 180)
                            
                            if displayed_data == 1 then
                                Print_displayed_data(k, x, y)
                            end 
                            
                            updated_anterior_indices[k] = true
                            matched_new_indices[j] = true
                            count = count + 1
                            break -- Rompe el bucle de maestros para pasar al siguiente blob detectado
                        else
                            log(string.format("  [DIST_FAIL] M%d: Falla por distancia (Diff: %.2f cm > Treshold: %.2f)", k, diff_distance, distance_treshold))
                            
                            
                            is_demonstrator = 1 -- Si falla por distancia, asumo que el patrón no se está siguiendo bien, así que me convierto en demostrador para forzar el aprendizaje del patrón correcto.
                            resetOBS() 
                            resetDEM()
                        end
                    else
                        -- Comentado para no saturar los logs, actívalo solo si necesitas mucho detalle:
                        log(string.format("  [ANG_FAIL] M%d: Falla por ángulo (Diff: %.2f° > Treshold: %.2f°)", k, diff_angle, angulo_treshold)..", "..robot.id)


                        is_demonstrator = 1 -- Si falla por distancia, asumo que el patrón no se está siguiendo bien, así que me convierto en demostrador para forzar el aprendizaje del patrón correcto.
                        resetOBS() 
                        resetDEM()
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
                --log(string.format("Nuevo maestro creado: ID %d, Ángulo %.2f°, Distancia %.2f cm, Tipo %d", new_k, new_data.angle, new_data.distance, new_data.type or 2))
            end
        end

        -- 4. PROCESAR BLOBS VERDES (FINAL DE PATRÓN)
        for _, green_blob in ipairs(green_blobs) do
            -- (Lógica de macheo igual que antes)
            for k, master_data in pairs(srt_anterior_data) do
                if master_data.angle and not updated_anterior_indices[k] then -- Solo si no ha sido actualizado por amarillo aun
                    -- ... (lógica de thresholds para verde aquí, simplificada por brevedad) ...
                    -- Asumimos match:
                     local max_ang = math.abs(green_blob.angle) + angulo_treshold
                     local min_ang = math.abs(green_blob.angle) - angulo_treshold
                     -- ... chequeos de distancia ...
                     -- Si match:
                     srt_anterior_data[k].type = 3 -- Marcar como final
                     updated_anterior_indices[k] = true 
                     master_tracking[k].End_of_pattern_detected = true -- Bandera crítica
                end
            end 
        end 

        -- =========================================================================
        -- 5. GESTIÓN DE MAESTROS PERDIDOS O FINALIZADOS 
        -- =========================================================================
        for k = 1, #srt_anterior_data do
            local track = master_tracking[k]
            
            -- Condición A: Maestro perdido (no actualizado en este tick)
            -- Condición B: Maestro marcó fin (Blob verde detectado)
            local is_lost = (srt_anterior_data[k] and srt_anterior_data[k].angle and not updated_anterior_indices[k])
            local is_finished = track.End_of_pattern_detected
            
            if (is_lost or is_finished) and track and not track.processed then
                
                -- EJECUTAR PROCESAMIENTO PESADO
                --log(string.format("\n[INFO] Maestro %d marcado como %s. Iniciando procesamiento final...", k, is_finished and "FINALIZADO" or "PERDIDO"))
                process_full_trajectory(k, track)
                
                -- mi rol cambió a demostrador y mis tablas se limpiaron. Solo entonces aborto.
                if is_demonstrator == 1 then
                    resetOBS() -- Limpio mis datos para no contaminar el aprendizaje del nuevo patrón. Solo lo hago si detecto que el patrón actual no se está siguiendo, para evitar resetearme innecesariamente.
                    return
                end

                track.processed = true -- Evitar reprocesar
                
                -- PROTEGER LA LIMPIEZA: verificar si srt_anterior_data y el índice k aún existen
                if srt_anterior_data and srt_anterior_data[k] then
                    srt_anterior_data[k].angle = nil
                    srt_anterior_data[k].distance = nil
                    srt_anterior_data[k].type = nil
                end
            end
        end

        -- =========================================================================
        -- 6. GRABACIÓN DE DATOS (Para maestros activos)
        -- =========================================================================
        for k, _ in pairs(updated_anterior_indices) do
            local master_data = srt_anterior_data[k]
            local track = master_tracking[k]

            -- Solo guardamos si el maestro está activo y NO hemos terminado
            if master_data and track and not track.processed then
                
                local current_distance = master_data.distance
                local current_angle_rad = master_data.angle * (math.pi / 180)
                local ty = master_data.type
                
                -- Conversión a coordenadas globales relativas al robot
                local ang_total = current_angle_rad + track.srt_ang_mark_ant
                local x_new, y_new = polarACartesianoPAR(current_distance, ang_total)
                local x_original = x_new + track.x_mark
                local y_original = y_new + track.y_mark

                -- LLAMADA NUEVA (Solo guarda)
                process_master_segmentation(k, track, x_original, y_original, ty)
            end
        end
    end 
end


-------------------------------------------------------------------------------------------------------------------------------------
-- FUNCIONES MATEMÁTICAS Y ESTADÍSTICAS
-------------------------------------------------------------------------------------------------------------------------------------

mostrar_todos_los_puntos = false 

function imprimir_resumen_segmentos(titulo, lista_segmentos, lista_tipos, lista_validos)
    log("\n=======================================================")
    log(" " .. titulo)
    log("=======================================================")
    
    if not lista_segmentos or #lista_segmentos == 0 then
        log(" >> No hay segmentos para mostrar.")
        log("=======================================================\n")
        return
    end

    for idx, points in ipairs(lista_segmentos) do
        local num_puntos = #points
        
        -- Formateo de la etiqueta de validez (si existe)
        local etiqueta_validez = ""
        if lista_validos and lista_validos[idx] ~= nil then
            if lista_validos[idx] then 
                etiqueta_validez = " [VALIDO]" 
            else 
                etiqueta_validez = " [INVALIDO]" 
            end
        end

        if num_puntos > 0 then
            local start_pt = points[1]
            local end_pt = points[num_puntos]

            log(string.format("SEGMENTO #%d%s", idx, etiqueta_validez))
            log(string.format(" > Total Puntos: %d", num_puntos))
            
            -- Lógica para imprimir con o sin 'tipos'
            if lista_tipos and lista_tipos[idx] then
                local types_list = lista_tipos[idx]
                local start_type = types_list[1] or -1
                local end_type = types_list[num_puntos] or -1
                log(string.format(" > Inicio: (%.3f, %.3f) [Type: %d]", start_pt[1], start_pt[2], start_type))
                log(string.format(" > Fin:    (%.3f, %.3f) [Type: %d]", end_pt[1], end_pt[2], end_type))
            else
                -- Impresión limpia si no hay información de 'type'
                log(string.format(" > Inicio: (%.3f, %.3f)", start_pt[1], start_pt[2]))
                log(string.format(" > Fin:    (%.3f, %.3f)", end_pt[1], end_pt[2]))
            end
            
            if mostrar_todos_los_puntos then
                local buffer_linea = " > TRAYECTORIA: "
                for p_idx, p in ipairs(points) do
                    buffer_linea = buffer_linea .. string.format("(%.2f, %.2f) ", p[1], p[2])
                    -- Salto de línea cada 5 puntos para no saturar la consola
                    if p_idx % 5 == 0 and p_idx < num_puntos then
                        log(buffer_linea)
                        buffer_linea = "                "
                    end
                end
                log(buffer_linea)
            end
            log("-------------------------------------------------------")
        end
    end
    log("=======================================================\n")
end

-- ============================================================================
-- 0. FILTRO DE RUIDO FÍSICO (Auto-recuperable)
-- ============================================================================
function filtrar_ruido(xx, yy, max_displacement)
    local valid_xx, valid_yy = {}, {}
    
    if #xx == 0 then return valid_xx, valid_yy end
    
    if FVS_LOG_MODE == 1 then
        log("\n--- INICIO DE FILTRADO DE RUIDO ---")
        log(" > Total de puntos a evaluar: " .. #xx)
        log(" > Límite de desplazamiento permitido: " .. max_displacement .. " cm")
        log("-----------------------------------")
    end

    -- El primer punto siempre se toma
    table.insert(valid_xx, xx[1])
    table.insert(valid_yy, yy[1])

    if FVS_LOG_MODE == 1 then 
        log(string.format("[Punto 1] ACEPTADO (Origen) -> (%.2f, %.2f)", xx[1], yy[1]))
    end
    local descartados = 0
    
    for i = 2, #xx do
        local x_curr, y_curr = xx[i], yy[i]
        
        -- Comparamos con el punto físicamente anterior en el array
        local x_prev, y_prev = xx[i-1], yy[i-1] 
        
        local dist = math.sqrt((x_curr - x_prev)^2 + (y_curr - y_prev)^2)
        
        if dist <= max_displacement then
            table.insert(valid_xx, x_curr)
            table.insert(valid_yy, y_curr)

            if FVS_LOG_MODE == 1 then
                 log(string.format("[Punto %d] ACEPTADO. Distancia al anterior: %.4f cm <= %.2f cm | Guardado: (%.2f, %.2f)", i, dist, max_displacement, x_curr, y_curr))
            end
        else
            descartados = descartados + 1
            if FVS_LOG_MODE == 1 then
                --log(string.format("[Punto %d] DESCARTADO. Dist. al anterior: %.4f > %.2f | Ignorado: (%.2f, %.2f)", i, dist, max_displacement, x_curr, y_curr))
            end
        end
    end
    
    if FVS_LOG_MODE == 1 then
         log(string.format("--- FILTRADO COMPLETADO. Puntos válidos: %d | Descartados: %d ---\n", #valid_xx, descartados))
    end
    
    return valid_xx, valid_yy
end

function detect_seg(xx, yy)
    local segmentos = {} 
    local k = 1       
    local i = 1       
    
    -- === NUEVOS PARÁMETROS DE FILTRADO ===
    -- Ajusta estos valores según la escala de tu entorno/robot
    local max_ruido = ERROR_MAX_ESTIMADO  -- Distancia máxima (en las mismas unidades que x,y) para considerarse "ruido"
    -- =====================================

    local estaba_girando = true 
    local ultimo_x, ultimo_y = xx[1], yy[1] 
    
    segmentos[k] = {} 

    if DS_LOG_MODE == 1 then 
        log("\n--- INICIO DE SEGMENTACIÓN ---")
        log(" > Total de puntos recibidos: " .. #xx)
        log(" > Sensibilidad configurada: " .. sensitivity)
        log(" > Filtro Max Ruido: " .. max_ruido)
        log(" > Filtro Min Puntos: " .. min_puntos)
        log("------------------------------")
    end

    -- [TODA LA LÓGICA DE DETECCIÓN SE MANTIENE EXACTAMENTE IGUAL]
    while i < #xx do
        local x1, y1 = xx[i], yy[i]
        local x2, y2 = xx[i + 1], yy[i + 1]

        local distancia = math.sqrt((x2 - x1)^2 + (y2 - y1)^2)

        if distancia > sensitivity then
            if estaba_girando then
                if #segmentos[k] > 0 then
                    table.insert(segmentos[k], { ultimo_x, ultimo_y })
                    if DS_LOG_MODE == 1 then
                        log(string.format("[Punto %d] ARRANQUE. Último pto estable Cierra Seg #%d: (%.2f, %.2f)", i, k, ultimo_x, ultimo_y))
                    end
                    
                    k = k + 1
                    segmentos[k] = {}
                    if DS_LOG_MODE == 1 then
                        log(string.format(" >> Iniciando Segmento #%d...", k))
                    end
                end
                
                table.insert(segmentos[k], { ultimo_x, ultimo_y })
                if DS_LOG_MODE == 1 then
                    log(string.format("[Punto %d] ARRANQUE. Inicia Seg #%d en el pto estable: (%.2f, %.2f)", i, k, ultimo_x, ultimo_y))
                end
                
                if x1 ~= ultimo_x or y1 ~= ultimo_y then
                    table.insert(segmentos[k], { x1, y1 })
                    if DS_LOG_MODE == 1 then
                        log(string.format("[Punto %d] MOVIÉNDOSE. Agregado al Seg #%d: (%.2f, %.2f)", i, k, x1, y1))
                    end
                end
                
                estaba_girando = false
            else
                table.insert(segmentos[k], { x1, y1 })
                if DS_LOG_MODE == 1 then
                    log(string.format("[Punto %d] MOVIÉNDOSE. Agregado al Seg #%d: (%.2f, %.2f)", i, k, x1, y1))
                end
            end
            
        else
            ultimo_x, ultimo_y = x1, y1
            
            if not estaba_girando then
                estaba_girando = true
                
                if DS_LOG_MODE == 1 then
                    log(string.format("[Punto %d] SE DETUVO. Empezando a estabilizar en: (%.2f, %.2f)", i, x1, y1))
                end
            else
                if DS_LOG_MODE == 1 then
                    log(string.format("[Punto %d] QUIETO/GIRANDO. Ignorado: (%.2f, %.2f)", i, x1, y1))
                end     
            end
        end
        i = i + 1
    end
    
    if #xx > 0 then
        if estaba_girando then
            if #segmentos[k] > 0 then
                table.insert(segmentos[k], { ultimo_x, ultimo_y })
                if DS_LOG_MODE == 1 then
                    log(string.format("[Fin de data] Robot estabilizado. Último punto cierra el Seg #%d: (%.2f, %.2f)", k, ultimo_x, ultimo_y))
                end
            end
        else
            table.insert(segmentos[k], { xx[#xx], yy[#yy] })
            if DS_LOG_MODE == 1 then
                log(string.format("[Fin de data] Robot moviéndose. Último punto cierra el Seg #%d: (%.2f, %.2f)", k, xx[#xx], yy[#yy]))
            end 
        end
    end
    
    -- === NUEVA LÓGICA DE POST-PROCESAMIENTO Y FILTRADO ===
    local segmentos_filtrados = {}
    local indice_filtrado = 1

    if DS_LOG_MODE == 1 then
        log("\n--- INICIANDO FILTRADO DE RUIDO ---")
    end

    for idx, seg in ipairs(segmentos) do
        local es_valido = true
        local dist_total = 0

        -- 1. Validar por cantidad de puntos
        if #seg < min_puntos then
            es_valido = false
            if DS_LOG_MODE == 1 then
                log(string.format(" >> Seg #%d ELIMINADO: Puntos insuficientes (%d < %d)", idx, #seg, min_puntos))
            end
        else
            -- 2. Validar por distancia total recorrida en el segmento
            local p_inicio = seg[1]
            local p_fin = seg[#seg]
            dist_total = math.sqrt((p_fin[1] - p_inicio[1])^2 + (p_fin[2] - p_inicio[2])^2)

            if dist_total <= max_ruido then
                es_valido = false
                if DS_LOG_MODE == 1 then
                    log(string.format(" >> Seg #%d ELIMINADO: Distancia por ruido (%.2f <= %.2f)", idx, dist_total, max_ruido))
                end
            end
        end

        -- Si pasó los filtros, lo guardamos en la nueva tabla limpia
        if es_valido then
            segmentos_filtrados[indice_filtrado] = seg
            indice_filtrado = indice_filtrado + 1
        end
    end

    local total_segs = #segmentos_filtrados

    if DS_LOG_MODE == 1 then
        log("--- FIN DE FILTRADO ---\n")
        if imprimir_resumen_segmentos then
            -- Pasamos la tabla filtrada en lugar de la original
            imprimir_resumen_segmentos("RESULTADO DE LA SEGMENTACIÓN", segmentos_filtrados, nil, nil)
        end
        log(string.format("--- FIN DE SEGMENTACIÓN. Total segmentos válidos obtenidos: %d ---\n", total_segs))
    end

    return segmentos_filtrados
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
        --if resetOBS then resetOBS() end 
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

        if #angles_a_recorrer == 0 then
            rel_angle = 0 
        end

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

    -- Logging original comentado
    log(string.format("CREATE PARAMS: Total segmentos construidos: %d ", #params)..", "..robot.id)
    for idx, seg in ipairs(params) do
        log(string.format(
            "Segmento %d -> Distancia: %.2f cm, ÁnguloRel: %.2f°",
            idx, seg[1], seg[2]
        ))
    end
    if #params == 0 then 
        log("No se generaron segmentos válidos para crear parámetros.") 
        -- resetOBS()
        -- return
    end

    return params
end

-- ============================================================================
-- 5. Datta Buffer: Procesa y Almacena los parámetros generados para su posterior reconstrucción
-- ============================================================================
-- BUFFER DE DATOS GLOBAL
DataBuffer = { stored = {} }
Global_Reconstruction_Done = false 

--  1. HELPER DE LOGS Y VISUALIZACIÓN
local function fmt_struct(s)
    if not s then return "nil" end
    local vals = {}
    for _, v in ipairs(s.value) do table.insert(vals, string.format("%.2f", v)) end
    local t_str = (s.type == 1) and "Ang" or "Dist"
    local count_str = (s.count and s.count > 1) and string.format(" [x%d]", s.count) or ""
    return string.format("(%s)%s {%s}", t_str, count_str, table.concat(vals, ", "))
end

--  2. HELPERS DE ALINEACIÓN Y VENTANA DESLIZANTE
-- Obtiene si una posición específica es 'A' (Ángulo) o 'D' (Distancia)
local function get_val_type(struct_type, index)
    local is_odd = (index % 2 ~= 0)
    if struct_type == 1 then return is_odd and "A" or "D"
    else return is_odd and "D" or "A" end
end

-- 3. Comprueba si short_dat está contenido dentro de long_dat a partir de start_idx
local function check_match(long_dat, short_dat, start_idx)
    if get_val_type(long_dat.type, start_idx) ~= get_val_type(short_dat.type, 1) then
        return false
    end

    for k = 1, #short_dat.value do
        local l_idx = start_idx + k - 1
        local v_type = get_val_type(short_dat.type, k)
        local diff = math.abs(long_dat.value[l_idx] - short_dat.value[k])
        local tol = (v_type == "A") and CONFIG.TOL_ANG or CONFIG.TOL_DIST
        
        if diff > tol then return false end
    end
    return true
end

--  4. FUNCIÓN PRINCIPAL DE ALMACENAMIENTO
function DataBuffer.add(new_dat)
    if Global_Reconstruction_Done then return end 
    local found = false

    for i, exist in ipairs(DataBuffer.stored) do
        
        -- CASO A: Coincidencia Exacta
        if #exist.value == #new_dat.value and exist.type == new_dat.type then
            if check_match(exist, new_dat, 1) then
                found = true
                exist.count = (exist.count or 1) + 1
                for k = 1, #exist.value do 
                    exist.value[k] = exist.value[k] + (new_dat.value[k] - exist.value[k]) / exist.count 
                end
                log(string.format("[INFO] Dato Refinado (Promedio de %d): %s", exist.count, fmt_struct(exist)))
                
                -- NUEVO: Restauración parcial por refinamiento
                update_patience("partial", exist.count) 
                break
            end
        end

        -- CASO B: El dato Nuevo contiene al Existente EN EL MEDIO
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
                    
                    exist.value = merged_values
                    exist.type = new_dat.type 
                    log(string.format("[INFO] Dato Actualizado (Extendido a %d val, solape en medio): %s", #exist.value, fmt_struct(exist)))
                    
                    -- NUEVO: Restauración parcial por actualización
                    update_patience("partial", exist.count)
                    break
                end
            end
            if found then break end
        end

        -- CASO C: El dato Existente contiene al Nuevo EN EL MEDIO
        if not found and #exist.value >= #new_dat.value + 2 then
            for start_idx = 2, #exist.value - #new_dat.value do
                if check_match(exist, new_dat, start_idx) then
                    found = true
                    exist.count = (exist.count or 1) + 1
                    
                    for k = 1, #new_dat.value do
                        local e_idx = start_idx + k - 1
                        exist.value[e_idx] = exist.value[e_idx] + (new_dat.value[k] - exist.value[e_idx]) / exist.count
                    end
                    
                    log(string.format("[INFO] Dato Actualizado (Subcadena promediada en medio): %s", fmt_struct(exist)))
                    
                    -- NUEVO: Restauración parcial por actualización
                    update_patience("partial", exist.count)
                    break
                end
            end
            if found then break end
        end
    end

    -- CASO D: No hubo coincidencias -> Dato Nuevo
    if not found then
        local new_v = {}
        for _, v in ipairs(new_dat.value) do table.insert(new_v, v) end
        local entry = { value = new_v, type = new_dat.type, count = 1 }
        table.insert(DataBuffer.stored, entry)
        log(string.format("[INFO] Nuevo Dato Registrado: %s", fmt_struct(entry))..", "..robot.id)
        
        -- NUEVO: Restauración total por dato completamente nuevo
        update_patience("full", 1)
    end
end

-- =========================================================================
--  6. Proceso de Reconstrucción Global: Métodos para combinar fragmentos y buscar solapes
-- =========================================================================

-- FUNCIONES MATEMÁTICAS PURAS (COMPARTIDAS)

local function calculate_overlap(s1, s2)
    local len1, len2 = #s1.value, #s2.value
    local max_ov = math.min(len1, len2)
    for ov = max_ov, CONFIG.MIN_OVERLAP, -1 do
        local match = true
        for k = 0, ov-1 do
            local idx1, idx2 = len1-ov+1+k, 1+k
            local odd1, odd2 = (idx1%2~=0), (idx2%2~=0)
            local t1 = (s1.type==1) and (odd1 and "A" or "D") or (odd1 and "D" or "A")
            local t2 = (s2.type==1) and (odd2 and "A" or "D") or (odd2 and "D" or "A")
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
    return { value = new_vals, type = s1.type }
end

local function clone_pool(pool)
    local new_pool = {}
    for i, v in ipairs(pool) do
        local nv = {}; for _, val in ipairs(v.value) do table.insert(nv, val) end
        table.insert(new_pool, { value = nv, type = v.type, id = v.id or i, count = v.count })
    end
    return new_pool
end

-- =========================================================================
--  6.1 MÉTODO 1: GREEDY (Con Logs Detallados)
-- =========================================================================
local function solve_greedy(initial_pool)
    local pool = clone_pool(initial_pool)
    
    -- log("=========================================")
    -- log("EJECUTANDO RECONSTRUCCIÓN (GLOBAL GREEDY)")
    -- log("=========================================")

    while #pool > 1 do
        local best = { i=-1, j=-1, ov=-1, struct=nil }
        
        for i=1, #pool do
            for j=1, #pool do
                if i ~= j then
                    local ov = calculate_overlap(pool[i], pool[j])
                    if ov > best.ov then
                        best = { i=i, j=j, ov=ov, struct=merge_fragments(pool[i], pool[j], ov) }
                    end
                end
            end
        end
        
        if best.ov > 0 then
            local str_A = fmt_struct(pool[best.i])
            local str_B = fmt_struct(pool[best.j])
            local str_Res = fmt_struct(best.struct)

            log(string.format("[MERGE] Uniendo Indices %d y %d (Solape: %d)", best.i, best.j, best.ov))
            log(" + Frag A: " .. str_A)
            log(" + Frag B: " .. str_B)
            log(" = Result: " .. str_Res)
            log("-----------------------------------------")

            pool[best.i] = best.struct
            table.remove(pool, best.j)
        else
            log("[INFO] No se encontraron más solapes posibles.")
            break
        end
    end
    return pool
end

-- =========================================================================
--  6.2. MÉTODO 2: EXHAUSTIVE BACKTRACKING (Busca TODAS las soluciones)
-- =========================================================================

local raw_solutions_found = {} 

local function recursive_finder(current_pool, history_log)
    if #current_pool == 1 then
        table.insert(raw_solutions_found, { result = current_pool[1], history = history_log })
        return
    end

    local candidates = {}
    for i=1, #current_pool do
        for j=1, #current_pool do
            if i ~= j then
                local ov = calculate_overlap(current_pool[i], current_pool[j])
                if ov > 0 then
                    table.insert(candidates, {i=i, j=j, ov=ov})
                end
            end
        end
    end

    if #candidates == 0 then return end 

    table.sort(candidates, function(a,b) return a.ov > b.ov end)

    for _, cand in ipairs(candidates) do
        local next_pool = {}
        local merged = merge_fragments(current_pool[cand.i], current_pool[cand.j], cand.ov)
        table.insert(next_pool, merged)
        
        for k=1, #current_pool do
            if k ~= cand.i and k ~= cand.j then
                table.insert(next_pool, current_pool[k])
            end
        end

        local next_history = {} 
        for _, h in ipairs(history_log) do table.insert(next_history, h) end
        table.insert(next_history, string.format("   Merge %d+%d (Ov:%d) -> %s", cand.i, cand.j, cand.ov, fmt_struct(merged)))

        recursive_finder(next_pool, next_history)
    end
end

local function select_best_solution(solutions)
    if #solutions == 0 then return nil end
    
    local unique_groups = {}
    for _, sol in ipairs(solutions) do
        local key = fmt_struct(sol.result)
        if not unique_groups[key] then
            unique_groups[key] = {
                result = sol.result,
                count = 1,
                history = sol.history
            }
        else
            unique_groups[key].count = unique_groups[key].count + 1
        end
    end

    local unique_list = {}
    for key, data in pairs(unique_groups) do
        table.insert(unique_list, data)
    end

    log(string.format("[GESTOR] M2 encontró %d caminos válidos, agrupados en %d soluciones únicas.", #solutions, #unique_list))

    local best_choice = nil

    if CONFIG.selection_mode == 1 then
        table.sort(unique_list, function(a, b) 
            if a.count == b.count then return #a.result.value > #b.result.value end
            return a.count > b.count 
        end)
        best_choice = unique_list[1]
        log(string.format("[GESTOR] CRITERIO 1: Seleccionada la más frecuente (Obtenida %d veces, Largo: %d val).", best_choice.count, #best_choice.result.value))

    elseif CONFIG.selection_mode == 2 then
        table.sort(unique_list, function(a, b) 
            if #a.result.value == #b.result.value then return a.count > b.count end
            return #a.result.value > #b.result.value 
        end)
        best_choice = unique_list[1]
        log(string.format("[GESTOR] CRITERIO 2: Seleccionada la más larga (Largo: %d val, Obtenida %d veces).", #best_choice.result.value, best_choice.count))

    elseif CONFIG.selection_mode == 3 then
        log("[GESTOR] CRITERIO 3: Lógica futura. Seleccionando por defecto la primera de la lista.")
        best_choice = unique_list[1]
    end
    
    return best_choice
end

local function solve_smart_backtracking_all(initial_pool)
    raw_solutions_found = {} 
    
    local sorted_pool = clone_pool(initial_pool)
    table.sort(sorted_pool, function(a,b) 
        local lenA = #a.value; local lenB = #b.value
        if a.count and b.count and a.count ~= b.count then return a.count > b.count end
        return lenA > lenB 
    end)

    recursive_finder(sorted_pool, {})
    return select_best_solution(raw_solutions_found)
end

-- =========================================================================
--  6.3: MÉTODO 3: ANÁLISIS DE PERIODICIDAD Y POLÍGONOS REGULARES
-- =========================================================================

-- 1. Función auxiliar para agrupar y contar pares {Distancia, Angulo}
local function extract_and_count_pairs(pool)
    local pairs_found = {}
    local total_pairs = 0

    for _, struct in ipairs(pool) do
        -- Necesitamos al menos 2 valores para formar un par
        local val_count = #struct.value
        if val_count >= 2 then
            local is_dist_first = (struct.type == 0)
            local i = 1
            
            while i < val_count do
                local dist, ang
                if is_dist_first then
                    dist = struct.value[i]
                    ang = struct.value[i+1]
                else
                    -- Si empieza con ángulo, el par es {Dist(i+1), Ang(i)} (o lo descartamos si no está completo)
                    -- Para simplificar, tomamos los pares en orden secuencial asumiendo pares completos
                    ang = struct.value[i]
                    dist = struct.value[i+1]
                end

                if dist and ang then
                    total_pairs = total_pairs + (struct.count or 1)
                    
                    -- Buscar si este par ya existe (con tolerancia)
                    local matched = false
                    for _, p in ipairs(pairs_found) do
                        if math.abs(p.d - dist) <= CONFIG.TOL_DIST and math.abs(p.a - ang) <= CONFIG.TOL_ANG then
                            -- Promediar e incrementar contador
                            p.d = p.d + (dist - p.d) / (p.count + 1)
                            p.a = p.a + (ang - p.a) / (p.count + 1)
                            p.count = p.count + (struct.count or 1)
                            matched = true
                            break
                        end
                    end
                    
                    if not matched then
                        table.insert(pairs_found, {d = dist, a = ang, count = (struct.count or 1)})
                    end
                end
                i = i + 2
            end
        end
    end
    
    return pairs_found, total_pairs
end

--2. Función principal del Método 3
local function check_regular_polygon(pool)
    -- log("=========================================")
    -- log("EJECUTANDO ANÁLISIS DE PERIODICIDAD (M3)")
    -- log("=========================================")

    local pairs, total = extract_and_count_pairs(pool)
    
    if total == 0 then
        -- log("[M3] No hay pares suficientes para analizar periodicidad.")
        return nil, false
    end

    -- Buscar el par predominante
    local dominant_pair = nil
    for _, p in ipairs(pairs) do
        local percentage = p.count / total
        if percentage >= CONFIG.THRESHOLD then
            dominant_pair = p
            break
        end
    end

    if not dominant_pair then
        -- log(string.format("[M3] No hay un par que supere el umbral del %.0f%%. No es una trayectoria periódica clara.", CONFIG.THRESHOLD * 100))
        return nil, false
    end

    -- log(string.format("[M3] Patrón periódico detectado: Distancia %.2f, Ángulo %.2f (Representa el %.0f%% de los datos)", dominant_pair.d, dominant_pair.a, (dominant_pair.count/total)*100))

    -- Verificación Matemática de Polígono Regular
    if dominant_pair.a == 0 then
        -- log("[M3] El ángulo es 0. Es una línea recta, no un polígono.")
        -- Devolver la cadena más larga
        table.sort(pool, function(a,b) return #a.value > #b.value end)
        return pool[1], true
    end

    local sides_calc = 360 / math.abs(dominant_pair.a)
    local sides = math.floor(sides_calc + 0.5) -- Redondear al entero más cercano
    
    if math.abs(sides_calc - sides) <= CONFIG.MATH_TOLERANCE then
        -- log(string.format("[M3] ¡Polígono Regular Confirmado! Calculado: %.2f lados. Se asume una figura de %d lados.", sides_calc, sides))
        
        -- Construir la cadena sintética perfecta
        local perfect_values = {}
        -- Empezamos con Distancia, luego Ángulo
        for i = 1, sides do
            table.insert(perfect_values, dominant_pair.d)
            table.insert(perfect_values, dominant_pair.a)
        end
        
        local synthetic_struct = {
            type = 0, -- 0 = Empieza con Distancia
            value = perfect_values,
            count = 1 -- Sintética
        }
        
        -- log("[M3] Generada cadena sintética perfecta para el polígono.")
        return synthetic_struct, true
    else
        -- log(string.format("[M3] La matemática no cierra un polígono (Lados calculados: %.2f). Es una trayectoria abierta periódica.", sides_calc))
        -- log("[M3] Acción temporal: Seleccionando la cadena más larga del buffer.")
        -- Devolver la cadena más larga
        table.sort(pool, function(a,b) return #a.value > #b.value end)
        return pool[1], true
    end
end

-- =========================================================================
--  6.4. GESTOR PRINCIPAL
-- =========================================================================
function run_global_reconstruction()
    if Global_Reconstruction_Done then return end
        
    local start_pool = {}
    for i, v in ipairs(DataBuffer.stored) do 
        table.insert(start_pool, { value = v.value, type = v.type, count = v.count, id = i }) 
    end
    
    if #start_pool == 0 then return end
    
    local final_result_struct = nil

    -- PASO 1: EVALUAR PERIODICIDAD (MÉTODO 3)
    local periodic_result, is_periodic = check_regular_polygon(start_pool)

    if is_periodic then
        final_result_struct = periodic_result
        log("[GESTOR] Reconstrucción basada en Método 3 (Patrón Periódico).")
    else
        -- PASO 2: EJECUTAR MÉTODO 1 (Greedy)
        local greedy_result_pool = solve_greedy(start_pool)
        local m1_perfect = (#greedy_result_pool == 1)

        if m1_perfect then
            log("[GESTOR] Método 1 Exitoso. Cadena única encontrada.")
            final_result_struct = greedy_result_pool[1]
        else
            log(string.format("[GESTOR] Método 1 incompleto. Quedaron %d islas.", #greedy_result_pool))
        end

        -- PASO 3: EVALUAR VARIABLE 1: ¿Ejecutamos el Método 2?
        local run_m2 = (not m1_perfect) or CONFIG.FORCE_M2_ALWAYS

        if run_m2 then
            if m1_perfect then
                log("[GESTOR] FORZANDO Método 2 a pesar de éxito en M1 (Variable CONFIG Activa).")
            else
                log("[GESTOR] Activando Método 2: Deep Backtracking...")
            end

            -- EJECUTAR MÉTODO 2
            local best_match = solve_smart_backtracking_all(start_pool)
            
            if best_match then
                log("[GESTOR] ¡MÉTODO 2 EXITOSO! Reemplazando solución principal.")
                final_result_struct = best_match.result
                
                if CONFIG.PRINT_RECURSIVE_PATHS then
                    log("[GESTOR] Historial del primer camino que descubrió esta solución:")
                    for _, step in ipairs(best_match.history) do log(step) end
                end
            else
                log("[GESTOR] Método 2 no encontró soluciones perfectas (Islas desconectadas).")
                
                if not m1_perfect then
                    log("[GESTOR] Se mantiene el mejor esfuerzo del Greedy (Isla más grande).")
                    table.sort(greedy_result_pool, function(a,b) return #a.value > #b.value end)
                    final_result_struct = greedy_result_pool[1]
                else
                    log("[GESTOR] Se mantiene la solución original perfecta del Greedy.")
                end
            end
        end
    end

    -- 4. RESULTADOS FINALES
    log("\n---------------- RESULTADOS ----------------")
    if final_result_struct then
        log(">> CADENA PRINCIPAL RECONSTRUIDA:")
        log(fmt_struct(final_result_struct))
        
        --SOLO AQUÍ SE DEBE ACTIVAR LA BANDERA:
        Global_Reconstruction_Done = true

    else
        log(">> ERROR CRÍTICO: No se pudo recuperar ninguna estructura válida.")
    end
    log("--------------------------------------------\n")

    --Global_Reconstruction_Done = true
    
    if final_result_struct then
        -- 1. Procesamos el resultado numérico (asumo que esto actualiza 'full_pattern_params')
        process_numeric_result(final_result_struct)
        
        -- =====================================================================
        -- OBJETIVO 1: GUARDAR LOS PARÁMETROS EN LA TABLA GLOBAL 'all_params'
        -- =====================================================================
        
        -- Determinamos el nuevo índice 'k'. 
        -- (#all_params te da el tamaño actual de la tabla. Si el original está en el 1, este será el 2, etc.)
        local k_nuevo = #all_params + 1 
        
        -- Creamos una tabla nueva para evitar el problema de referencias en Lua
        local copia_parametros = {}
        
        -- Asumiendo que process_numeric_result dejó el resultado final en la variable 'full_pattern_params'
        if full_pattern_params and #full_pattern_params > 0 then
            for i, par in ipairs(full_pattern_params) do
                -- par[1] es distancia, par[2] es ángulo
                table.insert(copia_parametros, {par[1], par[2]}) 
            end
            
            -- Guardamos la copia en nuestra tabla histórica
            all_params[k_nuevo] = copia_parametros
            log("[GESTOR] ¡Nuevo patrón aprendido y guardado en all_params[" .. k_nuevo .. "]!\n\n\n")
        else
            log("[GESTOR] Advertencia: process_numeric_result no generó parámetros válidos para guardar.\n\n\n")
        end

        -- ACTIVAR ESTADO FINAL DEL ROBOT

        resetOBS()
        is_demonstrator = 1
    end
end

--------------------------------------------------------------
--------------------------------------------------------------

-- Variable global para almacenar el resultado final
full_pattern_params = {} 

function process_numeric_result(result_struct)
    full_pattern_params = {}
    local values = result_struct.value
    local current_type = result_struct.type -- 1=Ang, 0=Dist
    
    log("[RECONSTRUCCIÓN FINALIZADA]")

    local temp_dist = 0
    local temp_ang = 0
    local expecting_dist = (current_type == 0)
    
    for i, val in ipairs(values) do
        if expecting_dist then
            temp_dist = val
            table.insert(full_pattern_params, { temp_dist, temp_ang })
            temp_ang = 0 
            expecting_dist = false
        else
            temp_ang = val 
            expecting_dist = true
        end
    end
    
    log(">> Patrón traducido a parametros físicos. Total Segmentos: " .. #full_pattern_params)
    for idx, seg in ipairs(full_pattern_params) do
        log(string.format("   Seg %d: Dist %.2f, Ang %.2f", idx, seg[1], seg[2]))
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

    if robot.id ~= "Footbot100" then
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
    -- 1. VARIABLES NUEVAS CRÍTICAS DE TRACKING Y BUFFER
    -- ============================
    DataBuffer.stored          = {} -- DataBuffer = { stored = {} } -- ESTO DESTRUYE LA FUNCIÓN
    Global_Reconstruction_Done = false
    master_tracking            = {}
    srt_anterior_data          = {}
    current_patience           = MAX_PATIENCE
    --full_pattern_params        = {}

    DZD_c1 = 0

    -- ===================f=========
    -- 2. VARIABLES DE POSICIONAMIENTO
    -- ============================
    distances        = {} -- Lista de posiciones relativas (x, y)
    angles           = {} -- Lista de ángulos relativos
    srt_distance_an  = 80
    srt_distance_new = 0

    srt_anterior_data = {}
    master_tracking = {}

    SRT_c1           = 0
    srt_c2           = 0
    srt_c3           = 0
    srt_c4           = 0
    srt_c5           = 0
    count            = 0
    srt_ang_mark_ant = 0
    x_mark, y_mark, x_new_r, x_ant_r, y_new_r, y_ant_r = 0, 0, 0, 0, 0, 0

    srt_angle_an     = 0

    -- ============================
    -- 3. VARIABLES DE MOVIMIENTO Y MARCADORES
    -- ============================
    motion_markers      = {}           -- Correcciones registradas
    last_motion_marker  = {0, 0}       -- Total correccion realizada
    motion_tick         = 0            -- Tick actual durante movimiento
    motion_active       = false        -- Indica si hay un movimiento en curso
    motion_target_ticks = 0            -- Total de ticks necesarios
    motion_cm_tick      = 0
    motion_dist         = 0

    prev_x_original, prev_y_original = 0, 0       -- guardan la última posición observada
    stationary_count  = 0                         -- contador que queremos incrementar (solo 1 vez por grupo)
    stationary_active = false                     -- bandera que indica que ya contamos este grupo igual

    segmentos       = {}     -- segmentos[2], segmentos[3], ... contendrán los segmentos
    movement_buffer = {}     -- buffer temporal con puntos mientras el demostrador está en movimiento

    srt_end         = 0

    -- ============================
    -- 4. MATEMÁTICAS Y CÁLCULO
    -- ============================
    DS_c1                 = 0
    equations             = {}
    distancias_a_recorrer = {} 
    intersections         = {}
    angles_a_recorrer     = {}
    params                = {} -- Reiniciado para no cruzar arrays locales
    posiciones_guardadas  = {}

    -- ============================
    -- 5. MÁQUINA DE ESTADOS Y CONTROL
    -- ============================
    k       = 0
    indice  = 1
    PI_c1   = 0
    PI_end  = false
    c3      = 0
    c4      = 0
    c5      = 0 -- ¡Se comparte con patron_imitado!

    -- ============================
    -- 6. RESETEO DE ACTUADORES Y TIMERS
    -- ============================
    setleds("black")
    r_Temporizador_1()
    r_Temporizador_2()
    r_move()
end


function resetDEM() 
    -- ============================
    -- 1. MÁQUINA DE ESTADOS Y BUCLES
    -- ============================
    ind    = 1
    k      = 0
    d1     = 0
    indice = 1
    c5     = 0 


    DZD_c1 = 0 

    -- ============================
    -- 2. VARIABLES DE EJECUCIÓN DEL PATRÓN
    -- ============================
    PO_counter_seg       = 0
    PO_end               = false
    movimiento_recorrido = 0
    parametros_elegidos  = {}
    posiciones_guardadas = {}

    -- ============================
    -- 3. PROMEDIOS Y TIMERS (EAEP)
    -- ============================
    demostrador_avg_angle  = 0
    demostrador_avg_dist_1 = 0
    EAEP_c1                = 0
    EAEP_c2                = 0
    EAEP_ang_ant           = 0
    MT_c1                  = 0

    -- ============================
    -- 4. RESETEO DE ACTUADORES Y TIMERS
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
            print(key .. ":")
            print_table(value) -- Si el valor es una tabla, recursivamente imprimimos sus claves
        else
            print(key .. " = " .. tostring(value))
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
        yellow_true = true
    end

    --============================== Octógono ==============================

    if figura == "octágono" then
        yellow_true = true
    end

    --============================== triangulo_irregular_1 ==============================
    if figura == "triangulo_irregular_1" then

        yellow_true = true
    end

    --============================== Pentágono_irregular ==============================

    if figura == "Pentágono_irregular" then

        yellow_true = true
    end

    --============================== Heptágono ==============================

    if figura == "Heptágono" then

        yellow_true = true   
    end

    --============================== trayectoria_1 ==============================
    if figura == "trayectoria_1" then

        yellow_true = true
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
            { 30 ,120 },
            { 30 ,120 },
            { 30 ,120 },
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
            { bigger*20  , 10},
            { bigger*20  , 90},
            { bigger*20  , 53},
            { bigger*20  , 74},
            { bigger*28.3, 84},
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