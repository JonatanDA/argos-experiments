--=============================================
--   MEJORAS :  
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

-- 187014   
math.randomseed(os.time())

ticks_per_second                    = 10                        -- Número de ticks que representa un segundo en la simulación (ajusta según la configuración de .argos)
max_time                            = 500 * ticks_per_second    -- Duración máxima de la simulación en ticks
Distribucion_sensores_de_proximidad = {0,-45,45,-90, 90,180}    -- Ángulos de los sensores de proximidad a considerar para la detección de obstáculos (en grados) --> {} o "uniformes"
--Distribucion_sensores_de_proximidad = "uniform"               -- Ángulos de los sensores de proximidad a considerar para la detección de obstáculos (en grados) --> {} o "uniformes"
Num_cantidad_leds                   = 2

------------------------------
--Parámetros de simulación internos

speed_dem = 27                  -- Velocidad del master al ejecutar el patrón
d_speed   = 30                  -- velocidad normal
g_speed   = 50                  -- Velocidad de giro en grados por segundo
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

-- function Print_tracker_trayectory_data(k, x, y,logging)

--     if type(k) == "number" then

--         print("Master" .. k .. " & " .. x .. " , " .. y)
--         if logging then 
--             log("Master" .. k .. " & " .. x .. " , " .. y)
--         end

--     elseif type(k) == "string" then
--         -- Obtener el vector de posición inicial
--         local x = robot.positioning.position.x
--         local y = robot.positioning.position.y

--         print(robot.id .. " & " .. x .. " , " .. y)  -- Imprime los datos globales del movimiento de los Qupa
--     end
-- end
-----------------------------------------------------------------------------------------------------------------------------------
-- Función que se ejecuta al inicio de la simulación
-------------------------------------------------------------------------------------------------------------------------------------

function init()

    robot.colored_blob_omnidirectional_camera.enable()
end

-------------------------------------------------------------------------------------------------------------------------------------
-- Función principal que se ejecuta en cada tick
-------------------------------------------------------------------------------------------------------------------------------------

timer = 1 -- Contador de ticks de simulación
function step()

    --if robot.id == "Qupa4" and Print_tracker_trayectory_data then print("Timer="..timer) end
    master() 
    timer = timer + 1
end

-------------------------------------------------------------------------------------------------------------------------------------
-- Funciones para cada rol
-------------------------------------------------------------------------------------------------------------------------------------

all_params              = {}    -- Almacenará todos los patrones descubiertos
parametros_elegidos     = {}    -- El patrón específico seleccionado para ejecutar

d1                      = 0
c5                      = 0
master_avg_dist    = 0
first_time_demonstrator = true
function master()

    if d1 == 0 then
        detectar_zona_demostracion()

    elseif d1 == 1 then
        -- =========================================================
        -- Selección aleatoria del patrón a demostrar
        -- =========================================================
        parametros_elegidos = original_params()
        parametros_elegidos = parametros_elegidos.params
        log(" ========= Ejecutando patrón original: " .. robot.id.." ========= \n")
        --print_table(parametros_elegidos)

        d1 = 2

    elseif d1 == 2 then
        Ejecutar_comportamiento(parametros_elegidos)
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

--bigger = 1                    -- Factor de escala para ajustar el tamaño de las figuras predefinidas (ajusta según tu espacio de simulación)
--figura = "triangulo_regular"  -- "triangulo_regular" -- "octagono" -- "triangulo_irregular_1" --"Pentágono_irregular"  -- "Heptágono"-- "" -- "trayectoria_1" 

function detectar_zona_demostracion()
    RandomWalking()
    setleds("black")
    
    -- Evaluación de ambos rangos de color/gris
    local G = ground{
        range = {
            {0.20, 0.30},   -- Rango 1: Gris 2
            {0.80, 0.90}    -- Rango 2: Nuevo rango
        },
        print_all_data        = false,
        print_detected_ranges = false,
        base_ground_active    = false
    }

    if #G.Gr > 0 then
        -- G.Gr está ordenado por número de rango; el primer elemento contiene el rango menor detectado
        local rango_detectado = G.Gr[1][1]

        if rango_detectado == 1 then
            bigger = 1
            figura = "Pentágono_irregular"
        elseif rango_detectado == 2 then
            bigger = 1
            figura = "triangulo_regular"
        end

        d1 = 1
        robot.wheels.set_velocity(0, 0)
        r_Temporizador_1()
        r_move()

        log("Zona de demostración detectada (" .. tostring(figura) .. "), " .. robot.id)
    end
end


-------------------------------------------------------------------------------------------------------------------------------------
-- función para realizar el comportamiento de movimiento (trayectoria)
-------------------------------------------------------------------------------------------------------------------------------------
SMART_MASTERS = true -- true: Se detienen si hay obstáculo. false: Son ciegos.

k               = 0
c5              = 0
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

    if k == 0 then
        if #params >= 1 then
            if indice <= #params then
                local valor1, valor2 = params[indice][1], params[indice][2]
                
                -- ¡CLAVE! Siempre llamamos a move, inyectando la bandera de pausa al final
                move(valor1, valor2, speed_dem, 10, false, robot.id, master_paused)

                -- Solo avanzamos la lógica del patrón si NO estamos pausados
                if not master_paused then

                    if Print_tracker_trayectory_data then Print_tracker_trayectory_data(robot.id, 0, 0) end

                    setleds("yellow")

                    -- c1 == 2 indica que el segmento terminó con éxito dentro de move()
                    if c1 == 2 then
                        PI_counter_seg = PI_counter_seg + 1
                        if indice == #params then
                            indice = indice + 1
                            PI_end = true
                            r_Temporizador_1()

                            if c5 == 1 then
                                k = 1
                            else
                                c5 = 1
                                k = 1
                            end
                        else
                            indice = indice + 1
                            c1 = 0
                        end
                    end
                end
            end
        else
            k = 1
            robot.leds.set_all_colors("black")
            log( " -------------------------------------------------------------------------" )
            log("Patrón no detectado :( " .. robot.id)
            log( " -------------------------------------------------------------------------" )
        end
    end

    if k == 1 then
        robot.wheels.set_velocity(0, 0)   -- Detenerse

        if robot.id == "Qupa4" then 
            k = 1
        else
            k = 2
            setleds("black")
        end

    elseif k == 2 then
        resetDEM()
        d1 = 2
        PI_counter_seg = 0
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

--- Lee los sensores de suelo (base y/o motor) y detecta qué lecturas caen en los rangos especificados.
-- @param opts Tabla de opciones de configuración:
--   - base_ground_active    (boolean): Activa la lectura de sensores base_ground (default: false).
--   - print_all_data        (boolean): Imprime el valor de cada sensor en consola (default: false).
--   - print_detected_ranges (boolean): Imprime los rangos detectados en consola (default: false).
--   - range                 (table)  : Lista de rangos {{min1, max1}, {min2, max2}, ...}.
-- @return table Retorna una tabla `Ground` con la estructura:
--   - Ground.base  : {valor_sensor_1, valor_sensor_2, ...}
--   - Ground.motor : {valor_sensor_1, valor_sensor_2, ...}
--   - Ground.Gr    : {{rango_index, sensor_index}, ...} ordenados ascendentemente por rango_index.
function ground(opts)
    opts = opts or {}
    local base_ground_active    = opts.base_ground_active      or false
    local print                 = opts.print_all_data          or false
    local print_detected_ranges = opts.print_detected_ranges   or false
    local ranges                = opts.range                   or {}

    local Ground = {
        base  = {},
        motor = {},
        Gr    = {}
    }

    -- Procesar sensores base_ground si están activos y definidos
    if base_ground_active and robot.base_ground then 
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

    -- Cantidad de sensores base para desfase de índice global seguro
    local base_count = (robot.base_ground and #robot.base_ground) or 0
    local motor_sensors = robot.motor_ground or {}

    -- Procesar sensores motor_ground
    for i, reading in ipairs(motor_sensors) do
        Ground.motor[i] = reading.value
        if print then
            log(string.format("[Motor_Ground %d] valor=%.3f", i, reading.value))
        end

        for r_index, bounds in ipairs(ranges) do
            local min_val = bounds[1]
            local max_val = bounds[2]
            if reading.value >= min_val and reading.value <= max_val then
                local global_index = i + base_count
                table.insert(Ground.Gr, {r_index, global_index})
            end
        end
    end

    -- Ordenar coincidencias por el número de rango (prioridad al primer rango coincidido)
    table.sort(Ground.Gr, function(a, b)
        return a[1] < b[1]
    end)

    if print_detected_ranges then
        log(" -------------------------------------------------- "..robot.id)
        if #Ground.Gr > 0 then
            log("Rangos detectados:")
            for i, pair in ipairs(Ground.Gr) do
                log(string.format("  - Rango Detectado: %d por el sensor #%d,", pair[1], pair[2]))
            end
        end
    end

   return Ground
end

-- Función de detección de proximidad
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

    Temporizador_2(0.1)
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

-------------------------------------------------------------------------------------------------------------------------------------
-------------------------------------------------------------------------------------------------------------------------------------
-- 
-------------------------------------------------------------------------------------------------------------------------------------
-------------------------------------------------------------------------------------------------------------------------------------


function resetDEM() 

    k = 0

    -- ============================¿
    d1 = 0
    posiciones_guardadas = {}
    indice = 1

    -- ============================
   
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