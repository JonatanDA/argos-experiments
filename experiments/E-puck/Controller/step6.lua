-- Variable global

-- 721549    semilla 1er exp. epuck
-- 187014    semilla 1er exp. FOOT

-- Activar para epuck
demostrador_threshold = 12
threshold = 10

-- Activar para footbot
-- demostrador_threshold = 32
-- threshold = 30

valor1 = 0
valor2 = 0

indice = 1

timer = 1
led_timer = 50;
c1 = 0
ang = 0
time_needed = 0
ticks_needed = 0
speed = 15
g_speed = 4

c3 = 0
c4 = 0
c5 = 0

b = 0

is_demonstrator = 0

math.randomseed(os.time())

perm1c = 20
perm2c = 20
permt = 10

sensitivity = 1.5
avance = 6

-- --------------------------------------------------------------------------------------------------

-- Función que se ejecuta al inicio de la simulación

function init()
    --[[
    for key, _ in pairs(robot) do
    	  log("Dispositivo detectado: " .. key)
    end
]]
    robot.colored_blob_omnidirectional_camera.enable()
    --log("Robot " .. robot.id .. " iniciado.")

    -- Obtener el vector de posicion inicial
    posicion_x = robot.positioning.position.x
    posicion_y = robot.positioning.position.y
    posicion = { posicion_x, posicion_y }
    --log("posicion: x=" .. posicion[1] .. ", y=" .. posicion[2])

    -- Obtener el vector de orientacion inicial
    orientation = robot.positioning.orientation
    vec2_angle(orientation)
    -- Imprimir el ángulo
    --log("Angle: " .. ang)

    determinar_obs_dem()
end

-- --------------------------------------------------------------------------------------------------

-- Función principal que se ejecuta en cada tick
function step()
    if is_demonstrator == 1 then
        demostrador()
        -- Ejecutar si es demostrador
    else
        observador()
        --log("Buscando demostrador")
    end
    timer = timer + 1
end

-------------------------------------------------------------------------------------------------------------------------------------
-------------------------------------------------------------------------------------------------------------------------------------
-------------------------------------------------------------------------------------------------------------------------------------
-------------------------------------------------------------------------------------------------------------------------------------
-------------------------------------------------------------------------------------------------------------------------------------
-- Funciones para cada rol

d1 = 0
demostrador_avg_angle = 0
function demostrador()
    local blobs = robot.colored_blob_omnidirectional_camera
    local angle = 0
    local total_distance = 0
    local count = 0
    --log("Cantidad de blobs detectados: " .. tostring(#blobs).. ", por " .. robot.id)
    --log(d1)
    --robot.wheels.set_velocity(-3, 3)
    --log("Buscando alumno....")
    if d1 == 0 then
        RandomWalking()
        --log("rannnnnn")
        demostrador_avg_angle = 0


        for i, blob in ipairs(blobs) do
            local r, g, b = blob.color.red, blob.color.green, blob.color.blue
            angle = blob.angle * (180 / math.pi) -- Obtener el ángulo del blob en Grados
            demostrador_avg_angle = angle + demostrador_avg_angle
            count = count + 1


            if r < 100 and g < 100 and b > 200 then  -- Si detecta un objeto azul
                d1 = 1
                demostrador_avg_angle = demostrador_avg_angle / count
                orientation = robot.positioning.orientation
                demostrador_avg_angle = demostrador_avg_angle + vec2_angle(orientation)
                
                log("Alumno detectado! por: " .. robot.id)
                robot.wheels.set_velocity(0, 0)
                robot.leds.set_all_colors("yellow")
                break
            end
        end
    elseif d1 == 1 then    

        --log("demostrador_avg_angle: "..demostrador_avg_angle.." "..robot.id)      
        move(0, demostrador_avg_angle)
        espere(50)

        if #blobs > 0 then

            demostrador_avg_dist = 0
                    -- Recorrer todos los blobs y sumar sus distancias
            for i, blob in ipairs(blobs) do
                total_distance = total_distance + blob.distance
                count = count + 1
                --log(string.format("Blob %d: Distancia = %.4f", i, blob.distance))
            end

            -- Calcular distancia promedio
            demostrador_avg_dist = total_distance / count
            --log(string.format("Distancia promedio de blobs: %.4f", demostrador_avg_dist))
      
        end
    

        -- Si la distancia promedio es menor al umbral, detener el robot
        if c1 == 2 and temp == false then
            if demostrador_avg_dist < demostrador_threshold then
                c1 = 0
                c2 = 0
                firstime = 0
                d1 = 2
                led_on = true
            else
                log("Alumno no esta prestando atención " .. robot.id)
                --resetDEM()
            end    
        end
        
        if temp== false then
            r_espere()
        end

    elseif d1 == 2 then
        d1 = 3  
    elseif d1 == 3 then

        if c5 == 1 then
            patron_imitado(demostrador_avg_angle)
            --log("Patron 1 (imitado) ejecutandose")
        else
            triangulo(demostrador_avg_angle)
            --log("Patron 2( triangulo) ejecutandose")
        end
    end
end


function observador()
    if c3 == 0 then
--[[         if c5 == 1 then
            if robot.random.uniform_int(0, 100) < 2 then
                log("Repitiendo patrón aprendido... " .. robot.id)
                c3 = 2
                c4 = 6
            end
        end ]]
        detect_and_stop()
        --log("detect and stop: "..c3)
        if c3 == 0 then
            --log(" RandomWalking: "..c3)
            RandomWalking()
        end
    elseif c3 == 1 then
        move_until_obstacle()
    elseif c3 == 2 then
        if c4 == 0 then
            store_relative_positions()
            --log(c4)
        elseif c4 == 1 then
            polarACartesiano(distances, angles)
            -- log(c4)
        elseif c4 == 2 then
            detect_seg(coordenadasx, coordenadasy)
            -- log(c4)
        elseif c4 == 3 then
            apply_linear_regression(segmentos)
            -- log(c4)
        elseif c4 == 4 then
            calculate_intersections(equations)
            --log(c4)
        elseif c4 == 5 then
            calculate_segments(intersections)
            -- log(c4)
            if #segments > 2 then
                valor1, valor2 = segments[indice][1], segments[indice][2]
            else
                resetOBS()
                robot.leds.set_all_colors("black")
                log("Patrón no detectado :( " .. robot.id)
            end
        elseif c4 == 6 then
            patron_imitado(demostrador_avg_angle)
            --c5 = 1
        end
        --log("analizando movimiento por ".. robot.id)
    end
end

-------------------------------------------------------------------------------------------------------------------------------------
-------------------------------------------------------------------------------------------------------------------------------------
-------------------------------------------------------------------------------------------------------------------------------------
-------------------------------------------------------------------------------------------------------------------------------------
-------------------------------------------------------------------------------------------------------------------------------------

-- Funciones Varias

-- Función para determiaar si un epuck será observador o demostrador
function determinar_obs_dem()
    -- Selección aleatoria del demostrador

    randoms = robot.random.uniform_int(0, 2)
    if randoms == 1 then
        is_demonstrator = 1
    else
        is_demonstrator = 0
    end

    if robot.id == "epuck0" then
        is_demonstrator = 0
    elseif robot.id == "epuck2" then
        is_demonstrator = 0
    elseif robot.id == "epuck4" then
        is_demonstrator = 1
    elseif robot.id == "epuck1" then
        is_demonstrator = 0
    elseif robot.id == "epuck3" then
        is_demonstrator = 0
    elseif robot.id == "epuck5" then
        is_demonstrator = 0
    end

    -- Encender LED rojo si es demostrador
    if is_demonstrator == 1 then
        robot.leds.set_all_colors("red") -- Cambia solo un LED específico
        led_on = true
        log("The " .. robot.id .. " is demostrador = " .. randoms)

        --led_timer = 10 -- 5 segundos a 10 ticks por segundo
        --log("Led rojo encendido por " ..robot.id)
    else
        --robot.leds.set_all_colors("blue")
        log("The " .. robot.id .. " is Observador= " .. randoms)
    end


    return is_demonstrator
end

function resetOBS()
    temp          = true
    cr            = 0
    crono         = 0

    distances     = {}
    angles        = {}

    coordenadasx  = {}
    coordenadasy  = {}

    --segments = {}

    intersections = {}

    segmentos     = {}

    equations     = {}
    

    indice        = 1

    c1            = 0
    c2            = 0

    c3            = 0
    c4            = 0
    b             = 0


    firstime      = 0
    r_espere()
end

function resetDEM()                 
    c2 = 0
    c1 = 0
    ind = 1

    b = 0

    d1 = 0
    led_on = true
    led_timer = 50;

    temp = true
    cr = 0
    crono = 0

    demostrador_avg_angle = 0

    robot.leds.set_all_colors("red")

-- cuando c5 = 1
    indice = 1
    r_espere()
end


temp = true
cr = 0
crono = 0
function r_espere()
    temp = true
    cr = 0
    crono = 0
end
    
function espere(c)
    --log("cr = "..cr)
    --log("c = "..c)

    crono = c - cr

    --log("crono = "..crono)
    cr = cr + 1

    --log(" cronómetro: "..cr)

    if crono <= 0 then
        --log(" pip---pip---pip ")
        --robot.leds.set_all_colors("black")
        temp = false
        cr = o            

    end
end

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

-------------------------------------------------------------------------------------------------------------------------------------
-------------------------------------------------------------------------------------------------------------------------------------
-------------------------------------------------------------------------------------------------------------------------------------
-------------------------------------------------------------------------------------------------------------------------------------
-------------------------------------------------------------------------------------------------------------------------------------

-- Funciones de comportamiento de los epuck

--187014

--[[
function proximidad()

    local sensores = robot.proximity
    local total_distance = 0
    local count = 0
    local total_angle = 0
    local dist = 0
    local ang = 0

    if #sensores > 0 then
        -- Recorrer todos los blobs y sumar sus distancias
        for i, sensor in ipairs(sensores) do
            total_distance = total_distance + sensor.value
            total_angle = total_angle  + (sensor.angle * sensor.value)
            count = count + 1
        end
        dist = total_distance / count
        proximity[1] = dist
        ang = (total_angle / count ) * 180/math.pi
        ang = math.abs(ang)
        proximity[2] = ang
        log("Distancia = "..dist.." , angulo = "..ang.."   id: "..robot.id)
        return proximity
    end
end
]]


function imprimir_posicion()
    local posicion = {}
    posicion[1] = robot.positioning.position.x
    posicion[2] = robot.positioning.position.y
    log("Posición en x: "..posicion[1]..", Posición en y: "..posicion[2].." ..."..robot.id)
end

-- función  para realizar el patron imitado
k = 0
indice = 1
triangulo_ang = 0
c5 = 0
sh_seg = 0
function patron_imitado(demostrador_avg_angle)
    if sh_seg == 0 then
        imprimir_posicion()
        sh_seg = 1
    end

    if indice == 1 then
        triangulo_ang = demostrador_avg_angle
        --log("Iniciando patrón imitado  "..robot.id)
    end

    if b <= #segments then
        local valor1, valor2 = segments[indice][1], segments[indice][2] + triangulo_ang
        --log("distancia en patron_imitado= ".. segments[indice][1].." "..robot.id)
        --log("angulo en patron_imitado= ".. segments[indice][2].." "..robot.id)
        b = move(valor1, valor2)

        if c1 == 2 then
            if indice == #segments then
                imprimir_posicion()
                log("Patrón imitado, finalizado...." .. robot.id)
                --log(" Ultimo Segmento: "..indice.." finalizado por: "..robot.id)
                --log("Segmentos finalizados, "..robot.id)
                
                b = b + 1
                if c5 == 1 then
                    k = k + 1
                    c1 = 0
                    c2 = 0
                    robot.leds.set_all_colors("green")
                else
                    resetOBS()
                    c5 = 1
                    is_demonstrator = 1
                    log("Ahora soy maestro!... " .. robot.id)
                    robot.leds.set_all_colors("red")

                end
            else
                -- log("Segmento numero: "..indice.." finalizado")
                --print_table(segments)
                indice = indice + 1
                c1 = 0
                sh_seg = 0
            end
        end
    end

    if k == 1 then
        orientation = robot.positioning.orientation
        angle = vec2_angle(orientation)
        move(-100, angle)
        if c1 == 2 then
            resetDEM()
            robot.leds.set_all_colors("red")
            k = 0
        end
    end
end

-- función  de detección de proximidad
proximity = { 0, 0 }
function proximidad()
    local sensores_frontales = { robot.proximity[1], robot.proximity[1] }

    local total_distance = 0
    local total_angle = 0
    local count = 0

    -- Recorrer los sensores y calcular el promedio
    for _, sensor in ipairs(sensores_frontales) do
        if sensor then
            total_distance = total_distance + sensor.value
            total_angle = total_angle + sensor.angle
            count = count + 1
        end      

    end

    if count > 0 then
        proximity[1] = total_distance / count                          -- Promedio de distancia
        proximity[2] = math.abs((total_angle / count) * 180 / math.pi) -- Promedio de ángulo en grados
        --log("intensidad = "..proximity[1])
        --log("angulo = "..proximity[2])
    end
    return proximity
end

-- función  de caminata aleatoria
b1 = 0
b2 = 0
angle = 0

function RandomWalking()
    local orientation = 0
    proximidad()
    if proximity[1] < 0.01 and b2 == 0 then
        -- Generar un giro aleatorio ocasionalmente
        local random = robot.random.uniform_int(0, 60)
        if random >= 10 and random <= 20 then
            robot.wheels.set_velocity(random, -random)
        elseif random >= 30 then
            robot.wheels.set_velocity(random, random)
        end
        --log("haciendo randommmmm")
    else
        b2 = 1
    end

    if b2 == 1 then
        if b1 == 0 then
            orientation = robot.positioning.orientation
            --log("orientacion= "..orientation)"  "..robot.id
            angle = vec2_angle(orientation) + 180
            --log("angle= "..angle.." ... "..robot.id)
            b1 = 1     

        end
        --log("Girando..... "..robot.id)

        move(0, angle)

        if c1 == 2 then
            --log("nuevamente RandomWalking.... "..robot.id)
            b1 = 0
            b2 = 0
            c1 = 0
            c2 = 0
            angle = 0
            firstime = 0
        end
    end
end

-- Función de busuqeda ejecutada por el observador

function detect_and_stop()
    if robot.colored_blob_omnidirectional_camera then
        local blobs = robot.colored_blob_omnidirectional_camera
        --log("Cantidad de blobs detectados: " .. tostring(#blobs).. ", por " .. robot.id)
        local i = 1

        -- Si no detecta rojo, seguir girando
        if c3 == 0 then
            for i, blob in ipairs(blobs) do
                local r, g, b = blob.color.red, blob.color.green, blob.color.blue
                local angle = blob.angle * (180 / math.pi)                        -- Obtener el ángulo del blob en Grados

                if r > 200 and g < 100 and b < 100 and math.abs(angle) <= 10 then -- Si detecta un objeto rojo
                    -- Imprimir los valores RGB detectados
                    --log(string.format("Blob %d detectado - R: %d, G: %d, B: %d", i, r, g, b))
                    --log("Angulo: " .. angle)

                    robot.wheels.set_velocity(0, 0)   -- Detenerse
                    robot.leds.set_all_colors("blue") -- Encender LEDs en azul
                    c3 = 1
                    log("Maestro detectado por " .. robot.id)
                    break -- No es necesario revisar más blobs
                end
            end
        else
            --log("Observador detectó demostrador y se detuvo")
            --robot.leds.set_all_colors("green")
        end
    else
        log("ERROR: Cámara no detectada.")
    end
end

function move_until_obstacle()
    -- Obtener los blobs detectados por la cámara
    local blobs = robot.colored_blob_omnidirectional_camera

    -- Verificar si hay blobs detectados
    if #blobs > 0 then
        local total_distance = 0
        local count = 0

        -- Recorrer todos los blobs y sumar sus distancias
        for i, blob in ipairs(blobs) do
            total_distance = total_distance + blob.distance
            count = count + 1
            --log(string.format("Blob %d: Distancia = %.4f", i, blob.distance))
        end

        -- Calcular distancia promedio
        local avg_distance = total_distance / count
        --log(string.format("Distancia promedio de blobs: %.4f", avg_distance))

        -- Si la distancia promedio es menor al umbral, detener el robot
        if avg_distance < threshold then
            robot.wheels.set_velocity(0, 0)    -- Detener el robot
            log("¡EStamos junto al maestro!, Deteniendo el " .. robot.id)
            robot.leds.set_all_colors("black") -- Encender LEDs en verde

            c3 = 2
            --robot.leds.set_all_colors("black")
        else
            -- Si no hay obstáculo, seguir avanzando
            robot.wheels.set_velocity(speed, speed) -- Velocidad fija
        end
    else
        -- Si no hay blobs detectados, el robot sigue moviéndose
        log("Maestro no disponible :( , continuando movimiento...")
        robot.leds.set_all_colors("black") -- Encender LEDs en verde

        --robot.wheels.set_velocity(10, 10)
        c3 = 0
    end
end

-------------------------------------------------------------------------------------------------------------------------------------
-------------------------------------------------------------------------------------------------------------------------------------
-------------------------------------------------------------------------------------------------------------------------------------
-------------------------------------------------------------------------------------------------------------------------------------
-------------------------------------------------------------------------------------------------------------------------------------
-- Funcines de movimiento de los epuck

-- Función para mover el e-puck una distancia y rotar a un ángulo específico

c1 = 0
c2 = 0
move_controller = 1

function move(distance, target_angle)
    --log("angulo = "..target_angle.."  "..robot.id)
    --log("distancia = "..distance.."  "..robot.id)
    if target_angle > 180 then
        target_angle = target_angle - 360
        --log("correccion 1  "..target_angle.."  "..robot.id)
    end
    if target_angle < -180 then
        target_angle = 360 + target_angle
        --log("correccion 2" ..target_angle.."  "..robot.id)
    end

    if distance < 0 then
        move_controller = -1
    else
        move_controller = 1
    end

    local t = timer
    -- Obtener el vector de posicion
    local posicion_x = robot.positioning.position.x
    local posicion_y = robot.positioning.position.y
    local posicion = { posicion_x, posicion_y }

    -- Obtener el cuaternión de orientación
    local orientation = robot.positioning.orientation
    vec2_angle(orientation)
    local angle = ang

    if c1 == 0 then
        --log("angulo actual :"..angle.."  y angulo deseados :" ..target_angle.."  "..robot.id)

        --log("valores deseado: distancia = "..distance.." angulo = "..target_angle)
        if angle < target_angle - 5 then
            robot.wheels.set_velocity(-g_speed, g_speed)
            --log("corrijiendo angulo 1, angulo actual = " .. angle)
        elseif angle > target_angle + 5 then
            robot.wheels.set_velocity(g_speed, -g_speed)
            --log("corrijiendo angulo 2, angulo actual = " .. angle)
        else
            robot.wheels.set_velocity(0, 0)
            c1 = 1

            --log("Angulo correjido = ".. angle)
            -- Calcular el tiempo necesario para recorrer la distancia
            time_needed = math.abs(distance) / speed
            -- Calcular el número de ticks basados en el timestep de 0.1 segundos
            ticks_needed = math.floor(time_needed / 0.1)

            ticks_needed = ticks_needed + t
        end
    end
    if c1 == 1 then    
        --log("recorriendo distancia  "..robot.id)
        -- Mover el robot durante el número de ticks calculado
        if t < ticks_needed then
            robot.wheels.set_velocity(speed * move_controller, speed * move_controller)
        else
            robot.wheels.set_velocity(0, 0)
            c1 = 2
            c2 = c2 + 1            
            --log("Robot: "..robot.id..",  posicion: x=" .. posicion[1] .. ", y=" .. posicion[2].." , Angle: " .. angle)

            --log("distancia recorrida  "..robot.id)
        end
    end
    return c2
end

--    Funciones de figuras

ind = 1
triangulo_ang = 0
function triangulo(demostrador_avg_angle)

    if sh_seg == 0 then
        imprimir_posicion()
        sh_seg = 1
    end
    
    if ind == 1 then
        triangulo_ang = demostrador_avg_angle
    end
    local params = {
        --{0,-60} ,
        { permt, -60 + triangulo_ang },
        { permt, -180 + triangulo_ang },
        { permt, 60 + triangulo_ang }
    }

    if b <= #params then
        local s1 = params[ind][1]
        local s2 = params[ind][2]
        b = move(s1, s2)

        if c1 == 2 then
            if b == #params then
                robot.leds.set_all_colors("green")
                k = 1
                b = b + 1
                c1 = 0;
                imprimir_posicion()
            else
                ind = ind + 1
                c1 = 0
                sh_seg = 0
            end
        end
    end

    if k == 1 then
        orientation = robot.positioning.orientation
        angle = vec2_angle(orientation)
        move(-100, angle)
        if c1 == 2 then
            resetDEM()
            robot.leds.set_all_colors("red")
            k = 0
        end
    end
end

function cuadro()
    local params = {
        --{0,0} ,
        { perm1c, -90 },
        { perm2c, -180 },
        { perm1c, 90 },
        { perm2c, 0 }
    }

    if b <= #params and k == 0 then
        local s1 = params[ind][1]
        local s2 = params[ind][2]

        b = move(s1, s2)

        if c1 == 2 then
            if b == #params then
                k = 1
                log("patrón original: cuadrado finalizado por: " .. robot.id)
                robot.leds.set_all_colors("green")
                c1 = 1
            elseif b < #params then
                ind = ind + 1
                c1 = 0
            end
        end
    end

    if k == 1 then
        orientation = robot.positioning.orientation
        vec2_angle(orientation)
        angle = ang
        move(50, ang)
        if c1 == 2 then
            resetDEM()
            robot.leds.set_all_colors("red")
            k = 0
        end

        if temp == false then
        end
    end
end

-------------------------------------------------------------------------------------------------------------------------------------
-------------------------------------------------------------------------------------------------------------------------------------
-------------------------------------------------------------------------------------------------------------------------------------
-------------------------------------------------------------------------------------------------------------------------------------
-------------------------------------------------------------------------------------------------------------------------------------
---
--- Funciones de operaciones y calculos

-- Función para calcular el ángulo a partir del vector de orientación
function vec2_angle(v)
    -- Convertir el cuaternión a un ángulo en el plano XY
    ang = math.atan2(2 * (v.w * v.z + v.x * v.y),
        1 - 2 * (v.y ^ 2 + v.z ^ 2))
    -- Convertir el ángulo de radianes a grados
    ang = ang * 180 / math.pi
    return ang
end

distances = {} -- Lista de posiciones relativas (x, y)
angles = {}    -- Lista de ángulos relativos


function store_relative_positions()
    if robot.colored_blob_omnidirectional_camera then
        local blobs = robot.colored_blob_omnidirectional_camera
        local sum_distance, sum_angle, count = 0, 0, 0

        for i, blob in ipairs(blobs) do
            if c4 == 0 then
                local r, g, b = blob.color.red, blob.color.green, blob.color.blue

                -- Filtrar solo blobs amarillos
                if r > 200 and g > 200 and b < 100 then
                    sum_distance = sum_distance + blob.distance
                    sum_angle = sum_angle + blob.angle
                    count = count + 1

                end
            end
        end


        -- Si detectamos blobs válidos, calcular el promedio
        if count > 0 then
            local avg_distance = sum_distance / count
            local avg_angle = sum_angle / count

            -- Guardar valores promediados en los vectores
            table.insert(distances, avg_distance)
            table.insert(angles, avg_angle)

            -- log(string.format("Promedio de blobs detectados: Distancia = %.2f, Ángulo = %.2f", avg_distance, avg_angle))

            -- Convertir a coordenadas cartesianas si es necesario

            --polarACartesianoPAR(avg_distance,avg_angle)
        elseif count == 0 then 
            c4 = 1
            robot.leds.set_all_colors("black")
            count = 0

            -- Imprimir los datos almacenados
            -- Imprimir las distancias almacenadas
            --[[
            log("Distancias relativas detectadas del : ".. robot.id)
            for i, dist in ipairs(distances) do
                log(string.format("Blob %d -> Distancia: %.2f", i, dist))
            end

            -- Imprimir los ángulos almacenados
            log("Ángulos relativos detectados del : ".. robot.id)
            for i, ang in ipairs(angles) do
                log(string.format("Blob %d -> Ángulo: %.2f rad", i, ang))
            end-- --]]
            return distances, angles
        end
    else
        log("ERROR: Cámara no detectada.")
    end
end

coordenadasx = {}
coordenadasy = {}

-- Función para convertir de polares a cartesianas
function polarACartesiano(v1, v2)
    local x
    local y

    --local coordenadas = {}

    -- Recorrer todas las mediciones
    for i = 1, #v1 do
        r = v1[i]
        theta = v2[i]
        y = r * math.sin(theta)
        x = r * math.cos(theta)
        table.insert(coordenadasx, x)
        table.insert(coordenadasy, y)
    end
    -- Imprimir los resultados en pares (x, y)
    --log("Coordenadas (x, y):")
    for i = 1, math.min(#coordenadasx, #coordenadasy) do
        --log(string.format("Coordenada %d -> (%.2f, %.2f)", i, coordenadasx[i], coordenadasy[i]))
    end
    c4 = 2
end

segmentos = {} -- Tabla para almacenar los segmentos (columnas)

function detect_seg(xx, yy)
    local k = 1       -- Contador de columnas
    local i = 1       -- Índice del vector de posiciones

    segmentos[k] = {} -- Inicializamos la primera columna

    while i <= #xx - avance do
        local x, y = xx[i], yy[i]
        local a, b = xx[i + avance], yy[i + avance]

        -- Si el siguiente punto (a, b) está dentro del rango, creamos nueva columna
        if a < x - sensitivity or a > x + sensitivity or b < y - sensitivity or b > y + sensitivity then
            -- Agregar el punto actual a la columna actual
            table.insert(segmentos[k], { x, y })
        else
            k = k + 1
            segmentos[k] = {} -- Nueva columna vacía
            i = i + avance    -- Avanzar en el índice
        end
        i = i + 1             -- Avanzar en el índice
    end

    -- Agregar el último punto a la última columna
    if i <= #xx then
        table.insert(segmentos[k], { xx[i], yy[i] })
    end

    -- Imprimir la tabla resultante
    for col, filas in pairs(segmentos) do
        -- log(string.format("Columna %d:", col))
        for fila, coord in ipairs(filas) do
            --  log(string.format("{%.2f, %.2f}", coord[1], coord[2]))
        end
    end

    c4 = 3
    return segmentos -- Retornar la tabla con los segmentos formados
end

equations = {}

function apply_linear_regression(segmentos)
    local functions = {} -- Tabla para almacenar las funciones resultantes

    for col, points in pairs(segmentos) do
        if #points > 1 then -- Aplicar solo si hay más de un punto
            local sum_x, sum_y, sum_xy, sum_x2 = 0, 0, 0, 0
            local n = #points

            -- Calcular sumatorias necesarias
            for _, point in ipairs(points) do
                local x, y = point[1], point[2]
                sum_x = sum_x + x
                sum_y = sum_y + y
                sum_xy = sum_xy + x * y
                sum_x2 = sum_x2 + x * x
            end

            -- Calcular pendiente m y la intersección b
            local denominator = (n * sum_x2 - sum_x * sum_x)
            if denominator ~= 0 then
                local m = (n * sum_xy - sum_x * sum_y) / denominator
                local b = (sum_y - m * sum_x) / n

                -- Guardar la ecuación resultante
                functions[col] = { m = m, b = b }

                --Imprimir la ecuación de la recta
                -- log(string.format("Columna %d: y = %.2fx + %.2f", col, m, b))
            else
                -- log(string.format("Columna %d: No se pudo calcular la regresión lineal", col))
            end
        end
    end

    local function_N = {}

    for col, eq in pairs(functions) do
        function_N[col] = { eq.m, eq.b }
    end

    local index = 1 -- Nuevo índice secuencial

    for _, eq in pairs(function_N) do
        equations[index] = eq
        index = index + 1
    end

    for col, eq in pairs(equations) do
        --log(string.format("Verificando funciones -> Columna %d: y = %.2fx + %.2f", col, eq.m, eq.b))

        local m, b = eq[1], eq[2] -- Extraer m y b correctamente
        --log("Robot " .. robot.id .. string.format(",  Columna %d: y = %.2fx + %.2f", col, m, b))
    end

    c4 = 4

    return equations
end

intersections = {}

function calculate_intersections(equations_f)
    local cols = {}

    for i = 1, #equations_f do
        table.insert(cols, i)
    end

    table.insert(cols, cols[1])

    --local cols = {4, 5, 10, 11, 4}  -- Columnas en orden para formar el cuadrilátero
    for i = 1, #cols - 1 do
        local col1, col2 = cols[i], cols[i + 1]
        local m1, b1 = equations_f[col1][1], equations[col1][2]
        local m2, b2 = equations_f[col2][1], equations[col2][2]

        if m1 ~= m2 then
            local x = (b2 - b1) / (m1 - m2)
            local y = m1 * x + b1
            table.insert(intersections, { x, y })
            --log("Robot " .. robot.id .. string.format("Intersección entre col %d y %d: (%.2f, %.2f)", col1, col2, x, y))
        end
    end

    c4 = 5

    return intersections
end

segments = {}

function calculate_segments(intersections)
    log(string.format("Total numero de segementos: %d", #intersections)) -- Ver cuántos puntos hay

    for i = 1, #intersections do
        local x1, y1 = intersections[i][1], intersections[i][2]
        local x2, y2 = intersections[(i % #intersections) + 1][1], intersections[(i % #intersections) + 1][2]

        --log(string.format("Conectando P%d (%.2f, %.2f) con P%d (%.2f, %.2f)",  i, x1, y1, (i % #intersections) + 1, x2, y2))

        local distance = math.sqrt((x2 - x1) ^ 2 + (y2 - y1) ^ 2)
        local angle = math.atan2(y2 - y1, x2 - x1) * (180 / math.pi)

        table.insert(segments, { distance, angle })
        log("Robot " .. robot.id .. string.format("Segmento %d: Distancia = %.2f cm, Ángulo = %.2f grados", i, distance, angle))
    end

    c4 = 6

    return segments
end

-- Función para convertir de polares a cartesianas pares de valores

function polarACartesianoPAR(r, theta)
    local x
    local y

    y = r * math.sin(theta)
    x = r * math.cos(theta)

    log("x = " .. x .. ", y =" .. y)
end

-- --------------------------------------------------------------------------------------------------
-- Función para limpiar el estado al finalizar
function reset()
    timer = 0
    c1 = 0
    ang = 0
    time_needed = 0
    ticks_needed = 0
    speed = 0
end

-- --------------------------------------------------------------------------------------------------
-- Función que se ejecuta al finalizar la simulación
function destroy()
end
