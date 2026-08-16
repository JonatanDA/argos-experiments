-- Variable global 

valor1 = 0
valor2 = 0

indice = 1

timer = 1
led_timer = 50;
c1 = 0
ang = 0
time_needed = 0
ticks_needed = 0
speed = 5
g_speed = 5

c2 = 0

c3 = 0
c4 = 0

d1 = 0


b = 0

is_demonstrator = 0

math.randomseed(os.time())

perm1c = 10
perm2c = 10
permt = 10

sensitivity = 1.5
avance = 6

-- --------------------------------------------------------------------------------------------------

-- Función que se ejecuta al inicio de la simulación

function init()

    robot.colored_blob_omnidirectional_camera.enable()

    posicion_x = robot.positioning.position.x
    posicion_y = robot.positioning.position.y
    posicion = {posicion_x, posicion_y}

    orientation = robot.positioning.orientation
	vec2_angle(orientation)
	angle = ang

    -- Determinar si el epuck sera observador o demostrador
    determinar_obs_dem()
end

----------------------------------------------------------------------------------------------------
----------------------------------------------------------------------------------------------------
----------------------------------------------------------------------------------------------------
-- Función principal que se ejecuta en cada tick
function step()

    -- Ejecutar el patrón si es demostrador
    if is_demonstrator == 1 then   
        local blobs = robot.colored_blob_omnidirectional_camera
        if d1 == 0 then
            RandomWalking()

            for i, blob in ipairs(blobs) do
                local r, g, b = blob.color.red, blob.color.green, blob.color.blue            
                
                if r < 100 and g < 100 and b > 200  then  -- Si detecta un objeto azul
                    d1 = 1
                    log("Alumno detectado! por: "..robot.id)
                    robot.wheels.set_velocity(0,0)
                    robot.leds.set_all_colors("yellow")
                    break
                end
            end

        elseif d1 == 1 then

            if led_on then
                led_timer = led_timer - 1
                if led_timer <= 0 then
                    led_on = false
                    log("Demostración iniciada por " ..robot.id)
                end
            else
                if robot.id == "epuck2" or robot.id == "epuck5" or robot.id == "epuck1" then
                    triangulo()
                    --Ejecutar patrón 1, del demostrador  
                else
                    triangulo()
                    --Ejecutar patrón 2, del demostrador 
                end  
            end 
        end    
    else       
        if c3 == 0 then       
            RandomWalking()
            detect_and_stop()
        elseif c3 == 1 then
            move_until_obstacle()
        elseif c3 == 2 then
            if c4 == 0 then
                store_relative_positions()  
            elseif c4==1 then               
                polarACartesiano(distances, angles) 
            elseif c4==2 then
                detect_seg(coordenadasx, coordenadasy)
            elseif c4==3 then
                apply_linear_regression(segmentos)
            elseif c4==4 then
                calculate_intersections(equations)
            elseif c4==5 then

                calculate_segments(intersections)
                if #segments > 2 then
                    valor1, valor2 = segments[indice][1], segments[indice][2]
                else
                    resetOBS()
                    robot.leds.set_all_colors("black")
                    log("Patrón no detectado :( ".. robot.id)
                end

            elseif c4==6 then
                move(valor1,valor2)
                if c1 == 2 then
                    if indice == #segments then
                        espere(0)
                        if temp == false then
                            log("Patrón imitado, finalizado...."..robot.id)
                            resetOBS()
                        end
                    else
                        indice = indice  + 1
                        valor1, valor2 = segments[indice][1], segments[indice][2]   
                        c1 = 0
                    end
                end
            end     
        end  
    end
    timer = timer + 1
end


-------------------------------------------------------------------------------------------------------------------------------------
-------------------------------------------------------------------------------------------------------------------------------------
-------------------------------------------------------------------------------------------------------------------------------------
-------------------------------------------------------------------------------------------------------------------------------------
-------------------------------------------------------------------------------------------------------------------------------------
-- Funciones para cada robot epuck

function epuck0 ()
    --triangulo()
end

function epuck1 ()
    --cuadro()
end

-------------------------------------------------------------------------------------------------------------------------------------
-------------------------------------------------------------------------------------------------------------------------------------
-------------------------------------------------------------------------------------------------------------------------------------
-------------------------------------------------------------------------------------------------------------------------------------
-------------------------------------------------------------------------------------------------------------------------------------

-- Funciones Varias

function resetOBS()
    temp = true
    cr = 0
    crono = 0

    distances = {}  
    angles = {} 

    coordenadasx = {}
    coordenadasy = {}

    segments = {}

    intersections = {}

    segmentos = {} 

    equations  = {}

    indice = 1

    c1 = 0
    c2 = 0

    c3 = 0
    c4= 0


    firstime = 0
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

end

temp = true
cr = 0
crono = 0

function espere(c)

    crono = c - cr
    cr = cr + 1
    if crono <= 0 then
        temp = false
        cr = o
    end
end

function print_table(tbl)
    for key, value in pairs(tbl) do
        if type(value) == "table" then
            log(key .. ":")
            print_table(value)  -- Si el valor es una tabla, recursivamente imprimimos sus claves
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
--- Funciones de operaciones y calculos

-- Función para calcular el ángulo a partir del vector de orientación
function vec2_angle(v)
    -- Convertir el cuaternión a un ángulo en el plano XY
    ang = math.atan2(2 * (v.w * v.z + v.x * v.y),
                             1 - 2 * (v.y^2 + v.z^2))
    -- Convertir el ángulo de radianes a grados
    ang = ang * 180 / math.pi
    
    return ang
end


distances = {}  -- Lista de posiciones relativas (x, y)
angles = {}  -- Lista de ángulos relativos

function store_relative_positions()
    if robot.colored_blob_omnidirectional_camera then
        local blobs = robot.colored_blob_omnidirectional_camera
        local sum_distance, sum_angle, count = 0, 0, 0

        for i, blob in ipairs(blobs) do

            if c4 == 0 then
                local r, g, b = blob.color.red, blob.color.green, blob.color.blue
                -- Filtrar solo blobs rojos          
                if r > 200 and g > 200 and b < 100  then 
                    sum_distance = sum_distance + blob.distance
                    sum_angle = sum_angle + blob.angle
                    count = count + 1
                else
                    c4 = 1
                    robot.leds.set_all_colors("black")
                    count = 0
                    return distances, angles  
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
    -- Recorrer todas las mediciones
    for i = 1, #v1 do
        r = v1[i]
        theta = v2[i]
        y = r * math.sin(theta)
        x = r * math.cos(theta)
        table.insert(coordenadasx, x)
        table.insert(coordenadasy, y)

    end
    c4 = 2

end


segmentos = {}  -- Tabla para almacenar los segmentos (columnas)

function detect_seg(xx, yy)
    local k = 1  -- Contador de columnas
    local i = 1  -- Índice del vector de posiciones
    segmentos[k] = {}  -- Inicializamos la primera columna

    while i <= #xx - avance do
        local x, y = xx[i], yy[i]
        local a, b = xx[i + avance], yy[i + avance]

        -- Si el siguiente punto (a, b) está dentro del rango, creamos nueva columna
        if a < x - sensitivity or a > x + sensitivity or b < y - sensitivity or b > y + sensitivity then
        -- Agregar el punto actual a la columna actual
            table.insert(segmentos[k], {x, y})
		  else 
            k = k + 1
            segmentos[k] = {}  -- Nueva columna vacía
            i = i + avance  -- Avanzar en el índice
        end
        i = i + 1 -- Avanzar en el índice
    end

    -- Agregar el último punto a la última columna
    if i <= #xx then
        table.insert(segmentos[k], {xx[i], yy[i]})
    end

    c4 = 3
    return segmentos  -- Retornar la tabla con los segmentos formados
end


equations  = {}

function apply_linear_regression(segmentos)
    local functions = {}  -- Tabla para almacenar las funciones resultantes

    for col, points in pairs(segmentos) do
        if #points > 1 then  -- Aplicar solo si hay más de un punto
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
                functions[col] = {m = m, b = b}

            else
                log(string.format("Columna %d: No se pudo calcular la regresión lineal", col))
            end
        end
    end

    local function_N = {}

    for col, eq in pairs(functions) do
        function_N[col] = {eq.m, eq.b}
    end

    local index = 1  -- Nuevo índice secuencial

    for _, eq in pairs(function_N) do
        equations[index] = eq
        index = index + 1
    end

    for col, eq in pairs(equations) do
        local m, b = eq[1], eq[2]  -- Extraer m y b correctamente
        log("Robot ".. robot.id..string.format(",  Columna %d: y = %.2fx + %.2f", col, m, b))
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

    for i = 1, #cols - 1 do
        local col1, col2 = cols[i], cols[i+1]
        local m1, b1 = equations_f[col1][1], equations[col1][2]
        local m2, b2 = equations_f[col2][1], equations[col2][2]

        if m1 ~= m2 then
            local x = (b2 - b1) / (m1 - m2)
            local y = m1 * x + b1
            table.insert(intersections, {x, y})
            log(string.format("Intersección entre col %d y %d: (%.2f, %.2f)", col1, col2, x, y))
        end
    end

    c4 = 5
    return intersections
end

segments = {}


function calculate_segments(intersections)

    log(string.format("Total numero de segementos: %d", #intersections))  -- Ver cuántos puntos hay

    for i = 1, #intersections do
        local x1, y1 = intersections[i][1], intersections[i][2]
        local x2, y2 = intersections[(i % #intersections) + 1][1], intersections[(i % #intersections) + 1][2]  
        local distance = math.sqrt((x2 - x1)^2 + (y2 - y1)^2)
        local angle = math.atan2(y2 - y1, x2 - x1) * (180 / math.pi)

        table.insert(segments, {distance, angle})
        log(string.format("Segmento %d: Distancia = %.2f cm, Ángulo = %.2f grados", i, distance, angle))
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

    log("x = "..x..", y ="..y)
end 

-------------------------------------------------------------------------------------------------------------------------------------
-------------------------------------------------------------------------------------------------------------------------------------
-------------------------------------------------------------------------------------------------------------------------------------
-------------------------------------------------------------------------------------------------------------------------------------
-------------------------------------------------------------------------------------------------------------------------------------
-- Funciones de comportamiento de los epuck


-- función  de caminata aleatoria
function RandomWalking()
    
    -- Generar un giro aleatorio ocasionalmente
    local random = robot.random.uniform_int(0,40)
    if  random >= 10 and random <= 20 then
        robot.wheels.set_velocity(random, -random)
    elseif  random >= 25  then
        robot.wheels.set_velocity(random, random)

    end
end

-- Función para determiaar si un epuck será observador o demostrador
function determinar_obs_dem()

    -- Selección aleatoria del demostrador

    randoms=robot.random.uniform_int(0,2)
    if randoms == 1 then
        is_demonstrator = 1
    else
        is_demonstrator = 0
    end

    if robot.id == "epuck0" then
        is_demonstrator = 1
    elseif robot.id == "epuck2" then
        is_demonstrator = 0
    elseif robot.id == "epuck4" then
        is_demonstrator = 1

    elseif robot.id == "epuck1" then
        is_demonstrator = 1
    elseif robot.id == "epuck3" then
        is_demonstrator = 0
    elseif robot.id == "epuck5" then
        is_demonstrator = 1
    end  

    -- Encender LED rojo si es demostrador
    if is_demonstrator == 1 then
		robot.leds.set_all_colors("red")  -- Cambia solo un LED específico
        led_on = true
        log("The " ..robot.id.. " is demostrador = " .. randoms)
    else
        log("The " ..robot.id.. " is Observador= " .. randoms) 
    end    


    return is_demonstrator
end

-- Función de busuqeda ejecutada por el observador
function detect_and_stop()
    if robot.colored_blob_omnidirectional_camera then
        local blobs = robot.colored_blob_omnidirectional_camera
		local i = 1

        -- Si no detecta rojo, seguir girando
        if c3 == 0 then
            for i, blob in ipairs(blobs) do
                local r, g, b = blob.color.red, blob.color.green, blob.color.blue            
                local angle = blob.angle *(180/math.pi) -- Obtener el ángulo del blob en Grados
                
                if r > 200 and g < 100 and b < 100 and math.abs(angle)<= 10  then  -- Si detecta un objeto rojo
                    robot.wheels.set_velocity(0, 0)  -- Detenerse
                    robot.leds.set_all_colors("blue")  -- Encender LEDs en verde
                    c3 = 1
                    log("Maestro detectado por " ..robot.id)
                    break  -- No es necesario revisar más blobs             
                end
            end		
        end

    else
        log("ERROR: Cámara no detectada.")
    end
end 


function move_until_obstacle()

    local threshold = 10  -- Umbral de 2 cm en unidades normalizadas
    -- Obtener los blobs detectados por la cámara
    local blobs = robot.colored_blob_omnidirectional_camera

    -- Verificar si hay blobs detectados
    if #blobs > 0 then
        local total_distance = 0
        local count = 0

        -- Recorrer todos los blobs y sumar sus distancias
        for i , blob in ipairs(blobs) do
            total_distance = total_distance + blob.distance
            count = count + 1
        end

        -- Calcular distancia promedio, si la distancia promedio es menor al umbral, detener el robot
        local avg_distance = total_distance / count
        if avg_distance < threshold then
            robot.wheels.set_velocity(0, 0)  -- Detener el robot
            log("¡EStamos junto al maestro!, Deteniendo el "..robot.id)
            robot.leds.set_all_colors("black")  -- Encender LEDs en verde
            c3 = 2
        else
            -- Si no hay obstáculo, seguir avanzando
            robot.wheels.set_velocity(5, 5)  -- Velocidad fija
        end
    else
        -- Si no hay blobs detectados, el robot sigue moviéndose
        log("Maestro no disponible :( , continuando movimiento...")
        robot.leds.set_all_colors("black")  -- Apagar LEDs
        c3 = 0
    end
end

-------------------------------------------------------------------------------------------------------------------------------------
-------------------------------------------------------------------------------------------------------------------------------------
-------------------------------------------------------------------------------------------------------------------------------------
-------------------------------------------------------------------------------------------------------------------------------------
-------------------------------------------------------------------------------------------------------------------------------------
-- Funcines de movimiento de los epuck


function proximidad()        
    
    local blobs = robot.colored_blob_omnidirectional_camera
    local total_distance = 0
    local count = 0
    local total_angle = 0   
    local c = 1
    local dist = 0
    local ang

    if #blobs > 0 then
        -- Recorrer todos los blobs y sumar sus distancias
        for i, blob in ipairs(blobs) do
            total_distance = total_distance + blob.distance
            total_angle = total_angle  + blob.angle
            count = count + 1
        end
        dist = total_distance / count
        ang = (total_angle / count ) * 180/math.pi
        return ang
    end
end

-- Función para mover el e-puck una distancia y rotar a un ángulo específico
firstime = 0

function move(distance, target_angle)

    if firstime == 0 then
        local posicion_x = robot.positioning.position.x
        local posicion_y = robot.positioning.position.y
        local posicion = {posicion_x, posicion_y}        
        firstime = 1
    end
    
	local t = timer
  -- Obtener el vector de posicion
    local posicion_x = robot.positioning.position.x
    local posicion_y = robot.positioning.position.y
    local posicion = {posicion_x, posicion_y}
    -- Obtener el cuaternión de orientación
    local orientation = robot.positioning.orientation
	vec2_angle(orientation)
	local angle = ang

    if c1 == 0 then
        if angle < target_angle - 5 then
            robot.wheels.set_velocity(-g_speed, g_speed)
        elseif angle > target_angle + 5 then
            robot.wheels.set_velocity(g_speed, -g_speed)
        else 
            robot.wheels.set_velocity(0, 0)
            c1 = 1 
   		    time_needed = distance / speed
  		    ticks_needed = math.floor(time_needed / 0.1) 
            ticks_needed = ticks_needed + t
        end
    end
    if c1 == 1 then
        -- Mover el robot durante el número de ticks calculado
        if t < ticks_needed then
            robot.wheels.set_velocity(speed, speed)
        else
            -- Detener el robot al finalizar
            robot.wheels.set_velocity(0, 0)
            c1 = 2
            c2 = c2 +1       
        end
    end 
	return c2
end



--    Funciones de figuras 

ind = 1
function triangulo()

    local params = {
        --{0,-60} ,
        {permt,-60} ,
        {permt,-180} ,
        {permt,60} 
    }

    if b <= #params  then
        local s1 = params[ind][1]
        local s2 = params[ind][2]
        local ang

        b = move(s1,s2)

        if c1 == 2 then
            if b == #params then
                  robot.leds.set_all_colors("blue")
                k = 1
            else
                ind = ind + 1
                c1 = 0
            end
        end
    end

    if k == 1 then
        
        orientation = robot.positioning.orientation
        vec2_angle(orientation)
        angle = ang
        move(50,ang)
        if c1 == 2 then     
            resetDEM()
            robot.leds.set_all_colors("red")   
            k = 0
        end
    end
end

k = 0
function cuadro()

    local params = {
        --{0,0} ,
        {perm1c,-70},
        {perm2c,70},
        {perm1c,-70},
        {perm2c,70} 
    }

    if b <= #params  and k == 0 then
        local s1 = params[ind][1]
        local s2 = params[ind][2] 
        b = move(s1,s2)
        if c1 == 2 then
            if b == #params then

                k = 1
                log("patrón original: cuadrado finalizado por: "..robot.id)
                robot.leds.set_all_colors("blue")
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
        move(50,ang)
        if c1 == 2 then     
            resetDEM()
            robot.leds.set_all_colors("red")   
            k = 0
        end
    end
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
