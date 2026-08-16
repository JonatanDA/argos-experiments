-- Variable global 

-- Variables globales para almacenar posiciones y ángulos relativos
distances = {}  -- Lista de posiciones relativas (x, y)
angles = {}  -- Lista de ángulos relativos


valor1 = 0
valor2 = 0

timer = 1
led_timer = 50;
c1 = 0
c2 = 0


c3 = 0

c4 = 0

ang = 0
time_needed = 0
ticks_needed = 0
speed = 3
g_speed = 3
b = 0
is_demonstrator = 0
math.randomseed(os.time())

perm1c = 8
perm2c = 20

permt = 10
-- --------------------------------------------------------------------------------------------------

-- Función que se ejecuta al inicio de la simulación

function init()

    --for key, _ in pairs(robot) do
    	--  log("Dispositivo detectado: " .. key)
    --end

    log("Robot " .. robot.id .. " iniciado.") 
	robot.colored_blob_omnidirectional_camera.enable()


     -- Obtener el vector de posicion inicial
    posicion_x = robot.positioning.position.x
    posicion_y = robot.positioning.position.y
    posicion = {posicion_x, posicion_y}
	 log("posicion: x=" .. posicion[1] .. ", y=" .. posicion[2])
     -- Obtener el vector de orientacion inicial
    orientation = robot.positioning.orientation
	 vec2_angle(orientation)
	 angle = ang
    -- Imprimir el ángulo
 	 log("Angle: " .. angle)

      determinar_obs_dem()
end

-- --------------------------------------------------------------------------------------------------

-- Función principal que se ejecuta en cada tick
function step()

    if led_on then
        led_timer = led_timer - 1
        if led_timer <= 0 then
            --robot.leds.set_all_colors("black")
            led_on = false
            log("Demostración iniciada por " ..robot.id)
        end
    end

    -- Ejecutar el patrón si es demostrador
    if is_demonstrator == 1 then
        if not led_on then
            --robot.leds.set_all_colors("blue")
            if robot.id == "epuck2" or robot.id == "epuck5" or robot.id == "epuck1" then
                cuadro()
                --log("Patron 1 (cuadro) ejecutandose")   
            else
                triangulo()
                --log("Patron 2( triangulo) ejecutandose")
            end        
        end
    else        
        detect_and_stop()
        if c3 == 1 then
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
                valor1, valor2 = segments[indice][1], segments[indice][2]
            elseif c4==6 then
                move(valor1,valor2)
                if c1 == 2 then
                    --log("maximo = "..#segments)
                    --log("indice = "..indice)
                    if indice == #segments then
                        --c2 = 0
                        c1 = 0
                        c4=7
                        --log(" Ultimo Segmento: "..indice.." finalizado")
                        log("Segmentos finalizados")
                    else
                       -- log("Segmento numero: "..indice.." finalizado")
                        indice = indice  + 1
                        valor1, valor2 = segments[indice][1], segments[indice][2]   
                        --c2 = 0
                        c1 = 0
                    end
                end
            end
           
            --log("analizando movimiento por ".. robot.id)         
        end
        --log("Buscando demostrador")
    end

    --execute_movements(segments)

    if is_demonstrator == 0 then  -- Solo el observador ejecuta esto

    end

    timer = timer + 1

end


-- --------------------------------------------------------------------------------------------------
-- Funciones para cada robot epuck

function epuck0 ()
    --triangulo()
end

function epuck1 ()
    --cuadro()
end



-- --------------------------------------------------------------------------------------------------
-- Funciones Varias
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

-- --------------------------------------------------------------------------------------------------
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
    log("Coordenadas (x, y):")
    for i = 1, math.min(#coordenadasx, #coordenadasy) do
       -- log(string.format("Coordenada %d -> (%.2f, %.2f)", i, coordenadasx[i], coordenadasy[i]))
    end
    c4 = 2

end


segmentos = {}  -- Tabla para almacenar los segmentos (columnas)

function detect_seg(xx, yy)
    local k = 1  -- Contador de columnas
    local i = 1  -- Índice del vector de posiciones

    segmentos[k] = {}  -- Inicializamos la primera columna

    while i <= #xx - 10 do
        local x, y = xx[i], yy[i]
        local a, b = xx[i + 10], yy[i + 10]

        -- Si el siguiente punto (a, b) está dentro del rango, creamos nueva columna
        if a < x - 1 or a > x + 1 or b < y - 1 or b > y + 1 then
        -- Agregar el punto actual a la columna actual
            table.insert(segmentos[k], {x, y})
		  else 
            k = k + 1
            segmentos[k] = {}  -- Nueva columna vacía
        end


        i = i + 10  -- Avanzar en el índice
    end

    -- Agregar el último punto a la última columna
    if i <= #xx then
        table.insert(segmentos[k], {xx[i], yy[i]})
    end

    -- Imprimir la tabla resultante
    for col, filas in pairs(segmentos) do
        --log(string.format("Columna %d:", col))
        for fila, coord in ipairs(filas) do
           -- log(string.format("{%.2f, %.2f}", coord[1], coord[2]))
        end
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

                -- Imprimir la ecuación de la recta
                --log(string.format("Columna %d: y = %.2fx + %.2f", col, m, b))
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
       --log(string.format("Verificando funciones -> Columna %d: y = %.2fx + %.2f", col, eq.m, eq.b))

    		local m, b = eq[1], eq[2]  -- Extraer m y b correctamente
            log(string.format("Columna %d: y = %.2fx + %.2f", col, m, b))
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
        local col1, col2 = cols[i], cols[i+1]
        local m1, b1 = equations_f[col1][1], equations[col1][2]
        local m2, b2 = equations_f[col2][1], equations[col2][2]

        if m1 ~= m2 then
            local x = (b2 - b1) / (m1 - m2)
            local y = m1 * x + b1
            table.insert(intersections, {x, y})
            --log(string.format("Intersección entre col %d y %d: (%.2f, %.2f)", col1, col2, x, y))
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

        --log(string.format("Conectando P%d (%.2f, %.2f) con P%d (%.2f, %.2f)",  i, x1, y1, (i % #intersections) + 1, x2, y2))

        local distance = math.sqrt((x2 - x1)^2 + (y2 - y1)^2)
        local angle = math.atan2(y2 - y1, x2 - x1) * (180 / math.pi)

        table.insert(segments, {distance, angle})
        --log(string.format("Segmento %d: Distancia = %.2f cm, Ángulo = %.2f grados", i, distance, angle))
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

    --log("x = "..x..", y ="..y)
end 

--------------------------------------------------------------------------------------------------
-- Funciones de comportamiento de los epuck

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
        is_demonstrator = 1
    end  


    --if robot.id == "epuck2" or robot.id == "epuck3" then
      --  is_demonstrator = 0
    --end

    -- Encender LED rojo si es demostrador
    if is_demonstrator == 1 then
		robot.leds.set_all_colors("red")  -- Cambia solo un LED específico
        led_on = true
        log("The " ..robot.id.. " is demostrador = " .. randoms)

        --led_timer = 10 -- 5 segundos a 10 ticks por segundo
        --log("Led rojo encendido por " ..robot.id)
    else
        --robot.leds.set_all_colors("blue")   
        log("The " ..robot.id.. " is Observador= " .. randoms) 
    end    


    return is_demonstrator
end

-- Función de busuqeda ejecutada por el observador


function detect_and_stop()
    if robot.colored_blob_omnidirectional_camera then
        local blobs = robot.colored_blob_omnidirectional_camera
		--log("Cantidad de blobs detectados: " .. tostring(#blobs).. ", por " .. robot.id)
		local i = 1

        -- Si no detecta rojo, seguir girando
        if c3 == 0 then
            robot.wheels.set_velocity(5, -5)  
            for i, blob in ipairs(blobs) do
                local r, g, b = blob.color.red, blob.color.green, blob.color.blue            
                local angle = blob.angle *(180/math.pi) -- Obtener el ángulo del blob en Grados
                
                if r > 200 and g < 100 and b < 100 and math.abs(angle)<= 10  then  -- Si detecta un objeto rojo
                    -- Imprimir los valores RGB detectados
                    --log(string.format("Blob %d detectado - R: %d, G: %d, B: %d", i, r, g, b))
                    --log("Angulo: " .. angle)
                    
                    robot.wheels.set_velocity(0, 0)  -- Detenerse
                    --robot.leds.set_all_colors("green")  -- Encender LEDs en verde
                    c3 = 1
                    log("Demosreador detectado por " ..robot.id.. ", c3 = "..c3)
                    --found_front_blob = true
                    break  -- No es necesario revisar más blobs             
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


function store_relative_positions()
    if robot.colored_blob_omnidirectional_camera then
        local blobs = robot.colored_blob_omnidirectional_camera
        local sum_distance, sum_angle, count = 0, 0, 0


        for i, blob in ipairs(blobs) do

            if c4 == 0 then
                local r, g, b = blob.color.red, blob.color.green, blob.color.blue

                -- Filtrar solo blobs rojos
                if r > 200 and g < 100 and b < 100 then  
                    if robot.id == "epuck1" then
                        --log("Blob detectado - Ángulo: " .. blob.angle .. " - Distancia: " .. blob.distance)
                    end

                    sum_distance = sum_distance + blob.distance
                    sum_angle = sum_angle + blob.angle
                    count = count + 1
                else
                    c4 = 1
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
            polarACartesianoPAR(avg_distance,avg_angle)
        end
    else
        log("ERROR: Cámara no detectada.")
    end
end



--Ejecutar el patron imitado
indice = 1

function execute_movements(val1,val2)

    --local distancia, angulo = seg[indice][1], seg[indice][2]
    local distancia, angulo = val1, val2
    --log("Moviendo: "..distancia.." cm en dirección: "..angulo)


    move(distancia, angulo) -- epuck0 se mueve 100 cm y gira a 60 grados   

end

-- --------------------------------------------------------------------------------------------------
-- Funcines de movimiento de los epuck

-- Función para mover el e-puck una distancia y rotar a un ángulo específico



function move(distance, target_angle)

	local t = timer
  -- Obtener el vector de posicion
    local posicion_x = robot.positioning.position.x
    local posicion_y = robot.positioning.position.y
    local posicion = {posicion_x, posicion_y}

    -- Obtener el cuaternión de orientación
    local orientation = robot.positioning.orientation
	vec2_angle(orientation)
	local angle = ang

    --log("angulo actual :"..angle)y angulo deseados :" 

    if c1 == 0 then

        --log("valores deseado: distancia = "..distance.." angulo = "..target_angle)
        if angle < target_angle - 5 then
            robot.wheels.set_velocity(-g_speed, g_speed)
				--log("corrijiendo angulo 1, angulo actua = " .. angle)
        elseif angle > target_angle + 5 then
            robot.wheels.set_velocity(g_speed, -g_speed)
				--log("corrijiendo angulo 2, angulo actual = " .. angle)
        else 
            robot.wheels.set_velocity(0, 0)
            c1 = 1 
		    
			--log("Angulo correjido = ".. angle)
  
   		   -- Calcular el tiempo necesario para recorrer la distancia
   		    time_needed = distance / speed
  		      -- Calcular el número de ticks basados en el timestep de 0.1 segundos
  		    ticks_needed = math.floor(time_needed / 0.1) 

            ticks_needed = ticks_needed + t
        end
    end
    if c1 == 1 then

        --log("recorriendo distancia ----")
        -- Mover el robot durante el número de ticks calculado

        if t < ticks_needed then
            robot.wheels.set_velocity(speed, speed)
				--log("Segmento c1 = " .. c1)
        else
            -- Detener el robot al finalizar
            robot.wheels.set_velocity(0, 0)
            log("posicion: x=" .. posicion[1] .. ", y=" .. posicion[2])
            -- Imprimir el ángulo
            log("Angle: " .. angle)
            c1 = 2

            c2 = c2 + 1
        end
    end 

	 return c2
end



--    Funciones de figuras 


function triangulo()

    if b == 0 then
        b = move(0, 0) -- epuck0 se mueve 100 cm y gira a 60 grados   
        if b == 1 then
		      --log("SEgmento  1 Finalizado")
            c1 = 0;
        end

    end
    
    if b == 1 then
        b = move(permt, 120) -- epuck0 se mueve 100 cm y gira a 60 grados
        if b == 2 then
		      --log("SEgmento 2 Finalizado")
            c1 = 0
        end
    end

    if b == 2 then
        b = move(permt, -120) -- epuck0 se mueve 100 cm y gira a 60 grados
        if b == 3 then
		      --log("SEgmento  3 Finalizado")
            c1 = 0
        end
    end

    if b == 3 then
        b = move(permt, 0) -- epuck0 se mueve 100 cm y gira a 60 grados
        if b == 4 then
            --log("SEgmento  4 Finalizado")
            robot.leds.set_all_colors("green")  -- Encender LEDs en verde
            c1 = 0
            b = b + 1
        end
    end
end



function cuadro()

    if b == 0 then
        b = move(0, 0) -- epuck0 se mueve 100 cm y gira a 60 grados   
        if b == 1 then
		      --log("SEgmento  1 Finalizado")
            c1 = 0;
        end

    end
    
    if b == 1 then
        b = move(perm1c, 90) -- epuck0 se mueve 100 cm y gira a 60 grados
        if b == 2 then
		    --log("SEgmento 2 Finalizado")
            c1 = 0
        end
    end

    if b == 2 then
        b = move(perm2c, 180) -- epuck0 se mueve 100 cm y gira a 60 grados
        if b == 3 then
		    --log("SEgmento  3 Finalizado")
            c1 = 0
        end
    end

    if b == 3 then
        b = move(perm1c, -90) -- epuck0 se mueve 100 cm y gira a 60 grados
        if b == 4 then
           --log("SEgmento  4 Finalizado")
            c1 = 0
        end
    end

    if b == 4 then
        b = move(perm2c, 0) -- epuck0 se mueve 100 cm y gira a 60 grados
        if b == 5 then
           -- log("SEgmento  5 Finalizado")
            robot.leds.set_all_colors("green")  -- Encender LEDs en verde
            c1 = 0
            b = b + 1
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
