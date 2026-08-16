-- Variable global para el temporizador
timer = 1
c1 = 0
c2 = 0
ang = 0
time_needed = 0
ticks_needed = 0
speed = 10
b = 0
is_demonstrator = false

-- Función que se ejecuta al inicio de la simulación
function init()

  

    log("Robot " .. robot.id .. " iniciado.")

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

    if robot.id == "epuck0" or robot.id == "epuck1" then
       is_demonstrator = determinar_obs_dem()
    end

end

-- Función principal que se ejecuta en cada tick
function step()

    if led_on then
        led_timer = led_timer - 1
        if led_timer <= 0 then
            robot.leds.set_all_colors("black")
            led_on = false
            log("Led apagado por " ..robot.id)
        end
    end

    -- Ejecutar el patrón si es demostrador
    if is_demonstrator and not led_on then
        if robot.id == "epuck0" then
            epuck0()
            --log("Patron de epuck0 ejecutandose")
        elseif robot.id == "epuck1" then
            epuck1()
            --log("Patron de epuck1 ejecutandose")
        end
    end

  --  if robot.id == "epuck0" then
    --    epuck0()
  --  elseif robot.id == "epuck1" then
   --     epuck1()
  --  end
    
    timer = timer + 1

end

-- Funciones para cada robot epuck

function epuck0 ()
    triangulo()
end

function epuck1 ()
    cuadro()
end




--    Funciones de figuras ----------------------------------------------------------------

function triangulo()

    if b == 0 then
        b = move(0, 0) -- epuck0 se mueve 100 cm y gira a 60 grados   
        if b == 1 then
		      log("SEgmento  1 Finalizado")
            c1 = 0;
        end

    end
    
    if b == 1 then
        b = move(50, 120) -- epuck0 se mueve 100 cm y gira a 60 grados
        if b == 2 then
		      log("SEgmento 2 Finalizado")
            c1 = 0
        end
    end

    if b == 2 then
        b = move(50, -120) -- epuck0 se mueve 100 cm y gira a 60 grados
        if b == 3 then
		      log("SEgmento  3 Finalizado")
            c1 = 0
        end
    end

    if b == 3 then
        b = move(50, 0) -- epuck0 se mueve 100 cm y gira a 60 grados
        if b == 4 then
            log("SEgmento  4 Finalizado")
            c1 = 0
            b = b + 1
        end
    end
end



function cuadro()

    if b == 0 then
        b = move(0, 0) -- epuck0 se mueve 100 cm y gira a 60 grados   
        if b == 1 then
		      log("SEgmento  1 Finalizado")
            c1 = 0;
        end

    end
    
    if b == 1 then
        b = move(50, 90) -- epuck0 se mueve 100 cm y gira a 60 grados
        if b == 2 then
		      log("SEgmento 2 Finalizado")
            c1 = 0
        end
    end

    if b == 2 then
        b = move(50, 180) -- epuck0 se mueve 100 cm y gira a 60 grados
        if b == 3 then
		      log("SEgmento  3 Finalizado")
            c1 = 0
        end
    end

    if b == 3 then
        b = move(50, -90) -- epuck0 se mueve 100 cm y gira a 60 grados
        if b == 4 then
            log("SEgmento  4 Finalizado")
            c1 = 0
        end
    end

    if b == 4 then
        b = move(50, 0) -- epuck0 se mueve 100 cm y gira a 60 grados
        if b == 5 then
            log("SEgmento  5 Finalizado")
            c1 = 0
            b = b + 1
        end
    end
end






-- --------------------------------------------------------------------------------------------------
-- Función para calcular el ángulo a partir del vector de orientación
function vec2_angle(v)
    -- Convertir el cuaternión a un ángulo en el plano XY
    ang = math.atan2(2 * (v.w * v.z + v.x * v.y),
                             1 - 2 * (v.y^2 + v.z^2))
    -- Convertir el ángulo de radianes a grados
    ang = ang * 180 / math.pi
    
    return ang
end



-- --------------------------------------------------------------------------------------------------
-- Función para determiaar si un epuck será observador o demostrador
function determinar_obs_dem()

    -- Selección aleatoria del demostrador
    random = robot.random.uniform_int(0,100)
    log("valor random = " .. random)
    if random <	 50 then
        is_demonstrator = true
        log("The " ..robot.id.. " is demostrador")
    else
        is_demonstrator = false
        log("The " ..robot.id.. " is Observador")
    end
    
    -- Encender LED rojo si es demostrador
    if is_demonstrator then
        robot.leds.set_all_colors("red")
        led_on = true
        led_timer = 10 -- 5 segundos a 10 ticks por segundo
        log("Led rojo encendido por " ..robot.id)
    end    
   

    return is_demonstrator
end

-- --------------------------------------------------------------------------------------------------
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



    if c1 == 0 then
        if angle < target_angle - 3 then
            robot.wheels.set_velocity(-1, 1)
				--log("corrijiendo angulo 1" .. angle)
        elseif angle > target_angle + 3 then
            robot.wheels.set_velocity(1, -1)
				--log("corrijiendo angulo 2" .. angle)
        else 
            robot.wheels.set_velocity(0, 0)
            c1 = 1 
		    
			  --log("Angulo correjido" )
  
   		   -- Calcular el tiempo necesario para recorrer la distancia
   		   time_needed = distance / speed
  		      -- Calcular el número de ticks basados en el timestep de 0.1 segundos
  		      ticks_needed = math.floor(time_needed / 0.1) 

            ticks_needed = ticks_needed + t
        end
    end

    if c1 == 1 then
    -- Mover el robot durante el número de ticks calculado

        if t < ticks_needed then
            robot.wheels.set_velocity(10, 10)
				--log("Segmento c1 = " .. c1)
        else
            -- Detener el robot al finalizar
            robot.wheels.set_velocity(0, 0)
				log("posicion: x=" .. posicion[1] .. ", y=" .. posicion[2])
    			-- Imprimir el ángulo
 			   log("Angle: " .. angle)
				c1 = 2
            c2 = c2 +1
        end
    end 

	 return c2
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
