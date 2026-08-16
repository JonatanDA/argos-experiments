--=============================================
--   MEJORAS :  -- Segir al maestro para no perderlo de vista si se aleja, o alejarse si se acerca para evitar coliciones
                -- Función de RW, evitar quedar en medio de dos objetos
                -- Fucnión de almacenado de datos.
                -- Calibración correcta de parámetros de simulación.
                -- Evitar continuar con el patrón si es interferido por un obstaculo (Evita propagar patrones incorrectos)
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

-- 187014    semilla 1er exp. FOOT

------------------------------
--Parámetro de simulación
Num_sensores_de_proximidad = 24 -- Numero de sensores del robot
angulo_inicial = -90 -- Angulo de inicio de cada patrón

angulo_treshold = 10            -- Sensibilidad del observador a caambios en los datos de angulo obtenidos del maestro, ajuste recomendado: 10 < sensitivity_2 < 50
demostrador_threshold = 41      -- Distancia máxima a la que a la que el maestro espera ver al observador
threshold = 40                  -- Distancia a la que se ubica el observador del maestro
avance = 1                      -- Influye en la sensibilidad de deteccion de segmentos, valor minimo = 1
sensitivity_2 = 1               -- Sensibilidad del observador a caambios en los datos de distancia obtenidos del maestro, ajuste recomendado: 1 < sensitivity_2 < 2

speed_dem = 5                   -- Velocidad del demostrador al ejecutar el patrón
d_speed = 15                    -- velocidad normal
g_speed = 5                    -- Velocidad de giro

max_tracking_distance = 70   -- Máxima distancia a la que se puede alejar el maestro antes de que el alumno se acerque para no perderlo de vista. cm
min_tracking_distance = 20      -- Miínima distancia a la que se puede acercar el maestro antes de que el alumno se aleje para no colisionar. cm
porcentaje = 0.6               -- 1 < porcentaje < 0.33 , indica que tanto el observador busca acercarse al maestro cuando este cruza el max_tracking_distance.
------------------------------
--Parametros de patrónes predefinidos
perm1c = 30
perm2c = 20

permt = 30

l1 = 80
l2 = 160

six1 = 60 -- lado corto 
six2= 80

-----------------------------
--Otros parámetros

math.randomseed(os.time())
sensitivity = ( ( (d_speed-speed_dem)/10 ) * avance ) / 3 -- speed_dem/10 indica cuanto avanza el robot demostrador en un tick, 
                                                            -- *avance indica con que valor se va a comparar, en este caso el minimo para maxima sensibilidad

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

    ]]
    if sensitivity >= ((speed_dem/10 ) * avance ) / 3 then sensitivity = ((speed_dem/10 ) * avance ) / 3 end

    robot.colored_blob_omnidirectional_camera.enable()
    determinar_obs_dem()
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
        --log("Buscando demostrador")
    end
    timer = timer + 1
end

-------------------------------------------------------------------------------------------------------------------------------------
-- Funciones para cada rol
-------------------------------------------------------------------------------------------------------------------------------------

d1 = 0
c5 = 0
demostrador_avg_angle = 0
demostrador_avg_dist = 0
function demostrador()
    if d1 == 0 then
        detectar_observador()

    elseif d1 == 1 then   
        espera_antes_de_ejecutar_patron()

    elseif d1 == 2 then
        if c5 == 1 then
            patron_imitado(segments)
            --log("Patron 1 (imitado) ejecutandose")
        else
            six(demostrador_avg_angle)
            --log("Patron 2( triangulo) ejecutandose")
        end
    elseif d1 == 3 then
        Maestro_triste()
    end
end

c3 = 0
c4 = 0
function observador()
    if c3 == 0 then
        detect_and_stop()

    elseif c3 == 1 then
        move_until_obstacle()

    elseif c3 == 2 then
        if c4 == 0 then
            store_relative_positions()
            --log(c4)
        elseif c4 == 1 then
            polarACartesiano(distances, angles, motion_markers)
            -- log(c4)
        elseif c4 == 2 then
            detect_seg(coordenadasx, coordenadasy)
            -- log(c4)
        elseif c4 == 3 then
            apply(segmentos)
            -- log(c4)
        elseif c4 == 4 then
            calculate_segments(distancias_a_recorrer, angles_a_recorrer)
            -- log(c4)
        elseif c4 == 5 then
            patron_imitado(segments)
        end
        --log("analizando movimiento por ".. robot.id)
    end
end

-------------------------------------------------------------------------------------------------------------------------------------
-- Other functions
------------------------------------------------------------------------------------------------------------------------------------

-- Función para determiaar si un epuck será observador o demostrador
is_demonstrator = 0
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
        setleds("red")
        log("The " .. robot.id .. " is demostrador = " .. randoms)
        --log("Led rojo encendido por " ..robot.id)
    else
        --robot.leds.set_all_colors("blue")
        log("The " .. robot.id .. " is Observador= " .. randoms)
    end


    return is_demonstrator
end

function resetOBS()
    motion_markers= {}           -- Correcciones registradas
    distances     = {} -- Lista de posiciones relativas (x, y)
    angles        = {}    -- Lista de ángulos relativos

    coordenadasx  = {}
    coordenadasy  = {}


    segmentos     = {}
    equations     = {}
    intersections = {}
    --segments = {}
    

    indice        = 1

    c1            = 0
    c2            = 0

    c3            = 0
    c4            = 0
    b             = 0

    robot.leds.set_all_colors("black")
    r_espere()
    r_move()
end

function resetDEM() 
    c1 = 0                
    c2 = 0

    ind = 1
    b = 0

    d1 = 0

    demostrador_avg_angle = 0

    indice = 1

    setleds("red")
    r_espere()
    r_move()

end

function r_move() 
    c1 = 0
    c2 = 0
    angle_ticks_needed = 0
    move_distance = 0
    move_angle = 0
end

--Funcion para hacer una espera
temp = true
cr = 0
crono = 0
function espere(c)
    --log("cr = "..cr)
    --log("c = "..c)

    crono = c - cr

    --log("crono = "..crono)
    cr = cr + 1

    --log(" cronómetro: "..cr.." "..robot.id)

    if crono <= 0 then
       -- log(" pip---pip---pip ".." "..robot.id)
        --robot.leds.set_all_colors("black")
        temp = false
        cr = 0          

    end
end
--resetear espera
function r_espere()
    temp = true
    cr = 0
    crono = 0
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

-- Función para setear determinados leds con color personalizado
function setleds(color)
    for i = 1, 13 do
        if i <= 12 and i >= 1 then
            -- Apagar LED
            robot.leds.set_single_color(i, "black")
        else
            -- Encender LED con el color recibido como parámetro
            robot.leds.set_single_color(i, color)
        end
    end
end--187014


function imprimir_posicion()
    local posicion = {}
    posicion[1] = robot.positioning.position.x
    posicion[2] = robot.positioning.position.y
    log("Posición en x: "..posicion[1]..", Posición en y: "..posicion[2].." ..."..robot.id)
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

function detectar_observador()
    local blobs = robot.colored_blob_omnidirectional_camera
    local angle = 0
    RandomWalking()
    demostrador_avg_angle = 0

    if RW_b1 == 0 then
        for i, blob in ipairs(blobs) do
            local r, g, b = blob.color.red, blob.color.green, blob.color.blue

            if r < 100 and g < 100 and b > 200 then  -- Si detecta un objeto azul
                d1 = 1
                angle = blob.angle * (180 / math.pi) -- Obtener el ángulo del blob en Grados
                demostrador_avg_angle = angle
                
                log("Alumno detectado! a: "..demostrador_avg_angle.."°, " .. robot.id)
                robot.wheels.set_velocity(0, 0)
                for i = 1, 13 do
                    if i <= 12 and i >= 1 then
                        robot.leds.set_single_color(i, "black")
                    else
                        robot.leds.set_single_color(i, "yellow")
                    end
                end
                break
            end
        end
    end
end

demostrador_avg_dist_1 = 0
EAEP_c1 = 0
function espera_antes_de_ejecutar_patron()

    local blobs = robot.colored_blob_omnidirectional_camera
    local total_distance = 0
    local count = 0 

    --log("demostrador_avg_angle: "..demostrador_avg_angle.." "..robot.id)     
    
    --if demostrador_avg_angle > 10 then
        move(0, demostrador_avg_angle)
    --else
        --c1 = 2
    --end
    espere(50)

    demostrador_avg_dist = 0
            -- Recorrer todos los blobs y sumar sus distancias
    for i, blob in ipairs(blobs) do
        local r,g,b = blob.color.red, blob.color.green, blob.color.blue
        local angle = blob.angle* 180/math.pi
        -- Filtrar solo blobs azules
        if r < 100 and g < 100 and b > 200  then 
            if math.abs(angle) < angulo_treshold then
                total_distance = total_distance + blob.distance
                count = count + 1
                EAEP_c1 = 1
                -- Calcular distancia promedio
                demostrador_avg_dist = total_distance / count
                demostrador_avg_dist_1 = demostrador_avg_dist
                -- log("Alumno "..i.." a " .. angle.."°, "..robot.id)
                -- log(string.format("Distancia promedio del alumno: %.4f", demostrador_avg_dist_1))           
            end
        end              
        --log(string.format("Blob %d: Distancia = %.4f", i, blob.distance))
    end


    --log(demostrador_threshold.." .. "..demostrador_avg_dist_1)
    -- Si la distancia promedio es menor al umbral, detener el robot
    if c1 == 2 and temp == false then
        if demostrador_avg_dist_1 > demostrador_threshold*0.2 and demostrador_avg_dist_1 < demostrador_threshold then
            c1 = 0
            c2 = 0
            d1 = 2
            log("Inicio de demostración" .. robot.id)
            log("Alumno a " .. demostrador_avg_dist_1.." cm".." , Máximo permitido: "..demostrador_threshold..", Minimo permitido: "..0.2*demostrador_threshold)
        else
            log("Alumno no me esta prestando atención :)" .. robot.id)
            log("Alumno a " .. demostrador_avg_dist_1.." cm".." , Máximo permitido: "..demostrador_threshold..", Minimo permitido: "..0.2*demostrador_threshold)
            resetDEM()
            d1 = 3
        end    
    end
    
    if temp== false then
        r_espere()
    end
    
end

function Maestro_triste()
    RandomWalking()
    espere(100)
    if temp== false then
        r_espere()
        d1 = 0
    end
end

-------------------------------------------------------------------------------------------------------------------------------------
-- función para realizar el patron imitado
-------------------------------------------------------------------------------------------------------------------------------------

k = 0
indice = 1
triangulo_ang = 0
c5 = 0
sh_seg = 0
PI_c1 = 0
function patron_imitado(segments_P)

    local segments = segments_P
    local prox = proximidad()
    --log("prox[1]: "..prox[1]..", prox[2]: "..prox[2]..", "..robot.id)

    if prox[1] < 0.9 then
        PI_c1 = 1
    elseif prox[1] >= 0.9 and  prox[2] >= -60 and  prox[2] <= 60 then 
        PI_c1 = 0
        k = 1
        setleds("black")
        log("patrón imitado: NO finalizado debido a obstauclos: " .. robot.id)
    else 
        PI_c1 = 1
    end

    if PI_c1 == 1 and k == 0 then

        if #segments > 1 then

            if indice == 1 and segments[indice][2] == 0 then
                triangulo_ang = angulo_inicial
            else 
                triangulo_ang = 0
            end

            if sh_seg == 0 then
                if c5 == 0 then
                    imprimir_posicion()
                end
                sh_seg = 1
                --log(string.format("C1 = %.2f, c2 = %.2f, angle_ticks_needed = %.2f, move_distance = %.2f, move_angle = %.2f",c1,c2,angle_ticks_needed,move_distance, move_angle))
            end
        
            if b <= #segments then
                local valor1, valor2 = segments[indice][1], segments[indice][2] + triangulo_ang
                --log("Distancia a recorrer en el segmento "..indice.." = ".. segments[indice][1]..", Distancia actual: "..move_distance..", "..robot.id)
                --log("Angulo del segmento = "..indice.." = ".. segments[indice][2]..", Angulo actual: "..move_angle*180/math.pi ..", "..robot.id)
                b = move(valor1, valor2)
        
                if c1 == 2 then
                    if indice == #segments then
                        if c5 == 0 then
                            imprimir_posicion()
                            log(" ========= Patrón aprendido: finalizado por: ".. robot.id.." ========= ")
                        end
                        --log(" Ultimo Segmento: "..indice.." finalizado por: "..robot.id)
                        --log("Segmentos finalizados, "..robot.id)
                                    
                        b = b + 1
                        if c5 == 1 then
                            k = k + 1
                            c1 = 0
                            c2 = 0
                            sh_seg = 0
                            setleds("green")
                            log("Patrón imitado: finalizado por "..robot.id)
                            --log("Encendiendo led en verde -----------"..robot.id)
                        else
                            resetOBS()
                            c5 = 1
                            is_demonstrator = 1
                            sh_seg = 0
                            log("Ahora soy maestro!... " .. robot.id)
                            --robot.leds.set_all_colors("red")
                            setleds("red")
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
        else
            resetOBS()
            robot.leds.set_all_colors("black")
            log("Patrón no detectado :( " .. robot.id)
        end
    end

    -- elseif prox[1] >= 0.9 and k == 0 then
    --     k = 1
    --     setleds("black")
    --     log("patrón imitado: NO finalizado debido a obstauclos: " .. robot.id)
    -- end

    if k == 1 then
        --move(-100, 0)
        RandomWalking()
        espere(20)
        if temp == false then
            resetDEM()
            r_espere()
            setleds("red")
            k = 0
            sh_seg = 0
        end
    end
end


-------------------------------------------------------------------------------------------------------------------------------------
-------------------------------------------------------------------------------------------------------------------------------------
-- Funcines de movimiento de los epuck
-------------------------------------------------------------------------------------------------------------------------------------
-------------------------------------------------------------------------------------------------------------------------------------

--=======================
--Footbot tiene 24 sensores de proximidad
--=======================

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
            move(0,180,30,30)   
        elseif RW_Counter < 2 and RW_b2 == 2 then  
            move(0,0,30,30)   
        else
            move(0,90,30,40)
        end
        if c1 == 2 then
           -- log("----------Colisión eviatada! ---------- ... "..robot.id)
            RW_b2 = 0
            RW_b1 = 0
            RW_Counter = 0
            c1 = 0
            c2 = 0
        end      
    end
end

-- Función para mover el e-puck una distancia y rotar a un ángulo específico

c1 = 0
c2 = 0
move_controller = 1
move_distance = 0 

ticks_needed = 0
actual_dis_ticks = 0

angle_ticks_needed = 0
actual_ang_ticks = 0

time_needed = 0
speed = d_speed

move_angle = 0
move_distance = 0

-- Parámetro físico del robot

function move(distance, target_angle, speed_d, speed_g)

    speed_d = speed_d or d_speed
    d_speed = speed_d

    speed_g = speed_g or g_speed
    g_speed = speed_g

    local L = 14 --   static const Real FOOTBOT_INTERWHEEL_DISTANCE      = 0.14f;
    local t = timer

    if is_demonstrator == 1 then 
        if k==0 then
            speed = speed_dem
        else 
            speed = d_speed
        end
    end

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


    -- ===============================
    -- Fase 1: Giro 
    -- ===============================
    if c1 == 0 then
        if angle_ticks_needed == 0 then
            local theta = math.rad(target_angle)  -- convertir a radianes
            local t_rotation = math.abs(theta) * L / (2 * g_speed)
            angle_ticks_needed = math.floor(t_rotation / 0.1)
            angle_ticks_needed = angle_ticks_needed + t

            move_distance = 0; --restart distance

            if robot.id == "epuck4" then 
               -- log("Rotación de "..target_angle.."° → "..angle_ticks_needed.." ticks, "..robot.id)
            end
        end

        if t <=  angle_ticks_needed then

            if target_angle > 0 then
                robot.wheels.set_velocity(-g_speed, g_speed)
                move_angle = (0.1*actual_ang_ticks*2*g_speed/L)
                -- if robot.id == "epuck0" then 
                --    -- log("t =  "..t.." , angle_ticks_needed"..angle_ticks_needed.." - ".. robot.id)
                --    log(" theta = "..move_angle)
                -- end
            else
                robot.wheels.set_velocity(g_speed, -g_speed)
                move_angle = (-0.1*actual_ang_ticks*2*g_speed/L)
                -- if robot.id == "epuck0" then 
                --    -- log("t =  "..t.." , angle_ticks_needed"..angle_ticks_needed.." - ".. robot.id)
                --    log(" theta = "..move_angle)
                -- end
            end
            actual_ang_ticks = actual_ang_ticks + 1
        else 
            --robot.wheels.set_velocity(0, 0)
            c1 = 1
            time_needed = math.abs(distance) / speed
            ticks_needed = math.floor(time_needed / 0.1)
            ticks_needed = ticks_needed + t 
            if target_angle > 0 then 
                move_angle = (0.1*actual_ang_ticks*2*g_speed/L)
                -- if robot.id == "epuck0" then 
                --     log("Angulo corregido ".. move_angle.." , "..robot.id)
                -- end
            else
                move_angle = (-0.1*actual_ang_ticks*2*g_speed/L)
                -- if robot.id == "epuck0" then 
                --     log("Angulo corregido ".. move_angle.." , "..robot.id)
                -- end
            end
        end
    end

    -- ===============================
    -- Fase 2: Movimiento lineal
    -- ===============================
    if c1 == 1 then    
        if t <= ticks_needed then
            move_distance = 0.1*speed*actual_dis_ticks*move_controller
            robot.wheels.set_velocity(speed * move_controller, speed * move_controller)
            actual_dis_ticks = actual_dis_ticks + 1
        else
            move_distance = 0.1*speed*actual_dis_ticks*move_controller
            robot.wheels.set_velocity(0, 0)
            c1 = 2
            c2 = c2 + 1    

            ticks_needed = 0
            actual_dis_ticks = 0
            angle_ticks_needed = 0
            actual_ang_ticks = 0

            if robot.id == "epuck0" then 
                --log("Distancia: "..distance.." y angulo: "..target_angle.." recorridos............... "..robot.id)
            end
        end
    end

    if c1 == 2 then    
        ticks_needed = 0
        actual_dis_ticks = 0
        angle_ticks_needed = 0
        actual_ang_ticks = 0
    end

    return c2,c1,move_distance,move_angle
end

-------------------------------------------------------------------------------------------------------------------------------------
-- Funciones de figuras
-------------------------------------------------------------------------------------------------------------------------------------
---
ind = 1
b = 0
triangulo_ang = 0
function triangulo(demostrador_avg_angle)

    local prox = proximidad()
    --log("prox[1]: "..prox[1]..", prox[2]: "..prox[2]..", "..robot.id)

    if prox[1] < 0.9 and  k == 0 then

        if sh_seg == 0 then
            imprimir_posicion()
            sh_seg = 1
        end
        
        if ind == 1 then
            triangulo_ang = 0
        end
        local params = {
            --{0,-60} ,
            { permt, -30 + triangulo_ang },
            { permt, -120 + triangulo_ang },
            { permt, -120 + triangulo_ang }
        }

        if b <= #params then
            local s1 = params[ind][1]
            local s2 = params[ind][2]
            b = move(s1, s2)

            if c1 == 2 then
                if b == #params then
                    setleds("green")
                    imprimir_posicion()
                    log("Patrón original: triangulo, finalizado por "..robot.id)
                    k = 1
                    b = b + 1
                    c1 = 0;
                else
                    ind = ind + 1
                    --imprimir_posicion()
                    c1 = 0
                    sh_seg = 0
                end
            end
        end

    elseif prox[1] >= 0.9 and k == 0 then
        k = 1
        setleds("black")
        log("patrón original: six NO finalizado debido a obstauclos: " .. robot.id)
    end

    if k == 1 then
        RandomWalking()
        espere(50)
        if temp == false then
            resetDEM()
            r_espere()
            setleds("red")
            k = 0
            sh_seg = 0
        end
    end
end

function cuadro()

    local prox = proximidad()
    --log("prox[1]: "..prox[1]..", prox[2]: "..prox[2]..", "..robot.id)

    if prox[1] < 0.9 and  k == 0 then

        if sh_seg == 0 then
            imprimir_posicion()
            sh_seg = 1
        end

        if ind == 1 then
            triangulo_ang = 0
        end
        local params = {
            --{0,0} ,
            { perm1c, -90 + triangulo_ang},
            { perm2c, -90 + triangulo_ang},
            { perm1c, -90 + triangulo_ang},
            { perm2c, -90 + triangulo_ang}
        }

        if b <= #params and k == 0 then
            local s1 = params[ind][1]
            local s2 = params[ind][2]

            b = move(s1, s2)

            if c1 == 2 then
                if b == #params then

                    setleds("green")
                    imprimir_posicion()
                    log("patrón original: cuadrado finalizado por: " .. robot.id)
                    k = 1
                    b = b + 1
                    c1 = 0
                elseif b < #params then
                    ind = ind + 1
                    --imprimir_posicion()
                    c1 = 0
                    sh_seg = 0
                end
            end
        end

    elseif prox[1] >= 0.9 and k == 0 then
        k = 1
        setleds("black")
        log("patrón original: six NO finalizado debido a obstauclos: " .. robot.id)
    end

    if k == 1 then
        RandomWalking()
        espere(50)
        if temp == false then
            resetDEM()
            r_espere()
            setleds("red")
            k = 0
            sh_seg = 0
        end
    end
end

function L()

    local prox = proximidad()
    --log("prox[1]: "..prox[1]..", prox[2]: "..prox[2]..", "..robot.id)

    if prox[1] < 0.9 and  k == 0 then

        if sh_seg == 0 then
            imprimir_posicion()
            sh_seg = 1
        end

        if ind == 1 then
            triangulo_ang = 0
        end
        local params = {
            --{0,0} ,
            { l1, -90 + triangulo_ang},
            { l2, -90 + triangulo_ang},
        }

        if b <= #params and k == 0 then
            local s1 = params[ind][1]
            local s2 = params[ind][2]

            b = move(s1, s2)

            if c1 == 2 then
                if b == #params then

                    setleds("green")
                    imprimir_posicion()
                    log("patrón original: L finalizado por: " .. robot.id)
                    k = 1
                    b = b + 1
                    c1 = 0
                elseif b < #params then
                    ind = ind + 1
                    --imprimir_posicion()
                    c1 = 0
                    sh_seg = 0
                end
            end
        end

    elseif prox[1] >= 0.9 and k == 0 then
        k = 1
        setleds("black")
        log("patrón original: six NO finalizado debido a obstauclos: " .. robot.id)
    end

    if k == 1 then
        RandomWalking()
        espere(50)
        if temp == false then
            resetDEM()
            r_espere()
            setleds("red")
            k = 0
            sh_seg = 0
        end
    end
end


function six()

    local prox = proximidad()
    --log("prox[1]: "..prox[1]..", prox[2]: "..prox[2]..", "..robot.id)

    if prox[1] < 0.9 and  k == 0 then

        if sh_seg == 0 then
            imprimir_posicion()
            sh_seg = 1
        end

        if ind == 1 then
            triangulo_ang = 0
        end
        local params = {
            --{0,0} ,
            { six1, -45 + triangulo_ang},
            { six2, -45 + triangulo_ang},
            { six1, -45 + triangulo_ang},
            { six1, -90 + triangulo_ang},
            { six2, -45 + triangulo_ang},
            { six1, -45 + triangulo_ang},
        }

        if b <= #params and k == 0 then
            local s1 = params[ind][1]
            local s2 = params[ind][2]

            b = move(s1, s2)
            --log("move_angle: "..move_angle.."°, move_distance: "..move_distance)
            if c1 == 2 then
                if b == #params then
                    setleds("green")
                    imprimir_posicion()
                    log("patrón original: six finalizado por: " .. robot.id)
                    k = 1
                    b = b + 1
                    c1 = 0
                elseif b < #params then
                    --espere(50)
                    -- if temp == false then
                        ind = ind + 1
                        --imprimir_posicion()
                        c1 = 0
                        sh_seg = 0
                        --r_espere()
                        --log("Segmento "..b..": finalizado por: " .. robot.id)
                    --end
                end
            end
        end

    elseif prox[1] >= 0.9 and k == 0 then
        k = 1
        setleds("black")
        log("patrón original: six NO finalizado debido a obstauclos: " .. robot.id)
    end

    if k == 1 then
        RandomWalking()
        espere(50)
        if temp == false then
            resetDEM()
            r_espere()
            setleds("red")
            k = 0
            sh_seg = 0
        end
    end
end

-------------------------------------------------------------------------------------------------------------------------------------
-------------------------------------------------------------------------------------------------------------------------------------
--Funciones del observador
-------------------------------------------------------------------------------------------------------------------------------------
-------------------------------------------------------------------------------------------------------------------------------------

----- Función de busuqeda ejecutada por el observador
function detect_and_stop()

    RandomWalking()

    if robot.colored_blob_omnidirectional_camera and RW_b1 == 0 then
        local blobs = robot.colored_blob_omnidirectional_camera
        --log("Cantidad de blobs detectados: " .. tostring(#blobs).. ", por " .. robot.id)
        local i = 1

        -- Si no detecta rojo, seguir moviendose
        if c3 == 0 then
            for i, blob in ipairs(blobs) do
                local r, g, b = blob.color.red, blob.color.green, blob.color.blue
                local angle = blob.angle * (180 / math.pi)                        -- Obtener el ángulo del blob en Grados

                if r > 200 and g < 100 and b < 100 and math.abs(angle) <= angulo_treshold then -- Si detecta un objeto rojo
                    -- Imprimir los valores RGB detectados
                    --log(string.format("Blob %d detectado - R: %d, G: %d, B: %d", i, r, g, b))

                    robot.wheels.set_velocity(0, 0)   -- Detenerse
                    robot.leds.set_all_colors("blue") -- Encender LEDs en azul
                    c3 = 1
                    log(" ========= Maestro detectado a " .. angle .."°, por ".. robot.id.." =========")
                    break -- No es necesario revisar más blobs
                end
            end
        else
            --log("Observador detectó demostrador y se detuvo")
            --robot.leds.set_all_colors("green")
        end
    else
        --log("Esquivando obstaculos ")
    end
end

moveuntil_distance_an = 40
moveuntil_angle_an = 0
moveuntil_distance_c1 = 0
function move_until_obstacle()
    -- Obtener los blobs detectados por la cámara
    local blobs = robot.colored_blob_omnidirectional_camera
    local total_distance = 0
    local count = 0

    espere(10)

    -- Recorrer todos los blobs y sumar sus distancias
    for i, blob in ipairs(blobs) do
        local r, g, b = blob.color.red, blob.color.green, blob.color.blue
        local angle = blob.angle*180/math.pi
        --log("Maestro "..i.." a "..blob.distance.."cm y "..angle.."°"..robot.id)

        --Filtrar solo blobs amarillos y solo al maestro de frente si hay más de uno
        if r > 200 and g > 200 and b < 100 and math.abs(angle) <= 3*angulo_treshold then

            distance_new = blob.distance
            if moveuntil_distance_c1 == 0 then                  
                moveuntil_distance_an = distance_new
                moveuntil_distance_c1 = 1
            end
            --Filtrar solo a blobs correspodnientes al maestro
            if distance_new  < moveuntil_distance_an + sensitivity_2*d_speed/10 and distance_new > moveuntil_distance_an - sensitivity_2*d_speed/10 then
                moveuntil_distance_an = distance_new
                total_distance = total_distance + blob.distance
                moveuntil_angle_an = blob.angle * (180 / math.pi)
                count = count + 1
                --log("Dirigiendose hacia el maestro "..i.." , que esta a: "..moveuntil_angle_an.."° "..robot.id)
            else
                --log("Maestro perdido;( o más de un maestro")
            end

        end 
        --log(string.format("Blob %d: Distancia = %.4f", i, blob.distance))
    end

    if count > 0  then

        if temp == false then
            r_espere()
        end
        local avg_distance = total_distance / count
        -- Calcular distancia promedio
        --log(string.format("Distancia promedio hasta el maestro: %.4f", avg_distance))

        -- Si la distancia promedio es menor al umbral, detener el robot
        if avg_distance < threshold then
            robot.wheels.set_velocity(0, 0)    -- Detener el robot
            log("¡Estamos junto al maestro!, Deteniendo el " .. robot.id)
            --robot.leds.set_all_colors("black") -- Encender LEDs en verde
            srt_angle_an = moveuntil_angle_an
            distance_new = 0
            distance_an = threshold
            moveuntil_distance_c1 = 0
            c3 = 2

            --robot.leds.set_all_colors("black")
        else
            -- Si no hay obstáculo, seguir avanzando
            robot.wheels.set_velocity(speed, speed) -- Velocidad fija
        end

    elseif temp == false then
        -- Si no hay blobs detectados, el robot sigue moviéndose
        log("Maestro no disponible :( , continuando movimiento..."..robot.id)
        robot.leds.set_all_colors("black") -- Apagar LEDs 
        resetOBS()
    end
end

--Función para guardar las posiciones relativas observadas
distances = {} -- Lista de posiciones relativas (x, y)
angles = {}    -- Lista de ángulos relativos
srt_distance_an = 80
srt_distance_new = 0
srt_distance_c1 = 0

srt_c2 = 0
srt_c3 = 0
srt_c4 = 0

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
function store_relative_positions()

    --log("Duardando posiciones relativas ..."..robot.id)

    espere(40) -- Necesario en el protocolo de comunicaión maestro-observador, permite al maestro obtener la posición y angulo del observador.
    if temp== false then
        robot.leds.set_all_colors("black") -- apagar LEDs 
        r_espere()
    end

    if robot.colored_blob_omnidirectional_camera then                
        
        local blobs = robot.colored_blob_omnidirectional_camera
        local sum_distance, sum_angle, count = 0, 0, 0

        --========================== Análisiss de blobs obtendidos. --==========================
        for i, blob in ipairs(blobs) do
            local r, g, b = blob.color.red, blob.color.green, blob.color.blue
            local angle = blob.angle * (180 / math.pi)  -- Obtener el ángulo del blob en Grados

            --========================== Detectar Blobs del maestro (amarillos), si hay más de uno cerca ignora al que no está en frente. 
            if r > 200 and g > 200 and b < 100 then 
                if  srt_angle_an < angle+angulo_treshold*2 and srt_angle_an > angle-angulo_treshold*2 then
                
                    --log("============== Detectando blobs del Maestro en angulo corrrecto: "..angle+angulo_treshold*2 .." > " ..angle.."> "..angle-angulo_treshold*2 .."============== ")
                    -- log("BLOB: "..aux)

                    srt_distance_new = blob.distance
                    if srt_distance_c1 == 0 then                  
                        srt_distance_an = srt_distance_new
                        srt_distance_c1 = 1
                    end
                    ---------Filtrar solo a blobs correspodnientes al maestro correcto. (Cuando dos maestros estan en el mismo angulo, se filtra haciendo uso de las distancias)
                    if srt_distance_new  < srt_distance_an + sensitivity_2*(speed_dem+d_speed)/10 and srt_distance_new > srt_distance_an - sensitivity_2*(speed_dem+d_speed)/10 then
                        srt_distance_an = srt_distance_new
                        sum_distance = sum_distance + blob.distance
                        sum_angle = sum_angle + blob.angle
                        srt_angle_an = sum_angle*180/math.pi
                        count = count + 1
                        --log("Detectando blobs del Maestro en distancia corrrecta, distancia:" ..srt_distance_new)
                        

                        --------- CONTROL DINÁMICO   (CUIDADO AL PASAR A EPUCK, PUES AHI ES IMPORTANTE HACER PRIMERO EL AVERAGE)
                                                
                        if srt_distance_an > max_tracking_distance and not motion_active then
                            motion_active = true
                            last_motion_marker = {srt_distance_an*porcentaje, srt_angle_an}
                            r_move()
                            --log("Maestro alejandose!!, distancia:" ..srt_distance_new)
                        elseif srt_distance_an < min_tracking_distance and not motion_active then
                            motion_active = true
                            last_motion_marker = {-threshold*porcentaje, srt_angle_an} 
                            r_move()
                            --log("Maestro muy cerca!!, distancia:" ..srt_distance_new)
                        end

                        --[[
                        --------- CONTROL DINÁMICO   (CUIDADO AL PASAR A EPUCK, PUES AHI ES IMPORTANTE HACER PRIMERO EL AVERAGE)
                                                
                        if srt_distance_an > max_tracking_distance and not motion_active then
                            motion_active = true
                            last_motion_marker = {srt_distance_an*porcentaje, srt_angle_an}
                            if srt_c3 == 2 then
                                srt_c3 = 3
                                --log(" ========= Necesitamos volver a corregir, hacia adelante!! ========= "..robot.id)
                            elseif srt_c3 == 0 then
                                srt_c3 = 1
                                --log(" ========= Vamos a corregir nuestro punto de observación hacia adelante :) ========= "..robot.id)
                            end

                            -- log("Maestro alejandose!!, distancia:" ..srt_distance_new)
                        elseif srt_distance_an < min_tracking_distance and not motion_active then
                            motion_active = true
                            last_motion_marker = {-threshold*porcentaje, srt_angle_an} 
                            if srt_c3 == 2 then
                                srt_c3 = 3
                                --log(" ========= Necesitamos volver a corregir, hacia atras!! ========= "..robot.id)
                            elseif srt_c3 == 0 then
                                srt_c3 = 1
                                --log(" ========= Vamos a corregir nuestro punto de observación hacia atrás :) ========= "..robot.id)
                            end

                            --log("Maestro muy cerca!!, distancia:" ..srt_distance_new)
                        elseif  srt_distance_an < max_tracking_distance  and srt_distance_an > min_tracking_distance then
                            if srt_c3 == 1 then
                                srt_c3 = 2
                                motion_active = false
                                --log(" ========= De nuevo en el rango permitido:) ========= "..robot.id)
                            elseif srt_c3 == 0 then
                                motion_active = false
                            end
                        else 
                            if srt_c3 == 3 then
                                --log(" ========= finalizar anterior correcición, hay que realizar una nueva!! ========= "..robot.id)
                                c1 = 2
                                c2 = 0
                                --srt_c3 = 0
                            end
                            --log("Moviendose para no perder de vista al maestro")
                        end
                        ]]

                        --log(" ======== El blob:"..i.." corresponde al maestro correcto: "..angle+angulo_treshold*2 .." > " ..srt_angle_an.."> "..angle-angulo_treshold*2 ..", "..robot.id)

                    else
                        -- if #blobs > 1 then 
                            --log("El blob:"..i.." NO corresponde al maestro correcto:  "..srt_distance_an + sensitivity_2*(speed_dem+d_speed)/10 .." > " ..srt_distance_new.."> "..srt_distance_an - sensitivity_2*(speed_dem+d_speed)/10 ..", "..robot.id)
                        -- end
                    end
                else
                    -- if #blobs > 1 then 
                        --log("El blob:"..i.." NO corresponde al maestro correcto: "..angle+angulo_treshold*2 .." > " ..srt_angle_an.."> "..angle-angulo_treshold*2 ..", "..robot.id)                        
                    -- end

                end

                -- Detectar blob verdes del maestro correcto que indican fin del patrón a imitar  
            elseif r < 100 and g > 200 and b < 100 and srt_angle_an < angle+angulo_treshold and srt_angle_an > angle-angulo_treshold then
                ---------Filtrar solo a blobs correspodnientes al maestro correcto. (Cuando dos maestros estan en el mismo angulo, se filtra haciendo uso de las distancias)
                   -- if srt_distance_new  < srt_distance_an + sensitivity_2*(speed_dem+d_speed)/10 and srt_distance_new > srt_distance_an - sensitivity_2*(speed_dem+d_speed)/10 then
             
                        if count == 0 then
                            count = -1
                        end
                    --end
            else
                -- if #blobs > 1 then 
                     --log("El blob:"..i.." NO corresponde a nungún maestro: ")
                -- end
            end 
        end    
        

        --log("count "..count..", "..robot.id)
        --========================== Acciones a ejecutar después del análisis--==========================
        if count > 0 then --========================== Si detectamos blobs validos del maestro .
            --log(" Count= "..count)
            local avg_distance = sum_distance / count
            local avg_angle = sum_angle / count

            --log(string.format("Maestro detectado a: Distancia = %.2f, Ángulo = %.2f", avg_distance, avg_angle*180/math.pi))
            -- Convertir a coordenadas cartesianas si es necesario
            --polarACartesianoPAR(avg_distance,avg_angle) 

            --if motion_active or srt_c3 == 2 then  -- Si se esta moviendo para no perder de vista al maestro o evitar interferir con el maestro
            
            if motion_active then  -- Si se esta moviendo para no perder de vista al maestro o evitar interferir con el maestro
                -- log("Observador en movimiento para no perder de vista al maestro.")
                -- log(string.format("Moviendose a nueva posición: %.2f y Nuevo angulo: %.2f",last_motion_marker[1], last_motion_marker[2]))
                move(last_motion_marker[1], last_motion_marker[2],d_speed,10)

                local ang_total_1 = move_angle + srt_ang_mark_ant
                -- if ang_total_1 >= 180 then
                --     ang_total_1 = ang_total_1 -360
                --     log("Angulo total 1 > 180, corregido, "..robot.id)
                -- elseif ang_total_1 <= -180 then
                --     ang_total_1 = ang_total_1+ 360
                --     log("Angulo total 1 < -180, corregido, "..robot.id)
                -- end

                x_new_r, y_new_r = polarACartesianoPAR(move_distance, ang_total_1)
                x_mark, y_mark = x_new_r+x_ant_r, y_new_r+y_ant_r
                local r, theta = cartesianoAPolarPAR(x_mark, y_mark)

                -- log("Posición: x_new_r: "..x_new_r..", y_new_r: "..y_new_r)
                -- log("Posición: x_ant_r: "..x_ant_r..", y_ant_r: "..y_ant_r)
                --log(" ============================================================= ")
                --log("Posición: x_mark: "..x_mark..", y_mark: "..y_mark .." --> ( "..r.." , "..theta *180/math.pi .." )" )
 
                --table.insert(motion_markers, {r, srt_ang_mark_ant + move_angle })
                table.insert(motion_markers, {r, theta })
                
                -- Guardar valores promediados en los vectores
                table.insert(distances, avg_distance)

                local ang_total_0 = avg_angle + srt_ang_mark_ant + move_angle
                -- if ang_total_0 >= 180 then
                --     ang_total_0 = ang_total_0 -360
                --     log("Angulo total 0 > 180, corregido, "..robot.id)
                -- elseif ang_total_0 <= -180 then
                --     ang_total_0 = ang_total_0+ 360
                --     log("Angulo total 0 < -180, corregido, "..robot.id)
                -- end
                table.insert(angles, ang_total_0)
                
                --local x_r, y_r = polarACartesianoPAR(avg_distance, ang_total_0)
                --log("Posición: x_r: "..x_r..", y_r: "..y_r.." --> ( "..avg_distance.." , "..ang_total_0*180/math.pi.." )" )
                --log(" ----------- avg_angle: "..avg_angle *180/math.pi .. ", srt_ang_mark_ant: "..srt_ang_mark_ant  *180/math.pi .. ", move_angle: "..move_angle *180/math.pi )
                
                --log(" ------------------------------------------------------------ ")
                --local r1, theta1 = cartesianoAPolarPAR(x_r + x_mark, y_r + y_mark)
                --log("Posición: x: "..x_r + x_mark..", y: "..y_r + y_mark.." --> ( "..r1.." , "..theta1 *180/math.pi .." )" )
                --log(" ------------------------------------------------------------ ")


                --log("angulo del marker: "..(theta)*180/math.pi )
                --log(string.format("Angulo ACTUAL recorrido: %.2f, Angulo objetivo: %.2f",move_angle*180/math.pi,last_motion_marker[2]))
                --log(string.format("Distancia ACTUAL recorrida: %.2f, Distancia objetivo: %.2f",move_distance,last_motion_marker[1]))

                if c1 == 2 then
                    srt_ang_mark_ant = move_angle + srt_ang_mark_ant
                    -- if srt_ang_mark_ant >= 180 then
                    --     srt_ang_mark_ant = srt_ang_mark_ant -360
                    --     log("Angulo de marca > 180, corregido, "..robot.id)
                    -- elseif srt_ang_mark_ant <= -180 then
                    --     srt_ang_mark_ant = srt_ang_mark_ant+ 360
                    --     log("Angulo de marca < -180, corregido, "..robot.id)
                    -- end
                    x_ant_r, y_ant_r = x_mark, y_mark

                    last_motion_marker[1] = r
                    last_motion_marker[2] = theta
                    -- log("Se ha corregido la posición para observar al maestro, en distancia: "..last_motion_marker[1].."y angulo: "..last_motion_marker[2]*180/math.pi.."°")
                    -- log("Posición marker: x: "..x_ant_r.."°, y: "..y_ant_r)


                    motion_active = false
                    --srt_c3 = 0  
                    --log(" ========= Fin de correccion simple :) ========= ")

                    -- if srt_c3 == 3 then
                    --     motion_active = true
                    --     srt_c3 = 0
                    --     --log(" ========= hay que corregir a un nuevo valor :) ========= ")

                    -- end

                    c1 = 0
                    c2 = 0

                    robot.wheels.set_velocity(0, 0)
                end          
            else
                --log("Observador quieto en: "..last_motion_marker[1].." a "..last_motion_marker[2]*180/math.pi.."°, maestro en rango de vista.")
                table.insert(motion_markers, last_motion_marker) -- Si esta quieto
                -- Guardar valores promediados en los vectores
                table.insert(distances, avg_distance)

                -- log(" >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> ")
                -- log("Posición: x_mark: "..x_ant_r..", y_mark: "..y_ant_r.." --> ( "..last_motion_marker[1].." , "..last_motion_marker[2] *180/math.pi .." )" )

                local ang_total_2 = avg_angle + srt_ang_mark_ant
                -- if ang_total_2 >= 180 then
                --     ang_total_2 = ang_total_2 -360
                --     log("Angulo total 2 > 180, corregido, "..robot.id)
                -- elseif ang_total_2 <= -180 then
                --     ang_total_2 = ang_total_2+ 360
                --     log("Angulo total 2 < -180, corregido, "..robot.id)
                -- end
                table.insert(angles, ang_total_2)
                
                local x_r, y_r = polarACartesianoPAR(avg_distance, ang_total_2)
                -- log("Posición: x_r: "..x_r..", y_r: "..y_r.." --> ( "..avg_distance.." , "..ang_total_2 *180/math.pi .." )" )
                -- log(" -                avg_angle: "..avg_angle  *180/math.pi .. ", srt_ang_mark_ant: "..srt_ang_mark_ant *180/math.pi )

                -- log(" ------------------------------------------------------------ ")
                --local r2, theta2 = cartesianoAPolarPAR(x_r + x_ant_r, y_r + y_ant_r)
                -- log("Posición: x: "..x_r + x_ant_r..", y: "..y_r + y_ant_r.." --> ( "..r2.." , "..theta2 *180/math.pi .." )" )
                -- log(" ------------------------------------------------------------ ")
            end

        elseif count == 0 then --========================== Si se perdió de visrta al maestro.
            resetOBS()
            robot.leds.set_all_colors("black")
            count = 0
            srt_distance_an = 80
            srt_distance_new = 0
            srt_distance_c1 = 0
            motion_active = false
            log("Maestro perdido de vista ... "..robot.id)

        elseif count == -1 then --========================== Si el maestro avisó y se recibió señal de fin de la demostración.
            --log("Maestro informó que terminó el patron ......")
            c4 = 1  --------------------OJO-------------------------- c4 = 1
            robot.leds.set_all_colors("black")
            count = 0
            srt_distance_an = 80
            srt_distance_new = 0
            srt_distance_c1 = 0
            r_move()
            log("El maestro a finalizado el patrón, análizando.... "..robot.id)
            --log(string.format("C1 = %.2f, c2 = %.2f, angle_ticks_needed = %.2f, move_distance = %.2f, move_angle = %.2f",c1,c2,angle_ticks_needed,move_distance, move_angle))

            -- Imprimir los datos almacenados
            -- log("======= Comparación de Distancias Relativas y Correcciones (motion_markers) =======")
            -- for i = 1, #distances do
            --     local dist = distances[i] or 0
            --     local marker_dist = motion_markers[i] and motion_markers[i][1] or 0
            --     log(string.format("Blob %d -> Distancia: %.2f  |  Marker Distancia: %.2f", i, dist, marker_dist))
            -- end
            
            -- log("======= Comparación de Ángulos Relativos y Correcciones (motion_markers) =======")
            -- for i = 1, #angles do
            --     local ang = angles[i] or 0
            --     local marker_ang = motion_markers[i] and motion_markers[i][2] or 0
            --     log(string.format("Blob %d -> Ángulo: %.2f rad  |  Marker Ángulo: %.2f rad", i, ang, marker_ang))
            -- end
            
            return distances, angles, motion_markers
        end
    else
        log("ERROR: Cámara no detectada."..robot.id)
    end
end

-------------------------------------------------------------------------------------------------------------------------------------
-- Funciones de operaciones y calculos
-------------------------------------------------------------------------------------------------------------------------------------

-- Función para convertir de polares a cartesianas con corrección de movimiento
coordenadasx = {}
coordenadasy = {}
coordenadasx_or = {}
coordenadasy_or = {}
coordenadasx_mk = {}
coordenadasy_mk = {}
function polarACartesiano(v1, v2, motion_markers)
    local x, y, x_m, y_m

    -- Validar que haya suficientes datos
    if #v1 ~= #motion_markers then
        log("ERROR: Tamaño de vectores 'distances' y 'motion_markers' no coincide.")
        return
    end

    for i = 1, #v1 do
        local r = v1[i]
        local theta = v2[i]

        -- Desempaquetar el marker correspondiente
        local r_m = motion_markers[i][1] or 0
        local theta_m = motion_markers[i][2] or 0

        -- Coordenadas relativas observadas
        y = r * math.sin(theta)
        x = r * math.cos(theta)

        -- Corrección del movimiento propio
        y_m = r_m * math.sin(theta_m)
        x_m = r_m * math.cos(theta_m)

        -- Coordenadas corregidas
        table.insert(coordenadasx, x + x_m)
        table.insert(coordenadasy, y + y_m)

        table.insert(coordenadasx_or, x )
        table.insert(coordenadasy_or, y )
        table.insert(coordenadasx_mk, x_m)
        table.insert(coordenadasy_mk, y_m)

        --log(string.format("Coordenada %d -> (%.2f, %.2f) | Marker (%.2f, %.2f)", i, coordenadasx[i], coordenadasy[i], r_m, theta_m))
        --log(string.format("angulo theta %d -> %.2f ", i, theta))
    end
    c4 = 2
    return coordenadasx,coordenadasy
end

segmentos = {} -- Tabla para almacenar los segmentos (columnas)
segmentos_or = {} -- Tabla para almacenar los segmentos (columnas)
segmentos_mk = {} -- Tabla para almacenar los segmentos (columnas)
distancias_a_recorrer = {} --Tabla para almacenar las distancias a recorrer
function detect_seg(xx, yy)
    local k = 1       -- Contador de columnas
    local i = 1       -- Índice del vector de posiciones

    segmentos[k] = {} -- Inicializamos la primera columna
    segmentos_or[k]  = {} -- Tabla para almacenar los segmentos (columnas)
    segmentos_mk [k] = {} -- Tabla para almacenar los segmentos (columnas)

    while i <= #xx - avance do
        local x, y = xx[i], yy[i]
        local x_or, y_or = coordenadasx_or[i], coordenadasy_or[i]
        local x_mk, y_mk = coordenadasx_mk[i], coordenadasy_mk[i]
        local a, b = xx[i + avance], yy[i + avance]

        -- Si el siguiente punto (a, b) está dentro del rango, creamos nueva columna
        if a < x - sensitivity or a > x + sensitivity or b < y - sensitivity or b > y + sensitivity then
            -- Agregar el punto actual a la columna actual
            -- log("sensitivity: "..sensitivity)
            -- log(" ---------------------------------------------------------------------------------------------------")
            -- log("Agregar el punto actual: "..i..", a la columna actual: "..k)
            -- log(" x = "..x.." , y = "..y.." /  x_0r = "..x_or.." , y_or = "..y_or.." / x_mk = "..x_mk.." , y_mk = "..y_mk)
            -- log(x + sensitivity.." < a = "..a.." < "..x - sensitivity)
            -- log(y + sensitivity.." < b = "..b.." < "..y - sensitivity)
            -- log(" ---------------------------------------------------------------------------------------------------")
            table.insert(segmentos[k], { x, y })
            table.insert(segmentos_or[k], { x_or, y_or })
            table.insert(segmentos_mk[k], { x_mk, y_mk })
        else
            -- log("sensitivity: "..sensitivity)
            -- log(" =================================================================================================== ")
            -- log("El punto actual: "..i..", indica nueva columna"..k)
            -- log(" x = "..x.." , y = "..y.." /  x_0r = "..x_or.." , y_or = "..y_or.." / x_mk = "..x_mk.." , y_mk = "..y_mk)
            -- log(x + sensitivity.." < a = "..a.." < "..x - sensitivity)
            -- log(y + sensitivity.." < b = "..b.." < "..y - sensitivity)
            -- log(" =================================================================================================== ")
            k = k + 1
            segmentos[k]    = {} -- Nueva columna vacía
            segmentos_or[k] = {} -- Nueva columna vacía
            segmentos_mk[k] = {} -- Nueva columna vacía
            --i = i + avance    -- Avanzar en el índice
        end
        i = i + 1             -- Avanzar en el índice
    end

    --Imprimir la tabla resultante
    -- for col, filas in pairs(segmentos) do
    --     log(string.format("Columna %d:", col))
    --     for fila, coord in ipairs(filas) do
    --         log("=================================================================")
    --         log(string.format("{%.2f, %.2f}", coord[1], coord[2]))

    --         local coord_or = segmentos_or[col][fila]
    --         local coord_mk = segmentos_mk[col][fila]

    --         log(string.format("{%.2f, %.2f}", coord_or[1], coord_or[2]))
    --         log(string.format("{%.2f, %.2f}", coord_mk[1], coord_mk[2]))
    --     end
    -- end
    c4 = 3
    return segmentos -- Retornar la tabla con los segmentos formados
end

equations = {}
angles_a_recorrer = {}
function apply(segmentos)

    -- Obtener claves ordenadas para recorrer consistentemente
    local ordered_keys = {}
    for k in pairs(segmentos) do
        table.insert(ordered_keys, k)
    end
    table.sort(ordered_keys)

    for _, col in ipairs(ordered_keys) do
        local points = segmentos[col]

        if #points > 10 then
            local sum_x, sum_y, sum_xy, sum_x2 = 0, 0, 0, 0
            local n = #points

            for _, point in ipairs(points) do
                local x, y = point[1], point[2]
                sum_x = sum_x + x
                sum_y = sum_y + y
                sum_xy = sum_xy + x * y
                sum_x2 = sum_x2 + x * x
            end

            local denominator = (n * sum_x2 - sum_x * sum_x)
            if denominator ~= 0 then
                -- ==================== Calcular regresión lineal del segmento ====================
                local m = (n * sum_xy - sum_x * sum_y) / denominator
                local b = (sum_y - m * sum_x) / n

                equations[col] = {m, b}


                -- ==================== Calcular la distancia entre el primer y último punto del segmento ====================
                local x1, y1 = points[1][1], points[1][2]
                local x2, y2 = points[#points][1], points[#points][2]
                local distancia = math.sqrt((x2 - x1)^2 + (y2 - y1)^2)
                table.insert(distancias_a_recorrer, distancia)

                -- ==================== Calcular ángulo relativo ====================
                local curr_angle = math.atan2(y2 - y1, x2 - x1) * (180 / math.pi)
                local rel_angle = 0
                if prev_angle then
                    rel_angle = curr_angle - prev_angle
                    if rel_angle > 180 then rel_angle = rel_angle - 360
                    elseif rel_angle < -180 then rel_angle = rel_angle + 360 end
                end
                table.insert(angles_a_recorrer, rel_angle)
                prev_angle = curr_angle



                --log("Robot " .. robot.id .. string.format(",  Columna %d: y = %.2fx + %.2f", col, m, b))
            else
                log("Robot " .. robot.id .. string.format(",  Columna %d: No se pudo calcular la regresión lineal", col))
            end
        end
    end
    c4 = 4
    return distancias_a_recorrer, angles_a_recorrer
end

segments = {}
function calculate_segments(distancias_a_recorrer, angles_a_recorrer)
    for i = 1, math.min(#distancias_a_recorrer, #angles_a_recorrer) do
        local distancia = distancias_a_recorrer[i]
        local angulo    = angles_a_recorrer[i]
        table.insert(segments, { distancia, angulo })
    end
    --Logging para verificar
    log(string.format("Total segmentos construidos: %d ", #segments)..", "..robot.id)
    for idx, seg in ipairs(segments) do
        log(string.format(
            "Segmento %d -> Distancia: %.2f cm, ÁnguloRel: %.2f°",
            idx, seg[1], seg[2]
        ))
    end
    c4 = 5
    return segments
end

-- Función para convertir de polares a cartesianas, pares de valores
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