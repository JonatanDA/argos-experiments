--=============================================
--   MEJORAS :  -- Mejoras en la función move -> cambio en la lógica de giro (speed_g = grados por segundo)
                -- Corección de errores.
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

ticks_per_second = 10
Num_sensores_de_proximidad = 24 -- Numero de sensores del robot
------------------------------
--Parámetro de simulación internos
angulo_inicial = -90            -- Angulo de inicio de cada patrón

angulo_treshold = 10            -- Sensibilidad del observador a caambios en los datos de angulo obtenidos del maestro, ajuste recomendado: 10 < sensitivity_2 < 50
demostrador_threshold = 41      -- Distancia máxima a la que a la que el maestro espera ver al observador
threshold = 40                  -- Distancia a la que se ubica el observador del maestro
avance = 1                      -- Influye en la sensibilidad de deteccion de segmentos, valor minimo = 1
sensitivity_2 = 2               -- Sensibilidad del observador a caambios en los datos de distancia obtenidos del maestro, ajuste recomendado: 1 < sensitivity_2 < 2

speed_dem = 5                   -- Velocidad del demostrador al ejecutar el patrón
d_speed = 15                    -- velocidad normal
g_speed = 50                     -- Velocidad de giro

max_tracking_distance = 65      -- Máxima distancia a la que se puede alejar el maestro antes de que el alumno se acerque para no perderlo de vista. cm
min_tracking_distance = 30      -- Miínima distancia a la que se puede acercar el maestro antes de que el alumno se aleje para no colisionar. cm
porcentaje = 0.5                -- 1 < porcentaje < 0.33 , indica que tanto el observador busca acercarse al maestro cuando este cruza el max_tracking_distance.

sensitivity = ( ( (d_speed-speed_dem)/ticks_per_second ) * avance ) / 4 -- speed_dem/ticks_per_second indica cuanto avanza el robot demostrador en un tick, 
                                                            -- *avance indica con que valor se va a comparar, en este caso el minimo para maxima sensibilidad
------------------------------
--Parametros de patrónes predefinidos
perm1c = 80
perm2c = 80

permt = 30

l1 = 30
l2 = 60

slide1 = 30
slide2 = 60

six1 = 60 -- lado corto 
six2= 80

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
    if robot.id == "Footbot3" then
        robot.leds.set_all_colors("yellow")
    end
    if sensitivity >= ((speed_dem/ticks_per_second ) * avance ) / 3 then sensitivity = ((speed_dem/ticks_per_second ) * avance ) / 3 end

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
        --log("detectar_observador")

    elseif d1 == 1 then   
        espera_antes_de_ejecutar_patron()
        --log("espera_antes_de_ejecutar_patron")

    elseif d1 == 2 then
        if c5 == 1 then
            patron_imitado(params)
            --log("Patron 1 (imitado) ejecutandose")
        else
           -- L(demostrador_avg_angle)
            --log("Patron 2( Cuadro) ejecutandose")
            Patron_Original{
                    params = {
                        { l1,  95 },
                        { l1, -45 },
                        { l2, -45 },
                        { l1,  45 },
                        { l1,  45 },
                        { l1,  90 },
                        { l1,  45 },
                        { l2,  45 },
                        { l1, -45 },
                        { l1, -45 },
                    -- { 3*slide2, 90 },
                    -- { 3*slide2, 120 },
                    -- { 1*slide2, 15 },
                    },
                name = "L"
            }
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
            calcular_intersecciones_con_extremos(equations,segmentos)
            -- log(c4)
        elseif c4 == 5 then
            calcular_distancias_y_angulos(intersections)
            -- log(c4)
        elseif c4 == 6 then
            create_params(distancias_a_recorrer, angles_a_recorrer)
            -- log(c4)
        elseif c4 == 7 then
            --setleds(("black"))
            patron_imitado(params)
        end
        --log("analizando movimiento por ".. robot.id)
    elseif c3 == 3 then
        Alumno_triste()
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

function detectar_observador()
    local blobs             = robot.colored_blob_omnidirectional_camera
    local angle             = 0
    demostrador_avg_angle   = 0
    RandomWalking()

    if RW_b1 == 0 then
        for i, blob in ipairs(blobs) do
            local r, g, b = blob.color.red, blob.color.green, blob.color.blue

            if r < 100 and g < 100 and b > 200 then  -- Si detecta un objeto azul
                d1 = 1
                angle = blob.angle * (180 / math.pi) -- Obtener el ángulo del blob en Grados
                demostrador_avg_angle = angle
                -- if robot.id == "Footbot11" then  
                    -- log("¡¡ ALUMNO DETECTADO !! a: "..demostrador_avg_angle.."°, por " .. robot.id)
                --  end
                robot.wheels.set_velocity(0, 0)
                r_Temporizador_1()
                r_move()
                setleds("yellow")
                break
            end
        end
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
        demostrador_avg_dist = 0 
        local vel_g = 10

        --log("demostrador_avg_angle: "..demostrador_avg_angle.." "..robot.id)     
        
        if math.abs(demostrador_avg_angle) > 10 then
            move(0, demostrador_avg_angle,nill,vel_g)
        else
            c1 = 2
        end
        Temporizador_1(5)

                -- Recorrer todos los blobs y sumar sus distancias
        for i, blob in ipairs(blobs) do
            local r,g,b = blob.color.red, blob.color.green, blob.color.blue
            local angle = blob.angle* 180/math.pi


            -- Filtrar solo blobs azules
            if r < 100 and g < 100 and b > 200  then 
                if EAEP_c1 == 0 then
                    EAEP_ang_ant = angle
                    EAEP_c1 = 1
                end
                -- log(string.format("Distancia: %.4f", blob.distance))  
                -- log(string.format("angulo actual: %.4f, angulo anterior: %.4f", blob.angle* 180/math.pi , EAEP_ang_ant))  
                if angle < EAEP_ang_ant + angulo_treshold/2  and angle > EAEP_ang_ant - angulo_treshold/2 then
                    total_distance = total_distance + blob.distance
                    count = count + 1
                    EAEP_ang_ant = angle
                    -- Calcular distancia promedio
                    demostrador_avg_dist = total_distance / count
                    demostrador_avg_dist_1 = demostrador_avg_dist

                    --if robot.id == "Footbot11" then  
                    --log("Alumno "..i.." a " .. angle.."°, "..robot.id)
                        -- log(string.format("Distancia promedio del alumno: %.4f", demostrador_avg_dist_1))  
                    --end
                else     
                    demostrador_avg_dist_1 = -1     
                end
            end              
            --log(string.format("Blob %d: Distancia = %.4f", i, blob.distance))
        end


        --log("demostrador_threshold: "..demostrador_threshold.." .. ".."demostrador_avg_dist_1: "..demostrador_avg_dist_1)
        -- log("c1 = "..c1)
        -- Si la distancia promedio es menor al umbral, detener el robot
        if temp == false and c1 == 2 then
            if demostrador_avg_dist_1 > demostrador_threshold*0.8 and demostrador_avg_dist_1 < demostrador_threshold then
                c1 = 0
                c2 = 0
                --d1 = 2            
                --log("Frente al observador........... angulo de: "..EAEP_ang_ant)
                EAEP_c2 = 1
                --if robot.id == "Footbot11" then  
                --end
            else
                -- log( " -------------------------------------------------------------------------" )
                -- log("Alumno no me esta prestando atención :( )" .. robot.id)
                -- log("Alumno a " .. demostrador_avg_dist_1.." cm".." , Máximo permitido: "..demostrador_threshold..", Minimo permitido: "..0.8*demostrador_threshold)
                -- log( " -------------------------------------------------------------------------"  )
                resetDEM() 
            end    
        end
        
    else
        move(0,EAEP_ang_ant,10,EAEP_ang_ant)
        if c1 == 2 then 
            d1 = 2
            r_move()
            r_Temporizador_1()
            -- log("Inicio de demostración por " .. robot.id)
            -- log("Alumno a " .. demostrador_avg_dist_1.." cm".." , Máximo permitido: "..demostrador_threshold..", Minimo permitido: "..0.2*demostrador_threshold)
        end
    end
end

ind = 1
PO_end = false 
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

            if c1 == 2 then
                if ind == #params then

                    setleds("green")
                    log( " -------------------------------------------------------------------------" )
 
                    imprimir_posicion(posiciones_guardadas)
                    log(" ========= patrón original: "..name.." finalizado por: " .. robot.id.." ========= ")
                    log( " -------------------------------------------------------------------------" )
                    k = 1
                    ind = ind + 1
                    c1 = 0
                    PO_end = true
                elseif ind < #params then
                    ind = ind + 1
                    c1 = 0
                end
            end
        end

    elseif prox[1] >= 0.9 and k == 0 then
        d1 = 3
        setleds("black")
        log( " -------------------------------------------------------------------------" )
        log("patrón original: "..name.." NO finalizado debido a obstauclos: " .. robot.id)
        log( " -------------------------------------------------------------------------" )
    end

    if k == 1 then
        d1 = 3
    end
end

MT_c1 = 0
function Maestro_triste()
    MT_c1 = MT_c1 + 1
    if MT_c1 == 2 then 
        setleds("black")
    end
    RandomWalking()
    Temporizador_1(15)
    if temp== false then
        resetDEM()
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

    local params = params_P
    local prox = proximidad()
    --log("prox[1]: "..prox[1]..", prox[2]: "..prox[2]..", "..robot.id)

    if prox[1] <= 0.9 then
        PI_c1 = 1
    elseif prox[1] > 0.9 and  prox[2] >= -90 and  prox[2] <= 90 then 
        PI_c1 = 0
        k = 1
        setleds("black")
        -- log( " -------------------------------------------------------------------------" )
        -- log("patrón imitado: NO finalizado debido a obstaculo en el camino!!, proximidad del ".. proximity[1]*100 .."%, "..robot.id)
        -- log( " -------------------------------------------------------------------------" )
    else 
        PI_c1 = 1
    end

    if PI_c1 == 1 and k == 0 then

        if #params > 1 then

            if indice == 1 and params[indice][2] == 0 then
            else 
            end
        
            if indice <= #params then
                local valor1, valor2 = params[indice][1], params[indice][2]
                --log("Distancia a recorrer en el segmento "..indice.." = ".. params[indice][1]..", Distancia actual: "..move_distance..", "..robot.id)
                --log("Angulo del segmento = "..indice.." = ".. params[indice][2]..", Angulo actual: "..move_angle*180/math.pi ..", "..robot.id)
                move(valor1, valor2, speed_dem, 10, false, robot.id)
        
                if c1 == 2 then
                    if indice == #params then
                        indice = indice + 1
                        PI_end = true
                        r_Temporizador_1()

                        --log(" Ultimo Segmento: "..indice.." finalizado por: "..robot.id)
                        --log("Segmentos finalizados, "..robot.id)
                                    
                        if c5 == 1 then
                            k = k + 1
                            c1 = 0
                            c2 = 0
                            setleds("green")
                            -- if robot.id == "Footbot11" then  
                            --     imprimir_posicion(posiciones_guardadas)
                            -- end
                            log(" ========================================================================= ")
                            log(" Patrón Imitado: finalizado por: ".. robot.id)
                            --log("Encendiendo led en verde -----------"..robot.id)
                        else
                            c5 = 1
                            k = 1
                            log(" ========= Patrón aprendido: finalizado por: ".. robot.id.." ========= ")
                            imprimir_posicion(posiciones_guardadas)
                            log(" ========= ¡ Ahora soy maestro ! " .. robot.id.." ========= ")
                        end
                    else
                        -- log("Segmento numero: "..indice.." finalizado")
                        --print_table(params)
                        indice = indice + 1
                        c1 = 0
                    end
                end
            end
        else
            resetOBS()
            robot.leds.set_all_colors("black")
            log( " -------------------------------------------------------------------------" )
            log("Patrón no detectado :( " .. robot.id)
            log( " -------------------------------------------------------------------------" )
        end
    else
    end

    -- elseif prox[1] >= 0.9 and k == 0 then
    --     k = 1
    --     setleds("black")
    --     log("patrón imitado: NO finalizado debido a obstauclos: " .. robot.id)
    -- end

    if k == 1 then
        RandomWalking()
        Temporizador_1(10)
        if temp == false then
            if PI_end then
                is_demonstrator = 1
                resetOBS()
            end
            resetDEM()
            k = 0 --Necesario para que se repita el patron si fallo (Poder verlo en la consola)
        end
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
                    setleds("blue") -- Encender LEDs en azul
                    c3 = 1
                    -- if robot.id == "Footbot3" then  
                    --     log(" ¡¡ MAESTRO DETECTADO !! a ".. blob.distance.. " y " .. angle .."°, por ".. robot.id)  
                    -- end    
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

MUO__distance_an = 40
MUO__angle_an = 0
MUO__c1 = 0
MUO__c2 = 0
MUO__c3 = 0
srt_angle_an = 0
function move_until_obstacle()
    -- Obtener los blobs detectados por la cámara
    local blobs = robot.colored_blob_omnidirectional_camera
    local total_distance = 0
    local count = 0

    Temporizador_1(1)

    -- Recorrer todos los blobs y sumar sus distancias
    for i, blob in ipairs(blobs) do
        local r, g, b = blob.color.red, blob.color.green, blob.color.blue
        local angle = blob.angle*180/math.pi
        --log("Maestro "..i.." a "..blob.distance.."cm y "..angle.."°"..robot.id)

        --Filtrar solo blobs amarillos 
        if r > 200 and g > 200 and b < 100  then
            if math.abs(angle) <= 3*angulo_treshold then --solo al maestro de frente si hay más de uno
                distance_new = blob.distance
                if MUO__c1 == 0 then                  
                    MUO__distance_an = distance_new
                    MUO__c1 = 1
                end
                --Filtrar blobs correspodnientes al maestro en distancia correcta
                local max = MUO__distance_an + sensitivity_2*d_speed / ticks_per_second
                local min = MUO__distance_an - sensitivity_2*d_speed / ticks_per_second
                if distance_new  < max and distance_new > min then
                    MUO__distance_an = distance_new
                    total_distance = total_distance + blob.distance
                    MUO__angle_an = blob.angle * (180 / math.pi)
                    count = count + 1
                else
                    --log("Maestro ["..i.."] en distancia incorrecta: "..max.." > "..distance_new.." > "..min)
                    setleds("black")
                end
            else
                --log("Maestro ["..i.."] en Angulo incorrecto: "..math.abs(angle).." < "..3*angulo_treshold)
                setleds("black")
            end
        else
            --log("BLOBS DE DIFERENTE COLOR !!")
        end 
        --log(string.format("Blob %d: Distancia = %.4f", i, blob.distance))
    end

    if MUO__c3 == 0 then 
        if count == 1  then
            MUO__c2 = 1
            local avg_distance = total_distance/count -- Calcular distancia promedio
            -- if robot.id == "Footbot3" then  
            --     log(string.format("Distancia promedio hasta el maestro: %.4f y angulo de %.4f , ", avg_distance,MUO__angle_an)..robot.id)
            -- end    
            if avg_distance < threshold then        -- Si la distancia promedio es menor al umbral, detener el robot
                MUO__c3 = 1
                srt_angle_an = MUO__angle_an
                robot.wheels.set_velocity(0, 0)     -- Detener el robot
                --     log("Estamos junto al maestro, Deteniendome " .. robot.id)
            else
                -- Si no hay obstáculo, seguir avanzando
                robot.wheels.set_velocity(d_speed, d_speed) -- Velocidad fija
            end
        elseif count > 1 then                           -- Si se ha detectado más de un maestro, el robot vuelve a moverse en RandomWalking
            --log("Confusión de maestros :( "..robot.id)
            r_Temporizador_1()
            c3 = 3
        elseif temp == false then                       -- Si no se ha establecido conexión con el maestro durante algún tiempo, el robot vuelve a moverse en RandomWalking
            --log("Maestro no disponible :( "..robot.id)
            r_Temporizador_1()
            c3 = 3
        elseif MUO__c2 == 1 and count == 0 then         -- Si se ha perdido de vista al maestro, el robot vuelve a moverse en RandomWalking
            --log("Maestro perdido de vista :( "..robot.id)
            r_Temporizador_1()
            c3 = 3
        end
    else 
        move(0,srt_angle_an,nill,srt_angle_an)
        if c1 == 2 then
            srt_angle_an = 0
            MUO__angle_an = 0
            distance_new = 0
            distance_an = threshold
            MUO__c1 = 0
            MUO__c2 = 0
            MUO__c3 = 0
            r_move()
            r_Temporizador_1()
            c3 = 2
        end
    end                  
end


--Función para guardar las posiciones relativas observadas
distances = {} -- Lista de posiciones relativas (x, y)
angles = {}    -- Lista de ángulos relativos
srt_distance_an = 80
srt_distance_new = 0

SRT_c1 = 0
srt_c2 = 0
srt_c3 = 0
srt_c4 = 0
srt_c5 = 0 

srt_ang_mark_ant = 0
x_mark, y_mark, x_new_r, x_ant_r, y_new_r, y_ant_r = 0, 0, 0, 0, 0 ,0 

srt_angle_an = 0
srt_end = 0

motion_markers = {}           -- Correcciones registradas
last_motion_marker = {0, 0}   -- Total correccion realizada
motion_tick = 0               -- Tick actual durante movimiento
motion_active = false         -- Indica si hay un movimiento en curso
motion_target_ticks = 0       -- Total de ticks necesarios
motion_cm_tick = 0   
motion_dist = 0   
function store_relative_positions()

    --log("Duardando posiciones relativas ..."..robot.id)
    local proximity = proximidad()
    if proximity[1] < 1 then 
        srt_c5 = 0
    else
        if srt_c5 == 5 * ticks_per_second then
            resetOBS()
            return
        end
        srt_c5 = srt_c5 + 1
    end

    Temporizador_1(15) -- Necesario en el protocolo de comunicaión demostrador-observador, permite al demostrador obtener la posición y angulo del observador.
      
    if temp == false then
        robot.leds.set_all_colors("black") -- apagar LEDs 
        --log("APAGANDO LEDS")
        r_Temporizador_1()
    end

    if robot.colored_blob_omnidirectional_camera then                
        
        local blobs = robot.colored_blob_omnidirectional_camera
        local sum_distance, sum_angle, count = 0, 0, 0

        --========================== Análisiss de blobs obtendidos. --==========================
        for i, blob in ipairs(blobs) do
            local r, g, b = blob.color.red, blob.color.green, blob.color.blue
            local angle = blob.angle * (180 / math.pi)  -- Obtener el ángulo del blob en Grados
            -- log("angle: "..angle)
            -- log(" r = "..r.." g = "..g.." b = "..b)
            --========================== Detectar Blobs del demostrador (amarillos), si hay más de uno cerca ignora al que no está en frente. 
            local max_ang = angle+angulo_treshold*2
            local min_ang = angle-angulo_treshold*2        
            if r > 200 and g > 200 and b < 100 then 
                if  srt_angle_an < max_ang and srt_angle_an > min_ang then
                
                    --log("============== Detectando blobs del demostrador en angulo corrrecto: "..angle+angulo_treshold*2 .." > " ..angle.."> "..angle-angulo_treshold*2 .."============== ")
                    -- log("BLOB: "..aux)

                    srt_distance_new = blob.distance
                    if SRT_c1 == 0 then                  
                        srt_distance_an = srt_distance_new
                        SRT_c1 = 1
                    end
                    ---------Filtrar solo a blobs correspodnientes al demostrador correcto. (Cuando dos demostradors estan en el mismo angulo, se filtra haciendo uso de las distancias)
                    local min = srt_distance_an - sensitivity_2*(speed_dem+d_speed)/ticks_per_second
                    local max = srt_distance_an + sensitivity_2*(speed_dem+d_speed)/ticks_per_second
                    if srt_distance_new  < max and srt_distance_new > min then
                        srt_distance_an = srt_distance_new
                        sum_distance = sum_distance + blob.distance
                        sum_angle = sum_angle + blob.angle
                        srt_angle_an = sum_angle*180/math.pi
                        count = count + 1
                        --log("Detectando blobs del demostrador en distancia corrrecta, distancia:" ..srt_distance_new)
                        

                        --------- CONTROL DINÁMICO   (CUIDADO AL PASAR A FootBot, PUES AHI ES IMPORTANTE HACER PRIMERO EL AVERAGE)
                                                
                        if srt_distance_an > max_tracking_distance and not motion_active then
                            motion_active = true
                            last_motion_marker = {srt_distance_an*porcentaje, srt_angle_an}
                            r_move()
                            --log("demostrador alejandose!!, distancia:" ..srt_distance_new)
                        elseif srt_distance_an < min_tracking_distance and not motion_active then
                            motion_active = true
                            last_motion_marker = {-threshold*porcentaje, srt_angle_an} 
                            r_move()
                            --log("demostrador muy cerca!!, distancia:" ..srt_distance_new)
                        end

                        --[[
                        --------- CONTROL DINÁMICO   (CUIDADO AL PASAR A FootBot, PUES AHI ES IMPORTANTE HACER PRIMERO EL AVERAGE)
                                                
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

                            -- log("demostrador alejandose!!, distancia:" ..srt_distance_new)
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

                            --log("demostrador muy cerca!!, distancia:" ..srt_distance_new)
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
                            --log("Moviendose para no perder de vista al demostrador")
                        end
                        ]]

                        --log(" ======== El blob:"..i.." corresponde al demostrador correcto: "..angle+angulo_treshold*2 .." > " ..srt_angle_an.."> "..angle-angulo_treshold*2 ..", "..robot.id)

                    else
                        -- if #blobs > 1 then 
                            --log("El blob:"..i.." NO corresponde al demostrador correcto: DISTANCIAS = "..max.." > " ..srt_distance_new.."> "..min..", "..robot.id)
                        -- end
                    end
                else
                    -- if #blobs > 1 then 
                        --log("El blob:"..i.." NO corresponde al demostrador correcto: ANGULOS =  "..max_ang .." > " ..srt_angle_an.."> "..min_ang ..", "..robot.id)                        
                    -- end

                end

                -- Detectar blob verdes del demostrador correcto que indican fin del patrón a imitar  
            elseif r < 100 and g > 200 and b < 100 and srt_angle_an < angle+angulo_treshold and srt_angle_an > angle-angulo_treshold then
                ---------Filtrar solo a blobs correspodnientes al demostrador correcto. (Cuando dos demostradors estan en el mismo angulo, se filtra haciendo uso de las distancias)
                -- if srt_distance_new  < srt_distance_an + sensitivity_2*(speed_dem+d_speed)/ticks_per_second and srt_distance_new > srt_distance_an - sensitivity_2*(speed_dem+d_speed)/ticks_per_second then    
                    srt_end = 1
                --end
            else
                -- if #blobs > 1 then 
                --log("El blob:"..i.." NO corresponde a nungún demostrador: ")
                -- end
            end 
        end     

        if count == 0 and srt_end == 1 then
            count   = -1
        else
            srt_end = 0
        end

        --log("count "..count..", "..robot.id)
        --========================== Acciones a ejecutar después del análisis--==========================
        if count > 1 then --========================== Si hubo confusión, mas de 1 demostrador .
            resetOBS()
            count = 0
            srt_distance_an = 80
            srt_distance_new = 0
            SRT_c1 = 0
            motion_active = false
            -- log( " -------------------------------------------------------------------------" )        
            -- log("Confusión de demostradors mientras lo observaba:( "..robot.id)
            -- log( " -------------------------------------------------------------------------" )

        elseif count == 1 then --========================== Si detectamos blobs validos del demostrador .
            --log(" Count= "..count)
            local avg_distance = sum_distance / count
            local avg_angle = sum_angle / count

            --log(string.format("demostrador detectado a: Distancia = %.2f, Ángulo = %.2f", avg_distance, avg_angle*180/math.pi))
            -- Convertir a coordenadas cartesianas si es necesario
            --polarACartesianoPAR(avg_distance,avg_angle) 

            --if motion_active or srt_c3 == 2 then  -- Si se esta moviendo para no perder de vista al demostrador o evitar interferir con el demostrador
            
            if motion_active then  -- Si se esta moviendo para no perder de vista al demostrador o evitar interferir con el demostrador
                -- log("Observador en movimiento para no perder de vista al demostrador.")
                -- log(string.format("Moviendose a nueva posición: %.2f y Nuevo angulo: %.2f",last_motion_marker[1], last_motion_marker[2]))
                move(last_motion_marker[1], last_motion_marker[2],d_speed,80)

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
                    -- log("Se ha corregido la posición para observar al demostrador, en distancia: "..last_motion_marker[1].."y angulo: "..last_motion_marker[2]*180/math.pi.."°")
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
                --log("Observador quieto en: "..last_motion_marker[1].." a "..last_motion_marker[2]*180/math.pi.."°, demostrador en rango de vista.")
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

        elseif count == 0 then --========================== Si se perdió de visrta al demostrador.
            resetOBS()
            -- log( " -------------------------------------------------------------------------" )
            -- log("demostrador perdido de vista mientras lo observaba :( "..robot.id)
            -- log( " -------------------------------------------------------------------------" ) 

        elseif count == -1 then --========================== Si el demostrador avisó y se recibió señal de fin de la demostración.
            --log("demostrador informó que terminó el patron ......")
            c4 = 1  --------------------OJO-------------------------- c4 = 1
            count = 0
            srt_distance_an = 80
            srt_distance_new = 0
            SRT_c1 = 0
            r_move()
            log("El demostrador a finalizado el patrón, análizando.... "..robot.id)
            log(" ========================================================================= ")
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
        log( " -------------------------------------------------------------------------" )
        log("ERROR: Cámara no detectada."..robot.id)
        log( " -------------------------------------------------------------------------" )
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
DS_c1 = 0
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
            DS_c1 = 1
        else
            -- log("sensitivity: "..sensitivity)
            -- log(" =================================================================================================== ")
            -- log("El punto actual: "..i..", indica nueva columna"..k)
            -- log(" x = "..x.." , y = "..y.." /  x_0r = "..x_or.." , y_or = "..y_or.." / x_mk = "..x_mk.." , y_mk = "..y_mk)
            -- log(x + sensitivity.." < a = "..a.." < "..x - sensitivity)
            -- log(y + sensitivity.." < b = "..b.." < "..y - sensitivity)
            -- log(" =================================================================================================== ")
            if DS_c1 == 1 then
                table.insert(segmentos[k], { x, y })
                table.insert(segmentos_or[k], { x_or, y_or })
                table.insert(segmentos_mk[k], { x_mk, y_mk })
                DS_c1 = 0
            end

            k = k + 1
            segmentos[k]    = {} -- Nueva columna vacía
            segmentos_or[k] = {} -- Nueva columna vacía
            segmentos_mk[k] = {} -- Nueva columna vacía
            --i = i + avance    -- Avanzar en el índice
        end
        i = i + 1             -- Avanzar en el índice
    end
    
    -- Agregar el último punto a la última columna
    local x, y = xx[i], yy[i]
    local x_or, y_or = coordenadasx_or[i], coordenadasy_or[i]
    local x_mk, y_mk = coordenadasx_mk[i], coordenadasy_mk[i]
    if i <= #xx then
        table.insert(segmentos[k], { x, y })
        table.insert(segmentos_or[k], { x_or, y_or })
        table.insert(segmentos_mk[k], { x_mk, y_mk })
    end
    --Imprimir la tabla resultante

    -- if robot.id == "Footbot3" then  
    --     for col, filas in pairs(segmentos) do
    --         log(string.format("Columna %d:", col))
    --         for fila, coord in ipairs(filas) do
    --             log("=================================================================")
    --             log(string.format("{%.2f, %.2f}", coord[1], coord[2]))

    --             local coord_or = segmentos_or[col][fila]
    --             local coord_mk = segmentos_mk[col][fila]

    --             log(string.format("{%.2f, %.2f}", coord_or[1], coord_or[2]))
    --             log(string.format("{%.2f, %.2f}", coord_mk[1], coord_mk[2]))
    --         end
    --     end
    -- end
    c4 = 3
    return segmentos -- Retornar la tabla con los segmentos formados
end

equations = {}
distancias_a_recorrer = {} --Tabla para almacenar las distancias a recorrer
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

                -- -- ==================== Calcular ángulo relativo ====================
                -- local curr_angle = math.atan2(y2 - y1, x2 - x1) * (180 / math.pi)
                -- local rel_angle = 0
                -- if prev_angle then
                --     rel_angle = curr_angle - prev_angle
                --     if rel_angle > 180 then rel_angle = rel_angle - 360
                --     elseif rel_angle < -180 then rel_angle = rel_angle + 360 end
                -- end
                -- table.insert(angles_a_recorrer, rel_angle)
                -- prev_angle = curr_angle



                --log("Robot " .. robot.id .. string.format(",  Columna %d: y = %.2fx + %.2f", col, m, b))
            else
                log("Robot " .. robot.id .. string.format(",  Columna %d: No se pudo calcular la regresión lineal", col))
            end
        end
    end
    c4 = 4
    return equations
end

intersections = {} -- Asegúrate de que 'intersections' esté definido en el ámbito correcto
function calcular_intersecciones_con_extremos(equations, segmentos)
    local ordered_keys = {}

    for k in pairs(equations) do
        table.insert(ordered_keys, k)
    end
    table.sort(ordered_keys)

    -- Si no hay ecuaciones, no hay intersecciones que calcular
    if #ordered_keys == 0 then
        resetalumno()
        return 
    end

    local first_key = ordered_keys[1]
    local first_equation_data = equations[first_key]
    local first_segment = segmentos[first_key]

    -- Verificación del primer punto y segmento
    if type(first_equation_data) ~= "table" or type(first_segment) ~= "table" or #first_segment == 0 then
        resetalumno()
        return 
    end

    local m1, b1 = first_equation_data[1], first_equation_data[2]
    local x_inicio = first_segment[1][1]
    local y_inicio = m1 * x_inicio + b1
    table.insert(intersections, {x_inicio, y_inicio})

    -- Calcular intersecciones entre segmentos consecutivos
    for i = 1, #ordered_keys - 1 do
        local k1 = ordered_keys[i]
        local k2 = ordered_keys[i + 1]

        local eq1 = equations[k1]
        local eq2 = equations[k2]

        -- -- Saltar si alguna ecuación consecutiva no es válida
        -- if type(eq1) ~= "table" or type(eq2) ~= "table" then
        --     goto continue_loop -- O 'continue' si tu versión de Lua es 5.2+
        -- end

        local m1, b1 = table.unpack(eq1)
        local m2, b2 = table.unpack(eq2)

        if m1 ~= m2 then
            local x_int = (b2 - b1) / (m1 - m2)
            local y_int = m1 * x_int + b1
            table.insert(intersections, {x_int, y_int})           
        end
    end

    -- Punto final
    local last_key = ordered_keys[#ordered_keys]
    local last_equation_data = equations[last_key]
    local last_segment = segmentos[last_key]

    -- Verificación del último punto y segmento
    -- if type(last_equation_data) ~= "table" or type(last_segment) ~= "table" or #last_segment == 0 then
    --     -- Si no podemos calcular el punto final, retornamos lo que ya tenemos.
    --     -- Esto asume que los puntos anteriores se calcularon correctamente.
    --     return intersections
    -- end

    local m_last, b_last = table.unpack(last_equation_data)
    local x_final = last_segment[#last_segment][1]
    local y_final = m_last * x_final + b_last
    table.insert(intersections, {x_final, y_final}) 
    
    c4 = 5 -- Ajusta el ámbito de 'c4' si es necesario.
    return intersections
end

angles_a_recorrer    = {}
function calcular_distancias_y_angulos(intersecciones)
    -- 1) Primer punto
    local x1, y1 = intersecciones[1][1], intersecciones[1][2]

    -- 2) Orientación inicial DEL OBSERVADO (su eje +X)
    local prev_angle = math.atan2(-y1, -x1) * (180/math.pi)

    -- 3) Ahora recorre todos los segmentos, incluyendo el 1→2
    for i = 1, #intersecciones - 1 do
        local xa, ya = intersecciones[i][1],   intersecciones[i][2]
        local xb, yb = intersecciones[i+1][1], intersecciones[i+1][2]

        -- -- ======== Calcular distancia Euclidiana entre puntos consecutivos ========
        -- local distancia = math.sqrt((x2 - x1)^2 + (y2 - y1)^2)
        -- table.insert(distancias_a_recorrer, distancia)

        -- Ángulo absoluto del segmento i→i+1
        local curr_angle = math.atan2(yb - ya, xb - xa) * (180/math.pi)

        -- Giro relativo desde el eje +X del observado (primer tramo)
        -- o desde el tramo anterior (tramos posteriores)
        local rel_angle = curr_angle - prev_angle

        -- Ajustar a [-180,180]
        if     rel_angle >  180 then rel_angle = rel_angle - 360
        elseif rel_angle < -180 then rel_angle = rel_angle + 360 end

        table.insert(angles_a_recorrer, rel_angle)
        prev_angle = curr_angle
    end
    c4 = 6
    return distancias_a_recorrer, angles_a_recorrer
end

params = {}
function create_params(distancias_a_recorrer, angles_a_recorrer)
    for i = 1, math.min(#distancias_a_recorrer, #angles_a_recorrer) do
        local distancia = distancias_a_recorrer[i]
        local angulo    = angles_a_recorrer[i]
        table.insert(params, { distancia, angulo })
    end
    -- --Logging para verificar
    -- log(string.format("Total segmentos construidos: %d ", #params)..", "..robot.id)
    -- for idx, seg in ipairs(params) do
    --     log(string.format(
    --         "Segmento %d -> Distancia: %.2f cm, ÁnguloRel: %.2f°",
    --         idx, seg[1], seg[2]
    --     ))
    -- end
    c4 = 7
    return params
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

-- Función para mover el e-puck una distancia y rotar a un ángulo específico
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
move_distance       = 0

wheel_speed         = 0
omega_rad           = 0

previous_speed_d    = nil
function move(distance, target_angle, speed_d, speed_g, imprimir,name, stop)

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

            -- Redondear el ángulo personalizado
            target_angle = redondear_personalizado(target_angle)

            -- ===============================
            -- Fase 1: Giro 
            -- ===============================
            if c1 == 0 then
                if angle_ticks_needed == 0 then

                    local omega_deg = math.min(math.abs(speed_g), 80)   -- máximo 8°/s
                    omega_rad = math.rad(omega_deg)                     -- radianes por segundo
                    wheel_speed = omega_rad * (L / 2)                   -- velocidad en m/s

                    local theta = math.rad(target_angle)  -- convertir a radianes
                    local t_rotation = math.abs(theta) / omega_rad
                    
                    angle_ticks_needed = math.floor(t_rotation * ticks_per_second)
                    angle_ticks_needed = angle_ticks_needed + t 

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
                    -- if robot.id == "epuck0" then 
                    --    log("Rotación de "..target_angle.."° → "..angle_ticks_needed.." ticks, "..robot.id)
                    -- end
                end

                if t < angle_ticks_needed then

                    if target_angle > 0 then
                        robot.wheels.set_velocity(-wheel_speed, wheel_speed)
                        move_angle = (1 / ticks_per_second) * actual_ang_ticks * omega_rad
                        -- if imprimir then 
                        --     log(" theta ".. move_angle *180/math.pi .." , "..robot.id)
                        -- end
                    else
                        robot.wheels.set_velocity(wheel_speed, -wheel_speed)
                        move_angle = -(1 / ticks_per_second) * actual_ang_ticks * omega_rad
                        -- if imprimir then 
                        --     log(" theta ".. move_angle *180/math.pi .." , "..robot.id)
                        -- end
                    end

                    actual_ang_ticks = actual_ang_ticks + 1

                else  
                    robot.wheels.set_velocity(0, 0)
                    c1 = 1
                    time_needed = math.abs(distance) / speed_d
                    ticks_needed = math.floor(time_needed * ticks_per_second)
                    ticks_needed = ticks_needed + t 
                    if target_angle > 0 then 
                        move_angle = ( (1/ticks_per_second) * actual_ang_ticks*omega_rad)
                        if imprimir then 
                            --log(" theta ".. move_angle *180/math.pi .." , "..robot.id)
                        end
                    else
                        move_angle = ( -(1/ticks_per_second) * actual_ang_ticks*omega_rad)
                        if imprimir then 
                            --log(" theta".. move_angle *180/math.pi .." , "..robot.id)
                        end
                    end
                end
            end

            -- ===============================
            -- Fase 2: Movimiento lineal
            -- ===============================
            if c1 == 1 then    

            -- Detectar cambio de velocidad
                if previous_speed_d == nil then
                    previous_speed_d = speed_d
                elseif previous_speed_d ~= speed_d then
                    -- Recalcular ticks con la nueva velocidad
                    local distancia_recorrida = move_distance
                    local distancia_restante = math.abs(distance) - math.abs(distancia_recorrida)
                    time_needed = distancia_restante / speed_d
                    ticks_needed = math.floor(time_needed * ticks_per_second) + t
                    previous_speed_d = speed_d
                    if imprimir then 
                        -- log("🔄 Cambio de velocidad detectado en tiempo real, recalculando ticks, "..robot.id)
                        -- log("Nueva velocidad: "..speed_d.." m/s")
                        -- log("Distancia restante: "..distancia_restante.." cm")
                        -- log("Nuevos ticks necesarios: "..ticks_needed)
                    end
                end


                if t < ticks_needed then
                    move_distance = 0.1*speed_d*actual_dis_ticks*move_controller
                    -- if robot.id == "Footbot14" then 
                    --     --log(" ------------------------------------------------------ ")
                    --     log("Distancia recorrida: "..move_distance.." cm, "..robot.id)
                    --     --log("speed : "..speed..", actual_dis_ticks: "..actual_dis_ticks..", move_controller: "..move_controller..", "..robot.id)
                    -- end

                    robot.wheels.set_velocity(speed_d * move_controller, speed_d * move_controller)
                    actual_dis_ticks = actual_dis_ticks + 1
                else
                    move_distance = 0.1*speed_d*actual_dis_ticks*move_controller
                    -- if robot.id == "Footbot14" then 
                    --     log("Distancia recorrida: "..move_distance.." cm, "..robot.id)
                    -- end
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
                ticks_needed = 0
                actual_dis_ticks = 0
                angle_ticks_needed = 0
                actual_ang_ticks = 0
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

    if robot.id == "Footbot4" then
        is_demonstrator = 1
    else
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
    MUO__distance_an = 40
    MUO__angle_an = 0
    MUO__c1 = 0
    MUO__c2 = 0
    MUO__c3 = 0

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

    srt_end             = 0

    -- ============================
    coordenadasx  = {}
    coordenadasy  = {}
    coordenadasx_or = {}
    coordenadasy_or = {}
    coordenadasx_mk = {}
    coordenadasy_mk = {}

    DS_c1 = 0
    segmentos     = {}

    equations     = {}

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
    setleds("red")
    if PO_end then
        setleds("orange")
    end
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

    log(string.format("Total segmentos construidos: %d ", #params)..", "..robot.id)
    for idx, seg in ipairs(params) do
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

-------------------------------------------------------------------------------------------------------------------------------------
-------------------------------------------------------------------------------------------------------------------------------------
-- Funciones de operaciones y calculos
-------------------------------------------------------------------------------------------------------------------------------------
-------------------------------------------------------------------------------------------------------------------------------------

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