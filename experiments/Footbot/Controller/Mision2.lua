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

--[[ 
--================== ACTIVAR PARA EPCUK ==================

-- 721549    semilla 1er exp. FootBot

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
total_ticks = 15000 * ticks_per_second
Num_sensores_de_proximidad = 24 -- Numero de sensores del robot
------------------------------
--Parámetro de simulación internos

angulo_treshold = 10            -- Sensibilidad del observador a caambios en los datos de angulo obtenidos del demostrador, ajuste recomendado: 10 < sensitivity_2 < 50
demostrador_threshold = 41      -- Distancia máxima a la que a la que el demostrador espera ver al observador
threshold = 40                  -- Distancia a la que se ubica el observador del demostrador
avance = 1                      -- Influye en la sensibilidad de deteccion de segmentos, valor minimo = 1
sensitivity_2 = 2               -- Sensibilidad del observador a caambios en los datos de distancia obtenidos del demostrador, ajuste recomendado: 1 < sensitivity_2 < 2

speed_dem = 5                   -- Velocidad del demostrador al ejecutar el patrón
d_speed = 15                    -- velocidad normal
g_speed = 50                    -- Velocidad de giro

max_tracking_distance = 65      -- Máxima distancia a la que se puede alejar el demostrador antes de que el observador se acerque para no perderlo de vista. cm
min_tracking_distance = 30      -- Miínima distancia a la que se puede acercar el demostrador antes de que el observador se aleje para no colisionar. cm
porcentaje = 0.5                -- 1 < porcentaje < 0.33 , indica que tanto el observador busca acercarse al demostrador cuando este cruza el max_tracking_distance.

sensitivity = ( ( (d_speed-speed_dem)/ticks_per_second ) * avance ) / 4 -- speed_dem/ticks_per_second indica cuanto avanza el robot demostrador en un tick, 
                                                                        -- *avance indica con que valor se va a comparar, en este caso el minimo para maxima sensibilidad
------------------------------
--Parametros de patrónes predefinidos
perm1c = 80
perm2c = 80

permt = 30

l1 = 10
l2 = 60

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

    if sensitivity >= ((speed_dem/ticks_per_second ) * avance ) / 3 then sensitivity = ((speed_dem/ticks_per_second ) * avance ) / 3 end

    robot.colored_blob_omnidirectional_camera.enable()
    determinar_rol()

    id_map = {
    Footbot0 = 1,
    Footbot1 = 2,
    Footbot2 = 3,
    Footbot3 = 4,
    Footbot4 = 5,
    Footbot5 = 6,
    Footbot6 = 7,
    Footbot7 = 8,
    Footbot8 = 9,
    Footbot9 = 10,
    Footbot10 = 11,
    Footbot11 = 12
    }
    reverse_map = {}
    for name, code in pairs(id_map) do
    reverse_map[code] = name
    end

end

-------------------------------------------------------------------------------------------------------------------------------------
-- Función principal que se ejecuta en cada tick
-------------------------------------------------------------------------------------------------------------------------------------

timer = 1
function step()
    setleds({ color = "orange", only = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12} })
    if scout then
        setleds({ color = "cyan", only = {1, 3, 5, 7, 9, 11} })
    end
    if     wh_role == 0 then
        forager()           -- Ejecutar si es forager
    elseif wh_role == 1 then
        alumno()            -- Ejecutar si es Recruited forager
           
    elseif wh_role == 2 then
        local r = {{0.95 , 1}}
        RW_V1( r )          -- Ejecutar si es Recruited scout
    end

    if timer > total_ticks - 1 then 
        if not scout then 
            print_all_sources_data(source)
            sort_and_print_sources({ ordenar = "value", imprimir = "best" })
        end
    end

    timer = timer + 1
end

-------------------------------------------------------------------------------------------------------------------------------------
-- Funciones para cada rol
-------------------------------------------------------------------------------------------------------------------------------------

motion = true --Realizar RandomWalking si es true.
f1 = 0
function forager()    
    if      f1 == 0 then
        exploration()
    elseif  f1 == 1 then
        waggle_dance()
    end
end

a1 = 0
a2 = 0 
alumno_counter = 0

tick_counter     = 0                          -- Nuevo contador de ticks
A_time           = 100
check_interval   = A_time * ticks_per_second  -- Intervalo de ticks para la revisión

tick_counter_2   = 0                          -- Nuevo contador de ticks
A_time_2         = A_time*0.8
check_interval_2 = A_time_2 * ticks_per_second-- Intervalo de ticks para la revisión
function alumno()

    if a1 == 0 then

 -- Incrementamos el contador de ticks
        if c3 == 0 then
            tick_counter = tick_counter + 1

            -- Verificamos si es el momento de chequear la condición
            if tick_counter >= check_interval then
                -- Reiniciamos el contador de ticks
                tick_counter = 0

                    -- Generamos un número aleatorio entre 1 y 100
                local random_number = robot.random.uniform_int(1, 100)
                if source_count > 0 then

                    if random_number <= 50 +10 * best_source.data.value[1] then -- 80% de probabilidad
                        a1                          = 1
                        alumno_counter              = 1
                        -- log("\n\n((((((((((((((((((((((((((((()))))))))))))))))))))))))))))")
                        -- log("Explotando Mejor Fuente Conocida ..."..robot.id)
                        -- log("random_number = "..random_number)
                        -- log("(((((((((((((((((((((((((((()))))))))))))))))))))))))))))\n\n(")
                    else
                        if timer     < 50000 then
                            angle_food_source_random    = robot.random.uniform_int( -45, 180 )
                            distance_food_source_random = robot.random.uniform_int( 800, 2000)
                        elseif timer < 65000 then
                            angle_food_source_random    = robot.random.uniform_int( -45, -90 )
                            distance_food_source_random = robot.random.uniform_int( 500, 2000)
                        elseif timer < 100000 then
                            angle_food_source_random    = robot.random.uniform_int( -45, 180 )
                            distance_food_source_random = robot.random.uniform_int( 800, 2000)
                        elseif timer < 120000 then
                            angle_food_source_random    = robot.random.uniform_int( -90,-180 )
                            distance_food_source_random = robot.random.uniform_int( 800, 1200)
                        else
                            angle_food_source_random    = robot.random.uniform_int(-180, 180 )
                            distance_food_source_random = robot.random.uniform_int( 500, 2000)
                        end
                        a1                          = 1
                        alumno_counter              = -1
                        -- log("\n\n((((((((((((((((((((((((((((()))))))))))))))))))))))))))))")
                        -- log("Saliendo a Explorar ..."..robot.id)
                        -- log("angle_food_source_random = "..angle_food_source_random..", distance_food_source_random = "..distance_food_source_random)
                        -- log("random_number = "..random_number)
                        -- log("(((((((((((((((((((((((((((()))))))))))))))))))))))))))))\n\n(")
                    end
                else
                    if timer     < 50000 then
                        angle_food_source_random    = robot.random.uniform_int( -45, 180 )
                        distance_food_source_random = robot.random.uniform_int( 800, 2000)
                    elseif timer < 65000 then
                        angle_food_source_random    = robot.random.uniform_int( -45, -90 )
                        distance_food_source_random = robot.random.uniform_int( 500, 2000)
                    elseif timer < 100000 then
                        angle_food_source_random    = robot.random.uniform_int( -45, 180 )
                        distance_food_source_random = robot.random.uniform_int( 800, 2000)
                    elseif timer < 120000 then
                        angle_food_source_random    = robot.random.uniform_int( -90,-180 )
                        distance_food_source_random = robot.random.uniform_int( 800, 1200)
                    else
                        angle_food_source_random    = robot.random.uniform_int(-180, 180 )
                        distance_food_source_random = robot.random.uniform_int( 500, 2000)
                    end
                    -- end
                    -- log("\n\n((((((((((((((((((((((((((((()))))))))))))))))))))))))))))")
                    -- log("No Conocemos Fuentes, Saliendo a Explorar ..."..robot.id)
                    -- log("angle_food_source_random = "..angle_food_source_random..", distance_food_source_random = "..distance_food_source_random)
                    -- log("random_number = "..random_number)
                    -- log("(((((((((((((((((((((((((((()))))))))))))))))))))))))))))\n\n(")
                    a1 = 1
                    alumno_counter = -1
                end
            end
        else
            alumno_counter = 0
        end

        if WR == 0 then
            observador(false)  
        else
            if a2 == 0 then 
                a2      = 1 
                A_time_2= A_time* (0.5 + 0.1* best_source.data.value[1])
            else
                observador_triste()
                tick_counter_2 = tick_counter_2 + 1
                if tick_counter_2 > check_interval_2 then
                    WR = 0
                end
            end
        end

    elseif a1 == 1 then 
        exploration()
    end
    --if robot.id == "Footbot7"  then log("alumno_counter = "..alumno_counter..", "..robot.id) end

end

d1 = 0
c5 = 0
demostrador_avg_angle = 0
demostrador_avg_dist = 0
function demostrador(m)
    if d1 == 0 then
        detectar_observador(m)
        --log("detectar_observador")

    elseif d1 == 1 then   
        espera_antes_de_ejecutar_patron()
        --log("espera_antes_de_ejecutar_patron")

    elseif d1 == 2 then
        if c5 == 1 then
            patron_imitado(params)
            --log("Patron 1 (imitado) ejecutandose")
        else
            Patron_Original{
                params = {
                    { l1, -45 },
                    { l1, -45 },
                    { l2, -45 },
                    { l1,  45 },
                    { l1,  45 },
                    { l1,  90 },
                    { l1,  45 },
                    { l2,  45 },
                    { l1, -45 },
                    { l1, -45 },
                },
                name = "8"
            }
        --log("Patron 2( Original) ejecutandose")
        end
    elseif d1 == 3 then
        --log("Mestro triste")
        demostrador_triste()
    end
end

c3 = 0
c4 = 0
function observador(m)
    if c3 == 0 then
        detect_and_stop(m)
        --if robot.id == "Footbot7"  then log("detect_and_stop "..robot.id) end

    elseif c3 == 1 then
        move_until_obstacle()
        --if robot.id == "Footbot7"  then log("move_until_obstacle "..robot.id) end

    elseif c3 == 2 then
        if c4 == 0 then
            store_relative_positions()
            --if robot.id == "Footbot7"  then  log("store_relative_positions "..robot.id) end
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
            --patron_imitado(params)
            trip_information(params)
        end
        --log("analizando movimiento por ".. robot.id)
    elseif c3 == 3 then
        observador_triste()
    end

end

-------------------------------------------------------------------------------------------------------------------------------------
-------------------------------------------------------------------------------------------------------------------------------------
-------------------------------------------------------------------------------------------------------------------------------------
--                                          Funciones de comportamiento de los FootBot                                               --
-------------------------------------------------------------------------------------------------------------------------------------
-------------------------------------------------------------------------------------------------------------------------------------
-------------------------------------------------------------------------------------------------------------------------------------

-------------------------------------------------------------------------------------------------------------------------------------
-------------------------------------------------------------------------------------------------------------------------------------
-- Funciones del Mestro / Alumno
-------------------------------------------------------------------------------------------------------------------------------------
-------------------------------------------------------------------------------------------------------------------------------------

-------------------------------------------------------------------------------------------------------------------------------------
-- exploration()
-------------------------------------------------------------------------------------------------------------------------------------

e1 = 0
function exploration()
    if e1 == 0 then
        --setleds("orange")
        --log("detect_nest-----------------"..robot.id)
        detect_nest()
    elseif e1 == 1 then
        --log("Go_to_source-----------------"..robot.id)
        Go_to_source()
    end
end


    DN_angle            = 0
    DN_light_angle      = 0
    DN_c1               = 0
    DN_c2               = 0 
    DN_average_angel    = 0
    function detect_nest()

        local G = ground{
            range = {
                {0.11, 0.2 }    -- gris centro
            } ,
            print_all_data        = false,
            print_detected_ranges = false                         
        }
        if DN_c2 == 0 then   
            --------------------------------- Comprobar si estamos en el círculo negro (rango 1)
            if #G.Gr == 0 then
                --log("Todavía no encontramos el círculo"..robot.id)
                local r = {{0.81 , 1}}
                RW_V1( r )
                return
            end
            --------------------------------- Ya estamos en el rango, verificar centrado
            
            local all_nest = true
            for i, val in ipairs(G.motor) do
                if val > 0.2 or val < 0.11 then
                    all_nest = false
                    r_move()
                    --log("Sin el centro del círculo"..robot.id)
                    break
                end
            end

            if not all_nest then
                -- Estrategia simple: girar hacia el lado más OSCURO
                local izquierda = G.motor[1] + G.motor[2]   -- lado izquierdo
                local derecha   = G.motor[3] + G.motor[4]   -- lado derecho
                local delante   = G.motor[1] + G.motor[4]   -- lado derecho
                local atras     = G.motor[2] + G.motor[3]   -- lado derecho

                if izquierda > derecha then
                    robot.wheels.set_velocity(  10.0,  0.0)     -- girar derecha
                elseif derecha > izquierda then
                    robot.wheels.set_velocity(  0.0,  10.0)     -- girar izquierda
                elseif delante > atras then
                    robot.wheels.set_velocity(-20.0, -40.0)   -- avanzar recto
                else
                    robot.wheels.set_velocity( 20.0,  40.0)   -- avanzar recto
                end
                return
            else
                DN_c2 = 1
                robot.wheels.set_velocity( 0.0,  0.0)   -- avanzar recto
            end
        end
        --------------------------------- Orientarse respecto al sol
        sun_compass()
        if SC_end == true then
            SC_end = false
            e1      = 1
            DN_c2   = 0
            r_move()
            --log("MIRANDO AL SOL, "..robot.id)
            return
        end 
    end

    GTS_c1                  = 0
    GTS_c2                  = 0 
    GTS_c3                  = 0  
    GTS_val                 = 0 
    GTS_params              = {}
    angle_food_source       = 0
    distance_food_source    = 0

    to_find                 = false ---------------===============--------------------
    GTS_counter             = 0     ---------------===============--------------------
    GTS_change_source       = 0
    function Go_to_source()
        local GTS_angle     = 0
        local GTS_distance  = 0
        local prox = proximidad()
        
        if scout or to_find or alumno_counter > 0 then 
            GTS_angle   = best_source.data.angle[1]
            GTS_distance= best_source.data.distance[1]

        else
            if alumno_counter  == 0 then
                GTS_angle   = angle_food_source
                GTS_distance= distance_food_source 
            else  
                GTS_angle   = angle_food_source_random
                GTS_distance= distance_food_source_random
            end
        end

        if GTS_c1== 0 then
            GTS_value = GTS_advance{ angle = GTS_angle, distance = GTS_distance }
            
            -- log("GTS_angle, "..GTS_angle.."°, "..robot.id)
            -- log("GTS_distance, "..GTS_distance.." cm, "..robot.id)
            -- log(" ================================================= ")
        end
        if GTS_c1== 1 then 
            exploring_areas_close{ angle = GTS_angle, distance = GTS_distance }
        end
        if GTS_c1== 2 then 
            GTS_go_back{ angle = GTS_angle, distance = GTS_distance , value = GTS_val}
            --log("GTS_angle, "..GTS_angle"°, "..robot.id)
        end
    end


        function GTS_advance(GTS_a)
            linear_velocity = conditional_speed()

            local GTS_angle         = GTS_a.angle
            local GTS_distance      = GTS_a.distance
            local GTS_value         = 0
            local stop              = false
            --local linear_velocity   = GTS_a.linear_velocity
            local proximity = proximidad()
            local G = ground{
                range = {
                    {0.71, 0.8 },   -- gris 7
                    {0.51, 0.6 },   -- gris 5
                    {0.31, 0.4 },   -- gris 3
                    {0.0 , 0.1 },   -- negro
                    {0.91, 1   },   -- Blanco
                    {0.21, 0.3 },   -- gris colmena
                    {0.11, 0.2 }    -- gris centro
                    }             
                , print_all_data        = false
                , print_detected_ranges = false
                -- base_ground_active = false
            }

            if GTS_c3 == 0 then 
                if proximity[1] > 0 and proximity[1] < 0.9 and math.abs(proximity[2]) <= 45 then   
                    stop = true
                    Temporizador_1(600 * 1/ticks_per_second)
                    --log("Esperando a que el obstaculo se quite de enfrente.."..robot.id)
                    if temp == false then 
                        --log("Esperando ..........."..robot.id)
                        if GTS_c2 ~= proximity[1] and GTS_c3 == 0 then 
                            GTS_c2 = proximity[1]
                            --log("Continuamos Esperando .."..robot.id)
                        else   
                            --log("Regresando a buscar oportunidad de salir a explorar.."..robot.id)    
                            if check_for_value{ table = G.Gr , equals_to = {6,7} } then
                                GTS_c3 =  1
                            else
                                GTS_c3 = -1
                            end
                        end
                    end     
                elseif proximity[1] == 1 then
                    if check_for_value{ table = G.Gr , equals_to = {7} } then
                        GTS_c3 = 1
                    elseif proximity[2] < 135 or proximity[2] > -135 then
                        stop = true
                        Temporizador_1(4)
                        if temp == false then 
                            --log("Obtaculo hizo contacto por mucho tiempo!!.."..robot.id)  
                            --if check_for_value{ table = G.Gr , equals_to = {3,5} } then
                                GTS_c3 = 1
                            --end
                        end
                    else
                        stop   = false
                        GTS_c3 = 0
                        r_Temporizador_1()
                    end
                else
                    stop   = false
                    GTS_c3 = 0
                    r_Temporizador_1()
                end
                move(GTS_distance,GTS_angle,linear_velocity,10,false,nill,stop)

            elseif GTS_c3 > 0 then  

                if GTS_c3 <= 10*ticks_per_second then 

                    local r = {{0.95 , 1}}
                    RW_V1( r )
                    GTS_c3 = GTS_c3 + 1
                    --log("ESPERANDO  !! ; "..GTS_c3..", "..robot.id) 

                else  
                    if not scout then 
                        e1 = 0
                        DN_angle            = 0
                        DN_light_angle      = 0
                        DN_c1               = 0
                        DN_c2               = 0 
                        DN_average_angel    = 0
                        GTS_c1              = 0
                        GTS_c2              = 0 
                        GTS_c3              = 0 
                        GTS_params          = {}
                        stop = false
                        r_Temporizador_1()
                        r_Temporizador_2()
                        r_move()
                        --log("Volviendo a exĺorar !!.."..robot.id) 
                        return

                    else
                        setleds("black")
                        resetforager{to_explorer = true}
                        --log("Volviendo a exĺorar by scout !!.."..robot.id) 
                        return      
                    end 
                end
            else 
                c1 = 2
            end

            if c1 == 2 then 
                GTS_val = G.Gr[1][1] 
                if GTS_val == 5 or GTS_val == 6 or GTS_val == 7 then 
                    GTS_val = 0
                end

                if GTS_c3 == -1 then 
                    GTS_val = 1
                end
                    
                --log("GTS_val = "..GTS_val)
                -- Construcción de zig-zag (zig = 45, zag = -90)
                if GTS_val > 0 then

                    if GTS_c3 == -1 then 
                        GTS_val = 0
                        GTS_c3  = 0
                    end
                    GTS_c1     = 2
                    --log(" ===== Encontramos la fuente en el angulo & distancia brindada, regresando ===== "..robot.id)   
                    --create_waggle_dance{distance = GTS_distance, angle = GTS_angle, value = GTS_val}
                else
                    -- if robot.id == "Footbot0" or  robot.id == "Footbot4"  then
                    --     GTS_c1 = 1
                    -- else
                    GTS_c1 = 1
                    -- end
                end
                r_move()
                return GTS_val
                --log ( "Valor de la fuente de alimento = "..GTS_value..", por scout: "..robot.id)
            end
        end

        G1                  = 0
        EAC_sun_angle_ant   = 0
        EAC_angle           = 0 
        EAC_ang_direction   = 0 
        exploracion         = false --=========
        function exploring_areas_close(GTS_a)
            local GTS_angle         = GTS_a.angle
            local GTS_distance      = (GTS_a.distance)/100
            local GTS_value         = GTS_a.value

            if      G1 == 0 then
                    local tr_l    = track_light{print = false}
                    EAC_sun_angle_ant = tr_l.avg_angle * 180/math.pi
                    G1 = 1 
            elseif  G1 == 1 then 
                spiral_exploration()
            elseif  G1 == 2 then 
                --move(SE_distance*100,90,10,10)
                move(2*SE_distance*100,0,10,10)
                if c1 == 2 then
                    G1 = 3 
                    r_move()
                    --log("En posicion inicial ( 0 , 0 )... "..robot.id)
                    --EAC_angle = 180 - ( SE_angle_deg )
                    EAC_angle = 180 - SE_angle_deg
                    if EAC_angle > 180 then
                        EAC_angle = EAC_angle - 360
                        EAC_ang_direction =  1
                    elseif EAC_angle < -180 then
                        EAC_angle = EAC_angle + 360
                    else
                        EAC_ang_direction = -1
                    end
                    -- EAC_angle = math.abs(EAC_angle)
                    -- log("SE_angle_deg =  "..SE_angle_deg.."° , "..robot.id)
                    -- log("EAC_angle =  "..EAC_angle.."° , "..robot.id)
                    -- log("GTS_distance =  "..GTS_distance.." m, "..robot.id)
                    -- log("SE_distance =  "..SE_distance.." m, "..robot.id)
                    triangle_info_sas = calculateTriangleProperties_SAS(GTS_distance, SE_distance, EAC_angle )

                    -- log("--- Resultados del Triángulo (Lado-Ángulo-Lado) ---")
                    -- log(string.format("Distancia AC: %.2f", triangle_info_sas.distance[1]))
                    -- log(string.format("Distancia CB: %.2f", triangle_info_sas.distance[2]))
                    -- log(string.format("Distancia AB: %.2f", triangle_info_sas.distance[3]))
                    -- log("")
                    -- log(string.format("Ángulo A (entre AC y CB): %.2f grados", triangle_info_sas.angle[1]))
                    -- log(string.format("Ángulo B (entre CB y AB): %.2f grados", triangle_info_sas.angle[2]))
                    -- log(string.format("Ángulo C (entre AB y AC): %.2f grados", triangle_info_sas.angle[3]))

                    -- -- Verificación de la suma de ángulos
                    -- local sum_angles_sas = triangle_info_sas.angle[1] + triangle_info_sas.angle[2] + triangle_info_sas.angle[3]
                    -- log(string.format("\nSuma de los ángulos internos: %.2f grados", sum_angles_sas))


                end
            elseif  G1 == 3 then      

                sun_compass(EAC_sun_angle_ant)
                if SC_end then
                    GTS_c1 = 2
                    if SE_source == true then
                        -- G1 = 0 
                        angulo_de_exploracion    = (GTS_angle) - EAC_ang_direction*triangle_info_sas.angle[1]
                        distancia_de_exploracion = triangle_info_sas.distance[3]*100 
                        valor_de_exploracion     = SE_value
                        exploracion              = true

                        -- log(string.format("[%s] ===== Encontramos fuente cerca al angulo & distancia brindados, regresando ===== . Distancia: %.2f cm | Ángulo: %.2f° | Valor: %.2f ",
                        --     robot.id, distancia_de_exploracion, angulo_de_exploracion , valor_de_exploracion))    
                        r_move()
                    else
                        exploracion              = false
                        --log(" ===== NO encontramos fuentes cerca, regresando ===== "..robot.id)                        
                    end
                end
            end
        end

        function GTS_go_back(GTS_a)
            linear_velocity = conditional_speed()
            local GTS_angle         = 0
            local GTS_distance      = 0
            local GTS_value         = 0
            if  not exploracion then
                GTS_angle         = GTS_a.angle
                GTS_distance      = GTS_a.distance
                GTS_value         = GTS_a.value
            else
                GTS_angle         = angulo_de_exploracion
                GTS_distance      = distancia_de_exploracion
                GTS_value         = valor_de_exploracion
            end
            
            local proximity = proximidad()
            local G = ground{
                range = {
                    {0.71, 0.8 },   -- gris 7
                    {0.51, 0.6 },   -- gris 5
                    {0.31, 0.4 },   -- gris 3
                    {0.0 , 0.1 },   -- negro
                    {0.91, 1   },   -- Blanco
                    {0.21, 0.3 },   -- gris colmena
                    {0.11, 0.2 }    -- gris centro
                    }                
                , print_all_data        = false
                , print_detected_ranges = false
                -- base_ground_active = false
            }

            if G.Gr[1][1] == 6 then
                c1 = 2
            end

            move(-GTS_distance,0,linear_velocity,10)
            if c1 == 2 then
                r_move()
                if GTS_value > 0 then 
                    GTS_params = create_waggle_dance{distance = GTS_distance, angle = GTS_angle, value = GTS_value}
                    if scout or to_find then 
                        setleds("orange")
                        robot.wheels.set_velocity(0, 0)   
                        
                        if to_find then
                            --setleds("cyan")  
                            log(" --------------------------------------------------------------------------------------------------------------- ")
                            log("Reporte -> scout Bee: "..robot.id)
                            log( "Fuente de alimento  encontrada | value = "..GTS_value.." | ángulo = "..GTS_angle.."° | Distancia = "..GTS_distance)
                            log(" --------------------------------------------------------------------------------------------------------------- ")
                        end 
                        resetforager{to_waggle = true}
                      
                    else
                        setleds("black")

                        if alumno_counter == -1 then  
                            local new_source = trip_information(GTS_params)
                            if not new_source then
                                setleds("black")
                                log(" --------------------------------------------------------------------------------------------------------------- ")
                                log("Reporte -> Scout Bee: "..robot.id)
                                log("Busqueda = Sin Nuevas MEJORES Fuentes| value = "..GTS_value.." | ángulo = "..GTS_angle.."° | Distancia = "..GTS_distance)
                                log(" --------------------------------------------------------------------------------------------------------------- ")
                                resetalumno()
                                alumno_counter = 0
                                return 
                            end
                        end

                        best_source = sort_and_print_sources({ ordenar = "value", imprimir = "none" })
                        -- =================================================== --   
                        best_source.data.angle   = {GTS_angle   }   
                        best_source.data.distance= {GTS_distance} 
                        best_source.data.value   = {GTS_value   }   
                        
                        --table.insert(best_source_actually_obj.params, GTS_params)
                        best_source.params       = {GTS_params  }  
                        --best_source             = {} 
                        best_source_actually_obj=  source_0
                        -- =================================================== --
                        --print_single_source_data_nested_params(best_source)
                        if best_source.data.value[1] == 8 then 
                            to_find = true
                            log("\n\n\n()()()()()()()()()()()()()()()()()()()()()()()()()()()()()()()()()()()()()()()()()()()()()()()()()()()()()()")
                        end


                        log(" /////////////////////////////////////////////////////////////////////////////////////////////////////////////// ")
                        log("Reporte -> Recruited Bee: "..robot.id)
                        log( "Fuente de alimento  validada | value = "..GTS_value.." | ángulo = "..GTS_angle.."° | Distancia = "..GTS_distance)
                        log(" /////////////////////////////////////////////////////////////////////////////////////////////////////////////// ")                                          

                        if to_find then
                            log("()()()()()()()()()()()()()()()()()()()()()()()()()()()()()()()()()()()()()()()()()()()()()()()()()()()()()()\n\n\n")
                            robot.wheels.set_velocity(0, 0)
                            setleds("black")
                            resetalumno()   
                            wh_role     = 0  
                            GTS_params  = best_source.params[1]
                            resetforager{to_waggle = true}
                            --print_all_sources_data(source)
                        else
                            -- wh_role = 1
                            -- resetalumno()  
                            -- setleds("cyan")  
                            -- return
                            -------------------------------------------------------------------
                            resetalumno()   
                            robot.wheels.set_velocity(0, 0)  
                            wh_role     = 0  
                            GTS_params  = best_source.params[1]
                            --print_all_sources_data(source)
                            resetforager{to_waggle = true}
                            --imprimir_parametros(GTS_params)
                            -------------------------------------------------------------------
                        end                  
                        --imprimir_parametros(GTS_params)
                        --log(" Fin de exploración")
                    end

                    GTS_counter = 1
                    --log(" ======================== GTS_COUNTER ============= "..GTS_counter..", "..robot.id)
                    return GTS_params
                else
                    if alumno_counter ~= -1 then
                        if not scout and not to_find then
                            wh_role = 1
                            setleds("black")
                            log(" \n\n=============================================================================================================== ")
                            log("Reporte -> Scout Bee: "..robot.id)
                            log("Fuente de alimento NO validada | value = "..GTS_value.." | ángulo = "..GTS_angle.."° | Distancia = "..GTS_distance)
                            log(" ===============================================================================================================\n\n ")
                            delete_source_by_id(source_count)
                            source_count = source_count - 1
                            resetalumno()
                        else
                            -- log(" =============================================================================================================== ")
                            -- log("Reporte -> Recruited Bee: "..robot.id)
                            -- log("Fuente de alimento NO encontrada | value = "..GTS_value.." | ángulo = "..GTS_angle.."° | Distancia = "..GTS_distance)
                            -- log(" =============================================================================================================== ")
                            resetforager{to_explorer = true}
                        end
                    else
                        -- log(" --------------------------------------------------------------------------------------------------------------- ")
                        -- log("Reporte -> Scout Bee: "..robot.id)
                        -- log("Busqueda = Sin Nuevas Fuentes| value = "..GTS_value.." | ángulo = "..GTS_angle.."° | Distancia = "..GTS_distance)
                        -- log(" --------------------------------------------------------------------------------------------------------------- ")
                        if not scout and not to_find then
                            wh_role = 1
                            setleds("black")
                            resetalumno()
                        else
                            resetforager{to_explorer = true}
                        end
                    end
                end

                alumno_counter = 0

            end
        end

            
            function check_for_value(opts)
                local target_table = opts.table
                local different_from = opts.different_from -- Puede ser un valor simple o una tabla de valores
                local equals_to = opts.equals_to           -- Puede ser un valor simple o una tabla de valores

                -- Función auxiliar para verificar si un valor está DENTRO de una tabla (es decir, ES IGUAL a ALGUNO de ellos)
                local function is_value_in_table(value, compare_table)
                    if type(compare_table) ~= "table" then
                        -- Si no es una tabla, simplemente compara directamente
                        return value == compare_table
                    end
                    -- Si es una tabla, busca el valor dentro de ella (OR Lógico: si es igual a CUALQUIERA)
                    for _, v in ipairs(compare_table) do
                        if value == v then
                            return true
                        end
                    end
                    return false
                end

                -- Validar que se recibió una tabla para procesar
                if type(target_table) ~= "table" then
                    return false
                end

                -- Recorre cada subtabla en target_table
                for i, inner_table in ipairs(target_table) do
                    -- Asegúrate de que el elemento actual sea una tabla y tenga un primer elemento
                    if type(inner_table) == "table" and inner_table[1] ~= nil then
                        local current_value = inner_table[1]

                        -- Lógica para 'different_from' (AND Lógico: diferente de TODOS los valores en la lista)
                        -- Si 'different_from' está definido:
                        --   Devuelve true si el current_value NO está en la lista de 'different_from'
                        --   (es decir, no es igual a 'val1' Y no es igual a 'val2' Y no es igual a 'valN').
                        if different_from ~= nil then
                            -- if not is_value_in_table(current_value, different_from) then
                            --     return true
                            -- end
                            -- La condición equivalente a (NO está en la lista) es la que ya teníamos.
                            -- Si is_value_in_table devuelve false (no lo encontró), entonces es diferente de todos.
                            if not is_value_in_table(current_value, different_from) then
                                return true
                            end
                        end

                        -- Lógica para 'equals_to' (OR Lógico: igual a ALGUNO de los valores en la lista)
                        -- Si 'equals_to' está definido:
                        --   Devuelve true si el current_value es igual a ALGUNO de los valores en la lista 'equals_to'.
                        if equals_to ~= nil then
                            if is_value_in_table(current_value, equals_to) then
                                return true
                            end
                        end
                    end
                end

                -- Si ninguna de las condiciones se cumplió después de revisar todas las subtables, devuelve false
                return false
            end

            l1 = 0
            function create_waggle_dance(opts)
                local GTS_distance  = opts.distance
                local GTS_angle     = opts.angle
                local GTS_value     = opts.value
                local imprimir      = opts.imprimir or false
                l1                  = GTS_distance / (GTS_value*20)
                local dir           = 1
                --log("L1 = "..l1.." , "..robot.id)
                GTS_params = {}
                table.insert(GTS_params, {l1 , GTS_angle})
                if GTS_angle >= 0 then 
                    dir = -1
                else
                    dir =  1
                end

                --table.insert(GTS_params, {l1, dir * 90})
                -- for i = 2, GTS_value do
                --     --table.insert(GTS_params, {l1, -dir * 90})
                --     table.insert(GTS_params, {l1,  dir * 90})
                -- end
                for i = 2, GTS_value do
                    -- Usamos el operador de módulo para saber si 'i' es par o impar
                    if i % 2 == 0 then
                        -- Si 'i' es par, se hace lo que ya tenías
                        table.insert(GTS_params, {l1, dir * 90})
                    else
                        -- Si 'i' es impar, se hace lo mismo pero con -dir
                        table.insert(GTS_params, {l1, -dir * 90})
                    end
                end

                table.insert(GTS_params, {0,  45})
                if imprimir then
                    imprimir_parametros(GTS_params)
                end
                return GTS_params
            end

            function conditional_speed()
                local G1 = ground{
                range = {
                    {0.71, 0.8 },   -- gris 7
                    {0.51, 0.6 },   -- gris 5
                    {0.31, 0.4 },   -- gris 3
                    {0.0 , 0.1 },   -- negro
                    {0.91, 1   },   -- Blanco
                    {0.21, 0.3 },   -- gris colmena
                    {0.11, 0.2 }    -- gris centro
                    }  
                }      
                local velocidad = 0
                if G1.Gr[1][1] == 5 then 
                    velocidad = 50
                    return velocidad
                else
                    velocidad = 5
                    return velocidad
                end
            end

-------------------------------------------------------------------------------------------------------------------------------------
-- waggle_dance()
-------------------------------------------------------------------------------------------------------------------------------------

w1 = 0
WR = 0
function waggle_dance()
    if w1 == 0 then

        if d1 == 0 then GTS_counter = GTS_counter + 1
            if GTS_counter > 2000 then 
                if not scout and not to_find then
                    wh_role = 1
                    resetforager{to_explorer = true}
                    resetalumno()
                    GTS_counter = 0
                    log(" NO ENCONTRAMOS APRENDIZ, VERIFICANDO LA MEJOIRES FUENTES.."..robot.id)
                else
                    resetforager{to_explorer = true}
                    GTS_counter = 0
                    --log(" NO ENCONTRAMOS APRENDIZ, BUSCANDO MEJOIRES FUENTES.."..robot.id)
                end
            end
        else GTS_counter = 0 
        end

        if d1 < 2 then 
            --log("demostrador-----------------"..robot.id)
            demostrador(false)
        elseif d1 == 2 then 
            --log("waggle_dance-----------------"..robot.id)
            waggle_run(GTS_params)
        end

    elseif w1 == 1 then 
        local r = {{0.95 , 1}}
        RW_V1( r )
        setleds({ color = "magenta", only = {1, 3, 5, 7, 9, 11} })
    end
end

    WD_c1   = 0
    vL      = 0
    vR      = 0
    ticks   = 0
    function waggle_run(p)
        if #p > 0 then  
            if not PO_end then 
                Patron_Original{
                    params = p,
                    name = "waggle dance"
                }
            else
                setleds({ color = "green" })
                if WD_c1 == 0 then
                    a = (#p-1) * l1 * math.sqrt(2)
                    local R = 0.5 * a
                    --log("RADIO: "..R)
                    local v_media = 10.0
                    local L = 14

                    vL = v_media * (1 - (L / (2 * R)))
                    vR = v_media * (1 + (L / (2 * R)))
                    -- log("vL: "..vL)
                    -- log("vR: "..vR)

                    local arco = math.pi * R
                    local t = arco / v_media
                    ticks = math.floor(t * ticks_per_second) + timer
                    WD_c1 = 1
                end
                if timer < ticks then
                    robot.wheels.set_velocity(vL, vR)
                    --log("waggle_dance........")
                else
                    robot.wheels.set_velocity(0, 0)
                    --log("waggle_dance stop........")
                    if scout or to_find then 
                        resetforager{to_explorer = true}
                        -- log("waggle_dance stop........"..robot.id)
                        -- log(" VOLVIENDO A EXPLORACIÓN "..robot.id)
                    else
                        -- =================================================== --
                        --log("waggle_dance stop........"..robot.id)
                        -- =================================================== --
                        --resetforager{to_waggle   = true}  ------------================= Sigue propagando la información a las demás recruited Bee.
                        -- =================================================== --
                        wh_role = 1
                        WR      = 1
                        resetforager{to_explorer   = true}
                        resetalumno()                        ------------================= Una vez propague la información cierta cantidad de veces,puede obtener nueva información. 
                        -- =================================================== --
                        setleds("magenta")

                    end
                end
            end
        else
            Temporizador_1(5)
            local r = {{0.95 , 1}}
            RW_V1( r )
            if temp == false then
                --log("waggle_dance NOT........"..robot.id)
                if scout then 
                    resetforager{to_explorer = true}
                else
                    wh_role = 1   
                    resetalumno()
                end
            end
        end
    end


-------------------------------------------------------------------------------------------------------------------------------------
-------------------------------------------------------------------------------------------------------------------------------------
-------------------------------------------------------------------------------------------------------------------------------------
--                                          Funciones de comportamiento de los FootBot                                               --
-------------------------------------------------------------------------------------------------------------------------------------
-------------------------------------------------------------------------------------------------------------------------------------
-------------------------------------------------------------------------------------------------------------------------------------

-------------------------------------------------------------------------------------------------------------------------------------
-------------------------------------------------------------------------------------------------------------------------------------
-- Funciones del Mestro
-------------------------------------------------------------------------------------------------------------------------------------
-------------------------------------------------------------------------------------------------------------------------------------


-------------------------------------------------------------------------------------------------------------------------------------
-------------------------------------------------------------------------------------------------------------------------------------
-- Funciones del Demostrador
-------------------------------------------------------------------------------------------------------------------------------------
-------------------------------------------------------------------------------------------------------------------------------------

function detectar_observador(m)
    local blobs = robot.colored_blob_omnidirectional_camera
    local angle = 0
    local motion = m
    --setleds({ color = "red" })
    if motion then
        RandomWalking()
    else
        local r = {{0.95 , 1}}
        RW_V1( r )
    end
    demostrador_avg_angle = 0

    if RW_b1 == 0 then
        for i, blob in ipairs(blobs) do
            local r, g, b = blob.color.red, blob.color.green, blob.color.blue

            if r < 100 and g < 100 and b > 200 then  -- Si detecta un objeto azul
                d1 = 1
                angle = blob.angle * (180 / math.pi) -- Obtener el ángulo del blob en Grados
                demostrador_avg_angle = angle
                -- if robot.id == "Footbot11" then  
                    -- log("¡¡ observador DETECTADO !! a: "..demostrador_avg_angle.."°, por " .. robot.id)
                --  end
                robot.wheels.set_velocity(0, 0)
                
                r_Temporizador_1()
                r_move()

                setleds({ color = "yellow" })
                break
            end
        end
    end
end

demostrador_avg_dist_1 = 0
EAEP_ang_ant = 0
EAEP_c1 = 0
EAEP_c2 = 0
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
            if demostrador_avg_dist_1 > demostrador_threshold*0.5 and demostrador_avg_dist_1 < demostrador_threshold then
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
                -- log("Alumno a " .. demostrador_avg_dist_1.." cm".." , Máximo permitido: "..demostrador_threshold..", Minimo permitido: "..0.5*demostrador_threshold)
                -- log( " -------------------------------------------------------------------------"  )
               resetforager{to_waggle = true}
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
    local prox = proximidad()

    if prox[1] < 0.9 and  k == 0 then
    --log("prox[1]: "..prox[1]..", prox[2]: "..prox[2]..", "..robot.id)

        if ind <= #params and k == 0 then
            local s1 = params[ind][1]
            local s2 = params[ind][2]

            move(s1, s2, speed_dem, 10, false, false)

            if c1 == 2 then
                if ind == #params then
                    local distance_food_source = 0
                    for i = 1, #params do
                        -- params[i] te da la sub-tabla, por ejemplo: {distancia_valor, angulo_valor}
                        -- params[i][1] accede al primer elemento de esa sub-tabla, que es la distancia
                        distance_food_source = distance_food_source + params[i][1]*10
                    end

                    --setleds({ color = "green" }) -- comentado: PARA ESTA MISIÓN SE DEBE CAMBIAR A VERDE DESDE EL ULTIMO PARÁMETRO

                    --imprimir_posicion(posiciones_guardadas)
                    --log(" ========= patrón original: "..name.." finalizado por: " .. robot.id.." ========= ")

                    -- log(" ================================================================= ")
                    -- log(" Información transmitida por: " .. robot.id)
                    -- log(" Fuente de alimento: "..params[1][2].."°".." | "..  (distance_food_source) .."m | valor: ".. #params-1)

                    k = 1
                    ind = ind + 1
                    c1 = 0
                    PO_end = true
                elseif ind < #params then

                    if ind >= #params - 1 then
                        setleds({ color = "green" }) -- comentado: PARA ESTA MISIÓN SE DEBE CAMBIAR A VERDE DESDE EL ULTIMO PARÁMETRO
                    end

                    ind = ind + 1
                    c1 = 0
                end
            end
        end

    elseif prox[1] >= 0.9 and k == 0 then
        resetforager{to_waggle = true}
        --setleds("black")
        -- log( " -------------------------------------------------------------------------" )
        -- log("patrón original: "..name.." NO finalizado debido a obstauclos: " .. robot.id)
        -- log( " -------------------------------------------------------------------------" )
    end

    if k == 1 then
        --resetforager{to_explorer = true}
    end
end

MT_c1 = 0
function demostrador_triste()
    MT_c1 = MT_c1 + 1
    if MT_c1 == 2 then 
        setleds("black")
    end
    local r = {{0.95 , 1}}
    RW_V1( r )
    Temporizador_1(10)
    if temp== false then
        resetDEM()
        --resetforager()
        setleds("orange")
    end
end

-------------------------------------------------------------------------------------------------------------------------------------
-- función para realizar el patron imitado
-------------------------------------------------------------------------------------------------------------------------------------

k                       = 0
indice                  = 1
triangulo_ang           = 0
c5                      = 0
PI_c1                   = 0
PI_end                  = false

angle_food_source      = 0
distance_food_source   = 0
function patron_imitado(params_P)

    local params = params_P
    local prox = proximidad()
    --log("prox[1]: "..prox[1]..", prox[2]: "..prox[2]..", "..robot.id)

    if prox[1] <= 0.5 then
        PI_c1 = 1
    elseif prox[1] > 0.5 and  prox[2] >= -90 and  prox[2] <= 90 then 
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
        
            if indice <= #params then
                local valor1, valor2 = params[indice][1], params[indice][2] + triangulo_ang
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
                            setleds({ color = "green" })
                            -- if robot.id == "Footbot11" then  
                            --     imprimir_posicion(posiciones_guardadas)
                            -- end
                            --log(" ========================================================================= ")
                            --log(" Patrón Imitado: finalizado por: ".. robot.id)
                            --log("Encendiendo led en verde -----------"..robot.id)
                        else
                            c5 = 1
                            k = 1
                            --log(" ========= Patrón aprendido: finalizado por: ".. robot.id.." ========= ")
                            --imprimir_posicion(posiciones_guardadas)
                            --log(" ========= ¡ Ahora soy demostrador ! " .. robot.id.." ========= ")

                            angle_food_source      = params[1][2]
                            distance_food_source   = #params
                            log(" ========= Imformación obtenida:  por: ".. robot.id.." ========= ")
                            log("Fuente de alimento: "..angle_food_source.."°".." | "..distance_food_source .."m")
                            log(" ================================================================= ")
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
    end

    -- elseif prox[1] >= 0.9 and k == 0 then
    --     k = 1
    --     setleds("black")
    --     log("patrón imitado: NO finalizado debido a obstauclos: " .. robot.id)
    -- end

    if k == 1 then
        local r = {{0.95 , 1}}
        RW_V1( r )
        Temporizador_1(1)
        --log(" ranking antes de maestriarrrrrrrrrrrrrrrrrr")
        if temp == false then
            --log(" maestriandoooooooooo")
            if PI_end then
                resetOBS()
                wh_role = 0
            end
            resetDEM()
            k = 0 --Necesario para que se repita el patron si fallo (Poder verlo en la consola)
            setleds("orange")
        end
    end
end


-- Variables de Gestión de Fuentes
source_0 = {
    id = 0, -- Un ID distintivo para indicar que es la fuente nula
    data = {
        angle    = {0},    -- Un solo elemento en 0
        distance = {0}, -- Un solo elemento en 0
        value    = {0},    -- Un solo elemento en 0
    },
    params = {
        {0, 0} -- Un solo par [distancia, ángulo] en 0 para el primer segmento
    }
}
New_source = {} 
source = {}
source_count = 0
best_source = nil
best_source_actually_obj = nil

-- Variables de Navegación y Estado
angle_food_source = 0
distance_food_source = 0
value_food_source = 0
function trip_information(params_P)
    local params_from_current_trip = params_P
    local current_trip_angle = 0
    local current_trip_distance = 0
    local current_trip_value = 0

    local candidate_updated_flag = false 

    if #params_from_current_trip > 0 then            
        current_trip_angle = params_from_current_trip[1][2]
        for i = 1, #params_from_current_trip do
            current_trip_distance = current_trip_distance + params_from_current_trip[i][1]*20
        end
        current_trip_value = #params_from_current_trip 
        
        -- Formato {numero} para los campos data, ya correcto aquí.
        local current_trip_source_data = {
            angle    = {current_trip_angle},
            distance = {current_trip_distance},
            value    = {current_trip_value},
        }

        local is_new_source = true
        local existing_source_matched = nil 

        for _, existing_src in ipairs(source) do
            local temp_candidate_obj = { id = -1, data = current_trip_source_data, params = {} } 
            --if robot.id == "Footbot8" then log(string.format("[DEBUG] Tick: %d - Tipo de is_similar_source antes de llamar: %s", timer, type(is_similar_source))) end

            if type(is_similar_source) ~= "function" then
                --if robot.id == "Footbot8" then  log("[ERROR FATAL] is_similar_source NO es una función en este punto. Deteniendo para inspección.") end
                -- Puedes añadir una línea para detener la simulación o hacer algo que te ayude a ver el estado.
                -- Por ejemplo, un bucle infinito temporal para pausar.
                -- while true do end
            end
            -- Fin de las líneas de depuración

            if is_similar_source(temp_candidate_obj, existing_src) then
                --if robot.id == "Footbot8" then  log("[t="..timer.."]["..robot.id.."] Fuente detectada SIMILAR a ID: "..existing_src.id..". No se añade nueva entrada.") end
                is_new_source = false
                existing_source_matched = existing_src 
                break 
            end
        end

        if is_new_source then
            source_count = source_count + 1
            local new_source_entry         = add_new_source(source_count) 
            new_source_entry.data.angle    = current_trip_source_data.angle 
            new_source_entry.data.distance = current_trip_source_data.distance 
            new_source_entry.data.value    = current_trip_source_data.value
            --if robot.id == "Footbot8" then log(string.format("[t=%d][%s] Nueva fuente (ID: %d) detectada y añadida a 'source'. Valor: %.2f", timer, robot.id, new_source_entry.id, current_trip_value)) end
            existing_source_matched = new_source_entry 
        end
        
        if existing_source_matched then
            if is_source_better(existing_source_matched, best_source) then
                best_source_actually_obj = existing_source_matched 
                candidate_updated_flag = true 
                -- if robot.id == "Footbot8" then  log(string.format("[t=%d][%s] Candidata (best_source_actually_obj, ID: %d) ACTUALIZADA: Mejor que la best_source confirmada. (Valor: %.2f)", 
                --                   timer, robot.id, best_source_actually_obj.id, best_source_actually_obj.data.value[1])) end

                angle_food_source    = best_source_actually_obj.data.angle[1]
                distance_food_source = best_source_actually_obj.data.distance[1]
                value_food_source    = best_source_actually_obj.data.value[1]
                -- if robot.id == "Footbot8" then 
                    -- log(" ----------------------------------------------------------------- ")
                    -- log("Información de CANDIDATA ACTUALIZADA para EXPLORACIÓN (Robot: ".. robot.id.."):")
                    -- log(string.format("  Ángulo:    %.2f°", angle_food_source))
                    -- log(string.format("  Distancia: %.2f cm", distance_food_source))
                    -- log(string.format("  Valor:     %.2f", value_food_source))
                    -- log(" ================================================================= ")
                -- end

                resetOBS() 
                setleds("orange")
                robot.wheels.set_velocity(0, 0)
                a1 = 1
                
            else
                best_source_actually_obj = nil 
                candidate_updated_flag = false 
                -- if robot.id == "Footbot8" then 
                --     log(" ----------------------------------------------------------------- ")
                --     log("[t="..timer.."]["..robot.id.."] Candidata NO mejor que best_source confirmada. best_source_actually_obj vaciada.")
                --     log(" ================================================================= ")
                -- end
                resetalumno()
                robot.leds.set_all_colors("black")
            end
        else
            best_source_actually_obj = nil
            candidate_updated_flag = false
            --if robot.id == "Footbot8" then  log(string.format("[t=%d][%s] No se obtuvo una fuente válida del viaje actual para comparación. best_source_actually_obj vaciada.", timer, robot.id)) end
            resetalumno()
            robot.leds.set_all_colors("black")
        end

    else
        resetalumno()
        robot.leds.set_all_colors("black")
        log( " -------------------------------------------------------------------------" )
        log("Información no detectada :( " .. robot.id)
        log( " -------------------------------------------------------------------------" )
        best_source_actually_obj = nil
        candidate_updated_flag = false
    end

    return candidate_updated_flag
end
-------------------------------------------------------------------------------------------------------------------------------------
-------------------------------------------------------------------------------------------------------------------------------------
--Funciones del observador
-------------------------------------------------------------------------------------------------------------------------------------
-------------------------------------------------------------------------------------------------------------------------------------

function observador_triste()
    setleds("black")
    RandomWalking()
    Temporizador_1(10)
    if temp== false then
        r_Temporizador_1()
        resetOBS()
    end
end

----- Función de busuqeda ejecutada por el observador
function detect_and_stop(m)

    local motion = m
    if motion then
        RandomWalking()
    else
        local r = {{0.95 , 1}}
        RW_V1( r )
    end

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
                    setleds({ color = "blue" }) -- Encender LEDs en azul
                    c3 = 1
                    -- if robot.id == "Footbot3" then  
                    --    log(" ¡¡ demostrador DETECTADO !! a ".. blob.distance.. " y " .. angle .."°, por ".. robot.id)  
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
        --log("demostrador "..i.." a "..blob.distance.."cm y "..angle.."°"..robot.id)

        --Filtrar solo blobs amarillos 
        if r > 200 and g > 200 and b < 100  then
            if math.abs(angle) <= 3*angulo_treshold then --solo al demostrador de frente si hay más de uno
                distance_new = blob.distance
                if MUO__c1 == 0 then                  
                    MUO__distance_an = distance_new
                    MUO__c1 = 1
                end
                --Filtrar blobs correspodnientes al demostrador en distancia correcta
                local max = MUO__distance_an + sensitivity_2*d_speed / ticks_per_second
                local min = MUO__distance_an - sensitivity_2*d_speed / ticks_per_second
                if distance_new  < max and distance_new > min then
                    MUO__distance_an = distance_new
                    total_distance = total_distance + blob.distance
                    MUO__angle_an = blob.angle * (180 / math.pi)
                    count = count + 1
                else
                    --log("demostrador ["..i.."] en distancia incorrecta: "..max.." > "..distance_new.." > "..min)
                    setleds("black")
                end
            else
                --log("demostrador ["..i.."] en Angulo incorrecto: "..math.abs(angle).." < "..3*angulo_treshold)
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
            --     log(string.format("Distancia promedio hasta el demostrador: %.4f y angulo de %.4f , ", avg_distance,MUO__angle_an)..robot.id)
            -- end    
            if avg_distance < threshold then        -- Si la distancia promedio es menor al umbral, detener el robot
                MUO__c3 = 1
                srt_angle_an = MUO__angle_an
                robot.wheels.set_velocity(0, 0)     -- Detener el robot
                --     log("Estamos junto al demostrador, Deteniendome " .. robot.id)
            else
                -- Si no hay obstáculo, seguir avanzando
                robot.wheels.set_velocity(d_speed, d_speed) -- Velocidad fija
            end
        elseif count > 1 then                           -- Si se ha detectado más de un demostrador, el robot vuelve a moverse en RandomWalking
            --log("Confusión de demostradors :( "..robot.id)
            r_Temporizador_1()
            c3 = 3
        elseif temp == false then                       -- Si no se ha establecido conexión con el demostrador durante algún tiempo, el robot vuelve a moverse en RandomWalking
            --log("demostrador no disponible :( "..robot.id)
            r_Temporizador_1()
            c3 = 3
        elseif MUO__c2 == 1 and count == 0 then         -- Si se ha perdido de vista al demostrador, el robot vuelve a moverse en RandomWalking
            --log("demostrador perdido de vista :( "..robot.id)
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
srt_end = 0 ------------------====================-----------------

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
            -- log( " -------------------------------------------------------------------------" )
            -- log("Observación interrumpida, obstaculo en el camino!!, proximidad del ".. proximity[1]*100 .."%, "..robot.id)
            -- log( " -------------------------------------------------------------------------" )
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
        --log(" ======================================================================= "..robot.id)
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
                        

                        --------- CONTROL DINÁMICO   (CUIDADO AL PASAR A EPUCK, PUES AHI ES IMPORTANTE HACER PRIMERO EL AVERAGE)
                                                
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
            elseif r < 100 and g > 200 and b < 100 and srt_angle_an < max_ang and srt_angle_an > min_ang then
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

        --log(" ---------------- COUNT =  "..count..", "..robot.id)
        --========================== Acciones a ejecutar después del análisis--==========================
        if count > 1 then --========================== Si hubo confusión, mas de 1 demostrador .
            resetOBS()
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
            -- log("El demostrador a finalizado el patrón, análizando.... "..robot.id)
            -- log(" ========================================================================= ")
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
        --log(" ======================================================================= "..robot.id)
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

        -- log(string.format("Coordenada %d -> (%.2f, %.2f) | Marker (%.2f, %.2f)", i, coordenadasx[i], coordenadasy[i], r_m, theta_m))
        -- log(string.format("angulo theta %d -> %.2f ", i, theta))
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

angles_a_recorrer = {}
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
        --Logging para verificar
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
TL_c1 = 0
function track_light(opts)
    opts = opts or {}
    local print_debug = opts.print or false -- Cambiado a print_debug para evitar conflicto con la función 'print' de Lua
    local active = opts.active or false
    local threshold = opts.threshold or 0.1  -- valor bajo el cual se activa LG

    local Light = {
        data       = robot.light,
        max_value  = 0,
        max_angle  = 0,
        avg_angle  = 0,   -- ángulo promedio ponderado (en radianes)
        LG         = 0
    }

    local sum_x_weighted = 0.0
    local sum_y_weighted = 0.0
    local total_weight = 0.0 -- Suma de todas las intensidades de luz

    if print_debug then
        log(" ---------------------------------------------------------------- "..robot.id)
    end

    for i, reading in ipairs(robot.light) do
        local angle_rad = reading.angle -- Los ángulos de ARGoS ya están en radianes [-pi, pi]
        local angle_deg = angle_rad * 180 / math.pi
        if print_debug then
            log(string.format("[Light %d] Valor=%.3f Ángulo=%.1f°", i, reading.value, angle_deg))
        end

        -- Valor máximo
        if reading.value > Light.max_value then
            Light.max_value = reading.value
            Light.max_angle = reading.angle -- Guardar en radianes
        end

        -- Acumulación para promedio ponderado (Circular Mean)
        -- Solo considera lecturas significativas para el promedio
        if reading.value > 0.001 then -- Un umbral pequeño para evitar ruido
            sum_x_weighted = sum_x_weighted + (math.cos(angle_rad) * reading.value)
            sum_y_weighted = sum_y_weighted + (math.sin(angle_rad) * reading.value)
            total_weight = total_weight + reading.value
        end
    end

    if print_debug then
        log(" ---------------------------------------------------------------- ")
    end

    if Light.max_value < threshold then
        Light.LG = 1
    else
        Light.LG = 0 -- Reset LG si la luz es significativa
    end

    -- Calcular el ángulo promedio ponderado (media circular)
    if total_weight > 0 then
        -- atan2(y, x) retorna el ángulo en radianes en el rango (-pi, pi]
        Light.avg_angle = math.atan2(sum_y_weighted, sum_x_weighted)
    else
        Light.avg_angle = 0 -- No hay luz significativa para promediar
    end

    if active then
        local steering_amount = Light.avg_angle -- Esto está en radianes (-pi a pi)
        
        local left_speed  = 5.0 - steering_amount * 1.5 -- Ajusta el multiplicador para sensibilidad
        local right_speed = 5.0 + steering_amount * 1.5
        robot.wheels.set_velocity(left_speed, right_speed)
        TL_c1 = 1
    elseif TL_c1 == 1 then -- Solo apaga las ruedas si antes estaban activas por light-tracking
        robot.wheels.set_velocity(0, 0)
        TL_c1 = 0
    end

    if print_debug then
        log(string.format("Angulo promedio de la fuente de luz = %.2f°", Light.avg_angle * 180/math.pi))
        log(" ---------------------------------------------------------------- "..robot.id)
    end
    return Light
end


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

-- Envía un byte de datos por RAB en un canal dado, incluyendo el código del emisor en el canal 1
function sendRAB(message)
    local print    = message.print       or false                 -- message.channel     → canal donde envío el dato útil (2–10); default=2
    local ch       = message.channel     or 2                     -- message.channel     → canal donde envío el dato útil (2–10); default=2
    local value    = message.value       or 0                     -- message.value       → dato útil (0–255); default=0
    local sender_c = id_map[robot.id]    or 0                     -- message.sender_c    → código numérico del emisor (0–255); default=0
    robot.range_and_bearing.clear_data()
    robot.range_and_bearing.set_data(1, sender_c)                 -- canal 1 : id del emisor
    robot.range_and_bearing.set_data(ch, value)                   -- canal ch: dato
    
    if print then
        log("Message_sent: "..
            string.format("De %s →*  code = %d | canal = %d | valor = %d",
            robot.id, sender_c, ch, value)
        )
    end
end

-- Revisa todos los paquetes recibidos y los imprime,
-- reconstruyendo el nombre del emisor
function receiveRAB(p)
    local print        = p or false
    local packets      = robot.range_and_bearing
    for slot, pkt in ipairs(packets) do
        local sender_c = pkt.data[1] or 0
        local value    = pkt.data[2] or 0  -- asumimos que el dato siempre va en canal 2
        local sender   = reverse_map[sender_c] or ("id" .. sender_c)
        local dist     = pkt.range
        local angle    = pkt.horizontal_bearing * 180 / math.pi

        if print and sender_c ~= 0 then 
            log("message_received by "..robot.id..": "..
                string.format(
                "De = %s (code=%d) | dato = %d | dist = %.2fm | ang = %.2f°",
                sender, sender_c, value, dist/100, angle
                )
            )
        end
        return pkt
    end
end

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
function r_RandomWalking()
    RW_b2 = 0
    RW_b1 = 0
    RW_Counter = 0
    r_move()
end
function RandomWalking()

    proximidad()
    if proximity[1] > 0.0001 and RW_b1 == 0  then

        if proximity[2] >= 60 and  proximity[2] <= 120  then
        elseif  proximity[2] <= -60 and  proximity[2] >= -120 then
        else
            RW_b1 = 1
        end
        --log("Obstaculo detectado a "..proximity[1].." En un angulo de: "..proximity[2].." por: "..robot.id)
    end
    
    if RW_b1 == 0 then
        -- Generar un giro aleatorio ocasionalmente
        local random = robot.random.uniform_int(0, 60)
        if random >= 10 and random < 15 then
            robot.wheels.set_velocity(random, -random)
        elseif random >= 15 and random < 20 then
            robot.wheels.set_velocity(-random, random)
        elseif random >= 30 then
            robot.wheels.set_velocity(random/2, random/2)
        end
    end

    if RW_b1 == 1 then
        if proximity[2] < 60 and  proximity[2] > -60 and proximity[1] > 0.0001 then
            RW_b2 = 1
            --log("detectado en frente: "..proximity[2].." debo retroceder... "..robot.id)

        elseif proximity[2] < -120 or  proximity[2] > 120 then
           -- log("detectado atras: "..proximity[2].." debo avanzar... "..robot.id)
            RW_b2 = 2
        -- elseif proximity[2] >= 90 and  proximity[2] <= 120  or proximity[2] <= -90 and  proximity[2] >= -120 then
        -- --     -- log("---------- No hay problema, objeto ubicado a un lado.  ---------- ... "..robot.id)
        --         RW_b1 = 0
        --         RW_b2 = 0
        --     RW_Counter = 1
        end  
        --log("Girando..... "..robot.id)

        if RW_b2 == 1 then

            if RW_b3 == 0 then
                RW_Counter = RW_Counter + 1
                RW_b3 = 1
                --log("CAMBIO DE DIRECCION, retrocedi por obstáculo... "..robot.id)
            end

            move( -20, 0, 40,80)
            if c1 == 2 then
                RW_b1 = 2
                r_move()
                --log("Retrocedi por obstáculo... "..robot.id)
            end

        elseif RW_b2 == 2 then
            if RW_b3 == 1 then
                RW_Counter = RW_Counter + 1
                RW_b3 = 0
                --log("CAMBIO DE DIRECCION, avance por obstáculo... "..robot.id)
            end
            move(20, 0, 40)
            if c1 == 2 then
                RW_b1 = 2
                r_move()
                --log("Avance por obstáculo... "..robot.id)
            end
        end
    end

    if RW_b1 == 2 then
        if RW_Counter < 2 and RW_b2 == 1 then  
            move(0,90,30,80)   
        elseif RW_Counter < 2 and RW_b2 == 2 then  
            move(0,0,30,80)   
        else
            move(0,0,30,80)
            --move(0,90,30,80)
        end
        if c1 == 2 then
            --log("----------Colisión eviatada! ---------- ... "..robot.id)
  
            r_move()
            r_RandomWalking()
        end      
    end
end

RW_V1_c1        = 0
RW_V1_c2        = 0
RW_V1_c3        = 0
RW_V1_c4        = 0 --===========================
RW_V1_angle     = 0
RW_V1_distance  = 0
function RW_V1( r )
 
    if RW_V1_c3 == 0 then
           --log("RW_V1: ----------------")
        local G = ground{
            range = r
            -- , print_all_data = false,
            -- , print_detected_ranges = false
            -- , base_ground_active = false
        }
        if #G.Gr > 0 and RW_V1_c1 == 0 and RW_b1 == 0 then
            local D  = false
            local DD = false
            local DI = false
            local A  = false
            local AD = false
            local AI = false
            for _, pair in ipairs(G.Gr) do
                local sensor_detectado = pair[2]
                --log("sensor: "..pair[2])

                if sensor_detectado == 9  then 
                    DI = true
                end
                if sensor_detectado == 12 then 
                    DD = true
                end
                if sensor_detectado == 10 then 
                    AI = true
                end
                if sensor_detectado == 11 then 
                    AD = true
                end
            end

            --if RW_V1_c1 == 0 then
            --log(" -----------------------------------  "..robot.id)
            if AD and DD and AI and DI then

                -- if robot.id == "Footbot7" then
                --     log("RW_V1_c2 = "..RW_V1_c2)
                -- end

                if  RW_V1_c2 == 1 then
                    RW_V1_distance  = 100
                    RW_V1_angle     = -180
                   
                    RW_V1_c3 = 1

                    -- if robot.id == "Footbot7" then
                        --log(" tratando de volver XD_1"..robot.id)
                    -- end

                elseif RW_V1_c2 == 2 then
                    RW_V1_distance  = 100
                    RW_V1_angle     = -120

                    -- if robot.id == "Footbot7" then
                        --log(" tratando de volver XD_2"..robot.id)
                    -- end

                elseif RW_V1_c2 > 2 then
                    RW_V1_c1 = 0
                    RW_V1_c2 = 0

                    RW_V1_c3 = 1

                    --RW_V1_c3 = 0
                -- if robot.id == "Footbot7" then
                        -- log(" -----------------------------------------------------------------  ")
                        -- log(" Sali de la colmena"..robot.id)
                        -- log(" -----------------------------------------------------------------  ")
                        return
                    --end
                else
                    RW_V1_angle = -180
                    RW_V1_distance  = 50
                    -- if robot.id == "Footbot7" then
                        --log(" tratando de volver"..robot.id)
                    -- end
                end
                --log(" AD and DD and AI and DI ")
            elseif AI and DI then
                RW_V1_angle     = -90
                RW_V1_distance  =  15
                --log(" AI and DI ")
            elseif AD and DD then
                RW_V1_angle     =  90
                RW_V1_distance  =  15
                --log(" AD and DD ")
            elseif DI and DD then 
                RW_V1_angle     = 180
                RW_V1_distance  =   5 
                --log(" DI and DD ")
            elseif AI and AD then
                RW_V1_angle     =   0
                RW_V1_distance  =  10
                
                --log(" AI and AD ")
            elseif AI then
                RW_V1_angle     = -45
                RW_V1_distance  =  10  
                --log(" AI ") 
            elseif AD then
                RW_V1_angle     =  45
                RW_V1_distance  =  10
                --log(" AD ")
            elseif DI then
                RW_V1_angle     =-135
                RW_V1_distance  =  10 
                --log(" DI ")
            elseif DD then
                RW_V1_angle     = 135
                RW_V1_distance  =  10
                --log(" DD ")
            end
 
            RW_V1_c1 = 1
            r_move()

        elseif RW_V1_c1 == 1 and RW_b1 == 0 then

            move(RW_V1_distance, RW_V1_angle,10,80,false)
            if c1 == 2 then
                RW_V1_c2    = RW_V1_c2 + 1
                RW_V1_c4    = RW_V1_c4 + 1
                r_move()
                r_RandomWalking()
                RW_V1_c1    = 0
                -- if robot.id == "Footbot7" then
                --     log("Fin de movimiento: "..robot.id.." , RW_V1_c2 = "..RW_V1_c2)
                -- end
            end
        else
            RandomWalking()
            RW_V1_c2 = 0
        end
    else

         --log("RW_V1: ----------------")
        local G = ground{
            range = {
                {0.21, 0.3 },   -- gris colmena
            }
             , print_all_data = false,
            -- , print_detected_ranges = false
            -- , base_ground_active = false
        }
        if #G.Gr > 1 then
            RW_V1_c3 = 0
            robot.wheels.set_velocity(100,100)
            -- log(" -----------------------------------------------------------------  ")
            -- log(" Volvi de la colmena"..robot.id)
            -- log(" -----------------------------------------------------------------  ")
            r_move()
            return
        end
        navegar_al_centro_por_coordenadas()
        --RandomWalking()
    end
end

-- =========================================================================
-- Función auxiliar para obtener el signo de un número
-- =========================================================================
function sign(x)
    if x > 0 then
        return 1
    elseif x < 0 then
        return -1
    else
        return 0
    end
end
-- La función vec2_angle no se modifica ni se usa en esta parte.
-- La lógica para convertir cuaterniones se ha integrado directamente.
function navegar_al_centro_por_coordenadas()
    -- Obtener la posición actual del robot
    local current_x = robot.positioning.position.x
    local current_y = robot.positioning.position.y

    -- Obtener la orientación actual del robot (cuaternión)
    local orientation_quat = robot.positioning.orientation

    -- Definir el centro como objetivo
    local target_x = 0.0
    local target_y = 0.0

    -- Parámetros de movimiento
    local speed_straight = 50
    local speed_turn_factor = 100 -- Factor para ajustar la velocidad de giro (ajustable)
    local max_wheel_speed = 100 -- Velocidad máxima de las ruedas

    -- --- Lógica de Movimiento ---

    -- 1. Calcular el ángulo al centro usando math.atan2 con coordenadas
    local angle_to_center = math.atan2(target_y - current_y, target_x - current_x)

    -- 2. Convertir el cuaternión de orientación del robot a un ángulo en Z (radianes)
    local orientation_angle = math.atan2(
        2 * (orientation_quat.w * orientation_quat.z + orientation_quat.x * orientation_quat.y),
        1 - 2 * (orientation_quat.y ^ 2 + orientation_quat.z ^ 2)
    )

    -- 3. Calcular la diferencia de ángulo y normalizarla
    local angle_difference = angle_to_center - orientation_angle
    angle_difference = math.atan2(math.sin(angle_difference), math.cos(angle_difference))

    -- 4. Tomar la decisión de movimiento
    -- Si el robot está casi alineado, avanza.
    if math.abs(angle_difference) < 0.1 then
        robot.wheels.set_velocity(speed_straight, speed_straight)
    else
        -- Calcular la velocidad de giro proporcional a la diferencia de ángulo
        local turn_speed = speed_turn_factor * angle_difference
        
        -- Ajustar las velocidades de las ruedas para girar
        local vL = -turn_speed
        local vR = turn_speed
        
        -- Limitar las velocidades para que no excedan el máximo
        if math.abs(vL) > max_wheel_speed then vL = max_wheel_speed * sign(vL) end
        if math.abs(vR) > max_wheel_speed then vR = max_wheel_speed * sign(vR) end

        robot.wheels.set_velocity(vL, vR)
    end
end

SC_c1           = 0
SC_c2           = 0
SC_end          = false
SC_angle        = 0
SC_offset_angle = 0
function sun_compass(SC_offset_angle)
    local SC_offset_angle = SC_offset_angle or 0
    local tr_l = track_light{print = false}
    if not SC_end then
        if     SC_c1 == 0 then
            if tr_l.avg_angle * 180/math.pi ~= 0 then 
                SC_angle = tr_l.avg_angle * 180/math.pi - SC_offset_angle
                SC_c1 = 1
                r_move()
                --if robot.id == "Footbot4" then
                    --log("| Angulo respecto al Sol: "..tr_l.avg_angle * 180/math.pi.."° | Angulo deseado: "..SC_offset_angle.." | Angulo a recorrer: "..SC_angle.."° | "..robot.id)
                --end
            else
                robot.wheels.set_velocity(0,0)   -- Detener
                --log("Esperando a ver el sol "..robot.id)
            end
        elseif SC_c1 == 2 then 

            if math.abs( SC_offset_angle - tr_l.avg_angle * 180/math.pi) < 1  then
                --log("Mirando al sol; "..tr_l.avg_angle * 180/math.pi..", "..robot.id)
                SC_c1 = 0
                SC_end = true
                r_move()
                return
            else
                SC_c1 = 0
                -- if robot.id == "Footbot4" then
                    --log("CRRECCIÓN: | Angulo respecto al Sol: "..tr_l.avg_angle * 180/math.pi.."° | Angulo deseado: "..SC_offset_angle.." | "..robot.id)
                -- end
            end
        elseif SC_c1 == 1 then 
            move(0 , SC_angle , nill , 10 , false )
            if c1 == 2 then 
                r_move()
                SC_c1 = 2
            end
        end
    end
end

-- Generar Espiral para explorar areas cernas a la ubicación obtenida

-- Variables de configuración de la espiral (ahora más orientadas al radio y velocidad lineal)
spiral_base_linear_speed = 0.1      -- Velocidad lineal promedio deseada (m/s)
spiral_initial_radius = 0.1        -- Radio inicial de la espiral (metros). No 0 para evitar divisiones
spiral_radius_increment = 0.0005     -- Cuánto aumenta el radio por tick (metros/tick)
spiral_max_radius = 2.0             -- Radio máximo para la espiral (metros)

-- Variables de estado de la espiral (DEBEN SER LOCALES AL SCRIPT si quieres que persistan entre llamadas)
spiral_current_radius = 0.0   -- Se inicializará en spiral_initialized = true
spiral_initialized = false    -- Flag para inicializar la espiral

ROBOT_WIDTH = 0.14                          -- Ancho del robot (distancia entre las ruedas) en METROS.  (14 cm) 
DELTA_TIME = 1/ticks_per_second             -- Tiempo que representa cada "tick" de la simulación en SEGUNDOS.

-- Posición actual del robot (inicialmente en el origen) en METROS -- También DEBEN SER LOCALES AL SCRIPT
robot_x = 0.0
robot_y = 0.0

-- Orientación actual del robot en radianes (inicialmente 0 grados, apuntando al eje X positivo) -- También DEBE SER LOCAL AL SCRIPT
SE_angle_orientation     = 0.0

-- Variables de salida (también deberían ser locales al script si quieres que persistan para ser leídas por otras funciones)
SE_angle_orientation_deg = 0.0
SE_angle     = 0.0
SE_angle_deg = 0.0
SE_distance  = 0.0
SE_value     = 0.0
SE_source    = false

function spiral_exploration()

    if scout then
        -- Esto tampoco sería relevante directamente para el radio, pero puedes usarlo para ajustar la velocidad lineal o el incremento de radio
        spiral_base_linear_speed = 0.5  -- Ejemplo: un poco más rápido
        spiral_radius_increment = 0.001 --  Ejemplo: se abre un poco más rápido
        spiral_max_radius = 2 -- Esto puede ya no ser relevante con la nueva lógica de radio
    else
        -- Ajustes para el caso no "scout"
        spiral_base_linear_speed = 0.5  -- Ejemplo: un poco más rápido
        spiral_radius_increment = 0.0005 -- Ejemplo: se abre un poco más lento
        spiral_max_radius = 1.5 -- Esto puede ya no ser relevante con la nueva lógica de radio 
    end 
    
    -- Inicializar la espiral la primera vez que se llama a la función
    if not spiral_initialized then
        -- Reiniciar las variables de posición y orientación para cada nueva espiral
        robot_x = 0.0
        robot_y = 0.0
        SE_angle_orientation = 0.0
        
        spiral_current_radius = spiral_initial_radius -- Reinicia el radio al iniciar la espiral
        spiral_initialized = true
        -- log(string.format("[%s] Espiral inicializada. Posición: (%.2f, %.2f) Orientación: %.2f°, Radio inicial: %.2f m",
        --     robot.id, robot_x, robot_y, SE_angle_orientation * 180 / math.pi, spiral_current_radius))
        -- Al inicializar, imprimimos el punto de partida (0,0)
        -- Puedes incluso hacer un primer movimiento mínimo aquí si quieres.
        return -- Salimos para que el primer tick de movimiento sea en la siguiente llamada
    end

    local G = ground{
        range = {
            {0.71, 0.8 },   -- gris 7
            {0.51, 0.6 },   -- gris 5
            {0.31, 0.4 },   -- gris 3
            {0.0 , 0.1 },   -- negro
            }                
        , print_all_data        = false
        , print_detected_ranges = false
        -- base_ground_active = false
    }
    
    if #G.Gr > 0 then
        SE_value = G.Gr[1][1] 
        -- if SE_value == 1 then 
        --     SE_value = 0
        -- end
        G1 = 2
        spiral_initialized = false -- Reinicia la bandera para la próxima vez
        SE_source = true
        robot.wheels.set_velocity(0,0)
        -- log(" --------------------------------------------------------------------------------------------------------------- ")
        -- log("Reporte -> scout Bee: "..robot.id)
        -- log( "Fuente de alimento  encontrada | value = "..SE_value.." | ángulo = "..SE_angle_deg.."° | Distancia = "..SE_distance.." m | Orientación Robot: "..SE_angle_orientation_deg.."°")
        -- log(" --------------------------------------------------------------------------------------------------------------- ")   
        return
    end

    -- Incrementar el radio de la espiral
    spiral_current_radius = spiral_current_radius + spiral_radius_increment
    -- Asegurarse de que el radio no exceda el máximo
    spiral_current_radius = math.min(spiral_current_radius, spiral_max_radius)

    -- Si el radio alcanza el límite, detener el robot
    if spiral_current_radius >= spiral_max_radius then
        G1 = 2
        SE_source = false
        spiral_initialized = false -- Reinicia la bandera para la próxima vez
        robot.wheels.set_velocity(0, 0)
        -- log(" --------------------------------------------------------------------------------------------------------------- ")
        -- log("Reporte -> scout Bee: "..robot.id)
        -- log( "NO se encontraron fuentes de alimento cerca (radio máximo alcanzado) | value = "..SE_value.." | ángulo = "..SE_angle_deg.."° | Distancia = "..SE_distance.." m | Orientación Robot: "..SE_angle_orientation_deg.."°")
        -- log(" --------------------------------------------------------------------------------------------------------------- ")   
        return
    end

    -- Calcular las velocidades de las ruedas en m/s
    local v_linear = spiral_base_linear_speed -- m/s
    local R_curvatura = spiral_current_radius -- m

    -- Evitar división por cero si el radio es muy pequeño
    if R_curvatura < 0.001 then -- Un umbral pequeño para evitar divisiones por cero
        R_curvatura = 0.001 -- Asegura que siempre haya un radio mínimo para calcular
    end

    -- Fórmulas para velocidades de ruedas en función del radio de curvatura
    -- Ajusta el signo si quieres cambiar la dirección de giro (horario vs antihorario)
    local v_right_mps = v_linear + (ROBOT_WIDTH / 2) * (v_linear / R_curvatura)
    local v_left_mps  = v_linear - (ROBOT_WIDTH / 2) * (v_linear / R_curvatura)

    -- Convertir de m/s a cm/s para set_velocity
    local v_right_cms = v_right_mps * 100
    local v_left_cms  = v_left_mps * 100

    -- Aplicar las velocidades a las ruedas
    robot.wheels.set_velocity(v_left_cms, v_right_cms)

    -- === CINEMÁTICA DIRECTA: Calcular la nueva posición y orientación ===
    -- Calcular la velocidad lineal y angular del robot
    local v_linear_actual = (v_right_mps + v_left_mps) / 2 -- m/s (la velocidad lineal real aplicada)
    local v_angular = (v_right_mps - v_left_mps) / ROBOT_WIDTH -- Radianes por segundo

    -- Calcular el cambio en la orientación (delta_theta)
    local delta_theta = v_angular * DELTA_TIME

    -- Calcular el cambio en la posición (delta_x, delta_y)
    local delta_x
    local delta_y

    -- Manejo especial para movimiento recto (v_angular es cero o muy cercano a cero)
    if math.abs(v_angular) < 0.0001 then -- Si el robot va prácticamente recto
        delta_x = v_linear_actual * DELTA_TIME * math.cos(SE_angle_orientation)
        delta_y = v_linear_actual * DELTA_TIME * math.sin(SE_angle_orientation)
    else -- Si el robot está girando
        -- Radio de curvatura instantáneo
        local R = v_linear_actual / v_angular

        -- Centro de rotación instantáneo (ICC)
        local icc_x = robot_x - R * math.sin(SE_angle_orientation)
        local icc_y = robot_y + R * math.cos(SE_angle_orientation)

        -- Traslación y rotación alrededor del ICC
        delta_x = math.cos(delta_theta) * (robot_x - icc_x) - math.sin(delta_theta) * (robot_y - icc_y) + icc_x - robot_x
        delta_y = math.sin(delta_theta) * (robot_x - icc_x) + math.cos(delta_theta) * (robot_y - icc_y) + icc_y - robot_y
    end

    -- Actualizar la posición y orientación del robot
    robot_x = robot_x + delta_x
    robot_y = robot_y + delta_y
    SE_angle_orientation = SE_angle_orientation + delta_theta -- Asegúrate de que el signo sea consistente con tu ICC

    -- Normalizar el ángulo SE_angle_orientation para mantenerlo en el rango [-pi, pi]
    SE_angle_orientation = math.atan2(math.sin(SE_angle_orientation), math.cos(SE_angle_orientation))

    -- === CALCULAR Y ALMACENAR LAS VARIABLES DE SALIDA ===
    -- Distancia desde el origen (0,0)
    SE_distance = math.sqrt(robot_x*robot_x + robot_y*robot_y)
    
    -- Ángulo de la posición del robot respecto al origen (en radianes)
    SE_angle = math.atan2(robot_y, robot_x)

    -- Convertir a grados para facilitar la lectura o uso posterior
    SE_angle_orientation_deg = SE_angle_orientation * 180 / math.pi
    SE_angle_deg = SE_angle * 180 / math.pi

    -- === IMPRIMIR COORDENADAS Y OTROS DATOS PARA DEPURACIÓN ===
    -- log(string.format("[%s] Posición: X=%.2f m, Y=%.2f m, Distancia=%.2f m, ÁnguloPos=%.2f°, OrientaciónRobot=%.2f°",
    --    robot.id, robot_x, robot_y, SE_distance, SE_angle_deg, SE_angle_orientation_deg))

    -- log(string.format("[%s] Espiral: L_Vel=%.2f cm/s, R_Vel=%.2f cm/s | CurrRadius=%.2f m",
    -- robot.id, v_left_cms, v_right_cms, spiral_current_radius))

    -- === OBTENER E IMPRIMIR EL ÁNGULO DE LUZ EN CADA TICK  ===
    local light_reading = track_light{ active = false }
    local current_light_angle_rad = light_reading.avg_angle
    local current_light_angle_deg = current_light_angle_rad * 180 / math.pi
    local max_angle_deg = light_reading.max_angle * 180 / math.pi

    -- Para depuración: Mostrar las velocidades aplicadas y el ángulo de luz en cada tick
    -- log(string.format("[%s] Light_Angle=%.2f° | max_angle=%.2f°|",
    -- robot.id, current_light_angle_deg,max_angle_deg))
end

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
                        if imprimir then 
                            log(" theta ".. move_angle *180/math.pi .." , "..robot.id)
                        end
                    else
                        robot.wheels.set_velocity(wheel_speed, -wheel_speed)
                        move_angle = -(1 / ticks_per_second) * actual_ang_ticks * omega_rad
                        if imprimir then 
                            log(" theta ".. move_angle *180/math.pi .." , "..robot.id)
                        end
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

wh_role = 0
scout   = false
function determinar_rol()

    if robot.id == "Footbot00" or robot.id == "Footbot44"  then 
        wh_role = 0               
        scout   = true                                         --forager 
        log("The " .. robot.id .. " is meatro")
    else
        wh_role = 1                                                        --Alumno
        log("The " .. robot.id .. " is Alumno")
    end

    return wh_role
end

function resetOBS()
    MUO__distance_an = 40
    MUO__angle_an = 0
    MUO__c1 = 0
    MUO__c2 = 0

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
    srt_ang_mark_ant    = 0
    x_mark, y_mark, x_new_r, x_ant_r, y_new_r, y_ant_r = 0, 0, 0, 0, 0 ,0 

    srt_angle_an        = 0

    motion_markers      = {}           -- Correcciones registradas
    last_motion_marker  = {0, 0}   -- Total correccion realizada
    motion_tick         = 0               -- Tick actual durante movimiento
    motion_active = false         -- Indica si hay un movimiento en curso
    motion_target_ticks = 0       -- Total de ticks necesarios
    motion_cm_tick      = 0   
    motion_dist         = 0   

    srt_end             = 0

    -- ============================
    coordenadasx    = {}
    coordenadasy    = {}
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
    params                = {}

    posiciones_guardadas = {}
    
    indice        = 1

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
    PO_end = false
    -- ============================¿
    setleds({ color = "red" })
    r_Temporizador_1()
    r_Temporizador_2()
    r_move()

end

function resetforager(opts)
    to_explorer = opts.to_explorer or false
    to_waggle   = opts.to_waggle  or false

    f1 = 0
    -- ================================
    e1                  = 0
    DN_angle            = 0
    DN_light_angle      = 0
    DN_c1               = 0
    DN_c2               = 0 
    DN_average_angel    = 0
    -- ================================
    GTS_c1              = 0
    GTS_c2              = 0
    GTS_c3              = 0
    GTS_val             = 0

    GTS_counter         = 0
    alumno_counter      = 0

    angle_food_source   = 0
    distance_food_source= 0
    -- ================================

    w1      = 0
    WD_c1   = 0
    vL      = 0
    vR      = 0
    ticks   = 0
    -- ================================

    G1                  = 0
    EAC_sun_angle_ant   = 0
    EAC_angle           = 0 
    EAC_ang_direction   = 0 
    --exploracion = false --=========

    SC_c1           = 0
    SC_end          = false
    SC_angle        = 0
    SC_offset_angle = 0
    -- ================================

    spiral_base_linear_speed= 0.1      
    spiral_initial_radius   = 0.1        
    spiral_radius_increment = 0.0005    
    spiral_max_radius       = 2.0        

    spiral_current_radius   = 0.0  
    spiral_initialized      = false   

    robot_x                 = 0.0
    robot_y                 = 0.0

    SE_angle_orientation    = 0.0

    SE_angle_orientation_deg= 0.0
    SE_angle                = 0.0
    SE_angle_deg            = 0.0
    SE_distance             = 0.0
    SE_value                = 0.0
    SE_source               = false
    -- ================================

    resetDEM()
    if to_explorer then
        GTS_params  = {}
        setleds("orange")
    elseif to_waggle then
        f1          = 1
        --setleds{color = "magenta"}
        setleds({ color = "magenta", only = {2, 3, 6, 8, 10, 12} })
    end
end

function resetalumno()
    a1                  = 0
    -- ================================
    e1                  = 0
    DN_angle            = 0
    DN_light_angle      = 0
    DN_c1               = 0
    DN_c2               = 0 
    DN_average_angel    = 0

    -- ================================
    GTS_c1              = 0
    GTS_c2              = 0
    GTS_c3              = 0
    GTS_val             = 0

    GTS_counter         = 0
    alumno_counter      = 0

    angle_food_source   = 0
    distance_food_source= 0

    -- ================================
    candidate_updated_flag = false

    -- ================================
    G1                  = 0
    EAC_sun_angle_ant   = 0
    EAC_angle           = 0 
    EAC_ang_direction   = 0 
    --exploracion = false --=========

    SC_c1           = 0
    SC_end          = false
    SC_angle        = 0
    SC_offset_angle = 0
    -- ================================

    spiral_base_linear_speed= 0.1      
    spiral_initial_radius   = 0.1        
    spiral_radius_increment = 0.0005    
    spiral_max_radius       = 2.0        

    spiral_current_radius   = 0.0  
    spiral_initialized      = false   

    robot_x                 = 0.0
    robot_y                 = 0.0

    SE_angle_orientation    = 0.0

    SE_angle_orientation_deg= 0.0
    SE_angle                = 0.0
    SE_angle_deg            = 0.0
    SE_distance             = 0.0
    SE_value                = 0.0
    SE_source               = false
    -- ================================

    resetOBS()
    if wh_role == 1 then
        GTS_params  = {}
    end
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
-- Todos en azul    -- Todos en RGB naranja     -- Solo LED 13(Direccional) en rojo -- Solo LEDs 3 a 6 en verde                     -- Solo los LEDs 1, 4, 7 en magenta
-- setleds("blue")  -- setleds({255, 165, 0})   -- setleds({ color = "red" })       -- setleds({ color = "green", range = {3, 6} }) -- setleds({ color = "magenta", only = {1, 4, 7} })

function setleds(config)
   -- Caso 1: Si es string → color por nombre para todos
   if type(config) == "string" then
      for i = 1, 13 do
         robot.leds.set_single_color(i, config)
      end
      return
   end

   -- Caso 2: Si es tabla con 3 elementos → RGB
   if type(config) == "table" and #config == 3 then
      for i = 1, 13 do
         robot.leds.set_single_color(i, config)
      end
      return
   end

   -- Caso 3: Tabla avanzada con configuración personalizada
   local color = config.color or "white"
   local range = config.range
   local only  = config.only

   -- Apagar todos primero
   for i = 1, 13 do
      --robot.leds.set_single_color(i, "black")
   end

   -- Encender LEDs por índice directo
   if only then
      for _, idx in ipairs(only) do
         if idx >= 1 and idx <= 13 then
            robot.leds.set_single_color(idx, color)
         end
      end

   -- Encender LEDs por rango
   elseif range then
      local from_i = range[1] or 1
      local to_i   = range[2] or 13
      for i = from_i, to_i do
         if i >= 1 and i <= 13 then
            robot.leds.set_single_color(i, color)
         end
      end

   -- Si no se especifica nada más, encender solo el LED frontal (13)
   else
      robot.leds.set_single_color(13, color)
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

function imprimir_parametros(params)
    log("----- Tabla de Parámetros -----"..robot.id)
    for i, fila in ipairs(params) do
        local distancia = fila[1]
        local angulo = fila[2]
        log(string.format("Fila %d: Distancia = %.2f, Ángulo = %.2f°", i, distancia, angulo))
    end
    log("----------- Fin ---------------")
end


------------------------- Funciones Para el manejo de "SOURCE " ------------------------------------------

function print_single_source_data_nested_params(source_obj)
    log("==========================================")
    -- Ajustado para ser más general, ya que 'robot.id' puede no estar disponible o ser el contexto
    log("Reporte de Detecciones para Fuente ID: " .. tostring(source_obj.id or "Desconocido")) 
    log("==========================================")

    local data_table = source_obj.data
    local params_sets = source_obj.params -- Esto es una lista de los objetos GTS_params, donde cada GTS_params es una tabla como {{dist, ang}, {dist, ang}}

    -- --- Imprimir Datos de Detección (angle, distance, value) ---
    if data_table and #data_table.angle > 0 then
        log("--- Datos de Detección (Observaciones) ---")
        for i = 1, #data_table.angle do
            log(string.format("  Detección #%d:", i))
            log(string.format("    Ángulo:    %.2f°", data_table.angle[i]))
            log(string.format("    Distancia: %.2f cm", data_table.distance[i]))
            log(string.format("    Valor:     %.2f", data_table.value[i]))
        end
        log("------------------------------------------")
    else
        log("No hay datos de detección registrados.")
        log("------------------------------------------")
    end

    -- --- Imprimir Datos de Parámetros ---
    if params_sets and #params_sets > 0 then
        log("--- Conjuntos de Parámetros de Ruta ---") -- Manteniendo el nombre, pero recuerda que son los sets de pares
        for i = 1, #params_sets do
            log(string.format("  Conjunto de Parámetros #%d:", i))
            local current_gts_params_set = params_sets[i] -- Esto es la tabla como {{5, 10}, {10, 20}}

            if current_gts_params_set and type(current_gts_params_set) == "table" then
                -- Ahora iteramos sobre los pares {dist, ang} dentro de este conjunto
                for j = 1, #current_gts_params_set do -- Aquí current_gts_params_set tiene elementos numéricos
                    local param_pair = current_gts_params_set[j] -- Esto es un par como {5, 10}
                    if param_pair and type(param_pair) == "table" and #param_pair >= 2 then
                        log(string.format("    Segmento %d: (Dist: %.2f, Ang: %.2f)", j, param_pair[1], param_pair[2]))
                    else
                        log(string.format("    Segmento %d: Datos inválidos o incompletos", j))
                    end
                end
            else
                log("  Conjunto de parámetros inválido o no es una tabla.")
            end
        end
        log("------------------------------------------")
    else
        log("No hay conjuntos de parámetros de ruta registrados.")
        log("------------------------------------------")
    end

    log("==========================================")
end
 
-- Nueva función para imprimir TODAS las fuentes (se mantiene igual)
function print_all_sources_data(sources_table)
    if not sources_table or #sources_table == 0 then
        log("--- No hay fuentes registradas en el sistema ---")
        return
    end

    log("\n\n############################################"..robot.id)
    log("### REPORTE COMPLETO DE TODAS LAS FUENTES ###")
    log("############################################"..robot.id)

    for i = 1, #sources_table do
        log(string.format(">>> Procesando Fuente #%d de %d <<<", i, #sources_table))
        print_single_source_data_nested_params(sources_table[i])
        if i < #sources_table then
            log("\n--------------------------------------------\n") -- Separador entre fuentes
        end
    end
    log("\n############################################\n\n")
end

function sort_and_print_sources(options)
    local order_by_param = options.ordenar or "value" -- Default a "value" si no se especifica
    local print_mode     = options.imprimir or "all"   -- Default a "all" si no se especifica

    if #source == 0 then
        log("[t="..timer.."]["..robot.id.."] No hay fuentes para ordenar o imprimir. Retornando nil.")
        return nil -- Retorna nil si no hay fuentes
    end

    -- Realizar el ordenamiento de la tabla 'source'
    table.sort(source, function(a, b)
        -- Obtener los valores de comparación para a y b
        local val_a_main = get_comparison_value(a, order_by_param)
        local val_b_main = get_comparison_value(b, order_by_param)

        -- Prioridad 1: Por el parámetro principal (value, distance, o angle)
        if order_by_param == "value" then
            if val_a_main ~= val_b_main then
                return val_a_main > val_b_main -- Mayor 'value' primero
            end
        elseif order_by_param == "distance" then
            if val_a_main ~= val_b_main then
                return val_a_main < val_b_main -- Menor 'distance' primero
            end
        elseif order_by_param == "angle" then
            if val_a_main ~= val_b_main then
                return val_a_main < val_b_main -- Menor ABSOLUTO 'angle' (más cercano a 0) primero
            end
        end

        -- Prioridad 2: Si los valores principales son iguales, ordenar por 'distance' (la más pequeña)
        local val_a_dist = get_comparison_value(a, "distance")
        local val_b_dist = get_comparison_value(b, "distance")
        if val_a_dist ~= val_b_dist then
            return val_a_dist < val_b_dist -- Menor 'distance' primero
        end

        -- Prioridad 3: Si 'value' y 'distance' son iguales, ordenar por 'angle' (más cercano a 0)
        local val_a_angle = get_comparison_value(a, "angle")
        local val_b_angle = get_comparison_value(b, "angle")
        return val_a_angle < val_b_angle -- Menor ABSOLUTO 'angle' primero
    end)

    -- La "mejor" fuente siempre estará en la primera posición después de ordenar
    local best_source = source[1]

    -- Imprimir según el modo especificado
    if print_mode == "all" then
        print_all_sources_data(source) -- Reutilizamos tu función existente
    elseif print_mode == "best" then
        log(" ------------------------------------------- "..robot.id)
        log("### REPORTE DE LA MEJOR FUENTE ("..order_by_param..") ###")
        log(" ------------------------------------------- "..robot.id)
        print_single_source_data_nested_params(best_source) 
        log("\n############################################\n\n")
    elseif print_mode ~= "none" then -- Manejar modos de impresión no reconocidos
        log("[t="..timer.."]["..robot.id.."] Modo de impresión no reconocido: '"..print_mode.."'. No se imprimirá nada.")
    end

    return best_source -- Retorna la mejor fuente
end

-- Función para añadir una NUEVA fuente al sistema.
function add_new_source(source_id)
    local new_source_entry = {
        id = source_id,
        data = {
            angle    = {}, -- Inicializado como tabla vacía (para contener {numero})
            distance = {}, -- Inicializado como tabla vacía (para contener {numero})
            value    = {}, -- Inicializado como tabla vacía (para contener {numero})
        },
        params = {} -- Inicializado como tabla vacía (para contener {{secuencia1}, {secuencia2}, ...})
    }
    table.insert(source, new_source_entry)
    return new_source_entry
end

-- Función para ELIMINAR una fuente de la tabla 'source' basándose en su 'id'.
-- Si existen múltiples entradas con el mismo ID, eliminará todas.
function delete_source_by_id(source_id_to_delete)
    -- Recorremos la tabla 'source' desde el final hacia el principio.
    -- Hacerlo así evita problemas con los índices cuando se eliminan elementos:
    -- si eliminas un elemento, los índices de los siguientes cambian,
    -- y recorrer hacia adelante podría hacer que te saltes elementos o accedas a índices inválidos.
    for i = #source, 1, -1 do
        -- Verificamos si la entrada en el índice 'i' existe y si su 'id' coincide.
        if source[i] and source[i].id == source_id_to_delete then
            -- Si coincide, eliminamos esa entrada del array 'source'.
            table.remove(source, i)
            -- log(string.format("[DEBUG] Eliminada fuente con ID: %d en índice %d.", source_id_to_delete, i))
        end
    end
    -- log(string.format("[DEBUG] Finalizada operación de eliminación para ID: %d.", source_id_to_delete))
end

LOCAL_VALUE_TOLERANCE = 1
LOCAL_DISTANCE_TOLERANCE = 100
LOCAL_ANGLE_TOLERANCE = 15
function is_similar_source(source1, source2)
    if not source1 or not source2 then return false end

    local val1 = source1.data.value[1] or 0
    local val2 = source2.data.value[1] or 0
    local dist1 = source1.data.distance[1] or 0
    local dist2 = source2.data.distance[1] or 0
    local angle1 = source1.data.angle[1] or 0
    local angle2 = source2.data.angle[1] or 0

    local value_diff = math.abs(val1 - val2)
    local distance_diff = math.abs(dist1 - dist2)
    local angle_diff = math.abs(angle1 - angle2)

    -- >>> AÑADE ESTAS LÍNEAS DE DEBUGGING <<<
    if robot.id == "Footbot8" then 
        -- log("--- DEBUG is_similar_source ---")
        -- log(string.format("  Fuente 1 (ID: %s): Val=%.5f, Dist=%.5f, Ang=%.5f", tostring(source1.id), val1, dist1, angle1))
        -- log(string.format("  Fuente 2 (ID: %s): Val=%.5f, Dist=%.5f, Ang=%.5f", tostring(source2.id), val2, dist2, angle2))
        -- log(string.format("  Diferencias: Val=%.5f (Tol: %.2f), Dist=%.5f (Tol: %.2f), Ang=%.5f (Tol: %.2f)",
        --                 value_diff, LOCAL_VALUE_TOLERANCE,
        --                 distance_diff, LOCAL_DISTANCE_TOLERANCE,
        --                 angle_diff, LOCAL_ANGLE_TOLERANCE))
        -- log(string.format("  Resultado similar: %s", tostring(value_diff <= LOCAL_VALUE_TOLERANCE and
        --                                                     distance_diff <= LOCAL_DISTANCE_TOLERANCE and
        --                                                     angle_diff <= LOCAL_ANGLE_TOLERANCE)))
        -- log("----------------------------------")
        -- >>> FIN DE LÍNEAS DE DEBUGGING <<<
    end

    return value_diff <= LOCAL_VALUE_TOLERANCE and
           distance_diff <= LOCAL_DISTANCE_TOLERANCE and
           angle_diff <= LOCAL_ANGLE_TOLERANCE
end

-- Función auxiliar para obtener el valor de comparación 
function get_comparison_value(source_obj, type)
    if not source_obj or not source_obj.data then
        if type == "value" then return -math.huge end
        if type == "distance" then return math.huge end
        if type == "angle" then return math.huge end
        return nil
    end

    -- Accedemos al valor en el índice [1] de la tabla, como se espera.
    if type == "value" then
        return source_obj.data.value[1] or -math.huge
    elseif type == "distance" then
        return source_obj.data.distance[1] or math.huge
    elseif type == "angle" then
        return math.abs(source_obj.data.angle[1] or math.huge)
    end
    return nil
end
-- Función para comparar si `source_candidate` es mejor que `current_best`
function is_source_better(source_candidate, current_best)
    if not current_best then return true end

    local val_cand = get_comparison_value(source_candidate, "value")
    local val_curr = get_comparison_value(current_best, "value")
    if val_cand > val_curr then return true end
    if val_cand < val_curr then return false end

    local dist_cand = get_comparison_value(source_candidate, "distance")
    local dist_curr = get_comparison_value(current_best, "distance")
    if dist_cand < dist_curr then return true end
    if dist_cand > dist_curr then return false end

    local ang_cand = get_comparison_value(source_candidate, "angle")
    local ang_curr = get_comparison_value(current_best, "angle")
    if ang_cand < ang_curr then return true end

    return false
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

function calculateTriangleProperties_SAS(dist_AC, dist_CB, angle_C_degrees)
    local results = {}

    -- Convertir el ángulo C a radianes para los cálculos de Lua
    local angle_C_rad = math.rad(angle_C_degrees)

    -- Distancias conocidas
    local b = dist_AC -- Lado opuesto al ángulo B
    local a = dist_CB -- Lado opuesto al ángulo A

    -- 1. Calcular la distancia AB (lado c) usando la Ley de los Cosenos
    -- c^2 = a^2 + b^2 - 2ab * cos(C)
    local c_squared = a^2 + b^2 - 2 * a * b * math.cos(angle_C_rad)
    local c = math.sqrt(c_squared)
    local dist_AB = c

    -- 2. Calcular los ángulos restantes usando la Ley de los Cosenos (más robusto para ángulos)
    -- Para el ángulo A: a^2 = b^2 + c^2 - 2bc * cos(A)
    local cos_A = (b^2 + c^2 - a^2) / (2 * b * c)
    local angle_A_rad = math.acos(cos_A)

    -- Para el ángulo B: b^2 = a^2 + c^2 - 2ac * cos(B)
    local cos_B = (a^2 + c^2 - b^2) / (2 * a * c)
    local angle_B_rad = math.acos(cos_B)

    -- Almacenar los resultados de los ángulos (en grados)
    results.angle = {}
    results.angle[1] = math.deg(angle_A_rad) -- Ángulo A (entre AB y AC)
    results.angle[2] = math.deg(angle_B_rad) -- Ángulo B (entre CB y AB)
    results.angle[3] = angle_C_degrees -- Ángulo C (entre AC y CB)

    -- Almacenar las distancias
    results.distance = {}
    results.distance[1] = dist_AC
    results.distance[2] = dist_CB
    results.distance[3] = dist_AB

    return results
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