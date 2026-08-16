--=============================================
--   MEJORAS :  -- Reconstrucción de patrón a travéz del método scs global, sin diccionario, uso de números.
--                -- Traducción de patrón a parámetros físicos
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
max_time = 400 * ticks_per_second -- Duración máxima de la simulación en ticks
Num_sensores_de_proximidad = 24 -- Numero de sensores del robot
------------------------------
--Parámetro de simulación internos

angulo_treshold = 10            -- Sensibilidad del observador a caambios en los datos de angulo obtenidos del maestro, ajuste recomendado: 10 < sensitivity_2 < 50
demostrador_threshold = 41      -- Distancia máxima a la que a la que el maestro espera ver al observador
threshold = 40                   -- Distancia a la que se ubica el observador del maestro
avance = 1                      -- Influye en la sensibilidad de deteccion de segmentos, valor minimo = 1
sensitivity_2 = 2               -- Sensibilidad del observador a caambios en los datos de distancia obtenidos del maestro, ajuste recomendado: 1 < sensitivity_2 < 2

speed_dem = 5                   -- Velocidad del demostrador al ejecutar el patrón
d_speed = 15                    -- velocidad normal
g_speed = 50                     -- Velocidad de giro

max_tracking_distance = 165      -- Máxima distancia a la que se puede alejar el maestro antes de que el alumno se acerque para no perderlo de vista. cm
min_tracking_distance = 30      -- Miínima distancia a la que se puede acercar el maestro antes de que el alumno se aleje para no colisionar. cm
porcentaje = 0.5                -- 1 < porcentaje < 0.33 , indica que tanto el observador busca acercarse al maestro cuando este cruza el max_tracking_distance.

sensitivity = ( ( (d_speed-speed_dem)/ticks_per_second ) * avance ) / 5 -- speed_dem/ticks_per_second indica cuanto avanza el robot demostrador en un tick, 
                                                            -- *avance indica con que valor se va a comparar, en este caso el minimo para maxima sensibilidad
------------------------------
--Parametros de patrónes predefinidos
perm1c = 80
perm2c = 80

permt = 30

l1 = 30
l2 = 60

slide1 = 10
slide2 = 30

six1 = 60 -- lado corto 
six2= 80


figura = "Heptágono" -- "triangulo_irregular_1" --"Pentágono_irregular"  -- "Heptágono"-- "" -- "trayectoria_1" 
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

    if timer > 1000 then
        -- Ejemplo de uso para un elemento específico
        -- local k = 1
        -- imprimir_tracking_entry(k, master_tracking[k])

        -- O imprimir TODO el master_tracking:
        -- for key, value in pairs(master_tracking) do
        --     imprimir_tracking_entry(key, value)
        -- end
        --ls = ls +1
    end 
    
    -- Suponiendo que tienes un contador global 'global_timer'
    if timer == 2000 and not Global_Reconstruction_Done then
        log("TIEMPO LÍMITE ALCANZADO: Ejecutando Reconstrucción Global...")
        run_global_reconstruction()
    end

    if timer == max_time then
        print("End_simulation")
    end

 

end

-------------------------------------------------------------------------------------------------------------------------------------
-- Funciones para cada rol
-------------------------------------------------------------------------------------------------------------------------------------

d1 = 0
c5 = 0
demostrador_avg_angle = 0
demostrador_avg_dist = 0
function demostrador()


    d1 = 2 -- ESTAMOS PROVISIONALMENTE FORZANDO AL DEMOSTRADOR A EJECUTAR EL PATRON DIRECTAMENTE PARA PODER VER EL FUNCIONAMIENTO DEL OBSERVADOR

    if d1 == 0 then
        detectar_zona_demostracion()
        --log("detectar_zona_demostracion")
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

            if figura == "triangulo_irregular_1" then
                Patron_Original{
                        params = {
                        { 80, 10},
                        { 60, 90},
                        { 100, 127},
                        { 0, 133}
                        },
                    name = "Triangulo irregular 1"
                }
            end

            if figura == "Pentágono_irregular" then
                Patron_Original{
                        params = {
                        { 50, 10},
                        { 50, 90},
                        { 50, 53},
                        { 50, 74},
                        { 58.3, 84},
                        { 0, 48}
                        },
                    name = "Pentágono_irregular"
                }
            end

            if figura == "Heptágono" then
                Patron_Original{
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
                Patron_Original{
                        params = {
                        { 40 , 5 },
                        { 50 ,-45 },
                        { 30 ,-90 },
                        { 10 ,110 },
                        { 20 ,-30 },
                        { 0 ,-180 },
                        },
                    name = "Trayectoria 1"
                }
            end
        end
    elseif d1 == 3 then
        Maestro_triste()
    end
end


c3 = 0
c4 = 0
function observador()

            --log("1" .. c4)
    if c3 == 0 then

            --log("2:   " .. c4)
        if c4 == 0 then
            store_relative_positions()
            --log("3:   " .. c4)
        elseif c4 == 1 then
            patron_imitado(full_pattern_params)
            --log("si llegue")
            
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
    --print( robot.id.." & posicion: x=" .. posicion[1] .. ", y=" .. posicion[2])
    print( robot.id.." & " .. posicion[1] .." , ".. posicion[2])
end

PO_c1 = 0
ind = 1
PO_end = false 
function Patron_Original(p)

    --============================== Triángullo ==============================
    if figura == "triangulo_irregular_1" then

        if robot.id == "Footbot2" then 
            if     c2 >= 2 then
                setleds("black")
            elseif c2 < 2 then
                setleds("yellow")
                print_positions()
            end
        end

        if robot.id == "Footbot1" then 

            if c2 < 3 and c2 >= 1 then
                setleds("yellow")
                print_positions()
            else
                setleds("black")
            end
        end

        if robot.id == "Footbot3" then 

            if c2 == 1 then
                setleds("black")
            else
                setleds("yellow")
                print_positions()
            end
        end
    end

    --============================== Pentágono_irregular ==============================

    if figura == "Pentágono_irregular" then

        if robot.id == "Footbot2" then 

            if     timer > 40 and timer < 480 then
                setleds("yellow")
                print_positions()
            else
                setleds("black")
            end

            -- if     timer > 0 then
            --     setleds("yellow")
            --     print_positions()
            -- else
            --     setleds("black")
            -- end
        end

        if robot.id == "Footbot1" then 

            if timer > 740 and timer < 1210 then
                setleds("yellow")
                print_positions()
            else
                setleds("black")
            end
        end

        if robot.id == "Footbot3" then 

            if timer > 1220 and timer < 1780 then
                setleds("yellow")
                print_positions()
            else
                setleds("black")
            end
        end
    end

    --============================== Heptágono ==============================

    if figura == "Heptágono" then

        if robot.id == "Footbot2" then 

            if     timer > 0 and timer < 450 then
                setleds("yellow")
                print_positions()
            elseif timer > 1150 and timer < 1400 then
                setleds("yellow")
                print_positions()
            else
                setleds("black")
            end

            -- if     timer > 1220 and timer < 1775 then
            --     setleds("yellow")
            --     print_positions()
            -- else
            --     setleds("black")
            -- end
        end

        if robot.id == "Footbot1" then 

            if timer > 460 and timer < 780 then
                setleds("yellow")
                print_positions()
            else
                setleds("black")
            end
        end

        if robot.id == "Footbot3" then 

            if timer > 620 and timer < 1130 then
                setleds("yellow")
                print_positions()
            else
                setleds("black")
            end
        end
    end

    --============================== trayectoria_1 ==============================
    if figura == "trayectoria_1" then

        if robot.id == "Footbot1" then 

            if timer > 0 and timer < 360 then
                setleds("yellow")
                print_positions()
            else
                setleds("black")
            end
        end

        if robot.id == "Footbot2" then 
            if   timer > 200 and timer < 590 then
                setleds("yellow")
                print_positions()
            else
                setleds("black")
            end
        end

        if robot.id == "Footbot3" then 
            if timer > 500 and timer < 700 then
                setleds("yellow")
                print_positions()
            else
                setleds("black")
            end
        end
    end

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
                    if robot.id == "Footbot1" then 
                        --setleds("green")
                    end

                    --setleds("green")

                    -- log( " -------------------------------------------------------------------------" )
                    -- imprimir_posicion(posiciones_guardadas)
                    -- log(" ========= patrón original: "..name.." finalizado por: " .. robot.id.." ========= ")
                    -- log( " -------------------------------------------------------------------------" )
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
                local valor1, valor2 = params[indice][1], params[indice][2]
                --log("Distancia a recorrer en el segmento "..indice.." = ".. params[indice][1]..", Distancia actual: "..move_distance..", "..robot.id)
                --log("Angulo del segmento = "..indice.." = ".. params[indice][2]..", Angulo actual: "..move_angle*180/math.pi ..", "..robot.id)
                move(valor1, valor2, speed_dem, 10, false, robot.id)
                print_positions()

                if c1 == 2 then
                    if indice == #params then
                        indice = indice + 1
                        PI_end = true
                        r_Temporizador_1()

                        --log(" Ultimo Segmento: "..indice.." finalizado por: "..robot.id)
                        --log("Segmentos finalizados, "..robot.id)
                                    
                        if c5 == 1 then
                            k = 1
                            r_move()
                            setleds("green")
                            -- if robot.id == "Footbot11" then  
                            --     imprimir_posicion(posiciones_guardadas)
                            -- end
                            --log(" ========================================================================= ")
                            --log(" Patrón Imitado: finalizado por: ".. robot.id)
                            --log("Encendiendo led en verde -----------"..robot.id)
                        else
                            c5 = 1
                            k = 1
                            setleds("green")
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


-- -- (Función de inicialización que ya me diste, la incluyo para que esté completa)
master_tracking = {}

function initialize_master_tracking(k, angle_deg, distance, ty)
    -- ... (cálculos de ángulo y posición existentes) ...
    local angle_rad = angle_deg * (math.pi / 180)
    local x, y = polarACartesianoPAR(distance, angle_rad) 

    master_tracking[k] = {
        -- ... (tus variables existentes: x_ant_r, srt_ang_mark_ant, etc.) ...
        x_ant_r = 0,
        y_ant_r = 0,
        srt_ang_mark_ant = 0,
        x_mark = 0, 
        y_mark = 0, 
        
        prev_x_original = x,
        prev_y_original = y,
        stationary_active = false,
        stationary_count = 0,
        movement_buffer = { {x, y} },
        type_buffer = { ty },
        
        segmentos = { movement = {}, type = {}, valid = {} },
        params = {},
        pattern_sequence = "",

        initial_stationary_check_ticks = 3, 

        saw_initial_stationary = true, 
        Pattern_start_detected = false,
        End_of_pattern_detected = false,

        printed = false,
    }
end

-- =========================================================================
--  NUCLEO DE RECONSTRUCCIÓN MATEMÁTICA (MODULAR: GREEDY + EXHAUSTIVE BACKTRACKING)
-- =========================================================================

-- CONFIGURACIÓN
local CONFIG = {
    TOL_DIST            = 2.0,
    TOL_ANG             = 3.0,
    MIN_OVERLAP         = 1,
    MAX_RECURSION_DEPTH = 15,    -- Límite de profundidad
    PRINT_RECURSIVE_PATHS = true -- BANDERA: true = Imprime los pasos de la solución elegida
}

-- BUFFER DE DATOS GLOBAL
DataBuffer = { stored = {} }
Global_Reconstruction_Done = false 

-- =========================================================================
--  1. HELPER DE LOGS Y VISUALIZACIÓN
-- =========================================================================
local function fmt_struct(s)
    if not s then return "nil" end
    local vals = {}
    for _, v in ipairs(s.value) do table.insert(vals, string.format("%.2f", v)) end
    local t_str = (s.type == 1) and "Ang" or "Dist"
    local count_str = (s.count and s.count > 1) and string.format(" [x%d]", s.count) or ""
    return string.format("(%s)%s {%s}", t_str, count_str, table.concat(vals, ", "))
end

function DataBuffer.add(new_dat)
    if Global_Reconstruction_Done then return end 
    local found = false
    for i, exist in ipairs(DataBuffer.stored) do
        if exist.type == new_dat.type and #exist.value == #new_dat.value then
            local all_match = true
            for k = 1, #new_dat.value do
                local is_odd = (k % 2 ~= 0)
                local val_type = (new_dat.type == 1) and (is_odd and "A" or "D") or (is_odd and "D" or "A")
                local diff = math.abs(exist.value[k] - new_dat.value[k])
                local tol = (val_type == "A") and CONFIG.TOL_ANG or CONFIG.TOL_DIST
                if diff > tol then all_match = false; break end
            end
            if all_match then
                found = true; exist.count = (exist.count or 1) + 1
                for k = 1, #exist.value do exist.value[k] = exist.value[k] + (new_dat.value[k] - exist.value[k]) / exist.count end
                -- log(string.format("[INFO] Dato Refinado (Promedio de %d): %s", exist.count, fmt_struct(exist)))
                break
            end
        end
    end
    if not found then
        local new_v = {}; for _,v in ipairs(new_dat.value) do table.insert(new_v, v) end
        local entry = { value = new_v, type = new_dat.type, count = 1 }
        table.insert(DataBuffer.stored, entry)
        log(string.format("[INFO] Nuevo Dato Registrado: %s", fmt_struct(entry)))
    end
end

-- =========================================================================
--  2. FUNCIONES MATEMÁTICAS PURAS (COMPARTIDAS)
-- =========================================================================

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
--  3. MÉTODO 1: GREEDY (Con Logs Detallados)
-- =========================================================================
local function solve_greedy(initial_pool)
    local pool = clone_pool(initial_pool)
    
    -- Cabecera idéntica a tu solicitud
    log("TIEMPO LÍMITE ALCANZADO: Ejecutando Reconstrucción Global...")
    log("=========================================")
    log("EJECUTANDO RECONSTRUCCIÓN (GLOBAL GREEDY)")
    log("=========================================")

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
            -- Pre-formateo para el log detallado
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
--  4. MÉTODO 2: EXHAUSTIVE BACKTRACKING (Busca TODAS las soluciones)
-- =========================================================================

-- Tabla donde acumularemos todas las soluciones perfectas encontradas
-- Estructura: { {pool_resultante}, {historial_de_logs} }
local solutions_found = {} 

local function recursive_finder(current_pool, depth, history_log)
    -- A) Caso Base: Éxito (Solo queda 1 pieza)
    if #current_pool == 1 then
        -- Guardamos esta solución y su historial
        table.insert(solutions_found, { result = current_pool[1], history = history_log })
        return
    end
    
    -- B) Límite de Profundidad
    if depth > CONFIG.MAX_RECURSION_DEPTH then return end

    -- C) Buscar candidatos
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

    if #candidates == 0 then return end -- Callejón sin salida

    -- Ordenar por heurística (mayores solapes primero), pero exploraremos todos
    table.sort(candidates, function(a,b) return a.ov > b.ov end)

    -- D) Iterar y Recursividad (Branching)
    for _, cand in ipairs(candidates) do
        local next_pool = {}
        -- Crear fusión
        local merged = merge_fragments(current_pool[cand.i], current_pool[cand.j], cand.ov)
        table.insert(next_pool, merged)
        
        -- Añadir el resto
        for k=1, #current_pool do
            if k ~= cand.i and k ~= cand.j then
                table.insert(next_pool, current_pool[k])
            end
        end

        -- Clonar historial para esta rama
        local next_history = {} 
        for _, h in ipairs(history_log) do table.insert(next_history, h) end
        table.insert(next_history, string.format("   [M2-Depth:%d] Merge %d+%d (Ov:%d) -> %s", depth, cand.i, cand.j, cand.ov, fmt_struct(merged)))

        -- Llamada Recursiva (NO retornamos, dejamos que explore todas)
        recursive_finder(next_pool, depth + 1, next_history)
    end
end

-- Función selectora (criterio futuro)
local function select_best_solution(solutions)
    if #solutions == 0 then return nil end
    
    -- CRITERIO ACTUAL: Aleatorio
    -- TODO: En el futuro, iterar 'solutions' y elegir la que tenga mayor longitud, o coincida con un patrón esperado.
    
    --local choice_idx = math.random(1, #solutions)

    local choice_idx = 14

    log(string.format("[GESTOR] Se encontraron %d soluciones perfectas. Seleccionada la #%d al azar.", #solutions, choice_idx))
    
    return solutions[choice_idx]
end

local function solve_smart_backtracking_all(initial_pool)
    solutions_found = {} -- Reiniciar acumulador global
    
    -- Preparar pool ordenado
    local sorted_pool = clone_pool(initial_pool)
    table.sort(sorted_pool, function(a,b) 
        local lenA = #a.value; local lenB = #b.value
        if a.count and b.count and a.count ~= b.count then return a.count > b.count end
        return lenA > lenB 
    end)

    -- Iniciar búsqueda recursiva
    recursive_finder(sorted_pool, 1, {})

    -- Seleccionar la mejor
    return select_best_solution(solutions_found)
end


-- =========================================================================
--  5. GESTOR PRINCIPAL
-- =========================================================================
function run_global_reconstruction()
    if Global_Reconstruction_Done then return end
    
    -- Preparamos datos iniciales
    local start_pool = {}
    for i, v in ipairs(DataBuffer.stored) do 
        table.insert(start_pool, { value = v.value, type = v.type, count = v.count, id = i }) 
    end
    
    if #start_pool == 0 then return end

    -- EJECUTAR MÉTODO 1 (Greedy) - Siempre se ejecuta primero para mostrar logs clásicos
    local greedy_result_pool = solve_greedy(start_pool)
    local final_result_struct = nil

    -- VERIFICACIÓN
    if #greedy_result_pool == 1 then
        log("[GESTOR] Método 1 Exitoso. Cadena única encontrada.")
        final_result_struct = greedy_result_pool[1]
    else
        log(string.format("[GESTOR] Método 1 incompleto. Quedaron %d islas.", #greedy_result_pool))
        log("[GESTOR] Activando Método 2: Deep Backtracking (Búsqueda de solución perfecta)...")
        
        -- EJECUTAR MÉTODO 2 (Exhaustivo)
        local best_match = solve_smart_backtracking_all(start_pool)
        
        if best_match then
            log("[GESTOR] ¡MÉTODO 2 EXITOSO! Se encontró un camino de fusión perfecto.")
            final_result_struct = best_match.result
            
            -- Imprimir camino si la bandera está activa
            if CONFIG.PRINT_RECURSIVE_PATHS then
                log("[GESTOR] Historial de fusión de la solución seleccionada:")
                for _, step in ipairs(best_match.history) do
                    log(step)
                end
            end
        else
            log("[GESTOR] Método 2 no encontró solución perfecta (Islas o Límite Profundidad).")
            log("[GESTOR] Se mantiene el mejor esfuerzo del Greedy.")
            -- Fallback: Usamos la isla más grande del Greedy
            table.sort(greedy_result_pool, function(a,b) return #a.value > #b.value end)
            final_result_struct = greedy_result_pool[1]
        end
    end

    -- RESULTADOS FINALES
    log("\n---------------- RESULTADOS ----------------")
    if final_result_struct then
        log(">> CADENA PRINCIPAL RECONSTRUIDA:")
        log(fmt_struct(final_result_struct))
    else
        log(">> ERROR CRÍTICO: No se pudo recuperar ninguna estructura válida.")
    end
    
    -- Si quedaron otras islas del Greedy (y falló el M2), se mostrarían aquí, 
    -- pero para simplificar la salida final solo mostramos la ganadora.
    log("--------------------------------------------\n")

    Global_Reconstruction_Done = true
    if final_result_struct then
        process_numeric_result(final_result_struct)
    end
end







-- Variable global para almacenar el resultado final
full_pattern_params = {} 

function process_numeric_result(result_struct)
    full_pattern_params = {}
    local values = result_struct.value
    local current_type = result_struct.type -- 1=Ang, 0=Dist
    
    log("\n[RECONSTRUCCIÓN FINALIZADA]")
    -- log("Estructura Resultante: Tipo="..current_type..", Datos={"..table.concat(values,", ").."}")

    local temp_dist = 0
    local temp_ang = 0
    local expecting_dist = (current_type == 0) -- Si es tipo 0, esperamos dist primero
    
    -- Máquina de estados simple para reconstruir pares {dist, ang}
    for i, val in ipairs(values) do
        if expecting_dist then
            -- Es una distancia
            temp_dist = val
            -- Ahora tenemos el par (Angulo previo + Distancia actual)
            -- Nota: Si es el primer segmento, el ángulo será 0 (valor inicial)
            table.insert(full_pattern_params, { temp_dist, temp_ang })
            
            -- Reiniciamos ángulo para el siguiente y esperamos un ángulo
            temp_ang = 0 
            expecting_dist = false
        else
            -- Es un ángulo
            temp_ang = val -- Lo guardamos para la SIGUIENTE distancia
            expecting_dist = true
        end
    end
    
    log(">> Patrón traducido a parametros físicos. Total Segmentos: " .. #full_pattern_params)
    for idx, seg in ipairs(full_pattern_params) do
        log(string.format("   Seg %d: Dist %.2f, Ang %.2f", idx, seg[1], seg[2]))
    end

    -- ACTIVAR ESTADO FINAL DEL ROBOT
    c4 = 1 
end

-- =========================================================================
-- ¡NUEVA FUNCIÓN DE SEGMENTACIÓN! (¡MODIFICADA!)
-- =========================================================================
-- Implementa la lógica de segmentación para un 'track' de maestro específico.
stationary_tolerance    = sensitivity -- Asegúrate que 'sensitivity' esté definida
function process_master_segmentation(k, track, x_original, y_original, ty)

    --log(string.format("\nProcesando segmentación para maestro "..k.." en posición (%.3f, %.3f) con type %d", x_original, y_original, ty))
    
    -- ¡MODIFICADO! Mover el cálculo de dx/dy y 'quieto' al inicio
    -- para la comprobación de N-ticks
    local is_stationary_this_tick = false
    if track.prev_x_original == nil then
        track.prev_x_original, track.prev_y_original = x_original, y_original
    else
        local dx = math.abs(x_original - track.prev_x_original)
        local dy = math.abs(y_original - track.prev_y_original)
        if dx <= stationary_tolerance and dy <= stationary_tolerance then
            is_stationary_this_tick = true
        end
    end

    -- =========================================================================
    -- ¡NUEVO! Comprobación de condición inicial (N-ticks)
    -- =========================================================================
    if track.initial_stationary_check_ticks == 0 then
        --log("Chequeando condición inicial para "..k..". Ticks restantes: "..track.initial_stationary_check_ticks)
        if is_stationary_this_tick then
            track.saw_initial_stationary = true
            --log("¡Maestro "..k.." detectado quieto en el N-tick check!")
        else
            track.saw_initial_stationary = false
            --log("Maestro "..k..": N-ticks terminados. NO se vio quieto al inicio.")
        end
    end

    track.initial_stationary_check_ticks = track.initial_stationary_check_ticks - 1

    -- =========================================================================


    if is_stationary_this_tick then
        -- ESTACIONARIO: Dentro de la tolerancia
        --log("maestro dentro del rango estacionario: dx ="..dx.." < "..stationary_tolerance..", dy="..dy.." < "..stationary_tolerance)
        if not track.stationary_active then
            -- Primer tick de este bloque estacionario

            track.stationary_active = true

            if track.stationary_count >= 1 then
                local seg_k = track.stationary_count -- (Índice del segmento que ACABA de terminar)
                track.segmentos.movement[seg_k] = track.segmentos.movement[seg_k] or {}
                track.segmentos.type[seg_k] = track.segmentos.type[seg_k] or {}
                track.segmentos.valid[seg_k] = track.segmentos.valid[seg_k] or {} -- ¡NUEVO! Inicializar array de validez

                -- Volcar el buffer (movimiento) al segmento anterior
                for i, pt in ipairs(track.movement_buffer) do
                    table.insert(track.segmentos.movement[seg_k], { pt[1], pt[2] })
                    local type_to_save = track.type_buffer[i]
                    table.insert(track.segmentos.type[seg_k], type_to_save)
                    --log(string.format("Maestro %d, Segmento %d, Punto %d: (%.3f, %.3f), Type %d", k, seg_k, i, pt[1], pt[2], type_to_save))

                end

                -- Añadir el punto actual (que inicia el estado estacionario)
                table.insert(track.segmentos.movement[seg_k], { x_original, y_original })
                local current_ty_to_save = ty
                table.insert(track.segmentos.type[seg_k], current_ty_to_save)

                srt_counter_segments = seg_k
                detect_initial_or_end_segment()

                -- =========================================================================
                -- ¡NUEVO! Lógica de validación (cuando se cierra un segmento normal)
                -- =========================================================================
                local is_valid = false
                local start_condition_met = false
                -- Si estamos aquí, el maestro ESTÁ quieto, así que el final se vio correctamente.
                local end_condition_met = true  

                -- 1. Comprobar condición INICIAL
                if seg_k == 1 then
                    -- Es el primer segmento. ¿Lo vimos desde el inicio?
                    -- (Ya sea por el blob ROJO (Pattern_start_detected) o por el chequeo de N-ticks)
                    if track.Pattern_start_detected or track.saw_initial_stationary then
                        start_condition_met = true
                    end
                else
                    -- Es un segmento posterior (seg_k > 1).
                    -- El inicio de este segmento (N) es el final 
                    -- del segmento anterior (N-1), que fue un 'quieto'.
                    -- Por lo tanto, SIEMPRE vimos el inicio.
                    start_condition_met = true
                end
                

                -- 2. Decisión Final

                if start_condition_met and end_condition_met then
                    is_valid = true
                    -- log("\nSegmento "..seg_k..": Válido".." MAESTRO: "..k) 

                    -- if track.Pattern_start_detected and seg_k == 1 then
                    --     log("Inicio detectado por blob ROJO")
                    -- else
                    --     log("Inicio detectado por estado quieto")
                    -- end

                    -- if track.End_of_pattern_detected then
                    --     log("end detectado por blob VERDE")
                    -- else
                    --     log("end detectado por estado quieto")
                    -- end
                else 
                    -- if not start_condition_met then
                    --     log("\nSegmento "..seg_k..": Inválido (Inicio NO Visto)".." MAESTRO: "..k)
                    -- else
                    --     log("\nSegmento "..seg_k..": Inválido (Final NO Visto)".." MAESTRO: "..k)
                    -- end
                end

                -- 3. Guardar el resultado (replicando la estructura de .type)
                for _ in ipairs(track.segmentos.movement[seg_k]) do
                    table.insert(track.segmentos.valid[seg_k], is_valid)
                end
                -- =========================================================================
                
                -- =========================================================================
                -- C) NUEVO: PROCESAMIENTO DE DATOS (GENERACIÓN DE PARAMS)
                -- =========================================================================
                -- Solo procesamos si tenemos segmentos grabados y si se detectó el fin del patrón
                if #track.segmentos.movement > 0 and track.End_of_pattern_detected then
                    --log("\n--- Iniciando Procesamiento de Trayectoria para Maestro "..k.." ---")

                    -- 1. Obtener Ecuaciones (apply)
                    -- 'apply' devuelve 'equations'
                    local equations = apply(track.segmentos.movement)

                    -- 2. Calcular Intersecciones
                    -- toma 'equations' y los segmentos originales
                    local intersections = calcular_intersecciones_con_extremos(equations, track.segmentos.movement)

                    -- 3. Calcular Distancias y Ángulos
                    -- devuelve dos tablas: distancias y ángulos
                    local distancias_a_recorrer, angles_a_recorrer = calcular_distancias_y_angulos(intersections)

                    -- 4. Crear Parámetros Finales
                    -- Esta función devuelve la tabla 'params' estructurada
                    local generated_params = create_params(distancias_a_recorrer, angles_a_recorrer)

                    -- 5. ALMACENAR EN EL MAESTRO

                    track.params = generated_params
                   -- log("\nMaestro "..k..": Procesamiento completado. Parametros guardados: "..#track.params)
                    
                else
                    -- log("Maestro "..k..": No hay segmentos para procesar.")
                    -- track.params = {} 
                end
                -- =========================================================================
                   
                -- Dentro de process_master_segmentation...
                -- (Todo igual hasta llegar al bloque D)

                -- =========================================================================
                -- D) NUEVO: PRE-PROCESAMIENTO NUMÉRICO (SIN LETRAS)
                -- =========================================================================
                -- Solo procesar si tenemos parámetros calculados
                if #track.params > 0 then
                    
                    local structure_values = {}
                    local structure_type = nil -- 0: Empieza Dist, 1: Empieza Angulo
                    
                    -- Iteramos sobre los parámetros {distancia, angulo}
                    for i, param in ipairs(track.params) do
                        local distancia = param[1]
                        local angulo    = param[2]
                        
                        -- Verificar validez del segmento (usando tu lógica existente)
                        local is_segment_valid = false
                        if track.segmentos.valid[i] and track.segmentos.valid[i][1] then
                            is_segment_valid = true
                        end

                        -- REGLA 1: El ángulo se agrega si i > 1
                        if i > 1 then
                            table.insert(structure_values, angulo)
                            -- Si es el PRIMER dato que insertamos en la lista, definimos el TYPE
                            if structure_type == nil then structure_type = 1 end -- Empezó con Angulo
                        end

                        -- REGLA 2: La distancia se agrega si es válida
                        if is_segment_valid then
                            table.insert(structure_values, distancia)
                            -- Si es el PRIMER dato que insertamos, definimos el TYPE
                            if structure_type == nil then structure_type = 0 end -- Empezó con Distancia
                        end
                    end

                    -- Si logramos recolectar datos, enviarlos al Buffer
                    if #structure_values > 0 and structure_type ~= nil then
                        local data_packet = {
                            value = structure_values,
                            type = structure_type
                        }
                        
                        -- log(">> Enviando paquete al buffer: Tipo " .. structure_type .. ", Len " .. #structure_values)
                        DataBuffer.add(data_packet)
                    end
                    
                    -- NOTA: El trigger del algoritmo (run_global_reconstruction) 
                    -- NO se llama aquí. Se debe llamar en el loop principal (step) 
                    -- cuando 'timer > 4000' (o la condición que desees).
                end
                -- =========================================================================

                track.initial_stationary_check_ticks = 3 -- Reiniciar para no volver a entrar

                -- Iniciar el buffer del siguiente segmento
                track.movement_buffer = { { x_original, y_original } }
                track.type_buffer = { ty }

            end

            track.stationary_count = track.stationary_count + 1
        end
        -- (Si ya estaba stationary_active, no hacer nada)
    else
        -- MOVIMIENTO: Fuera de la tolerancia
        track.stationary_active = false
        table.insert(track.movement_buffer, { x_original, y_original })
        table.insert(track.type_buffer, ty )
    end

    -- Actualizar anterior para la siguiente comparación
    track.prev_x_original, track.prev_y_original = x_original, y_original
end

srt_counter_segments    = 0
srt_counter_segments_prev = 0 -- ¡NUEVO! Para controlar la ejecución de la sección 8
srt_anterior_data = {}
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

    Temporizador_1(20) 
      
    if temp == false then
        robot.leds.set_all_colors("black") 
    end

    -- 1. LÓGICA DE SEGUIMIENTO (AMARILLO Y VERDE)
    if robot.colored_blob_omnidirectional_camera then                
        
        local blobs = robot.colored_blob_omnidirectional_camera
        local yellow_blobs = {} 
        local green_blobs = {}  
        local count = 0   

        -- =========================================================================
        -- 1. RECOLECCIÓN (Sin cambios)
        -- ... (Tu código de recolección va aquí) ...
        for i, blob in ipairs(blobs) do
            local r, g, b = blob.color.red, blob.color.green, blob.color.blue
            local angle = blob.angle * (180 / math.pi)
            
            --log(string.format("Blob %d RGB=(%d,%d,%d) angle=%.2f° distance=%.2f cm", i, r, g, b, angle, blob.distance))

            if r > 200 and g > 200 and b < 100 then 
                table.insert(yellow_blobs, {angle = angle, distance = blob.distance, type = 2})
                -- log("\nBlob "..i.." , amarillo detectado: " .. angle .. "°, " .. blob.distance .. "cm\n")
            elseif r < 100 and g > 200 and b < 100 then 
                -- log("Blob "..i.." , verde detectado: " .. angle .. "°, " .. blob.distance .. "cm\n")
                table.insert(green_blobs, {angle = angle, distance = blob.distance})
            elseif  r > 200 and g < 100 and b < 100  then 
                table.insert(yellow_blobs, {angle = angle, distance = blob.distance, type = 1})
                --log("Blob "..i.." , rojo detectado: " .. angle .. "°, " .. blob.distance .. "cm\n")
            end 
        end  

        -- =========================================================================
        -- 2. PREPARACIÓN (Sin cambios)
        -- ... (Tu código de preparación va aquí) ...
        local srt_anterior_angle_positive = {}
        local srt_anterior_angle_negative = {}
        local updated_anterior_indices = {}
        local matched_new_indices = {}

        for k, data in pairs(srt_anterior_data) do
            if data and data.angle then 
                if data.angle >= 0 then
                    table.insert(srt_anterior_angle_positive, {angle = data.angle, original_index = k})
                else
                    table.insert(srt_anterior_angle_negative, {angle = data.angle, original_index = k})
                end
            end
        end

        -- =========================================================================
        -- 3. COMPARACIÓN Y ACTUALIZACIÓN (Blobs Amarillos) (Sin cambios)
        -- ... (Tu código de comparación va aquí) ...
        for j, new_data in ipairs(yellow_blobs) do
            local target_anterior_list
            if new_data.angle >= 0 then
                target_anterior_list = srt_anterior_angle_positive
            else
                target_anterior_list = srt_anterior_angle_negative
            end

            for _, anterior_info in ipairs(target_anterior_list) do
                local k = anterior_info.original_index
                if not updated_anterior_indices[k] then
                    local anterior_data = srt_anterior_data[k]
                    if anterior_data.angle then -- Chequeo extra por si acaso
                        local new_angle_abs = math.abs(new_data.angle)
                        local anterior_angle_abs = math.abs(anterior_data.angle)
                        
                        local max_ang = new_angle_abs + angulo_treshold
                        local min_ang = new_angle_abs - angulo_treshold

                        if anterior_angle_abs < max_ang and anterior_angle_abs > min_ang then
                            local new_dist = new_data.distance
                            local anterior_dist = anterior_data.distance
                            local max_dist = anterior_dist + sensitivity_2*(speed_dem+d_speed)/ticks_per_second
                            local min_dist = anterior_dist - sensitivity_2*(speed_dem+d_speed)/ticks_per_second

                            if new_dist < max_dist and new_dist > min_dist then
                                srt_anterior_data[k].angle = new_data.angle
                                srt_anterior_data[k].distance = new_data.distance
                                srt_anterior_data[k].type = new_data.type 

                                --log("Maestro " .. k .. " actualizado: Ángulo " .. srt_anterior_data[k].angle .. "°, Distancia " .. srt_anterior_data[k].distance .. "cm" .. ", tipo " .. srt_anterior_data[k].type)

                                updated_anterior_indices[k] = true
                                matched_new_indices[j] = true
                                count = count + 1
                                break 
                            end 
                        end
                    end 
                end 
            end 
        end 

        -- =========================================================================
        -- 4. AÑADIR NUEVOS MAESTROS (Blobs Amarillos) 
        for j, new_data in ipairs(yellow_blobs) do
            if not matched_new_indices[j] then

                table.insert(srt_anterior_data, {
                    angle = new_data.angle, 
                    distance = new_data.distance, 
                    type = new_data.type or 2
                })
                
                local new_k = #srt_anterior_data
                updated_anterior_indices[new_k] = true
                
                log("\n\nMaestro " .. new_k .. " creado: Ángulo " .. new_data.angle .. "°, Distancia " .. new_data.distance .. "cm" .. ", tipo " .. (new_data.type).."\n\n")

                initialize_master_tracking(new_k, new_data.angle, new_data.distance,srt_anterior_data[new_k].type)
            end
        end

        -- =========================================================================
        -- 5. PROCESAR BLOBS VERDES 
        for _, green_blob in ipairs(green_blobs) do
            local green_angle_abs = math.abs(green_blob.angle)
            
            for k, master_data in pairs(srt_anterior_data) do
                if master_data.angle and master_data.distance and master_data.type then
                    local master_angle_abs = math.abs(master_data.angle)
                    local max_ang = green_angle_abs + angulo_treshold
                    local min_ang = green_angle_abs - angulo_treshold

                    if master_angle_abs < max_ang and master_angle_abs > min_ang then
                        local master_dist = master_data.distance
                        local green_dist = green_blob.distance
                        local max_dist = master_dist + sensitivity_2*(speed_dem+d_speed)/ticks_per_second
                        local min_dist = master_dist - sensitivity_2*(speed_dem+d_speed)/ticks_per_second

                        if green_dist < max_dist and green_dist > min_dist then
                            srt_anterior_data[k].type = 3
                            updated_anterior_indices[k] = true 
                            -- log("¡MATCH! Este blob verde corresponde al maestro " .. k)
                            -- log("Maestro " .. k .. " ha marcado el final (Type 3).")
                            break 
                        end
                    end
                end
            end 
        end 

        -- =========================================================================
        -- 6. ELIMINAR Y PROCESAR MAESTROS PERDIDOS (¡MODIFICADO!)
        -- =========================================================================
        for k = 1, #srt_anterior_data do
            
            -- (Si el maestro 'k' existe Y *no* fue actualizado en este tick)
            if srt_anterior_data[k] and srt_anterior_data[k].angle and not updated_anterior_indices[k] then
                
                local track = master_tracking[k]
                
                -- Entramos aquí SOLO la primera vez que confirmamos que se perdió
                if track and not track.printed and not track.End_of_pattern_detected then
                    
                    -- A) VOLCADO FINAL DEL BUFFER (Tu código existente)
                    -- =============================================================
                    if track.movement_buffer and #track.movement_buffer > 0 then
                        local final_seg_k = track.stationary_count 
                        srt_counter_segments = final_seg_k
                        
                        -- Inicializar tablas si no existen
                        track.segmentos.movement[final_seg_k] = track.segmentos.movement[final_seg_k] or {}
                        track.segmentos.type[final_seg_k] = track.segmentos.type[final_seg_k] or {}
                        track.segmentos.valid[final_seg_k] = track.segmentos.valid[final_seg_k] or {} 
                        
                        -- Volcar puntos y tipos
                        for i, pt in ipairs(track.movement_buffer) do
                            table.insert(track.segmentos.movement[final_seg_k], { pt[1], pt[2] })
                            table.insert(track.segmentos.type[final_seg_k], track.type_buffer[i])
                        end

                        -- Limpiar buffers temporales
                        track.movement_buffer = {}
                        track.type_buffer = {}

                        -- B) VALIDACIÓN DEL ÚLTIMO SEGMENTO (Tu código existente)
                        -- =============================================================
                        local is_valid = false
                        local start_condition_met = false
                        local end_condition_met = false

                        -- 1. Condición Inicial
                        if final_seg_k == 1 then
                            if track.Pattern_start_detected or track.saw_initial_stationary then
                                start_condition_met = true
                            end
                        else
                            start_condition_met = true
                        end

                        -- 2. Condición Final
                        if track.stationary_active then
                            end_condition_met = true
                        end
                        
                 -- 3. Decisión Final
                        if start_condition_met and end_condition_met then
                            is_valid = true
                            -- log("\nSegmento final "..final_seg_k..": Válido".." MESTRO: "..k)  

                            -- if track.Pattern_start_detected and final_seg_k == 1 then
                            --     log("Inicio detectado por blob ROJO")
                            -- else
                            --     log("Inicio detectado por estado quieto")
                            -- end

                            -- if track.stationary_active then
                            --     log("end detectado por estado quieto")
                            -- end
                        else 
                            -- if not start_condition_met then
                            --     log("\nSegmento final "..final_seg_k..": Inválido (Inicio NO Visto)".." MAESTRO: "..k)
                            -- else
                            --     log("\nSegmento final "..final_seg_k..": Inválido (Final NO Visto)".." MAESTRO: "..k)
                            -- end
                        end

                        -- Guardar validación
                        for i, pt in ipairs(track.segmentos.movement[final_seg_k]) do
                            table.insert(track.segmentos.valid[final_seg_k], is_valid) 
                        end
                    end

                    -- =========================================================================
                    -- C) NUEVO: PROCESAMIENTO DE DATOS (GENERACIÓN DE PARAMS)
                    -- =========================================================================
                    -- Solo procesamos si tenemos segmentos grabados
                    if #track.segmentos.movement > 0 then
                        --log("\n--- Iniciando Procesamiento de Trayectoria para Maestro "..k.." ---")

                        -- 1. Obtener Ecuaciones (apply)
                        -- Se asume que 'apply' devuelve 'equations'
                        local equations = apply(track.segmentos.movement)

                        -- 2. Calcular Intersecciones
                        -- Se asume que toma 'equations' y los segmentos originales
                        local intersections = calcular_intersecciones_con_extremos(equations, track.segmentos.movement)

                        -- 3. Calcular Distancias y Ángulos
                        -- Se asume que devuelve dos tablas: distancias y ángulos
                        local distancias_a_recorrer, angles_a_recorrer = calcular_distancias_y_angulos(intersections)

                        -- 4. Crear Parámetros Finales
                        -- Esta función devuelve la tabla 'params' estructurada
                        local generated_params = create_params(distancias_a_recorrer, angles_a_recorrer)

                        -- 5. ALMACENAR EN EL MAESTRO

                        track.params = generated_params
                       --log("\nMaestro "..k..": Procesamiento completado. Parametros guardados: "..#track.params)
                        
                    else
                        --log("Maestro "..k..": No hay segmentos para procesar.")
                        track.params = {} 
                    end

                    -- =========================================================================
                    -- D) NUEVO: PRE-PROCESAMIENTO NUMÉRICO (SIN LETRAS)
                    -- =========================================================================
                    -- Solo procesar si tenemos parámetros calculados
                    if #track.params > 0 then
                        
                        local structure_values = {}
                        local structure_type = nil -- 0: Empieza Dist, 1: Empieza Angulo
                        
                        -- Iteramos sobre los parámetros {distancia, angulo}
                        for i, param in ipairs(track.params) do
                            local distancia = param[1]
                            local angulo    = param[2]
                            
                            -- Verificar validez del segmento (usando tu lógica existente)
                            local is_segment_valid = false
                            if track.segmentos.valid[i] and track.segmentos.valid[i][1] then
                                is_segment_valid = true
                            end

                            -- REGLA 1: El ángulo se agrega si i > 1
                            if i > 1 then
                                table.insert(structure_values, angulo)
                                -- Si es el PRIMER dato que insertamos en la lista, definimos el TYPE
                                if structure_type == nil then structure_type = 1 end -- Empezó con Angulo
                            end

                            -- REGLA 2: La distancia se agrega si es válida
                            if is_segment_valid then
                                table.insert(structure_values, distancia)
                                -- Si es el PRIMER dato que insertamos, definimos el TYPE
                                if structure_type == nil then structure_type = 0 end -- Empezó con Distancia
                            end
                        end

                        -- Si logramos recolectar datos, enviarlos al Buffer
                        if #structure_values > 0 and structure_type ~= nil then
                            local data_packet = {
                                value = structure_values,
                                type = structure_type
                            }
                            
                            -- log(">> Enviando paquete al buffer: Tipo " .. structure_type .. ", Len " .. #structure_values)
                            DataBuffer.add(data_packet)
                        end
                        
                    end
                    -- =========================================================================


                    -- =========================================================================
                    track.printed = true -- Marcar como procesado/impreso
                end

                -- Limpiar datos del sensor (para que no se vuelva a procesar en el bucle principal)
                srt_anterior_data[k].angle = nil
                srt_anterior_data[k].distance = nil
                srt_anterior_data[k].type = nil
                
            end
        end -- Fin bucle 'k'

        -- =========================================================================
        -- 7. PROCESAR TRAYECTORIA DE MAESTROS ACTIVOS (¡MODIFICADO!)
        -- (Aplica la lógica de segmentación a cada maestro activo)
        -- =========================================================================
        
        -- Iterar sobre todos los maestros que fueron actualizados (amarillo o verde)
        for k, _ in pairs(updated_anterior_indices) do
            
            local master_data = srt_anterior_data[k]
            local track = master_tracking[k] -- Obtener la ESTRUCTURA de tracking

            -- Solo procesar si el maestro existe, tiene tracking,
            -- y NO es tipo 3 (finalizado)
            if master_data and track then

                -- Usar los datos actuales del maestro
                local current_distance = master_data.distance
                local current_angle_rad = master_data.angle * (math.pi / 180)
                local ty = master_data.type
                -- Calcular X, Y (asumiendo robot quieto en (0,0) y srt_ang_mark_ant = 0)
                -- (Usa los valores 'base' del tracking de ESE maestro)
                local ang_total = current_angle_rad + track.srt_ang_mark_ant
                local x_new, y_new = polarACartesianoPAR(current_distance, ang_total)
                
                local x_original = x_new + track.x_mark
                local y_original = y_new + track.y_mark

                -- Llamar a la nueva función de segmentación
                process_master_segmentation(k, track, x_original, y_original, ty)

            end -- fin 'if master_data and track...'
        end -- fin 'for k in updated_anterior_indices'

        -- =========================================================================
        -- 8. 
        -- =========================================================================
     

        -- =========================================================================
        -- 9.
        -- ========================================================================


    end -- fin 'if robot.colored_blob_omnidirectional_camera'
end


function detect_initial_or_end_segment()
    -- Esta función ahora está integrada en 'store_relative_positions'
    -- y se maneja dentro del bloque 8.

    -- Comprobar si el contador de segmentos ha cambiado
    if srt_counter_segments ~= srt_counter_segments_prev then
        --log("\nCambio detectado en srt_counter_segments (Antes: "..srt_counter_segments_prev..", Ahora: "..srt_counter_segments.."). Revisando banderas.")

        -- Recorrer master_tracking
        for k, tracking_data in ipairs(master_tracking) do
            -- Solo revisar la data del maestro que probablemente cambió (o todos, es más simple)
            local movement_size = #tracking_data.segmentos.movement
            --log("Maestro "..k..", tamaño de segmentos.movement: "..movement_size)
            if movement_size == 1 then -- 1. Si el tamaño de 'segmentos.movement' es 1
                local type_data = tracking_data.segmentos.type[1] -- Buscar si hay algún 1 o 3 en la tabla de tipos
                --log("Revisando maestro "..k..", segmento 1, tipos: "..table.concat(type_data, ", "))

                if type_data[1] == 1 and not tracking_data.Pattern_start_detected then
                    tracking_data.Pattern_start_detected = true -- Si es 1, activar la bandera de segmento inicial
                    --log("\n\nPattern_start_detected in maestro "..k.."\n\n")
                elseif type_data[#type_data] == 3 and not tracking_data.End_of_pattern_detected then
                    tracking_data.End_of_pattern_detected = true -- Si es 3, activar la bandera de segmento final
                    --log("\n\nEnd_of_pattern_detected in maestro "..k.."\n\n")
                else
                    -- Si el tamaño es 1 pero no contiene 1 ni 3, no hace nada (según tu lógica)
                end

            elseif movement_size > 1 then -- 2. Si el tamaño de 'segmentos.movement' es > 1
                -- (Revisar el último segmento guardado)
                local last_seg_k = #tracking_data.segmentos.movement
                local type_data = tracking_data.segmentos.type[last_seg_k]
                --log("Revisando maestro "..k..", segmento "..last_seg_k..", tipos: "..table.concat(type_data, ", "))

                if type_data[#type_data] == 3 and not tracking_data.End_of_pattern_detected then
                    tracking_data.End_of_pattern_detected = true -- Si es 3, activar la bandera de segmento final
                    --log("\n\nEnd_of_pattern_detected in maestro "..k.."\n\n")
                end
            end
        end -- fin for master_tracking

        -- Actualizar el valor anterior para el próximo tick
        srt_counter_segments_prev = srt_counter_segments

    end -- fin 'if srt_counter_segments ~= srt_counter_segments_prev'

end
-------------------------------------------------------------------------------------------------------------------------------------
-- Funciones de operaciones y calculos
-------------------------------------------------------------------------------------------------------------------------------------

equations = {}
distancias_a_recorrer = {} --Tabla para almacenar las distancias a recorrer
function apply(segmentos)

    equations = {}
    distancias_a_recorrer = {} --Tabla para almacenar las distancias a recorrer

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
                log("Robot " .. robot.id .. string.format(",  Columna %d: No se pudo calcular la regresión lineal", col).." - Datos: \n"..n.." puntos".." \nsum_x2: "..sum_x2.." \nsum_x*sum_x: "..sum_x*sum_x.."\ndenominator: "..denominator)
            end
        end
    end
    --c4 = 2
    return equations
end

intersections = {} -- Asegúrate de que 'intersections' esté definido en el ámbito correcto
function calcular_intersecciones_con_extremos(equations, segmentos)

    intersections = {} -- Asegúrate de que 'intersections' esté definido en el ámbito correcto

    local ordered_keys = {}

    for k in pairs(equations) do
        table.insert(ordered_keys, k)
    end
    table.sort(ordered_keys)

    -- Si no hay ecuaciones, no hay intersecciones que calcular
    if #ordered_keys == 0 then
        resetOBS()
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
    
    --c4 = 3 -- Ajusta el ámbito de 'c4' si es necesario.
    return intersections
end

angles_a_recorrer    = {}
function calcular_distancias_y_angulos(intersecciones)

    angles_a_recorrer    = {}
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
    --c4 = 4
    return distancias_a_recorrer, angles_a_recorrer
end

params = {}
function create_params(distancias_a_recorrer, angles_a_recorrer)
    params = {}  -- Reiniciar tabla de parámetros
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
    --c4 = 5
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

                    local omega_deg = math.min(math.abs(speed_g), 80)   -- máximo 80°/s
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
    stationary_tolerance = sensitivity     

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