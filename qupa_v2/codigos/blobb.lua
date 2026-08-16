local BASE_SPEED, TURN_GAIN, THRESHOLD = 8.0, 6.0, 0.15

local HAS = {}

local function to_deg(rad) return rad * 180 / math.pi end

local function atan2(y, x)
  if math.atan2 then return math.atan2(y, x) end
  if x == 0 then return (y >= 0 and math.pi/2 or -math.pi/2) end
  local a = math.atan(y/x)
  if x < 0 then a = a + math.pi end
  return a
end

-- Función de Clustering: Agrupa blobs por distancia para identificar "Robots" individuales
local function seen_color(target_color)
  local cam = robot.colored_blob_omnidirectional_camera
  local readings = cam.readings or cam
  if type(readings) ~= "table" then return false, 0.0, 0, 0.0 end

  local detected_robots = {} 
  local CLUSTER_THRESHOLD = 15.0 -- cm de tolerancia para agrupar LEDs en un solo robot

  for i = 1, #readings do
    local b = readings[i]
    local r, g, bl = b.color.red, b.color.green, b.color.blue
    
    local is_match = false
    if target_color == "red"   then is_match = (r == 255 and g == 0 and bl == 0)
    elseif target_color == "blue"  then is_match = (r == 0 and g == 0 and bl == 255)
    elseif target_color == "green" then is_match = (r == 0 and g == 255 and bl == 0) end

    if is_match then
      local found_group = false
      -- Buscamos si este LED pertenece a un robot que ya empezamos a trackear
      for _, robot_obj in ipairs(detected_robots) do
        if math.abs(robot_obj.dist - b.distance) < CLUSTER_THRESHOLD then
          robot_obj.sx = robot_obj.sx + math.cos(b.angle)
          robot_obj.sy = robot_obj.sy + math.sin(b.angle)
          robot_obj.count = robot_obj.count + 1
          found_group = true
          break
        end
      end
      
      if not found_group then
        table.insert(detected_robots, {
          dist = b.distance, 
          sx = math.cos(b.angle), 
          sy = math.sin(b.angle), 
          count = 1
        })
      end
    end
  end

  -- Si encontramos al menos un "Robot" (grupo de LEDs)
  if #detected_robots > 0 then
    local best_sx, best_sy, best_dist = 0, 0, 0
    -- Para este ejemplo, seguimos al robot más cercano
    table.sort(detected_robots, function(a,b) return a.dist < b.dist end)
    local target = detected_robots[1]
    
    local final_bearing = atan2(target.sy, target.sx)
    return true, final_bearing, #detected_robots, target.dist
  end

  return false, 0.0, 0, 0.0
end

function init()
  -- 1. Verificación de Actuadores
  HAS.wheels = robot.wheels and type(robot.wheels.set_velocity) == "function"
  HAS.leds   = robot.leds and type(robot.leds.set_all_colors) == "function"

  -- 2. Habilitar Cámara
  if robot.colored_blob_omnidirectional_camera and
     type(robot.colored_blob_omnidirectional_camera.enable) == "function" then
    robot.colored_blob_omnidirectional_camera.enable()
  end

  -- 3. Identidad Visual Agnóstica
  if HAS.leds then
    -- Extraemos el número del ID (sea q0, robot1, qupa_10, etc.)
    local id_num = tonumber(robot.id:match("(%d+)$")) or 0
    
    -- Lógica de colores por turnos (Agnóstica):
    -- Robot 0, 3, 6... -> ROJO (Líderes)
    -- Robot 1, 4, 7... -> AZUL (Seguidores)
    -- Robot 2, 5, 8... -> VERDE (Otros)
    local remainder = id_num % 3
    
    if remainder == 0 then
      robot.leds.set_all_colors(255, 0, 0)   -- Rojo puro
    elseif remainder == 1 then
      robot.leds.set_all_colors(0, 0, 255)   -- Azul puro
    else
      robot.leds.set_all_colors(0, 255, 0)   -- Verde puro
    end
  end
end

function step()
  -- 1. Evasión de obstáculos
  local prox = robot.proximity or robot.qupa_proximity
  local vx, vy = 0.0, 0.0
  local prox_readings = prox.readings or prox
  
  for _, r in ipairs(prox_readings) do
    vx = vx + r.value * math.cos(r.angle)
    vy = vy + r.value * math.sin(r.angle)
  end
  
  local mag = math.sqrt(vx*vx + vy*vy)
  local ang_prox = atan2(vy, vx)

  local vL, vR = BASE_SPEED, BASE_SPEED

  if mag > THRESHOLD then
    local turn = TURN_GAIN * (ang_prox > 0 and -1 or 1)
    vL, vR = BASE_SPEED + turn, BASE_SPEED - turn
  else
    -- 2. Seguimiento de Líder (Agnóstico)
    -- El robot busca cualquier grupo de LEDs rojos (un "Robot Rojo")
    local seen, bearing, robot_count, dist_avg = seen_color("red")
    
    if seen then
      local gain = 4.0
      vL = BASE_SPEED - (gain * bearing)
      vR = BASE_SPEED + (gain * bearing)

      -- Log para saber cuántos ROBOTS (no blobs) hay cerca
      log(string.format("Viendo %d robot(s) rojo(s). Rumbo: %.1f°, Dist: %.1f", 
          robot_count, to_deg(bearing), dist_avg))
    end
  end

  if HAS.wheels then
    local function clamp(x,a,b) return math.min(math.max(x, a), b) end
    robot.wheels.set_velocity(clamp(vL,-15,15), clamp(vR,-15,15))
  end
end