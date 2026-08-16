-- camera_blobs_qupa.lua (Versión Debug Extremo)
local BASE_SPEED, TURN_GAIN, THRESHOLD = 8.0, 6.0, 0.15 -- Subimos threshold para evitar ruidos

local HAS = {}

-- Función auxiliar para grados
local function to_deg(rad) return rad * 180 / math.pi end

local function atan2(y, x)
  if math.atan2 then return math.atan2(y, x) end
  if x == 0 then return (y >= 0 and math.pi/2 or -math.pi/2) end
  local a = math.atan(y/x)
  if x < 0 then a = a + math.pi end
  return a
end

-- Detecta blobs y escupe DATA pura a la terminal
local function seen_color(color)
  local cam = robot.colored_blob_omnidirectional_camera
  local readings = cam.readings or cam -- Compatibilidad con distintas versiones
  
  if type(readings) ~= "table" then return false, 0.0, 0, 0.0 end

  local n, sx, sy, dist_sum = 0, 0.0, 0.0, 0.0
  
  for i = 1, #readings do
    local b = readings[i]
    local r = (b.color and b.color.red)   or b.red   or 0
    local g = (b.color and b.color.green) or b.green or 0
    local bl= (b.color and b.color.blue)  or b.blue  or 0
    
    local is_match =
      (color == "red"   and r > 200 and g < 100 and bl < 100) or
      (color == "green" and g > 200 and r < 100 and bl < 100) or
      (color == "blue"  and bl > 200 and r < 100 and g < 100)

    if is_match then
      local ang = b.angle or 0.0
      local dist = b.distance or 0.0
      sx = sx + math.cos(ang)
      sy = sy + math.sin(ang)
      dist_sum = dist_sum + dist
      n = n + 1
      
      -- LOG DETALLADO POR BLOB (Solo para el robot q0 para no saturar)
    log(string.format("  [DETECCIÓN] %s #%d: Dist=%.2f cm, Ang=%.1f°", color, i, dist, to_deg(ang)))
    end
  end

  if n > 0 then
    return true, atan2(sy, sx), n, (dist_sum / n)
  else
    return false, 0.0, 0, 0.0
  end
end

function init()
  HAS.wheels = robot.wheels and type(robot.wheels.set_velocity)=="function"
  HAS.leds   = robot.leds   and type(robot.leds.set_all_colors)=="function"

  if robot.colored_blob_omnidirectional_camera and
     type(robot.colored_blob_omnidirectional_camera.enable)=="function" then
    robot.colored_blob_omnidirectional_camera.enable()
  end

  -- Identidad visual
  if HAS.leds then
    local n = tonumber(robot.id:match("(%d+)$")) or 0
    local palette={{255,0,0},{0,255,0},{0,0,255},{255,255,0}}
    local c=palette[(n % #palette) + 1]
    robot.leds.set_all_colors(c[1],c[2],c[3])
  end
end

function step()
  -- (1) Evasión por Vectores (Lógica Braitenberg)
  local prox = robot.qupa_proximity or robot.proximity
  local vx, vy = 0.0, 0.0
  local readings = prox.readings or prox
  
  for _, r in ipairs(readings) do
    vx = vx + r.value * math.cos(r.angle)
    vy = vy + r.value * math.sin(r.angle)
  end
  
  local mag = math.sqrt(vx*vx + vy*vy)
  local ang_prox = atan2(vy, vx)

  local vL, vR = BASE_SPEED, BASE_SPEED
  local current_state = "WANDER"

  -- Prioridad 1: Evitar Choque
  if mag > THRESHOLD then
    current_state = "AVOIDING"
    local turn = TURN_GAIN * (ang_prox > 0 and -1 or 1)
    vL = BASE_SPEED + turn
    vR = BASE_SPEED - turn
  else
    -- Prioridad 2: Seguir al "Líder" Rojo
    local seen, bearing, count, dist_avg = seen_color("red")
    if seen then
      current_state = "FOLLOWING_RED"
      local gain = 4.0
      vL = BASE_SPEED - (gain * bearing)
      vR = BASE_SPEED + (gain * bearing)
      
      -- LOG DE SEGUIMIENTO
      if robot.id == "q0" then
        logerr(string.format("[STATE=%s] Blobs=%d, DistProm=%.1f cm, Bearing=%.1f°", 
               current_state, count, dist_avg, to_deg(bearing)))
      end
    end
  end

  -- (3) Actuadores con Clamping
  if HAS.wheels then
    local function clamp(x,a,b) if x<a then return a elseif x>b then return b else return x end end
    robot.wheels.set_velocity(clamp(vL,-15,15), clamp(vR,-15,15))
  end
  
  -- Log de estado general en la terminal de ARGoS
  -- log(string.format("Robot %s: %s | MagProx: %.3f", robot.id, current_state, mag))
end

function reset() init() end
function destroy() end