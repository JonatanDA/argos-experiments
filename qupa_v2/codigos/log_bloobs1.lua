-- camera_blobs_qupa.lua (Versión Selectividad Total)
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

-- Detecta blobs con coincidencia exacta de color
local function seen_color(target_color)
  local cam = robot.colored_blob_omnidirectional_camera
  local readings = cam.readings or cam
  if type(readings) ~= "table" then return false, 0.0, 0, 0.0 end

  local n, sx, sy, dist_sum = 0, 0.0, 0.0, 0.0
  
  for i = 1, #readings do
    local b = readings[i]
    local r, g, bl = b.color.red, b.color.green, b.color.blue
    
    -- LOG DE COLOR (Solo para el primer blob y el robot q0)
    if i == 1 and robot.id == "q0" and (robot.iterations % 10 == 0) then
       log(string.format("[DEBUG COLOR] R:%d G:%d B:%d | Dist:%.1f", r, g, bl, b.distance))
    end

    local is_match = false
    if target_color == "red"   then is_match = (r == 255 and g == 0 and bl == 0)
    elseif target_color == "blue"  then is_match = (r == 0 and g == 0 and bl == 255)
    elseif target_color == "green" then is_match = (r == 0 and g == 255 and bl == 0) end

    if is_match then
      sx = sx + math.cos(b.angle)
      sy = sy + math.sin(b.angle)
      dist_sum = dist_sum + b.distance
      n = n + 1
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

  -- Identidad visual: Cada robot brilla con su color para ser detectado
  if HAS.leds then
    local n = tonumber(robot.id:match("(%d+)$")) or 0
    if n == 0 then robot.leds.set_all_colors(255,0,0) -- Líder Rojo
    elseif n == 1 then robot.leds.set_all_colors(0,0,255) -- Seguidor Azul
    else robot.leds.set_all_colors(0,255,0) end -- Otros Verde
  end
end

function step()
  -- 1. Evasión de obstáculos (Lógica simplificada para pruebas)
  local prox = robot.proximity or robot.qupa_proximity
  local vx, vy = 0.0, 0.0
  local readings = prox.readings or prox
  
  for _, r in ipairs(readings) do
    vx = vx + r.value * math.cos(r.angle)
    vy = vy + r.value * math.sin(r.angle)
  end
  
  local mag = math.sqrt(vx*vx + vy*vy)
  local ang_prox = atan2(vy, vx)

  local vL, vR = BASE_SPEED, BASE_SPEED

  if mag > THRESHOLD then
    local turn = TURN_GAIN * (ang_prox > 0 and -1 or 1)
    vL = BASE_SPEED + turn
    vR = BASE_SPEED - turn
  else
    -- 2. Seguimiento del Líder Rojo
    local seen, bearing, count, dist_avg = seen_color("red")
    if seen then
      local gain = 4.0
      vL = BASE_SPEED - (gain * bearing)
      vR = BASE_SPEED + (gain * bearing)

      logerr(string.format("[FOLLOW] Blobs=%d, Dist=%.1f, Bearing=%.1f°", count, dist_avg, to_deg(bearing)))
    end
  end

  if HAS.wheels then
    local function clamp(x,a,b) if x<a then return a elseif x>b then return b else return x end end
    robot.wheels.set_velocity(clamp(vL,-15,15), clamp(vR,-15,15))
  end
end

function reset() init() end
function destroy() end