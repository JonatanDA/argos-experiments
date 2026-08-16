-- camera_blobs_qupa.lua  (versión "array de blobs")
local BASE_SPEED, TURN_GAIN, THRESHOLD = 8.0, 6.0, 0.01

local HAS = {}

-- Por si tu Lua no trae atan2
local function atan2(y, x)
  if math.atan2 then return math.atan2(y, x) end
  if x == 0 then return (y >= 0 and math.pi/2 or -math.pi/2) end
  local a = math.atan(y/x)
  if x < 0 then a = a + math.pi end
  return a
end

-- Detecta si hay blobs del color pedido y devuelve (seen, ang_prom, n)
local function seen_color(color)
  local blobs = robot.colored_blob_omnidirectional_camera
  if type(blobs) ~= "table" then return false, 0.0, 0 end

  local n, sx, sy = 0, 0.0, 0.0
  for i = 1, #blobs do
    local b = blobs[i]
    local r = (b.color and b.color.red)   or b.red   or 0
    local g = (b.color and b.color.green) or b.green or 0
    local bl= (b.color and b.color.blue)  or b.blue  or 0
    local is_match =
      (color == "red"   and r > 200 and g < 100 and bl < 100) or
      (color == "green" and g > 200 and r < 100 and bl < 100) or
      (color == "blue"  and bl> 200 and r < 100 and g  < 100)

    if is_match then
      local ang = b.angle or 0.0   -- bearing del blob en rad
      sx = sx + math.cos(ang)
      sy = sy + math.sin(ang)
      n = n + 1
    end
  end
  if n > 0 then
    return true, atan2(sy, sx), n
  else
    return false, 0.0, 0
  end
end

function init()
  HAS.wheels = robot.wheels and type(robot.wheels.set_velocity)=="function"
  HAS.leds   = robot.leds   and type(robot.leds.set_all_colors)=="function"

  -- Intentar habilitar cámara si la interfaz lo permite
  if robot.colored_blob_omnidirectional_camera and
     type(robot.colored_blob_omnidirectional_camera.enable)=="function" then
    robot.colored_blob_omnidirectional_camera.enable()
  end

  -- Colorea cada robot distinto para “dar algo que ver”
  if HAS.leds then
    local n = tonumber(robot.id:match("(%d+)$")) or 0
    local palette={{255,0,0},{0,255,0},{0,0,255},{255,255,0},{0,255,255},{255,0,255}}
    local c=palette[(n % #palette) + 1]
    robot.leds.set_all_colors(c[1],c[2],c[3])
  end
end

function step()
  -- (1) Evitar obstáculos
  local prox = robot.qupa_proximity or robot.proximity
  local vx, vy = 0.0, 0.0
  if prox then
    for _, r in ipairs(prox) do
      vx = vx + r.value * math.cos(r.angle)
      vy = vy + r.value * math.sin(r.angle)
    end
  end
  local mag = math.sqrt(vx*vx + vy*vy)
  local ang = atan2(vy, vx)

  local vL, vR = BASE_SPEED, BASE_SPEED
  if mag > THRESHOLD then
    local turn = TURN_GAIN * (ang > 0 and -1 or 1)
    vL = BASE_SPEED + turn
    vR = BASE_SPEED - turn
  end

  -- (2) Usar cámara seguir ROJO 
  local seen, bearing, count = seen_color("red")
  log(string.format("[id=%s] blobs_rojos=%d", robot.id, count))
  if seen then
    -- Pequeño giro hacia el blob promedio
    local gain = 4.0
    vL = BASE_SPEED - gain * bearing
    vR = BASE_SPEED + gain * bearing
    if HAS.leds then robot.leds.set_all_colors(255,0,0) end
  end

  if HAS.wheels then
    local function clamp(x,a,b) if x<a then return a elseif x>b then return b else return x end end
    robot.wheels.set_velocity(clamp(vL,-15,15), clamp(vR,-15,15))
  end
end

function reset()
  if HAS.leds   then robot.leds.set_all_colors(0,0,0) end
  if HAS.wheels then robot.wheels.set_velocity(0,0) end
end

function destroy()
  if robot.colored_blob_omnidirectional_camera and
     type(robot.colored_blob_omnidirectional_camera.disable)=="function" then
    robot.colored_blob_omnidirectional_camera.disable()
  end
end
