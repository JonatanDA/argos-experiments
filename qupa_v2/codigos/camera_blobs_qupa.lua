-- camera_blobs_qupa.lua  
local SPD_FWD  = 8.0
local SPD_TURN = 6.0
local PROX_THR = 0.02
local TARGET   = "blue"   -- "red" | "green" | "blue" | nil (nil = cualquier color)

local has_leds   = (robot.leds   and robot.leds.set_all_colors)
local has_wheels = (robot.wheels and robot.wheels.set_velocity)

local function my_color_from_id()
  local d = tonumber(string.match(robot.id, "(%d+)$")) or 0
  if d % 3 == 0 then return 255,0,0   -- rojo
  elseif d % 3 == 1 then return 0,255,0 -- verde
  else return 0,0,255                 -- azul
  end
end

local function is_match_color(b, want)
  if not want then return true end
  local r = (b.color and b.color.red)   or b.red   or 0
  local g = (b.color and b.color.green) or b.green or 0
  local bl= (b.color and b.color.blue)  or b.blue  or 0
  if want == "red"   then return r > 200 and g < 100 and bl < 100 end
  if want == "green" then return g > 200 and r < 100 and bl < 100 end
  if want == "blue"  then return bl> 200 and r < 100 and g  < 100 end
  return false
end

function init()
  if has_leds then
    local r,g,b = my_color_from_id()
    robot.leds.set_all_colors(r,g,b)
  end
  if robot.colored_blob_omnidirectional_camera
     and type(robot.colored_blob_omnidirectional_camera.enable)=="function" then
    robot.colored_blob_omnidirectional_camera.enable()
  end
end

function step()
  -- (1) seleccionar mejor blob (más cerca) usando cámara como ARRAY
  local blobs = robot.colored_blob_omnidirectional_camera
  local best, best_score, seen_count = nil, -1e9, 0
  if type(blobs)=="table" then
    for i=1,#blobs do
      local b = blobs[i]
      if is_match_color(b, TARGET) then
        seen_count = seen_count + 1
        local d = (b.distance and b.distance > 1e-6) and b.distance or 1e-6
        local score = 1.0 / d
        if score > best_score then
          best_score = score
          best = b
        end
      end
    end
  end
  -- log básico
  log(string.format("[%-4s] blobs(%s)=%d", robot.id, tostring(TARGET or "any"), seen_count))

  -- (2) evitar con proximidad
  local avoid = 0.0
  local prox = robot.qupa_proximity or robot.proximity
  if prox then
    local ax, ay = 0,0
    for _, p in ipairs(prox) do
      ax = ax + p.value * math.cos(p.angle)
      ay = ay + p.value * math.sin(p.angle)
    end
    local amag = math.sqrt(ax*ax + ay*ay)
    if amag > PROX_THR then
      avoid = (ay > 0 and -1 or 1) * SPD_TURN
    end
  end

  -- (3) control de ruedas
  local vL, vR = SPD_FWD, SPD_FWD
  if best then
    local ang = best.angle or 0
    local turn = math.max(-SPD_TURN, math.min(SPD_TURN, ang * 10.0))
    vL = SPD_FWD + turn - avoid
    vR = SPD_FWD - turn + avoid
    if has_leds then robot.leds.set_all_colors(255,0,0) end
  else
    -- exploración suave si no ve objetivo
    vL = SPD_FWD
    vR = SPD_FWD * 0.5
  end

  if has_wheels then
    local function clamp(x,a,b) if x<a then return a elseif x>b then return b else return x end end
    robot.wheels.set_velocity(clamp(vL,-15,15), clamp(vR,-15,15))
  end
end

function reset()
  if has_leds then robot.leds.set_all_colors(0,0,0) end
  if has_wheels then robot.wheels.set_velocity(0,0) end
end

function destroy()
  if robot.colored_blob_omnidirectional_camera
     and type(robot.colored_blob_omnidirectional_camera.disable)=="function" then
    robot.colored_blob_omnidirectional_camera.disable()
  end
end