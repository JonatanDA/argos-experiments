-- qupa_avoid.lua
local BASE_SPEED = 10.0     -- velocidad base (cm/s aprox)
local TURN_GAIN  = 8.0     -- cuanto gira ante obstáculo
local THRESHOLD  = 0.1    -- umbral de "hay algo cerca"

-- helpers: presencia de APIs según el robot
local has_wheels = (robot.wheels and robot.wheels.set_velocity)
local has_leds   = (robot.leds   and robot.leds.set_all_colors)

function init()
  -- nada especial que hacer
end

function step()
  -- 1) leer proximidad (QUPA o genérico)
  local prox = robot.qupa_proximity or robot.proximity
  local vx, vy = 0.0, 0.0
  if prox then
    for i, r in ipairs(prox) do
      vx = vx + r.value * math.cos(r.angle)
      vy = vy + r.value * math.sin(r.angle)
    end
  end
  local mag = math.sqrt(vx*vx + vy*vy)
  local ang = math.atan(vy, vx)  -- atan(y,x) en Lua 5.3

  -- 2) decisión de movimiento
  local vL, vR = BASE_SPEED, BASE_SPEED
  if mag > THRESHOLD then
    -- obstáculo: si está más a la izquierda (ang>0), girar a la derecha, y viceversa
    local turn = TURN_GAIN * (ang > 0 and -1 or 1)
    vL = BASE_SPEED + turn
    vR = BASE_SPEED - turn
    if has_leds then robot.leds.set_all_colors(255, 0, 0) end -- rojo
  else
    if has_leds then robot.leds.set_all_colors(0, 255, 0) end -- verde
  end

  -- 3) aplicar velocidades
  if has_wheels then
    -- clamp suave (por si acaso)
    local function clamp(x, a, b) if x < a then return a elseif x > b then return b else return x end end
    vL = clamp(vL, -8, 8); vR = clamp(vR, -8, 8)
    robot.wheels.set_velocity(vL, vR)
  end
end

function reset()
  if has_leds then robot.leds.set_all_colors(0, 0, 0) end
  if has_wheels then robot.wheels.set_velocity(0, 0) end
end

function destroy()
  -- nada
end