-- avoid_scaled.lua
local FWD      = 8.0      -- velocidad base
local MAX_SPD  = 8.0
local PROX_THR = 0.1      -- umbral más bajo (antes 0.15/0.2)
local PROX_GAIN= 25.0      -- cuánto giro por lectura

local has_wheels = robot.wheels and robot.wheels.set_velocity
local has_leds   = robot.leds   and robot.leds.set_all_colors

local function clamp(x,a,b) if x<a then return a elseif x>b then return b else return x end end

function init()
  if has_leds then robot.leds.set_all_colors(0,128,255) end
end

function step()
  local prox = robot.qupa_proximity or robot.proximity or {}
  local torque, maxv = 0, 0
  for _,p in ipairs(prox) do
    local v = p.value or 0
    local a = p.angle or 0
    maxv   = math.max(maxv, v)
    torque = torque + v * math.sin(a)   -- suma repulsiva (izq/der)
  end

  local vL, vR = FWD, FWD
  if maxv > PROX_THR then
    local turn = clamp(PROX_GAIN * torque, -MAX_SPD, MAX_SPD)
    vL = FWD - turn
    vR = FWD + turn
    if has_leds then robot.leds.set_all_colors(255,80,0) end
  else
    if has_leds then robot.leds.set_all_colors(0,200,80) end
  end

  if has_wheels then
    robot.wheels.set_velocity(clamp(vL,-MAX_SPD,MAX_SPD), clamp(vR,-MAX_SPD,MAX_SPD))
  end

  -- debug: en el primer robot imprime magnitud y torque
  if robot.id == "q0" then
    log(string.format("[prox] max=%.3f torque=%.3f", maxv, torque))
  end
end

function reset()
  if has_leds then robot.leds.set_all_colors(0,0,0) end
  if has_wheels then robot.wheels.set_velocity(0,0) end
end

function destroy() end
