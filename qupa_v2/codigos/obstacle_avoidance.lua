-- avoid_scaled.lua corregido para evitar colisiones frontales y laterales
local FWD      = 8.0      -- velocidad base
local MAX_SPD  = 8.0
local PROX_THR = 0.1      -- Bajamos el umbral para que reaccione antes (20% de proximidad)
local PROX_GAIN= 15.0     -- Ajustamos ganancia para un giro más suave pero firme

local has_wheels = robot.wheels and robot.wheels.set_velocity
local has_leds   = robot.leds   and robot.leds.set_all_colors

local function clamp(x,a,b) if x<a then return a elseif x>b then return b else return x end end

function init()
  if has_leds then robot.leds.set_all_colors(0,128,255) end
end

function step()
  local prox = robot.proximity or robot.qupa_proximity or {}
  local readings = prox.readings or prox
  
  local torque = 0
  local maxv = 0
  local front_v = 0

  -- 1. Procesamos los sensores
  for i, p in ipairs(readings) do
    local v = p.value or 0
    local a = p.angle or 0
    
    maxv = math.max(maxv, v)
    
    -- Guardamos el valor del sensor frontal (índice 1 / 0 grados)
    if i == 1 then front_v = v end

    -- Torque: Invertimos el signo del seno para que la repulsión sea lógica
    -- Si el objeto está a la derecha (ángulo negativo en ARGoS), torque positivo (gira a la izq)
    torque = torque - v * math.sin(a) 
  end

  -- 2. Lógica de Evasión
  local vL, vR = FWD, FWD
  
  if maxv > PROX_THR then
    -- ANTI-BLOQUEO FRONTAL: Si el obstáculo es muy frontal, el torque es casi 0.
    -- Forzamos un torque artificial para que el robot sepa a dónde tirar.
    if front_v > PROX_THR and math.abs(torque) < 0.05 then
      torque = 0.5 -- Gira a la izquierda por defecto
    end

    local turn = clamp(PROX_GAIN * torque, -MAX_SPD, MAX_SPD)
    vL = FWD - turn
    vR = FWD + turn
    
    if has_leds then robot.leds.set_all_colors(255,80,0) end -- Naranja: Evitando
  else
    if has_leds then robot.leds.set_all_colors(0,200,80) end -- Verde: Libre
  end

  -- 3. Aplicar motores
  if has_wheels then
    robot.wheels.set_velocity(clamp(vL,-MAX_SPD,MAX_SPD), clamp(vR,-MAX_SPD,MAX_SPD))
  end

  log(string.format("[prox] max=%.3f torque=%.3f vL=%.1f vR=%.1f", maxv, torque, vL, vR))

end

function reset()
  if has_leds then robot.leds.set_all_colors(0,0,0) end
  if has_wheels then robot.wheels.set_velocity(0,0) end
end

function destroy() end