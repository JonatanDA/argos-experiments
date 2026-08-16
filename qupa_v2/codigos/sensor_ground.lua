-- sensor_ground.lua
-- Random walk y detectar mancha negra en el suelo:
--   - EXPLORE: moverse al azar
--   - APPROACH: entra a la mancha 
--   - DWELL: se queda quieto X segundos
--   - LEAVE: sale de la mancha y vuelve a explorar

-------------------- parámetros fáciles de tunear --------------------
local TICKS_PER_SEC   = 10     -- ponlo igual que <experiment ticks_per_second="...">
local WAIT_SEC        = 5      -- tiempo quieto dentro de la mancha (seg)
local APPROACH_SEC    = 1.2    -- tiempo de “entrada” a la mancha (seg)
local DARK_THR        = 0.25   -- umbral: 0=negro, 1=blanco 
local SPD_FWD         = 7.0    -- avance normal
local SPD_TURN        = 6.0    -- giro
local SPD_SLOW        = 3.0    -- avance lento al entrar
local TURN_PROB       = 0.05   -- prob. de iniciar un giro aleatorio en EXPLORE
local TURN_TICKS_MIN  = 8      -- duración giro aleatorio (en ticks)
local TURN_TICKS_MAX  = 18
---------------------------------------------------------------------

local has_wheels = (robot.wheels and robot.wheels.set_velocity)
local has_leds   = (robot.leds   and robot.leds.set_all_colors)

-- estado
local state = "EXPLORE"
local approach_ticks = 0
local wait_ticks = 0
local turn_ticks = 0
local turn_sign  = 1

-- helpers
local function clamp(x,a,b) if x<a then return a elseif x>b then return b else return x end end

-- devuelve array plano de valores [0..1] del sensor de suelo 
local function ground_values()
  local g = robot.qupa_base_ground or robot.base_ground
            or robot.qupa_motor_ground or robot.motor_ground
  local vals = {}
  if not g then return vals end
  local src = g.readings or g
  if type(src)=="table" then
    if src.left or src.center or src.right then
      if src.left   then vals[#vals+1] = src.left.value   or src.left   or 0 end
      if src.center then vals[#vals+1] = src.center.value or src.center or 0 end
      if src.right  then vals[#vals+1] = src.right.value  or src.right  or 0 end
    else
      for i=1,#src do
        local r = src[i]
        vals[#vals+1] = (type(r)=="table" and r.value) or r or 0
      end
    end
  end
  return vals
end

-- decide si está sobre negro 
local function is_black(vals)
  if #vals == 0 then return false, 1.0, 1.0 end
  local minv, sum = 1.0, 0.0
  for i=1,#vals do
    local v = vals[i] or 1.0
    if v < minv then minv = v end
    sum = sum + v
  end
  local avg = sum / #vals
  return (minv < DARK_THR), minv, avg
end

local function set_led(r,g,b) if has_leds then robot.leds.set_all_colors(r,g,b) end end

function init()
  math.randomseed(os.time() + (tonumber(string.match(robot.id,"(%d+)$")) or 0))
  set_led(255,255,255) -- blanco: arranque
end

function step()
  local vals = ground_values()
  local on_black, vmin, vavg = is_black(vals)

  -- log lecturas
  if #vals > 0 then
    local s = {}
    for i=1,#vals do s[#s+1] = string.format("%.2f", vals[i]) end
    log(string.format("[%s] st=%s ground=%s | min=%.2f avg=%.2f",
      robot.id, state, table.concat(s," "), vmin, vavg))
  else
    log(string.format("[%s] st=%s ground=NO_READINGS", robot.id, state))
  end

  local vL, vR = 0, 0

  if state == "EXPLORE" then
    -- si ve negro: entrar
    if on_black then
      state = "APPROACH"
      approach_ticks = math.floor(APPROACH_SEC * TICKS_PER_SEC)
      set_led(0,0,255) -- azul: entrando
      vL, vR = SPD_SLOW, SPD_SLOW
    else
      -- random walk
      if turn_ticks > 0 then
        vL, vR =  SPD_TURN*turn_sign, -SPD_TURN*turn_sign
        turn_ticks = turn_ticks - 1
      else
        vL, vR = SPD_FWD, SPD_FWD
        if math.random() < TURN_PROB then
          turn_sign = (math.random() < 0.5) and -1 or 1
          turn_ticks = math.random(TURN_TICKS_MIN, TURN_TICKS_MAX)
        end
      end
      set_led(0,255,0) -- verde: explorando
    end

  elseif state == "APPROACH" then
    -- avanzar lento para meterse dentro
    vL, vR = SPD_SLOW, SPD_SLOW
    approach_ticks = approach_ticks - 1
    if approach_ticks <= 0 then
      state = "DWELL"
      wait_ticks = math.floor(WAIT_SEC * TICKS_PER_SEC)
      vL, vR = 0, 0
      set_led(255,0,0) -- rojo: esperando
    end

  elseif state == "DWELL" then
    -- quedarse quieto
    vL, vR = 0, 0
    wait_ticks = wait_ticks - 1
    if wait_ticks <= 0 then
      state = "LEAVE"
      set_led(255,255,0) -- amarillo: saliendo
    end

  elseif state == "LEAVE" then
    -- salir: si sigue negro, avanza; si ya no, vuelve a explorar
    if on_black then
      vL, vR = SPD_FWD, SPD_FWD
    else
      state = "EXPLORE"
      turn_ticks = 0
      set_led(0,255,0)
      vL, vR = SPD_FWD, SPD_FWD
    end
  end

  if has_wheels then
    robot.wheels.set_velocity(clamp(vL,-15,15), clamp(vR,-15,15))
  end
end

function reset()
  set_led(0,0,0)
  if has_wheels then robot.wheels.set_velocity(0,0) end
  state, approach_ticks, wait_ticks, turn_ticks = "EXPLORE", 0, 0, 0
end

function destroy() end
