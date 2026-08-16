-- qupa_move.lua: simple reactive controller for QUPA
-- Moves forward; if proximity detects an obstacle, turns in place.

local wheels = robot.wheels or robot.qupa_wheels
local prox   = robot.proximity or robot.qupa_proximity

function init()
  if wheels and wheels.set_velocity then
    wheels.set_velocity(8, 8)
  end
end

function step()
  local s = prox or robot.proximity
  local bump = 0
  if s and s.readings then
    for i = 1, #s.readings do
      local v = s.readings[i].value or 0
		log("INFO: s[i] = " .. v)
		logerr("value: s[i] = " .. v)
      if v > bump then bump = v end
    end
  end

  if wheels and wheels.set_velocity then
    if bump > 0.15 then
      wheels.set_velocity(8, -8) -- turn to avoid
    else
      wheels.set_velocity(8, 8)  -- forward
    end
  end
end

function reset()
  if wheels and wheels.set_velocity then
    wheels.set_velocity(0, 0)
  end
end

function destroy()
  if wheels and wheels.set_velocity then
    wheels.set_velocity(0, 0)
  end
end