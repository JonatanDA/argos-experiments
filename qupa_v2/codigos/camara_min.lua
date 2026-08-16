local SPD=8.0

function init()
  if robot.leds then
    local n = tonumber(robot.id:match("(%d+)$")) or 0
    local colors={{255,0,0},{0,255,0},{0,0,255},{255,255,0},{0,255,255},{255,0,255}}
    local c=colors[(n%#colors)+1]
    robot.leds.set_all_colors(c[1],c[2],c[3])
  end
  if robot.colored_blob_omnidirectional_camera then
    robot.colored_blob_omnidirectional_camera.enable()   -- << CLAVE
  end
end

function step()
  local cam = robot.colored_blob_omnidirectional_camera
  local blobs = (cam and cam.readings) or {}
  log(string.format("[%s] blobs=%d", robot.id, #blobs))

  local vL, vR = SPD, SPD
  local b = blobs[1]
  if b and b.angle then
    local turn = math.max(-6, math.min(6, b.angle*8))
    vL, vR = SPD+turn, SPD-turn
  end
  if robot.wheels then robot.wheels.set_velocity(vL, vR) end
end

function reset() if robot.wheels then robot.wheels.set_velocity(0,0) end end
function destroy() end
