-- Use Shift + Click to select a robot
-- When a robot is selected, its variables appear in this editor

-- Use Ctrl + Click (Cmd + Click on Mac) to move a selected robot to a different location



-- Put your global variables here



--[[ This function is executed every time you press the 'execute' button ]]
function init()
   -- put your code here
end



--[[ This function is executed at each time step
     It must contain the logic of your controller ]]
function step()
   -- p utyour code here
	--x = robot.proximity[1].value
	--logerr("value 1:" .. x)
	--[[for i=1, 6 do
		x = robot.proximity[i].value
		logerr("index-value:" .. x)
	end]]


  local P = robot.qupa_proximity or {}
  logerr(string.format("n_sens=%d", #P))
  for i=1,6 do
    local s = robot.proximity[i]
    logerr(string.format("i=%d v=%.3f ang=%.1f°",
      i, s.value or -1, math.deg(s.angle or 0)))
  end

end



--[[ This function is executed every time you press the 'reset'
     button in the GUI. It is supposed to restore the state
     of the controller to whatever it was right after init() was
     called. The state of sensors and actuators is reset
     automatically by ARGoS. ]]
function reset()
   -- put your code here
end



--[[ This function is executed only once, when the robot is removed
     from the simulation ]]
function destroy()
   -- put your code here
end
