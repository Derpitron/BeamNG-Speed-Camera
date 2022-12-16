-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

-- ORBIT CAMERA
-- Author: Derpitron

-- local sensors = require("sensors")

local M = {}
M.__index = M

print("theRealTest200")

local calculatedCamPos = vec3()
local camPos = vec3()
local targetPos = vec3()
local ref = vec3()

--todo: function to increase distance based on speed
function M:update(data)
  --Set the speedDist(ance) to be 0.001 metres * velocity limited to 150 m/s, with the effect maxxing out at 130 m/s
  local speedDist = 0.001 * math.min(data.vel:length()/150, 0.85)
  local dist = self.camDist

	calculatedCamPos:setScaled(dist + speedDist)

  camPos:set(push3(calculatedCamPos) + targetPos + self.orbitOffset)
  targetPos = data.veh:getBBCenter() - (data.pos + ref)
  self.camLastPos:set(camPos)

  self.camLastTargetPos:set(targetPos)
  self.camVel:set((push3(camPos) - self.camLastPos) / data.dt)
  self.camLastPos:set(camPos)
  self.camLastDist = dist

  -- application
  data.res.pos = vec3(camPos)
  data.res.targetPos:set(targetPos)

end
--todo: function to increase or decrease distance based on g force

return M