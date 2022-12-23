-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

-- Sense of Speed
-- Author: Derpitron

local sensors = require("sensors")

local M = {}
M.__index = M

print("theRealTest200")

-- Constrain the value of var between lo(wer bound) and up(per bound)
local function constrain(lo, var, up)
  return math.max(math.min(var, up), lo)
end

function M:update(data, dtReal, dtSim)
  -- Recieve camera data
  local camData = data
  -- Recieve g-force sensor data
  local gx2 = sensors.gx2
  local gy2 = sensors.gy2
  local gz2 = sensors.gz2

  -- Set default values for additional Pos(ition), Rot(ation), and FOV(Field of View) values
  local newPos = vec3(0, 0, 0)
  local newRot = vec3(0, 0, 0)
  local newFOV = 0

  -- ADDITIONAL CAMERA POSITION:
  -- Coefficients for limiting g-force camera position.
  -- TODO: Get coefficient values from user settings app
  local PosYCoeff = 0.95
  local PosZCoeff = 0.95

  -- Additional camera position based on g forces
  -- TODO: Get constraint values from user settings app
  newPos = {
    0,
    PosYCoeff * constrain(-1.3, gy2 ,1.3),
    PosZCoeff * constrain(-1.3, gz2 - 1 ,1.3)
  }

end

return function(...)
  local o = ... or {}
  setmetatable(o, M)
  o:init()
  return o
end