-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local core_camera = require('/lua/ge/extensions/core/camera.lua')
local sensors = require('/lua/vehicle/sensors.lua')

local M = {}
M.__index = M
print("Arcade script loaded :brain:")
-- Utility function to constrain a value(val) between two defined limits, the lo(wer)Bound, and hi(gher)Bound
local function constrain(val, loBound, hiBound)
  return math.max(math.min(val, hiBound), loBound)
end

-- Main updating function
function M:update(data, dtReal, dtSim)
  -- Gets the ID of the current vehicle
  local vID = be:getPlayerVehicleID(0)
  --Gets the camera data of the current vehicle
  local camData = core_camera.getCameraDataById(vID).orbit
  -- Additional distance applied on speed and G-force
  local addDist = vec3(0, 0, 0)

  -- Calculate the distance to be added 
  if (not self.target) and self.camDist then
    -- vector of addDist is equal to [ 0i, (1% of speed in m/s limited to 110 m/s) + (2/3 * of y-axis g force limited to 1.5 in either direction) )j, ( (z-axis g force - 1) limited to 1 in either direction )k ]
    addDist = {
      0,
      (0.01 * math.min(data.vel:length(), 110)) + (2/3 * constrain(sensors.gy2, -1.5, 1.5)),
      constrain(sensors.gy2 - 1, -1, 1)
    }
  end
  -- Add the calculated extra distance(addDist) to the calculated camera Position (camData.calculatedCamPos), to be further processed and used by orbit.lua
  camData.calculatedCamPos = camData.calculatedCamPos + addDist
  -- Add the calculated extra distance(addDist) to the camera Position (data.res.pos), to be further processed and used by orbit.lua. This line is in for testing.
  --TODO: Find out what `res` is
  data.res.pos = data.res.pos + addDist
end

M.constrain = constrain
M.update = update

-- DO NOT CHANGE CLASS IMPLEMENTATION BELOW

return M