-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

-- Fetch Data
-- Author: Derpitron

local M = {}

print("TheVLuaTest400")

--Get vehicle data. Runs every frame
local function updateGFX(dt)

  print("TheVLuaFuncTest500")

  local data = {}
  data.vel = vec3(obj:getVelocity())

  --Get vehicle sensors data
  data.sensors = {}
  data.sensors.gx2 = sensors.gx2
  data.sensors.gy2 = sensors.gy2
  data.sensors.gz2 = sensors.gz2

  --Send data to GameEngine
  obj:queueGameEngineLua("getVehData("..serialize(data)..")")
end

M.updateGFX = updateGFX

return M