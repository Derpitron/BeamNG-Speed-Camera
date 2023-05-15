-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

-- Fetch Data
-- Author: Derpitron

local M = {}
local lpack = require('lpack')

--Get vehicle data. Runs every frame
local function updateGFX(dt)
  --Table containing data of all vehicles
  local data = {}
  --ID of individual vehicle
  local vehicleID = obj:getID()

  --Sub-table containing the data for the specific vehicle of ID "vehicleID" 
  data[vehicleID] = {}

  --Vehicle velocity
  data[vehicleID].velocity = vec3(obj:getVelocity())

  --Vehicle sensors (g-force) data
  data[vehicleID].gforces = {}
  data[vehicleID].gforces.x = sensors.gx2/-9.81
  data[vehicleID].gforces.y = sensors.gy2/-9.81
  data[vehicleID].gforces.z = sensors.gz2/-9.81

  --Vehicle direction vectors
  data[vehicleID].vectors = {}
  data[vehicleID].vectors.forward = vec3(obj:getDirectionVector  ()):normalized()
  data[vehicleID].vectors.up      = vec3(obj:getDirectionVectorUp()):normalized()

  --Global coordinates of vehicle centre
  data[vehicleID].centrePosition = vec3(obj:getPosition())

  --Vehicle refnodes

  --Send data to GameEngine
  obj:queueGameEngineLua(string.format("GetVehicleData(%q)", lpack.encode(data)))
end

M.updateGFX = updateGFX

return M