-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

-- Sense of Speed
-- Author: Derpitron

-- local sensors = require("lua/vehicle/sensors.lua")

local M = {}
M.dependencies = {"core_camera", "core_vehicle_manager"}
local lpack = require('lpack')

-- Constrain the value of var between lo(wer bound) and up(per bound)
local function clamp(lo, var, up)
  return math.max(math.min(var, up), lo)
end

-- Gets vehicle data from vehicle lua
local data = {}
function GetVehicleData(inputData)
  data = lpack.decode(inputData)
end

-- Runs every frame
local function onUpdate(dtSim, dtRaw)
  -- Get current vehicle's ID

  -- Get current active camera's name
  local activeCamName = core_camera.getActiveCamName()

  -- `next()` function checks whether a table is empty.
  -- Taken from: https://stackoverflow.com/a/1252776
  local next = next
  for vehicleID, vehicleData in pairs(data) do
    if activeCamName == 'orbit' and next(vehicleData) then
      -- Get orbit camera data
      local cameraData = core_camera.getCameraDataById(vehicleID)['orbit']
    
      -- Recieve vehicle data. Note: overall data should be assigned from fetchData.lua if everything is set up properly.
      
      --g-forces experienced by the vehicle
      local gforces = vec3()
      gforces.x = vehicleData.gforces.x
      gforces.y = vehicleData.gforces.y
      gforces.z = vehicleData.gforces.z
    
      --normalized orientation vectors of the vehicle, for forward and upward directions respectively.
      local vectors     = {}
      vectors.forward   = vehicleData.vectors.forward
      vectors.up        = vehicleData.vectors.up
      
      local camPosition = cameraData.camLastPos
    
      --local desiredOffsetCoordinates = vec3(0, 1, 0)
    
      local desiredGlobalOffsetCoordinates = camPosition + (vectors.forward * 1) + (vectors.up * 0)

      --dump(cameraData)

      local newRot = vec3(0, -10, 0)

      dump(cameraData)
    
      --core_camera.setOffset(vehicleID, desiredGlobalOffsetCoordinates)
    end
  end
end

-- Export module functions
M.constrain = clamp
M.onUpdate = onUpdate
M.GetVehicleData = GetVehicleData

return M