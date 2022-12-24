-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

-- Sense of Speed
-- Author: Derpitron

-- local sensors = require("lua/vehicle/sensors.lua")

local M = {}
M.dependencies = {"core_camera", "core_vehicle_manager"}

print("theRealTest200")

-- Constrain the value of var between lo(wer bound) and up(per bound)
local function constrain(lo, var, up)
  return math.max(math.min(var, up), lo)
end


-- Gets vehicle data from vehicle lua
local vehData = {}

local function getVehData(data)
  vehData = data
end

-- Runs every frame
local function onUpdate(dtSim, dtRaw)
  print("TheFunctionTest300")

  --DATA FETCHING:
  -- Get current vehicle's ID
  local vehID = be:getPlayerVehicleID(0)

  -- Get the current camera's name
  local activeCam = core_camera.getActiveCamName()

  if activeCam == 'orbit' and vehData then

    -- Get orbit camera data
    local camData = core_camera.getCameraDataById(vehID)['orbit']

    -- Recieve g-force sensor data. Note: vehData (should) be assigned from fetchData.lua if everything is set up properly.
    local gx2 = vehData.sensors.gx2
    local gy2 = vehData.sensors.gy2
    local gz2 = vehData.sensors.gz2


    --SETTING UP VARIABLES:
    -- Additional Position
    local addPos = vec3(0, 0, 0)
    -- Additional Rotation
    local addRot = vec3(0, 0, 0)
    -- Additional Field of View
    local addFOV = 0

    -- Coefficients for limiting g-force camera position.
    -- TODO: Get constraint values from user settings app
    local PosYCoeff = 0.95
    local PosZCoeff = 0.95


    -- ADDITIONAL CAMERA POSITION CALCULATION:
    -- Additional camera position based on g forces
    addPos = {
      0,
      PosYCoeff * constrain(-1.3, gy2 ,1.3),
      PosZCoeff * constrain(-1.3, gz2 - 1 ,1.3)
    }

    core_camera.setOffset(vehID, addPos)
  end

end

M.constrain = constrain
M.onUpdate = onUpdate
M.getVehData = getVehData

return M