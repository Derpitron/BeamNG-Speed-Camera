-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

-- Sense of Speed
-- Author: Derpitron

-- local sensors = require("lua/vehicle/sensors.lua")

local M = {}
M.dependencies = {"core_camera", "core_vehicle_manager"}
local lpack = require('lpack')


--print("theRealTest200")

-- Constrain the value of var between lo(wer bound) and up(per bound)
local function constrain(lo, var, up)
  return math.max(math.min(var, up), lo)
end


-- Gets vehicle data from vehicle lua
local vehData = {}
function GetVehData(data)
  vehData = lpack.decode(data)
end

-- Runs every frame
local function onUpdate(dtSim, dtRaw)
  --print("TheFunctionTest300")

  --DATA FETCHING:
  -- Get current vehicle's ID
  local vehID = be:getPlayerVehicleID(0)

  -- Get the current camera's name
  local activeCam = core_camera.getActiveCamName()

  -- Used for checking whether a table is empty.
  -- Taken from: https://stackoverflow.com/a/1252776/19195633
  local next = next
  -- If the active camera is orbit and vehData is NOT empty
  if activeCam == 'orbit' and next(vehData) then

    -- Get orbit camera data
    local camData = core_camera.getCameraDataById(vehID)['orbit']

    -- Recieve vehicle data. Note: vehData (should) be assigned from fetchData.lua if everything is set up properly.
    local gx2 = vehData.sensors.gx2
    local gy2 = vehData.sensors.gy2
    local gz2 = vehData.sensors.gz2
    local dirVec = vehData.dirVec
    local dirVecUp = vehData.dirVecUp


    --SETTING UP VARIABLES:
    -- Additional Position
    local addPos = vec3(0, 0, 0)
    -- Additional Rotation
    local addRot = vec3(0, 0, 0)
    -- Additional Field of View
    local addFOV = 0

    --core_camera.setDefaultRotation(vehID, vec3(0, -13, 0))
    --addPos = {
    --  0,
    --  dirVec   * gy2 * 100000,
    --  dirVecUp * gz2 * 100000
    --}

    --addRot = {
    --  -1 * gx2/5,
    --  -1 * gy2/5,
    --  -1 * gz2/5
    --}

    --core_camera.setRotation(vehID, addRot)
    --core_camera.setOffset(vehID, addPos)























    -- Coefficients for limiting g-force camera position.
    -- TODO: Get constraint values from user settings app
    -- local PosYCoeff = 0.95
    -- local PosZCoeff = 0.95


    -- ADDITIONAL CAMERA POSITION CALCULATION:
    -- Additional camera position based on g forces
    addPos = {
      0,
      PosYCoeff * constrain(-1.3, (-1 * gy2) ,1.3),
      PosZCoeff * constrain(-1.3, (-1 * (gz2 - 1)) ,1.3)
    }

  end

end

-- Export module functions
M.constrain = constrain
M.onUpdate = onUpdate
M.GetVehData = GetVehData

return M