-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local vecY = vec3(0,1,0)
local vecZ = vec3(0,0,1)

local function debuggy(point, text, color)
  debugDrawer:drawSphere(point, 0.05, color, false)
  debugDrawer:drawText(point, text, color, false)
  print(text, dump(point))
end

local collision = require('core/cameraModes/collision')

local C = {}
C.__index = C

function C:init()
  self.disabledByDefault = true
  self.lastCameraRotation = vec3()
  self.forwardVelocitySmoother = newTemporalSmoothing(100)
  local chaseDirectionSmoothFactor = 0.0008
  self.directionSmootherX = newTemporalSmoothing(chaseDirectionSmoothFactor)
  self.directionSmootherY = newTemporalSmoothing(chaseDirectionSmoothFactor)
  self.directionSmootherZ = newTemporalSmoothing(chaseDirectionSmoothFactor)
  self.lastVehiclePosition = vec3()
  self.isLookingForward = true

  self.lastCameraUpVector = vec3()
  self.cameraResetCounter = 0

  self.collision = collision()
  self.collision:init()

  self:onVehicleCameraConfigChanged()
  self:onSettingsChanged()
  self:reset()
end

function C:onVehicleCameraConfigChanged()
  if self.defaultRotation == nil then
    self.defaultRotation = vec3(0, -17, 0)
  else
    self.defaultRotation = vec3(self.defaultRotation)
    self.defaultRotation.y = -self.defaultRotation.y
  end
  self.cameraRotation = vec3(self.defaultRotation)
  self.minDistance = self.distanceMin or 3
  self.defaultDistance = self.distance or 5
  self.currentDistance = self.defaultDistance
  self.lastDistance = self.defaultDistance
  self.targetingMode = self.targetingMode or 'ref'
  self.fieldOfView = self.fieldOfView or 65
  self.offset = vec3(self.offset)
  self.cameraBaseOffset = vec3()
end

function C:onSettingsChanged()
  self.relaxation = settings.getValue('cameraOrbitRelaxation') or 3
  self.rollSmoothing = math.max(settings.getValue('cameraChaseRollSmoothing') or 1, 0.000001)
  self:reset() --TODO is this really necessary?
end

function C:reset()
  self.cameraRotation = vec3(self.defaultRotation)
  self.cameraRotation.x = 0
  self.isLookingForward = true
  self.cameraResetCounter = 2
  self.relativeYaw = 0
  self.relativePitch = 0
end

local rot = vec3()
function C:update(data)
  data.res.collisionCompatible = true
  -- update input
  local inputDeadzone = 0.5
  self.relativeYaw =   clamp(self.relativeYaw   + 0.15*MoveManager.yawRelative  , -1, 1)
  self.relativePitch = clamp(self.relativePitch + 0.15*MoveManager.pitchRelative, -1, 1)
  local usedRelativeYaw   = self.relativeYaw
  local usedRelativePitch = self.relativePitch
  if math.abs(usedRelativeYaw)   < inputDeadzone then usedRelativeYaw   = 0 end
  if math.abs(usedRelativePitch) < inputDeadzone then usedRelativePitch = 0 end

  local yawChange = 200*usedRelativeYaw + 100*data.dt*(MoveManager.yawRight - MoveManager.yawLeft)
  self.cameraRotation.x = 0
  if not self.isLookingForward then
    self.cameraRotation.x = -180
  end

  local inputThreshold = 0.05

  if yawChange > inputThreshold then
    self.cameraRotation.x = 90
  elseif yawChange < -inputThreshold then
    self.cameraRotation.x = -90
  end
  if not self.isLookingForward then
    self.cameraRotation.x = -self.cameraRotation.x
  end

  local pitchChange = 200*usedRelativePitch + 100*data.dt*(MoveManager.pitchUp - MoveManager.pitchDown)
  self.cameraRotation.y = self.defaultRotation.y
  if pitchChange > inputThreshold then
    self.cameraRotation.y = self.defaultRotation.y + 30
  elseif pitchChange < -inputThreshold then
    if self.isLookingForward then
      self.cameraRotation.x = -180
    else
      self.cameraRotation.x = 0
    end
  end

  self.cameraRotation.y = clamp(self.cameraRotation.y, -85, 85)

  -- make sure the rotation is never bigger than 2 PI
  if self.cameraRotation.x > 180 then
    self.cameraRotation.x = self.cameraRotation.x - 360
    self.lastCameraRotation.x = self.lastCameraRotation.x - math.pi * 2
  elseif self.cameraRotation.x < -180 then
    self.cameraRotation.x = self.cameraRotation.x + 360
    self.lastCameraRotation.x = self.lastCameraRotation.x + math.pi * 2
  end

  local distanceChange = 0.1 * data.dt * (MoveManager.zoomIn - MoveManager.zoomOut) * self.fieldOfView
  self.currentDistance = self.defaultDistance
  if distanceChange > inputThreshold then
    self.currentDistance = self.defaultDistance * 2
  elseif distanceChange < -inputThreshold then
    self.currentDistance = self.minDistance
  end

  --
  local refNodePosition  = data.veh:getNodePosition(self.refNodes.ref)
  local leftNodePosition = data.veh:getNodePosition(self.refNodes.left)
  local backNodePosition = data.veh:getNodePosition(self.refNodes.back)

  -- calculate the camera offset: rotate with the vehicle
  local vehicleAxisX = leftNodePosition - refNodePosition
  local vehicleAxisY = backNodePosition - refNodePosition

  if vehicleAxisX:squaredLength() == 0 or vehicleAxisY:squaredLength() == 0 then
    data.res.pos = data.pos
    data.res.rot = quatFromDir(vecY, vecZ)
    return false
  end

  local vehicleAxisZ = vehicleAxisX:cross(vehicleAxisY):normalized()

  if self.offset and self.offset.x then
    self.cameraBaseOffset:set(
      self.offset.x / (vehicleAxisX:length() + 1e-30),
      self.offset.y / (vehicleAxisY:length() + 1e-30),
      self.offset.z / (vehicleAxisZ:length() + 1e-30)
    )
  else
    self.cameraBaseOffset:set(0,0,0)
  end


  local cameraTargetPosition
  if self.targetingMode == 'center' then
    cameraTargetPosition = data.veh:getBBCenter()
  else
    local cameraOffset =
      vehicleAxisX * self.cameraBaseOffset.x +
      vehicleAxisY * self.cameraBaseOffset.y +
      vehicleAxisZ * self.cameraBaseOffset.z
      
      cameraTargetPosition = data.pos + refNodePosition + cameraOffset
  end

  local vehicleDirection = (refNodePosition - backNodePosition); vehicleDirection:normalize()

  if self.cameraResetCounter ~= 0 then
    self.lastVehiclePosition = vec3(data.pos)
  end

  local upVector = vehicleDirection:cross(leftNodePosition); upVector:normalize()

  if self.cameraResetCounter ~= 1 then
    if self.rollSmoothing > 0.0001 then
      local upVectorSmoothingRatio = 1 / (data.dt * self.rollSmoothing)
      upVector = (1 / (upVectorSmoothingRatio + 1) * upVector + (upVectorSmoothingRatio / (upVectorSmoothingRatio + 1)) * self.lastCameraUpVector); upVector:normalize()
    else
      -- if rolling is disabled, we are always up no matter what ...
      upVector:set(vecZ)
    end
    vehicleDirection:set(
      self.directionSmootherX:getUncapped(vehicleDirection.x, data.dt*1000),
      self.directionSmootherY:getUncapped(vehicleDirection.y, data.dt*1000),
      self.directionSmootherZ:getUncapped(vehicleDirection.z, data.dt*1000)
    );
    vehicleDirection:normalize()
  end
  self.lastCameraUpVector:set(upVector)

  -- decide on a looking direction
  -- the reason for this: on reload, the vehicle jumps and the velocity is not correct anymore
  local velocity = (data.pos - self.lastVehiclePosition) / data.dt
  local forwardVelocityComponent = velocity:dot(vehicleDirection)
  local nonForwardVelocityComponent = velocity:distance(forwardVelocityComponent * vehicleDirection)
  local forwardVelo = self.forwardVelocitySmoother:getUncapped(forwardVelocityComponent, data.dt)
  if self.cameraResetCounter == 0 then
    if self.isLookingForward and forwardVelo < -1.5 and math.abs(forwardVelo) > nonForwardVelocityComponent then
      if self.cameraRotation.x >= 0 then
        self.cameraRotation:set(self.defaultRotation)
        self.cameraRotation.x = 180
      else
        self.cameraRotation:set(self.defaultRotation)
        self.cameraRotation.x = -180
      end
      self.isLookingForward = false
    elseif not self.isLookingForward and forwardVelo > 1.5 then
      self.cameraRotation:set(self.defaultRotation)
      self.cameraRotation.x = 0
      self.isLookingForward = true
    end
  end
  self.lastVehiclePosition:set(data.pos)

  rot:set(
    math.rad(self.cameraRotation.x),
    math.rad(self.cameraRotation.y),
    math.rad(self.cameraRotation.z)
  )

  -- smoothing
  local ratio = 5 / (data.dt * 8)
  rot.x = 1 / (ratio + 1) * rot.x + (ratio / (ratio + 1)) * self.lastCameraRotation.x
  rot.y = 1 / (ratio + 1) * rot.y + (ratio / (ratio + 1)) * self.lastCameraRotation.y

  local dist = 1 / (ratio + 1) * self.currentDistance + (ratio / (ratio + 1)) * self.lastDistance

  local calculatedCamPos = (dist-0.8) * vec3(
      math.sin(rot.x) * math.cos(rot.y)
    , math.cos(rot.x) * math.cos(rot.y)
    , math.sin(rot.y)
  )

  local qdir_heading = quatFromDir(-vehicleDirection, upVector)
  calculatedCamPos = qdir_heading * calculatedCamPos

  local camPos = calculatedCamPos + cameraTargetPosition

  local dir_target = (cameraTargetPosition - camPos); dir_target:normalize()
  local qdir_target = quatFromDir(dir_target, upVector)

  self.lastCameraRotation:set(rot)
  self.lastDistance = dist
  self.cameraResetCounter = math.max(self.cameraResetCounter - 1, 0)

  -- application
  data.res.pos = camPos
  data.res.rot = qdir_target
  data.res.fov = self.fieldOfView -- +70
  data.res.cameraTargetPosition = cameraTargetPosition

  debuggy(vec3(1,0,0), "+x", ColorF(1,0,0,1))
  debuggy(vec3(0,1,0), "+y", ColorF(0,1,0,1))
  debuggy(vec3(0,0,1), "+z", ColorF(0,0,1,1))

  debuggy(vec3(vehicleAxisX), "vehicleAxisX", ColorF(0.75,0,0,1))
  debuggy(vec3(vehicleAxisY), "vehicleAxisY", ColorF(0,0.75,0,1))
  debuggy(vec3(vehicleAxisZ), "vehicleAxisZ", ColorF(0,0,0.75,1))

  debuggy(vec3(0.77, 0.65, 1.2), "cameraChase", ColorF(1, 0.7, 0, 1));

  self.collision:update(data)
  return true
end

function C:setRefNodes(centerNodeID, leftNodeID, backNodeID)
  self.refNodes = self.refNodes or {}
  self.refNodes.ref = centerNodeID
  self.refNodes.left = leftNodeID
  self.refNodes.back = backNodeID
end

function C:mouseLocked(locked)
  if locked then return end
  self.relativeYaw = 0
  self.relativePitch = 0
end

-- DO NOT CHANGE CLASS IMPLEMENTATION BELOW

return function(...)
  local o = ... or {}
  setmetatable(o, C)
  o:init()
  return o
end
