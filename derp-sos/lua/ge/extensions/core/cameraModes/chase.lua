-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local collision = require('core/cameraModes/collision')

local function debuggy(point, text, color)
  debugDrawer:drawSphere(point, 0.05, color, false)
  debugDrawer:drawText(point, text, color, false)
  print(text, dump(point))
end

-- Inverse of LuaQuat:setFromEuler(x,y,z)
local function getEulerForSetFromEuler(q)
  -- q.x, q.y, q.z, q.w expected from your module's quaternion type
  local qx, qy, qz, qw = q.x, q.y, q.z, q.w

  -- rotation-matrix terms (standard quaternion -> rotation-matrix)
  local R11 = 1 - 2 * (qy*qy + qz*qz)
  local R21 = 2 * (qx*qy + qw*qz)
  local R31 = 2 * (qx*qz - qw*qy)

  local R22 = 1 - 2 * (qx*qx + qz*qz)
  local R23 = 2 * (qy*qz - qw*qx)

  -- Y-Z-X extraction: alpha = Y, beta = Z, gamma = X
  local alpha = math.atan2(-R31, R11)                         -- Y
  local beta  = math.asin( math.max(-1, math.min(1, R21)) )   -- Z (clamped)
  local gamma = math.atan2(-R23, R22)                         -- X

  -- return table as vec3(x=gamma, y=alpha, z=beta) ready for setFromEuler
  return vec3(gamma, alpha, beta)
end


local C = {}
C.__index = C

function C:init()
  self.disabledByDefault = true
  self.lastCamRot = vec3()
  self.fwdVeloSmoother = newTemporalSmoothing(100)
  local chaseDirSmoothCoef = 0.0008
  self.dirSmoothX = newTemporalSmoothing(chaseDirSmoothCoef)
  self.dirSmoothY = newTemporalSmoothing(chaseDirSmoothCoef)
  self.dirSmoothZ = newTemporalSmoothing(chaseDirSmoothCoef)
  self.lastDataPos = vec3()
  self.forwardLooking = true
  self.lastRefPos = vec3()
  self.last_cam_upVector = vec3()
  self.camResetted = 0

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
  self.camRot_V_deg = vec3(self.defaultRotation)
  self.camMinDist = self.distanceMin or 3
  self.distance = self.distance or 5
  self.defaultDistance = self.distance
  self.camDist = self.defaultDistance
  self.lastCamDist = self.defaultDistance
  self.mode = self.mode or 'ref'
  self.fov_V = self.fov_V or 65
  self.offset = vec3(self.offset)
  self.camBase = vec3()
end

function C:onSettingsChanged()
  self.relaxation = settings.getValue('cameraOrbitRelaxation') or 3
  self.rollSmoothing = math.max(settings.getValue('cameraChaseRollSmoothing') or 1, 0.000001)
  self:reset() --TODO is this really necessary?
end

function C:reset()
  self.camRot_V_deg = vec3(self.defaultRotation)
  self.camRot_V_deg.x = 0
  self.forwardLooking = true
  self.camResetted = 2
  self.relYaw = 0
  self.relPitch = 0
end

local rot_V_rad = vec3()
function C:update(data)
  data.res.collisionCompatible = true
  -- mouse/controller pan input
  local deadzone = 0.5
  self.relYaw =   clamp(self.relYaw   + 0.15*MoveManager.yawRelative  , -1, 1)
  self.relPitch = clamp(self.relPitch + 0.15*MoveManager.pitchRelative, -1, 1)
  local relYawUsed   = self.relYaw
  local relPitchUsed = self.relPitch
  if math.abs(relYawUsed)   < deadzone then relYawUsed   = 0 end
  if math.abs(relPitchUsed) < deadzone then relPitchUsed = 0 end

  local dx = 200*relYawUsed + 100*data.dt*(MoveManager.yawRight - MoveManager.yawLeft)
  self.camRot_V_deg.x = 0
  if not self.forwardLooking then
    self.camRot_V_deg.x = -180
  end

  local triggerValue = 0.05

  if dx > triggerValue then
    self.camRot_V_deg.x = 90
  elseif dx < -triggerValue then
    self.camRot_V_deg.x = -90
  end
  if not self.forwardLooking then
    self.camRot_V_deg.x = -self.camRot_V_deg.x
  end

  local dy = 200*relPitchUsed + 100*data.dt*(MoveManager.pitchUp - MoveManager.pitchDown)
  self.camRot_V_deg.y = self.defaultRotation.y
  if dy > triggerValue then
    self.camRot_V_deg.y = self.defaultRotation.y + 30
  elseif dy < -triggerValue then
    if self.forwardLooking then
      self.camRot_V_deg.x = -180
    else
      self.camRot_V_deg.x = 0
    end
  end

  self.camRot_V_deg.y = clamp(self.camRot_V_deg.y, -85, 85)

  -- make sure the rotation is never bigger than 2 PI
  if self.camRot_V_deg.x > 180 then
    self.camRot_V_deg.x = self.camRot_V_deg.x - 360
    self.lastCamRot.x = self.lastCamRot.x - math.pi * 2
  elseif self.camRot_V_deg.x < -180 then
    self.camRot_V_deg.x = self.camRot_V_deg.x + 360
    self.lastCamRot.x = self.lastCamRot.x + math.pi * 2
  end

  local ddist = 0.1 * data.dt * (MoveManager.zoomIn - MoveManager.zoomOut) * self.fov_V
  self.camDist = self.defaultDistance
  if ddist > triggerValue then
    self.camDist = self.defaultDistance * 2
  elseif ddist < -triggerValue then
    self.camDist = self.camMinDist
  end

  local refNode  = data.veh:getNodePosition(self.refNodes.ref)
  local leftNode = data.veh:getNodePosition(self.refNodes.left)
  local backNode = data.veh:getNodePosition(self.refNodes.back)

  -- calculate the camera offset: rotate with the vehicle
  -- Palette: unit direction vectors.
  local veh_leftVector = (leftNode - refNode):normalized()
  local veh_backVector = (backNode - refNode):normalized()
  local veh_upVector = veh_leftVector:cross(veh_backVector):normalized()

  local veh_forwardVector = -veh_backVector:normalized()


  local cam_upVector = vec3(veh_upVector)

  if veh_leftVector:squaredLength() == 0 or veh_backVector:squaredLength() == 0 then
    data.res.pos = data.pos
    data.res.rot = quatFromDir(vec3(0,1,0), vec3(0,0,1))
    return false
  end

  if self.offset and self.offset.x then
    -- Palette: 
    -- self.camBase is the camera's default centre. this gives you a working centre of the vehicle.
    -- (we assume) it is guaranteed to be correct, vanilla, and centred because vehicle jbeam -> `self.offset`, vehicle global rotation -> it's dir vectors 
    self.camBase:set(
      self.offset.x * veh_leftVector +
      self.offset.y * veh_backVector +
      self.offset.z * veh_upVector
    )
  else
    self.camBase:set(0,0,0)
  end

  local targetPos_g
  if self.mode == 'center' then
    targetPos_g = data.veh:getBBCenter()
  else
    targetPos_g = data.pos + self.camBase
  end

  if self.camResetted ~= 1 then
    if self.rollSmoothing > 0.0001 then
      local upSmoothratio = 1 / (data.dt * self.rollSmoothing)
      cam_upVector = (1 / (upSmoothratio + 1) * cam_upVector + (upSmoothratio / (upSmoothratio + 1)) * self.last_cam_upVector):normalized()
    else
      -- if rolling is disabled, we are always up no matter what ...
      cam_upVector:set(vec3(0,0,1))
    end
    veh_forwardVector = vec3(
      self.dirSmoothX:getUncapped(veh_forwardVector.x, data.dt*1000),
      self.dirSmoothY:getUncapped(veh_forwardVector.y, data.dt*1000),
      self.dirSmoothZ:getUncapped(veh_forwardVector.z, data.dt*1000)
    ):normalized()
  end
  self.last_cam_upVector:set(cam_upVector)

  -- decide on a looking direction
  -- the reason for this: on reload, the vehicle jumps and the velocity is not correct anymore
  local vel = (data.pos - self.lastDataPos) / data.dt
  local velF = vel:dot(veh_forwardVector)
  local velNF = vel:distance(velF * veh_forwardVector)
  local forwardVelo = self.fwdVeloSmoother:getUncapped(velF, data.dt)
  if self.camResetted == 0 then
    if self.forwardLooking and forwardVelo < -1.5 and math.abs(forwardVelo) > velNF then
      if self.camRot_V_deg.x >= 0 then
        self.camRot_V_deg:set(self.defaultRotation)
        self.camRot_V_deg.x = 180
      else
        self.camRot_V_deg:set(self.defaultRotation)
        self.camRot_V_deg.x = -180
      end
      self.forwardLooking = false
    elseif not self.forwardLooking and forwardVelo > 1.5 then
      self.camRot_V_deg:set(self.defaultRotation)
      self.camRot_V_deg.x = 0
      self.forwardLooking = true
    end
  end
  self.lastDataPos:set(data.pos)

  rot_V_rad:set(
    math.rad(self.camRot_V_deg.x),
    math.rad(self.camRot_V_deg.y),
    math.rad(self.camRot_V_deg.z)
  )

  -- smoothing
  local ratio_V = 1 / (data.dt * 8)
  rot_V_rad.x = 1 / (ratio_V + 1) * rot_V_rad.x + (ratio_V / (ratio_V + 1)) * self.lastCamRot.x
  rot_V_rad.y = 1 / (ratio_V + 1) * rot_V_rad.y + (ratio_V / (ratio_V + 1)) * self.lastCamRot.y

  local dist_V = 1 / (ratio_V + 1) * self.camDist + (ratio_V / (ratio_V + 1)) * self.lastCamDist


  local camPos_V = dist_V * vec3(
      math.sin(rot_V_rad.x) * math.cos(rot_V_rad.y)
    , math.cos(rot_V_rad.x) * math.cos(rot_V_rad.y)
    , math.sin(rot_V_rad.y)
  )

  local qdir_veh = quatFromDir(veh_backVector, cam_upVector)
  local camPos_g = targetPos_g + (qdir_veh * camPos_V)

  local dir_cam2target = (targetPos_g - camPos_g):normalized()
  local qdir_cam2target = quatFromDir(dir_cam2target, cam_upVector)

  local edir_cam2target = getEulerForSetFromEuler(qdir_cam2target)
  qdir_cam2target:setFromEuler(
    edir_cam2target.x,
    edir_cam2target.y,
    edir_cam2target.z
  )

  self.lastCamRot:set(rot_V_rad)
  self.lastCamDist = dist_V
  self.camResetted = math.max(self.camResetted - 1, 0)

  -- application
  data.res.pos = camPos_g
  data.res.rot = qdir_cam2target
  data.res.fov = self.fov_V
  data.res.targetPos = targetPos_g

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
  self.relYaw = 0
  self.relPitch = 0
end

-- DO NOT CHANGE CLASS IMPLEMENTATION BELOW

return function(...)
  local o = ... or {}
  setmetatable(o, C)
  o:init()
  return o
end
