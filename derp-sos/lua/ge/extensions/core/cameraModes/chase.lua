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
  self.lastCamUp = vec3()
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
  self.camBase_v = vec3()
end

function C:onSettingsChanged()
  -- self.relaxation = settings.getValue('cameraOrbitRelaxation') or 3
  -- self.rollSmoothing = math.max(settings.getValue('cameraChaseRollSmoothing') or 1, 0.000001)
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
  self.camRot_V_deg.y = self.defaultRotation.y

  self.camRot_V_deg.y = clamp(self.camRot_V_deg.y, -85, 85)

  -- make sure the rotation is never bigger than 2 PI
  if self.camRot_V_deg.x > 180 then
    self.camRot_V_deg.x = self.camRot_V_deg.x - 360
    self.lastCamRot.x = self.lastCamRot.x - math.pi * 2
  elseif self.camRot_V_deg.x < -180 then
    self.camRot_V_deg.x = self.camRot_V_deg.x + 360
    self.lastCamRot.x = self.lastCamRot.x + math.pi * 2
  end

  self.camDist = self.defaultDistance

  local ref_v  = data.veh:getNodePosition(self.refNodes.ref)
  local left_v = data.veh:getNodePosition(self.refNodes.left)
  local back_v = data.veh:getNodePosition(self.refNodes.back)

  -- calculate the camera offset: rotate with the vehicle
  -- calculate the camera offset: rotate with the vehicle
  local nx_v = left_v - ref_v
  local ny_v = back_v - ref_v

  if nx_v:squaredLength() == 0 or ny_v:squaredLength() == 0 then
    data.res.pos = data.pos
    data.res.rot = quatFromDir(vec3(0,1,0), vec3(0,0,1))
    return false
  end

  local nz_v = nx_v:cross(ny_v):normalized()

  if self.offset and self.offset.x then
    self.camBase_v:set(
      self.offset.x / (nx_v:length() + 1e-30),
      self.offset.y / (ny_v:length() + 1e-30),
      self.offset.z / (nz_v:length() + 1e-30)
    )
  else
    self.camBase_v:set(0,0,0)
  end

    --SHIM HERE affects targetPos_g and camPos_V!
    local camOffset2_V =
      nx_v * self.camBase_v.x +
      ny_v * self.camBase_v.y +
      nz_v * self.camBase_v.z

  local targetPos_g = data.pos + ref_v + camOffset2_V

  local car_forward_v = -car_backward_v; car_forward_v:normalize()

  local up = dir:cross(left_v); up:normalize()

  if self.camResetted ~= 1 then
    if self.rollSmoothing > 0.0001 then
      local upSmoothratio = 1 / (data.dt * self.rollSmoothing)
      up = (1 / (upSmoothratio + 1) * up + (upSmoothratio / (upSmoothratio + 1)) * self.lastCamUp); up:normalize()
    else
      -- if rolling is disabled, we are always up no matter what ...
      up:set(vecZ)
    end
    dir:set(
      self.dirSmoothX:getUncapped(dir.x, data.dt*1000),
      self.dirSmoothY:getUncapped(dir.y, data.dt*1000),
      self.dirSmoothZ:getUncapped(dir.z, data.dt*1000)
    );
    dir:normalize()
  end
  self.lastCamUp:set(up)

  -- decide on a looking direction
  -- the reason for this: on reload, the vehicle jumps and the velocity is not correct anymore
  local vel = (data.pos - self.lastDataPos) / data.dt
  local velF = vel:dot(dir)
  local velNF = vel:distance(velF * dir)
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
  
  -- this variable exists for smoothing camera rotation.
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

  --SHIM HERE! affects camPos_V
  local camPos_V = dist_V * vec3(
      math.sin(rot_V_rad.x) * math.cos(rot_V_rad.y)
    , math.cos(rot_V_rad.x) * math.cos(rot_V_rad.y)
    , math.sin(rot_V_rad.y)
  )

  local qdir_veh = quatFromDir(-dir, up)
  local camPos_r = qdir_veh * camPos_V

  local camPos_g = camPos_r + targetPos_g

  local dir_cam2target = (targetPos_g - camPos_g); dir_cam2target:normalize()
  local qdir_cam2target = quatFromDir(dir_cam2target, up)

  local edir_cam2target = getEulerForSetFromEuler(qdir_cam2target)
  -- SHIM HERE!
  -- affects camera rotation in look, not space
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
  data.res.fov = self.fov_V -- +70
  data.res.targetPos = targetPos_g

  data.res.collisionCompatible = true
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
