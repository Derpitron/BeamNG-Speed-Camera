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

  --prev pos, vel provided by camData
  self.prev_veh_accel = vec3(0,0,0)
  self.prev_veh_jerk = vec3(0,0,0)

  self.last_cam_upVector = vec3(0,0,1)
  self.camResetted = 0

  self.sumDt = 0

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
  self.rot = vec3(self.defaultRotation)
  self.camMinDist = self.distanceMin or 3
  self.distance = self.distance or 5
  self.defaultDistance = self.distance
  self.dist = self.defaultDistance
  self.lastCamDist = self.defaultDistance
  self.mode = self.mode or 'ref'
  self.fov = self.fov or 65
  self.offset = vec3(self.offset)
  self.camBase = vec3()
end


function C:onSettingsChanged()
  -- self.relaxation = settings.getValue('cameraOrbitRelaxation') or 3
  -- self.rollSmoothing = math.max(settings.getValue('cameraChaseRollSmoothing') or 1, 0.000001)
  self:reset() --TODO is this really necessary?
end

function C:reset()
  --prev pos, vel provided by camData
  self.prev_veh_accel = vec3(0,0,0)
  self.prev_veh_jerk = vec3(0,0,0)

  self.rot = vec3(self.defaultRotation)
  self.rot.x = 0
  self.camResetted = 2
  self.relYaw = 0
  self.relPitch = 0
end

function C:update(data)
  data.res.collisionCompatible = true
  self.sumDt = self.sumDt + data.dtSim

  -- 
  -- VEHICLE DATA
  --

  local refNode  = data.veh:getNodePosition(self.refNodes.ref)
  local leftNode = data.veh:getNodePosition(self.refNodes.left)
  local backNode = data.veh:getNodePosition(self.refNodes.back)

  local veh_leftVector = (leftNode - refNode):normalized()
  local veh_backVector = (backNode - refNode):normalized()
  local veh_upVector = veh_leftVector:cross(veh_backVector):normalized()

  local veh_forwardVector = -veh_backVector:normalized()

  -- Do dot product of each one here into whatever unit vector above u want, so as to get the e.g forward, backward velocity.
  local veh_posit = vec3(data.pos) -- or getBBCenter, refNode?
  local veh_veloc = vec3(data.vel)
  local veh_accel = vec3((data.vel - data.prevVel) / data.dtSim)
  local veh_jerk = vec3((veh_accel - self.prev_veh_accel) / data.dtSim)


  --
  -- CAMERA DATA
  -- All this data is respective to the camera itself.
  --

  local cam_leftVector = vec3(1,0,0)
  local cam_backVector = vec3(0,1,0)
  local cam_upVector = vec3(0,0,1)

  local cam_offset = vec3(self.offset)

  self.camBase:set(
    cam_offset.x * veh_leftVector +
    cam_offset.y * veh_backVector +
    cam_offset.z * veh_upVector
  )


  -- Applies local panning to camPos. doesn't yet consider the veicle's rotation. local to the vehicle's coordinate frame only.
  local camPos_R = self.dist * vec3(
      - (math.sin(math.rad(self.rot.x)) * math.cos(math.rad(self.rot.y))) -- yaw (eventually about vehicle
    , math.cos(math.rad(self.rot.x)) * math.cos(math.rad(self.rot.y)) -- pitch (eventually about vehicle
    , math.sin(math.rad(self.rot.y))                                  -- roll (eventually about vehicle
  )

  local rot_crash= vec3(0,0,0)

  local qdir_cam2vehicle = quatFromDir(veh_backVector, cam_upVector)

  local targetPos = self.camBase

  -- Put everything into global coordinate system
  local targetPos_g = vec3(data.pos) + vec3(targetPos)  -- or getBBCenter, ref?
  -- Applies vehicle's rotation to the camera.
  local camPos_v = qdir_cam2vehicle * camPos_R
  local camPos_g = targetPos_g + camPos_v

  local cam_forwardVector = -(qdir_cam2vehicle * camPos_R):normalized()
  local qdir_cam2target = quatFromDir(veh_forwardVector, cam_upVector)


  -- application. Do NOT change this except for variable names
  data.res.pos = camPos_g
  data.res.rot = qdir_cam2target
  data.res.fov = self.fov
  data.res.targetPos = targetPos_g

  -- Loop here
  self.collision:update(data)

  self.prev_veh_accel:set(veh_accel)
  self.prev_veh_jerk:set(veh_jerk)
  dump(veh_accel)
  --dump(vec3(
  --  veh_jerk:dot(veh_leftVector),
  --  veh_jerk:dot(veh_forwardVector),
  --  veh_jerk:dot(veh_backVector)
  --))
  self.lastCamRot:set(self.rot)
  self.lastCamDist = self.dist
  self.camResetted = math.max(self.camResetted - 1, 0)

  local function locals()
  local variables = {}
  local idx = 1
  while true do
    local ln, lv = debug.getlocal(2, idx)
    if ln ~= nil then
      variables[ln] = lv
    else
      break
    end
    idx = 1 + idx
  end
  return variables
end

  --dump(locals())

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
