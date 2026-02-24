-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local collision = require('core/cameraModes/collision')

local collision = require('core/cameraModes/collision')
local fxcontrol__derp_sos = require('derp-sos/fxcontrol')

local C = {}
C.__index = C

function C:init()
  self.disabledByDefault = true
  self.camLastRot = vec3()
  self.fwdVeloSmoother = newTemporalSmoothing(100)
  local chaseDirSmoothCoef = 0.0008
  self.dirSmoothX = newTemporalSmoothing(chaseDirSmoothCoef)
  self.dirSmoothY = newTemporalSmoothing(chaseDirSmoothCoef)
  self.dirSmoothZ = newTemporalSmoothing(chaseDirSmoothCoef)
  self.lastDataPos = vec3()
  self.forwardLooking = true
  self.lastRefPos = vec3()
  self.camLastUp = vec3()
  self.camResetted = 0

  self.sumDt = 0

  self.collision = collision()
  self.collision:init()

  --#region derp_sos
  self.fxcontrol__derp_sos = fxcontrol__derp_sos()
  self.fxcontrol__derp_sos:init()
  --#endregion

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
  self.camDist = self.defaultDistance
  self.camLastDist = self.defaultDistance
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

  -- make sure the rotation is never bigger than 2 PI
  if self.camRot.x > 180 then
    self.camRot.x = self.camRot.x - 360
    self.camLastRot.x = self.camLastRot.x - math.pi * 2
  elseif self.camRot.x < -180 then
    self.camRot.x = self.camRot.x + 360
    self.camLastRot.x = self.camLastRot.x + math.pi * 2
  end

  local ddist = 0.1 * data.dt * (MoveManager.zoomIn - MoveManager.zoomOut) * self.fov
  self.camDist = self.defaultDistance
  if ddist > triggerValue then
    self.camDist = self.defaultDistance * 2
  elseif ddist < -triggerValue then
    self.camDist = self.camMinDist
  end

  --
  local ref  = data.veh:getNodePosition(self.refNodes.ref)
  local left = data.veh:getNodePosition(self.refNodes.left)
  local back = data.veh:getNodePosition(self.refNodes.back)

  -- calculate the camera offset: rotate with the vehicle
  local nx = left - ref
  local ny = back - ref

  local cam_offset = vec3(self.offset)

  local nz = nx:cross(ny):normalized()

  if self.offset and self.offset.x then
    self.camBase:set(self.offset.x / (nx:length() + 1e-30), self.offset.y / (ny:length() + 1e-30), self.offset.z / (nz:length() + 1e-30))
  else
    self.camBase:set(0,0,0)
  end


  local targetPos
  if self.mode == 'center' then
    targetPos = data.veh:getBBCenter()
  else
    local camOffset2 = nx * self.camBase.x + ny * self.camBase.y + nz * self.camBase.z
    targetPos = data.pos + ref + camOffset2
  end

  local dir = (ref - back); dir:normalize()

  if self.camResetted ~= 0 then
    self.lastDataPos = vec3(data.pos)
  end

  local up = dir:cross(left); up:normalize()

  if self.camResetted ~= 1 then
    if self.rollSmoothing > 0.0001 then
      local upSmoothratio = 1 / (data.dt * self.rollSmoothing)
      up = (1 / (upSmoothratio + 1) * up + (upSmoothratio / (upSmoothratio + 1)) * self.camLastUp); up:normalize()
    else
      -- if rolling is disabled, we are always up no matter what ...
      up:set(vecZ)
    end
    dir:set(self.dirSmoothX:getUncapped(dir.x, data.dt*1000), self.dirSmoothY:getUncapped(dir.y, data.dt*1000), self.dirSmoothZ:getUncapped(dir.z, data.dt*1000)); dir:normalize()
  end
  self.camLastUp:set(up)

  -- decide on a looking direction
  -- the reason for this: on reload, the vehicle jumps and the velocity is not correct anymore
  local vel = (data.pos - self.lastDataPos) / data.dt
  local velF = vel:dot(dir)
  local velNF = vel:distance(velF * dir)
  local forwardVelo = self.fwdVeloSmoother:getUncapped(velF, data.dt)
  if self.camResetted == 0 then
    if self.forwardLooking and forwardVelo < -1.5 and math.abs(forwardVelo) > velNF then
      if self.camRot.x >= 0 then
        self.camRot:set(self.defaultRotation)
        self.camRot.x = 180
      else
        self.camRot:set(self.defaultRotation)
        self.camRot.x = -180
      end
      self.forwardLooking = false
    elseif not self.forwardLooking and forwardVelo > 1.5 then
      self.camRot:set(self.defaultRotation)
      self.camRot.x = 0
      self.forwardLooking = true
    end
  end
  self.lastDataPos:set(data.pos)

  rot:set(math.rad(self.camRot.x), math.rad(self.camRot.y), math.rad(self.camRot.z))

  -- smoothing
  local ratio = 1 / (data.dt * 8)
  rot.x = 1 / (ratio + 1) * rot.x + (ratio / (ratio + 1)) * self.camLastRot.x
  rot.y = 1 / (ratio + 1) * rot.y + (ratio / (ratio + 1)) * self.camLastRot.y

  local dist = 1 / (ratio + 1) * self.camDist + (ratio / (ratio + 1)) * self.camLastDist

  local calculatedCamPos = dist * vec3(
     math.sin(rot.x) * math.cos(rot.y)
    , math.cos(rot.x) * math.cos(rot.y)
    , math.sin(rot.y)
  )

  local rot_crash= vec3(0,0,0)

  local qdir_cam2vehicle = quatFromDir(veh_backVector, cam_upVector)

  local targetPos = self.camBase

  self.camLastRot:set(rot)
  self.camLastDist = dist
  self.camResetted = math.max(self.camResetted - 1, 0)

    --#region derp_sos
  --- conventions:
  --- <variable>__<what it belongs to>_<what space it's in>__<effect name>
  --- w, v, t, c means world, vehicle, target, camera space respectively.
  ---
  --- <effect name> from thereon means the first effect applied, then the next effect that's applied ON TOP OF THAT/DEPENDENT ON THAT, etc etc.
  ---   e.g ...inputOrbit_...
  -- TODO: vehicle mass, fov

  -- TODO: is this necessary
  local rot__vehicle_w = quatFromDir(
    -back:normalized(),
    left:cross(back):normalized()
  )

  local output         = {
    dt = data.dt,
    dtSim = data.dtSim,

    veh = {
      pos = data.vehPos,
      rot = rot__vehicle_w,
      vel = rot__vehicle_w:inversed() * vel,
      accel = rot__vehicle_w:inversed() * ((vel - data.prevVel) / data.dt),
    },

    target_v = quatFromAxisAngle(Z, math.pi) * self.offset,

    pan = {
      yaw = rot.x,
      pitch = rot.y,
      radius = dist
    },

    fov = self.fov
  }

  -- final filtered, fx'ed camera outputs
  local cam_res        = self.fxcontrol__derp_sos:calculate(output)
  camPos               = cam_res.pos
  qdir_target          = cam_res.rot
  local fov            = cam_res.fov
  targetPos            = cam_res.targetPos
  --#endregion


  -- application
  data.res.pos = camPos
  data.res.rot = qdir_target
  data.res.fov = fov
  data.res.targetPos = targetPos

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
