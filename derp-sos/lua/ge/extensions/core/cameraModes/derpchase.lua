-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local vecY = vec3(0, 1, 0)
local vecZ = vec3(0, 0, 1)

local collision = require('core/cameraModes/collision')

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
  self.camRot = vec3(self.defaultRotation)
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
  self.relaxation = settings.getValue('cameraOrbitRelaxation') or 3
  self.rollSmoothing = math.max(settings.getValue('cameraChaseRollSmoothing') or 1, 0.000001)
  self:reset()   --TODO is this really necessary?
end

function C:reset()
  self.camRot = vec3(self.defaultRotation)
  self.camRot.x = 0
  self.forwardLooking = true
  self.camResetted = 2
  self.relYaw = 0
  self.relPitch = 0
end

--#region myfunctions

---All the following functions must do is provide a INTERFACE. No logic, no constants, no presets.
---CONSTANT MODIFIERS, PHASE 0
---PHASE 0 is a lightweight wrapper over vanilla camera that the user may use to provide discrete and constant tweaks to orientation, offset, and FOV.
-- users they want to set a constant absolute offset (eg x -> fov = x), nothing potentially recursive
function Constant_user_defined_position_offset(
    enabled_boolean,
    additive_offset_vec3,
    direction_vectors_vec3_vec3
)
  --#region Constant_user_defined_offset function body
  local offset_contribution_vec3 = vec3(0, 0, 0)
  if enabled_boolean == true then
    --Unrolling the tables for performance
    local x_leftward_unit_vector_vec3 = direction_vectors_vec3_vec3.x
    local y_backward_unit_vector_vec3 = direction_vectors_vec3_vec3.y
    local z_upward_unit_vector_vec3   = direction_vectors_vec3_vec3.z

    --Apply our offset calculations to our piece of the offset pie.
    --We are responsible only for cooking this relative value'd vec3 and when we return it we do not care what else happens
    offset_contribution_vec3          =
        vec3(
          x_leftward_unit_vector_vec3 * additive_offset_vec3.x +
          y_backward_unit_vector_vec3 * additive_offset_vec3.y +
          z_upward_unit_vector_vec3 * additive_offset_vec3.z
        )
  end
  return offset_contribution_vec3
  --#endregion Constant_user_defined_offset function body
end

function Constant_user_defined_fov_offset(
    enabled_boolean,
    additive_fov_offset_scalar_degree
)
  --#region Constant_user_defined_fov_offset function body
  local offset_contribution_vec3 = 0
  if enabled_boolean == true then
    --Apply our offset calculations to our piece of the offset pie.
    --We are responsible only for cooking this relative value'd scalar and when we return it we do not care what else happens
    offset_contribution_vec3 = additive_fov_offset_scalar_degree
  end
  return offset_contribution_vec3
  --#endregion Constant_user_defined_fov_offset function body
end

---CONDITIONAL MODIFIERS, PHASE 1
---These modifiers are for "constant" effects like offset/orientation depending on some rate variable/time series quantity e.g velocity, acceleration
---These must settle back to the mean position of VANILLA when the conditions.
-- POSITION MODIFIER
function Rate_variable_derived_position_offset(
    enabled_boolean,
    rate_variable_vec3,
    direction_vectors_vec3_vec3,
    multiplicative_constant_value_coefficients_vec3,
    variable_clamp_ranges_vec3_vec2, --TODO: allow partial filling in of clamp ranges. e.g clamp only x and y, and z should not be passed in, then handle that
    variable_smoother_function,      --TODO: define a schema for variable_smoother_function. e.g func(var, dt)
    dt
)
  --#region rate_variable_derived_offset function body
  local offset_contribution_vec3 = vec3(0, 0, 0)
  if enabled_boolean == true then
    -- Clamping the components of time-series variable with the clamp ranges specified per component
    if variable_clamp_ranges_vec3_vec2 then
      --Unrolling the tables for performance
      local variable_clamp_range_x = variable_clamp_ranges_vec3_vec2.x
      local variable_clamp_range_y = variable_clamp_ranges_vec3_vec2.y
      local variable_clamp_range_z = variable_clamp_ranges_vec3_vec2.z

      --CLAMP!
      rate_variable_vec3 =
          vec3(
            clamp(rate_variable_vec3.x, variable_clamp_range_x[1], variable_clamp_range_x[2]),
            clamp(rate_variable_vec3.x, variable_clamp_range_y[1], variable_clamp_range_y[2]),
            clamp(rate_variable_vec3.x, variable_clamp_range_z[1], variable_clamp_range_z[2])
          )
    end

    --Unrolling the tables for performance
    local x_leftward_unit_vector_vec3 = direction_vectors_vec3_vec3.x
    local y_backward_unit_vector_vec3 = direction_vectors_vec3_vec3.y
    local z_upward_unit_vector_vec3   = direction_vectors_vec3_vec3.z

    if variable_smoother_function == nil then
      local function smoothing_function(var, dt)
        return var
      end
    end

    --Apply our offset calculations to our piece of the offset pie.
    --We are responsible only for cooking this relative value'd vec3 and when we return it we do not care what else happens
    offset_contribution_vec3 =
        vec3(
          x_leftward_unit_vector_vec3 * multiplicative_constant_value_coefficients_vec3.x *
          variable_smoother_function(rate_variable_vec3.x, dt) +
          y_backward_unit_vector_vec3 * multiplicative_constant_value_coefficients_vec3.y *
          variable_smoother_function(rate_variable_vec3.y, dt) +
          z_upward_unit_vector_vec3 * multiplicative_constant_value_coefficients_vec3.z *
          variable_smoother_function(rate_variable_vec3.z, dt)
        )
  end
  return offset_contribution_vec3
  --#endregion rate_variable_derived_offset function body
end

--FOV MODIFIER
function Rate_variable_derived_fov_offset(
    enabled_boolean,
    rate_variable_vec3,
    coefficient_scalar,
    clamp_range_vec2,
    variable_smoothing_function
)
  --#region rate_variable_derived_fov_offset function body
  local fov_offset_contribution_scalar = 0
  if enabled_boolean == true then
    local rate_variable_magnitude = rate_variable_vec3:length()
    -- Clamping the magnitude of rate_variable
    if clamp_range_vec2 then
      --CLAMP!
      rate_variable_magnitude = clamp(rate_variable_magnitude, clamp_range_vec2[1], clamp_range_vec2[2])
    end

    --Apply our offset calculations to our piece of the offset pie.
    -- We are responsible only for cooking this relative value'd vec3 and when we return it we do not care what else happens
    fov_offset_contribution_scalar = coefficient_scalar *
        variable_smoothing_function(rate_variable_magnitude, data.dt)
  end
  return fov_offset_contribution_scalar
  --#endregion rate_variable_derived_fov_offset function body
end

--#endregion myfunctions

local rot = vec3()
function C:update(data)
  --#region theircode

  data.res.collisionCompatible = true
  -- update input
  local deadzone               = 0.5
  self.relYaw                  = clamp(self.relYaw + 0.15 * MoveManager.yawRelative, -1, 1)
  self.relPitch                = clamp(self.relPitch + 0.15 * MoveManager.pitchRelative, -1, 1)
  local relYawUsed             = self.relYaw
  local relPitchUsed           = self.relPitch
  if math.abs(relYawUsed) < deadzone then relYawUsed = 0 end
  if math.abs(relPitchUsed) < deadzone then relPitchUsed = 0 end

  local dx = 200 * relYawUsed + 100 * data.dt * (MoveManager.yawRight - MoveManager.yawLeft)
  self.camRot.x = 0
  if not self.forwardLooking then
    self.camRot.x = -180
  end

  local triggerValue = 0.05

  if dx > triggerValue then
    self.camRot.x = 90
  elseif dx < -triggerValue then
    self.camRot.x = -90
  end
  if not self.forwardLooking then
    self.camRot.x = -self.camRot.x
  end

  local dy = 200 * relPitchUsed + 100 * data.dt * (MoveManager.pitchUp - MoveManager.pitchDown)
  self.camRot.y = self.defaultRotation.y
  if dy > triggerValue then
    self.camRot.y = self.defaultRotation.y + 30
  elseif dy < -triggerValue then
    if self.forwardLooking then
      self.camRot.x = -180
    else
      self.camRot.x = 0
    end
  end

  self.camRot.y = clamp(self.camRot.y, -85, 85)

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
  local nx   = left - ref
  local ny   = back - ref

  if nx:squaredLength() == 0 or ny:squaredLength() == 0 then
    data.res.pos = data.pos
    data.res.rot = quatFromDir(vecY, vecZ)
    return false
  end

  local nz = nx:cross(ny):normalized()

  if self.offset and self.offset.x then
    self.camBase:set(self.offset.x / (nx:length() + 1e-30), self.offset.y / (ny:length() + 1e-30),
      self.offset.z / (nz:length() + 1e-30))
  else
    self.camBase:set(0, 0, 0)
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
    dir:set(self.dirSmoothX:getUncapped(dir.x, data.dt * 1000), self.dirSmoothY:getUncapped(dir.y, data.dt * 1000),
      self.dirSmoothZ:getUncapped(dir.z, data.dt * 1000)); dir:normalize()
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

  local qdir_heading = quatFromDir(-dir, up)
  calculatedCamPos = qdir_heading * calculatedCamPos

  local camPos = calculatedCamPos + targetPos

  local dir_target = (targetPos - camPos); dir_target:normalize()
  local qdir_target = quatFromDir(dir_target, up)

  self.camLastRot:set(rot)
  self.camLastDist = dist
  self.camResetted = math.max(self.camResetted - 1, 0)

  --#endregion theircode

  --#region MY CODE
  local left_vec3_metres_leftfacing_car_unitvector_relativeto_carorigin = (left - ref); left_vec3_metres_leftfacing_car_unitvector_relativeto_carorigin
      :normalize()

  local car_dirvec_left          = left_vec3_metres_leftfacing_car_unitvector_relativeto_carorigin
  local car_dirvec_back          = dir
  local car_dirvec_up            = up
  -- TODO Idk if the below 3 lines even work. and is the backwards vector backwards wrt the car or global?
  local cam_dirvec_left          = qdir_target * vec3(1, 0, 0)
  local cam_dirvec_back          = qdir_target * vec3(0, 1, 0)
  local cam_dirvec_up            = qdir_target * vec3(0, 0, 1)

  local car_direction_vectors    = { x = car_dirvec_left, y = car_dirvec_back, z = car_dirvec_up }
  local camera_direction_vectors = { x = cam_dirvec_left, y = cam_dirvec_back, z = cam_dirvec_up }

  --gnd stands for ground, as in ground-state
  local gnd_cam_pos              = camPos
  local gnd_cam_fov              = self.fov
  local gnd_cam_rot              = qdir_target
  local gnd_target_pos           = targetPos

  -- USERS DEFINE YOUR EFFECTS HERE.
  local final_offset_vec3 -- =
  --Rate_variable_derived_position_offset{enabled_boolean=true, rate_variable_vec3=data.vel, direction_vectors_vec3_vec3=camera_direction_vectors} +
  --Rate_variable_derived_position_offset{enabled_boolean=true, rate_variable_vec3=data.acc, direction_vectors_vec3_vec3=   car_direction_vectors}

  local final_fov_offset_scalar -- =
  --Rate_variable_derived_fov_offset{enabled_boolean=vtrue, rate_variable_vec3=data.vel} +
  --Rate_variable_derived_fov_offset{enabled_boolean=true, rate_variable_vec3=data.acc}

  local final_orientation_offset_quat -- =

  -- FINAL FINAL FINAL application
  --data.res.pos                           = gnd_cam_pos + final_offset_vec3
  --data.res.fov                           = gnd_cam_fov + final_fov_offset_scalar
  --data.res.rot                           = gnd_cam_rot * final_orientation_offset_quat

  -- This is extra metadata we pass back to the camera for the next frame.
  -- I.e we trick the camera into believing nothing is amiss and it calculates everything as vanilla.
  --data.res.pos                    = gnd_cam_pos
  --data.res.fov                    = gnd_cam_fov
  --data.res.rot                    = gnd_cam_rot
  --data.res.targetPos                  = gnd_target_pos

  --#endregion

  -- application
  data.res.pos                   = camPos
  data.res.rot                   = qdir_target
  data.res.fov                   = self.fov
  data.res.targetPos             = targetPos

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
