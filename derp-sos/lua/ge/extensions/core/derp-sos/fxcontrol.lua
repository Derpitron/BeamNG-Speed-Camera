-- licensed under AGPL-3.0-or-later
-- written by Derpitron

local C = {}
C.__index = C

-- this is a delegate module.
-- it recieves all pertinent primitive data from camera and then reconstructs the necessary
-- things itself
-- this is so it can be agnostic to any vanilla chase camera architecture changes, and to be agnostic to the structure of the camera framework used
-- e.g portable to orbit, driver, etc 3rd party camera.

function C:init()
    self.dt = 0.0
    self.t = self.dt

    -- all these frames' positions and rotations are wrt the WORLD frame
    self.vehicle = {
        pos = vec3(),
        rot = quatFromAxisAngle(vec3(1,0,0),0),
        mass = 0.0,

        -- this pos is wrt `pos`.
        chassis_base = vec3(),

        vel = vec3(),
        accel = vec3(),
        jerk = vec3(),
    }

    self.target = {
        pos = vec3()
    }

    self.camera = {
        pos = vec3(),
        rot = quatFromAxisAngle(vec3(1,0,0),0),
        fov = 0.0,
    }

    self.prev.vehicle = self.vehicle
    self.prev.target = self.target
    self.prev.camera = self.camera

    print("initalised derp_sos camera")
end

function C:input(datapkg) if datapkg ~= nil then
    
end end

-- TODO: i need a better method for this. should not pollute global/fxcontrol object state with this
function C:output()
    local cam_res = {}
          cam_res.pos = self.camera.pos
          cam_res.rot = self.camera.rot
          cam_res.fov = self.camera.fov
    cam_res.targetPos = self.target.pos
    return cam_res
end

-- an example function. returns vehicle's acceleration in it's own coordinate frame.
-- todo: create module system for this
function C:get_accel_vehicle() if (self.vehicle.accel ~= nil and self.vehicle.rot ~= nil) then
    return self.vehicle.rot:inverse() * self.vehicle.accel
end end

-- DO NOT CHANGE CLASS IMPLEMENTATION BELOW

return function(...)
  local o = ... or {}
  setmetatable(o, C)
  o:init()
  return o
end
