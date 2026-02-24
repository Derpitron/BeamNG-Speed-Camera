-- licensed under AGPL-3.0-or-later
-- written by Derpitron

--#region tools
local function drawPoint(point, text, color, origin)
    if origin == nil then origin = vec3() end

    local newPoint = point
    newPoint:setAdd(origin)

    debugDrawer:drawSphere(newPoint, 0.05, color, false)
    debugDrawer:drawText(newPoint, text, color, false)
    print(text, dump(point))
end

local function drawVec(point, text, color, origin)
    if origin == nil then origin = vec3() end
    drawPoint(point, text, color, origin)
    debugDrawer:drawSphere(origin, 0.05, ColorF(0, 0, 0, 1), false)
    debugDrawer:drawLine(origin, point, color, 0.04)
end

-- typically basis_quat is the object's forward facing direction
local function drawBasis(basis_quat, basis_name, origin)
    if origin == nil then origin = vec3() end
    drawVec((basis_quat * vec3(1, 0, 0)):normalized(), basis_name .. ".x", ColorF(1, 0, 0, 1), origin)
    drawVec((basis_quat * vec3(0, 1, 0)):normalized(), basis_name .. ".y", ColorF(0, 1, 0, 1), origin)
    drawVec((basis_quat * vec3(0, 0, 1)):normalized(), basis_name .. ".z", ColorF(0, 0, 1, 1), origin)
end

local function drawOnGraph(val, text)
    guihooks.graph({ text, val, 1000, "" })
end

local function round(vec, eps)
    eps = eps or 1
    local out = vec3()
    out.x = math.floor(vec.x / eps + 0.5) * eps
    out.y = math.floor(vec.y / eps + 0.5) * eps
    out.z = math.floor(vec.z / eps + 0.5) * eps
    return out
end

local function clamp(val, low, high)
    return math.min(math.max(val, low), high)
end

X = vec3(1, 0, 0)
Y = vec3(0, 1, 0)
Z = vec3(0, 0, 1)

local sin, cos, pi, rad = math.sin, math.cos, math.pi, math.rad

--#endregion

-- quaternion and vec3 methods available in beamng engine: beamng/common/mathlib.lua (do NOT use euler angle api here. it is outdated and inconsistent. use quaternions to implement desired rotations in a specified, consistent order of axes.)
-- function smoothers, filters, etc (signal processing): beamng/common/filters.lua

local temporal_exponential = newTemporalSmoothingNonLinear()

local C = {}
C.__index = C

-- this is a delegate module.
-- it recieves all pertinent primitive data from camera and then reconstructs the necessary things itself
-- this is so it can be agnostic to the structure of the actual source camera.
-- useful for if the game updates or you want to port to another camera.

function C:init()
    print("initalised derp_sos camera")
end

function C:calculate(input)
    --#region data schema
    if input == nil then
        -- coordinate frame conventions:
        -- +x: rightward
        -- +y: forward
        -- +z: upward
        -- always, even for vehicle.
        input = {
            dt = 0.0,    -- gfx dt
            dtSim = 0.0, -- physics dt. = 0 at game pause

            veh = {
                pos = vec3(),  -- position of `ref` node in world
                rot = quat(),  -- rotation of vehicle in world
                vel = vec3(),  -- local vehicle velocity
                accel = vec3() -- local vehicle acceleration
            },

            target_v = vec3(), -- wrt veh.pos, using the above coordinate axe conventions

            pan = {
                yaw = 0.0,   -- rads
                pitch = 0.0, -- rads
                radius = 0.0 -- metres
            },

            fov = 0.0 -- degrees
        }
    end
    --#endregion

    --#region FX: pan
    local cam_t__pan = vec3( -- input orbit
        -1 * sin(input.pan.yaw) * cos(input.pan.pitch),
        -1 * cos(input.pan.yaw) * cos(input.pan.pitch),
        sin(input.pan.pitch)
    )
    cam_t__pan:setScaled(input.pan.radius)

    local dir_cam_v = quatFromDir(
        -cam_t__pan, -- dir
        Z            -- up
    )
    local cam_v__pan = cam_t__pan + input.target_v
    --#endregion

    local cam_w__result = {
        pos = input.veh.pos + (input.veh.rot * cam_v__pan),
        rot = dir_cam_v * input.veh.rot,
        fov = input.fov,
        targetPos = input.veh.pos + (input.veh.rot * input.target_v)
    }
    return cam_w__result
end

--#region boilerplate
-- DO NOT CHANGE CLASS IMPLEMENTATION BELOW

return function(...)
    local o = ... or {}
    setmetatable(o, C)
    o:init()
    return o
end
--#endregion
