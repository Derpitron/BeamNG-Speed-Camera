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
    guihooks.graph({ text, val, 100, "" })
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

--#endregion

--#region fx
-- quaternion and vec3 methods available in beamng engine: beamng/common/mathlib.lua (do NOT use euler angle api here. it is outdated and inconsistent. use quaternions to implement desired rotations in a specified, consistent order of axes.)
-- function smoothers, filters, etc (signal processing): beamng/common/filters.lua

local temporal_exponential = newTemporalSmoothingNonLinear()
local sin, cos = math.sin, math.cos

local C = {}
C.__index = C

-- this is a delegate module.
-- it recieves all pertinent primitive data from camera and then reconstructs the necessary things itself
-- this is so it can be agnostic to the structure of the actual source camera.
-- useful for if the game updates or you want to port to another camera.

function C:init()
    print("initalised derp_sos camera")
end

function C:hello_world()
    print("hello from derp_sos!")
end

function C:calculate(input)
    if input == nil then
        input = {
            dt = 0.0,    -- gfx dt
            dtSim = 0.0, -- physics dt. = 0 at game pause

            veh_w = {
                pos = vec3(),
                rot = quatFromAxisAngle(X, 0.0),
                vel = vec3(),
                accel = vec3()
            },

            target_w = {
                pos = vec3()
            },

            inputorbit = {
                cam__rot_t = vec3(),
                radius = 0.0
            },

            fov = 0.0
        }
    end

    --#region inputorbit FX
    --- set the user inputted revolution/orbit of the camera
    --- this is where camera construction begins
    local cam_w = vec3( -- input orbit
        sin(input.inputorbit.cam__rot_t.x) * cos(input.inputorbit.cam__rot_t.y),
        cos(input.inputorbit.cam__rot_t.x) * cos(input.inputorbit.cam__rot_t.y),
        sin(input.inputorbit.cam__rot_t.y)
    )
    cam_w:setScaled(input.inputorbit.radius)
    cam_w:setRotate(quatFromAxisAngle(Z, math.pi) * input.veh_w.rot) -- implements world-space position *behind* car
    cam_w:setAdd(input.target_w.pos)

    -- axis direction from cam to target. angle is so the quat-rotation may point upwards
    local dir_cam_w = quatFromDir(
        input.target_w.pos - cam_w, -- dir
        input.veh_w.rot*Z           -- up
    )

    --#endregion

    local cam_w__result = {
        pos = cam_w,
        rot = dir_cam_w,
        fov = input.fov,
        targetPos = input.target_w.pos
    }

    return cam_w__result
end

-- DO NOT CHANGE CLASS IMPLEMENTATION BELOW

return function(...)
    local o = ... or {}
    setmetatable(o, C)
    o:init()
    return o
end
