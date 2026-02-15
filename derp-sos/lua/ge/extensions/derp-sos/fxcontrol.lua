-- licensed under AGPL-3.0-or-later
-- written by Derpitron

--#region tools
local function drawPoint(point, text, color)
    debugDrawer:drawSphere(point, 0.05, color, false)
    debugDrawer:drawText(point, text, color, false)
    print(text, dump(point))
end

local function drawVec(point, text, color)
    drawPoint(point, text, color)
    debugDrawer:drawSphere(vec3(0, 0, 0), 0.05, ColorF(0, 0, 0, 1), false)
    debugDrawer:drawLine(vec3(0, 0, 0), point, color, 0.04)
end

local function drawBasis(basis_quat, basis_name)
    -- typically basis_quat is the object's forward facing direction
    drawVec((basis_quat * vec3(1, 0, 0)):normalized(), basis_name .. ".x", ColorF(1, 0, 0, 1))
    drawVec((basis_quat * vec3(0, 1, 0)):normalized(), basis_name .. ".y", ColorF(0, 1, 0, 1))
    drawVec((basis_quat * vec3(0, 0, 1)):normalized(), basis_name .. ".z", ColorF(0, 0, 1, 1))
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

X = vec3(1, 0, 0)
Y = vec3(0, 1, 0)
Z = vec3(0, 0, 1)

--#endregion

--#region fx
-- quaternion and vec3 methods available in beamng engine: beamng/common/mathlib.lua (do NOT use euler angle api here. it is outdated and inconsistent. use quaternions to implement desired rotations in a specified, consistent order of axes.)
-- function smoothers, filters, etc (signal processing): beamng/common/filters.lua

local temporal_linear = newTemporalSmoothing()
local temporal_spring = newTemporalSpring()
local temporal_sigmoid = newTemporalSigmoidSmoothing()
local temporal_exponential = newTemporalSmoothingNonLinear()

local function accel_pitch(
    accel__VEHICLE_WORLD,
    dir__VEHICLE_WORLD,
    dir__CAMERA_WORLD,
    dt
)
    local accel__VEHICLE = (dir__VEHICLE_WORLD:inversed() * accel__VEHICLE_WORLD)
    local accelF = accel__VEHICLE.y
    local accelF_linear = temporal_linear:getWithRate(accelF, dt, 8)
    local accelF_spring = temporal_spring:getWithSpringDamp(accelF, dt, 8, 10)
    local accelF_sigmoid = temporal_sigmoid:getWithRateAccel(accelF, dt, 8, 10, 30)
    local accelF_exponential = temporal_exponential:getWithRate(accelF, dt, 8)

    local y_scale = 15
    guihooks.graph(
        { "acc raw", accelF, y_scale, "" },
        { "linear", accelF_linear, y_scale, ""},
        { "spring", accelF_spring, y_scale, ""},
        { "sigmoid", accelF_sigmoid, y_scale, ""},
        { "exponential", accelF_exponential, y_scale, ""}
    )

    -- this sets the axis to be the vehicle's x axis, but in terms the camera understands. you may set only `X` as well if you want
    -- vehicle -> camera = (veh -> world) -> (cam -> world)^-1
    local pitch_axis_accel = (dir__VEHICLE_WORLD * dir__CAMERA_WORLD:inversed()) * X -- pitch
    local angle_rad_accel = math.rad(-(
        accelF_exponential
    ))
    local pitch_accel__PITCH_CAMERA = quatFromAxisAngle(pitch_axis_accel, angle_rad_accel)

    return pitch_accel__PITCH_CAMERA
end

--#endregion

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
            dt = 0.0,

            origin__VEHICLE__WORLD = vec3(),
            chassis_base__VEHICLE__WORLD = vec3(),
            origin__TARGET__WORLD = vec3(),
            -- TODO: shim calculatedCamPos

            dir__VEHICLE_WORLD = quatFromDir(vec3(1, 0, 0), 0),
            dir__CAMERA_WORLD = quatFromDir(vec3(1, 0, 0), 0),

            vel__VEHICLE_WORLD = vec3(),
            accel__VEHICLE_WORLD = vec3(),
        }
    end

    -- the order of application matters here.
    local final_rot = accel_pitch(input.accel__VEHICLE_WORLD, input.dir__VEHICLE_WORLD, input.dir__CAMERA_WORLD, input.dt) *
    input.dir__CAMERA_WORLD

    local cam_res = {
        pos = vec3(),
        rot = final_rot,
        fov = 0.0,
        targetPos = vec3()
    }

    return cam_res
end

-- DO NOT CHANGE CLASS IMPLEMENTATION BELOW

return function(...)
    local o = ... or {}
    setmetatable(o, C)
    o:init()
    return o
end
