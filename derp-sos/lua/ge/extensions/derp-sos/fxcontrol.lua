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
    local accelF_exponential = temporal_exponential:getWithRate(accelF, dt, 8)
    local accelF_linear = temporal_linear:getWithRate(accelF, dt, 8)
    local accelF_spring = temporal_spring:getWithSpringDamp(accelF, dt, 8, 10)
    local accelF_sigmoid = temporal_sigmoid:getWithRateAccel(accelF, dt, 8, 10, 30)

    local y_scale = 15
    guihooks.graph(
        { "acc raw", accelF, y_scale, "" },
        { "linear", accelF_linear, y_scale, "" },
        { "spring", accelF_spring, y_scale, "" },
        { "sigmoid", accelF_sigmoid, y_scale, "" },
        { "exponential", accelF_exponential, y_scale, "" }
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
            dt = 0.0,    -- gfx dt
            dtSim = 0.0, -- physics dt. = 0 at game pause

            vehicle_w = {
                pos = vec3(),
                rot = quatFromAxisAngle(X, 0.0),
                vel = vec3(),
                accel = vec3()
            },

            target_w = {
                pos = vec3()
            },

            inputorbit = {
                rot__camera_t = vec3(),
                radius = 0.0
            },

            fov = 0.0
        }
    end

    --#region accel pushback
    local accel_v = input.vehicle_w.rot:inversed() * input.vehicle_w.accel
    local accelF_exponential = temporal_exponential:getWithRate(accel_v.y, input.dt, 2)
    local offset_c__accelpushback = ((input.vehicle_w.rot * Y) * ((-accelF_exponential)/20)) + ((input.vehicle_w.rot * Z) * ((-accelF_exponential)/20))
    --#endregion

    --#region inputorbit FX
    --- set the user inputted revolution/orbit of the camera
    local camera_t__inputorbit =  { pos = input.inputorbit.radius * vec3(
        math.sin(input.inputorbit.rot__camera_t.x) * math.cos(input.inputorbit.rot__camera_t.y),
        math.cos(input.inputorbit.rot__camera_t.x) * math.cos(input.inputorbit.rot__camera_t.y),
        math.sin(input.inputorbit.rot__camera_t.y)
    ) }
    --- transform the camera to world space equivalent, centred on target
    local camera_w__inputorbit = {
        pos = input.target_w.pos + ( (quatFromAxisAngle(Z, math.pi) * input.vehicle_w.rot) * camera_t__inputorbit.pos),
    }

    local camera_w__inputorbit_accelpushback = {
        pos = camera_w__inputorbit.pos + offset_c__accelpushback,
    }
    -- set camera direction to face target
    camera_w__inputorbit_accelpushback.rot = quatFromDir( (input.target_w.pos - camera_w__inputorbit_accelpushback.pos):normalized(), input.vehicle_w.rot*Z )
    --#endregion

    local camera_w__result = {
        pos = camera_w__inputorbit_accelpushback.pos,
        rot = camera_w__inputorbit_accelpushback.rot,
        fov = input.fov,
        targetPos = input.target_w.pos
    }

    return camera_w__result
end

-- DO NOT CHANGE CLASS IMPLEMENTATION BELOW

return function(...)
    local o = ... or {}
    setmetatable(o, C)
    o:init()
    return o
end
