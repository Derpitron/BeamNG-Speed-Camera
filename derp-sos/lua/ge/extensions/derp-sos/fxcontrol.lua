--#region prelude
local X = vec3(1, 0, 0)
local Y = vec3(0, 1, 0)
local Z = vec3(0, 0, 1)
local sin, cos, pi, rad = math.sin, math.cos, math.pi, math.rad
--#endregion

local viz = require("derp-sos/viz")
local types = require("derp-sos/types")

--#region customfilters


--- round `vec` down to nearest `eps` value component-wise
local function round(vec, eps)
	eps = eps or 1
	local out = vec3(
		math.floor(vec.x / eps + 0.5) * eps,
		math.floor(vec.y / eps + 0.5) * eps,
		math.floor(vec.z / eps + 0.5) * eps
	)
	return out
end

local function clamp(val, low, high)
	low = low or FLT_MIN
	high = high or FLT_MAX
	return math.min(math.max(val, low), high)
end

--TODO: requiring ffi lib is ugly.
local prev = {} -- caches up to `1` previous value of the input.
--- returns difference between current value and previous value. stateful function
local function delta(val, varname)
	if prev[varname] == nil then prev[varname] = types.zero(types.get_lua_type(val)) end
	local buffered_prev_val = prev[varname]
	prev[varname] = val
	return val - buffered_prev_val
end

--#endregion

-- quat, vec3 library:			 beamng/common/mathlib.lua (do NOT use euler angles at all. they're deprecated and implemented inconsistently)
-- smoothers, signal processing: beamng/common/filters.lua

local vel_smooth_X = newTemporalSmoothingNonLinear()
local vel_smooth_Y = newTemporalSmoothingNonLinear()
local vel_smooth_Z = newTemporalSmoothingNonLinear()

local C = {}
C.__index = C

function C:init()
	print("initialised derp_sos camera")
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
			dt = 0.0,
			dtSim = 0.0,

			veh = {
				ref = { -- jbeam refnode based vehicle dimensions
					pos = vec3(),
					rot = quat(),
					target_offset = vec3(),

					vel = vec3(),
					accel = vec3(),
				},

				bb = { -- object-oriented bounding box based vehicle dimensions
					pos = vec3(),
					left = vec3(),
					rear = vec3(),
					up = vec3(),
				}
			},

			pan = {
				yaw = rot.x,
				pitch = rot.y,
				radius = dist
			},

			fov = self.fov
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
		Z      -- up
	)
	local cam_v__pan = vec3(cam_t__pan + input.veh.ref.target_offset)
	--#endregion

	--#region EDITABLE FX
	-- you may replace ANY term from hereon out.

	local vel_bb_w = vec3(delta(input.veh.bb.pos, "input.veh.bb.pos") / input.dt)
	local vel_bb_w_smooth = vec3(
		vel_smooth_X:getWithRate(vel_bb_w.x, input.dt, 8),
		vel_smooth_Y:getWithRate(vel_bb_w.y, input.dt, 8),
		vel_smooth_Z:getWithRate(vel_bb_w.z, input.dt, 8)
	)
	if vel_bb_w_smooth:length() < 5 then vel_bb_w_smooth = vec3(-input.veh.bb.rear) end
	local bouncy_veh_rot = quatFromDir(vel_bb_w_smooth, Z)

	local cam_w__result = {
		pos = input.veh.ref.pos + (bouncy_veh_rot * cam_v__pan),
		rot = dir_cam_v * bouncy_veh_rot,
		fov = input.fov,
		targetPos = input.veh.ref.pos + (input.veh.ref.rot * input.veh.ref.target_offset)
	}
	--#endregion
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
