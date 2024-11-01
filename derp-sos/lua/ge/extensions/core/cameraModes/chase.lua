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
	self.camera_previous_frame_rotation_radians = vec3()
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
	if self.defaultRotation_euler3_degrees == nil then
		self.defaultRotation_euler3_degrees = vec3(0, -17, 0)
	else
		self.defaultRotation_euler3_degrees = vec3(self.defaultRotation_euler3_degrees)
		self.defaultRotation_euler3_degrees.y = -self.defaultRotation_euler3_degrees.y
	end
	self.camera_current_rotation_euler3_degrees = vec3(self.defaultRotation_euler3_degrees)
	self.camMinDist_wtf = self.distanceMin or 3
	self.distance = self.distance or 5
	self.defaultcameradistance_scalar_metre = self.distance
	self.cameradistance_scalar_metre = self.defaultcameradistance_scalar_metre
	self.camera_previous_frame_distance_metres = self.defaultcameradistance_scalar_metre
	self.mode = self.mode or 'ref'
	self.fov_degrees = self.fov_degrees or 65
	self.offset_vec3_metres_relativeto_what = vec3(self.offset_vec3_metres_relativeto_what)
	self.camera_base_location_afteroffset_vec3_metres_relativeto_what = vec3()
end

function C:onSettingsChanged()
	self.relaxation = settings.getValue('cameraOrbitRelaxation') or 3
	self.rollSmoothing = math.max(settings.getValue('cameraChaseRollSmoothing') or 1, 0.000001)
	self:reset() --TODO is this really necessary?
end

function C:reset()
	self.camera_current_rotation_euler3_degrees = vec3(self.defaultRotation_euler3_degrees)
	self.camera_current_rotation_euler3_degrees.x = 0
	self.forwardLooking = true
	self.camResetted = 2
	self.relative_Yaw_degreesBydegrees = 0
	self.relative_Pitch_degreesBydegrees = 0
end

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
				z_upward_unit_vector_vec3   * additive_offset_vec3.z
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
	variable_smoother_function, --TODO: define a schema for variable_smoother_function. e.g func(var, dt)
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
				x_leftward_unit_vector_vec3 * multiplicative_constant_value_coefficients_vec3.x * variable_smoother_function(rate_variable_vec3.x, dt) +
				y_backward_unit_vector_vec3 * multiplicative_constant_value_coefficients_vec3.y * variable_smoother_function(rate_variable_vec3.y, dt) +
				z_upward_unit_vector_vec3   * multiplicative_constant_value_coefficients_vec3.z * variable_smoother_function(rate_variable_vec3.z, dt)
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
		fov_offset_contribution_scalar = coefficient_scalar * variable_smoothing_function(rate_variable_magnitude, data.dt)
	end
	return fov_offset_contribution_scalar
	--#endregion rate_variable_derived_fov_offset function body
end

local rotation_euler3_radians = vec3()
function C:update(data)
	data.res.collisionCompatible         = true
	-- update input
	local deadzone                       = 0.5 --This is used to create that "snapping" effect where the camera aligns itself to pi/2 ass angles in the circle in the up facing plane i guess
	--- Some dimensionless constant bullshit
	-- These two take units degrees/degrees.
	-- I guess the relative-base value is 180 degrees
	-- but from whence does moveManager's values come in? Why mult by 0.15?
	self.relative_Yaw_degreesBydegrees   = clamp(self.relative_Yaw_degreesBydegrees + 0.15 * MoveManager.yawRelative, -1,
		1)
	self.relative_Pitch_degreesBydegrees = clamp(self.relative_Pitch_degreesBydegrees + 0.15 * MoveManager.pitchRelative,
		-1, 1)
	local relYawUsed                     = self.relative_Yaw_degreesBydegrees
	local relPitchUsed                   = self.relative_Pitch_degreesBydegrees
	-- Dont register any fucking camera rotation unless user moves the mouse more than such that the value of the above bullshit is greater than "0.5" deadzone
	if math.abs(relYawUsed) < deadzone then relYawUsed = 0 end
	if math.abs(relPitchUsed) < deadzone then relPitchUsed = 0 end


	-- INSPECT
	self.camera_current_rotation_euler3_degrees.x = 0
	if not self.forwardLooking then
		self.camera_current_rotation_euler3_degrees.x = -180 --Look backwards if we wanna look backward. Deep...
	end

	local triggerValue = 0.05

	local dx = 200 * relYawUsed + 100 * data.dt * (MoveManager.yawRight - MoveManager.yawLeft) --What the hell?
	if dx > triggerValue then
		self.camera_current_rotation_euler3_degrees.x = 90
	elseif dx < -triggerValue then
		self.camera_current_rotation_euler3_degrees.x = -90
	end
	if not self.forwardLooking then
		self.camera_current_rotation_euler3_degrees.x = -self.camera_current_rotation_euler3_degrees.x
	end

	local dy = 200 * relPitchUsed + 100 * data.dt * (MoveManager.pitchUp - MoveManager.pitchDown)
	self.camera_current_rotation_euler3_degrees.y = self.defaultRotation_euler3_degrees.y
	if dy > triggerValue then
		self.camera_current_rotation_euler3_degrees.y = self.defaultRotation_euler3_degrees.y + 30
	elseif dy < -triggerValue then
		if self.forwardLooking then
			self.camera_current_rotation_euler3_degrees.x = -180
		else
			self.camera_current_rotation_euler3_degrees.x = 0
		end
	end
	--END INSPECT

	self.camera_current_rotation_euler3_degrees.y = clamp(self.camera_current_rotation_euler3_degrees.y, -85, 85) --Yeah sure man whatever you say


	-- make sure the rotation is never bigger than 2 PI
	-- Each 180 degree branch of this if-else block is almost like each handles it's respective semicircle.
	-- When one branch overflows, it wants to pass its rotation onto the other semicircle but seamlessly to the user's appearance.
	-- Thus rotation technically jumps from 181 degrees -> -179 degrees. But they appear equivalent.
	if self.camera_current_rotation_euler3_degrees.x > 180 then
		self.camera_current_rotation_euler3_degrees.x = self.camera_current_rotation_euler3_degrees.x - 360
		self.camera_previous_frame_rotation_radians.x = self.camera_previous_frame_rotation_radians.x - math.pi * 2
	elseif self.camera_current_rotation_euler3_degrees.x < -180 then
		self.camera_current_rotation_euler3_degrees.x = self.camera_current_rotation_euler3_degrees.x + 360
		self.camera_previous_frame_rotation_radians.x = self.camera_previous_frame_rotation_radians.x + math.pi * 2
	end

	-- Where does the FOV come into play?
	local ddist = 0.1 * data.dt * (MoveManager.zoomIn - MoveManager.zoomOut) * self.fov_degrees
	self.cameradistance_scalar_metre = self.defaultcameradistance_scalar_metre
	if ddist > triggerValue then
		self.cameradistance_scalar_metre = self.defaultcameradistance_scalar_metre * 2
	elseif ddist < -triggerValue then
		self.cameradistance_scalar_metre = self.camMinDist_wtf
	end

	--Reference node positions of the car. Derived from JBeam data.
	local ref_vec3_point_carorigin_relativeto_carorigin          = data.veh:getNodePosition(self.refNodes.ref)
	local left_vec3_point_relativeto_carorigin                   = data.veh:getNodePosition(self.refNodes.left)
	local back_vec3_point_relativeto_carorigin                   = data.veh:getNodePosition(self.refNodes.back)

	-- calculate the camera offset: rotate with the vehicle
	local nx_leftwardfacing_directionvector_relativeto_carorigin = left_vec3_point_relativeto_carorigin -
		ref_vec3_point_carorigin_relativeto_carorigin
	local ny_backwardfacing_directionvector_relativeto_carorigin = back_vec3_point_relativeto_carorigin -
		ref_vec3_point_carorigin_relativeto_carorigin

	-- Im not even gonna question this. Judging by the conditional this is for some extreme freaky exceptional situation.
	if nx_leftwardfacing_directionvector_relativeto_carorigin:squaredLength() == 0 or ny_backwardfacing_directionvector_relativeto_carorigin:squaredLength() == 0 then
		data.res.pos = data.pos          -- Where does data.pos come from before this bro
		data.res.rot = quatFromDir(vecY, vecZ) -- The quaternion rotation that just so happens to be equal to the one needed to transform vecY to vecZ. Why does this exist?
		return false
	end

	-- Simple cross product to get the upward uni direction vector wrt car origin
	-- nx, ny, nz have 1 metre length/magnitude. (unit vector)
	local nz_upwardfacing_directionvector_relativeto_carorigin = nx_leftwardfacing_directionvector_relativeto_carorigin
		:cross(ny_backwardfacing_directionvector_relativeto_carorigin):normalized()

	-- What is the purpose of this?
	-- I think I get it.
	-- This is used to calculate the minimum radius of the camera path circle, whenever it rotates around the car..?
	if self.offset_vec3_metres_relativeto_what and self.offset_vec3_metres_relativeto_what.x then
		-- camerabasdfsg = offset * unit vectors of the car wrt car origin
		self.camera_base_location_afteroffset_vec3_metres_relativeto_what:set(
			self.offset_vec3_metres_relativeto_what.x /
			(nx_leftwardfacing_directionvector_relativeto_carorigin:length() + 1e-30),
			self.offset_vec3_metres_relativeto_what.y /
			(ny_backwardfacing_directionvector_relativeto_carorigin:length() + 1e-30),
			self.offset_vec3_metres_relativeto_what.z /
			(nz_upwardfacing_directionvector_relativeto_carorigin:length() + 1e-30))
	else
		self.camera_base_location_afteroffset_vec3_metres_relativeto_what:set(0, 0, 0)
	end

	-- FULCRUM HAHAHHAHAHHAHAHA IS THIS THE CAMERA ROTATION CIRCLE CENTRE
	local target_position_vec3_metres_relativeto_absolute
	-- When is it ever *not* 'center'..
	if self.mode == 'center' then
		target_position_vec3_metres_relativeto_absolute = data.veh:getBBCenter() -- Centre of geometry/masses of the car I think
		--wtf is the point or use of the below if-else branch?
	else
		-- Camera base offset in stage 2 of the pipeline = car unit direction vectors * stage 1 offset
		local camera_base_location_2_afteroffset_vec3_metres_relativeto_absolute =
			nx_leftwardfacing_directionvector_relativeto_carorigin *
			self.camera_base_location_afteroffset_vec3_metres_relativeto_what.x +
			ny_backwardfacing_directionvector_relativeto_carorigin *
			self.camera_base_location_afteroffset_vec3_metres_relativeto_what.y +
			nz_upwardfacing_directionvector_relativeto_carorigin *
			self.camera_base_location_afteroffset_vec3_metres_relativeto_what.z
		target_position_vec3_metres_relativeto_absolute = data.pos + ref_vec3_point_carorigin_relativeto_carorigin +
			camera_base_location_2_afteroffset_vec3_metres_relativeto_absolute
	end

	-- CAR UNIT VECTOR FACING BACKWARD
	local dir_vec3_metres_backwardfacing_car_unitvector_relativeto_carorigin = (ref_vec3_point_carorigin_relativeto_carorigin - back_vec3_point_relativeto_carorigin); dir_vec3_metres_backwardfacing_car_unitvector_relativeto_carorigin
		:normalize()

	if self.camResetted ~= 0 then
		self.lastDataPos = vec3(data.pos)
	end

	-- CAR UNIT VECTOR FACING UPWARD
	local up_vec3_metres_upwardfacing_car_unitvector_relativeto_carorigin =
		dir_vec3_metres_backwardfacing_car_unitvector_relativeto_carorigin:cross(left_vec3_point_relativeto_carorigin); up_vec3_metres_upwardfacing_car_unitvector_relativeto_carorigin
		:normalize()

	if self.camResetted ~= 1 then
		if self.rollSmoothing > 0.0001 then
			local upSmoothratio = 1 / (data.dt * self.rollSmoothing)
			-- linear interpolating between current and last frame's up carvec values smoothened or something
			up_vec3_metres_upwardfacing_car_unitvector_relativeto_carorigin = (1 / (upSmoothratio + 1) * up_vec3_metres_upwardfacing_car_unitvector_relativeto_carorigin + (upSmoothratio / (upSmoothratio + 1)) * self.camLastUp); up_vec3_metres_upwardfacing_car_unitvector_relativeto_carorigin
				:normalize()
		else
			-- if rolling is disabled, we are always up no matter what.
			--Always face absolutely, globally up.
			up_vec3_metres_upwardfacing_car_unitvector_relativeto_carorigin:set(vecZ)
		end
		-- linear interpolating between current and last frame's backward carvec values smoothened or something
		dir_vec3_metres_backwardfacing_car_unitvector_relativeto_carorigin:set(
			self.dirSmoothX:getUncapped(dir_vec3_metres_backwardfacing_car_unitvector_relativeto_carorigin.x,
				data.dt * 1000),
			self.dirSmoothY:getUncapped(dir_vec3_metres_backwardfacing_car_unitvector_relativeto_carorigin.y,
				data.dt * 1000),
			self.dirSmoothZ:getUncapped(dir_vec3_metres_backwardfacing_car_unitvector_relativeto_carorigin.z,
				data.dt * 1000)); dir_vec3_metres_backwardfacing_car_unitvector_relativeto_carorigin:normalize()
	end
	self.camLastUp:set(up_vec3_metres_upwardfacing_car_unitvector_relativeto_carorigin)

	-- decide on a looking direction
	-- the reason for this: on reload, the vehicle jumps and the velocity is not correct anymore
	local vel = (data.pos - self.lastDataPos) / data.dt
	-- What the hell?
	local velF = vel:dot(dir_vec3_metres_backwardfacing_car_unitvector_relativeto_carorigin)
	local velNF = vel:distance(velF * dir_vec3_metres_backwardfacing_car_unitvector_relativeto_carorigin)
	local forwardVelo = self.fwdVeloSmoother:getUncapped(velF, data.dt)
	if self.camResetted == 0 then
		if self.forwardLooking and forwardVelo < -1.5 and math.abs(forwardVelo) > velNF then
			if self.camera_current_rotation_euler3_degrees.x >= 0 then
				self.camera_current_rotation_euler3_degrees:set(self.defaultRotation_euler3_degrees)
				self.camera_current_rotation_euler3_degrees.x = 180
			else
				self.camera_current_rotation_euler3_degrees:set(self.defaultRotation_euler3_degrees)
				self.camera_current_rotation_euler3_degrees.x = -180
			end
			self.forwardLooking = false
		elseif not self.forwardLooking and forwardVelo > 1.5 then
			self.camera_current_rotation_euler3_degrees:set(self.defaultRotation_euler3_degrees)
			self.camera_current_rotation_euler3_degrees.x = 0
			self.forwardLooking = true
		end
	end
	self.lastDataPos:set(data.pos)

	rotation_euler3_radians:set(
		math.rad(self.camera_current_rotation_euler3_degrees.x),
		math.rad(self.camera_current_rotation_euler3_degrees.y),
		math.rad(self.camera_current_rotation_euler3_degrees.z)
	)

	-- smoothing
	-- linear interpolating between current and last frame's rotation values
	local ratio = 1 / (data.dt * 8)
	rotation_euler3_radians.x = 1 / (ratio + 1) * rotation_euler3_radians.x +
		(ratio / (ratio + 1)) * self.camera_previous_frame_rotation_radians.x
	rotation_euler3_radians.y = 1 / (ratio + 1) * rotation_euler3_radians.y +
		(ratio / (ratio + 1)) * self.camera_previous_frame_rotation_radians.y

	-- What the hell?
	-- Linear interpolating between current and last frame's distance values
	local dist = 1 / (ratio + 1) * self.cameradistance_scalar_metre +
		(ratio / (ratio + 1)) * self.camera_previous_frame_distance_metres

	-- Cook the final camera position we want.
	local calculatedCameraPosition_point_vec3_metres_relativeto_what = dist * vec3(
		math.sin(rotation_euler3_radians.x) * math.cos(rotation_euler3_radians.y)
		, math.cos(rotation_euler3_radians.x) * math.cos(rotation_euler3_radians.y)
		, math.sin(rotation_euler3_radians.y)
	)

	-- Funny quaternion rotation recipe from forward carvec about the upward carvec as the axis of rotation.
	local quaternion_direction_of_heading = quatFromDir(
		-dir_vec3_metres_backwardfacing_car_unitvector_relativeto_carorigin,
		up_vec3_metres_upwardfacing_car_unitvector_relativeto_carorigin)
	-- Cook the calculatedCameraPosition with the quaternion rotation sauce. Idk why?
	calculatedCameraPosition_point_vec3_metres_relativeto_what = quaternion_direction_of_heading *
		calculatedCameraPosition_point_vec3_metres_relativeto_what

	-- What the hell?
	local finalvanilla_camera_position_vec3_metres_relativeto_absolute =
		calculatedCameraPosition_point_vec3_metres_relativeto_what + target_position_vec3_metres_relativeto_absolute

	-- The line drawn from the fulcrum to the final camera position
	local dir_target_to_camera_vec3_metres_direction_vector = (target_position_vec3_metres_relativeto_absolute - finalvanilla_camera_position_vec3_metres_relativeto_absolute); dir_target_to_camera_vec3_metres_direction_vector
		:normalize()
	-- Funny quaternion rotation recipe of cam-target vec about the upward-carvec as the axis of rotation
	local finalvanilla_camera_rotation_quaternion = quatFromDir(dir_target_to_camera_vec3_metres_direction_vector,
		up_vec3_metres_upwardfacing_car_unitvector_relativeto_carorigin)

	self.camera_previous_frame_rotation_radians:set(rotation_euler3_radians)
	self.camera_previous_frame_distance_metres = dist
	self.camResetted = math.max(self.camResetted - 1, 0)

	--#region MY CODE
	local left_vec3_metres_leftfacing_car_unitvector_relativeto_carorigin = (left_vec3_point_relativeto_carorigin - ref_vec3_point_carorigin_relativeto_carorigin); left_vec3_metres_leftfacing_car_unitvector_relativeto_carorigin
		:normalized()

	local car_direction_vector_leftward    = left_vec3_metres_leftfacing_car_unitvector_relativeto_carorigin
	local car_direction_vector_backward    = dir_vec3_metres_backwardfacing_car_unitvector_relativeto_carorigin
	local car_direction_vector_upward      = up_vec3_metres_upwardfacing_car_unitvector_relativeto_carorigin
	local car_direction_vectors            = vec3(car_direction_vector_leftward, car_direction_vector_backward, car_direction_vector_upward)

	-- This seems sketchy ah hell. and is this even backward or is it forward facing wrt the car? :skull:
	local camera_direction_vector_leftward = finalvanilla_camera_rotation_quaternion * vec3(1, 0, 0)
	local camera_direction_vector_backward = finalvanilla_camera_rotation_quaternion * vec3(0, 1, 0)
	local camera_direction_vector_upward   = finalvanilla_camera_rotation_quaternion * vec3(0, 0, 1)
	local camera_direction_vectors         = vec3(camera_direction_vector_leftward, camera_direction_vector_backward, camera_direction_vector_upward)

	local ground_state_camera_position     = finalvanilla_camera_position_vec3_metres_relativeto_absolute
	local ground_state_fov                 = 65
	local ground_state_camera_rotation     = finalvanilla_camera_rotation_quaternion
	local ground_state_target_position     = target_position_vec3_metres_relativeto_absolute

	--- TO IMPLEMENT
	-- orientation: defined as the camera spinning about its own axes.
	-- For single letter codes after functions, check the design document's table.
	-- TODO: What the fuck is an impact/collision? How can we quantify it? How does NFS rivals/heat/unbound do it? How does assetto corsa Kirbycam do it?
	-- TODO: The children yearn for smoothing and interpolation. Find out how the fuck one implements smoothing, and what smoothie functions/dt/etc to use?
	-- TODO: See about exposing these functions to global namespace so that any module or script could call it?
	-- TODO: Inspect BeamNG World Editor's World and Local rotation again? We need to find out how the shit we implement J and N in my design document.
	-- TODO: Do impact based effects fall into PHASE 2: PERIODIC EFFECTS, or should they be a discrete PHASE 3?
	--
	-- I think of the phases like a stack where each item at the top is the latest in the pipeline.
	-- {vanilla_camera, PHASE 0, PHASE 1, PHASE 2}
	-- Never fuck with vanilla_camera. It is sacred and no function here should be allowed to tamper with it's calculations.
	-- PHASE 2 mustn't fuck with PHASE 1.
	-- PHASE 2 thinks of PHASE 1 as it's mean/ground state position to return to.
	-- PHASE 1 Thinks of PHASE 0 as it's mean/ground state position to return to.
	-- PHASE 0 reverts back to vanilla_camera if the user doesn't change anything in it.                -- K (I think) (Where should this go? If user wants Dolly Zoom, it Must somehow override any other position/FOV settings.)

	-- These to be summed/added to the ground state position/rotation/FOV.
	-- This is the real meat and potatoes. This is where "the user" should define the parameters of the function they want to use.
	local final_offset_vec3 -- =
	--Rate_variable_derived_position_offset{enabled_boolean=true, rate_variable_vec3=data.vel, direction_vectors_vec3_vec3=camera_direction_vectors} +
	--Rate_variable_derived_position_offset{enabled_boolean=true, rate_variable_vec3=data.acc, direction_vectors_vec3_vec3=   car_direction_vectors}

	local final_fov_offset_scalar -- =
	--Rate_variable_derived_fov_offset{enabled_boolean=vtrue, rate_variable_vec3=data.vel} +
	--Rate_variable_derived_fov_offset{enabled_boolean=true, rate_variable_vec3=data.acc}

	local final_orientation_offset_quat -- =

	-- FINAL FINAL FINAL application
	-- thta means APPLYING the FUCKING cooked values to the CAMERA RAHHHHH
	-- These are the data to be applied to the real camera itself
	-- I.e what the user sees.
	data.res.pos                           = ground_state_camera_position + final_offset_vec3
	data.res.fov                           = ground_state_fov + final_fov_offset_scalar
	data.res.rot                           = ground_state_camera_rotation * final_orientation_offset_quat

	-- This is extra metadata we pass back to the camera for the next frame.
	-- I.e we trick the camera into believing nothing is amiss and it calculates everything as vanilla.
	data.res.pos_ground                    = ground_state_camera_position
	data.res.fov_ground                    = ground_state_fov
	data.res.rot_ground                    = ground_state_camera_rotation
	data.res.targetPos                     = ground_state_target_position

	--#endregion

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
	self.relative_Yaw_degreesBydegrees = 0
	self.relative_Pitch_degreesBydegrees = 0
end

-- DO NOT CHANGE CLASS IMPLEMENTATION BELOW

return function(...)
	local o = ... or {}
	setmetatable(o, C)
	o:init()
	return o
end
