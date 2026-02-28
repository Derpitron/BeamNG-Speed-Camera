---@alias vec3 table
---@alias quat table
---@alias colorRGBA "colorF"

local types = {}

---returns `zero` of the input type.
types.zero = function(T)
	if T == 'vec3' then return vec3()
	elseif T == 'quat' then return quat()
    else return 0
end end

local ffi = require('ffi')
---returns type of the input value
types.get_lua_type = function(val)
	if ffi.istype('struct __luaVec3_t', val) then return 'vec3'     -- type-name defined ONLY within LuaLS.
	elseif ffi.istype('struct __luaQuat_t', val) then return 'quat' -- type-name defined ONLY within LuaLS.
	else return type(val)
end end

return types