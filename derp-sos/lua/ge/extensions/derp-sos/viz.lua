local viz = {}

--#region debugDraw tools

--- draws the given point in world space
--- 
--- `origin` default value is vec3(0,0,0)
---@param point vec3
---@param text string
---@param color colorRGBA
---@param origin? vec3 
viz.drawPoint = function(point, text, color, origin)
    origin = origin or vec3()

    local point_wrt_origin = origin + point

    debugDrawer:drawSphere(point_wrt_origin, 0.05, color, false)
    debugDrawer:drawText(point_wrt_origin, text, color, false)
end

--- draws a straight `color`'ed  line from `origin` to `point` with `text` as label on the point.
--- 
--- `origin` default value is vec3(0,0,0)
---@param point vec3
---@param text string
---@param color colorRGBA
---@param origin? vec3
viz.drawVec = function(point, text, color, origin)
    origin = origin or vec3()
    viz.drawPoint(point, text, color, origin)
    debugDrawer:drawSphere(origin, 0.05, ColorF(0, 0, 0, 1), false)
    debugDrawer:drawLine(origin, point, color, 0.04)
end

X = vec3(1, 0, 0)
Y = vec3(0, 1, 0)
Z = vec3(0, 0, 1)
--- draws the quaternion's +XYZ basis vectors in world-space with `basis_name` labelled on all 3 points rendered
--- 
--- typically basis_quat is quat(object.Y, object.Z) -> object.X
---@param basis_quat quat
---@param basis_name string
---@param origin? vec3
viz.drawBasis = function (basis_quat, basis_name, origin)
    origin = origin or vec3()
    viz.drawVec((basis_quat * X):normalized(), basis_name .. ".x", ColorF(1, 0, 0, 1), origin)
    viz.drawVec((basis_quat * Y):normalized(), basis_name .. ".y", ColorF(0, 1, 0, 1), origin)
    viz.drawVec((basis_quat * Z):normalized(), basis_name .. ".z", ColorF(0, 0, 1, 1), origin)
end

--- graphs the value continously on Generic Graph (Simple)
---@param val number
---@param text? string
viz.drawOnGraph = function(val, text)
    text = text or ''
    guihooks.graph({ text, val, 100, "" })
end

--#endregion

return viz