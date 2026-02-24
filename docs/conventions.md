  --- Naming Convention
  --- <vector_name>__BODY_COORDFRAME: 
  --- this is vector `<vector_name>` (remove the angle-brackets.). it may be a point or proper (affine) vector and theres no convention differentiating the 2
  --- it is of frame/object BODY, and it's value is expressed in coordinate frame COORDFRAME.
  --- typically you should put these values in a proper vec3/luaQuat and not expose them globally, for organisation.
  ---
  --- e.g:
  ---   vel__VEHICLE_WORLD: VEHICLE'S velocity vector in world-space. 
  ---   e.g if the vehicle goes 30m/s forward in +x direction in the global coordinate frame, it would be vel__VEHICLE_WORLD = vec3(30, 0, 0)
  ---   e.g however if the vehicle goes the same 30m/s in -y direction in the global coordinate frame, it would be vel__VEHICLE_WORLD = vec3(0, -30, 0).
  ---
  --- e.g vel__VEHICLE_VEHICLE: VEHICLE's velocity vector in it's own coordinate frame.
  --- e.g no matter what direction the vehicle faces in the world, if it travels forward 30m/s, vel__VEHICLE_VEHICLE always = vec3(0, 30, 0).

    -- q*(1,0,0) -> that space's +x basis vector, but how world space sees it
  -- q*(0,1,0) -> .. +y ..
  -- q*(0,0,1) -> .. +z ..

  -- wanna rotate cam_dir? pre-multiply it with a quatFromAxisAngle(
  --  cam_dir_WORLD * whatever axis u wanna rotate around vecXYZ or whatever, 
  --  desired Angle
  -- )

  -- origin: means that now VEHICLE may be a coordinate space. "VEHICLE" is in the coordinate spaces namespace.
  -- ref is position of vehicle chassis origin, within and with respect to vehicle's local coordinate space.

  -- note: beamng camera uses perspective view. not isometric, orthographic

  give dir__CAMERA_WORLD     -- give this a world vec. it will return the world value of (that vector but if you took it in the camera's axes. it is not seeing that vector's components in the camera axes specifically.)