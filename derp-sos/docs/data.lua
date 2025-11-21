data = {
    dt = 0.015743945167376,
    dtRaw = 0.012361199999759,
    dtReal = 0.015743945167376,
    dtSim = 0.016000000759959, -- this is what we want. but its scaled with time?

    openxrSessionRunning = false,
    pos = vec3(0.00230802712, -0.01420665835, 0.1103360131), --vehicle now position
    prevPos = vec3(0.002310449956, -0.01420844533, 0.1103438288), --vehicle previous position

    -- we get these. now we want to "apply" or rather un-apply the vehicle's global rotation from these vectors -> gives us vehicle's un-rotated positions
    -- but still global.
    --vehicle current state
    vehPos = vec3(0.002308027006, -0.01420665873, 0.1103360134), --vehicle now position, double precision
    prevVehPos = vec3(0.002310450028, -0.01420844537, 0.1103438316), -- vehicle previous position, double precision

    vel = vec3(-0.0001514388862, 0.0001491649258, -0.000488642831), -- vehicle current velocity, double preicion
    prevVel = vec3(-2.55191447e-05, -0.0001720877581, 0.00052174949345), -- vehicle previous velocity, double preicision


    -- camera current state. global in value-frame of reference
    -- apply the inverse of quatFromDir(-dir, up)?
    res = {
        collisionCompatible = true,
        fov = 60,
        nearClip = 0.10000000149012,
        pos = vec3(0.002310449956, -0.01420844533, 0.1103438288),
        rot = quat(1, 0, 0, 0),
        targetPos = vec3(0.00230802712, -0.01420665835, 0.1103360131)
    },
    --better dont refer this one at all. its global and tainted. better save ur own camera's own local state.

    speed = 30,
    teleported = false,
    veh = <userdata 1>,
    vid = 21034
}
