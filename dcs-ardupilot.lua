-- https://github.com/ArduPilot/ardupilot/tree/master/libraries/SITL/examples/JSON
local DcsArdupilot = {}
DcsArdupilot.modName = 'dcs-ardupilot'
DcsArdupilot.modDir = lfs.writedir()..[[Scripts\\]]..DcsArdupilot.modName..[[\\]]

-- TODO: move these to a config file
-- DcsArdupilot.configFile = DcsArdupilot.modDir..[[config.json]]
DcsArdupilot.serverHost = "*" --"127.0.0.1"
DcsArdupilot.serverPort = 9002
lunajson = dofile(DcsArdupilot.modDir .. "lunajson.lua")
struct = dofile( DcsArdupilot.modDir .. "struct.lua")
-- Not sure what these do
--DcsArdupilot.gArguments = {[404]="%.1f"}

-- Simulation id
--gSimID = string.format("%08x",os.time())

-- State data for export
--gSendStrings = {gSimID, '*'}
--gLastData = {}

function LuaExportStart()
    -- Works once just before mission start.
    log.write(DcsArdupilot.modName, log.INFO, "Loading modName: " .. DcsArdupilot.modName)

    -- Capture our home position
    start_data = LoGetSelfData()
    start_pos = {}
    start_pos.lat = start_data.LatLongAlt.Lat
    start_pos.long = start_data.LatLongAlt.Long
    start_pos.alt = start_data.LatLongAlt.Alt
    log.write(DcsArdupilot.modName, log.DEBUG, "Home Position: " .. lunajson.encode(start_pos))

    -- create a socket
    log.write(DcsArdupilot.modName, log.INFO, "Opening socket on host : " .. DcsArdupilot.serverHost
            .. ", port: " .. DcsArdupilot.serverPort)
    socket = require("socket")
    udp = socket.udp()
    udp:setsockname(DcsArdupilot.serverHost, DcsArdupilot.serverPort)
    udp:settimeout(.01) -- set the timeout for reading the socket. This is blocking, so make it short
end


function LuaExportBeforeNextFrame()
    -- See if we got control inputs from autopilot
    host, port = ProcessInput()
    if host and port then
        -- If we got a datagram, host and port will not be nill, send position data back
        ProcessOutput(host, port)
    end
end

function LuaExportAfterNextFrame()
    -- See if we are sending something to sim
    --ProcessOutput()
end

function LuaExportStop()
    -- Works once just after mission stop.
    -- close the socket
    log.write(DcsArdupilot.modName, log.INFO, "Closing socket")
    udp:close()
    log.write(DcsArdupilot.modName, log.INFO, "Closing modName: " .. DcsArdupilot.modName)
end

function LuaExportActivityNextEvent(t)
    local tNext = t
    return tNext
end


function ProcessInput()
    -- check to see if we recieved data from autopilot
    local inputData, host, port = udp:receivefrom()
    if inputData then
        -- if we have inputData, we got something
        structFormat = "HHIHHHHHHHHHHHHHHHH" -- the format of data we receive from ardupilot
        local magic, frame_rate, frame_count, p1, p2, p3, p4, p5, p6, p7, p8, p9, p10, p11, p12, p13, p14, p15, p16 = struct.unpack(structFormat, inputData)
        data = {}
        data.magic = magic -- will always be 18458
        data.frame_rate = frame_rate -- can ignore, autopilot will respond as fast as sim loops (1x per frame)
        data.frame_count = frame_count -- the number of frames teh autopilot has been with us
        data.pwm = {p1, p2, p3, p4, p5, p6, p7, p8, p9, p10, p11, p12, p13, p14, p15, p16} -- 16 pwm servo signals
        -- every x000 frames create a log entry with frame count and frame rate so we know things are working
        if math.fmod(data.frame_count, 2000) == 0 then
            log.write(DcsArdupilot.modName, log.DEBUG, "ArduPilot Input:" .. host .. ":" ..  port
                    .. ", frame_count:" .. data.frame_count .. ", frame rate:" .. data.frame_rate .. "Hz")
        end
        if math.fmod(data.frame_count, 100) == 0 then
            log.write(DcsArdupilot.modName, log.DEBUG, "PWM" .. lunajson.encode(data.pwm))
        end
        -- TODO drive inputs to sim
        -- the output process will need the host and port to which replies should be sent
        return host, port
    end
    -- we didnt get a packet from autopilot.. Return nil values so we dont bother processing outputs
    return nil, nil
end

function ProcessOutput(host, port)
    -- https://github.com/peterb154/dcsStats/blob/master/docs/Export.lua
    self_data = LoGetSelfData() -- get information about self
    accel = LoGetAccelerationUnits()
    heliFm = LoGetHelicopterFMData()
    velocity = LoGetVectorVelocity()
    north_distance = getLatDistMeters(start_pos.lat, self_data.LatLongAlt.Lat)
    east_distance = getLatDistMeters(start_pos.long, self_data.LatLongAlt.Long)
    down_distance = start_pos.alt - self_data.LatLongAlt.Alt
    -- Prepare return data
    returnData = {}
    returnData.timestamp = LoGetModelTime() -- (s) physics time
    returnData.imu = {}
    returnData.imu.gyro = {heliFm.angular_speed.x, heliFm.angular_speed.y, heliFm.angular_speed.z} -- (roll, pitch, yaw) (radians/sec) body frame
    returnData.imu.accel_body = {accel.x, accel.y, accel.z} -- (x, y, z) (m/s^2) body frame
    returnData.position = {north_distance, east_distance, down_distance} --(north, east, down) (m) earth frame offset from home
    returnData.attitude = {self_data.Bank, self_data.Pitch, self_data.Heading} --(roll, pitch, yaw) (radians)
    returnData.velocity = {velocity.x, velocity.y, velocity.z} --(north, east, down) (m/s) earth frame
    returnData.airspeed = LoGetTrueAirSpeed() --(m/s)
    returnData.rng_1 = LoGetAltitudeAboveGroundLevel() -- meters
    returnJson = "\n" .. lunajson.encode(returnData) .. "\n"

    pitch_deg = self_data.Pitch * 57.2958
    bank_deg = self_data.Bank * 57.2958
    yaw_deg = self_data.Heading * 57.2958

    log.write(DcsArdupilot.modName, log.DEBUG, "Attitude:"
            .. " Pitch:" .. self_data.Pitch .. "rad " .. pitch_deg
            .. ", Bank: " .. self_data.Bank .. "rad " .. bank_deg
            .. ", Heading:" .. self_data.Heading .. "rad " .. yaw_deg
    )
    --log.write(DcsArdupilot.modName, log.DEBUG, "Sending via udp to host: " .. host .. ":" .. port
    --        .. " frame:" .. data.frame_count .. " data: " .. returnJson)

    -- send telemetry data back to autopilot
    udp:sendto(returnJson, host, port)
end

-- Return the distance in meters north from starting decimal latitude to a current latitude
function getLatDistMeters(start_lat, curr_lat)
    return (curr_lat - start_lat) * 110.95 * 1000 -- 111 km per degree of latitude
end

-- Return the distance in meters east from starting decimal longitude to a current longitude
function getLongDistMeters(start_long, cur_long)
    return (start_long - cur_long) * 110.95 * 1000 -- 111 km per degree of longitude (dcs world is flat)
end

-- Helper Functions
--function ResetChangeValues()
--    logfile:write("Sending all values.", "\n")
--    for lArgument, lFormat in pairs(gArguments) do
--        gLastData[lArgument] = "99999"
--    end
--end

--function StrSplit(str, delim, maxNb)
--    -- Eliminate bad cases...
--    if string.find(str, delim) == nil then
--        return { str }
--    end
--    if maxNb == nil or maxNb < 1 then
--        maxNb = 0    -- No limit
--    end
--    local result = {}
--    local pat = "(.-)" .. delim .. "()"
--    local nb = 0
--    local lastPos
--    for part, pos in string.gfind(str, pat) do
--        nb = nb + 1
--        result[nb] = part
--        lastPos = pos
--        if nb == maxNb then break end
--    end
--    -- Handle the last field
--    if nb ~= maxNb then
--        result[nb + 1] = string.sub(str, lastPos)
--    end
--    return result
--end

--function fileExists(file_name)
--    local file=io.open(file_name, "r")
--    if file==nil then return false end
--    file:close()
--    return true
--end


--function round(num, idp)
--    local mult = 10^(idp or 0)
--    return math.floor(num * mult + 0.5) / mult
--end

-- Status Gathering Functions
--function ProcessMainPanel()
--    local lArgument , lFormat , lArgumentValue
--    local HSI    = LoGetControlPanel_HSI()
--    local lDevice = GetDevice(0)
--    lDevice:update_arguments()
--
--    for lArgument, lFormat in pairs(gArguments) do
--        lArgumentValue = string.format(lFormat,lDevice:get_argument_value(lArgument))
--        SendData(lArgument, lArgumentValue)
--    end
--end

-- Network Functions

--function SendData(id, value)
--
--    if string.len(value) > 3 and value == string.sub("-0.00000000",1, string.len(value)) then
--        value = value:sub(2)
--    end
--
--    if gLastData[id] ~= value then
--        table.insert(gSendStrings, id .. "=" .. value)
--        gLastData[id] = value
--
--        if #gSendStrings > 140 then
--            socket.try(udpIn:send(table.concat(gSendStrings, ":").."\n"))
--            gSendStrings = {gSimID, '*'}
--        end
--    end
--end

--function FlushData()
--    if #gSendStrings > 0 then
--        socket.try(udpIn:send(table.concat(gSendStrings, ":").."\n"))
--        gSendStrings = {gSimID, '*'}
--    end
--end

--function getArduPilotIp()
--    local lInput, host, port = udpIn:receivefrom()
--    return host, port
--end


--function table.slice(tbl, first, last, step)
--    local sliced = {}
--
--    for i = first or 1, last or #tbl, step or 1 do
--        sliced[#sliced+1] = tbl[i]
--    end
--
--    return sliced
--end

--self_data = LoGetSelfData()
--self_data: {
--    "Pitch":-0.40197667479515076,
--    "Bank":-2.5361464023590088
--    "Type":{"level3":1,"level1":1,"level4":63,"level2":1},
--    "Country" :2,
--    "GroupName":"P-51D 1",
--    "Flags":{
--        "Jamming":false,
--        "IRJamming":false,
--        "Born":true,
--        "Static":false,
--        "Invisible":false,
--        "Human":true,
--        "AI_ON":true,
--        "RadarActive":false},
--    "Coalition":"Enemies",
--    "Heading":5.9856127202510834,
--    "Name":"P-51D",
--    "Position":{
--            "y":964.82305889272232,
--            "x":-301762.93725766777,
--            "z":616941.5437520761
--    },
--    "UnitName":"New callsign",
--    "LatLongAlt":{"Long":41.660127480734502,"Lat":42.091277705478156,"Alt":964.82305889272232},
--    "CoalitionID":2,
--}

--accel = LoGetAccelerationUnits()
--accel: {
--    "y":1.9723620414733887,
--    "x":0.056879669427871704,
--    "z":-0.61202901601791382
--}

--heliFm = LoGetHelicopterFMData()
--heliFm: {
--    "pitch":-0.094340428709983826,
--    "angular_speed":{
--        "y":0.078427798810916299,
--        "x":-0.87308068202101108,
--        "z":-0.18558780724066243
--    },
--    "G_factor":{
--        "y":1.8350873829049823,
--        "x":0.046292543871402442,
--        "z":-0.38158822895674716
--    },
--    "yaw":5.6689238548278809,
--    "speed":{
--        "y":-10.212099848013555,
--        "x":129.3692824482346,
--        "z":43.732588050640715
--    },
--    "roll":-0.52198284864425659,
--    "angular_acceleration":{
--        "y":-0.42924940680227641,
--        "x":-0.054684072772725419,
--        "z":-0.17442057446637346
--    },
--    "acceleration":{
--        "y":10.871967615510123,
--        "x":1.4805861588495999,
--        "z":-10.862435390206837
--    }
--}
