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

    -- create a socket
    socket = require("socket")
    log.write(DcsArdupilot.modName, log.INFO, "Opening socket on host : " .. DcsArdupilot.serverHost .. ", port: " .. DcsArdupilot.serverPort)
    udpIn = socket.udp()
    udpIn:setsockname(DcsArdupilot.serverHost, DcsArdupilot.serverPort)
    udpIn:settimeout(.01) -- set the timeout for reading the socket. This is blocking, so make it short

end


function LuaExportBeforeNextFrame()
    -- See if we got something from sim
end

function LuaExportAfterNextFrame()
    -- See if we are sending something to sim
    ProcessOutput()
end

function LuaExportStop()
    -- Works once just after mission stop.
    log.write(DcsArdupilot.modName, log.INFO, "Closing modName: " .. DcsArdupilot.modName)
    -- close the socket
    log.write(DcsArdupilot.modName, log.INFO, "Closing socket")
    udpIn:close()
end

function LuaExportActivityNextEvent(t)
    local tNext = t
    host, port = ProcessInput()
    ProcessOutput(host, port)
    return tNext
end


function ProcessInput()
    log.write(DcsArdupilot.modName, log.DEBUG, "Processing input")
    local lInput, host, port = udpIn:receivefrom()

    if lInput then
        --log.write(DcsArdupilot.modName, log.DEBUG, "peer: " .. host .. ":" .. port)
        structFormat = "HHIHHHHHHHHHHHHHHHH" -- the format of data we receive from ardupilot
        local magic, frame_rate, frame_count,
        p1, p2, p3, p4, p5, p6, p7, p8, p9, p10, p11, p12, p13, p14, p15, p16 = struct.unpack(structFormat, lInput)
        data = {}
        data.magic = magic
        data.frame_rate = frame_rate
        data.frame_count = frame_count
        data.pwm = {p1, p2, p3, p4, p5, p6, p7, p8, p9, p10, p11, p12, p13, p14, p15, p16}
        log.write(DcsArdupilot.modName, log.DEBUG, "Compiled data input: " .. lunajson.encode(data))
        -- TODO drive inputs to sim
        return host, port
    end
end

function ProcessOutput(host, port)
    --  package.path  = package.path..";"..lfs.currentdir().."/LuaSocket/?.lua"
    --  package.cpath = package.cpath..";"..lfs.currentdir().."/LuaSocket/?.dll"
    returnData = {}
    returnData.timestamp = LoGetModelTime() -- (s) physics time
    returnData.imu = {0, 0, 0} -- gyro(roll, pitch, yaw) (radians/sec) body frame
    returnData.accel_body = {1, 0, 0} --(x, y, z) (m/s^2) body frame
    returnData.position = {0, 0, 0} --(north, east, down) (m) earth frame
    returnData.velocity = {0, 0, 0} --(north, east, down) (m/s) earth frame
    returnData.airspeed = 0 --(m/s)
    returnJson = lunajson.encode(returnData)

    log.write(DcsArdupilot.modName, log.INFO, "Sending via udp to host: " .. host .. ":" .. port .. " data:" .. returnJson)
    udpOut = socket.udp()
    udpOut:sendto(returnJson, host, port)
    udpOut.close()
    log.write(DcsArdupilot.modName, log.INFO, "closed socket")
    --returnJson = lunajson.encode(returnData)
    --ProcessMainPanel()
    --FlushData()
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
