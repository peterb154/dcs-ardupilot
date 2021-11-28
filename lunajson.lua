-- TODO: hard coding this is BS. Figure out how to do relative imports
libDir = lfs.writedir()..[[Scripts\dcs-ardupilot\lunajson\]]
local newdecoder = dofile(libDir .. 'decoder.lua')
local newencoder = dofile(libDir .. 'encoder.lua')
local sax = dofile(libDir .. 'sax.lua')
-- If you need multiple contexts of decoder and/or encoder,
-- you can require lunajson.decoder and/or lunajson.encoder directly.
return {
	decode = newdecoder(),
	encode = newencoder(),
	newparser = sax.newparser,
	newfileparser = sax.newfileparser,
}
