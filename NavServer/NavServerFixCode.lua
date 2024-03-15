local function fixSpaceCode(filename)
	local f = io.open(filename, "rb")
	local s = f:read "*a"
	f:close(f)

	local t = s:gsub("[ \t\r]+\n", "\n")
			   :gsub("\n(\t+)", function(s) return "\n" .. string.rep(" ", #s * 4) end)
	if s ~= t then
		f = io.open(filename, "wb")
		f:write(t)
		f:close()
		print(" * " .. filename)
	end
end

local function fixTabCode(filename)
	local f = io.open(filename, "rb")
	local s = f:read "*a"
	f:close(f)

	local t = s:gsub("[ \t\r]+\n", "\n")
			   :gsub("\n( +)", function(s) return "\n" .. string.rep("\t", math.floor(#s / 4)) .. string.rep(" ", #s % 4) end)
	if s ~= t then
		f = io.open(filename, "wb")
		f:write(t)
		f:close()
		print(" * " .. filename)
	end
end

fixSpaceCode "NavServer.h"
fixSpaceCode "NavServer.internal.h"
fixSpaceCode "NavServer.cpp"
fixSpaceCode "NavServerJNI.cpp"
fixSpaceCode "NavServerTest.cpp"
fixTabCode "RecastAPI.java"
fixTabCode "NavServerFixCode.lua"
fixTabCode "NavServerBuildLinux.sh"
