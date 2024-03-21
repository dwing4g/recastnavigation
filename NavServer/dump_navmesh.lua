-- usage:
-- luajit dump_navmesh.lua test.navmesh > test.navmesh.txt

local format = string.format
local floor = math.floor
local io = io
local write = io.write
local clock = os.clock
local arg = {...}
local _G = _G
local DT_POLYREF64 = true

local ffi = require "ffi"
ffi.cdef[[
typedef union { float f; char c[4]; } FLOAT_PACK;
]]
local fp = ffi.new "FLOAT_PACK[1]"

local timeBegin = clock()

local str, pos
local function readFloat(name)
	ffi.copy(fp, str:sub(pos + 1, pos + 4), 4)
	pos = pos + 4
	local v = fp[0].f
	if name then
		_G[name] = v
	end
	return v
end

local function readByte(name)
	local v = str:byte(pos + 1)
	pos = pos + 1
	if name then
		_G[name] = v
	end
	return v
end

local function readUShort(name)
	local v = str:byte(pos + 1) + str:byte(pos + 2) * 0x100
	pos = pos + 2
	if name then
		_G[name] = v
	end
	return v
end

local function readShort()
	local v = str:byte(pos + 1) + str:byte(pos + 2) * 0x100
	pos = pos + 2
	return v < 0x8000 and v or v - 0x10000
end

local function readInt(name)
	local v = str:byte(pos + 1) + str:byte(pos + 2) * 0x100 + str:byte(pos + 3) * 0x10000 + str:byte(pos + 4) * 0x1000000
	pos = pos + 4
	v = v < 0x80000000 and v or v - 0x100000000
	if name then
		_G[name] = v
	end
	return v
end

local function readUInt(name)
	local v = str:byte(pos + 1) + str:byte(pos + 2) * 0x100 + str:byte(pos + 3) * 0x10000 + str:byte(pos + 4) * 0x1000000
	pos = pos + 4
	if name then
		_G[name] = v
	end
	return v
end

local function readNeis()
	local v = readUShort()
	return format(v < 0x8000 and "%d" or "0x%X", v)
end

local lastPos
local function a()
	if pos ~= lastPos then
		write(format("%8X | ", pos))
		lastPos = pos
	else
		write "         | "
	end
end
local function A(a)
	write("         |", a or " ")
end

local function o(fmt, ...)
	write(format(fmt, ...), "\n")
end

local function O(fmt, ...)
	write(format(fmt, ...))
end

local function calcBits(v)
	local n = 0
	while v > 0 do
		v = floor(v / 2)
		n = n + 1
	end
	return n
end

local f = arg[1] and io.open(arg[1], "rb")
str = f:read "*a"
f:close()
pos = 0

local tileCount = 1
local tileBits = calcBits(tileCount - 1)
local tileN = 2 ^ tileBits
local polyN = 2 ^ (22 - tileBits)
local traceTiles = {}
local totalVertCount = 0
local totalPolyCount = 0
local totalLinkCount = 0
local totalDetailMeshCount = 0
local totalDetailVertCount = 0
local totalDetailTriCount = 0
local totalBvNodeCount = 0
local totalOffMeshConCount = 0
local i = 0
--	local traceTile = { id = i, polys = {}, links = {} }
--	traceTiles[i] = traceTile
a() o("    dtMeshHeader.magic          : %08X", readInt("magic"))
if _G.magic ~= 0x444E4156 then
	o "ERROR: invalid magic"
end
a() o("    dtMeshHeader.version        : %d", readInt())
a() o("    dtMeshHeader.x              : %d", readInt())
a() o("    dtMeshHeader.y              : %d", readInt())
a() o("    dtMeshHeader.layer          : %d", readInt())
a() o("    dtMeshHeader.userId         : %d", readUInt())
a() o("    dtMeshHeader.polyCount      : %d", readInt("polyCount"))
a() o("    dtMeshHeader.vertCount      : %d", readInt("vertCount"))
a() o("    dtMeshHeader.maxLinkCount   : %d", readInt("maxLinkCount"))
a() o("    dtMeshHeader.detailMeshCount: %d", readInt("detailMeshCount"))
a() o("    dtMeshHeader.detailVertCount: %d", readInt("detailVertCount"))
a() o("    dtMeshHeader.detailTriCount : %d", readInt("detailTriCount"))
a() o("    dtMeshHeader.bvNodeCount    : %d", readInt("bvNodeCount"))
a() o("    dtMeshHeader.offMeshConCount: %d", readInt("offMeshConCount"))
a() o("    dtMeshHeader.offMeshBase    : %d", readInt())
a() o("    dtMeshHeader.walkableHeight : %f", readFloat())
a() o("    dtMeshHeader.walkableRadius : %f", readFloat())
a() o("    dtMeshHeader.walkableClimb  : %f", readFloat())
a() o("    dtMeshHeader.bmin:%12f,%12f,%12f", readFloat(), readFloat(), readFloat())
a() o("    dtMeshHeader.bmax:%12f,%12f,%12f", readFloat(), readFloat(), readFloat())
a() o("    dtMeshHeader.bvQuantFactor  : %f", readFloat())
a() o("    verts: float[3(xyz) * %d] ... -- see below", _G.vertCount)
totalVertCount = totalVertCount + _G.vertCount
totalPolyCount = totalPolyCount + _G.polyCount
totalLinkCount = totalLinkCount + _G.maxLinkCount
totalDetailMeshCount = totalDetailMeshCount + _G.detailMeshCount
totalDetailVertCount = totalDetailVertCount + _G.detailVertCount
totalDetailTriCount = totalDetailTriCount + _G.detailTriCount
totalBvNodeCount = totalBvNodeCount + _G.bvNodeCount
totalOffMeshConCount = totalOffMeshConCount + _G.offMeshConCount
local vs = {}
local vertPos = pos
for j = 0, _G.vertCount - 1 do
	vs[j * 3    ] = readFloat()
	vs[j * 3 + 1] = readFloat()
	vs[j * 3 + 2] = readFloat()
end
A() o("    polys: dtPoly[%d]", _G.polyCount)
if _G.polyCount > 1024 then
	o("ERROR: too many polys (%d > 1024) in this tile (idx:%d)", _G.polyCount, i)
end
for j = 0, _G.polyCount - 1 do
	A() o("      dtPoly[%d/%d]", j, _G.polyCount)
	a() o("        dtPoly.firstLink  : %d", readInt("firstLink"))
	a() o("        dtPoly.verts      : %d, %d, %d, %d, %d, %d", readUShort("v0"), readUShort("v1"), readUShort("v2"), readUShort("v3"), readUShort("v4"), readUShort("v5"))
	a() o("        dtPoly.neis       : %s, %s, %s, %s, %s, %s", readNeis(), readNeis(), readNeis(), readNeis(), readNeis(), readNeis())
	a() o("        dtPoly.flags      : 0x%X", readUShort())
	a() o("        dtPoly.vertCount  : %d", readByte("vertCount"))
	a() o("        dtPoly.areaAndtype: 0x%X -- area:%d, type:%d", readByte("areaAndtype"), _G.areaAndtype % 64, floor(_G.areaAndtype / 64))
	local xx, zz, okX, okZ
	local cx, cy, cz = 0, 0, 0
	for k = 0, _G.vertCount - 1 do
		local idx = _G["v" .. k]
		local x, y, z = vs[idx * 3] or -99999, vs[idx * 3 + 1] or -99999, vs[idx * 3 + 2] or -99999
		A("-") o("%-8X          verts[%d]:%9.3f,%9.3f,%9.3f -- idx:%d", vertPos + idx * 3 * 4, k, x, y, z, idx)
		if k > 0 then
			if x ~= xx then okX = true end
			if z ~= zz then okZ = true end
		end
		xx, zz = x, z
		cx, cy, cz = cx + x, cy + y, cz + z
	end
	if (not okX or not okZ) and _G.vertCount >= 3 then
		o "ERROR: invalid poly"
	end
	if _G.vertCount > 0 then
		cx, cy, cz = cx / _G.vertCount, cy / _G.vertCount, cz / _G.vertCount
	end
	-- traceTile.polys[j] = { id = j, cx = cx, cy = cy, cz = cz, firstLink = _G.firstLink }
end
A() o("    links: dtLink[%d]", _G.maxLinkCount)
for j = 0, _G.maxLinkCount - 1 do
	A() o("      dtLink[%d/%d]", j, _G.maxLinkCount)
	local salt, tile, poly, tileX, tileZ
	if DT_POLYREF64 then
		a() O("        dtLink.ref : %d, %d", readUInt("ref0"), readUInt("ref1"))
		if _G.ref0 > 0 or _G.ref1 > 0 then
			salt = floor(_G.ref1 / (2^16))
			tile = _G.ref1 % (2^16) * (2^12) + floor(_G.ref0 / (2^20))
			poly = _G.ref0 % (2^20)
			-- tileX = tile % tileW
			-- tileZ = floor(tile / tileW)
			o(" -- salt:%d, tile:%d, poly:%d", salt, tile, poly)
		else
			o ""
		end
	else
		a() O("        dtLink.ref : %d", readUInt("ref"))
		if _G.ref > 0 then
			salt = floor(_G.ref / (2^22))
			tile = floor(_G.ref / polyN) % tileN
			poly = _G.ref % polyN
			-- tileX = tile % tileW
			-- tileZ = floor(tile / tileW)
			o(" -- salt:%d, tile:%d, poly:%d", salt, tile, poly)
		else
			o ""
		end
	end
	a() o("        dtLink.next: %d", readInt("next"))
	a() o("        dtLink.edge: %d", readByte())
	a() o("        dtLink.side: %d", readByte())
	a() o("        dtLink.bmin: %d", readByte())
	a() o("        dtLink.bmax: %d", readByte())
	-- traceTile.links[j] = { tile = tile, poly = poly, next = _G.next }
end
A() o("    detailMeshes: dtPolyDetail[%d]", _G.detailMeshCount)
for j = 0, _G.detailMeshCount - 1 do
	A() o("      dtPolyDetail[%d/%d]", j, _G.detailMeshCount)
	a() o("        dtPolyDetail.vertBase : %d", readUInt())
	a() o("        dtPolyDetail.triBase  : %d", readUInt())
	a() o("        dtPolyDetail.vertCount: %d", readByte())
	a() o("        dtPolyDetail.triCount : %d", readByte())
	pos = pos + 2 -- align
end
A() o("    detailVerts: float[3(xyz) * %d] ...", _G.detailVertCount) -- TODO
pos = pos + _G.detailVertCount * 3 * 4
A() o("    detailTris: byte[4 * %d] ...", _G.detailTriCount) -- TODO
pos = pos + _G.detailTriCount * 4
A() o("    bvTree: dtBVNode[%d]", _G.bvNodeCount)
for j = 0, _G.bvNodeCount - 1 do
	A() o("      dtBVNode[%d/%d]", j, _G.bvNodeCount)
	a() o("        dtBVNode.bmin: %d, %d, %d", readUShort(), readUShort(), readUShort())
	a() o("        dtBVNode.bmax: %d, %d, %d", readUShort(), readUShort(), readUShort())
	a() o("        dtBVNode.i   : %d", readInt())
end
A() o("    offMeshCons: dtOffMeshConnection[%d]", _G.offMeshConCount)
for j = 0, _G.offMeshConCount - 1 do
	A() o("      dtOffMeshConnection[%d/%d]", j, _G.offMeshConCount)
	a() o("        dtOffMeshConnection.pos   :%9.3f,%9.3f,%9.3f => %9.3f,%9.3f,%9.3f", readFloat(), readFloat(), readFloat(), readFloat(), readFloat(), readFloat())
	a() o("        dtOffMeshConnection.rad   : %f", readFloat())
	a() o("        dtOffMeshConnection.poly  : %d", readUShort())
	a() o("        dtOffMeshConnection.flags : 0x%X", readByte())
	a() o("        dtOffMeshConnection.side  : %d", readByte())
	a() o("        dtOffMeshConnection.userId: %d", readUInt())
end
if pos ~= #str then
	o("ERROR: unmatched tile data size: %d != %d", pos, #str)
end
--[[
o("------- TRACE -------")
local groupCount, polyCount = 0, 0
local function markGroup(tile, poly)
	if poly.group then return end
	poly.group = groupCount
	polyCount = polyCount + 1
	local link = poly.firstLink
	while link >= 0 do
		link = tile.links[link]
		local newTile = traceTiles[link.tile]
		local newPoly = newTile.polys[link.poly]
		if newPoly then
			markGroup(newTile, newPoly)
		else
			o("ERROR: not found poly: tile=%d, poly=%d -> tile=%d poly=%d", tile.id, poly.id, link.tile, link.poly)
		end
		link = link.next
	end
end
for i = 0, tileCount - 1 do
	local tile = traceTiles[i]
	for j = 0, 1e9 do
		local poly = tile.polys[j]
		if not poly then break end
		if not poly.group then -- new group
			groupCount, polyCount = groupCount + 1, 0
			markGroup(tile, poly)
			o("group %-3d: %5d polys: %9.3f,%9.3f,%9.3f", groupCount, polyCount, poly.cx, poly.cy, poly.cz)
		end
	end
end
--]]
o("INFO: totalVertCount       =%7d * 12 =%9d", totalVertCount,       totalVertCount       * 12)
o("INFO: totalPolyCount       =%7d * 32 =%9d", totalPolyCount,       totalPolyCount       * 32)
o("INFO: totalLinkCount       =%7d * 12 =%9d", totalLinkCount,       totalLinkCount       * 12)
o("INFO: totalDetailMeshCount =%7d * 12 =%9d", totalDetailMeshCount, totalDetailMeshCount * 12)
o("INFO: totalDetailVertCount =%7d * 12 =%9d", totalDetailVertCount, totalDetailVertCount * 12)
o("INFO: totalDetailTriCount  =%7d *  4 =%9d", totalDetailTriCount,  totalDetailTriCount  *  4)
o("INFO: totalBvNodeCount     =%7d * 16 =%9d", totalBvNodeCount,     totalBvNodeCount     * 16)
o("INFO: totalOffMeshConCount =%7d * 36 =%9d", totalOffMeshConCount, totalOffMeshConCount * 36)
o("-------- EOF -------- (%f sec)", clock() - timeBegin)
