require("config")

if true then -- use luna_gen
	local test_names={ 'vector3','quater','vector3N','vector3NView','quaterN', 'quaterNView', 'vectorn','vectornView', 'matrixn', 'matrixnView','matrix3','matrix4','transf'}
	for i,v in ipairs(test_names) do
		_G[v]=test[v]
	end
	Physics.registerMainLib()
	Physics.registerPhysics()
	USE_LUNA_GEN=true
end
require("module")
M=require("scilua")

function d(fn)
	dofile(fn..'.lua')
	d=nil
end
-- usage1: l console "d'test_tt.lua'"
-----> runs test_tt.lua
-- usage2: l console
function ctor()
	if d==nil then 
		this("exit",{}) 
	else
--		dbg.console()
		this("exit",{})
	end
end

function dtor()
end

function onCallback(w, userData)
end

function frameMove(fElapsedTime)
end
