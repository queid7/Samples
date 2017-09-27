require("config")
require("module")

package.path=package.path..";../Samples/ysscripts/lib/?.lua" --;"..package.path
require("OsModel")

--wrlpath=package.resourcePath.."opensim/gait2592_modified.wrl"
--luamsclpath=package.resourcePath.."opensim/gait2592_modified.luamscl"
--indof="../Resource/motion/opensim/g25_cmurun_repeat_bvh.dof"
--outdof="../Resource/motion/opensim/g25_cmurun_repeat.dof"

--local lq=quater(math.rad(25), vector3(0,0,1))
--local lq=lq*quater(math.rad(15), vector3(1,0,0))
--local rq=quater(math.rad(15), vector3(0,0,1))
--local rq=rq*quater(math.rad(-30), vector3(1,0,0))
	

wrlpath=package.resourcePath.."opensim/FullBody2_lee.wrl"
luamsclpath=package.resourcePath.."opensim/FullBody2_lee.luamscl"
indof="../Resource/motion/opensim/full2_cmurun_repeat_bvh.dof"
outdof="../Resource/motion/opensim/full2_cmurun_repeat.dof"

local lq=quater(math.rad(15), vector3(0,0,1))
local lq=lq*quater(math.rad(20), vector3(1,0,0))
local rq=quater(math.rad(10), vector3(0,0,1))
local rq=rq*quater(math.rad(-20), vector3(1,0,0))
local lhq=quater(math.rad(90), vector3(1,0,0))
local rhq=quater(math.rad(-90), vector3(1,0,0))

function ctor()
	local mOsim=OsModel(wrlpath, luamsclpath)
	mOsim:loadDOF(indof)
	local loader=mOsim.mLoader

	local mot = mMotionDOFcontainer.mot
	for i=0, mot:numFrames()-1 do
		loader:setPoseDOF(mot:row(i))
		loader:getBoneByName('ankle_l'):getFrame().rotation:leftMult(lq)
		loader:getBoneByName('mtp_l'):getFrame().rotation:leftMult(lq)
		loader:getBoneByName('ankle_r'):getFrame().rotation:leftMult(rq)
		loader:getBoneByName('mtp_r'):getFrame().rotation:leftMult(rq)
		if lhq~=nil then
			loader:getBoneByName('radius_hand_l'):getFrame().rotation:rightMult(lhq)
			loader:getBoneByName('radius_hand_r'):getFrame().rotation:rightMult(rhq)
		end
		loader:getPoseDOF(mot:row(i))
	end

	mMotionDOFcontainer:exportMot(outdof)

	mOsim:loadDOF(outdof)
end

function dtor()
end

function frameMove(fElapsedTime)
end

function onCallback(w, userData)
end
