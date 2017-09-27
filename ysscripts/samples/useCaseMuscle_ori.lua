require("useCaseMuscle_gait1956_6step")

package.path=package.path..";../Samples/ysscripts/lib/?.lua" --;"..package.path
require("utilfunc")

useCases.gait_muscle=deepCopyTable(useCases.gait1956_6steps)

-- x, y, z or true 
-- if true, mirror all values
useCases.gait_muscle.mirrorAxis = {}
useCases.gait_muscle.mirrorAxis["ang"]={"y", "z"}
useCases.gait_muscle.mirrorAxis["pos"]={"x"}

useCases.gait_muscle.optParamNames = {"MAPswingFootMod","MAPspprtFootMod"}
useCases.gait_muscle.parts = {"Foot"}
useCases.gait_muscle.partProps = {}
useCases.gait_muscle.partProps["Foot"] = "pos"

	--optParam = makeOptParam(3,"MAPswingFootMod,"..grpName,"MAPspprtFootMod,"..grpName)
	--optParam = makeOptParam(3,"MAPswingHipMod,"..grpName,"MAPspprtHipMod,"..grpName, "MAP1swingKneeMod,"..grpName, "MAP1spprtKneeMod,"..grpName, "MAPswingAnkleMod,"..grpName, "MAPspprtAnkleMod,"..grpName, "MAP1swingMtpMod,"..grpName,"MAP1spprtMtpMod,"..grpName)
	--optParam = makeOptParam(3,"MAPswingFootMod,"..grpName,"MAPspprtFootMod,"..grpName, "MAPswingFootangleMod,"..grpName, "MAPspprtFootangleMod,"..grpName)


function useCases.gait_muscle:makeOptParam(numKey,...)

	local gtarget=array:new()
	local prec=self.prec
	for iid,id in ipairs({...}) do
		for i=0, numKey-1 do
			--dcjo
			if string.sub(id,1,4)=="MAP1" then
				gtarget:pushBack({"map,"..i..","..string.sub(id,5),prec})
			elseif string.sub(id,1,3)=="MAP" then
				gtarget:pushBack({"map,"..i..","..string.sub(id,4)..",x",prec})
				gtarget:pushBack({"map,"..i..","..string.sub(id,4)..",y",prec})
				gtarget:pushBack({"map,"..i..","..string.sub(id,4)..",z",prec})
			else
				gtarget:pushBack({"keyframe,"..i..","..id..",x",prec})
				gtarget:pushBack({"keyframe,"..i..","..id..",y",prec})
				gtarget:pushBack({"keyframe,"..i..","..id..",z",prec})
			end
		end
	end
	return gtarget

end

function useCases.gait_muscle:makeOptParamAxis(numKey,axis,...)
	-- todo 
--[[
	local gtarget=array:new()
	--local prec=0.05
	local prec=self.prec
	local symmetricMapUsed=false
	for iid,id in ipairs({...}) do
		for i=0, numKey-1 do
			if string.sub(id,1,3)=="MAP" then
				gtarget:pushBack({"map,"..i..","..string.sub(id,4)..","..axis,prec})
				--symmetricMapUsed=true
			else
				gtarget:pushBack({"keyframe,"..i..","..id..","..axis,prec})
			end
		end
	end

	return gtarget
	]]
end

function useCases.gait_muscle:updateStageParam(stage, stageParam)
	
	gains={}
	local endSegW=self.endSegW
	local grpName=self.grpName
	local genOpt=function(starts, ends, ps, pe, name, axis, others)
		local out=useCases.genOpt(useCase, starts, ends, ps, pe, name, axis,others, grpName, path)
		assert(out)
		return out
	end
--	local tgt_all={'footLmod','footRmod'}
--	local tgt_L={'footLmod', }
--	local tgt_R={'footRmod', }

	local optParamFullNames = {}
	for i, pName in ipairs(self.optParamNames) do
		optParamFullNames[i] = pName..","..grpName
	end

	local optParam = self:makeOptParam(3, table.unpack(optParamFullNames))

	do
		-- dcjo
		-- optimize walk cycles (assuming symmetry)
		--local endSegW=stage+23
		local mod=math.mod(stage-1,4)
		if mod==0 then
			stageParam[stage]={startSeg=1, endSeg=endSegW, nvar=2,setStageFunc="setStage_param", optParam, objCost=objCost,baseStage=1}
			--stageParam[stage]={startSeg=1, endSeg=endSegW, nvar=2,setStageFunc="setStage_param", param=self:makeOptParam(3,"MAPswingFootMod,"..grpName,"MAPspprtFootMod,"..grpName), objCost=objCost,baseStage=1}
			--stageParam[stage]={startSeg=1, endSeg=endSegW, nvar=2,setStageFunc="setStage_param", param=self:makeOptParam(3,"MAPswingHipMod,"..grpName,"MAPspprtHipMod,"..grpName, "MAP1swingKneeMod,"..grpName, "MAP1spprtKneeMod,"..grpName, "MAPswingAnkleMod,"..grpName, "MAPspprtAnkleMod,"..grpName, "MAP1swingMtpMod,"..grpName,"MAP1spprtMtpMod,"..grpName), objCost=objCost,baseStage=1}
			--stageParam[stage]={startSeg=1, endSeg=endSegW, nvar=2,setStageFunc="setStage_param", param=self:makeOptParam(3,"MAPswingFootMod,"..grpName,"MAPspprtFootMod,"..grpName, "MAPswingFootangleMod,"..grpName, "MAPspprtFootangleMod,"..grpName), objCost=objCost,baseStage=1}
		elseif mod==1 then
			stageParam[stage]={startSeg=1, endSeg=endSegW, nvar=2,setStageFunc="setStage_param", param=self:makeOptParamAxis(3,'z',"MAPswingFootMod,"..grpName,"MAPspprtFootMod,"..grpName), objCost=objCost,baseStage=1}
		elseif mod==2 then
			stageParam[stage]={startSeg=1, endSeg=endSegW, nvar=2,setStageFunc="setStage_param", param=self:makeOptParamAxis(3,'x',"MAPswingFootMod,"..grpName,"MAPspprtFootMod,"..grpName), objCost=objCost,baseStage=1}
		elseif mod==3 then
			stageParam[stage]={startSeg=1, endSeg=endSegW, nvar=2,setStageFunc="setStage_param", param=self:makeOptParamAxis(3,'y',"MAPswingFootMod,"..grpName,"MAPspprtFootMod,"..grpName), objCost=objCost,baseStage=1}
		end
		--dbg.console()
	end
	--stageParam[stage+1]={baseStage=1, startSeg=1}
end

function useCases.gait_muscle:mapControlParam(title, param)

	local out={}

	assert(string.sub(title, 1,4)=="map,")
	local tokens=string.tokenize(title,',')
	local idx=tonumber(tokens[2])
	local name=tokens[3]

	local convertMap=function(name, mirror)
		-- dcjo
		local sIdx, eIdx, sTempIdx, eTempIdx;
		local parts = self.parts
		--local parts={"Foot"}
		--local parts={"Footangle", "Foot"}
		--local parts={"Hip", "Knee", "Ankle", "Mtp"}
		local key, id, part
		for i,v in ipairs(parts) do
			sTempIdx, eTempIdx = string.find(name, v)
			if sTempIdx~=nil then
				sIdx = sTempIdx
				eIdx = eTempIdx
				part = v
				break
			end
		end
		assert(sIdx)
		key=string.sub(name, 1, sIdx-1)
		id=string.lower(string.sub(name, eIdx+1))
--		id=string.lower(string.sub(name, Idx[2]+1, Idx[2]+1))..string.sub(name, Idx[2]+2)
--		local FootIdx=string.find(name, "Foot")
--		local key=string.sub(name, 1, FootIdx+3)
--		local id=string.lower(string.sub(name, FootIdx+4, FootIdx+4))..string.sub(name, FootIdx+5)
		local convertMap

		local firstFoot = useCases[tokens[4]].segNames[1]

		if mirror then
			if firstFoot == 'l' then
				convertMap=
				{
					swing={'R', ',L'}, 
					spprt={'L', ',L'},
				}
			else
				convertMap=
				{
					swing={'L', ',R'}, 
					spprt={'R', ',R'},
				}
			end
		else
			if firstFoot == 'l' then
				convertMap=
				{
					swing={'L', ',R1'}, 
					spprt={'R', ',R1'},
				}
			else
				convertMap=
				{
					swing={'R', ',L1'}, 
					spprt={'L', ',L1'},
				}
			end
		end
		return {","..string.lower(part)..convertMap[key][1]..id..",", convertMap[key][2],}, part
	end

	local grp=tokens[4]
	local axis=tokens[5]
	local cmap, part =convertMap(name)
	local cmapm=convertMap(name,true)
	local name2='keyframe,'..idx..cmap[1]..grp..cmap[2]
	local name2_mirror='keyframe,'..idx..cmapm[1]..grp..cmapm[2]

--	print(title)
--	print(name2)
--	print(name2_mirror)
	
	local mirAxis = self.mirrorAxis[self.partProps[part]]

	if axis~= nil then
		
		array.pushBack(out, {name2..","..axis,param}) 

		if table.member(axis, mirAxis) then
			array.pushBack(out, {name2_mirror..","..axis,param*-1}) 
		else
			array.pushBack(out, {name2_mirror..","..axis,param}) 
		end

	else
		array.pushBack(out, {name2, param})
		
		if mirAxis then
			array.pushBack(out, {name2_mirror, param*-1})
		else
			array.pushBack(out, {name2_mirror, param})
		end

	end

	--[[
	if axis~=nil then
		array.pushBack(out, {name2..","..axis,param}) 

		if axis=='y' or axis=='z' then
			array.pushBack(out, {name2_mirror..","..axis,param*-1}) 
		else
			array.pushBack(out, {name2_mirror..","..axis,param}) 
		end
	else
		array.pushBack(out, {name2, param})
		array.pushBack(out, {name2_mirror, param})
--		local param2=param:copy()
--		array.pushBack(out, {name2, param2})
--		param2.x=param2.x*-1
--		array.pushBack(out, {name2_mirror, param2})
	end
]]
	for i,v in ipairs(out) do
		print(v[1])
		print(v[2])
	end

	return out

end

do
	local function accumulate(cp_mod)
		local useCase=useCases.gait_muscle
		useCases.accumulate(useCase, cp_mod)
	end

	useCases.gait_muscle.noCOMjoint=true
	useCases.gait_muscle.spprtImpFromImp=true
	function modifySwingFoot(key, id, mod,inc,postfix)
		postfix=postfix or 'MocapMod'
		local useCase=useCases.walk3
		local c='map,'..key..',swingFoot'..postfix..','..id
		if inc then
			useCase.controlParam[c]=useCase.controlParam[c]+mod
		else
			useCase.controlParam[c]=mod
		end
	end

	--useCases.gait_muscle.useBulletColdet=true
	useCases.gait_muscle.useBulletColdet=false

	accumulate({
		--['keyframe,0,footRmod,gait_muscle,l,y']  = 0.,
		--['keyframe,1,footRmod,gait_muscle,l,y']  = 0.05,
		['keyframe,0,footLmod,gait_muscle,r,y']  = 0.0,
		['keyframe,1,footLmod,gait_muscle,r,y']  = 0.2,
		['keyframe,0,footRmod,gait_muscle,L1,y'] = 0.0,
		['keyframe,1,footRmod,gait_muscle,L1,y'] = 0.2,
		['map,0,swingFootMod,gait_muscle,y']     = 0.1,
		['map,1,swingFootMod,gait_muscle,y']     = 0.2,
		['map,2,swingFootMod,gait_muscle,y']     = 0.1,

		--['keyframe,0,footLmod,gait_muscle,l,y']  = 0.,
		--['keyframe,1,footLmod,gait_muscle,l,y']  = 0.,
		['keyframe,0,footRmod,gait_muscle,r,y']  = 0.,
		['keyframe,1,footRmod,gait_muscle,r,y']  = -.1,
		['keyframe,0,footLmod,gait_muscle,L1,y'] = 0.,
		['keyframe,1,footLmod,gait_muscle,L1,y'] = -.1,
		['map,0,spprtFootMod,gait_muscle,y']     = 0.,
		['map,1,spprtFootMod,gait_muscle,y']     = 0.,
		['map,2,spprtFootMod,gait_muscle,y']     = 0.,
	})
	
	useCases.unmapControlParam(useCases.gait_muscle)

end


