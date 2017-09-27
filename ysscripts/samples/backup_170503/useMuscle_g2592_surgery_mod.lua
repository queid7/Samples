package.path=package.path..";../Samples/ysscripts/lib/?.lua" --;"..package.path
require("utilfunc")

package.path=package.path..";../Samples/ysscripts/samples/?.lua" --;"..package.path
require("useCaseMuscle_gait1956_6steps")


useCases.g2592_surgery_mod=deepCopyTable(useCases.gait1956_6steps)

useCases.g2592_surgery_mod.optimizerMethod='Optimizer.methods.CMAes_ys'

-- dcjo-surgery
useCases.g2592_surgery_mod.useAngleOffset = true
useCases.g2592_surgery_mod.maxAngleOffset = {hipRmax = vector3(20,20,20), hipLmax = vector3(20,20,20), kneeRmax = 30, kneeLmax = 30, ankleRmax = vector3(80, 30, 30), ankleLmax = vector3(80,30,30), mtpRmax = 70, mtpLmax = 70}
--useCases.g2592_surgery_mod.useOffsetPosLimit = true

-- dcjo-surgery_modified
-- muscle property modification
-- driverDynamicMuscle line 92
useCases.g2592_surgery_mod.muscleModification = {
--[[	{'soleus_r', 'l_m_opt', -0.005},
	{'lat_gas_r', 'l_m_opt', -0.005},
	{'med_gas_r', 'l_m_opt', -0.005},
	{'soleus_l', 'l_m_opt', -0.005},
	{'lat_gas_l', 'l_m_opt', -0.005},
	{'med_gas_l', 'l_m_opt', -0.005},
	]]
--[[	{'soleus_r', 'l_t_sl', -0.01},
	{'lat_gas_r', 'l_t_sl', -0.01},
	{'med_gas_r', 'l_t_sl', -0.01},
	{'soleus_l', 'l_t_sl', -0.01},
	{'lat_gas_l', 'l_t_sl', -0.01},
	{'med_gas_l', 'l_t_sl', -0.01},
]]

}

--useCases.g2592_surgery_mod.useAnkleangleCorrection = true

-- dcjo modify ankle angle of ref. motion
--useCases.g2592_surgery_mod.modifyMotion={{1,5},{11,-30},{19,-10}}

----useCases.g2592_surgery_mod.endSegW=10
--useCases.g2592_surgery_mod.endSegW=15
--useCases.g2592_surgery_mod.prec=.2
----useCases.g2592_surgery_mod.endSegW=20
----useCases.g2592_surgery_mod.prec=.05

--dcjo useAngleOffset
useCases.g2592_surgery_mod.endSegW=15
useCases.g2592_surgery_mod.prec=.40
--useCases.g2592_surgery_mod.prec=0.00 -- error

useCases.g2592_surgery_mod.modelName='g2592_surgery_mod'
useCases.g2592_surgery_mod.grpName='g2592_surgery_mod'
useCases.g2592_surgery_mod.grpNames={'g2592_surgery_mod'}

-- nearest to human MET
useCases.g2592_surgery_mod.ddqObjWeight=10*(25-7)
useCases.g2592_surgery_mod.aObjWeight=5000*62
useCases.g2592_surgery_mod.lambdaObjWeight=.0001
useCases.g2592_surgery_mod.EEobjWeight=100
useCases.g2592_surgery_mod.EEobjWeightAngular=10
useCases.g2592_surgery_mod.momentumWeight=10


function useCases.g2592_surgery_mod:makeOptParam(numKey,...)

	local gtarget=array:new()
	--local prec=0.05
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

useCases.g2592_surgery_mod.mapControlParam=function(graph, title, param)
	local out={}

	assert(string.sub(title, 1,4)=="map,")
	local tokens=string.tokenize(title,',')
	local idx=tonumber(tokens[2])
	local name=tokens[3]

	local convertMap=function(name, mirror)
		-- dcjo
		local sIdx, eIdx, sTempIdx, eTempIdx;
--		local parts={"Footangle", "Foot"}
		local parts={"Hip", "Knee", "Ankle", "Mtp"}
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
		return {","..string.lower(part)..convertMap[key][1]..id..",", convertMap[key][2],}
	end

	local grp=tokens[4]
	local axis=tokens[5]
	local cmap=convertMap(name)
	local cmapm=convertMap(name,true)
	local name2='keyframe,'..idx..cmap[1]..grp..cmap[2]
	local name2_mirror='keyframe,'..idx..cmapm[1]..grp..cmapm[2]

--	print(title)
--	print(name2)
--	print(name2_mirror)

	if axis~=nil then
		array.pushBack(out, {name2..","..axis,param}) 

--[[		--ys
		local mirrorAxis
		if g_nobugfix~=nil and g_nobugfix==true then
			mirrorAxis = 'x'
		else
			mirrorAxis = 'z'
		end
		--if axis=='x' then
		--if axis=='z' then	--ys
		if axis==mirrorAxis then
			array.pushBack(out, {name2_mirror..","..axis,param*-1}) 
		else
			array.pushBack(out, {name2_mirror..","..axis,param}) 
		end
]]

		-- useAngleOffset == true
		--if axis=='y' or axis=='z' then -- dcjo. surgery only angle correction
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

	return out
end

function useCases.g2592_surgery_mod:updateStageParam(stage, stageParam)
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
	
	do
		-- dcjo
		-- optimize walk cycles (assuming symmetry)
		--local endSegW=stage+23
		local mod=math.mod(stage-1,4)
		if mod==0 then
			stageParam[stage]={startSeg=1, endSeg=endSegW, nvar=2,setStageFunc="setStage_param", param=self:makeOptParam(3,"MAPswingHipMod,"..grpName,"MAPspprtHipMod,"..grpName, "MAP1swingKneeMod,"..grpName, "MAP1spprtKneeMod,"..grpName, "MAPswingAnkleMod,"..grpName, "MAPspprtAnkleMod,"..grpName, "MAP1swingMtpMod,"..grpName,"MAP1spprtMtpMod,"..grpName), objCost=objCost,baseStage=1}
			--stageParam[stage]={startSeg=1, endSeg=endSegW, nvar=2,setStageFunc="setStage_param", param=self:makeOptParam(3,"MAPswingFootMod,"..grpName,"MAPspprtFootMod,"..grpName, "MAPswingFootangleMod,"..grpName, "MAPspprtFootangleMod,"..grpName), objCost=objCost,baseStage=1}
--[[
		elseif mod==1 then
			stageParam[stage]={startSeg=1, endSeg=endSegW, nvar=2,setStageFunc="setStage_param", param=self:makeOptParamAxis(3,'z',"MAPswingFootMod,"..grpName,"MAPspprtFootMod,"..grpName), objCost=objCost,baseStage=1}
		elseif mod==2 then
			stageParam[stage]={startSeg=1, endSeg=endSegW, nvar=2,setStageFunc="setStage_param", param=self:makeOptParamAxis(3,'x',"MAPswingFootMod,"..grpName,"MAPspprtFootMod,"..grpName), objCost=objCost,baseStage=1}
		elseif mod==3 then
			stageParam[stage]={startSeg=1, endSeg=endSegW, nvar=2,setStageFunc="setStage_param", param=self:makeOptParamAxis(3,'y',"MAPswingFootMod,"..grpName,"MAPspprtFootMod,"..grpName), objCost=objCost,baseStage=1}
		--]]
		end
		--dbg.console()
	end
	--stageParam[stage+1]={baseStage=1, startSeg=1}
end


-- the initial pendControlParam and pendOptimizationPath can be automatically generated by using createInitialPendControlParam.lua
useCases.g2592_surgery_mod.pendControlParam=
{
	['keyframe,0,pendDesiredVel,g2592_surgery_mod,l,z']=0,['keyframe,0,pendDesiredVel,g2592_surgery_mod,l,x']=1,['keyframe,0,pendDesiredVel,g2592_surgery_mod,r,z']=0,['keyframe,0,pendDesiredVel,g2592_surgery_mod,r,x']=1,['keyframe,0,pendDesiredVel,g2592_surgery_mod,L1,z']=0,['keyframe,0,pendDesiredVel,g2592_surgery_mod,L1,x']=1,['keyframe,0,pendDesiredVel,g2592_surgery_mod,R1,z']=0,['keyframe,0,pendDesiredVel,g2592_surgery_mod,R1,x']=1,['keyframe,0,pendDesiredVel,g2592_surgery_mod,L2,z']=0,['keyframe,0,pendDesiredVel,g2592_surgery_mod,L2,x']=1,['keyframe,0,pendDesiredVel,ignore,0,z']=0,['keyframe,0,pendDesiredVel,ignore,0,x']=1,['keyframe,0,pendDesiredVel,ignore,1,z']=0,['keyframe,0,pendDesiredVel,ignore,1,x']=1,
	['keyframe,0,pendDesiredVel,ignore,0,x']=0.55721712382895, ['keyframe,0,pendDesiredVel,ignore,0,z']=-0.11549022684414, ['keyframe,0,pendDesiredVel,g2592_surgery_mod,l,x']=0.83765088460274, ['keyframe,0,pendDesiredVel,g2592_surgery_mod,l,z']=-0.03971153581855, ['keyframe,0,pendDesiredVel,g2592_surgery_mod,r,x']=1.0292990315838, ['keyframe,0,pendDesiredVel,g2592_surgery_mod,r,z']=0.012304645261958, ['keyframe,0,pendDesiredVel,g2592_surgery_mod,L1,x']=1.0830140604807, ['keyframe,0,pendDesiredVel,g2592_surgery_mod,L1,z']=0.021311276252793, ['keyframe,0,pendDesiredVel,g2592_surgery_mod,R1,x']=1.0448069373172, ['keyframe,0,pendDesiredVel,g2592_surgery_mod,R1,z']=0.015212087553609, ['keyframe,0,pendDesiredVel,g2592_surgery_mod,L2,x']=0.99864134148705, ['keyframe,0,pendDesiredVel,g2592_surgery_mod,L2,z']=0.011195374086852, ['keyframe,0,pendDesiredVel,ignore,1,x']=0.9726342007727, ['keyframe,0,pendDesiredVel,ignore,1,z']=0.0046873881077062, 
	['keyframe,0,pendDesiredVel,ignore,0,x']=-0.038573565913782, ['keyframe,0,pendDesiredVel,ignore,0,z']=-0.42183387297139, ['keyframe,0,pendDesiredVel,g2592_surgery_mod,l,x']=2.8733663187598, ['keyframe,0,pendDesiredVel,g2592_surgery_mod,l,z']=0.77374805250546, 
	['keyframe,0,pendDesiredVel,ignore,0,x']=-0.03848565284349, ['keyframe,0,pendDesiredVel,ignore,0,z']=-0.42174541529281, ['keyframe,0,pendDesiredVel,g2592_surgery_mod,l,x']=2.8733683928023, ['keyframe,0,pendDesiredVel,g2592_surgery_mod,l,z']=0.77375288187355, 
	['keyframe,0,pendDesiredVel,g2592_surgery_mod,l,x']=1.8518302499598, ['keyframe,0,pendDesiredVel,g2592_surgery_mod,l,z']=0.32721002628792, ['keyframe,0,pendDesiredVel,g2592_surgery_mod,r,x']=0.64195188458919, ['keyframe,0,pendDesiredVel,g2592_surgery_mod,r,z']=-0.23746102054191, ['keyframe,0,pendDesiredVel,g2592_surgery_mod,L1,x']=1.0569602524197, ['keyframe,0,pendDesiredVel,g2592_surgery_mod,L1,z']=0.0089665722850331, 
	['keyframe,0,pendDesiredVel,g2592_surgery_mod,r,x']=0.65321586961613, ['keyframe,0,pendDesiredVel,g2592_surgery_mod,r,z']=-0.23322355700538, ['keyframe,0,pendDesiredVel,g2592_surgery_mod,L1,x']=1.1600582644838, ['keyframe,0,pendDesiredVel,g2592_surgery_mod,L1,z']=0.16409120361367, ['keyframe,0,pendDesiredVel,g2592_surgery_mod,R1,x']=1.0469024908636, ['keyframe,0,pendDesiredVel,g2592_surgery_mod,R1,z']=0.021499874539069, 
	['keyframe,0,pendDesiredVel,ignore,0,x']=0.057317680328125, ['keyframe,0,pendDesiredVel,ignore,0,z']=-0.37352874903012, ['keyframe,0,pendDesiredVel,g2592_surgery_mod,l,x']=1.688193340133, ['keyframe,0,pendDesiredVel,g2592_surgery_mod,l,z']=0.33135464415606, ['keyframe,0,pendDesiredVel,g2592_surgery_mod,r,x']=0.56260872770468, ['keyframe,0,pendDesiredVel,g2592_surgery_mod,r,z']=-0.29475535050687, 
	['keyframe,0,pendDesiredVel,g2592_surgery_mod,l,x']=1.6732338838641, ['keyframe,0,pendDesiredVel,g2592_surgery_mod,l,z']=0.31227141866769, ['keyframe,0,pendDesiredVel,g2592_surgery_mod,r,x']=0.72633156918842, ['keyframe,0,pendDesiredVel,g2592_surgery_mod,r,z']=-0.30056568375898, ['keyframe,0,pendDesiredVel,g2592_surgery_mod,L1,x']=1.1885783553992, ['keyframe,0,pendDesiredVel,g2592_surgery_mod,L1,z']=0.18162859438481, 
	['keyframe,0,pendDesiredVel,g2592_surgery_mod,r,x']=0.72682249891879, ['keyframe,0,pendDesiredVel,g2592_surgery_mod,r,z']=-0.27195354224673, ['keyframe,0,pendDesiredVel,g2592_surgery_mod,L1,x']=1.1844005645379, ['keyframe,0,pendDesiredVel,g2592_surgery_mod,L1,z']=0.21070016989902, ['keyframe,0,pendDesiredVel,g2592_surgery_mod,R1,x']=1.0436382930349, ['keyframe,0,pendDesiredVel,g2592_surgery_mod,R1,z']=0.019573432369505, 
	['keyframe,0,pendDesiredVel,g2592_surgery_mod,L1,x']=1.1546812185886, ['keyframe,0,pendDesiredVel,g2592_surgery_mod,L1,z']=0.13057608466461, ['keyframe,0,pendDesiredVel,g2592_surgery_mod,R1,x']=1.0169825693409, ['keyframe,0,pendDesiredVel,g2592_surgery_mod,R1,z']=-0.031308161078641, ['keyframe,0,pendDesiredVel,g2592_surgery_mod,L2,x']=0.99332532373797, ['keyframe,0,pendDesiredVel,g2592_surgery_mod,L2,z']=0.023230828039423, ['keyframe,0,pendDesiredVel,ignore,1,x']=0.97210147412166, ['keyframe,0,pendDesiredVel,ignore,1,z']=0.0051664564521254, 
}
useCases.g2592_surgery_mod.pendOptimizationPath=
{
	firstFrames={1*2,39*2,77*2,115*2,153*2,191*2,229*2,266*2},
	segments={'ignore,0','g2592_surgery_mod,l','g2592_surgery_mod,r','g2592_surgery_mod,L1','g2592_surgery_mod,R1','g2592_surgery_mod,L2','ignore,1'},
}
--useCases.g2592_surgery_mod.measureOptCost=useCases.measureOptCost

useCases.g2592_surgery_mod.segmentations=
{
	g2592_surgery_mod={
		firstFrames={1*2,39*2,77*2,115*2,153*2,191*2,229*2,266*2},
		--        (blank)    L    R     L     R     L     (blank)
		names={             'l', 'r', 'L1', 'R1', 'L2',  },
		swingL={             0,   1,   0,    1,    0,    },
		swingR={             1,   0,   1,    0,    1,    },

		footRefL= "convertFromSwingL",
		footRefR= "convertFromSwingR",
		usePositionControl=false,
	},
}
useCases.g2592_surgery_mod.graphParam=
{
	g2592_surgery_mod={
		seg={            'l',  'r',  'L1', 'R1','L'},
		num_key={         3,    3,    3  ,  4,  4},
		key_first={       0,    0,    0  ,  0,  0},
		key_last={       'r', 'L1',  'R1', 'L', 'R1'},
	},
}
useCases.g2592_surgery_mod.graph=
{
	{
		"addInterpolatedSegment",
		grpName="g2592_surgery_mod",
		name="L",
		seg0={"g2592_surgery_mod", "L2"},
		seg1={"g2592_surgery_mod", "L1"},
		startWeight=0, endWeight=1
	},
	{"connectMulti", "g2592_surgery_mod", "l", "r", "L1", "R1", "L", "R1"},
	--{"initialSegment", "g2592_surgery_mod", "l"}
	{"initialSegment", "g2592_surgery_mod", "R1"}
	-- R1 L R1 L R1 L ...
	-- L(115*2) R1(153*2) L(191*2)
}
useCases.g2592_surgery_mod.segNames=
{
	"l", "r", "L1", 'R1','L',
}

-- dcjo
useCases.g2592_surgery_mod.keyframes={
		pendDesiredVel={numKey=1, default=vector3(1,0,0)},
		swingFootForce={numKey=3, default=vector3(0,10,0)},
		hipLmod={numKey=3, numKeyFrom='key', default=vector3(0,0,0), valueType='vector'},
		hipRmod={numKey=3, numKeyFrom='key', default=vector3(0,0,0)},
		kneeLmod={numKey=3, numKeyFrom='key', default=0, valueType='scalar'},
		kneeRmod={numKey=3, numKeyFrom='key', default=0, valueType='scalar'},
		ankleLmod={numKey=3, numKeyFrom='key', default=vector3(0,0,0)},
		ankleRmod={numKey=3, numKeyFrom='key', default=vector3(0,0,0)},
		mtpLmod={numKey=3, numKeyFrom='key', default=0, valueType='scalar'},
		mtpRmod={numKey=3, numKeyFrom='key', default=0, valueType='scalar'},

--		footLmod={numKey=3, numKeyFrom='key', default=vector3(0,0,0)},
--		footRmod={numKey=3, numKeyFrom='key', default=vector3(0,0,0)},
--		footangleLmod={numKey=3, numKeyFrom='key', default=vector3(0,0,0)},
--		footangleRmod={numKey=3, numKeyFrom='key', default=vector3(0,0,0)},
		footLmocapMod={numKey=3, numKeyFrom='key', default=vector3(0,0,0)},
		footRmocapMod={numKey=3, numKeyFrom='key', default=vector3(0,0,0)},
		desiredHeadAcc={numKey=3, numKeyFrom='key', default=vector3(0,0,0)},
		desiredMomentum={numKey=3, numKeyFrom='key', default=vector3(0,0,0)},
		desiredDotMomentum={numKey=3, numKeyFrom='key', isContinuous=false,default=vector3(0,0,0)},
		desiredLeaning={numKey=1, isContinuous=false,default=vector3(0,0,0)},
		spprtImpL={numKey=2, default=1},
		spprtImpR={numKey=2, default=1},
		mocapCompensationCoef={numKey=1, default=vector3(1, 1.0, 1)},
}

do 
	useCases.g2592_surgery_mod.controlParam={
		['useCases,g2592_surgery_mod,COMobjWeight']=0, 
		['useCases,g2592_surgery_mod,conservativeW']=1, 
		--['useCases,g2592_surgery_mod,contactMargin']=0.01, 
		['useCases,g2592_surgery_mod,dotMomentumScale']=0.3, 
		['useCases,g2592_surgery_mod,excludeRoot']=true, 
		['useCases,g2592_surgery_mod,headControlWeight']=0, 
		['useCases,g2592_surgery_mod,k_d_HEAD']=14, 
		['useCases,g2592_surgery_mod,k_p_HEAD']=0, 
		['useCases,g2592_surgery_mod,k_d_EE']=24, 
		['useCases,g2592_surgery_mod,k_d_momentum']=10, 
		['useCases,g2592_surgery_mod,k_p_EE']=120, 

		--['useCases,g2592_surgery_mod,maxPenetratingVel']=0, 
		['useCases,g2592_surgery_mod,momentumThr']=50, 
		['useCases,g2592_surgery_mod,noComvelDependentFootAdjustment']=true, 
		['useCases,g2592_surgery_mod,noIntersectionPrevenction']=true, 
		['useCases,g2592_surgery_mod,numericalDerivDmot']=true, 
		['useCases,g2592_surgery_mod,perClassContactMargin']=1, 
		['useCases,g2592_surgery_mod,turnGain']=10, 
		['useCases,g2592_surgery_mod,velMarginOffset']=0, 

		['useCases,g2592_surgery_mod,actuationType']      = useCases.g2592_surgery_mod.actuationType,
		['useCases,g2592_surgery_mod,k_p_ID']             = useCases.g2592_surgery_mod.k_p_ID,
		['useCases,g2592_surgery_mod,k_d_ID']             = useCases.g2592_surgery_mod.k_d_ID,
		['useCases,g2592_surgery_mod,ddqObjWeight']       = useCases.g2592_surgery_mod.ddqObjWeight,
		['useCases,g2592_surgery_mod,tauObjWeight']       = useCases.g2592_surgery_mod.tauObjWeight,
		['useCases,g2592_surgery_mod,ftObjWeight']        = useCases.g2592_surgery_mod.ftObjWeight,
		['useCases,g2592_surgery_mod,aObjWeight']         = useCases.g2592_surgery_mod.aObjWeight,
		['useCases,g2592_surgery_mod,lambdaObjWeight']    = useCases.g2592_surgery_mod.lambdaObjWeight,
		['useCases,g2592_surgery_mod,EEobjWeight']        = useCases.g2592_surgery_mod.EEobjWeight,
		['useCases,g2592_surgery_mod,EEobjWeightAngular'] = useCases.g2592_surgery_mod.EEobjWeightAngular,
		['useCases,g2592_surgery_mod,momentumWeight']     = useCases.g2592_surgery_mod.momentumWeight,
		['useCases,g2592_surgery_mod,tauMax']             = useCases.g2592_surgery_mod.tauMax,
		['useCases,g2592_surgery_mod,ftMax']              = useCases.g2592_surgery_mod.ftMax,
	}

	-- update controlParam 
	local function accumulate(cp_mod)
		local useCase=useCases.g2592_surgery_mod
		print("accumulate in surgery")
		useCases.accumulate(useCase, cp_mod)
	end

	accumulate({
		----['keyframe,0,footRmod,g2592_surgery_mod,l,y']  = 0.,
		--['keyframe,1,footRmod,g2592_surgery_mod,l,y']  = 0.05,
		--['keyframe,0,footLmod,g2592_surgery_mod,r,y']  = 0.05,
		--['keyframe,1,footLmod,g2592_surgery_mod,r,y']  = 0.1,
		--['keyframe,0,footRmod,g2592_surgery_mod,L1,y'] = 0.0,
		--['keyframe,1,footRmod,g2592_surgery_mod,L1,y'] = 0.1,
		--['map,0,swingFootMod,g2592_surgery_mod,y']     = 0.1,
		--['map,1,swingFootMod,g2592_surgery_mod,y']     = 0.3,
		--['map,2,swingFootMod,g2592_surgery_mod,y']     = 0.2,

		------['keyframe,0,footLmod,g2592_surgery_mod,l,y']  = 0.,
		------['keyframe,1,footLmod,g2592_surgery_mod,l,y']  = 0.,
		----['keyframe,0,footRmod,g2592_surgery_mod,r,y']  = 0.,
		----['keyframe,1,footRmod,g2592_surgery_mod,r,y']  = -.1,
		----['keyframe,0,footLmod,g2592_surgery_mod,L1,y'] = 0.,
		----['keyframe,1,footLmod,g2592_surgery_mod,L1,y'] = -.1,
		----['map,0,spprtFootMod,g2592_surgery_mod,y']     = 0.,
		----['map,1,spprtFootMod,g2592_surgery_mod,y']     = 0.,
		----['map,2,spprtFootMod,g2592_surgery_mod,y']     = 0.,
	})
	useCases.unmapControlParam(useCases.g2592_surgery_mod)


	-------------------------------------------------
	local R_hip_flex={'add_brev_r', 'add_long_r', 'glut_med1_r', 'glut_min1_r', 'grac_r', 'iliacus_r', 'pect_r', 'psoas_r', 'rect_fem_r', 'sar_r', 'tfl_r'}
	local R_hip_ext={'add_long_r', 'add_mag1_r', 'add_mag2_r', 'add_mag3_r', 'bifemlh_r', 'glut_max1_r', 'glut_max2_r', 'glut_max3_r', 'glut_med3_r', 'glut_min3_r', 'semimem_r', 'semiten_r'}
	local R_hip_abd={'glut_max1_r', 'glut_med1_r', 'glut_med2_r', 'glut_med3_r', 'glut_min1_r', 'glut_min2_r', 'glut_min3_r', 'peri_r', 'sar_r', 'tfl_r'}
	local R_hip_add={'add_brev_r', 'add_long_r', 'add_mag1_r', 'add_mag2_r', 'add_mag3_r', 'bifemlh_r', 'grac_r', 'pect_r', 'semimem_r', 'semiten_r'}
	local R_knee_bend={'bifemlh_r', 'bifemsh_r', 'grac_r', 'lat_gas_r', 'med_gas_r', 'sar_r', 'semimem_r', 'semiten_r'}
	local R_knee_ext={'rect_fem_r', 'vas_int_r', 'vas_lat_r', 'vas_med_r'}
	local R_ankle_pf={'flex_dig_r', 'flex_hal_r', 'lat_gas_r', 'med_gas_r', 'per_brev_r', 'per_long_r', 'soleus_r', 'tib_post_r'}
	local R_ankle_df={'ext_dig_r', 'ext_hal_r', 'per_tert_r', 'tib_ant_r'}

	local L_hip_flex={'add_brev_l', 'add_long_l', 'glut_med1_l', 'glut_min1_l', 'grac_l', 'iliacus_l', 'pect_l', 'psoas_l', 'rect_fem_l', 'sar_l', 'tfl_l'}
	local L_hip_ext={'add_long_l', 'add_mag1_l', 'add_mag2_l', 'add_mag3_l', 'bifemlh_l', 'glut_max1_l', 'glut_max2_l', 'glut_max3_l', 'glut_med3_l', 'glut_min3_l', 'semimem_l', 'semiten_l'}
	local L_hip_abd={'glut_max1_l', 'glut_med1_l', 'glut_med2_l', 'glut_med3_l', 'glut_min1_l', 'glut_min2_l', 'glut_min3_l', 'peri_l', 'sar_l', 'tfl_l'}
	local L_hip_add={'add_brev_l', 'add_long_l', 'add_mag1_l', 'add_mag2_l', 'add_mag3_l', 'bifemlh_l', 'grac_l', 'pect_l', 'semimem_l', 'semiten_l'}
	local L_knee_bend={'bifemlh_l', 'bifemsh_l', 'grac_l', 'lat_gas_l', 'med_gas_l', 'sar_l', 'semimem_l', 'semiten_l'}
	local L_knee_ext={'rect_fem_l', 'vas_int_l', 'vas_lat_l', 'vas_med_l'}
	local L_ankle_pf={'flex_dig_l', 'flex_hal_l', 'lat_gas_l', 'med_gas_l', 'per_brev_l', 'per_long_l', 'soleus_l', 'tib_post_l'}
	local L_ankle_df={'ext_dig_l', 'ext_hal_l', 'per_tert_l', 'tib_ant_l'}

	local glut_meds={'glut_med1_r', 'glut_med2_r', 'glut_med3_r', 'glut_med1_l', 'glut_med2_l', 'glut_med3_l'}
	local glut_mins={'glut_min1_r', 'glut_min2_r', 'glut_min3_r', 'glut_min1_l', 'glut_min2_l', 'glut_min3_l'}

	local lglut_meds={'glut_med1_l', 'glut_med2_l', 'glut_med3_l'}
	local lglut_mins={'glut_min1_l', 'glut_min2_l', 'glut_min3_l'}

	local hamstrings={'semiten_r','semiten_l', 'semimem_r', 'semimem_l', 'bifemlh_r', 'bifemsh_r', 'bifemlh_l', 'bifemsh_l'}
	local rect_fems={'rect_fem_l', 'rect_fem_r'}
	local psoases={'psoas_r', 'psoas_l'}

	if g_mode=='1x' then
		g_nobugfix=true
		--iter 118 bestfvever 5.8226075914069e-07
		accumulate({['map,0,swingFootMod,g2592_surgery_mod,x']= 0.0347720834374, ['map,0,swingFootMod,g2592_surgery_mod,y']= -0.30775028709688, ['map,0,swingFootMod,g2592_surgery_mod,z']= -0.015839891161517, ['map,1,swingFootMod,g2592_surgery_mod,x']= -0.037229018189522, ['map,1,swingFootMod,g2592_surgery_mod,y']= 0.068387638632828, ['map,1,swingFootMod,g2592_surgery_mod,z']= -0.062432068770353, ['map,2,swingFootMod,g2592_surgery_mod,x']= -0.0041280934069137, ['map,2,swingFootMod,g2592_surgery_mod,y']= 0.31738778084309, ['map,2,swingFootMod,g2592_surgery_mod,z']= 0.077092889872588, ['map,0,spprtFootMod,g2592_surgery_mod,x']= -0.020016014715454, ['map,0,spprtFootMod,g2592_surgery_mod,y']= -0.04006379370993, ['map,0,spprtFootMod,g2592_surgery_mod,z']= -0.10363134026533, ['map,1,spprtFootMod,g2592_surgery_mod,x']= -0.13507550600224, ['map,1,spprtFootMod,g2592_surgery_mod,y']= -0.36372277242858, ['map,1,spprtFootMod,g2592_surgery_mod,z']= 0.066700340771708, ['map,2,spprtFootMod,g2592_surgery_mod,x']= -0.053814760720267, ['map,2,spprtFootMod,g2592_surgery_mod,y']= -0.92505944566029, ['map,2,spprtFootMod,g2592_surgery_mod,z']= -0.013363893363109, })

	elseif g_mode=='test' then
		useCases.g2592_surgery_mod.aObjWeight = .1*useCases.g2592_surgery_mod.aObjWeight
--		useCases.g2592_surgery_mod.cmaEffortWeight = 1e5
-- todo
-- manually
-- 7728.7284      iter:382
accumulate({['map,0,swingHipMod,g2592_surgery_mod,x']= 0.3066528153321, ['map,0,swingHipMod,g2592_surgery_mod,y']= -0.37315447971851, ['map,0,swingHipMod,g2592_surgery_mod,z']= 0.58073067722974, ['map,1,swingHipMod,g2592_surgery_mod,x']= 0.23707434711628, ['map,1,swingHipMod,g2592_surgery_mod,y']= -0.17346231754128, ['map,1,swingHipMod,g2592_surgery_mod,z']= -0.048836904656796, ['map,2,swingHipMod,g2592_surgery_mod,x']= 0.51404094101592, ['map,2,swingHipMod,g2592_surgery_mod,y']= -0.077137041796311, ['map,2,swingHipMod,g2592_surgery_mod,z']= 0.028090643735948, ['map,0,spprtHipMod,g2592_surgery_mod,x']= -0.23984346183345, ['map,0,spprtHipMod,g2592_surgery_mod,y']= -0.054847060995987, ['map,0,spprtHipMod,g2592_surgery_mod,z']= -0.066222322661651, ['map,1,spprtHipMod,g2592_surgery_mod,x']= 0.8026581702399, ['map,1,spprtHipMod,g2592_surgery_mod,y']= -0.33794248920622, ['map,1,spprtHipMod,g2592_surgery_mod,z']= -0.47279180758331, ['map,2,spprtHipMod,g2592_surgery_mod,x']= 0.13035295229683, ['map,2,spprtHipMod,g2592_surgery_mod,y']= -0.0076917321797753, ['map,2,spprtHipMod,g2592_surgery_mod,z']= -0.8457252705659, ['map,0,swingKneeMod,g2592_surgery_mod']= 0.36145467062101, ['map,1,swingKneeMod,g2592_surgery_mod']= 0.57849194735123, ['map,2,swingKneeMod,g2592_surgery_mod']= -0.042085588525542, ['map,0,spprtKneeMod,g2592_surgery_mod']= 0.081399595424846, ['map,1,spprtKneeMod,g2592_surgery_mod']= 0.26303191058387, ['map,2,spprtKneeMod,g2592_surgery_mod']= 0.21544965326047, ['map,0,swingAnkleMod,g2592_surgery_mod,x']= 0.15956681646063, ['map,0,swingAnkleMod,g2592_surgery_mod,y']= 0.57262919784811, ['map,0,swingAnkleMod,g2592_surgery_mod,z']= 0.059594176217477, ['map,1,swingAnkleMod,g2592_surgery_mod,x']= 0.40699890612855, ['map,1,swingAnkleMod,g2592_surgery_mod,y']= -0.1146577369141, ['map,1,swingAnkleMod,g2592_surgery_mod,z']= -0.26163390468605, ['map,2,swingAnkleMod,g2592_surgery_mod,x']= -0.15880765130901, ['map,2,swingAnkleMod,g2592_surgery_mod,y']= 0.52723350876243, ['map,2,swingAnkleMod,g2592_surgery_mod,z']= 0.65517987036852, ['map,0,spprtAnkleMod,g2592_surgery_mod,x']= 0.57042962260355, ['map,0,spprtAnkleMod,g2592_surgery_mod,y']= -0.37763816073672, ['map,0,spprtAnkleMod,g2592_surgery_mod,z']= -0.078574678918425, ['map,1,spprtAnkleMod,g2592_surgery_mod,x']= 0.33348158038693, ['map,1,spprtAnkleMod,g2592_surgery_mod,y']= 0.061511208341552, ['map,1,spprtAnkleMod,g2592_surgery_mod,z']= -0.32830633970005, ['map,2,spprtAnkleMod,g2592_surgery_mod,x']= 0.25978784411883, ['map,2,spprtAnkleMod,g2592_surgery_mod,y']= 0.86764238881988, ['map,2,spprtAnkleMod,g2592_surgery_mod,z']= 0.51999970841108, ['map,0,swingMtpMod,g2592_surgery_mod']= 0.65714477176861, ['map,1,swingMtpMod,g2592_surgery_mod']= -0.42911178572968, ['map,2,swingMtpMod,g2592_surgery_mod']= 0.93742792716489, ['map,0,spprtMtpMod,g2592_surgery_mod']= -0.47864459935096, ['map,1,spprtMtpMod,g2592_surgery_mod']= 0.024453288637411, ['map,2,spprtMtpMod,g2592_surgery_mod']= 0.025459432348064, })
-- iter:151 value: 9229.2751
--accumulate({['map,0,swingHipMod,g2592_surgery_mod,x']= 0.18340600870698, ['map,0,swingHipMod,g2592_surgery_mod,y']= -0.44917797452104, ['map,0,swingHipMod,g2592_surgery_mod,z']= 0.37585413380455, ['map,1,swingHipMod,g2592_surgery_mod,x']= 0.07672749288369, ['map,1,swingHipMod,g2592_surgery_mod,y']= -0.095030686542637, ['map,1,swingHipMod,g2592_surgery_mod,z']= -0.012134412422046, ['map,2,swingHipMod,g2592_surgery_mod,x']= 0.77326563652663, ['map,2,swingHipMod,g2592_surgery_mod,y']= -0.067723119668172, ['map,2,swingHipMod,g2592_surgery_mod,z']= -0.17999666013336, ['map,0,spprtHipMod,g2592_surgery_mod,x']= -0.25916890867341, ['map,0,spprtHipMod,g2592_surgery_mod,y']= -0.070912593355417, ['map,0,spprtHipMod,g2592_surgery_mod,z']= 0.32381615961375, ['map,1,spprtHipMod,g2592_surgery_mod,x']= 0.57795393160175, ['map,1,spprtHipMod,g2592_surgery_mod,y']= -0.2285129010196, ['map,1,spprtHipMod,g2592_surgery_mod,z']= -0.68666057922069, ['map,2,spprtHipMod,g2592_surgery_mod,x']= 0.30531533498673, ['map,2,spprtHipMod,g2592_surgery_mod,y']= -0.40878576725738, ['map,2,spprtHipMod,g2592_surgery_mod,z']= -0.89646876239908, ['map,0,swingKneeMod,g2592_surgery_mod']= 0.38812852699068, ['map,1,swingKneeMod,g2592_surgery_mod']= 0.51162816700972, ['map,2,swingKneeMod,g2592_surgery_mod']= -0.2221537300804, ['map,0,spprtKneeMod,g2592_surgery_mod']= 0.17453197352893, ['map,1,spprtKneeMod,g2592_surgery_mod']= 0.27594422860556, ['map,2,spprtKneeMod,g2592_surgery_mod']= 0.41893290921386, ['map,0,swingAnkleMod,g2592_surgery_mod,x']= 0.12368728886526, ['map,0,swingAnkleMod,g2592_surgery_mod,y']= 0.5883376367555, ['map,0,swingAnkleMod,g2592_surgery_mod,z']= 0.12189195335351, ['map,1,swingAnkleMod,g2592_surgery_mod,x']= 0.42838731307192, ['map,1,swingAnkleMod,g2592_surgery_mod,y']= -0.18566958681735, ['map,1,swingAnkleMod,g2592_surgery_mod,z']= 0.0037324893945557, ['map,2,swingAnkleMod,g2592_surgery_mod,x']= -0.089462776586368, ['map,2,swingAnkleMod,g2592_surgery_mod,y']= 0.66135952330817, ['map,2,swingAnkleMod,g2592_surgery_mod,z']= 0.4815814624008, ['map,0,spprtAnkleMod,g2592_surgery_mod,x']= 0.55790463020377, ['map,0,spprtAnkleMod,g2592_surgery_mod,y']= -0.29548222674326, ['map,0,spprtAnkleMod,g2592_surgery_mod,z']= -0.23937384066656, ['map,1,spprtAnkleMod,g2592_surgery_mod,x']= 0.1318073660537, ['map,1,spprtAnkleMod,g2592_surgery_mod,y']= 0.048695480347707, ['map,1,spprtAnkleMod,g2592_surgery_mod,z']= -0.41137673850063, ['map,2,spprtAnkleMod,g2592_surgery_mod,x']= 0.23584221229205, ['map,2,spprtAnkleMod,g2592_surgery_mod,y']= 0.6873457654051, ['map,2,spprtAnkleMod,g2592_surgery_mod,z']= 0.55933740647925, ['map,0,swingMtpMod,g2592_surgery_mod']= 0.27030957799274, ['map,1,swingMtpMod,g2592_surgery_mod']= -0.38548466623407, ['map,2,swingMtpMod,g2592_surgery_mod']= 0.5990775216325, ['map,0,spprtMtpMod,g2592_surgery_mod']= -0.43615610710284, ['map,1,spprtMtpMod,g2592_surgery_mod']= -0.1053145263711, ['map,2,spprtMtpMod,g2592_surgery_mod']= 0.1391913592394, })

-- 10000??
--accumulate({['map,0,swingHipMod,g2592_surgery_mod,x']= 0.19019922467028, ['map,0,swingHipMod,g2592_surgery_mod,y']= -0.36157199681326, ['map,0,swingHipMod,g2592_surgery_mod,z']= 0.60744254272265, ['map,1,swingHipMod,g2592_surgery_mod,x']= 0.18362358621097, ['map,1,swingHipMod,g2592_surgery_mod,y']= -0.048612612202587, ['map,1,swingHipMod,g2592_surgery_mod,z']= 0.23656043932779, ['map,2,swingHipMod,g2592_surgery_mod,x']= 0.69669502387837, ['map,2,swingHipMod,g2592_surgery_mod,y']= -0.036142239494927, ['map,2,swingHipMod,g2592_surgery_mod,z']= -0.32327752410101, ['map,0,spprtHipMod,g2592_surgery_mod,x']= -0.60367025936154, ['map,0,spprtHipMod,g2592_surgery_mod,y']= -0.011800875915448, ['map,0,spprtHipMod,g2592_surgery_mod,z']= 0.19557697069319, ['map,1,spprtHipMod,g2592_surgery_mod,x']= 0.82628172666413, ['map,1,spprtHipMod,g2592_surgery_mod,y']= -0.28962904294137, ['map,1,spprtHipMod,g2592_surgery_mod,z']= -0.34844882125911, ['map,2,spprtHipMod,g2592_surgery_mod,x']= 0.58546662132511, ['map,2,spprtHipMod,g2592_surgery_mod,y']= 0.066417378251334, ['map,2,spprtHipMod,g2592_surgery_mod,z']= -0.7874407223236, ['map,0,swingKneeMod,g2592_surgery_mod']= 0.3065797460012, ['map,1,swingKneeMod,g2592_surgery_mod']= 0.47469967441132, ['map,2,swingKneeMod,g2592_surgery_mod']= -0.30431974001991, ['map,0,spprtKneeMod,g2592_surgery_mod']= 0.32935788151813, ['map,1,spprtKneeMod,g2592_surgery_mod']= 0.08373101808144, ['map,2,spprtKneeMod,g2592_surgery_mod']= 0.3292741769177, ['map,0,swingAnkleMod,g2592_surgery_mod,x']= 0.14859157100766, ['map,0,swingAnkleMod,g2592_surgery_mod,y']= 0.15595938416219, ['map,0,swingAnkleMod,g2592_surgery_mod,z']= 0.093661142392336, ['map,1,swingAnkleMod,g2592_surgery_mod,x']= 0.38256560532076, ['map,1,swingAnkleMod,g2592_surgery_mod,y']= -0.055099045406155, ['map,1,swingAnkleMod,g2592_surgery_mod,z']= 0.011373982954324, ['map,2,swingAnkleMod,g2592_surgery_mod,x']= -0.10903119322912, ['map,2,swingAnkleMod,g2592_surgery_mod,y']= 0.81809364967592, ['map,2,swingAnkleMod,g2592_surgery_mod,z']= 0.26234296388997, ['map,0,spprtAnkleMod,g2592_surgery_mod,x']= 0.63252982493217, ['map,0,spprtAnkleMod,g2592_surgery_mod,y']= -0.10417434038373, ['map,0,spprtAnkleMod,g2592_surgery_mod,z']= -0.18807152130698, ['map,1,spprtAnkleMod,g2592_surgery_mod,x']= -0.11889094473457, ['map,1,spprtAnkleMod,g2592_surgery_mod,y']= 0.2920675147826, ['map,1,spprtAnkleMod,g2592_surgery_mod,z']= -0.5335683651688, ['map,2,spprtAnkleMod,g2592_surgery_mod,x']= 0.28525104407635, ['map,2,spprtAnkleMod,g2592_surgery_mod,y']= 0.43306810947049, ['map,2,spprtAnkleMod,g2592_surgery_mod,z']= 0.55677792066701, ['map,0,swingMtpMod,g2592_surgery_mod']= 0.1176543534912, ['map,1,swingMtpMod,g2592_surgery_mod']= -0.0014826075982647, ['map,2,swingMtpMod,g2592_surgery_mod']= 0.34584382638473, ['map,0,spprtMtpMod,g2592_surgery_mod']= -0.23041174064813, ['map,1,spprtMtpMod,g2592_surgery_mod']= -0.17008790873557, ['map,2,spprtMtpMod,g2592_surgery_mod']= 0.16761177916034, })


	elseif g_mode=='test_stickleg' then
		useCases.g2592_surgery_mod.aObjWeight = .1*useCases.g2592_surgery_mod.aObjWeight
--		useCases.g2592_surgery_mod.cmaEffortWeight = 1e5
-- todo
-- manually
accumulate({['map,0,swingHipMod,g2592_surgery_mod,x']= 1.0901240077405, ['map,0,swingHipMod,g2592_surgery_mod,y']= -0.910459520938, ['map,0,swingHipMod,g2592_surgery_mod,z']= 0.42652543468564, ['map,1,swingHipMod,g2592_surgery_mod,x']= 1.2048398181976, ['map,1,swingHipMod,g2592_surgery_mod,y']= -0.80999984359897, ['map,1,swingHipMod,g2592_surgery_mod,z']= 0.27982376795835, ['map,2,swingHipMod,g2592_surgery_mod,x']= -0.0032915698940714, ['map,2,swingHipMod,g2592_surgery_mod,y']= 0.22524088482814, ['map,2,swingHipMod,g2592_surgery_mod,z']= -0.40782936959809, ['map,0,spprtHipMod,g2592_surgery_mod,x']= -0.40678818282867, ['map,0,spprtHipMod,g2592_surgery_mod,y']= -0.26862210497108, ['map,0,spprtHipMod,g2592_surgery_mod,z']= 0.13359199269604, ['map,1,spprtHipMod,g2592_surgery_mod,x']= 0.73888308064771, ['map,1,spprtHipMod,g2592_surgery_mod,y']= -0.097600537068923, ['map,1,spprtHipMod,g2592_surgery_mod,z']= -0.41752393067805, ['map,2,spprtHipMod,g2592_surgery_mod,x']= 0.1058824653307, ['map,2,spprtHipMod,g2592_surgery_mod,y']= 0.022496711134946, ['map,2,spprtHipMod,g2592_surgery_mod,z']= 0.0071852561184838, ['map,0,swingKneeMod,g2592_surgery_mod']= -0.085057283995602, ['map,1,swingKneeMod,g2592_surgery_mod']= -0.24009865519611, ['map,2,swingKneeMod,g2592_surgery_mod']= -0.71696415817297, ['map,0,spprtKneeMod,g2592_surgery_mod']= -0.2512617764245, ['map,1,spprtKneeMod,g2592_surgery_mod']= 1.0301138204523, ['map,2,spprtKneeMod,g2592_surgery_mod']= -0.092363243061526, ['map,0,swingAnkleMod,g2592_surgery_mod,x']= 0.76505007386868, ['map,0,swingAnkleMod,g2592_surgery_mod,y']= -0.56668057142796, ['map,0,swingAnkleMod,g2592_surgery_mod,z']= 0.26945005735683, ['map,1,swingAnkleMod,g2592_surgery_mod,x']= 1.4092747028017, ['map,1,swingAnkleMod,g2592_surgery_mod,y']= -0.72955723637048, ['map,1,swingAnkleMod,g2592_surgery_mod,z']= 0.17259282371968, ['map,2,swingAnkleMod,g2592_surgery_mod,x']= 0.089047243978786, ['map,2,swingAnkleMod,g2592_surgery_mod,y']= -0.49377382206773, ['map,2,swingAnkleMod,g2592_surgery_mod,z']= 0.54279419316432, ['map,0,spprtAnkleMod,g2592_surgery_mod,x']= 1.1121519910986, ['map,0,spprtAnkleMod,g2592_surgery_mod,y']= 0.27994881256947, ['map,0,spprtAnkleMod,g2592_surgery_mod,z']= -0.34541087581683, ['map,1,spprtAnkleMod,g2592_surgery_mod,x']= -0.35055619897333, ['map,1,spprtAnkleMod,g2592_surgery_mod,y']= 0.061269562037809, ['map,1,spprtAnkleMod,g2592_surgery_mod,z']= 0.011030102374788, ['map,2,spprtAnkleMod,g2592_surgery_mod,x']= -0.13902068046841, ['map,2,spprtAnkleMod,g2592_surgery_mod,y']= 0.37898945481637, ['map,2,spprtAnkleMod,g2592_surgery_mod,z']= 0.86897505355093, ['map,0,swingMtpMod,g2592_surgery_mod']= -0.15930209502852, ['map,1,swingMtpMod,g2592_surgery_mod']= 0.36369074358267, ['map,2,swingMtpMod,g2592_surgery_mod']= -0.63750407547218, ['map,0,spprtMtpMod,g2592_surgery_mod']= 0.29931830142904, ['map,1,spprtMtpMod,g2592_surgery_mod']= 0.0093167865976485, ['map,2,spprtMtpMod,g2592_surgery_mod']= 0.28901446952427, })

	else
		--elseif g_mode=='age_30' then
		--useCases.g2592_surgery_mod.age = 30
		--useCases.g2592_surgery_mod.cmaEffortWeight = 1e-6
		--useCases.g2592_surgery_mod.weakRatio = .2
		--useCases.g2592_surgery_mod.weakenMuscles=concat_array(R_ankle_pf,  L_ankle_pf)
		--useCases.g2592_surgery_mod.weakenMuscles=hamstrings
		--useCases.g2592_surgery_mod.cmaLFootCFWeight = 1e-6
		--useCases.g2592_surgery_mod.speedMod=vector3(1,0,0)

		if string.find(g_mode, 'cont_')~=nil then
			--iter:233 bestfvever:3.9107901094599e-07
			accumulate({['map,0,swingFootMod,g2592_surgery_mod,x']= -0.19260809813539, ['map,0,swingFootMod,g2592_surgery_mod,y']= -0.2678698527285, ['map,0,swingFootMod,g2592_surgery_mod,z']= 0.39270879747744, ['map,1,swingFootMod,g2592_surgery_mod,x']= 0.29956185318915, ['map,1,swingFootMod,g2592_surgery_mod,y']= 0.084301191558227, ['map,1,swingFootMod,g2592_surgery_mod,z']= -0.0033021297875873, ['map,2,swingFootMod,g2592_surgery_mod,x']= -0.056554082472535, ['map,2,swingFootMod,g2592_surgery_mod,y']= 0.062772658538821, ['map,2,swingFootMod,g2592_surgery_mod,z']= 0.11418129072202, ['map,0,spprtFootMod,g2592_surgery_mod,x']= 0.10207767062676, ['map,0,spprtFootMod,g2592_surgery_mod,y']= -0.13220686186758, ['map,0,spprtFootMod,g2592_surgery_mod,z']= 0.26029167304072, ['map,1,spprtFootMod,g2592_surgery_mod,x']= 0.32699176281187, ['map,1,spprtFootMod,g2592_surgery_mod,y']= 0.16514243689712, ['map,1,spprtFootMod,g2592_surgery_mod,z']= 0.035345947692649, ['map,2,spprtFootMod,g2592_surgery_mod,x']= -0.33451420741516, ['map,2,spprtFootMod,g2592_surgery_mod,y']= -0.42767628178044, ['map,2,spprtFootMod,g2592_surgery_mod,z']= 0.097795024242233, })
			useCases.g2592_surgery_mod.prec=.1
		end

		if string.find(g_mode, 'a.1')~=nil then
			useCases.g2592_surgery_mod.aObjWeight = .1*useCases.g2592_surgery_mod.aObjWeight
		elseif string.find(g_mode, 'a1')~=nil then
			useCases.g2592_surgery_mod.aObjWeight = 1*useCases.g2592_surgery_mod.aObjWeight
		end

		if string.find(g_mode, 'tl.5')~=nil then
			useCases.g2592_surgery_mod.tenLenRatio = .5
		elseif string.find(g_mode, 'tl.6')~=nil then
			useCases.g2592_surgery_mod.tenLenRatio = .6
		elseif string.find(g_mode, 'tl.7')~=nil then
			useCases.g2592_surgery_mod.tenLenRatio = .7
		elseif string.find(g_mode, 'tl.8')~=nil then
			useCases.g2592_surgery_mod.tenLenRatio = .8
		elseif string.find(g_mode, 'tl.9')~=nil then
			useCases.g2592_surgery_mod.tenLenRatio = .9
		end

		if string.find(g_mode, 'tl2.5')~=nil then
			useCases.g2592_surgery_mod.tenLenRatio2 = .5
		elseif string.find(g_mode, 'tl2.6')~=nil then
			useCases.g2592_surgery_mod.tenLenRatio2 = .6
		elseif string.find(g_mode, 'tl2.7')~=nil then
			useCases.g2592_surgery_mod.tenLenRatio2 = .7
		elseif string.find(g_mode, 'tl2.8')~=nil then
			useCases.g2592_surgery_mod.tenLenRatio2 = .8
		elseif string.find(g_mode, 'tl2.9')~=nil then
			useCases.g2592_surgery_mod.tenLenRatio2 = .9
		end

		if string.find(g_mode, 'ol.5')~=nil then
			useCases.g2592_surgery_mod.optLenRatio = .5
		elseif string.find(g_mode, 'ol.6')~=nil then
			useCases.g2592_surgery_mod.optLenRatio = .6
		elseif string.find(g_mode, 'ol.7')~=nil then
			useCases.g2592_surgery_mod.optLenRatio = .7
		elseif string.find(g_mode, 'ol.8')~=nil then
			useCases.g2592_surgery_mod.optLenRatio = .8
		elseif string.find(g_mode, 'ol.9')~=nil then
			useCases.g2592_surgery_mod.optLenRatio = .9
		end

		if string.find(g_mode, 'tlol.5')~=nil then
			useCases.g2592_surgery_mod.tenLenRatio = .5
			useCases.g2592_surgery_mod.optLenRatio = .5
		elseif string.find(g_mode, 'tlol.82')~=nil then
			useCases.g2592_surgery_mod.tenLenRatio = .82
			useCases.g2592_surgery_mod.optLenRatio = .82
		elseif string.find(g_mode, 'tlol.83')~=nil then
			useCases.g2592_surgery_mod.tenLenRatio = .83
			useCases.g2592_surgery_mod.optLenRatio = .83
		elseif string.find(g_mode, 'tlol.84')~=nil then
			useCases.g2592_surgery_mod.tenLenRatio = .84
			useCases.g2592_surgery_mod.optLenRatio = .84
		elseif string.find(g_mode, 'tlol.85')~=nil then
			useCases.g2592_surgery_mod.tenLenRatio = .85
			useCases.g2592_surgery_mod.optLenRatio = .85
		elseif string.find(g_mode, 'tlol.86')~=nil then
			useCases.g2592_surgery_mod.tenLenRatio = .86
			useCases.g2592_surgery_mod.optLenRatio = .86
		elseif string.find(g_mode, 'tlol.87')~=nil then
			useCases.g2592_surgery_mod.tenLenRatio = .87
			useCases.g2592_surgery_mod.optLenRatio = .87
		elseif string.find(g_mode, 'tlol.88')~=nil then
			useCases.g2592_surgery_mod.tenLenRatio = .88
			useCases.g2592_surgery_mod.optLenRatio = .88
		elseif string.find(g_mode, 'tlol.89')~=nil then
			useCases.g2592_surgery_mod.tenLenRatio = .89
			useCases.g2592_surgery_mod.optLenRatio = .89
		elseif string.find(g_mode, 'tlol.8.9')~=nil then
			useCases.g2592_surgery_mod.tenLenRatio = .8
			useCases.g2592_surgery_mod.optLenRatio = .9
		elseif string.find(g_mode, 'tlol.6')~=nil then
			useCases.g2592_surgery_mod.tenLenRatio = .6
			useCases.g2592_surgery_mod.optLenRatio = .6
		elseif string.find(g_mode, 'tlol.7')~=nil then
			useCases.g2592_surgery_mod.tenLenRatio = .7
			useCases.g2592_surgery_mod.optLenRatio = .7
		elseif string.find(g_mode, 'tlol.8')~=nil then
			useCases.g2592_surgery_mod.tenLenRatio = .8
			useCases.g2592_surgery_mod.optLenRatio = .8
		elseif string.find(g_mode, 'tlol.9')~=nil then
			useCases.g2592_surgery_mod.tenLenRatio = .9
			useCases.g2592_surgery_mod.optLenRatio = .9
		end

		if string.find(g_mode, 'tlol2.5')~=nil then
			useCases.g2592_surgery_mod.tenLenRatio2 = .5
			useCases.g2592_surgery_mod.optLenRatio2 = .5
		elseif string.find(g_mode, 'tlol2.82')~=nil then
			useCases.g2592_surgery_mod.tenLenRatio2 = .82
			useCases.g2592_surgery_mod.optLenRatio2 = .82
		elseif string.find(g_mode, 'tlol2.83')~=nil then
			useCases.g2592_surgery_mod.tenLenRatio2 = .83
			useCases.g2592_surgery_mod.optLenRatio2 = .83
		elseif string.find(g_mode, 'tlol2.84')~=nil then
			useCases.g2592_surgery_mod.tenLenRatio2 = .84
			useCases.g2592_surgery_mod.optLenRatio2 = .84
		elseif string.find(g_mode, 'tlol2.85')~=nil then
			useCases.g2592_surgery_mod.tenLenRatio2 = .85
			useCases.g2592_surgery_mod.optLenRatio2 = .85
		elseif string.find(g_mode, 'tlol2.86')~=nil then
			useCases.g2592_surgery_mod.tenLenRatio2 = .86
			useCases.g2592_surgery_mod.optLenRatio2 = .86
		elseif string.find(g_mode, 'tlol2.87')~=nil then
			useCases.g2592_surgery_mod.tenLenRatio2 = .87
			useCases.g2592_surgery_mod.optLenRatio2 = .87
		elseif string.find(g_mode, 'tlol2.88')~=nil then
			useCases.g2592_surgery_mod.tenLenRatio2 = .88
			useCases.g2592_surgery_mod.optLenRatio2 = .88
		elseif string.find(g_mode, 'tlol2.89')~=nil then
			useCases.g2592_surgery_mod.tenLenRatio2 = .89
			useCases.g2592_surgery_mod.optLenRatio2 = .89
		elseif string.find(g_mode, 'tlol2.8.9')~=nil then
			useCases.g2592_surgery_mod.tenLenRatio2 = .8
			useCases.g2592_surgery_mod.optLenRatio2 = .9
		elseif string.find(g_mode, 'tlol2.6')~=nil then
			useCases.g2592_surgery_mod.tenLenRatio2 = .6
			useCases.g2592_surgery_mod.optLenRatio2 = .6
		elseif string.find(g_mode, 'tlol2.7')~=nil then
			useCases.g2592_surgery_mod.tenLenRatio2 = .7
			useCases.g2592_surgery_mod.optLenRatio2 = .7
		elseif string.find(g_mode, 'tlol2.8')~=nil then
			useCases.g2592_surgery_mod.tenLenRatio2 = .8
			useCases.g2592_surgery_mod.optLenRatio2 = .8
		elseif string.find(g_mode, 'tlol2.9')~=nil then
			useCases.g2592_surgery_mod.tenLenRatio2 = .9
			useCases.g2592_surgery_mod.optLenRatio2 = .9
		end

		if string.find(g_mode, 'mf.2')~=nil then
			useCases.g2592_surgery_mod.weakRatio = .2
		elseif string.find(g_mode, 'mf.1')~=nil then
			useCases.g2592_surgery_mod.weakRatio = .1
		end

		if string.find(g_mode, 'hamst_psoas_tlol')~=nil then
			useCases.g2592_surgery_mod.tenLenMuscles=concat_array(hamstrings, psoases)
			useCases.g2592_surgery_mod.optLenMuscles=concat_array(hamstrings, psoases)

		elseif string.find(g_mode, 'biankpf_tl')~=nil then
			useCases.g2592_surgery_mod.tenLenMuscles=concat_array(R_ankle_pf,  L_ankle_pf)

		elseif string.find(g_mode, 'biankpf_ol')~=nil then
			useCases.g2592_surgery_mod.optLenMuscles=concat_array(R_ankle_pf,  L_ankle_pf)

		elseif string.find(g_mode, 'hamst_psoas_tl')~=nil then
			useCases.g2592_surgery_mod.tenLenMuscles=concat_array(hamstrings, psoases)

		elseif string.find(g_mode, 'hamst_psoas_ol')~=nil then
			useCases.g2592_surgery_mod.optLenMuscles=concat_array(hamstrings, psoases)

		elseif string.find(g_mode, 'hamst_tlol')~=nil then
			useCases.g2592_surgery_mod.tenLenMuscles=hamstrings
			useCases.g2592_surgery_mod.optLenMuscles=hamstrings

		elseif string.find(g_mode, 'hamst_tl')~=nil then
			useCases.g2592_surgery_mod.tenLenMuscles=hamstrings

		elseif string.find(g_mode, 'hamst_ol')~=nil then
			useCases.g2592_surgery_mod.optLenMuscles=hamstrings

		elseif string.find(g_mode, 'psoas_tlol')~=nil then
			useCases.g2592_surgery_mod.tenLenMuscles=psoases
			useCases.g2592_surgery_mod.optLenMuscles=psoases

		elseif string.find(g_mode, 'psoas_tl')~=nil then
			useCases.g2592_surgery_mod.tenLenMuscles=psoases

		elseif string.find(g_mode, 'psoas_ol')~=nil then
			useCases.g2592_surgery_mod.optLenMuscles=psoases
		end

		if string.find(g_mode, 'psoas_tl2')~=nil then
			useCases.g2592_surgery_mod.tenLenMuscles2=psoases
		elseif string.find(g_mode, 'psoas_tlol2')~=nil then
			useCases.g2592_surgery_mod.tenLenMuscles2=psoases
			useCases.g2592_surgery_mod.optLenMuscles2=psoases
		end

		if string.find(g_mode, 'ankPF_mf')~=nil then
			useCases.g2592_surgery_mod.weakenMuscles=concat_array(R_ankle_pf,  L_ankle_pf)

		elseif string.find(g_mode, 'ankPF')~=nil then
			useCases.g2592_surgery_mod.weakenMuscles=concat_array(R_ankle_pf,  L_ankle_pf)
			if string.find(g_mode, 'ankPF.15')~=nil then
				useCases.g2592_surgery_mod.weakRatio = .15
			elseif string.find(g_mode, 'ankPF.1')~=nil then
				useCases.g2592_surgery_mod.weakRatio = .1
			elseif string.find(g_mode, 'ankPF.2')~=nil then
				useCases.g2592_surgery_mod.weakRatio = .2
			elseif string.find(g_mode, 'ankPF.3')~=nil then
				useCases.g2592_surgery_mod.weakRatio = .3
			end

		elseif string.find(g_mode, 'ankDF')~=nil then
			useCases.g2592_surgery_mod.weakenMuscles=concat_array(R_ankle_df,  L_ankle_df)
			if string.find(g_mode, 'ankDF.1')~=nil then
				useCases.g2592_surgery_mod.weakRatio = .1
			elseif string.find(g_mode, 'ankDF.01')~=nil then
				useCases.g2592_surgery_mod.weakRatio = .01
			elseif string.find(g_mode, 'ankDF.05')~=nil then
				useCases.g2592_surgery_mod.weakRatio = .05
			elseif string.find(g_mode, 'ankDF.2')~=nil then
				useCases.g2592_surgery_mod.weakRatio = .2
			elseif string.find(g_mode, 'ankDF.3')~=nil then
				useCases.g2592_surgery_mod.weakRatio = .3
			end

		elseif string.find(g_mode, 'ankLPF')~=nil then
			useCases.g2592_surgery_mod.weakenMuscles=L_ankle_pf
			if string.find(g_mode, 'ankLPF.1')~=nil then
				useCases.g2592_surgery_mod.weakRatio = .1
			elseif string.find(g_mode, 'ankLPF.01')~=nil then
				useCases.g2592_surgery_mod.weakRatio = .01
			elseif string.find(g_mode, 'ankLPF.2')~=nil then
				useCases.g2592_surgery_mod.weakRatio = .2
			end

		elseif string.find(g_mode, 'ankLDF')~=nil then
			useCases.g2592_surgery_mod.weakenMuscles=L_ankle_df
			if string.find(g_mode, 'ankLDF.1')~=nil then
				useCases.g2592_surgery_mod.weakRatio = .1
			elseif string.find(g_mode, 'ankLDF.01')~=nil then
				useCases.g2592_surgery_mod.weakRatio = .01
			elseif string.find(g_mode, 'ankLDF.02')~=nil then
				useCases.g2592_surgery_mod.weakRatio = .02
			elseif string.find(g_mode, 'ankLDF.03')~=nil then
				useCases.g2592_surgery_mod.weakRatio = .03
			elseif string.find(g_mode, 'ankLDF.04')~=nil then
				useCases.g2592_surgery_mod.weakRatio = .04
			elseif string.find(g_mode, 'ankLDF.05')~=nil then
				useCases.g2592_surgery_mod.weakRatio = .05
			elseif string.find(g_mode, 'ankLDF.2')~=nil then
				useCases.g2592_surgery_mod.weakRatio = .2
			elseif string.find(g_mode, 'ankLDF.3')~=nil then
				useCases.g2592_surgery_mod.weakRatio = .3
			end

		elseif string.find(g_mode, 'hipAB')~=nil then
			useCases.g2592_surgery_mod.weakenMuscles=concat_array(R_hip_abd, L_hip_abd)
			if string.find(g_mode, 'hipAB.2')~=nil then
				useCases.g2592_surgery_mod.weakRatio = .2
			elseif string.find(g_mode, 'hipAB.15')~=nil then
				useCases.g2592_surgery_mod.weakRatio = .15
			elseif string.find(g_mode, 'hipAB.3')~=nil then
				useCases.g2592_surgery_mod.weakRatio = .3
			elseif string.find(g_mode, 'hipAB.4')~=nil then
				useCases.g2592_surgery_mod.weakRatio = .4
			end

		elseif string.find(g_mode, 'hipLAB')~=nil then
			useCases.g2592_surgery_mod.weakenMuscles=L_hip_abd
			if string.find(g_mode, 'hipLAB.2')~=nil then
				useCases.g2592_surgery_mod.weakRatio = .2
			elseif string.find(g_mode, 'hipLAB.3')~=nil then
				useCases.g2592_surgery_mod.weakRatio = .3
			end

		elseif string.find(g_mode, 'gluts')~=nil then
			useCases.g2592_surgery_mod.weakenMuscles = concat_array(glut_meds,glut_mins)
			if string.find(g_mode, 'gluts.15')~=nil then
				useCases.g2592_surgery_mod.weakRatio = .15
			elseif string.find(g_mode, 'gluts.1')~=nil then
				useCases.g2592_surgery_mod.weakRatio = .1
			elseif string.find(g_mode, 'gluts.2')~=nil then
				useCases.g2592_surgery_mod.weakRatio = .2
			elseif string.find(g_mode, 'gluts.3')~=nil then
				useCases.g2592_surgery_mod.weakRatio = .3
			elseif string.find(g_mode, 'gluts.4')~=nil then
				useCases.g2592_surgery_mod.weakRatio = .4
			elseif string.find(g_mode, 'gluts.5')~=nil then
				useCases.g2592_surgery_mod.weakRatio = .5
			end

		elseif string.find(g_mode, 'lglut')~=nil then
			useCases.g2592_surgery_mod.weakenMuscles = concat_array(lglut_meds,lglut_mins)
			if string.find(g_mode, 'lglut.1')~=nil then
				useCases.g2592_surgery_mod.weakRatio = .1
			elseif string.find(g_mode, 'lglut.2')~=nil then
				useCases.g2592_surgery_mod.weakRatio = .2
			elseif string.find(g_mode, 'lglut.3')~=nil then
				useCases.g2592_surgery_mod.weakRatio = .3
			elseif string.find(g_mode, 'lglut.4')~=nil then
				useCases.g2592_surgery_mod.weakRatio = .4
			elseif string.find(g_mode, 'lglut.5')~=nil then
				useCases.g2592_surgery_mod.weakRatio = .5
			elseif string.find(g_mode, 'lglut.6')~=nil then
				useCases.g2592_surgery_mod.weakRatio = .6
			end

		elseif string.find(g_mode, 'kneeEX')~=nil then
			useCases.g2592_surgery_mod.weakenMuscles=concat_array(R_knee_ext,L_knee_ext)
			if string.find(g_mode, 'kneeEX.05')~=nil then
				useCases.g2592_surgery_mod.weakRatio = .05
			elseif string.find(g_mode, 'kneeEX.15')~=nil then
				useCases.g2592_surgery_mod.weakRatio = .15
			elseif string.find(g_mode, 'kneeEX.1')~=nil then
				useCases.g2592_surgery_mod.weakRatio = .1
			elseif string.find(g_mode, 'kneeEX.2')~=nil then
				useCases.g2592_surgery_mod.weakRatio = .2
			elseif string.find(g_mode, 'kneeEX.3')~=nil then
				useCases.g2592_surgery_mod.weakRatio = .3
			end

		elseif string.find(g_mode, 'kneeLEX')~=nil then
			useCases.g2592_surgery_mod.weakenMuscles=L_knee_ext
			if string.find(g_mode, 'kneeLEX.05')~=nil then
				useCases.g2592_surgery_mod.weakRatio = .05
			elseif string.find(g_mode, 'kneeLEX.15')~=nil then
				useCases.g2592_surgery_mod.weakRatio = .15
			elseif string.find(g_mode, 'kneeLEX.1')~=nil then
				useCases.g2592_surgery_mod.weakRatio = .1
			elseif string.find(g_mode, 'kneeLEX.2')~=nil then
				useCases.g2592_surgery_mod.weakRatio = .2
			end

		elseif string.find(g_mode, 'hipEX')~=nil then
			useCases.g2592_surgery_mod.weakenMuscles=concat_array(R_hip_ext,L_hip_ext)
			if string.find(g_mode, 'hipEX.1')~=nil then
				useCases.g2592_surgery_mod.weakRatio = .1
			elseif string.find(g_mode, 'hipEX.2')~=nil then
				useCases.g2592_surgery_mod.weakRatio = .2
			elseif string.find(g_mode, 'hipEX.3')~=nil then
				useCases.g2592_surgery_mod.weakRatio = .3
			end

		--elseif string.find(g_mode, 'hamst')~=nil then
			--useCases.g2592_surgery_mod.weakenMuscles=hamstrings
			--if string.find(g_mode, 'hamst.1')~=nil then
				--useCases.g2592_surgery_mod.weakRatio = .1
			--elseif string.find(g_mode, 'hamst.2')~=nil then
				--useCases.g2592_surgery_mod.weakRatio = .2
			--elseif string.find(g_mode, 'hamst.3')~=nil then
				--useCases.g2592_surgery_mod.weakRatio = .3
			--end

		elseif string.find(g_mode, 'rf')~=nil then
			useCases.g2592_surgery_mod.weakenMuscles=rect_fems
			if string.find(g_mode, 'rf.1')~=nil then
				useCases.g2592_surgery_mod.weakRatio = .1
			elseif string.find(g_mode, 'rf.2')~=nil then
				useCases.g2592_surgery_mod.weakRatio = .2
				useCases.g2592_surgery_mod.weakenMuscles=rect_fems
			end

		--elseif string.find(g_mode, 'psoas')~=nil then
			--useCases.g2592_surgery_mod.weakenMuscles=psoases
			--if string.find(g_mode, 'psoas.1')~=nil then
				--useCases.g2592_surgery_mod.weakRatio = .1
			--elseif string.find(g_mode, 'psoas.2')~=nil then
				--useCases.g2592_surgery_mod.weakRatio = .2
			--end

		end

		if string.find(g_mode, 'efrtm5')~=nil then
			useCases.g2592_surgery_mod.cmaEffortWeight = 1e-5
		elseif string.find(g_mode, 'efrtm4')~=nil then
			useCases.g2592_surgery_mod.cmaEffortWeight = 1e-4
		elseif string.find(g_mode, 'efrtm3')~=nil then
			useCases.g2592_surgery_mod.cmaEffortWeight = 1e-3
		elseif string.find(g_mode, 'efrtm6')~=nil then
			useCases.g2592_surgery_mod.cmaEffortWeight = 1e-6
		end

		if string.find(g_mode, 'efcim5')~=nil then
			useCases.g2592_surgery_mod.cmaEffortDistWeight = 1e-5
		elseif string.find(g_mode, 'efcim4')~=nil then
			useCases.g2592_surgery_mod.cmaEffortDistWeight = 1e-4
		elseif string.find(g_mode, 'efcim3')~=nil then
			useCases.g2592_surgery_mod.cmaEffortDistWeight = 1e-3
		elseif string.find(g_mode, 'efcim6')~=nil then
			useCases.g2592_surgery_mod.cmaEffortDistWeight = 1e-6
		elseif string.find(g_mode, 'efcim2')~=nil then
			useCases.g2592_surgery_mod.cmaEffortDistWeight = 1e-2
		elseif string.find(g_mode, 'efcim1')~=nil then
			useCases.g2592_surgery_mod.cmaEffortDistWeight = 1e-1
		end

		if string.find(g_mode, 'lfcfm4')~=nil then
			useCases.g2592_surgery_mod.cmaLFootCFWeight = 1e-4
		elseif string.find(g_mode, 'lfcfm5')~=nil then
			useCases.g2592_surgery_mod.cmaLFootCFWeight = 1e-5
		elseif string.find(g_mode, 'lfcfm6')~=nil then
			useCases.g2592_surgery_mod.cmaLFootCFWeight = 1e-6
		elseif string.find(g_mode, 'lfcfm2')~=nil then
			useCases.g2592_surgery_mod.cmaLFootCFWeight = 1e-2
		elseif string.find(g_mode, 'lfcfm3')~=nil then
			useCases.g2592_surgery_mod.cmaLFootCFWeight = 1e-3
		end

		if string.find(g_mode, 'lankpffom4')~=nil then
			useCases.g2592_surgery_mod.cmaLAnkPFFoWeight = 1e-4
		elseif string.find(g_mode, 'lankpffom5')~=nil then
			useCases.g2592_surgery_mod.cmaLAnkPFFoWeight = 1e-5
		elseif string.find(g_mode, 'lankpffom6')~=nil then
			useCases.g2592_surgery_mod.cmaLAnkPFFoWeight = 1e-6
		elseif string.find(g_mode, 'lankpffom3')~=nil then
			useCases.g2592_surgery_mod.cmaLAnkPFFoWeight = 1e-3
		end

		if string.find(g_mode, 'lrankpffom4')~=nil then
			useCases.g2592_surgery_mod.cmaLAnkPFFoWeight = 1e-4
			useCases.g2592_surgery_mod.cmaRAnkPFFoWeight = 1e-4
		elseif string.find(g_mode, 'lrankpffom5')~=nil then
			useCases.g2592_surgery_mod.cmaLAnkPFFoWeight = 1e-5
			useCases.g2592_surgery_mod.cmaRAnkPFFoWeight = 1e-5
		elseif string.find(g_mode, 'lrankpffom6')~=nil then
			useCases.g2592_surgery_mod.cmaLAnkPFFoWeight = 1e-6
			useCases.g2592_surgery_mod.cmaRAnkPFFoWeight = 1e-6
		elseif string.find(g_mode, 'lrankpffom3')~=nil then
			useCases.g2592_surgery_mod.cmaLAnkPFFoWeight = 1e-3
			useCases.g2592_surgery_mod.cmaRAnkPFFoWeight = 1e-3
		end

		if string.find(g_mode, 'clmpm5')~=nil then
			useCases.g2592_surgery_mod.cmaPoseClamp = 1e-5
		elseif string.find(g_mode, 'clmpm4')~=nil then
			useCases.g2592_surgery_mod.cmaPoseClamp = 1e-4
		elseif string.find(g_mode, 'clmpm3')~=nil then
			useCases.g2592_surgery_mod.cmaPoseClamp = 1e-3
		elseif string.find(g_mode, 'clmpm6')~=nil then
			useCases.g2592_surgery_mod.cmaPoseClamp = 1e-6
		end

		if string.find(g_mode, 'push20_')~=nil then
			magn=20
		elseif string.find(g_mode, 'push40_')~=nil then
			magn=40
		elseif string.find(g_mode, 'push60_')~=nil then
			magn=60
		elseif string.find(g_mode, 'push80_')~=nil then
			magn=80
		elseif string.find(g_mode, 'push90_')~=nil then
			magn=90
		elseif string.find(g_mode, 'push100_')~=nil then
			magn=100
		elseif string.find(g_mode, 'push110_')~=nil then
			magn=110
		elseif string.find(g_mode, 'push120_')~=nil then
			magn=120
		elseif string.find(g_mode, 'push1000_')~=nil then
			magn=1000
		end

		if string.find(g_mode, 'age30')~=nil then
			useCases.g2592_surgery_mod.age = 30
		elseif string.find(g_mode, 'age50')~=nil then
			useCases.g2592_surgery_mod.age = 50
		elseif string.find(g_mode, 'age70')~=nil then
			useCases.g2592_surgery_mod.age = 70
		end

		if string.find(g_mode, 'a.1_hamst_psoas_tl.8_ankPF_mf.2_push')~=nil then
			--iter:179 bestfvever:1.6876442520237e-06
			accumulate({['map,0,swingFootMod,g2592_surgery_mod,x']= -0.086404799999669, ['map,0,swingFootMod,g2592_surgery_mod,y']= 0.19240951905766, ['map,0,swingFootMod,g2592_surgery_mod,z']= 0.55579676273367, ['map,1,swingFootMod,g2592_surgery_mod,x']= 0.66963538731804, ['map,1,swingFootMod,g2592_surgery_mod,y']= 0.17855220520046, ['map,1,swingFootMod,g2592_surgery_mod,z']= -0.07063979441948, ['map,2,swingFootMod,g2592_surgery_mod,x']= -0.52133779009622, ['map,2,swingFootMod,g2592_surgery_mod,y']= 0.27912539029751, ['map,2,swingFootMod,g2592_surgery_mod,z']= 0.016619112095564, ['map,0,spprtFootMod,g2592_surgery_mod,x']= 0.050424489613135, ['map,0,spprtFootMod,g2592_surgery_mod,y']= -1.0259768713766, ['map,0,spprtFootMod,g2592_surgery_mod,z']= -0.095664952002717, ['map,1,spprtFootMod,g2592_surgery_mod,x']= -0.031728504928429, ['map,1,spprtFootMod,g2592_surgery_mod,y']= -0.59243930537485, ['map,1,spprtFootMod,g2592_surgery_mod,z']= -0.37463741424218, ['map,2,spprtFootMod,g2592_surgery_mod,x']= -0.12630034785026, ['map,2,spprtFootMod,g2592_surgery_mod,y']= -0.40255332181395, ['map,2,spprtFootMod,g2592_surgery_mod,z']= 0.47607683649369, })

			useCases.g2592_surgery_mod.impulse = {
				{startframe=100, dir=vector3(0,0,1), mag=magn, dur=.2},
				{startframe=300, dir=vector3(0,0,-1), mag=magn, dur=.2},
				{startframe=500, dir=vector3(1,0,0), mag=magn, dur=.2},
				{startframe=700, dir=vector3(-1,0,0), mag=magn, dur=.2},
				{startframe=900, dir=vector3(0,0,1), mag=magn, dur=.2},
				{startframe=1100, dir=vector3(0,0,-1), mag=magn, dur=.2},
			}
		end
		--knee limit
		if g_mode=='a.1_kneelim30_hiplim10_m4_efcim5' then
			useCases.g2592_surgery_mod.cmaLRKneeLimitWeight = 1e-4
			useCases.g2592_surgery_mod.cmaLRKneeMaxExtAng = -30
			useCases.g2592_surgery_mod.cmaLRHipLimitWeight = useCases.g2592_surgery_mod.cmaLRKneeLimitWeight
			useCases.g2592_surgery_mod.cmaLRHipMaxExtAng = -10
			--iter:148 bestfvever:0.0014072872775303
			accumulate({['map,0,swingFootMod,g2592_surgery_mod,x']= -0.27156173633567, ['map,0,swingFootMod,g2592_surgery_mod,y']= -0.60004513627203, ['map,0,swingFootMod,g2592_surgery_mod,z']= 0.76081448523497, ['map,1,swingFootMod,g2592_surgery_mod,x']= 0.052136373274667, ['map,1,swingFootMod,g2592_surgery_mod,y']= 0.28772566194875, ['map,1,swingFootMod,g2592_surgery_mod,z']= -0.19644526788733, ['map,2,swingFootMod,g2592_surgery_mod,x']= -0.17368811977695, ['map,2,swingFootMod,g2592_surgery_mod,y']= 0.76298531697562, ['map,2,swingFootMod,g2592_surgery_mod,z']= 0.021349096590465, ['map,0,spprtFootMod,g2592_surgery_mod,x']= 0.076437611374153, ['map,0,spprtFootMod,g2592_surgery_mod,y']= -0.53423548482621, ['map,0,spprtFootMod,g2592_surgery_mod,z']= 0.11844190649319, ['map,1,spprtFootMod,g2592_surgery_mod,x']= -0.12084682580061, ['map,1,spprtFootMod,g2592_surgery_mod,y']= 0.67940218967468, ['map,1,spprtFootMod,g2592_surgery_mod,z']= 0.036022652887122, ['map,2,spprtFootMod,g2592_surgery_mod,x']= 0.20255125842068, ['map,2,spprtFootMod,g2592_surgery_mod,y']= -0.75505665803657, ['map,2,spprtFootMod,g2592_surgery_mod,z']= 0.24844297477109, })

		elseif g_mode=='a.1_ankPF.2_efcim5_h.2' then
			accumulate({
				['map,0,swingFootMod,g2592_surgery_mod,y']     = 0.2,
				['map,1,swingFootMod,g2592_surgery_mod,y']     = 0.2,
				['map,2,swingFootMod,g2592_surgery_mod,y']     = 0.2,
				['map,0,spprtFootMod,g2592_surgery_mod,y']     = 0.2,
				['map,1,spprtFootMod,g2592_surgery_mod,y']     = 0.2,
				['map,2,spprtFootMod,g2592_surgery_mod,y']     = 0.2,
			})
			useCases.unmapControlParam(useCases.g2592_surgery_mod)
		end
	end
	-------------------------------------------------

	useCases.g2592_surgery_mod.controlParam['useCases,g2592_surgery_mod,aObjWeight'] = useCases.g2592_surgery_mod.aObjWeight
	useCases.unmapControlParam(useCases.g2592_surgery_mod)
end

