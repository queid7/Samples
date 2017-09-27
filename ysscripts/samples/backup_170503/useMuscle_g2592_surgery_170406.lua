package.path=package.path..";../Samples/ysscripts/lib/?.lua" --;"..package.path
require("utilfunc")

package.path=package.path..";../Samples/ysscripts/samples/?.lua" --;"..package.path
require("useCaseMuscle_gait1956_6steps")


useCases.g2592_surgery=deepCopyTable(useCases.gait1956_6steps)

useCases.g2592_surgery.optimizerMethod='Optimizer.methods.CMAes_ys'

-- dcjo-surgery
useCases.g2592_surgery.useAngleOffset = true
useCases.g2592_surgery.maxAngleOffset = {hipRmax = vector3(20,20,20), hipLmax = vector3(20,20,20), kneeRmax = 30, kneeLmax = 30, ankleRmax = vector3(80, 30, 30), ankleLmax = vector3(80,30,30), mtpRmax = 70, mtpLmax = 70}
--useCases.g2592_surgery.useOffsetPosLimit = true



--useCases.g2592_surgery.useAnkleangleCorrection = true

-- dcjo modify ankle angle of ref. motion
--useCases.g2592_surgery.modifyMotion={{1,5},{11,-30},{19,-10}}

----useCases.g2592_surgery.endSegW=10
--useCases.g2592_surgery.endSegW=15
--useCases.g2592_surgery.prec=.2
----useCases.g2592_surgery.endSegW=20
----useCases.g2592_surgery.prec=.05

--dcjo useAngleOffset
useCases.g2592_surgery.endSegW=15
useCases.g2592_surgery.prec=.40
--useCases.g2592_surgery.prec=0.00 -- error

useCases.g2592_surgery.modelName='g2592_surgery'
useCases.g2592_surgery.grpName='g2592_surgery'
useCases.g2592_surgery.grpNames={'g2592_surgery'}

-- nearest to human MET
useCases.g2592_surgery.ddqObjWeight=10*(25-7)
useCases.g2592_surgery.aObjWeight=5000*62
useCases.g2592_surgery.lambdaObjWeight=.0001
useCases.g2592_surgery.EEobjWeight=100
useCases.g2592_surgery.EEobjWeightAngular=10
useCases.g2592_surgery.momentumWeight=10


function useCases.g2592_surgery:makeOptParam(numKey,...)

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

useCases.g2592_surgery.mapControlParam=function(graph, title, param)
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

function useCases.g2592_surgery:updateStageParam(stage, stageParam)
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
useCases.g2592_surgery.pendControlParam=
{
	['keyframe,0,pendDesiredVel,g2592_surgery,l,z']=0,['keyframe,0,pendDesiredVel,g2592_surgery,l,x']=1,['keyframe,0,pendDesiredVel,g2592_surgery,r,z']=0,['keyframe,0,pendDesiredVel,g2592_surgery,r,x']=1,['keyframe,0,pendDesiredVel,g2592_surgery,L1,z']=0,['keyframe,0,pendDesiredVel,g2592_surgery,L1,x']=1,['keyframe,0,pendDesiredVel,g2592_surgery,R1,z']=0,['keyframe,0,pendDesiredVel,g2592_surgery,R1,x']=1,['keyframe,0,pendDesiredVel,g2592_surgery,L2,z']=0,['keyframe,0,pendDesiredVel,g2592_surgery,L2,x']=1,['keyframe,0,pendDesiredVel,ignore,0,z']=0,['keyframe,0,pendDesiredVel,ignore,0,x']=1,['keyframe,0,pendDesiredVel,ignore,1,z']=0,['keyframe,0,pendDesiredVel,ignore,1,x']=1,
	['keyframe,0,pendDesiredVel,ignore,0,x']=0.55721712382895, ['keyframe,0,pendDesiredVel,ignore,0,z']=-0.11549022684414, ['keyframe,0,pendDesiredVel,g2592_surgery,l,x']=0.83765088460274, ['keyframe,0,pendDesiredVel,g2592_surgery,l,z']=-0.03971153581855, ['keyframe,0,pendDesiredVel,g2592_surgery,r,x']=1.0292990315838, ['keyframe,0,pendDesiredVel,g2592_surgery,r,z']=0.012304645261958, ['keyframe,0,pendDesiredVel,g2592_surgery,L1,x']=1.0830140604807, ['keyframe,0,pendDesiredVel,g2592_surgery,L1,z']=0.021311276252793, ['keyframe,0,pendDesiredVel,g2592_surgery,R1,x']=1.0448069373172, ['keyframe,0,pendDesiredVel,g2592_surgery,R1,z']=0.015212087553609, ['keyframe,0,pendDesiredVel,g2592_surgery,L2,x']=0.99864134148705, ['keyframe,0,pendDesiredVel,g2592_surgery,L2,z']=0.011195374086852, ['keyframe,0,pendDesiredVel,ignore,1,x']=0.9726342007727, ['keyframe,0,pendDesiredVel,ignore,1,z']=0.0046873881077062, 
	['keyframe,0,pendDesiredVel,ignore,0,x']=-0.038573565913782, ['keyframe,0,pendDesiredVel,ignore,0,z']=-0.42183387297139, ['keyframe,0,pendDesiredVel,g2592_surgery,l,x']=2.8733663187598, ['keyframe,0,pendDesiredVel,g2592_surgery,l,z']=0.77374805250546, 
	['keyframe,0,pendDesiredVel,ignore,0,x']=-0.03848565284349, ['keyframe,0,pendDesiredVel,ignore,0,z']=-0.42174541529281, ['keyframe,0,pendDesiredVel,g2592_surgery,l,x']=2.8733683928023, ['keyframe,0,pendDesiredVel,g2592_surgery,l,z']=0.77375288187355, 
	['keyframe,0,pendDesiredVel,g2592_surgery,l,x']=1.8518302499598, ['keyframe,0,pendDesiredVel,g2592_surgery,l,z']=0.32721002628792, ['keyframe,0,pendDesiredVel,g2592_surgery,r,x']=0.64195188458919, ['keyframe,0,pendDesiredVel,g2592_surgery,r,z']=-0.23746102054191, ['keyframe,0,pendDesiredVel,g2592_surgery,L1,x']=1.0569602524197, ['keyframe,0,pendDesiredVel,g2592_surgery,L1,z']=0.0089665722850331, 
	['keyframe,0,pendDesiredVel,g2592_surgery,r,x']=0.65321586961613, ['keyframe,0,pendDesiredVel,g2592_surgery,r,z']=-0.23322355700538, ['keyframe,0,pendDesiredVel,g2592_surgery,L1,x']=1.1600582644838, ['keyframe,0,pendDesiredVel,g2592_surgery,L1,z']=0.16409120361367, ['keyframe,0,pendDesiredVel,g2592_surgery,R1,x']=1.0469024908636, ['keyframe,0,pendDesiredVel,g2592_surgery,R1,z']=0.021499874539069, 
	['keyframe,0,pendDesiredVel,ignore,0,x']=0.057317680328125, ['keyframe,0,pendDesiredVel,ignore,0,z']=-0.37352874903012, ['keyframe,0,pendDesiredVel,g2592_surgery,l,x']=1.688193340133, ['keyframe,0,pendDesiredVel,g2592_surgery,l,z']=0.33135464415606, ['keyframe,0,pendDesiredVel,g2592_surgery,r,x']=0.56260872770468, ['keyframe,0,pendDesiredVel,g2592_surgery,r,z']=-0.29475535050687, 
	['keyframe,0,pendDesiredVel,g2592_surgery,l,x']=1.6732338838641, ['keyframe,0,pendDesiredVel,g2592_surgery,l,z']=0.31227141866769, ['keyframe,0,pendDesiredVel,g2592_surgery,r,x']=0.72633156918842, ['keyframe,0,pendDesiredVel,g2592_surgery,r,z']=-0.30056568375898, ['keyframe,0,pendDesiredVel,g2592_surgery,L1,x']=1.1885783553992, ['keyframe,0,pendDesiredVel,g2592_surgery,L1,z']=0.18162859438481, 
	['keyframe,0,pendDesiredVel,g2592_surgery,r,x']=0.72682249891879, ['keyframe,0,pendDesiredVel,g2592_surgery,r,z']=-0.27195354224673, ['keyframe,0,pendDesiredVel,g2592_surgery,L1,x']=1.1844005645379, ['keyframe,0,pendDesiredVel,g2592_surgery,L1,z']=0.21070016989902, ['keyframe,0,pendDesiredVel,g2592_surgery,R1,x']=1.0436382930349, ['keyframe,0,pendDesiredVel,g2592_surgery,R1,z']=0.019573432369505, 
	['keyframe,0,pendDesiredVel,g2592_surgery,L1,x']=1.1546812185886, ['keyframe,0,pendDesiredVel,g2592_surgery,L1,z']=0.13057608466461, ['keyframe,0,pendDesiredVel,g2592_surgery,R1,x']=1.0169825693409, ['keyframe,0,pendDesiredVel,g2592_surgery,R1,z']=-0.031308161078641, ['keyframe,0,pendDesiredVel,g2592_surgery,L2,x']=0.99332532373797, ['keyframe,0,pendDesiredVel,g2592_surgery,L2,z']=0.023230828039423, ['keyframe,0,pendDesiredVel,ignore,1,x']=0.97210147412166, ['keyframe,0,pendDesiredVel,ignore,1,z']=0.0051664564521254, 
}
useCases.g2592_surgery.pendOptimizationPath=
{
	firstFrames={1*2,39*2,77*2,115*2,153*2,191*2,229*2,266*2},
	segments={'ignore,0','g2592_surgery,l','g2592_surgery,r','g2592_surgery,L1','g2592_surgery,R1','g2592_surgery,L2','ignore,1'},
}
--useCases.g2592_surgery.measureOptCost=useCases.measureOptCost

useCases.g2592_surgery.segmentations=
{
	g2592_surgery={
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
useCases.g2592_surgery.graphParam=
{
	g2592_surgery={
		seg={            'l',  'r',  'L1', 'R1','L'},
		num_key={         3,    3,    3  ,  4,  4},
		key_first={       0,    0,    0  ,  0,  0},
		key_last={       'r', 'L1',  'R1', 'L', 'R1'},
	},
}
useCases.g2592_surgery.graph=
{
	{
		"addInterpolatedSegment",
		grpName="g2592_surgery",
		name="L",
		seg0={"g2592_surgery", "L2"},
		seg1={"g2592_surgery", "L1"},
		startWeight=0, endWeight=1
	},
	{"connectMulti", "g2592_surgery", "l", "r", "L1", "R1", "L", "R1"},
	--{"initialSegment", "g2592_surgery", "l"}
	{"initialSegment", "g2592_surgery", "R1"}
	-- R1 L R1 L R1 L ...
	-- L(115*2) R1(153*2) L(191*2)
}
useCases.g2592_surgery.segNames=
{
	"l", "r", "L1", 'R1','L',
}

-- dcjo
useCases.g2592_surgery.keyframes={
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
	useCases.g2592_surgery.controlParam={
		['useCases,g2592_surgery,COMobjWeight']=0, 
		['useCases,g2592_surgery,conservativeW']=1, 
		--['useCases,g2592_surgery,contactMargin']=0.01, 
		['useCases,g2592_surgery,dotMomentumScale']=0.3, 
		['useCases,g2592_surgery,excludeRoot']=true, 
		['useCases,g2592_surgery,headControlWeight']=0, 
		['useCases,g2592_surgery,k_d_HEAD']=14, 
		['useCases,g2592_surgery,k_p_HEAD']=0, 
		['useCases,g2592_surgery,k_d_EE']=24, 
		['useCases,g2592_surgery,k_d_momentum']=10, 
		['useCases,g2592_surgery,k_p_EE']=120, 

		--['useCases,g2592_surgery,maxPenetratingVel']=0, 
		['useCases,g2592_surgery,momentumThr']=50, 
		['useCases,g2592_surgery,noComvelDependentFootAdjustment']=true, 
		['useCases,g2592_surgery,noIntersectionPrevenction']=true, 
		['useCases,g2592_surgery,numericalDerivDmot']=true, 
		['useCases,g2592_surgery,perClassContactMargin']=1, 
		['useCases,g2592_surgery,turnGain']=10, 
		['useCases,g2592_surgery,velMarginOffset']=0, 

		['useCases,g2592_surgery,actuationType']      = useCases.g2592_surgery.actuationType,
		['useCases,g2592_surgery,k_p_ID']             = useCases.g2592_surgery.k_p_ID,
		['useCases,g2592_surgery,k_d_ID']             = useCases.g2592_surgery.k_d_ID,
		['useCases,g2592_surgery,ddqObjWeight']       = useCases.g2592_surgery.ddqObjWeight,
		['useCases,g2592_surgery,tauObjWeight']       = useCases.g2592_surgery.tauObjWeight,
		['useCases,g2592_surgery,ftObjWeight']        = useCases.g2592_surgery.ftObjWeight,
		['useCases,g2592_surgery,aObjWeight']         = useCases.g2592_surgery.aObjWeight,
		['useCases,g2592_surgery,lambdaObjWeight']    = useCases.g2592_surgery.lambdaObjWeight,
		['useCases,g2592_surgery,EEobjWeight']        = useCases.g2592_surgery.EEobjWeight,
		['useCases,g2592_surgery,EEobjWeightAngular'] = useCases.g2592_surgery.EEobjWeightAngular,
		['useCases,g2592_surgery,momentumWeight']     = useCases.g2592_surgery.momentumWeight,
		['useCases,g2592_surgery,tauMax']             = useCases.g2592_surgery.tauMax,
		['useCases,g2592_surgery,ftMax']              = useCases.g2592_surgery.ftMax,
	}

	-- update controlParam 
	local function accumulate(cp_mod)
		local useCase=useCases.g2592_surgery
		print("accumulate in surgery")
		useCases.accumulate(useCase, cp_mod)
	end

	accumulate({
		----['keyframe,0,footRmod,g2592_surgery,l,y']  = 0.,
		--['keyframe,1,footRmod,g2592_surgery,l,y']  = 0.05,
		--['keyframe,0,footLmod,g2592_surgery,r,y']  = 0.05,
		--['keyframe,1,footLmod,g2592_surgery,r,y']  = 0.1,
		--['keyframe,0,footRmod,g2592_surgery,L1,y'] = 0.0,
		--['keyframe,1,footRmod,g2592_surgery,L1,y'] = 0.1,
		--['map,0,swingFootMod,g2592_surgery,y']     = 0.1,
		--['map,1,swingFootMod,g2592_surgery,y']     = 0.3,
		--['map,2,swingFootMod,g2592_surgery,y']     = 0.2,

		------['keyframe,0,footLmod,g2592_surgery,l,y']  = 0.,
		------['keyframe,1,footLmod,g2592_surgery,l,y']  = 0.,
		----['keyframe,0,footRmod,g2592_surgery,r,y']  = 0.,
		----['keyframe,1,footRmod,g2592_surgery,r,y']  = -.1,
		----['keyframe,0,footLmod,g2592_surgery,L1,y'] = 0.,
		----['keyframe,1,footLmod,g2592_surgery,L1,y'] = -.1,
		----['map,0,spprtFootMod,g2592_surgery,y']     = 0.,
		----['map,1,spprtFootMod,g2592_surgery,y']     = 0.,
		----['map,2,spprtFootMod,g2592_surgery,y']     = 0.,
	})
	useCases.unmapControlParam(useCases.g2592_surgery)


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
		accumulate({['map,0,swingFootMod,g2592_surgery,x']= 0.0347720834374, ['map,0,swingFootMod,g2592_surgery,y']= -0.30775028709688, ['map,0,swingFootMod,g2592_surgery,z']= -0.015839891161517, ['map,1,swingFootMod,g2592_surgery,x']= -0.037229018189522, ['map,1,swingFootMod,g2592_surgery,y']= 0.068387638632828, ['map,1,swingFootMod,g2592_surgery,z']= -0.062432068770353, ['map,2,swingFootMod,g2592_surgery,x']= -0.0041280934069137, ['map,2,swingFootMod,g2592_surgery,y']= 0.31738778084309, ['map,2,swingFootMod,g2592_surgery,z']= 0.077092889872588, ['map,0,spprtFootMod,g2592_surgery,x']= -0.020016014715454, ['map,0,spprtFootMod,g2592_surgery,y']= -0.04006379370993, ['map,0,spprtFootMod,g2592_surgery,z']= -0.10363134026533, ['map,1,spprtFootMod,g2592_surgery,x']= -0.13507550600224, ['map,1,spprtFootMod,g2592_surgery,y']= -0.36372277242858, ['map,1,spprtFootMod,g2592_surgery,z']= 0.066700340771708, ['map,2,spprtFootMod,g2592_surgery,x']= -0.053814760720267, ['map,2,spprtFootMod,g2592_surgery,y']= -0.92505944566029, ['map,2,spprtFootMod,g2592_surgery,z']= -0.013363893363109, })

	elseif g_mode=='test_limited_ankle' then
		useCases.g2592_surgery.aObjWeight = .1*useCases.g2592_surgery.aObjWeight
		useCases.g2592_surgery.maxAngleOffset.ankleRmax:setX(80)
		useCases.g2592_surgery.maxAngleOffset.ankleLmax:setX(80)

		useCases.g2592_surgery.cmaEffortWeight = 4.33e-09
accumulate({['map,0,swingHipMod,g2592_surgery,x']= 0.13547327705982, ['map,0,swingHipMod,g2592_surgery,y']= -0.62825693724008, ['map,0,swingHipMod,g2592_surgery,z']= 0.23321705176926, ['map,1,swingHipMod,g2592_surgery,x']= 0.16571950315074, ['map,1,swingHipMod,g2592_surgery,y']= 0.36506266515053, ['map,1,swingHipMod,g2592_surgery,z']= 0.25140401954513, ['map,2,swingHipMod,g2592_surgery,x']= 0.78707455878539, ['map,2,swingHipMod,g2592_surgery,y']= 0.13528444810783, ['map,2,swingHipMod,g2592_surgery,z']= 0.2797773219569, ['map,0,spprtHipMod,g2592_surgery,x']= 0.46326192829164, ['map,0,spprtHipMod,g2592_surgery,y']= -0.33950497660769, ['map,0,spprtHipMod,g2592_surgery,z']= -0.18094929815952, ['map,1,spprtHipMod,g2592_surgery,x']= 0.65305695250077, ['map,1,spprtHipMod,g2592_surgery,y']= -0.6174698642424, ['map,1,spprtHipMod,g2592_surgery,z']= -0.2748423183586, ['map,2,spprtHipMod,g2592_surgery,x']= 1.3628980408765, ['map,2,spprtHipMod,g2592_surgery,y']= -0.017845891596927, ['map,2,spprtHipMod,g2592_surgery,z']= -1.095372378279, ['map,0,swingKneeMod,g2592_surgery']= 0.61966210847, ['map,1,swingKneeMod,g2592_surgery']= 0.38487807539905, ['map,2,swingKneeMod,g2592_surgery']= -0.59060250653663, ['map,0,spprtKneeMod,g2592_surgery']= -0.37265121375521, ['map,1,spprtKneeMod,g2592_surgery']= -0.20479487084991, ['map,2,spprtKneeMod,g2592_surgery']= 0.066797053058073, ['map,0,swingAnkleMod,g2592_surgery,x']= 0.5576572188456, ['map,0,swingAnkleMod,g2592_surgery,y']= -0.25453957491321, ['map,0,swingAnkleMod,g2592_surgery,z']= -0.39502167435288, ['map,1,swingAnkleMod,g2592_surgery,x']= 0.34260806531151, ['map,1,swingAnkleMod,g2592_surgery,y']= -0.64121546738645, ['map,1,swingAnkleMod,g2592_surgery,z']= 0.036683376576892, ['map,2,swingAnkleMod,g2592_surgery,x']= 0.23332033714062, ['map,2,swingAnkleMod,g2592_surgery,y']= 0.62980116038429, ['map,2,swingAnkleMod,g2592_surgery,z']= 0.79303509124595, ['map,0,spprtAnkleMod,g2592_surgery,x']= 0.8027134105397, ['map,0,spprtAnkleMod,g2592_surgery,y']= -0.087666825785693, ['map,0,spprtAnkleMod,g2592_surgery,z']= -0.74954074748454, ['map,1,spprtAnkleMod,g2592_surgery,x']= 0.24006217416892, ['map,1,spprtAnkleMod,g2592_surgery,y']= 0.37149161316448, ['map,1,spprtAnkleMod,g2592_surgery,z']= 0.016196704042314, ['map,2,spprtAnkleMod,g2592_surgery,x']= -0.090346333758015, ['map,2,spprtAnkleMod,g2592_surgery,y']= 0.37969006176272, ['map,2,spprtAnkleMod,g2592_surgery,z']= 0.37401858967333, ['map,0,swingMtpMod,g2592_surgery']= 0.0013395948576964, ['map,1,swingMtpMod,g2592_surgery']= 0.30955570577365, ['map,2,swingMtpMod,g2592_surgery']= 0.51513775228397, ['map,0,spprtMtpMod,g2592_surgery']= 0.0227314662088, ['map,1,spprtMtpMod,g2592_surgery']= 0.7253216432023, ['map,2,spprtMtpMod,g2592_surgery']= -0.48073478300984, })

	
	elseif g_mode=='test' then
		useCases.g2592_surgery.aObjWeight = .1*useCases.g2592_surgery.aObjWeight
		--useCases.g2592_surgery.cmaEffortDistWeight = 1.76e-08
		useCases.g2592_surgery.cmaEffortWeight = 4.33e-09

		-- iter
--accumulate({['map,0,swingHipMod,g2592_surgery,x']= 0.13891678714829, ['map,0,swingHipMod,g2592_surgery,y']= -0.45413553265796, ['map,0,swingHipMod,g2592_surgery,z']= -0.15351274511259, ['map,1,swingHipMod,g2592_surgery,x']= 0.48549675696014, ['map,1,swingHipMod,g2592_surgery,y']= 0.048981456911578, ['map,1,swingHipMod,g2592_surgery,z']= -0.67306823680005, ['map,2,swingHipMod,g2592_surgery,x']= 0.42757247580431, ['map,2,swingHipMod,g2592_surgery,y']= -0.19681789043612, ['map,2,swingHipMod,g2592_surgery,z']= 0.33626906243181, ['map,0,spprtHipMod,g2592_surgery,x']= -0.24065462012265, ['map,0,spprtHipMod,g2592_surgery,y']= -0.087679200166787, ['map,0,spprtHipMod,g2592_surgery,z']= 0.21360103795492, ['map,1,spprtHipMod,g2592_surgery,x']= 0.58565064976979, ['map,1,spprtHipMod,g2592_surgery,y']= -0.084778568995706, ['map,1,spprtHipMod,g2592_surgery,z']= 0.89299168880253, ['map,2,spprtHipMod,g2592_surgery,x']= 0.34916458621298, ['map,2,spprtHipMod,g2592_surgery,y']= -0.96949306894224, ['map,2,spprtHipMod,g2592_surgery,z']= -0.50171933122528, ['map,0,swingKneeMod,g2592_surgery']= 0.46029153708705, ['map,1,swingKneeMod,g2592_surgery']= 0.29000872919219, ['map,2,swingKneeMod,g2592_surgery']= -0.071417451992243, ['map,0,spprtKneeMod,g2592_surgery']= 0.37549543536355, ['map,1,spprtKneeMod,g2592_surgery']= 0.37852417436918, ['map,2,spprtKneeMod,g2592_surgery']= 0.052320120187208, ['map,0,swingAnkleMod,g2592_surgery,x']= 0.41350380347135, ['map,0,swingAnkleMod,g2592_surgery,y']= 0.067772434331722, ['map,0,swingAnkleMod,g2592_surgery,z']= -0.26874041843796, ['map,1,swingAnkleMod,g2592_surgery,x']= 0.023178311829302, ['map,1,swingAnkleMod,g2592_surgery,y']= 0.47415421849845, ['map,1,swingAnkleMod,g2592_surgery,z']= 0.18450402565765, ['map,2,swingAnkleMod,g2592_surgery,x']= 0.34276551637579, ['map,2,swingAnkleMod,g2592_surgery,y']= 0.24946484717582, ['map,2,swingAnkleMod,g2592_surgery,z']= 0.1890980483721, ['map,0,spprtAnkleMod,g2592_surgery,x']= 0.20162551253411, ['map,0,spprtAnkleMod,g2592_surgery,y']= -0.27312399116965, ['map,0,spprtAnkleMod,g2592_surgery,z']= -0.293815181256, ['map,1,spprtAnkleMod,g2592_surgery,x']= 0.063890753930193, ['map,1,spprtAnkleMod,g2592_surgery,y']= 0.58675778617102, ['map,1,spprtAnkleMod,g2592_surgery,z']= -0.21401639741778, ['map,2,spprtAnkleMod,g2592_surgery,x']= 0.15949325064529, ['map,2,spprtAnkleMod,g2592_surgery,y']= -0.030671599774767, ['map,2,spprtAnkleMod,g2592_surgery,z']= 0.39862867761868, ['map,0,swingMtpMod,g2592_surgery']= -0.40727865894241, ['map,1,swingMtpMod,g2592_surgery']= -0.072843828212913, ['map,2,swingMtpMod,g2592_surgery']= -0.057679862683878, ['map,0,spprtMtpMod,g2592_surgery']= 0.016861685706035, ['map,1,spprtMtpMod,g2592_surgery']= 0.030352920776035, ['map,2,spprtMtpMod,g2592_surgery']= -0.2646674474493, })




	
	elseif g_mode=='test_wrong_passive' then
		useCases.g2592_surgery.aObjWeight = .1*useCases.g2592_surgery.aObjWeight
--		useCases.g2592_surgery.cmaEffortDistWeight = 1.76e-08
--accumulate({['map,0,swingHipMod,g2592_surgery,x']= 0.20621419965616, ['map,0,swingHipMod,g2592_surgery,y']= -0.16520137155779, ['map,0,swingHipMod,g2592_surgery,z']= 0.20964997203883, ['map,1,swingHipMod,g2592_surgery,x']= 0.67892079179363, ['map,1,swingHipMod,g2592_surgery,y']= -0.094025148900626, ['map,1,swingHipMod,g2592_surgery,z']= 0.1304540023904, ['map,2,swingHipMod,g2592_surgery,x']= 0.33996155709824, ['map,2,swingHipMod,g2592_surgery,y']= -0.016196693750237, ['map,2,swingHipMod,g2592_surgery,z']= 0.20506200666805, ['map,0,spprtHipMod,g2592_surgery,x']= 0.048214485350453, ['map,0,spprtHipMod,g2592_surgery,y']= -0.22596751148703, ['map,0,spprtHipMod,g2592_surgery,z']= 0.021473479355361, ['map,1,spprtHipMod,g2592_surgery,x']= 0.63412354909977, ['map,1,spprtHipMod,g2592_surgery,y']= -0.13544866648985, ['map,1,spprtHipMod,g2592_surgery,z']= 0.054068840662468, ['map,2,spprtHipMod,g2592_surgery,x']= 0.43882200417952, ['map,2,spprtHipMod,g2592_surgery,y']= -0.2246537513307, ['map,2,spprtHipMod,g2592_surgery,z']= -0.29852987090809, ['map,0,swingKneeMod,g2592_surgery']= 0.37865494318483, ['map,1,swingKneeMod,g2592_surgery']= 0.41465056668843, ['map,2,swingKneeMod,g2592_surgery']= -0.15362944949565, ['map,0,spprtKneeMod,g2592_surgery']= -0.16913784286905, ['map,1,spprtKneeMod,g2592_surgery']= -0.094309658202739, ['map,2,spprtKneeMod,g2592_surgery']= -0.22212266595626, ['map,0,swingAnkleMod,g2592_surgery,x']= 0.0090338714668416, ['map,0,swingAnkleMod,g2592_surgery,y']= -0.60557971435272, ['map,0,swingAnkleMod,g2592_surgery,z']= -0.55939767329865, ['map,1,swingAnkleMod,g2592_surgery,x']= 0.32545552652704, ['map,1,swingAnkleMod,g2592_surgery,y']= -0.75283699237875, ['map,1,swingAnkleMod,g2592_surgery,z']= 0.48274777602872, ['map,2,swingAnkleMod,g2592_surgery,x']= 0.44357771042599, ['map,2,swingAnkleMod,g2592_surgery,y']= -0.56323759325546, ['map,2,swingAnkleMod,g2592_surgery,z']= 0.70650293540873, ['map,0,spprtAnkleMod,g2592_surgery,x']= 0.60694776646023, ['map,0,spprtAnkleMod,g2592_surgery,y']= 0.25938901390582, ['map,0,spprtAnkleMod,g2592_surgery,z']= -0.30222193108893, ['map,1,spprtAnkleMod,g2592_surgery,x']= 0.20814981826142, ['map,1,spprtAnkleMod,g2592_surgery,y']= -0.11018246923487, ['map,1,spprtAnkleMod,g2592_surgery,z']= -0.087559023744909, ['map,2,spprtAnkleMod,g2592_surgery,x']= -0.050066056385581, ['map,2,spprtAnkleMod,g2592_surgery,y']= 0.61786822489374, ['map,2,spprtAnkleMod,g2592_surgery,z']= 0.6682426966309, ['map,0,swingMtpMod,g2592_surgery']= 0.089768855091434, ['map,1,swingMtpMod,g2592_surgery']= 0.28961869387555, ['map,2,swingMtpMod,g2592_surgery']= 0.0038311826966065, ['map,0,spprtMtpMod,g2592_surgery']= 0.16617215306844, ['map,1,spprtMtpMod,g2592_surgery']= 0.35081398372488, ['map,2,spprtMtpMod,g2592_surgery']= -0.18563934737423, })

		
--0808
--useCases.g2592_surgery.cmaEffortWeight = 4.33e-09
--accumulate({['map,0,swingHipMod,g2592_surgery,x']= 0.09082172652367, ['map,0,swingHipMod,g2592_surgery,y']= -0.26757788425293, ['map,0,swingHipMod,g2592_surgery,z']= 0.17719667403175, ['map,1,swingHipMod,g2592_surgery,x']= 0.5688287137944, ['map,1,swingHipMod,g2592_surgery,y']= 0.099718133108417, ['map,1,swingHipMod,g2592_surgery,z']= 0.52350511489291, ['map,2,swingHipMod,g2592_surgery,x']= 0.93821674392057, ['map,2,swingHipMod,g2592_surgery,y']= -0.0077716006605996, ['map,2,swingHipMod,g2592_surgery,z']= -0.020185059248363, ['map,0,spprtHipMod,g2592_surgery,x']= -0.35992293940001, ['map,0,spprtHipMod,g2592_surgery,y']= -0.10793672017618, ['map,0,spprtHipMod,g2592_surgery,z']= 0.55460288082767, ['map,1,spprtHipMod,g2592_surgery,x']= 1.0661171261213, ['map,1,spprtHipMod,g2592_surgery,y']= -0.31649830679381, ['map,1,spprtHipMod,g2592_surgery,z']= 0.23358449331663, ['map,2,spprtHipMod,g2592_surgery,x']= 0.65026299860326, ['map,2,spprtHipMod,g2592_surgery,y']= -0.083019216004243, ['map,2,spprtHipMod,g2592_surgery,z']= -0.51244954283564, ['map,0,swingKneeMod,g2592_surgery']= 0.62724862137871, ['map,1,swingKneeMod,g2592_surgery']= 0.14071990792225, ['map,2,swingKneeMod,g2592_surgery']= -0.72370431502123, ['map,0,spprtKneeMod,g2592_surgery']= 0.48262156621047, ['map,1,spprtKneeMod,g2592_surgery']= -0.12543933746304, ['map,2,spprtKneeMod,g2592_surgery']= -0.13864483267434, ['map,0,swingAnkleMod,g2592_surgery,x']= 0.11568635261492, ['map,0,swingAnkleMod,g2592_surgery,y']= -1.0394770895381, ['map,0,swingAnkleMod,g2592_surgery,z']= -0.31107246969537, ['map,1,swingAnkleMod,g2592_surgery,x']= 0.62688713279632, ['map,1,swingAnkleMod,g2592_surgery,y']= 0.23667421382397, ['map,1,swingAnkleMod,g2592_surgery,z']= 0.2966335470453, ['map,2,swingAnkleMod,g2592_surgery,x']= 0.11382830253729, ['map,2,swingAnkleMod,g2592_surgery,y']= 0.38536262797936, ['map,2,swingAnkleMod,g2592_surgery,z']= 0.67528187037469, ['map,0,spprtAnkleMod,g2592_surgery,x']= 0.16574109479251, ['map,0,spprtAnkleMod,g2592_surgery,y']= 0.28689531288662, ['map,0,spprtAnkleMod,g2592_surgery,z']= -0.54900727532331, ['map,1,spprtAnkleMod,g2592_surgery,x']= 0.65087559704975, ['map,1,spprtAnkleMod,g2592_surgery,y']= 0.055213720878214, ['map,1,spprtAnkleMod,g2592_surgery,z']= -0.88950839295914, ['map,2,spprtAnkleMod,g2592_surgery,x']= -0.2083413792172, ['map,2,spprtAnkleMod,g2592_surgery,y']= 0.59988172246565, ['map,2,spprtAnkleMod,g2592_surgery,z']= 0.98088300632422, ['map,0,swingMtpMod,g2592_surgery']= 0.082070720743388, ['map,1,swingMtpMod,g2592_surgery']= -0.50499191426515, ['map,2,swingMtpMod,g2592_surgery']= 0.063496160241869, ['map,0,spprtMtpMod,g2592_surgery']= -0.62259808283644, ['map,1,spprtMtpMod,g2592_surgery']= 0.42190747176005, ['map,2,spprtMtpMod,g2592_surgery']= -0.259293652726, })
--		accumulate({['map,0,swingHipMod,g2592_surgery,x']= 0.45790293353438, ['map,0,swingHipMod,g2592_surgery,y']= -0.28576471622255, ['map,0,swingHipMod,g2592_surgery,z']= 0.36639191899122, ['map,1,swingHipMod,g2592_surgery,x']= 0.66204988689926, ['map,1,swingHipMod,g2592_surgery,y']= 0.12818284487074, ['map,1,swingHipMod,g2592_surgery,z']= 0.078483825437417, ['map,2,swingHipMod,g2592_surgery,x']= 0.61888343217651, ['map,2,swingHipMod,g2592_surgery,y']= -0.051273787338596, ['map,2,swingHipMod,g2592_surgery,z']= -0.32842178644417, ['map,0,spprtHipMod,g2592_surgery,x']= -0.21423727795177, ['map,0,spprtHipMod,g2592_surgery,y']= -0.067367284607812, ['map,0,spprtHipMod,g2592_surgery,z']= 0.33629948599507, ['map,1,spprtHipMod,g2592_surgery,x']= 0.79034696907948, ['map,1,spprtHipMod,g2592_surgery,y']= -0.18745191976578, ['map,1,spprtHipMod,g2592_surgery,z']= 0.1368463484387, ['map,2,spprtHipMod,g2592_surgery,x']= 0.43206811466584, ['map,2,spprtHipMod,g2592_surgery,y']= -0.12167453617558, ['map,2,spprtHipMod,g2592_surgery,z']= -0.16862803228276, ['map,0,swingKneeMod,g2592_surgery']= 0.42929333833781, ['map,1,swingKneeMod,g2592_surgery']= 0.18880943696014, ['map,2,swingKneeMod,g2592_surgery']= -0.37845216399433, ['map,0,spprtKneeMod,g2592_surgery']= 0.12775853002178, ['map,1,spprtKneeMod,g2592_surgery']= -0.0061661596517956, ['map,2,spprtKneeMod,g2592_surgery']= 0.04159759075436, ['map,0,swingAnkleMod,g2592_surgery,x']= 0.10320077217192, ['map,0,swingAnkleMod,g2592_surgery,y']= -0.70257665054822, ['map,0,swingAnkleMod,g2592_surgery,z']= -0.049811500043311, ['map,1,swingAnkleMod,g2592_surgery,x']= 0.38524313482853, ['map,1,swingAnkleMod,g2592_surgery,y']= 0.31166646035062, ['map,1,swingAnkleMod,g2592_surgery,z']= 0.057817198965205, ['map,2,swingAnkleMod,g2592_surgery,x']= 0.30898805065092, ['map,2,swingAnkleMod,g2592_surgery,y']= 0.45694811004025, ['map,2,swingAnkleMod,g2592_surgery,z']= 0.75998822324401, ['map,0,spprtAnkleMod,g2592_surgery,x']= 0.13777979957332, ['map,0,spprtAnkleMod,g2592_surgery,y']= 0.33457152265388, ['map,0,spprtAnkleMod,g2592_surgery,z']= -0.26656195579127, ['map,1,spprtAnkleMod,g2592_surgery,x']= 0.54500734916635, ['map,1,spprtAnkleMod,g2592_surgery,y']= 0.17759335615954, ['map,1,spprtAnkleMod,g2592_surgery,z']= -0.31058656890418, ['map,2,spprtAnkleMod,g2592_surgery,x']= -0.074148414370615, ['map,2,spprtAnkleMod,g2592_surgery,y']= 0.53826243027179, ['map,2,spprtAnkleMod,g2592_surgery,z']= 0.66169013221427, ['map,0,swingMtpMod,g2592_surgery']= -0.13185738039323, ['map,1,swingMtpMod,g2592_surgery']= -0.17125845632413, ['map,2,swingMtpMod,g2592_surgery']= -0.0025032250497435, ['map,0,spprtMtpMod,g2592_surgery']= -0.27403029272932, ['map,1,spprtMtpMod,g2592_surgery']= 0.75744773701396, ['map,2,spprtMtpMod,g2592_surgery']= -0.024339376666701, })

--accumulate({['map,0,swingHipMod,g2592_surgery,x']= 0.010377914785313, ['map,0,swingHipMod,g2592_surgery,y']= -0.26357487057289, ['map,0,swingHipMod,g2592_surgery,z']= -0.16126085790917, ['map,1,swingHipMod,g2592_surgery,x']= 1.3374283196293, ['map,1,swingHipMod,g2592_surgery,y']= 0.27149057210316, ['map,1,swingHipMod,g2592_surgery,z']= 0.61862841978246, ['map,2,swingHipMod,g2592_surgery,x']= 0.40829625761731, ['map,2,swingHipMod,g2592_surgery,y']= 0.048197385977398, ['map,2,swingHipMod,g2592_surgery,z']= -0.20942345410667, ['map,0,spprtHipMod,g2592_surgery,x']= -0.32016911276493, ['map,0,spprtHipMod,g2592_surgery,y']= -0.13356875583382, ['map,0,spprtHipMod,g2592_surgery,z']= 0.2246115326938, ['map,1,spprtHipMod,g2592_surgery,x']= 0.65054287875455, ['map,1,spprtHipMod,g2592_surgery,y']= -0.18884934929368, ['map,1,spprtHipMod,g2592_surgery,z']= -0.026800322128891, ['map,2,spprtHipMod,g2592_surgery,x']= 0.45189700986957, ['map,2,spprtHipMod,g2592_surgery,y']= 0.0089738287797801, ['map,2,spprtHipMod,g2592_surgery,z']= -0.15973879275926, ['map,0,swingKneeMod,g2592_surgery']= 0.37129341977944, ['map,1,swingKneeMod,g2592_surgery']= -0.14080828649598, ['map,2,swingKneeMod,g2592_surgery']= -0.35247817696681, ['map,0,spprtKneeMod,g2592_surgery']= 0.2698524890682, ['map,1,spprtKneeMod,g2592_surgery']= 0.42824905200176, ['map,2,spprtKneeMod,g2592_surgery']= 0.26062571656085, ['map,0,swingAnkleMod,g2592_surgery,x']= 0.16146102496253, ['map,0,swingAnkleMod,g2592_surgery,y']= -0.13965251525887, ['map,0,swingAnkleMod,g2592_surgery,z']= 0.17277846234593, ['map,1,swingAnkleMod,g2592_surgery,x']= 0.64934358215921, ['map,1,swingAnkleMod,g2592_surgery,y']= 0.3795596634933, ['map,1,swingAnkleMod,g2592_surgery,z']= 0.053176538082342, ['map,2,swingAnkleMod,g2592_surgery,x']= -0.0090516609655613, ['map,2,swingAnkleMod,g2592_surgery,y']= -0.54080907471063, ['map,2,swingAnkleMod,g2592_surgery,z']= 0.52712267028057, ['map,0,spprtAnkleMod,g2592_surgery,x']= 0.22319132547018, ['map,0,spprtAnkleMod,g2592_surgery,y']= 0.024675954848358, ['map,0,spprtAnkleMod,g2592_surgery,z']= -0.73562022694169, ['map,1,spprtAnkleMod,g2592_surgery,x']= 0.3426335756726, ['map,1,spprtAnkleMod,g2592_surgery,y']= 0.38070388820429, ['map,1,spprtAnkleMod,g2592_surgery,z']= 0.33722405930759, ['map,2,spprtAnkleMod,g2592_surgery,x']= 0.12456591441323, ['map,2,spprtAnkleMod,g2592_surgery,y']= -0.10207977636607, ['map,2,spprtAnkleMod,g2592_surgery,z']= 0.34006424037916, ['map,0,swingMtpMod,g2592_surgery']= -0.11220877785362, ['map,1,swingMtpMod,g2592_surgery']= 0.26266519667051, ['map,2,swingMtpMod,g2592_surgery']= 0.29455337004772, ['map,0,spprtMtpMod,g2592_surgery']= -0.62965734710969, ['map,1,spprtMtpMod,g2592_surgery']= 0.28016524915627, ['map,2,spprtMtpMod,g2592_surgery']= 0.26506431521332, })
-- todo
-- manually



-- no cmaEffort		(stick leg)
--accumulate({['map,0,swingHipMod,g2592_surgery,x']= 1.0901240077405, ['map,0,swingHipMod,g2592_surgery,y']= -0.910459520938, ['map,0,swingHipMod,g2592_surgery,z']= 0.42652543468564, ['map,1,swingHipMod,g2592_surgery,x']= 1.2048398181976, ['map,1,swingHipMod,g2592_surgery,y']= -0.80999984359897, ['map,1,swingHipMod,g2592_surgery,z']= 0.27982376795835, ['map,2,swingHipMod,g2592_surgery,x']= -0.0032915698940714, ['map,2,swingHipMod,g2592_surgery,y']= 0.22524088482814, ['map,2,swingHipMod,g2592_surgery,z']= -0.40782936959809, ['map,0,spprtHipMod,g2592_surgery,x']= -0.40678818282867, ['map,0,spprtHipMod,g2592_surgery,y']= -0.26862210497108, ['map,0,spprtHipMod,g2592_surgery,z']= 0.13359199269604, ['map,1,spprtHipMod,g2592_surgery,x']= 0.73888308064771, ['map,1,spprtHipMod,g2592_surgery,y']= -0.097600537068923, ['map,1,spprtHipMod,g2592_surgery,z']= -0.41752393067805, ['map,2,spprtHipMod,g2592_surgery,x']= 0.1058824653307, ['map,2,spprtHipMod,g2592_surgery,y']= 0.022496711134946, ['map,2,spprtHipMod,g2592_surgery,z']= 0.0071852561184838, ['map,0,swingKneeMod,g2592_surgery']= -0.085057283995602, ['map,1,swingKneeMod,g2592_surgery']= -0.24009865519611, ['map,2,swingKneeMod,g2592_surgery']= -0.71696415817297, ['map,0,spprtKneeMod,g2592_surgery']= -0.2512617764245, ['map,1,spprtKneeMod,g2592_surgery']= 1.0301138204523, ['map,2,spprtKneeMod,g2592_surgery']= -0.092363243061526, ['map,0,swingAnkleMod,g2592_surgery,x']= 0.76505007386868, ['map,0,swingAnkleMod,g2592_surgery,y']= -0.56668057142796, ['map,0,swingAnkleMod,g2592_surgery,z']= 0.26945005735683, ['map,1,swingAnkleMod,g2592_surgery,x']= 1.4092747028017, ['map,1,swingAnkleMod,g2592_surgery,y']= -0.72955723637048, ['map,1,swingAnkleMod,g2592_surgery,z']= 0.17259282371968, ['map,2,swingAnkleMod,g2592_surgery,x']= 0.089047243978786, ['map,2,swingAnkleMod,g2592_surgery,y']= -0.49377382206773, ['map,2,swingAnkleMod,g2592_surgery,z']= 0.54279419316432, ['map,0,spprtAnkleMod,g2592_surgery,x']= 1.1121519910986, ['map,0,spprtAnkleMod,g2592_surgery,y']= 0.27994881256947, ['map,0,spprtAnkleMod,g2592_surgery,z']= -0.34541087581683, ['map,1,spprtAnkleMod,g2592_surgery,x']= -0.35055619897333, ['map,1,spprtAnkleMod,g2592_surgery,y']= 0.061269562037809, ['map,1,spprtAnkleMod,g2592_surgery,z']= 0.011030102374788, ['map,2,spprtAnkleMod,g2592_surgery,x']= -0.13902068046841, ['map,2,spprtAnkleMod,g2592_surgery,y']= 0.37898945481637, ['map,2,spprtAnkleMod,g2592_surgery,z']= 0.86897505355093, ['map,0,swingMtpMod,g2592_surgery']= -0.15930209502852, ['map,1,swingMtpMod,g2592_surgery']= 0.36369074358267, ['map,2,swingMtpMod,g2592_surgery']= -0.63750407547218, ['map,0,spprtMtpMod,g2592_surgery']= 0.29931830142904, ['map,1,spprtMtpMod,g2592_surgery']= 0.0093167865976485, ['map,2,spprtMtpMod,g2592_surgery']= 0.28901446952427, })


	elseif g_mode=='test_stickleg' then
		useCases.g2592_surgery.aObjWeight = .1*useCases.g2592_surgery.aObjWeight
--		useCases.g2592_surgery.cmaEffortWeight = 1e5
-- todo
-- manually
accumulate({['map,0,swingHipMod,g2592_surgery,x']= 1.0901240077405, ['map,0,swingHipMod,g2592_surgery,y']= -0.910459520938, ['map,0,swingHipMod,g2592_surgery,z']= 0.42652543468564, ['map,1,swingHipMod,g2592_surgery,x']= 1.2048398181976, ['map,1,swingHipMod,g2592_surgery,y']= -0.80999984359897, ['map,1,swingHipMod,g2592_surgery,z']= 0.27982376795835, ['map,2,swingHipMod,g2592_surgery,x']= -0.0032915698940714, ['map,2,swingHipMod,g2592_surgery,y']= 0.22524088482814, ['map,2,swingHipMod,g2592_surgery,z']= -0.40782936959809, ['map,0,spprtHipMod,g2592_surgery,x']= -0.40678818282867, ['map,0,spprtHipMod,g2592_surgery,y']= -0.26862210497108, ['map,0,spprtHipMod,g2592_surgery,z']= 0.13359199269604, ['map,1,spprtHipMod,g2592_surgery,x']= 0.73888308064771, ['map,1,spprtHipMod,g2592_surgery,y']= -0.097600537068923, ['map,1,spprtHipMod,g2592_surgery,z']= -0.41752393067805, ['map,2,spprtHipMod,g2592_surgery,x']= 0.1058824653307, ['map,2,spprtHipMod,g2592_surgery,y']= 0.022496711134946, ['map,2,spprtHipMod,g2592_surgery,z']= 0.0071852561184838, ['map,0,swingKneeMod,g2592_surgery']= -0.085057283995602, ['map,1,swingKneeMod,g2592_surgery']= -0.24009865519611, ['map,2,swingKneeMod,g2592_surgery']= -0.71696415817297, ['map,0,spprtKneeMod,g2592_surgery']= -0.2512617764245, ['map,1,spprtKneeMod,g2592_surgery']= 1.0301138204523, ['map,2,spprtKneeMod,g2592_surgery']= -0.092363243061526, ['map,0,swingAnkleMod,g2592_surgery,x']= 0.76505007386868, ['map,0,swingAnkleMod,g2592_surgery,y']= -0.56668057142796, ['map,0,swingAnkleMod,g2592_surgery,z']= 0.26945005735683, ['map,1,swingAnkleMod,g2592_surgery,x']= 1.4092747028017, ['map,1,swingAnkleMod,g2592_surgery,y']= -0.72955723637048, ['map,1,swingAnkleMod,g2592_surgery,z']= 0.17259282371968, ['map,2,swingAnkleMod,g2592_surgery,x']= 0.089047243978786, ['map,2,swingAnkleMod,g2592_surgery,y']= -0.49377382206773, ['map,2,swingAnkleMod,g2592_surgery,z']= 0.54279419316432, ['map,0,spprtAnkleMod,g2592_surgery,x']= 1.1121519910986, ['map,0,spprtAnkleMod,g2592_surgery,y']= 0.27994881256947, ['map,0,spprtAnkleMod,g2592_surgery,z']= -0.34541087581683, ['map,1,spprtAnkleMod,g2592_surgery,x']= -0.35055619897333, ['map,1,spprtAnkleMod,g2592_surgery,y']= 0.061269562037809, ['map,1,spprtAnkleMod,g2592_surgery,z']= 0.011030102374788, ['map,2,spprtAnkleMod,g2592_surgery,x']= -0.13902068046841, ['map,2,spprtAnkleMod,g2592_surgery,y']= 0.37898945481637, ['map,2,spprtAnkleMod,g2592_surgery,z']= 0.86897505355093, ['map,0,swingMtpMod,g2592_surgery']= -0.15930209502852, ['map,1,swingMtpMod,g2592_surgery']= 0.36369074358267, ['map,2,swingMtpMod,g2592_surgery']= -0.63750407547218, ['map,0,spprtMtpMod,g2592_surgery']= 0.29931830142904, ['map,1,spprtMtpMod,g2592_surgery']= 0.0093167865976485, ['map,2,spprtMtpMod,g2592_surgery']= 0.28901446952427, })

	else
		--elseif g_mode=='age_30' then
		--useCases.g2592_surgery.age = 30
		--useCases.g2592_surgery.cmaEffortWeight = 1e-6
		--useCases.g2592_surgery.weakRatio = .2
		--useCases.g2592_surgery.weakenMuscles=concat_array(R_ankle_pf,  L_ankle_pf)
		--useCases.g2592_surgery.weakenMuscles=hamstrings
		--useCases.g2592_surgery.cmaLFootCFWeight = 1e-6
		--useCases.g2592_surgery.speedMod=vector3(1,0,0)

		if string.find(g_mode, 'cont_')~=nil then
			--iter:233 bestfvever:3.9107901094599e-07
			accumulate({['map,0,swingFootMod,g2592_surgery,x']= -0.19260809813539, ['map,0,swingFootMod,g2592_surgery,y']= -0.2678698527285, ['map,0,swingFootMod,g2592_surgery,z']= 0.39270879747744, ['map,1,swingFootMod,g2592_surgery,x']= 0.29956185318915, ['map,1,swingFootMod,g2592_surgery,y']= 0.084301191558227, ['map,1,swingFootMod,g2592_surgery,z']= -0.0033021297875873, ['map,2,swingFootMod,g2592_surgery,x']= -0.056554082472535, ['map,2,swingFootMod,g2592_surgery,y']= 0.062772658538821, ['map,2,swingFootMod,g2592_surgery,z']= 0.11418129072202, ['map,0,spprtFootMod,g2592_surgery,x']= 0.10207767062676, ['map,0,spprtFootMod,g2592_surgery,y']= -0.13220686186758, ['map,0,spprtFootMod,g2592_surgery,z']= 0.26029167304072, ['map,1,spprtFootMod,g2592_surgery,x']= 0.32699176281187, ['map,1,spprtFootMod,g2592_surgery,y']= 0.16514243689712, ['map,1,spprtFootMod,g2592_surgery,z']= 0.035345947692649, ['map,2,spprtFootMod,g2592_surgery,x']= -0.33451420741516, ['map,2,spprtFootMod,g2592_surgery,y']= -0.42767628178044, ['map,2,spprtFootMod,g2592_surgery,z']= 0.097795024242233, })
			useCases.g2592_surgery.prec=.1
		end

		if string.find(g_mode, 'a.1')~=nil then
			useCases.g2592_surgery.aObjWeight = .1*useCases.g2592_surgery.aObjWeight
		elseif string.find(g_mode, 'a1')~=nil then
			useCases.g2592_surgery.aObjWeight = 1*useCases.g2592_surgery.aObjWeight
		end

		if string.find(g_mode, 'tl.5')~=nil then
			useCases.g2592_surgery.tenLenRatio = .5
		elseif string.find(g_mode, 'tl.6')~=nil then
			useCases.g2592_surgery.tenLenRatio = .6
		elseif string.find(g_mode, 'tl.7')~=nil then
			useCases.g2592_surgery.tenLenRatio = .7
		elseif string.find(g_mode, 'tl.8')~=nil then
			useCases.g2592_surgery.tenLenRatio = .8
		elseif string.find(g_mode, 'tl.9')~=nil then
			useCases.g2592_surgery.tenLenRatio = .9
		end

		if string.find(g_mode, 'tl2.5')~=nil then
			useCases.g2592_surgery.tenLenRatio2 = .5
		elseif string.find(g_mode, 'tl2.6')~=nil then
			useCases.g2592_surgery.tenLenRatio2 = .6
		elseif string.find(g_mode, 'tl2.7')~=nil then
			useCases.g2592_surgery.tenLenRatio2 = .7
		elseif string.find(g_mode, 'tl2.8')~=nil then
			useCases.g2592_surgery.tenLenRatio2 = .8
		elseif string.find(g_mode, 'tl2.9')~=nil then
			useCases.g2592_surgery.tenLenRatio2 = .9
		end

		if string.find(g_mode, 'ol.5')~=nil then
			useCases.g2592_surgery.optLenRatio = .5
		elseif string.find(g_mode, 'ol.6')~=nil then
			useCases.g2592_surgery.optLenRatio = .6
		elseif string.find(g_mode, 'ol.7')~=nil then
			useCases.g2592_surgery.optLenRatio = .7
		elseif string.find(g_mode, 'ol.8')~=nil then
			useCases.g2592_surgery.optLenRatio = .8
		elseif string.find(g_mode, 'ol.9')~=nil then
			useCases.g2592_surgery.optLenRatio = .9
		end

		if string.find(g_mode, 'tlol.5')~=nil then
			useCases.g2592_surgery.tenLenRatio = .5
			useCases.g2592_surgery.optLenRatio = .5
		elseif string.find(g_mode, 'tlol.82')~=nil then
			useCases.g2592_surgery.tenLenRatio = .82
			useCases.g2592_surgery.optLenRatio = .82
		elseif string.find(g_mode, 'tlol.83')~=nil then
			useCases.g2592_surgery.tenLenRatio = .83
			useCases.g2592_surgery.optLenRatio = .83
		elseif string.find(g_mode, 'tlol.84')~=nil then
			useCases.g2592_surgery.tenLenRatio = .84
			useCases.g2592_surgery.optLenRatio = .84
		elseif string.find(g_mode, 'tlol.85')~=nil then
			useCases.g2592_surgery.tenLenRatio = .85
			useCases.g2592_surgery.optLenRatio = .85
		elseif string.find(g_mode, 'tlol.86')~=nil then
			useCases.g2592_surgery.tenLenRatio = .86
			useCases.g2592_surgery.optLenRatio = .86
		elseif string.find(g_mode, 'tlol.87')~=nil then
			useCases.g2592_surgery.tenLenRatio = .87
			useCases.g2592_surgery.optLenRatio = .87
		elseif string.find(g_mode, 'tlol.88')~=nil then
			useCases.g2592_surgery.tenLenRatio = .88
			useCases.g2592_surgery.optLenRatio = .88
		elseif string.find(g_mode, 'tlol.89')~=nil then
			useCases.g2592_surgery.tenLenRatio = .89
			useCases.g2592_surgery.optLenRatio = .89
		elseif string.find(g_mode, 'tlol.8.9')~=nil then
			useCases.g2592_surgery.tenLenRatio = .8
			useCases.g2592_surgery.optLenRatio = .9
		elseif string.find(g_mode, 'tlol.6')~=nil then
			useCases.g2592_surgery.tenLenRatio = .6
			useCases.g2592_surgery.optLenRatio = .6
		elseif string.find(g_mode, 'tlol.7')~=nil then
			useCases.g2592_surgery.tenLenRatio = .7
			useCases.g2592_surgery.optLenRatio = .7
		elseif string.find(g_mode, 'tlol.8')~=nil then
			useCases.g2592_surgery.tenLenRatio = .8
			useCases.g2592_surgery.optLenRatio = .8
		elseif string.find(g_mode, 'tlol.9')~=nil then
			useCases.g2592_surgery.tenLenRatio = .9
			useCases.g2592_surgery.optLenRatio = .9
		end

		if string.find(g_mode, 'tlol2.5')~=nil then
			useCases.g2592_surgery.tenLenRatio2 = .5
			useCases.g2592_surgery.optLenRatio2 = .5
		elseif string.find(g_mode, 'tlol2.82')~=nil then
			useCases.g2592_surgery.tenLenRatio2 = .82
			useCases.g2592_surgery.optLenRatio2 = .82
		elseif string.find(g_mode, 'tlol2.83')~=nil then
			useCases.g2592_surgery.tenLenRatio2 = .83
			useCases.g2592_surgery.optLenRatio2 = .83
		elseif string.find(g_mode, 'tlol2.84')~=nil then
			useCases.g2592_surgery.tenLenRatio2 = .84
			useCases.g2592_surgery.optLenRatio2 = .84
		elseif string.find(g_mode, 'tlol2.85')~=nil then
			useCases.g2592_surgery.tenLenRatio2 = .85
			useCases.g2592_surgery.optLenRatio2 = .85
		elseif string.find(g_mode, 'tlol2.86')~=nil then
			useCases.g2592_surgery.tenLenRatio2 = .86
			useCases.g2592_surgery.optLenRatio2 = .86
		elseif string.find(g_mode, 'tlol2.87')~=nil then
			useCases.g2592_surgery.tenLenRatio2 = .87
			useCases.g2592_surgery.optLenRatio2 = .87
		elseif string.find(g_mode, 'tlol2.88')~=nil then
			useCases.g2592_surgery.tenLenRatio2 = .88
			useCases.g2592_surgery.optLenRatio2 = .88
		elseif string.find(g_mode, 'tlol2.89')~=nil then
			useCases.g2592_surgery.tenLenRatio2 = .89
			useCases.g2592_surgery.optLenRatio2 = .89
		elseif string.find(g_mode, 'tlol2.8.9')~=nil then
			useCases.g2592_surgery.tenLenRatio2 = .8
			useCases.g2592_surgery.optLenRatio2 = .9
		elseif string.find(g_mode, 'tlol2.6')~=nil then
			useCases.g2592_surgery.tenLenRatio2 = .6
			useCases.g2592_surgery.optLenRatio2 = .6
		elseif string.find(g_mode, 'tlol2.7')~=nil then
			useCases.g2592_surgery.tenLenRatio2 = .7
			useCases.g2592_surgery.optLenRatio2 = .7
		elseif string.find(g_mode, 'tlol2.8')~=nil then
			useCases.g2592_surgery.tenLenRatio2 = .8
			useCases.g2592_surgery.optLenRatio2 = .8
		elseif string.find(g_mode, 'tlol2.9')~=nil then
			useCases.g2592_surgery.tenLenRatio2 = .9
			useCases.g2592_surgery.optLenRatio2 = .9
		end

		if string.find(g_mode, 'mf.2')~=nil then
			useCases.g2592_surgery.weakRatio = .2
		elseif string.find(g_mode, 'mf.1')~=nil then
			useCases.g2592_surgery.weakRatio = .1
		end

		if string.find(g_mode, 'hamst_psoas_tlol')~=nil then
			useCases.g2592_surgery.tenLenMuscles=concat_array(hamstrings, psoases)
			useCases.g2592_surgery.optLenMuscles=concat_array(hamstrings, psoases)

		elseif string.find(g_mode, 'biankpf_tl')~=nil then
			useCases.g2592_surgery.tenLenMuscles=concat_array(R_ankle_pf,  L_ankle_pf)

		elseif string.find(g_mode, 'biankpf_ol')~=nil then
			useCases.g2592_surgery.optLenMuscles=concat_array(R_ankle_pf,  L_ankle_pf)

		elseif string.find(g_mode, 'hamst_psoas_tl')~=nil then
			useCases.g2592_surgery.tenLenMuscles=concat_array(hamstrings, psoases)

		elseif string.find(g_mode, 'hamst_psoas_ol')~=nil then
			useCases.g2592_surgery.optLenMuscles=concat_array(hamstrings, psoases)

		elseif string.find(g_mode, 'hamst_tlol')~=nil then
			useCases.g2592_surgery.tenLenMuscles=hamstrings
			useCases.g2592_surgery.optLenMuscles=hamstrings

		elseif string.find(g_mode, 'hamst_tl')~=nil then
			useCases.g2592_surgery.tenLenMuscles=hamstrings

		elseif string.find(g_mode, 'hamst_ol')~=nil then
			useCases.g2592_surgery.optLenMuscles=hamstrings

		elseif string.find(g_mode, 'psoas_tlol')~=nil then
			useCases.g2592_surgery.tenLenMuscles=psoases
			useCases.g2592_surgery.optLenMuscles=psoases

		elseif string.find(g_mode, 'psoas_tl')~=nil then
			useCases.g2592_surgery.tenLenMuscles=psoases

		elseif string.find(g_mode, 'psoas_ol')~=nil then
			useCases.g2592_surgery.optLenMuscles=psoases
		end

		if string.find(g_mode, 'psoas_tl2')~=nil then
			useCases.g2592_surgery.tenLenMuscles2=psoases
		elseif string.find(g_mode, 'psoas_tlol2')~=nil then
			useCases.g2592_surgery.tenLenMuscles2=psoases
			useCases.g2592_surgery.optLenMuscles2=psoases
		end

		if string.find(g_mode, 'ankPF_mf')~=nil then
			useCases.g2592_surgery.weakenMuscles=concat_array(R_ankle_pf,  L_ankle_pf)

		elseif string.find(g_mode, 'ankPF')~=nil then
			useCases.g2592_surgery.weakenMuscles=concat_array(R_ankle_pf,  L_ankle_pf)
			if string.find(g_mode, 'ankPF.15')~=nil then
				useCases.g2592_surgery.weakRatio = .15
			elseif string.find(g_mode, 'ankPF.1')~=nil then
				useCases.g2592_surgery.weakRatio = .1
			elseif string.find(g_mode, 'ankPF.2')~=nil then
				useCases.g2592_surgery.weakRatio = .2
			elseif string.find(g_mode, 'ankPF.3')~=nil then
				useCases.g2592_surgery.weakRatio = .3
			end

		elseif string.find(g_mode, 'ankDF')~=nil then
			useCases.g2592_surgery.weakenMuscles=concat_array(R_ankle_df,  L_ankle_df)
			if string.find(g_mode, 'ankDF.1')~=nil then
				useCases.g2592_surgery.weakRatio = .1
			elseif string.find(g_mode, 'ankDF.01')~=nil then
				useCases.g2592_surgery.weakRatio = .01
			elseif string.find(g_mode, 'ankDF.05')~=nil then
				useCases.g2592_surgery.weakRatio = .05
			elseif string.find(g_mode, 'ankDF.2')~=nil then
				useCases.g2592_surgery.weakRatio = .2
			elseif string.find(g_mode, 'ankDF.3')~=nil then
				useCases.g2592_surgery.weakRatio = .3
			end

		elseif string.find(g_mode, 'ankLPF')~=nil then
			useCases.g2592_surgery.weakenMuscles=L_ankle_pf
			if string.find(g_mode, 'ankLPF.1')~=nil then
				useCases.g2592_surgery.weakRatio = .1
			elseif string.find(g_mode, 'ankLPF.01')~=nil then
				useCases.g2592_surgery.weakRatio = .01
			elseif string.find(g_mode, 'ankLPF.2')~=nil then
				useCases.g2592_surgery.weakRatio = .2
			end

		elseif string.find(g_mode, 'ankLDF')~=nil then
			useCases.g2592_surgery.weakenMuscles=L_ankle_df
			if string.find(g_mode, 'ankLDF.1')~=nil then
				useCases.g2592_surgery.weakRatio = .1
			elseif string.find(g_mode, 'ankLDF.01')~=nil then
				useCases.g2592_surgery.weakRatio = .01
			elseif string.find(g_mode, 'ankLDF.02')~=nil then
				useCases.g2592_surgery.weakRatio = .02
			elseif string.find(g_mode, 'ankLDF.03')~=nil then
				useCases.g2592_surgery.weakRatio = .03
			elseif string.find(g_mode, 'ankLDF.04')~=nil then
				useCases.g2592_surgery.weakRatio = .04
			elseif string.find(g_mode, 'ankLDF.05')~=nil then
				useCases.g2592_surgery.weakRatio = .05
			elseif string.find(g_mode, 'ankLDF.2')~=nil then
				useCases.g2592_surgery.weakRatio = .2
			elseif string.find(g_mode, 'ankLDF.3')~=nil then
				useCases.g2592_surgery.weakRatio = .3
			end

		elseif string.find(g_mode, 'hipAB')~=nil then
			useCases.g2592_surgery.weakenMuscles=concat_array(R_hip_abd, L_hip_abd)
			if string.find(g_mode, 'hipAB.2')~=nil then
				useCases.g2592_surgery.weakRatio = .2
			elseif string.find(g_mode, 'hipAB.15')~=nil then
				useCases.g2592_surgery.weakRatio = .15
			elseif string.find(g_mode, 'hipAB.3')~=nil then
				useCases.g2592_surgery.weakRatio = .3
			elseif string.find(g_mode, 'hipAB.4')~=nil then
				useCases.g2592_surgery.weakRatio = .4
			end

		elseif string.find(g_mode, 'hipLAB')~=nil then
			useCases.g2592_surgery.weakenMuscles=L_hip_abd
			if string.find(g_mode, 'hipLAB.2')~=nil then
				useCases.g2592_surgery.weakRatio = .2
			elseif string.find(g_mode, 'hipLAB.3')~=nil then
				useCases.g2592_surgery.weakRatio = .3
			end

		elseif string.find(g_mode, 'gluts')~=nil then
			useCases.g2592_surgery.weakenMuscles = concat_array(glut_meds,glut_mins)
			if string.find(g_mode, 'gluts.15')~=nil then
				useCases.g2592_surgery.weakRatio = .15
			elseif string.find(g_mode, 'gluts.1')~=nil then
				useCases.g2592_surgery.weakRatio = .1
			elseif string.find(g_mode, 'gluts.2')~=nil then
				useCases.g2592_surgery.weakRatio = .2
			elseif string.find(g_mode, 'gluts.3')~=nil then
				useCases.g2592_surgery.weakRatio = .3
			elseif string.find(g_mode, 'gluts.4')~=nil then
				useCases.g2592_surgery.weakRatio = .4
			elseif string.find(g_mode, 'gluts.5')~=nil then
				useCases.g2592_surgery.weakRatio = .5
			end

		elseif string.find(g_mode, 'lglut')~=nil then
			useCases.g2592_surgery.weakenMuscles = concat_array(lglut_meds,lglut_mins)
			if string.find(g_mode, 'lglut.1')~=nil then
				useCases.g2592_surgery.weakRatio = .1
			elseif string.find(g_mode, 'lglut.2')~=nil then
				useCases.g2592_surgery.weakRatio = .2
			elseif string.find(g_mode, 'lglut.3')~=nil then
				useCases.g2592_surgery.weakRatio = .3
			elseif string.find(g_mode, 'lglut.4')~=nil then
				useCases.g2592_surgery.weakRatio = .4
			elseif string.find(g_mode, 'lglut.5')~=nil then
				useCases.g2592_surgery.weakRatio = .5
			elseif string.find(g_mode, 'lglut.6')~=nil then
				useCases.g2592_surgery.weakRatio = .6
			end

		elseif string.find(g_mode, 'kneeEX')~=nil then
			useCases.g2592_surgery.weakenMuscles=concat_array(R_knee_ext,L_knee_ext)
			if string.find(g_mode, 'kneeEX.05')~=nil then
				useCases.g2592_surgery.weakRatio = .05
			elseif string.find(g_mode, 'kneeEX.15')~=nil then
				useCases.g2592_surgery.weakRatio = .15
			elseif string.find(g_mode, 'kneeEX.1')~=nil then
				useCases.g2592_surgery.weakRatio = .1
			elseif string.find(g_mode, 'kneeEX.2')~=nil then
				useCases.g2592_surgery.weakRatio = .2
			elseif string.find(g_mode, 'kneeEX.3')~=nil then
				useCases.g2592_surgery.weakRatio = .3
			end

		elseif string.find(g_mode, 'kneeLEX')~=nil then
			useCases.g2592_surgery.weakenMuscles=L_knee_ext
			if string.find(g_mode, 'kneeLEX.05')~=nil then
				useCases.g2592_surgery.weakRatio = .05
			elseif string.find(g_mode, 'kneeLEX.15')~=nil then
				useCases.g2592_surgery.weakRatio = .15
			elseif string.find(g_mode, 'kneeLEX.1')~=nil then
				useCases.g2592_surgery.weakRatio = .1
			elseif string.find(g_mode, 'kneeLEX.2')~=nil then
				useCases.g2592_surgery.weakRatio = .2
			end

		elseif string.find(g_mode, 'hipEX')~=nil then
			useCases.g2592_surgery.weakenMuscles=concat_array(R_hip_ext,L_hip_ext)
			if string.find(g_mode, 'hipEX.1')~=nil then
				useCases.g2592_surgery.weakRatio = .1
			elseif string.find(g_mode, 'hipEX.2')~=nil then
				useCases.g2592_surgery.weakRatio = .2
			elseif string.find(g_mode, 'hipEX.3')~=nil then
				useCases.g2592_surgery.weakRatio = .3
			end

		--elseif string.find(g_mode, 'hamst')~=nil then
			--useCases.g2592_surgery.weakenMuscles=hamstrings
			--if string.find(g_mode, 'hamst.1')~=nil then
				--useCases.g2592_surgery.weakRatio = .1
			--elseif string.find(g_mode, 'hamst.2')~=nil then
				--useCases.g2592_surgery.weakRatio = .2
			--elseif string.find(g_mode, 'hamst.3')~=nil then
				--useCases.g2592_surgery.weakRatio = .3
			--end

		elseif string.find(g_mode, 'rf')~=nil then
			useCases.g2592_surgery.weakenMuscles=rect_fems
			if string.find(g_mode, 'rf.1')~=nil then
				useCases.g2592_surgery.weakRatio = .1
			elseif string.find(g_mode, 'rf.2')~=nil then
				useCases.g2592_surgery.weakRatio = .2
				useCases.g2592_surgery.weakenMuscles=rect_fems
			end

		--elseif string.find(g_mode, 'psoas')~=nil then
			--useCases.g2592_surgery.weakenMuscles=psoases
			--if string.find(g_mode, 'psoas.1')~=nil then
				--useCases.g2592_surgery.weakRatio = .1
			--elseif string.find(g_mode, 'psoas.2')~=nil then
				--useCases.g2592_surgery.weakRatio = .2
			--end

		end

		if string.find(g_mode, 'efrtm5')~=nil then
			useCases.g2592_surgery.cmaEffortWeight = 1e-5
		elseif string.find(g_mode, 'efrtm4')~=nil then
			useCases.g2592_surgery.cmaEffortWeight = 1e-4
		elseif string.find(g_mode, 'efrtm3')~=nil then
			useCases.g2592_surgery.cmaEffortWeight = 1e-3
		elseif string.find(g_mode, 'efrtm6')~=nil then
			useCases.g2592_surgery.cmaEffortWeight = 1e-6
		end

		if string.find(g_mode, 'efcim5')~=nil then
			useCases.g2592_surgery.cmaEffortDistWeight = 1e-5
		elseif string.find(g_mode, 'efcim4')~=nil then
			useCases.g2592_surgery.cmaEffortDistWeight = 1e-4
		elseif string.find(g_mode, 'efcim3')~=nil then
			useCases.g2592_surgery.cmaEffortDistWeight = 1e-3
		elseif string.find(g_mode, 'efcim6')~=nil then
			useCases.g2592_surgery.cmaEffortDistWeight = 1e-6
		elseif string.find(g_mode, 'efcim2')~=nil then
			useCases.g2592_surgery.cmaEffortDistWeight = 1e-2
		elseif string.find(g_mode, 'efcim1')~=nil then
			useCases.g2592_surgery.cmaEffortDistWeight = 1e-1
		end

		if string.find(g_mode, 'lfcfm4')~=nil then
			useCases.g2592_surgery.cmaLFootCFWeight = 1e-4
		elseif string.find(g_mode, 'lfcfm5')~=nil then
			useCases.g2592_surgery.cmaLFootCFWeight = 1e-5
		elseif string.find(g_mode, 'lfcfm6')~=nil then
			useCases.g2592_surgery.cmaLFootCFWeight = 1e-6
		elseif string.find(g_mode, 'lfcfm2')~=nil then
			useCases.g2592_surgery.cmaLFootCFWeight = 1e-2
		elseif string.find(g_mode, 'lfcfm3')~=nil then
			useCases.g2592_surgery.cmaLFootCFWeight = 1e-3
		end

		if string.find(g_mode, 'lankpffom4')~=nil then
			useCases.g2592_surgery.cmaLAnkPFFoWeight = 1e-4
		elseif string.find(g_mode, 'lankpffom5')~=nil then
			useCases.g2592_surgery.cmaLAnkPFFoWeight = 1e-5
		elseif string.find(g_mode, 'lankpffom6')~=nil then
			useCases.g2592_surgery.cmaLAnkPFFoWeight = 1e-6
		elseif string.find(g_mode, 'lankpffom3')~=nil then
			useCases.g2592_surgery.cmaLAnkPFFoWeight = 1e-3
		end

		if string.find(g_mode, 'lrankpffom4')~=nil then
			useCases.g2592_surgery.cmaLAnkPFFoWeight = 1e-4
			useCases.g2592_surgery.cmaRAnkPFFoWeight = 1e-4
		elseif string.find(g_mode, 'lrankpffom5')~=nil then
			useCases.g2592_surgery.cmaLAnkPFFoWeight = 1e-5
			useCases.g2592_surgery.cmaRAnkPFFoWeight = 1e-5
		elseif string.find(g_mode, 'lrankpffom6')~=nil then
			useCases.g2592_surgery.cmaLAnkPFFoWeight = 1e-6
			useCases.g2592_surgery.cmaRAnkPFFoWeight = 1e-6
		elseif string.find(g_mode, 'lrankpffom3')~=nil then
			useCases.g2592_surgery.cmaLAnkPFFoWeight = 1e-3
			useCases.g2592_surgery.cmaRAnkPFFoWeight = 1e-3
		end

		if string.find(g_mode, 'clmpm5')~=nil then
			useCases.g2592_surgery.cmaPoseClamp = 1e-5
		elseif string.find(g_mode, 'clmpm4')~=nil then
			useCases.g2592_surgery.cmaPoseClamp = 1e-4
		elseif string.find(g_mode, 'clmpm3')~=nil then
			useCases.g2592_surgery.cmaPoseClamp = 1e-3
		elseif string.find(g_mode, 'clmpm6')~=nil then
			useCases.g2592_surgery.cmaPoseClamp = 1e-6
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
			useCases.g2592_surgery.age = 30
		elseif string.find(g_mode, 'age50')~=nil then
			useCases.g2592_surgery.age = 50
		elseif string.find(g_mode, 'age70')~=nil then
			useCases.g2592_surgery.age = 70
		end

		if string.find(g_mode, 'a.1_hamst_psoas_tl.8_ankPF_mf.2_push')~=nil then
			--iter:179 bestfvever:1.6876442520237e-06
			accumulate({['map,0,swingFootMod,g2592_surgery,x']= -0.086404799999669, ['map,0,swingFootMod,g2592_surgery,y']= 0.19240951905766, ['map,0,swingFootMod,g2592_surgery,z']= 0.55579676273367, ['map,1,swingFootMod,g2592_surgery,x']= 0.66963538731804, ['map,1,swingFootMod,g2592_surgery,y']= 0.17855220520046, ['map,1,swingFootMod,g2592_surgery,z']= -0.07063979441948, ['map,2,swingFootMod,g2592_surgery,x']= -0.52133779009622, ['map,2,swingFootMod,g2592_surgery,y']= 0.27912539029751, ['map,2,swingFootMod,g2592_surgery,z']= 0.016619112095564, ['map,0,spprtFootMod,g2592_surgery,x']= 0.050424489613135, ['map,0,spprtFootMod,g2592_surgery,y']= -1.0259768713766, ['map,0,spprtFootMod,g2592_surgery,z']= -0.095664952002717, ['map,1,spprtFootMod,g2592_surgery,x']= -0.031728504928429, ['map,1,spprtFootMod,g2592_surgery,y']= -0.59243930537485, ['map,1,spprtFootMod,g2592_surgery,z']= -0.37463741424218, ['map,2,spprtFootMod,g2592_surgery,x']= -0.12630034785026, ['map,2,spprtFootMod,g2592_surgery,y']= -0.40255332181395, ['map,2,spprtFootMod,g2592_surgery,z']= 0.47607683649369, })

			useCases.g2592_surgery.impulse = {
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
			useCases.g2592_surgery.cmaLRKneeLimitWeight = 1e-4
			useCases.g2592_surgery.cmaLRKneeMaxExtAng = -30
			useCases.g2592_surgery.cmaLRHipLimitWeight = useCases.g2592_surgery.cmaLRKneeLimitWeight
			useCases.g2592_surgery.cmaLRHipMaxExtAng = -10
			--iter:148 bestfvever:0.0014072872775303
			accumulate({['map,0,swingFootMod,g2592_surgery,x']= -0.27156173633567, ['map,0,swingFootMod,g2592_surgery,y']= -0.60004513627203, ['map,0,swingFootMod,g2592_surgery,z']= 0.76081448523497, ['map,1,swingFootMod,g2592_surgery,x']= 0.052136373274667, ['map,1,swingFootMod,g2592_surgery,y']= 0.28772566194875, ['map,1,swingFootMod,g2592_surgery,z']= -0.19644526788733, ['map,2,swingFootMod,g2592_surgery,x']= -0.17368811977695, ['map,2,swingFootMod,g2592_surgery,y']= 0.76298531697562, ['map,2,swingFootMod,g2592_surgery,z']= 0.021349096590465, ['map,0,spprtFootMod,g2592_surgery,x']= 0.076437611374153, ['map,0,spprtFootMod,g2592_surgery,y']= -0.53423548482621, ['map,0,spprtFootMod,g2592_surgery,z']= 0.11844190649319, ['map,1,spprtFootMod,g2592_surgery,x']= -0.12084682580061, ['map,1,spprtFootMod,g2592_surgery,y']= 0.67940218967468, ['map,1,spprtFootMod,g2592_surgery,z']= 0.036022652887122, ['map,2,spprtFootMod,g2592_surgery,x']= 0.20255125842068, ['map,2,spprtFootMod,g2592_surgery,y']= -0.75505665803657, ['map,2,spprtFootMod,g2592_surgery,z']= 0.24844297477109, })

		elseif g_mode=='a.1_ankPF.2_efcim5_h.2' then
			accumulate({
				['map,0,swingFootMod,g2592_surgery,y']     = 0.2,
				['map,1,swingFootMod,g2592_surgery,y']     = 0.2,
				['map,2,swingFootMod,g2592_surgery,y']     = 0.2,
				['map,0,spprtFootMod,g2592_surgery,y']     = 0.2,
				['map,1,spprtFootMod,g2592_surgery,y']     = 0.2,
				['map,2,spprtFootMod,g2592_surgery,y']     = 0.2,
			})
			useCases.unmapControlParam(useCases.g2592_surgery)
		end
	end
	-------------------------------------------------

	useCases.g2592_surgery.controlParam['useCases,g2592_surgery,aObjWeight'] = useCases.g2592_surgery.aObjWeight
	useCases.unmapControlParam(useCases.g2592_surgery)
end


