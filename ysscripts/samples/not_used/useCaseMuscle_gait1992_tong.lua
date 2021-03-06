package.path=package.path..";../Samples/ysscripts/lib/?.lua" --;"..package.path
require("utilfunc")

package.path=package.path..";../Samples/ysscripts/samples/?.lua" --;"..package.path
require("useCaseMuscle_gait1992_6steps")

useCases.gait1992_tong=deepCopyTable(useCases.gait1992_6steps)

useCases.gait1992_tong.endSegW=10

useCases.gait1992_tong.modelName='gait1992_tong'
useCases.gait1992_tong.grpName='gait1992_tong'
useCases.gait1992_tong.grpNames={'gait1992_tong'}


useCases.gait1992_tong.k_p_ID=120
useCases.gait1992_tong.k_d_ID=2.*math.sqrt(120)
--useCases.gait1992_tong.ddqObjWeight=1000000
useCases.gait1992_tong.ddqObjWeight=10000000

--useCases.gait1992_tong.actuationType = ActuationType.tau
--useCases.gait1992_tong.actuationType = ActuationType.ft
--useCases.gait1992_tong.actuationType = ActuationType.a
useCases.gait1992_tong.actuationType = ActuationType.u

useCases.gait1992_tong.tauObjWeight=1
useCases.gait1992_tong.ftObjWeight=1
useCases.gait1992_tong.aObjWeight=10
useCases.gait1992_tong.lambdaObjWeight=10

--useCases.gait1992_tong.EEobjWeight=60000000
useCases.gait1992_tong.EEobjWeight=33300000
--
useCases.gait1992_tong.EEobjWeightAngular=60000000

--useCases.gait1992_tong.momentumWeight=8000000
useCases.gait1992_tong.momentumWeight=0

--useCases.gait1992_tong.headControlWeight=0
useCases.gait1992_tong.headControlWeight=36000000
--useCases.gait1992_tong.headControlWeight=20000000 --ys

useCases.gait1992_tong.initMsclForceScale=4

-- the initial pendControlParam and pendOptimizationPath can be automatically generated by using createInitialPendControlParam.lua
	pendControlParam={['keyframe,0,pendDesiredVel,gait1992_tong,l,z']=0,['keyframe,0,pendDesiredVel,gait1992_tong,l,x']=1,['keyframe,0,pendDesiredVel,gait1992_tong,r,z']=0,['keyframe,0,pendDesiredVel,gait1992_tong,r,x']=1,['keyframe,0,pendDesiredVel,gait1992_tong,L1,z']=0,['keyframe,0,pendDesiredVel,gait1992_tong,L1,x']=1,['keyframe,0,pendDesiredVel,gait1992_tong,R1,z']=0,['keyframe,0,pendDesiredVel,gait1992_tong,R1,x']=1,['keyframe,0,pendDesiredVel,gait1992_tong,L2,z']=0,['keyframe,0,pendDesiredVel,gait1992_tong,L2,x']=1,['keyframe,0,pendDesiredVel,ignore,0,z']=0,['keyframe,0,pendDesiredVel,ignore,0,x']=1,['keyframe,0,pendDesiredVel,ignore,1,z']=0,['keyframe,0,pendDesiredVel,ignore,1,x']=1,
['keyframe,0,pendDesiredVel,ignore,0,x']=0.67397424028586, ['keyframe,0,pendDesiredVel,ignore,0,z']=-0.0074665069825716, ['keyframe,0,pendDesiredVel,gait1992_tong,l,x']=1.0870479602205, ['keyframe,0,pendDesiredVel,gait1992_tong,l,z']=0.07024324142064, ['keyframe,0,pendDesiredVel,gait1992_tong,r,x']=1.210684865058, ['keyframe,0,pendDesiredVel,gait1992_tong,r,z']=0.10197338160236, ['keyframe,0,pendDesiredVel,gait1992_tong,L1,x']=1.1962961005512, ['keyframe,0,pendDesiredVel,gait1992_tong,L1,z']=0.03865663624365, ['keyframe,0,pendDesiredVel,gait1992_tong,R1,x']=1.1351762847156, ['keyframe,0,pendDesiredVel,gait1992_tong,R1,z']=0.03853403534704, ['keyframe,0,pendDesiredVel,gait1992_tong,L2,x']=1.0323519384173, ['keyframe,0,pendDesiredVel,gait1992_tong,L2,z']=0.0084288611334129, ['keyframe,0,pendDesiredVel,ignore,1,x']=0.99600099116465, ['keyframe,0,pendDesiredVel,ignore,1,z']=-0.001430496053799, 
['keyframe,0,pendDesiredVel,ignore,0,x']=0.50413622445006, ['keyframe,0,pendDesiredVel,ignore,0,z']=-0.05049302871811, ['keyframe,0,pendDesiredVel,gait1992_tong,l,x']=1.4142621836795, ['keyframe,0,pendDesiredVel,gait1992_tong,l,z']=0.20268192708886, ['keyframe,0,pendDesiredVel,gait1992_tong,r,x']=1.3452082385225, ['keyframe,0,pendDesiredVel,gait1992_tong,r,z']=0.1517419819397, ['keyframe,0,pendDesiredVel,gait1992_tong,L1,x']=1.1497993301781, ['keyframe,0,pendDesiredVel,gait1992_tong,L1,z']=0.011127796803486, ['keyframe,0,pendDesiredVel,gait1992_tong,R1,x']=0.95350408914623, ['keyframe,0,pendDesiredVel,gait1992_tong,R1,z']=-0.021106557516788, ['keyframe,0,pendDesiredVel,gait1992_tong,L2,x']=0.85590007662266, ['keyframe,0,pendDesiredVel,gait1992_tong,L2,z']=0.019184374821157, ['keyframe,0,pendDesiredVel,ignore,1,x']=0.9663149455938, ['keyframe,0,pendDesiredVel,ignore,1,z']=-0.0086197735010652, 
['keyframe,0,pendDesiredVel,ignore,0,x']=0.50404682631138, ['keyframe,0,pendDesiredVel,ignore,0,z']=-0.050514940678028, ['keyframe,0,pendDesiredVel,gait1992_tong,l,x']=1.4142908487564, ['keyframe,0,pendDesiredVel,gait1992_tong,l,z']=0.20269391413581, ['keyframe,0,pendDesiredVel,gait1992_tong,r,x']=1.345229178375, ['keyframe,0,pendDesiredVel,gait1992_tong,r,z']=0.15174731023168, ['keyframe,0,pendDesiredVel,gait1992_tong,L1,x']=1.1498256706975, ['keyframe,0,pendDesiredVel,gait1992_tong,L1,z']=0.011127832149238, ['keyframe,0,pendDesiredVel,gait1992_tong,R1,x']=0.95352733720468, ['keyframe,0,pendDesiredVel,gait1992_tong,R1,z']=-0.021102666296777, ['keyframe,0,pendDesiredVel,gait1992_tong,L2,x']=0.85589768364064, ['keyframe,0,pendDesiredVel,gait1992_tong,L2,z']=0.019190599831802, ['keyframe,0,pendDesiredVel,ignore,1,x']=0.96631137589109, ['keyframe,0,pendDesiredVel,ignore,1,z']=-0.008620319987034, 
['keyframe,0,pendDesiredVel,ignore,0,x']=0.50390418098759, ['keyframe,0,pendDesiredVel,ignore,0,z']=-0.050549859516892, ['keyframe,0,pendDesiredVel,gait1992_tong,l,x']=1.4143374011315, ['keyframe,0,pendDesiredVel,gait1992_tong,l,z']=0.2027133765509, ['keyframe,0,pendDesiredVel,gait1992_tong,r,x']=1.3452631636312, ['keyframe,0,pendDesiredVel,gait1992_tong,r,z']=0.15175603807387, ['keyframe,0,pendDesiredVel,gait1992_tong,L1,x']=1.1498682226361, ['keyframe,0,pendDesiredVel,gait1992_tong,L1,z']=0.011127945716879, ['keyframe,0,pendDesiredVel,gait1992_tong,R1,x']=0.95356483309342, ['keyframe,0,pendDesiredVel,gait1992_tong,R1,z']=-0.021096362557798, ['keyframe,0,pendDesiredVel,gait1992_tong,L2,x']=0.85589386928721, ['keyframe,0,pendDesiredVel,gait1992_tong,L2,z']=0.019200621799857, ['keyframe,0,pendDesiredVel,ignore,1,x']=0.96630563622305, ['keyframe,0,pendDesiredVel,ignore,1,z']=-0.0086211979430672, 
['keyframe,0,pendDesiredVel,ignore,0,x']=0.38159450212516, ['keyframe,0,pendDesiredVel,ignore,0,z']=-0.052708303002759, ['keyframe,0,pendDesiredVel,gait1992_tong,l,x']=1.854845943837, ['keyframe,0,pendDesiredVel,gait1992_tong,l,z']=0.3472581596977, ['keyframe,0,pendDesiredVel,gait1992_tong,r,x']=1.3736132116719, ['keyframe,0,pendDesiredVel,gait1992_tong,r,z']=0.14136332477824, 
['keyframe,0,pendDesiredVel,ignore,0,x']=0.38542263433817, ['keyframe,0,pendDesiredVel,ignore,0,z']=-0.045134733001122, ['keyframe,0,pendDesiredVel,gait1992_tong,l,x']=1.8301027676867, ['keyframe,0,pendDesiredVel,gait1992_tong,l,z']=0.33711303141227, ['keyframe,0,pendDesiredVel,gait1992_tong,r,x']=1.3796814777883, ['keyframe,0,pendDesiredVel,gait1992_tong,r,z']=0.15145052868814, 
['keyframe,0,pendDesiredVel,gait1992_tong,l,x']=1.7305776119094, ['keyframe,0,pendDesiredVel,gait1992_tong,l,z']=0.25538064590366, ['keyframe,0,pendDesiredVel,gait1992_tong,r,x']=1.2410661200316, ['keyframe,0,pendDesiredVel,gait1992_tong,r,z']=0.092299655249861, ['keyframe,0,pendDesiredVel,gait1992_tong,L1,x']=1.1511820918024, ['keyframe,0,pendDesiredVel,gait1992_tong,L1,z']=0.016150117032203, ['keyframe,0,pendDesiredVel,gait1992_tong,R1,x']=1.0098735198416, ['keyframe,0,pendDesiredVel,gait1992_tong,R1,z']=0.0057170216311841, ['keyframe,0,pendDesiredVel,gait1992_tong,L2,x']=0.82990395462959, ['keyframe,0,pendDesiredVel,gait1992_tong,L2,z']=0.056722578636763, ['keyframe,0,pendDesiredVel,ignore,1,x']=0.94754533408323, ['keyframe,0,pendDesiredVel,ignore,1,z']=-0.01126038851346, 
['keyframe,0,pendDesiredVel,gait1992_tong,l,x']=1.7305998709824, ['keyframe,0,pendDesiredVel,gait1992_tong,l,z']=0.25527996963598, ['keyframe,0,pendDesiredVel,gait1992_tong,r,x']=1.2409415653836, ['keyframe,0,pendDesiredVel,gait1992_tong,r,z']=0.092159323153746, ['keyframe,0,pendDesiredVel,gait1992_tong,L1,x']=1.1511456386732, ['keyframe,0,pendDesiredVel,gait1992_tong,L1,z']=0.016050815884303, ['keyframe,0,pendDesiredVel,gait1992_tong,R1,x']=1.0098701157038, ['keyframe,0,pendDesiredVel,gait1992_tong,R1,z']=0.0056243081237624, ['keyframe,0,pendDesiredVel,gait1992_tong,L2,x']=0.82980798757505, ['keyframe,0,pendDesiredVel,gait1992_tong,L2,z']=0.0567217231482, ['keyframe,0,pendDesiredVel,ignore,1,x']=0.94751338642262, ['keyframe,0,pendDesiredVel,ignore,1,z']=-0.011270092695708, 
}

useCases.gait1992_tong.pendControlParam=pendControlParam
--useCases.gait1992_tong.measureOptCost=useCases.measureOptCost


useCases.gait1992_tong.keyframes={
	pendDesiredVel={numKey=1, default=vector3(1,0,0)},
	swingFootForce={numKey=3, default=vector3(0,10,0)},
	footLmod={numKey=3, numKeyFrom='key', default=vector3(0,0,0)},
	footRmod={numKey=3, numKeyFrom='key', default=vector3(0,0,0)},
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

useCases.gait1992_tong.pendOptimizationPath=
{
	firstFrames={1,130,195,270,331,404,470,542,},
	segments={'ignore,0','gait1992_tong,l','gait1992_tong,r','gait1992_tong,L1','gait1992_tong,R1','gait1992_tong,L2','ignore,1'},
}

useCases.gait1992_tong.segmentations=
{
	gait1992_tong={
	firstFrames={1,130,195,270,331,404,470,542,},
		--        (blank)    L    R    L     R     L     (blank)
		names={             'l', 'r', 'L1', 'R1', 'L2',  },
		swingL={             0,   1,   0,    1,    0,    },
		swingR={             1,   0,   1,    0,    1,    },
		footRefL= "convertFromSwingL",
		footRefR= "convertFromSwingR",
		usePositionControl=false,
	},
}
useCases.gait1992_tong.graphParam={
	gait1992_tong={
		seg={            'l',  'r',  'L1', 'R1','L'},
		num_key={         3,    3,    3  ,  4,  4},
		key_first={       0,    0,    0  ,  0,  0},
		key_last={       'r', 'L1',  'R1', 'L', 'R1'},

	},
}
useCases.gait1992_tong.graph={
	{
		"addInterpolatedSegment",
		grpName="gait1992_tong",
		name="L",
		seg0={"gait1992_tong", "L2"},
		seg1={"gait1992_tong", "L1"},
		startWeight=0, endWeight=1
	},
	{"connectMulti", "gait1992_tong", "l", "r", "L1", "R1", "L", "R1"},
	{"initialSegment", "gait1992_tong", "l"}
}
useCases.gait1992_tong.segNames={
	"l", "r", "L1", 'R1','L',
}

do 
	useCases.gait1992_tong.controlParam={
		['useCases,gait1992_tong,COMobjWeight']=0, 
		['useCases,gait1992_tong,conservativeW']=1, 
		['useCases,gait1992_tong,contactMargin']=0.01, 
		['useCases,gait1992_tong,dotMomentumScale']=0.3, 
		['useCases,gait1992_tong,excludeRoot']=true, 
		['useCases,gait1992_tong,headControlWeight']=0, 
		['useCases,gait1992_tong,k_d_HEAD']=14, 
		['useCases,gait1992_tong,k_p_HEAD']=0, 
		['useCases,gait1992_tong,k_d_EE']=24, 
		['useCases,gait1992_tong,k_d_momentum']=10, 
		['useCases,gait1992_tong,k_p_EE']=120, 
		--['useCases,run,invAccConeCoef']=1, 
		--['useCases,run,invFricCoef']=0.8, 

		['useCases,gait1992_tong,maxPenetratingVel']=0, 
		['useCases,gait1992_tong,momentumThr']=50, 
		['useCases,gait1992_tong,noComvelDependentFootAdjustment']=true, 
		['useCases,gait1992_tong,noIntersectionPrevenction']=true, 
		['useCases,gait1992_tong,numericalDerivDmot']=true, 
		['useCases,gait1992_tong,perClassContactMargin']=1, 
		['useCases,gait1992_tong,turnGain']=10, 
		['useCases,gait1992_tong,velMarginOffset']=0, 

		['useCases,gait1992_tong,actuationType']	=useCases.gait1992_tong.actuationType,
		['useCases,gait1992_tong,k_p_ID']			=useCases.gait1992_tong.k_p_ID,
		['useCases,gait1992_tong,k_d_ID']			=useCases.gait1992_tong.k_d_ID,
		['useCases,gait1992_tong,ddqObjWeight']	=useCases.gait1992_tong.ddqObjWeight,
		['useCases,gait1992_tong,tauObjWeight']	=useCases.gait1992_tong.tauObjWeight,
		['useCases,gait1992_tong,ftObjWeight']	=useCases.gait1992_tong.ftObjWeight,
		['useCases,gait1992_tong,aObjWeight']		=useCases.gait1992_tong.aObjWeight,
		['useCases,gait1992_tong,lambdaObjWeight']=useCases.gait1992_tong.lambdaObjWeight,
		['useCases,gait1992_tong,EEobjWeight']        = useCases.gait1992_tong.EEobjWeight,
		['useCases,gait1992_tong,EEobjWeightAngular'] = useCases.gait1992_tong.EEobjWeightAngular,
		['useCases,gait1992_tong,momentumWeight']     = useCases.gait1992_tong.momentumWeight,
		['useCases,gait1992_tong,headControlWeight']     = useCases.gait1992_tong.headControlWeight,
		['useCases,gait1992_tong,tauMax']			=useCases.gait1992_tong.tauMax,
		['useCases,gait1992_tong,ftMax']			=useCases.gait1992_tong.ftMax,
	}

	local function accumulate(cp_mod)
		local useCase=useCases.gait1992_tong
		useCases.accumulate(useCase, cp_mod)
	end

	useCases.gait1992_tong.noCOMjoint=true
	useCases.gait1992_tong.spprtImpFromImp=true
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

	useCases.gait1992_tong.useBulletColdet=true

	--accumulate({
		--['keyframe,0,footRmod,gait1992_tong,l,y'] =0.2,
		--['keyframe,1,footRmod,gait1992_tong,l,y'] =0.5,
		--['keyframe,0,footLmod,gait1992_tong,r,y'] =0.3,
		--['keyframe,1,footLmod,gait1992_tong,r,y'] =0.6,
		--['keyframe,0,footRmod,gait1992_tong,L1,y']=0.3,
		--['keyframe,1,footRmod,gait1992_tong,L1,y']=0.6,
		--['map,0,swingFootMod,gait1992_tong,y']= 0.3,
		--['map,1,swingFootMod,gait1992_tong,y']= 0.6,
		--['map,2,swingFootMod,gait1992_tong,y']= 0.6,

		--['keyframe,0,footLmod,gait1992_tong,l,y'] =-0.1,
		--['keyframe,1,footLmod,gait1992_tong,l,y'] =-0.1,
		--['keyframe,0,footRmod,gait1992_tong,r,y'] =-0.1,
		--['keyframe,1,footRmod,gait1992_tong,r,y'] =-0.1,
		--['keyframe,0,footLmod,gait1992_tong,L1,y']=-0.1,
		--['keyframe,1,footLmod,gait1992_tong,L1,y']=-0.1,
		--['map,0,spprtFootMod,gait1992_tong,y']= -0.1,
		--['map,1,spprtFootMod,gait1992_tong,y']= -0.1,
		--['map,2,spprtFootMod,gait1992_tong,y']= -0.1,
	--})
	accumulate({
		['keyframe,0,footRmod,gait1992_tong,l,y']  = 0.,
		['keyframe,1,footRmod,gait1992_tong,l,y']  = 0.,
		['keyframe,0,footLmod,gait1992_tong,r,y']  = 0.,
		['keyframe,1,footLmod,gait1992_tong,r,y']  = 0.,
		['keyframe,0,footRmod,gait1992_tong,L1,y'] = 0.,
		['keyframe,1,footRmod,gait1992_tong,L1,y'] = 0.,
		['map,0,swingFootMod,gait1992_tong,y']     = 0.,
		['map,1,swingFootMod,gait1992_tong,y']     = 0.,
		['map,2,swingFootMod,gait1992_tong,y']     = 0.,

		['keyframe,0,footLmod,gait1992_tong,l,y']  = 0.,
		['keyframe,1,footLmod,gait1992_tong,l,y']  = 0.,
		['keyframe,0,footRmod,gait1992_tong,r,y']  = 0.,
		['keyframe,1,footRmod,gait1992_tong,r,y']  = 0.,
		['keyframe,0,footLmod,gait1992_tong,L1,y'] = 0.,
		['keyframe,1,footLmod,gait1992_tong,L1,y'] = 0.,
		['map,0,spprtFootMod,gait1992_tong,y']     = 0.,
		['map,1,spprtFootMod,gait1992_tong,y']     = 0.,
		['map,2,spprtFootMod,gait1992_tong,y']     = 0.,
	})

--	------------------New optimization-------- [1e+100,-1e+100] -> oeval:90527.432395601 dim:18}) -- skipped oeval:90527.432395601 
--0.000 --0.000 --0.000 --0.000 --0.000 --0.000 --0.000 --0.000 --0.000 --0.000 
--Fri Dec 27 05:16:24 2013 setStage2 startSeg:1 endSeg:10 : [2.5079924996948e-06,107948.23719974] -> oeval:2.381396396676e-06 dim:6}) -- skipped oeval:2.381396396676e-06 

accumulate({ ['map,0,swingFootMod,gait1992_tong,x']= 0.28144762529357, ['map,0,swingFootMod,gait1992_tong,y']= 0.57366102028254, ['map,0,swingFootMod,gait1992_tong,z']= 0.50207066174743, ['map,1,swingFootMod,gait1992_tong,x']= -0.58086369129171, ['map,1,swingFootMod,gait1992_tong,y']= -0.14337491636872, ['map,1,swingFootMod,gait1992_tong,z']= -0.17487684394393, ['map,2,swingFootMod,gait1992_tong,x']= 0.48819895394125, ['map,2,swingFootMod,gait1992_tong,y']= -0.10426273476283, ['map,2,swingFootMod,gait1992_tong,z']= 0.41235244050313, ['map,0,spprtFootMod,gait1992_tong,x']= 0.084739325594179, ['map,0,spprtFootMod,gait1992_tong,y']= -1.1460803226064, ['map,0,spprtFootMod,gait1992_tong,z']= -0.1246325998813, ['map,1,spprtFootMod,gait1992_tong,x']= 0.63604116038347, ['map,1,spprtFootMod,gait1992_tong,y']= -0.23776638828331, ['map,1,spprtFootMod,gait1992_tong,z']= 0.099524994742026, ['map,2,spprtFootMod,gait1992_tong,x']= 0.027638228010936, ['map,2,spprtFootMod,gait1992_tong,y']= 0.36188241266017, ['map,2,spprtFootMod,gait1992_tong,z']= 0.1952664193425, })
--0.995 --0.980 --0.980 --0.980 --0.980 --0.980 --0.980 --0.980 --0.980 --0.980 
--Fri Dec 27 07:43:57 2013 setStage3 startSeg:1 endSeg:10 : [2.3741597858738e-06,2.952779122892e-06] -> oeval:2.3328566621491e-06 dim:6}) -- skipped oeval:2.3328566621491e-06 

accumulate({ ['map,0,swingFootMod,gait1992_tong,z']= 0.51949084188383, ['map,1,swingFootMod,gait1992_tong,z']= -0.1360355194822, ['map,2,swingFootMod,gait1992_tong,z']= 0.3889134025581, ['map,0,spprtFootMod,gait1992_tong,z']= -0.10592677904664, ['map,1,spprtFootMod,gait1992_tong,z']= 0.099564107569088, ['map,2,spprtFootMod,gait1992_tong,z']= 0.20730473595416, })
--1.049 --0.992 --0.992 --0.989 --0.989 --0.989 --0.989 --0.988 --0.988 --0.988 
--Fri Dec 27 10:12:03 2013 setStage4 startSeg:1 endSeg:10 : [2.3055592842591e-06,2.4919344112465e-06] -> oeval:2.3049040151247e-06 dim:6}) -- skipped oeval:2.3049040151247e-06 

accumulate({ ['map,0,swingFootMod,gait1992_tong,x']= 0.29110382068752, ['map,1,swingFootMod,gait1992_tong,x']= -0.59328459107073, ['map,2,swingFootMod,gait1992_tong,x']= 0.47169138428371, ['map,0,spprtFootMod,gait1992_tong,x']= 0.092989536333288, ['map,1,spprtFootMod,gait1992_tong,x']= 0.64966688053426, ['map,2,spprtFootMod,gait1992_tong,x']= -0.036830182567466, })
--1.003 --0.996 --0.995 --0.993 --0.992 --0.987 --0.987 --0.987 --0.987 --0.987 
--Fri Dec 27 12:40:23 2013 setStage5 startSeg:1 endSeg:10 : [2.2767470179443e-06,2.3138447695319e-06] -> oeval:2.2738234967838e-06 dim:18}) -- skipped oeval:2.2738234967838e-06 

accumulate({ ['map,0,swingFootMod,gait1992_tong,y']= 0.49839651410515, ['map,1,swingFootMod,gait1992_tong,y']= -0.15598952521317, ['map,2,swingFootMod,gait1992_tong,y']= -0.081763265786775, ['map,0,spprtFootMod,gait1992_tong,y']= -1.0946047562373, ['map,1,spprtFootMod,gait1992_tong,y']= -0.21414599385514, ['map,2,spprtFootMod,gait1992_tong,y']= 0.39954062915561, })
--1.037 
	useCases.unmapControlParam(useCases.gait1992_tong)
	
end


