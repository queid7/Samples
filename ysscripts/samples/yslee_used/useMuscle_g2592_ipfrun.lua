package.path=package.path..";../Samples/ysscripts/lib/?.lua" --;"..package.path
require("utilfunc")

package.path=package.path..";../Samples/ysscripts/samples/?.lua" --;"..package.path
require("useMuscle_g2592_gait")


useCases.g2592_ipfrun=deepCopyTable(useCases.g2592_gait)

useCases.g2592_ipfrun.optimizerMethod='Optimizer.methods.CMAes_ys'

useCases.g2592_ipfrun.endSegW=80
useCases.g2592_ipfrun.prec=.2

useCases.g2592_ipfrun.modelName='g2592_ipfrun'
useCases.g2592_ipfrun.grpName='g2592_ipfrun'
useCases.g2592_ipfrun.grpNames={'g2592_ipfrun'}


useCases.g2592_ipfrun.ddqObjWeight=10000000*(25-7)
useCases.g2592_ipfrun.aObjWeight=100000000*62
useCases.g2592_ipfrun.lambdaObjWeight=100
useCases.g2592_ipfrun.EEobjWeight=10000000
useCases.g2592_ipfrun.EEobjWeightAngular=100
useCases.g2592_ipfrun.momentumWeight=10000000

useCases.g2592_ipfrun.lambdaObjWeight=1000000

--useCases.full_same.EEobjWeight=1000000


-- the initial pendControlParam and pendOptimizationPath can be automatically generated by using createInitialPendControlParam.lua
useCases.g2592_ipfrun.pendControlParam=
{
['keyframe,0,pendDesiredVel,g2592_ipfrun,l,z']=0,['keyframe,0,pendDesiredVel,g2592_ipfrun,l,x']=1,['keyframe,0,pendDesiredVel,g2592_ipfrun,r,z']=0,['keyframe,0,pendDesiredVel,g2592_ipfrun,r,x']=1,['keyframe,0,pendDesiredVel,g2592_ipfrun,L1,z']=0,['keyframe,0,pendDesiredVel,g2592_ipfrun,L1,x']=1,['keyframe,0,pendDesiredVel,g2592_ipfrun,R1,z']=0,['keyframe,0,pendDesiredVel,g2592_ipfrun,R1,x']=1,['keyframe,0,pendDesiredVel,g2592_ipfrun,L2,z']=0,['keyframe,0,pendDesiredVel,g2592_ipfrun,L2,x']=1,['keyframe,0,pendDesiredVel,ignore,0,z']=0,['keyframe,0,pendDesiredVel,ignore,0,x']=1,['keyframe,0,pendDesiredVel,ignore,1,z']=0,['keyframe,0,pendDesiredVel,ignore,1,x']=1,
['keyframe,0,pendDesiredVel,ignore,0,x']=0.27454274916781, ['keyframe,0,pendDesiredVel,ignore,0,z']=-0.53368795449817, ['keyframe,0,pendDesiredVel,g2592_ipfrun,l,x']=0.26123059653128, ['keyframe,0,pendDesiredVel,g2592_ipfrun,l,z']=-0.53457953203496, ['keyframe,0,pendDesiredVel,g2592_ipfrun,r,x']=0.25674745821108, ['keyframe,0,pendDesiredVel,g2592_ipfrun,r,z']=-0.53334529543633, ['keyframe,0,pendDesiredVel,g2592_ipfrun,L1,x']=0.26524936015123, ['keyframe,0,pendDesiredVel,g2592_ipfrun,L1,z']=-0.53108313618634, ['keyframe,0,pendDesiredVel,g2592_ipfrun,R1,x']=0.27841741405833, ['keyframe,0,pendDesiredVel,g2592_ipfrun,R1,z']=-0.53092197199819, ['keyframe,0,pendDesiredVel,g2592_ipfrun,L2,x']=0.29426067785674, ['keyframe,0,pendDesiredVel,g2592_ipfrun,L2,z']=-0.52799421954717, ['keyframe,0,pendDesiredVel,ignore,1,x']=0.30960864065891, ['keyframe,0,pendDesiredVel,ignore,1,z']=-0.52001606610877, 
['keyframe,0,pendDesiredVel,ignore,0,x']=0.2747737319032, ['keyframe,0,pendDesiredVel,ignore,0,z']=-0.53371901327057, ['keyframe,0,pendDesiredVel,g2592_ipfrun,l,x']=0.26204460034561, ['keyframe,0,pendDesiredVel,g2592_ipfrun,l,z']=-0.53383984874993, ['keyframe,0,pendDesiredVel,g2592_ipfrun,r,x']=0.25860850994182, ['keyframe,0,pendDesiredVel,g2592_ipfrun,r,z']=-0.53236852995563, ['keyframe,0,pendDesiredVel,g2592_ipfrun,L1,x']=0.26877061853418, ['keyframe,0,pendDesiredVel,g2592_ipfrun,L1,z']=-0.52846754364723, ['keyframe,0,pendDesiredVel,g2592_ipfrun,R1,x']=0.28277319247098, ['keyframe,0,pendDesiredVel,g2592_ipfrun,R1,z']=-0.52769844303582, ['keyframe,0,pendDesiredVel,g2592_ipfrun,L2,x']=0.29903331477888, ['keyframe,0,pendDesiredVel,g2592_ipfrun,L2,z']=-0.52434937986167, ['keyframe,0,pendDesiredVel,ignore,1,x']=0.31531294014475, ['keyframe,0,pendDesiredVel,ignore,1,z']=-0.51556688644752, 
['keyframe,0,pendDesiredVel,ignore,0,x']=0.27465801101933, ['keyframe,0,pendDesiredVel,ignore,0,z']=-0.53360025303667, ['keyframe,0,pendDesiredVel,g2592_ipfrun,l,x']=0.26153402652936, ['keyframe,0,pendDesiredVel,g2592_ipfrun,l,z']=-0.5344403883127, ['keyframe,0,pendDesiredVel,g2592_ipfrun,r,x']=0.25741650709809, ['keyframe,0,pendDesiredVel,g2592_ipfrun,r,z']=-0.5328312074445, ['keyframe,0,pendDesiredVel,g2592_ipfrun,L1,x']=0.26622635254283, ['keyframe,0,pendDesiredVel,g2592_ipfrun,L1,z']=-0.53041950989239, ['keyframe,0,pendDesiredVel,g2592_ipfrun,R1,x']=0.27973181851696, ['keyframe,0,pendDesiredVel,g2592_ipfrun,R1,z']=-0.52993060952477, ['keyframe,0,pendDesiredVel,g2592_ipfrun,L2,x']=0.2958574474953, ['keyframe,0,pendDesiredVel,g2592_ipfrun,L2,z']=-0.52677695243413, ['keyframe,0,pendDesiredVel,ignore,1,x']=0.31127522230611, ['keyframe,0,pendDesiredVel,ignore,1,z']=-0.51872814800921, 
['keyframe,0,pendDesiredVel,ignore,0,x']=0.70272519555154, ['keyframe,0,pendDesiredVel,ignore,0,z']=-1.490660229654, ['keyframe,0,pendDesiredVel,g2592_ipfrun,l,x']=-0.13577239950266, ['keyframe,0,pendDesiredVel,g2592_ipfrun,l,z']=1.200198628103, ['keyframe,0,pendDesiredVel,g2592_ipfrun,r,x']=-0.12903929043507, ['keyframe,0,pendDesiredVel,g2592_ipfrun,r,z']=-0.33220380266587, ['keyframe,0,pendDesiredVel,g2592_ipfrun,L1,x']=0.2154981244236, ['keyframe,0,pendDesiredVel,g2592_ipfrun,L1,z']=0.11737191850964, ['keyframe,0,pendDesiredVel,g2592_ipfrun,R1,x']=0.16598113346969, ['keyframe,0,pendDesiredVel,g2592_ipfrun,R1,z']=-0.68342176936861, ['keyframe,0,pendDesiredVel,g2592_ipfrun,L2,x']=0.2804132892824, ['keyframe,0,pendDesiredVel,g2592_ipfrun,L2,z']=0.010895822145381, ['keyframe,0,pendDesiredVel,ignore,1,x']=0.34573564128018, ['keyframe,0,pendDesiredVel,ignore,1,z']=-0.78741299712662, 
['keyframe,0,pendDesiredVel,ignore,0,x']=0.70217126332524, ['keyframe,0,pendDesiredVel,ignore,0,z']=-1.4909459354114, ['keyframe,0,pendDesiredVel,g2592_ipfrun,l,x']=-0.13596173786766, ['keyframe,0,pendDesiredVel,g2592_ipfrun,l,z']=1.1999373440271, ['keyframe,0,pendDesiredVel,g2592_ipfrun,r,x']=-0.1289924042809, ['keyframe,0,pendDesiredVel,g2592_ipfrun,r,z']=-0.33264597956807, ['keyframe,0,pendDesiredVel,g2592_ipfrun,L1,x']=0.2154845021388, ['keyframe,0,pendDesiredVel,g2592_ipfrun,L1,z']=0.11732453075142, ['keyframe,0,pendDesiredVel,g2592_ipfrun,R1,x']=0.16595974914078, ['keyframe,0,pendDesiredVel,g2592_ipfrun,R1,z']=-0.68345138502836, ['keyframe,0,pendDesiredVel,g2592_ipfrun,L2,x']=0.28041915307207, ['keyframe,0,pendDesiredVel,g2592_ipfrun,L2,z']=0.011031318596234, ['keyframe,0,pendDesiredVel,ignore,1,x']=0.34573981316746, ['keyframe,0,pendDesiredVel,ignore,1,z']=-0.78744833883463, 
['keyframe,0,pendDesiredVel,ignore,0,x']=0.70197367931722, ['keyframe,0,pendDesiredVel,ignore,0,z']=-1.4910424115089, ['keyframe,0,pendDesiredVel,g2592_ipfrun,l,x']=-0.13602655036884, ['keyframe,0,pendDesiredVel,g2592_ipfrun,l,z']=1.1998433795653, ['keyframe,0,pendDesiredVel,g2592_ipfrun,r,x']=-0.1289702719855, ['keyframe,0,pendDesiredVel,g2592_ipfrun,r,z']=-0.33281146774124, ['keyframe,0,pendDesiredVel,g2592_ipfrun,L1,x']=0.21548164148513, ['keyframe,0,pendDesiredVel,g2592_ipfrun,L1,z']=0.11730841712202, ['keyframe,0,pendDesiredVel,g2592_ipfrun,R1,x']=0.16595168960139, ['keyframe,0,pendDesiredVel,g2592_ipfrun,R1,z']=-0.68346241686814, ['keyframe,0,pendDesiredVel,g2592_ipfrun,L2,x']=0.28042081668763, ['keyframe,0,pendDesiredVel,g2592_ipfrun,L2,z']=0.011083272414338, ['keyframe,0,pendDesiredVel,ignore,1,x']=0.34574117069349, ['keyframe,0,pendDesiredVel,ignore,1,z']=-0.78746241982738, 
}
useCases.g2592_ipfrun.pendOptimizationPath=
{
	firstFrames={1*4,7*4,11*4,15*4,19*4,23*4,27*4,31*4},
	--firstFrames={1*4,8*4,12*4,16*4,20*4,24*4,28*4,31*4},
	segments={'ignore,0','g2592_ipfrun,l','g2592_ipfrun,r','g2592_ipfrun,L1','g2592_ipfrun,R1','g2592_ipfrun,L2','ignore,1'},
}
--useCases.g2592_ipfrun.measureOptCost=useCases.measureOptCost

useCases.g2592_ipfrun.segmentations=
{
	g2592_ipfrun={
		firstFrames={1*4,7*4,11*4,15*4,19*4,23*4,27*4,31*4},
		--firstFrames={1*4,8*4,12*4,16*4,20*4,24*4,28*4,31*4},
		--        (blank)    L    R     L     R     L     (blank)
		names={             'l', 'r', 'L1', 'R1', 'L2',  },
		swingL={             0,   1,   0,    1,    0,    },
		swingR={             1,   0,   1,    0,    1,    },
		footRefL= "convertFromSwingL",
		footRefR= "convertFromSwingR",
		usePositionControl=false,
	},
}
useCases.g2592_ipfrun.graphParam=
{
	g2592_ipfrun={
		seg={            'l',  'r',  'L1', 'R1','L'},
		num_key={         3,    3,    3  ,  4,  4},
		key_first={       0,    0,    0  ,  0,  0},
		key_last={       'r', 'L1',  'R1', 'L', 'R1'},
	},
}
useCases.g2592_ipfrun.graph=
{
	{
		"addInterpolatedSegment",
		grpName="g2592_ipfrun",
		name="L",
		seg0={"g2592_ipfrun", "L2"},
		seg1={"g2592_ipfrun", "L1"},
		startWeight=0, endWeight=1
	},
	{"connectMulti", "g2592_ipfrun", "l", "r", "L1", "R1", "L", "R1"},
	--{"initialSegment", "g2592_ipfrun", "l"}
	{"initialSegment", "g2592_ipfrun", "R1"}
}
useCases.g2592_ipfrun.segNames=
{
	"l", "r", "L1", 'R1','L',
}

do 
	useCases.g2592_ipfrun.controlParam={
		['useCases,g2592_ipfrun,COMobjWeight']=0, 
		['useCases,g2592_ipfrun,conservativeW']=1, 
		--['useCases,g2592_ipfrun,contactMargin']=0.01, 
		['useCases,g2592_ipfrun,dotMomentumScale']=0.3, 
		['useCases,g2592_ipfrun,excludeRoot']=true, 
		['useCases,g2592_ipfrun,headControlWeight']=0, 
		['useCases,g2592_ipfrun,k_d_HEAD']=14, 
		['useCases,g2592_ipfrun,k_p_HEAD']=0, 
		['useCases,g2592_ipfrun,k_d_EE']=24, 
		['useCases,g2592_ipfrun,k_d_momentum']=10, 
		['useCases,g2592_ipfrun,k_p_EE']=120, 

		--['useCases,g2592_ipfrun,maxPenetratingVel']=0, 
		['useCases,g2592_ipfrun,momentumThr']=50, 
		['useCases,g2592_ipfrun,noComvelDependentFootAdjustment']=true, 
		['useCases,g2592_ipfrun,noIntersectionPrevenction']=true, 
		['useCases,g2592_ipfrun,numericalDerivDmot']=true, 
		['useCases,g2592_ipfrun,perClassContactMargin']=1, 
		['useCases,g2592_ipfrun,turnGain']=10, 
		['useCases,g2592_ipfrun,velMarginOffset']=0, 

		['useCases,g2592_ipfrun,actuationType']      = useCases.g2592_ipfrun.actuationType,
		['useCases,g2592_ipfrun,k_p_ID']             = useCases.g2592_ipfrun.k_p_ID,
		['useCases,g2592_ipfrun,k_d_ID']             = useCases.g2592_ipfrun.k_d_ID,
		['useCases,g2592_ipfrun,ddqObjWeight']       = useCases.g2592_ipfrun.ddqObjWeight,
		['useCases,g2592_ipfrun,tauObjWeight']       = useCases.g2592_ipfrun.tauObjWeight,
		['useCases,g2592_ipfrun,ftObjWeight']        = useCases.g2592_ipfrun.ftObjWeight,
		['useCases,g2592_ipfrun,aObjWeight']         = useCases.g2592_ipfrun.aObjWeight,
		['useCases,g2592_ipfrun,lambdaObjWeight']    = useCases.g2592_ipfrun.lambdaObjWeight,
		['useCases,g2592_ipfrun,EEobjWeight']        = useCases.g2592_ipfrun.EEobjWeight,
		['useCases,g2592_ipfrun,EEobjWeightAngular'] = useCases.g2592_ipfrun.EEobjWeightAngular,
		['useCases,g2592_ipfrun,momentumWeight']     = useCases.g2592_ipfrun.momentumWeight,
		['useCases,g2592_ipfrun,tauMax']             = useCases.g2592_ipfrun.tauMax,
		['useCases,g2592_ipfrun,ftMax']              = useCases.g2592_ipfrun.ftMax,
	}

	local function accumulate(cp_mod)
		local useCase=useCases.g2592_ipfrun
		useCases.accumulate(useCase, cp_mod)
	end


----------------------------------------------
if g_mode=='default' then

elseif g_mode=='a_.01_mscl_2x' then
	useCases.g2592_ipfrun.aObjWeight = .01*useCases.g2592_ipfrun.aObjWeight
	useCases.g2592_ipfrun.initMsclForceScale=2

	----iter:165 bestfvever:4.3394735736534e-06
	--accumulate({['map,0,swingFootMod,g2592_ipfrun,x']= 1.8203839934653, ['map,0,swingFootMod,g2592_ipfrun,y']= -6.7859919723345, ['map,0,swingFootMod,g2592_ipfrun,z']= 0.38965366563156, ['map,1,swingFootMod,g2592_ipfrun,x']= -2.0936590850239, ['map,1,swingFootMod,g2592_ipfrun,y']= 2.9250746279961, ['map,1,swingFootMod,g2592_ipfrun,z']= 0.99624574111192, ['map,2,swingFootMod,g2592_ipfrun,x']= 1.4852894472553, ['map,2,swingFootMod,g2592_ipfrun,y']= -1.0018754757389, ['map,2,swingFootMod,g2592_ipfrun,z']= -0.72168636686209, ['map,0,spprtFootMod,g2592_ipfrun,x']= -0.80575887559792, ['map,0,spprtFootMod,g2592_ipfrun,y']= -3.8552316280795, ['map,0,spprtFootMod,g2592_ipfrun,z']= -0.20880876898075, ['map,1,spprtFootMod,g2592_ipfrun,x']= -1.0113137468454, ['map,1,spprtFootMod,g2592_ipfrun,y']= -1.1250317535752, ['map,1,spprtFootMod,g2592_ipfrun,z']= 0.11300122223506, ['map,2,spprtFootMod,g2592_ipfrun,x']= 0.12503700363453, ['map,2,spprtFootMod,g2592_ipfrun,y']= -1.0749270918241, ['map,2,spprtFootMod,g2592_ipfrun,z']= 0.44933586916992, })

	--iter:161 bestfvever:4.0745571256719e-06
	accumulate({['map,0,swingFootMod,g2592_ipfrun,x']= 0.68925015851907, ['map,0,swingFootMod,g2592_ipfrun,y']= -3.8192955890805, ['map,0,swingFootMod,g2592_ipfrun,z']= 1.7695329587425, ['map,1,swingFootMod,g2592_ipfrun,x']= -1.9926645973878, ['map,1,swingFootMod,g2592_ipfrun,y']= 3.2503238114996, ['map,1,swingFootMod,g2592_ipfrun,z']= 0.11179025471089, ['map,2,swingFootMod,g2592_ipfrun,x']= 2.2163844481498, ['map,2,swingFootMod,g2592_ipfrun,y']= -1.4108931072669, ['map,2,swingFootMod,g2592_ipfrun,z']= -1.3383642516084, ['map,0,spprtFootMod,g2592_ipfrun,x']= -1.8247668972528, ['map,0,spprtFootMod,g2592_ipfrun,y']= -2.681808848167, ['map,0,spprtFootMod,g2592_ipfrun,z']= -0.07713457806662, ['map,1,spprtFootMod,g2592_ipfrun,x']= -0.44866406515373, ['map,1,spprtFootMod,g2592_ipfrun,y']= -7.9618250117272, ['map,1,spprtFootMod,g2592_ipfrun,z']= -0.87489352162934, ['map,2,spprtFootMod,g2592_ipfrun,x']= 1.5059977646296, ['map,2,spprtFootMod,g2592_ipfrun,y']= -2.1714461464928, ['map,2,spprtFootMod,g2592_ipfrun,z']= 2.7470355510508, })

elseif g_mode=='a_.01' then
	useCases.g2592_ipfrun.aObjWeight = .01*useCases.g2592_ipfrun.aObjWeight

	--iter:139 bestfvever:4.7777838085458e-06
	accumulate({['map,0,swingFootMod,g2592_ipfrun,x']= 2.0582651013322, ['map,0,swingFootMod,g2592_ipfrun,y']= -5.868821840211, ['map,0,swingFootMod,g2592_ipfrun,z']= 0.37258587046315, ['map,1,swingFootMod,g2592_ipfrun,x']= -2.3344841778861, ['map,1,swingFootMod,g2592_ipfrun,y']= 2.7346041615509, ['map,1,swingFootMod,g2592_ipfrun,z']= 1.5617749925371, ['map,2,swingFootMod,g2592_ipfrun,x']= 1.1717440855457, ['map,2,swingFootMod,g2592_ipfrun,y']= -0.47622424458591, ['map,2,swingFootMod,g2592_ipfrun,z']= -0.67322788546227, ['map,0,spprtFootMod,g2592_ipfrun,x']= 1.1912292512333, ['map,0,spprtFootMod,g2592_ipfrun,y']= -5.0681319382497, ['map,0,spprtFootMod,g2592_ipfrun,z']= 0.54603560878761, ['map,1,spprtFootMod,g2592_ipfrun,x']= -0.49458712893062, ['map,1,spprtFootMod,g2592_ipfrun,y']= 3.2040704624784, ['map,1,spprtFootMod,g2592_ipfrun,z']= 0.39782824471612, ['map,2,spprtFootMod,g2592_ipfrun,x']= -0.16485663195326, ['map,2,spprtFootMod,g2592_ipfrun,y']= -5.258695877442, ['map,2,spprtFootMod,g2592_ipfrun,z']= 0.17381314466291, })

	--iter:165 bestfvever:4.3394735736534e-06
	accumulate({['map,0,swingFootMod,g2592_ipfrun,x']= 1.8203839934653, ['map,0,swingFootMod,g2592_ipfrun,y']= -6.7859919723345, ['map,0,swingFootMod,g2592_ipfrun,z']= 0.38965366563156, ['map,1,swingFootMod,g2592_ipfrun,x']= -2.0936590850239, ['map,1,swingFootMod,g2592_ipfrun,y']= 2.9250746279961, ['map,1,swingFootMod,g2592_ipfrun,z']= 0.99624574111192, ['map,2,swingFootMod,g2592_ipfrun,x']= 1.4852894472553, ['map,2,swingFootMod,g2592_ipfrun,y']= -1.0018754757389, ['map,2,swingFootMod,g2592_ipfrun,z']= -0.72168636686209, ['map,0,spprtFootMod,g2592_ipfrun,x']= -0.80575887559792, ['map,0,spprtFootMod,g2592_ipfrun,y']= -3.8552316280795, ['map,0,spprtFootMod,g2592_ipfrun,z']= -0.20880876898075, ['map,1,spprtFootMod,g2592_ipfrun,x']= -1.0113137468454, ['map,1,spprtFootMod,g2592_ipfrun,y']= -1.1250317535752, ['map,1,spprtFootMod,g2592_ipfrun,z']= 0.11300122223506, ['map,2,spprtFootMod,g2592_ipfrun,x']= 0.12503700363453, ['map,2,spprtFootMod,g2592_ipfrun,y']= -1.0749270918241, ['map,2,spprtFootMod,g2592_ipfrun,z']= 0.44933586916992, })

end
----------------------------

useCases.unmapControlParam(useCases.g2592_ipfrun)
end


