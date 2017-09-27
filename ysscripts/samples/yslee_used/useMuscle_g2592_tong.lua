package.path=package.path..";../Samples/ysscripts/lib/?.lua" --;"..package.path
require("utilfunc")

package.path=package.path..";../Samples/ysscripts/samples/?.lua" --;"..package.path
require("useMuscle_g2592_gait")


useCases.g2592_tong=deepCopyTable(useCases.g2592_gait)

useCases.g2592_tong.optimizerMethod='Optimizer.methods.CMAes_ys'

--useCases.g2592_tong.endSegW=10
useCases.g2592_tong.endSegW=15
useCases.g2592_tong.prec=.2
--useCases.g2592_tong.prec=.1

useCases.g2592_tong.modelName='g2592_tong'
useCases.g2592_tong.grpName='g2592_tong'
useCases.g2592_tong.grpNames={'g2592_tong'}

-- the initial pendControlParam and pendOptimizationPath can be automatically generated by using createInitialPendControlParam.lua
useCases.g2592_tong.pendControlParam=
{
['keyframe,0,pendDesiredVel,g2592_tong,l,z']=0,['keyframe,0,pendDesiredVel,g2592_tong,l,x']=1,['keyframe,0,pendDesiredVel,g2592_tong,r,z']=0,['keyframe,0,pendDesiredVel,g2592_tong,r,x']=1,['keyframe,0,pendDesiredVel,g2592_tong,L1,z']=0,['keyframe,0,pendDesiredVel,g2592_tong,L1,x']=1,['keyframe,0,pendDesiredVel,g2592_tong,R1,z']=0,['keyframe,0,pendDesiredVel,g2592_tong,R1,x']=1,['keyframe,0,pendDesiredVel,g2592_tong,L2,z']=0,['keyframe,0,pendDesiredVel,g2592_tong,L2,x']=1,['keyframe,0,pendDesiredVel,ignore,0,z']=0,['keyframe,0,pendDesiredVel,ignore,0,x']=1,['keyframe,0,pendDesiredVel,ignore,1,z']=0,['keyframe,0,pendDesiredVel,ignore,1,x']=1,
['keyframe,0,pendDesiredVel,ignore,0,x']=0.66962051405945, ['keyframe,0,pendDesiredVel,ignore,0,z']=-0.18381670796392, ['keyframe,0,pendDesiredVel,g2592_tong,l,x']=0.94465402897006, ['keyframe,0,pendDesiredVel,g2592_tong,l,z']=-0.08959452442499, ['keyframe,0,pendDesiredVel,g2592_tong,r,x']=1.1448350461865, ['keyframe,0,pendDesiredVel,g2592_tong,r,z']=-0.019028985186213, ['keyframe,0,pendDesiredVel,g2592_tong,L1,x']=1.219142677813, ['keyframe,0,pendDesiredVel,g2592_tong,L1,z']=0.018133702339633, ['keyframe,0,pendDesiredVel,g2592_tong,R1,x']=1.1808840813187, ['keyframe,0,pendDesiredVel,g2592_tong,R1,z']=0.014632311233628, ['keyframe,0,pendDesiredVel,g2592_tong,L2,x']=1.0726788728268, ['keyframe,0,pendDesiredVel,g2592_tong,L2,z']=0.0011759143402322, ['keyframe,0,pendDesiredVel,ignore,1,x']=1.001168258635, ['keyframe,0,pendDesiredVel,ignore,1,z']=-0.0066424642965002, 
['keyframe,0,pendDesiredVel,ignore,0,x']=0.66955164882083, ['keyframe,0,pendDesiredVel,ignore,0,z']=-0.18395009583681, ['keyframe,0,pendDesiredVel,g2592_tong,l,x']=0.94456564368442, ['keyframe,0,pendDesiredVel,g2592_tong,l,z']=-0.089717006277478, ['keyframe,0,pendDesiredVel,g2592_tong,r,x']=1.1447995248424, ['keyframe,0,pendDesiredVel,g2592_tong,r,z']=-0.019109379677296, ['keyframe,0,pendDesiredVel,g2592_tong,L1,x']=1.2190941359284, ['keyframe,0,pendDesiredVel,g2592_tong,L1,z']=0.018055826654249, ['keyframe,0,pendDesiredVel,g2592_tong,R1,x']=1.1808449651169, ['keyframe,0,pendDesiredVel,g2592_tong,R1,z']=0.014581646229227, ['keyframe,0,pendDesiredVel,g2592_tong,L2,x']=1.0727323072721, ['keyframe,0,pendDesiredVel,g2592_tong,L2,z']=0.0011579659018924, ['keyframe,0,pendDesiredVel,ignore,1,x']=1.0012439131541, ['keyframe,0,pendDesiredVel,ignore,1,z']=-0.0066446355153523, 
['keyframe,0,pendDesiredVel,ignore,0,x']=0.66953785718483, ['keyframe,0,pendDesiredVel,ignore,0,z']=-0.18397691858239, ['keyframe,0,pendDesiredVel,g2592_tong,l,x']=0.94454776246108, ['keyframe,0,pendDesiredVel,g2592_tong,l,z']=-0.089741636529682, ['keyframe,0,pendDesiredVel,g2592_tong,r,x']=1.1447925066554, ['keyframe,0,pendDesiredVel,g2592_tong,r,z']=-0.019125433335292, ['keyframe,0,pendDesiredVel,g2592_tong,L1,x']=1.219084330762, ['keyframe,0,pendDesiredVel,g2592_tong,L1,z']=0.018040139546162, ['keyframe,0,pendDesiredVel,g2592_tong,R1,x']=1.1808370202132, ['keyframe,0,pendDesiredVel,g2592_tong,R1,z']=0.014571400758833, ['keyframe,0,pendDesiredVel,g2592_tong,L2,x']=1.0727434218634, ['keyframe,0,pendDesiredVel,g2592_tong,L2,z']=0.0011543258372948, ['keyframe,0,pendDesiredVel,ignore,1,x']=1.0012596039332, ['keyframe,0,pendDesiredVel,ignore,1,z']=-0.0066450871502405, 
['keyframe,0,pendDesiredVel,ignore,0,x']=0.28846795527917, ['keyframe,0,pendDesiredVel,ignore,0,z']=-0.3336044680913, ['keyframe,0,pendDesiredVel,g2592_tong,l,x']=1.2757760816674, ['keyframe,0,pendDesiredVel,g2592_tong,l,z']=0.043149087391045, ['keyframe,0,pendDesiredVel,g2592_tong,r,x']=1.3357383993046, ['keyframe,0,pendDesiredVel,g2592_tong,r,z']=0.05160545920664, ['keyframe,0,pendDesiredVel,g2592_tong,L1,x']=1.1622595069635, ['keyframe,0,pendDesiredVel,g2592_tong,L1,z']=0.012067079789902, ['keyframe,0,pendDesiredVel,g2592_tong,R1,x']=1.1042060703375, ['keyframe,0,pendDesiredVel,g2592_tong,R1,z']=-0.038891802335703, ['keyframe,0,pendDesiredVel,g2592_tong,L2,x']=1.0753765034361, ['keyframe,0,pendDesiredVel,g2592_tong,L2,z']=-0.003411330346385, ['keyframe,0,pendDesiredVel,ignore,1,x']=1.0055423225754, ['keyframe,0,pendDesiredVel,ignore,1,z']=-0.005161361653873, 
['keyframe,0,pendDesiredVel,ignore,0,x']=0.28849458328745, ['keyframe,0,pendDesiredVel,ignore,0,z']=-0.33356796804292, ['keyframe,0,pendDesiredVel,g2592_tong,l,x']=1.2757800942976, ['keyframe,0,pendDesiredVel,g2592_tong,l,z']=0.043174649602597, ['keyframe,0,pendDesiredVel,g2592_tong,r,x']=1.3357644301148, ['keyframe,0,pendDesiredVel,g2592_tong,r,z']=0.051635916830859, ['keyframe,0,pendDesiredVel,g2592_tong,L1,x']=1.1622809761041, ['keyframe,0,pendDesiredVel,g2592_tong,L1,z']=0.012088236128798, ['keyframe,0,pendDesiredVel,g2592_tong,R1,x']=1.1042081666204, ['keyframe,0,pendDesiredVel,g2592_tong,R1,z']=-0.038881923137569, ['keyframe,0,pendDesiredVel,g2592_tong,L2,x']=1.0753708812512, ['keyframe,0,pendDesiredVel,g2592_tong,L2,z']=-0.0034113228487757, ['keyframe,0,pendDesiredVel,ignore,1,x']=1.0055416451873, ['keyframe,0,pendDesiredVel,ignore,1,z']=-0.0051615733179333, 
['keyframe,0,pendDesiredVel,ignore,0,x']=0.087115191889932, ['keyframe,0,pendDesiredVel,ignore,0,z']=-0.41522253170469, ['keyframe,0,pendDesiredVel,g2592_tong,l,x']=1.7910940739155, ['keyframe,0,pendDesiredVel,g2592_tong,l,z']=0.26340507761381, ['keyframe,0,pendDesiredVel,g2592_tong,r,x']=1.3766624245191, ['keyframe,0,pendDesiredVel,g2592_tong,r,z']=0.074682444481818, 
['keyframe,0,pendDesiredVel,ignore,0,x']=0.087002068435311, ['keyframe,0,pendDesiredVel,ignore,0,z']=-0.41530198454446, ['keyframe,0,pendDesiredVel,g2592_tong,l,x']=1.7910702778853, ['keyframe,0,pendDesiredVel,g2592_tong,l,z']=0.26338128720823, ['keyframe,0,pendDesiredVel,g2592_tong,r,x']=1.3766428160885, ['keyframe,0,pendDesiredVel,g2592_tong,r,z']=0.074676230458115, 
['keyframe,0,pendDesiredVel,ignore,0,x']=0.087000625549475, ['keyframe,0,pendDesiredVel,ignore,0,z']=-0.41530299845602, ['keyframe,0,pendDesiredVel,g2592_tong,l,x']=1.7910699777259, ['keyframe,0,pendDesiredVel,g2592_tong,l,z']=0.26338098520761, ['keyframe,0,pendDesiredVel,g2592_tong,r,x']=1.3766425635445, ['keyframe,0,pendDesiredVel,g2592_tong,r,z']=0.074676150596708, 
['keyframe,0,pendDesiredVel,ignore,0,x']=0.087000625328907, ['keyframe,0,pendDesiredVel,ignore,0,z']=-0.41530299861101, ['keyframe,0,pendDesiredVel,g2592_tong,l,x']=1.79106997768, ['keyframe,0,pendDesiredVel,g2592_tong,l,z']=0.26338098516145, ['keyframe,0,pendDesiredVel,g2592_tong,r,x']=1.3766425635059, ['keyframe,0,pendDesiredVel,g2592_tong,r,z']=0.074676150584499, 
['keyframe,0,pendDesiredVel,g2592_tong,r,x']=0.97604333605472, ['keyframe,0,pendDesiredVel,g2592_tong,r,z']=-0.10993662175825, ['keyframe,0,pendDesiredVel,g2592_tong,L1,x']=1.0965048647436, ['keyframe,0,pendDesiredVel,g2592_tong,L1,z']=-0.0094927135112825, ['keyframe,0,pendDesiredVel,g2592_tong,R1,x']=1.2265946897683, ['keyframe,0,pendDesiredVel,g2592_tong,R1,z']=0.011979514699824, ['keyframe,0,pendDesiredVel,g2592_tong,L2,x']=1.1671688578095, ['keyframe,0,pendDesiredVel,g2592_tong,L2,z']=0.039219866751207, ['keyframe,0,pendDesiredVel,ignore,1,x']=1.0112150711488, ['keyframe,0,pendDesiredVel,ignore,1,z']=-0.0026498233024977, 
['keyframe,0,pendDesiredVel,g2592_tong,r,x']=0.97612313175009, ['keyframe,0,pendDesiredVel,g2592_tong,r,z']=-0.10985623763177, ['keyframe,0,pendDesiredVel,g2592_tong,L1,x']=1.0965330046857, ['keyframe,0,pendDesiredVel,g2592_tong,L1,z']=-0.0094532307843218, ['keyframe,0,pendDesiredVel,g2592_tong,R1,x']=1.2266175767892, ['keyframe,0,pendDesiredVel,g2592_tong,R1,z']=0.012014621481885, ['keyframe,0,pendDesiredVel,g2592_tong,L2,x']=1.1671782422758, ['keyframe,0,pendDesiredVel,g2592_tong,L2,z']=0.039231074004354, ['keyframe,0,pendDesiredVel,ignore,1,x']=1.011214710477, ['keyframe,0,pendDesiredVel,ignore,1,z']=-0.0026497301332736, 
['keyframe,0,pendDesiredVel,g2592_tong,r,x']=0.9761259498814, ['keyframe,0,pendDesiredVel,g2592_tong,r,z']=-0.10985339500877, ['keyframe,0,pendDesiredVel,g2592_tong,L1,x']=1.0965340112712, ['keyframe,0,pendDesiredVel,g2592_tong,L1,z']=-0.0094518241647491, ['keyframe,0,pendDesiredVel,g2592_tong,R1,x']=1.2266183905441, ['keyframe,0,pendDesiredVel,g2592_tong,R1,z']=0.01201586424517, ['keyframe,0,pendDesiredVel,g2592_tong,L2,x']=1.1671785748589, ['keyframe,0,pendDesiredVel,g2592_tong,L2,z']=0.03923147115995, ['keyframe,0,pendDesiredVel,ignore,1,x']=1.0112146980334, ['keyframe,0,pendDesiredVel,ignore,1,z']=-0.0026497266983117, 
}
useCases.g2592_tong.pendOptimizationPath=
{
	firstFrames={1*4,18*4,35*4,52*4,69*4,86*4,103*4,118*4},
	segments={'ignore,0','g2592_tong,l','g2592_tong,r','g2592_tong,L1','g2592_tong,R1','g2592_tong,L2','ignore,1'},
}
--useCases.g2592_tong.measureOptCost=useCases.measureOptCost

useCases.g2592_tong.segmentations=
{
	g2592_tong={
		firstFrames={1*4,18*4,35*4,52*4,69*4,86*4,103*4,118*4},
		--        (blank)    L    R     L     R     L     (blank)
		names={             'l', 'r', 'L1', 'R1', 'L2',  },
		swingL={             0,   1,   0,    1,    0,    },
		swingR={             1,   0,   1,    0,    1,    },
		footRefL= "convertFromSwingL",
		footRefR= "convertFromSwingR",
		usePositionControl=false,
	},
}
useCases.g2592_tong.graphParam=
{
	g2592_tong={
		seg={            'l',  'r',  'L1', 'R1','L'},
		num_key={         3,    3,    3  ,  4,  4},
		key_first={       0,    0,    0  ,  0,  0},
		key_last={       'r', 'L1',  'R1', 'L', 'R1'},
	},
}
useCases.g2592_tong.graph=
{
	{
		"addInterpolatedSegment",
		grpName="g2592_tong",
		name="L",
		seg0={"g2592_tong", "L2"},
		seg1={"g2592_tong", "L1"},
		startWeight=0, endWeight=1
	},
	{"connectMulti", "g2592_tong", "l", "r", "L1", "R1", "L", "R1"},
	--{"initialSegment", "g2592_tong", "l"}
	{"initialSegment", "g2592_tong", "R1"}
}
useCases.g2592_tong.segNames=
{
	"l", "r", "L1", 'R1','L',
}

do 
	useCases.g2592_tong.controlParam={
		['useCases,g2592_tong,COMobjWeight']=0, 
		['useCases,g2592_tong,conservativeW']=1, 
		--['useCases,g2592_tong,contactMargin']=0.01, 
		['useCases,g2592_tong,dotMomentumScale']=0.3, 
		['useCases,g2592_tong,excludeRoot']=true, 
		['useCases,g2592_tong,headControlWeight']=0, 
		['useCases,g2592_tong,k_d_HEAD']=14, 
		['useCases,g2592_tong,k_p_HEAD']=0, 
		['useCases,g2592_tong,k_d_EE']=24, 
		['useCases,g2592_tong,k_d_momentum']=10, 
		['useCases,g2592_tong,k_p_EE']=120, 

		--['useCases,g2592_tong,maxPenetratingVel']=0, 
		['useCases,g2592_tong,momentumThr']=50, 
		['useCases,g2592_tong,noComvelDependentFootAdjustment']=true, 
		['useCases,g2592_tong,noIntersectionPrevenction']=true, 
		['useCases,g2592_tong,numericalDerivDmot']=true, 
		['useCases,g2592_tong,perClassContactMargin']=1, 
		['useCases,g2592_tong,turnGain']=10, 
		['useCases,g2592_tong,velMarginOffset']=0, 

		['useCases,g2592_tong,actuationType']      = useCases.g2592_tong.actuationType,
		['useCases,g2592_tong,k_p_ID']             = useCases.g2592_tong.k_p_ID,
		['useCases,g2592_tong,k_d_ID']             = useCases.g2592_tong.k_d_ID,
		['useCases,g2592_tong,ddqObjWeight']       = useCases.g2592_tong.ddqObjWeight,
		['useCases,g2592_tong,tauObjWeight']       = useCases.g2592_tong.tauObjWeight,
		['useCases,g2592_tong,ftObjWeight']        = useCases.g2592_tong.ftObjWeight,
		['useCases,g2592_tong,aObjWeight']         = useCases.g2592_tong.aObjWeight,
		['useCases,g2592_tong,lambdaObjWeight']    = useCases.g2592_tong.lambdaObjWeight,
		['useCases,g2592_tong,EEobjWeight']        = useCases.g2592_tong.EEobjWeight,
		['useCases,g2592_tong,EEobjWeightAngular'] = useCases.g2592_tong.EEobjWeightAngular,
		['useCases,g2592_tong,momentumWeight']     = useCases.g2592_tong.momentumWeight,
		['useCases,g2592_tong,tauMax']             = useCases.g2592_tong.tauMax,
		['useCases,g2592_tong,ftMax']              = useCases.g2592_tong.ftMax,
	}

	local function accumulate(cp_mod)
		local useCase=useCases.g2592_tong
		useCases.accumulate(useCase, cp_mod)
	end

	accumulate({
		----['keyframe,0,footRmod,g2592_tong,l,y']  = 0.,
		----['keyframe,1,footRmod,g2592_tong,l,y']  = 0.05,
		--['keyframe,0,footLmod,g2592_tong,r,y']  = 0.0,
		--['keyframe,1,footLmod,g2592_tong,r,y']  = 0.2,
		--['keyframe,0,footRmod,g2592_tong,L1,y'] = 0.0,
		--['keyframe,1,footRmod,g2592_tong,L1,y'] = 0.2,
		--['map,0,swingFootMod,g2592_tong,y']     = 0.1,
		--['map,1,swingFootMod,g2592_tong,y']     = 0.2,
		--['map,2,swingFootMod,g2592_tong,y']     = 0.1,

		----['keyframe,0,footLmod,g2592_tong,l,y']  = 0.,
		----['keyframe,1,footLmod,g2592_tong,l,y']  = 0.,
		--['keyframe,0,footRmod,g2592_tong,r,y']  = 0.,
		--['keyframe,1,footRmod,g2592_tong,r,y']  = -.1,
		--['keyframe,0,footLmod,g2592_tong,L1,y'] = 0.,
		--['keyframe,1,footLmod,g2592_tong,L1,y'] = -.1,
		--['map,0,spprtFootMod,g2592_tong,y']     = 0.,
		--['map,1,spprtFootMod,g2592_tong,y']     = 0.,
		--['map,2,spprtFootMod,g2592_tong,y']     = 0.,
	})
	useCases.unmapControlParam(useCases.g2592_tong)

-------------------------------------------------
if g_mode=='default' then
	--iter:1031 bestfvever:115982.05848753
	accumulate({['map,0,swingFootMod,g2592_tong,x']= 0.040001176028811, ['map,0,swingFootMod,g2592_tong,y']= 0.023861591774672, ['map,0,swingFootMod,g2592_tong,z']= 0.082539775558856, ['map,1,swingFootMod,g2592_tong,x']= 0.13938847064347, ['map,1,swingFootMod,g2592_tong,y']= -0.073086739251364, ['map,1,swingFootMod,g2592_tong,z']= 0.21497629179507, ['map,2,swingFootMod,g2592_tong,x']= -0.26333740019617, ['map,2,swingFootMod,g2592_tong,y']= -0.12932315360837, ['map,2,swingFootMod,g2592_tong,z']= -0.13099369216128, ['map,0,spprtFootMod,g2592_tong,x']= 0.24992197209064, ['map,0,spprtFootMod,g2592_tong,y']= -0.017652121527154, ['map,0,spprtFootMod,g2592_tong,z']= -0.08856601964248, ['map,1,spprtFootMod,g2592_tong,x']= -0.049153137465756, ['map,1,spprtFootMod,g2592_tong,y']= -0.085613660838339, ['map,1,spprtFootMod,g2592_tong,z']= -0.135783072616, ['map,2,spprtFootMod,g2592_tong,x']= 0.058637746951215, ['map,2,spprtFootMod,g2592_tong,y']= -0.026839715782981, ['map,2,spprtFootMod,g2592_tong,z']= 0.29240395656223, })

elseif g_mode=='a_.01_mscl_1.5x' then
	useCases.g2592_tong.aObjWeight = .01*useCases.g2592_tong.aObjWeight
	useCases.g2592_tong.initMsclForceScale=1.5

	--iter:677 bestfvever:165350.9729732
	accumulate({['map,0,swingFootMod,g2592_tong,x']= -0.20477850525396, ['map,0,swingFootMod,g2592_tong,y']= 0.23252507573506, ['map,0,swingFootMod,g2592_tong,z']= -0.42729526665528, ['map,1,swingFootMod,g2592_tong,x']= -0.46170748537279, ['map,1,swingFootMod,g2592_tong,y']= -0.20634147833778, ['map,1,swingFootMod,g2592_tong,z']= -0.74999393799857, ['map,2,swingFootMod,g2592_tong,x']= -0.32994030092913, ['map,2,swingFootMod,g2592_tong,y']= -1.7680343219447, ['map,2,swingFootMod,g2592_tong,z']= 0.21298043071166, ['map,0,spprtFootMod,g2592_tong,x']= 0.39848900316028, ['map,0,spprtFootMod,g2592_tong,y']= -0.1873955137064, ['map,0,spprtFootMod,g2592_tong,z']= -0.40276236775202, ['map,1,spprtFootMod,g2592_tong,x']= 0.38913220752182, ['map,1,spprtFootMod,g2592_tong,y']= 0.010448071897746, ['map,1,spprtFootMod,g2592_tong,z']= -0.20545862499979, ['map,2,spprtFootMod,g2592_tong,x']= 0.70329055983431, ['map,2,spprtFootMod,g2592_tong,y']= 0.10842294082534, ['map,2,spprtFootMod,g2592_tong,z']= 0.46792797121638, })

elseif g_mode=='a_.1_mscl_2x' then
	useCases.g2592_tong.aObjWeight = .1*useCases.g2592_tong.aObjWeight
	useCases.g2592_tong.initMsclForceScale=2

elseif g_mode=='a_.01_mscl_2x' then
	useCases.g2592_tong.aObjWeight = .01*useCases.g2592_tong.aObjWeight
	useCases.g2592_tong.initMsclForceScale=2

	--iter:235 bestfvever:4.9612258286739e-07
	accumulate({['map,0,swingFootMod,g2592_tong,x']= -0.079082543229578, ['map,0,swingFootMod,g2592_tong,y']= -0.30697585605981, ['map,0,swingFootMod,g2592_tong,z']= 0.19495307791308, ['map,1,swingFootMod,g2592_tong,x']= 0.22494784254524, ['map,1,swingFootMod,g2592_tong,y']= 0.2387021610343, ['map,1,swingFootMod,g2592_tong,z']= -0.0098176941767793, ['map,2,swingFootMod,g2592_tong,x']= -0.22449527135057, ['map,2,swingFootMod,g2592_tong,y']= 0.42085587399894, ['map,2,swingFootMod,g2592_tong,z']= 0.12517564685392, ['map,0,spprtFootMod,g2592_tong,x']= 0.55566131149749, ['map,0,spprtFootMod,g2592_tong,y']= -2.2644503232944, ['map,0,spprtFootMod,g2592_tong,z']= 0.0046527662342095, ['map,1,spprtFootMod,g2592_tong,x']= -0.58964039538183, ['map,1,spprtFootMod,g2592_tong,y']= -0.60243512213688, ['map,1,spprtFootMod,g2592_tong,z']= -0.33965978832085, ['map,2,spprtFootMod,g2592_tong,x']= 0.19032829176689, ['map,2,spprtFootMod,g2592_tong,y']= 0.45022453848098, ['map,2,spprtFootMod,g2592_tong,z']= -0.14772222262795, })

elseif g_mode=='test' then
	useCases.g2592_tong.aObjWeight = .01*useCases.g2592_tong.aObjWeight
	useCases.g2592_tong.initMsclForceScale=2
	useCases.g2592_tong.speedMod=vector3(0,0,0)
	--useCases.g2592_tong.speedMod=vector3(2,0,0)

elseif g_mode=='a_.1' then
	useCases.g2592_tong.aObjWeight = .1*useCases.g2592_tong.aObjWeight

	--iter:667 bestfvever:102762.9528626
	accumulate({['map,0,swingFootMod,g2592_tong,x']= 0.24418870879846, ['map,0,swingFootMod,g2592_tong,y']= -0.23392391514169, ['map,0,swingFootMod,g2592_tong,z']= -0.89007369214973, ['map,1,swingFootMod,g2592_tong,x']= -0.52162156669343, ['map,1,swingFootMod,g2592_tong,y']= -0.51730253688522, ['map,1,swingFootMod,g2592_tong,z']= 0.65023026215433, ['map,2,swingFootMod,g2592_tong,x']= 0.39545705659493, ['map,2,swingFootMod,g2592_tong,y']= 0.30189517073583, ['map,2,swingFootMod,g2592_tong,z']= 0.089974438439691, ['map,0,spprtFootMod,g2592_tong,x']= 0.40484472171496, ['map,0,spprtFootMod,g2592_tong,y']= -0.97372818527663, ['map,0,spprtFootMod,g2592_tong,z']= 0.76592662831254, ['map,1,spprtFootMod,g2592_tong,x']= 0.44294750483901, ['map,1,spprtFootMod,g2592_tong,y']= -0.11644095354793, ['map,1,spprtFootMod,g2592_tong,z']= -0.074201814606859, ['map,2,spprtFootMod,g2592_tong,x']= -0.3881348086657, ['map,2,spprtFootMod,g2592_tong,y']= -0.060782009994977, ['map,2,spprtFootMod,g2592_tong,z']= -0.61078062652584, })

elseif g_mode=='a_.01' then
	useCases.g2592_tong.aObjWeight = .01*useCases.g2592_tong.aObjWeight

	--iter:654 bestfvever:109082.93190131
	accumulate({['map,0,swingFootMod,g2592_tong,x']= -0.035220714414521, ['map,0,swingFootMod,g2592_tong,y']= -0.094815490639756, ['map,0,swingFootMod,g2592_tong,z']= -0.081466908789248, ['map,1,swingFootMod,g2592_tong,x']= -0.10276993742739, ['map,1,swingFootMod,g2592_tong,y']= -0.057491993483249, ['map,1,swingFootMod,g2592_tong,z']= -0.16222757457323, ['map,2,swingFootMod,g2592_tong,x']= 0.21824999259733, ['map,2,swingFootMod,g2592_tong,y']= -0.19710536099409, ['map,2,swingFootMod,g2592_tong,z']= 0.11841760420738, ['map,0,spprtFootMod,g2592_tong,x']= -0.088212319163489, ['map,0,spprtFootMod,g2592_tong,y']= -0.23530267310323, ['map,0,spprtFootMod,g2592_tong,z']= 0.15602625488484, ['map,1,spprtFootMod,g2592_tong,x']= -0.047186197300385, ['map,1,spprtFootMod,g2592_tong,y']= -0.111828729837, ['map,1,spprtFootMod,g2592_tong,z']= 0.24134379059316, ['map,2,spprtFootMod,g2592_tong,x']= 0.10808403979848, ['map,2,spprtFootMod,g2592_tong,y']= -0.12561196580233, ['map,2,spprtFootMod,g2592_tong,z']= 0.050920207733556, })

else

	if string.find(g_mode, 'a.01')~=nil then
		useCases.g2592_tong.aObjWeight = .01*useCases.g2592_tong.aObjWeight
	elseif string.find(g_mode, 'a1')~=nil then
		useCases.g2592_tong.aObjWeight = 1*useCases.g2592_tong.aObjWeight
	end

	if string.find(g_mode, 'mscl2')~=nil then
		useCases.g2592_tong.initMsclForceScale=2
	end

	if string.find(g_mode, 'push100_')~=nil then
		magn=100
	elseif string.find(g_mode, 'push120_')~=nil then
		magn=120
	elseif string.find(g_mode, 'push140_')~=nil then
		magn=140
	elseif string.find(g_mode, 'push160_')~=nil then
		magn=160
	elseif string.find(g_mode, 'push180_')~=nil then
		magn=180
	elseif string.find(g_mode, 'push200_')~=nil then
		magn=200
	elseif string.find(g_mode, 'push220_')~=nil then
		magn=220
	end

	if string.find(g_mode, 'push')~=nil then
		--iter:235 bestfvever:4.9612258286739e-07
		accumulate({['map,0,swingFootMod,g2592_tong,x']= -0.079082543229578, ['map,0,swingFootMod,g2592_tong,y']= -0.30697585605981, ['map,0,swingFootMod,g2592_tong,z']= 0.19495307791308, ['map,1,swingFootMod,g2592_tong,x']= 0.22494784254524, ['map,1,swingFootMod,g2592_tong,y']= 0.2387021610343, ['map,1,swingFootMod,g2592_tong,z']= -0.0098176941767793, ['map,2,swingFootMod,g2592_tong,x']= -0.22449527135057, ['map,2,swingFootMod,g2592_tong,y']= 0.42085587399894, ['map,2,swingFootMod,g2592_tong,z']= 0.12517564685392, ['map,0,spprtFootMod,g2592_tong,x']= 0.55566131149749, ['map,0,spprtFootMod,g2592_tong,y']= -2.2644503232944, ['map,0,spprtFootMod,g2592_tong,z']= 0.0046527662342095, ['map,1,spprtFootMod,g2592_tong,x']= -0.58964039538183, ['map,1,spprtFootMod,g2592_tong,y']= -0.60243512213688, ['map,1,spprtFootMod,g2592_tong,z']= -0.33965978832085, ['map,2,spprtFootMod,g2592_tong,x']= 0.19032829176689, ['map,2,spprtFootMod,g2592_tong,y']= 0.45022453848098, ['map,2,spprtFootMod,g2592_tong,z']= -0.14772222262795, })

		useCases.g2592_tong.impulse = {
			{startframe=100, dir=vector3(0,0,1), mag=magn, dur=.2},
			{startframe=300, dir=vector3(0,0,-1), mag=magn, dur=.2},
			{startframe=500, dir=vector3(1,0,0), mag=magn, dur=.2},
			{startframe=700, dir=vector3(-1,0,0), mag=magn, dur=.2},
			{startframe=900, dir=vector3(0,0,1), mag=magn, dur=.2},
			{startframe=1100, dir=vector3(0,0,-1), mag=magn, dur=.2},
		}
	end


	if g_mode=='a1_mscl2' then
		--iter:179 bestfvever:6.4305358206643e-07
		accumulate({['map,0,swingFootMod,g2592_tong,x']= -0.34826108327698, ['map,0,swingFootMod,g2592_tong,y']= -0.5260007963361, ['map,0,swingFootMod,g2592_tong,z']= 0.037343642523222, ['map,1,swingFootMod,g2592_tong,x']= 0.43626254461585, ['map,1,swingFootMod,g2592_tong,y']= 0.40673509568182, ['map,1,swingFootMod,g2592_tong,z']= 0.13272317569757, ['map,2,swingFootMod,g2592_tong,x']= -0.28544726213086, ['map,2,swingFootMod,g2592_tong,y']= 0.12851961273662, ['map,2,swingFootMod,g2592_tong,z']= -0.012506890771542, ['map,0,spprtFootMod,g2592_tong,x']= 0.84982506235355, ['map,0,spprtFootMod,g2592_tong,y']= -1.9766987860235, ['map,0,spprtFootMod,g2592_tong,z']= -0.038378000618216, ['map,1,spprtFootMod,g2592_tong,x']= 0.65327908346509, ['map,1,spprtFootMod,g2592_tong,y']= -0.87221763127033, ['map,1,spprtFootMod,g2592_tong,z']= 0.20404557055837, ['map,2,spprtFootMod,g2592_tong,x']= 0.33789326524176, ['map,2,spprtFootMod,g2592_tong,y']= 0.72063995544004, ['map,2,spprtFootMod,g2592_tong,z']= -0.35334555456655, })
	end

end
-------------------------------------------------

useCases.g2592_tong.controlParam['useCases,g2592_tong,aObjWeight'] = useCases.g2592_tong.aObjWeight

useCases.unmapControlParam(useCases.g2592_tong)
end


