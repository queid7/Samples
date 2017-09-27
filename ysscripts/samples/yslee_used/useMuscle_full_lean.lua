package.path=package.path..";../Samples/ysscripts/lib/?.lua" --;"..package.path
require("utilfunc")

package.path=package.path..";../Samples/ysscripts/samples/?.lua" --;"..package.path
require("useMuscle_full_same")


useCases.full_lean=deepCopyTable(useCases.full_same)

useCases.full_lean.optimizerMethod='Optimizer.methods.CMAes_ys'

useCases.full_lean.endSegW=10
useCases.full_lean.prec=.2
--useCases.full_lean.endSegW=20
--useCases.full_lean.prec=.05

useCases.full_lean.modelName='full_lean'
useCases.full_lean.grpName='full_lean'
useCases.full_lean.grpNames={'full_lean'}

--useCases.full_lean.ddqObjWeight=20000000*(25-7)
--useCases.full_lean.aObjWeight=100000000*62
--useCases.full_lean.lambdaObjWeight=.1
--useCases.full_lean.EEobjWeight=10000000
--useCases.full_lean.EEobjWeightAngular=1000000
--useCases.full_lean.momentumWeight=1000000

-- the initial pendControlParam and pendOptimizationPath can be automatically generated by using createInitialPendControlParam.lua
useCases.full_lean.pendControlParam=
{
['keyframe,0,pendDesiredVel,full_lean,l,z']=0,['keyframe,0,pendDesiredVel,full_lean,l,x']=1,['keyframe,0,pendDesiredVel,full_lean,r,z']=0,['keyframe,0,pendDesiredVel,full_lean,r,x']=1,['keyframe,0,pendDesiredVel,full_lean,L1,z']=0,['keyframe,0,pendDesiredVel,full_lean,L1,x']=1,['keyframe,0,pendDesiredVel,full_lean,R1,z']=0,['keyframe,0,pendDesiredVel,full_lean,R1,x']=1,['keyframe,0,pendDesiredVel,full_lean,L2,z']=0,['keyframe,0,pendDesiredVel,full_lean,L2,x']=1,['keyframe,0,pendDesiredVel,ignore,0,z']=0,['keyframe,0,pendDesiredVel,ignore,0,x']=1,['keyframe,0,pendDesiredVel,ignore,1,z']=0,['keyframe,0,pendDesiredVel,ignore,1,x']=1,
['keyframe,0,pendDesiredVel,ignore,0,x']=0.49221236658979, ['keyframe,0,pendDesiredVel,ignore,0,z']=-0.0054183433195051, ['keyframe,0,pendDesiredVel,full_lean,l,x']=0.65157250045983, ['keyframe,0,pendDesiredVel,full_lean,l,z']=-0.015155085230071, ['keyframe,0,pendDesiredVel,full_lean,r,x']=0.71655843417388, ['keyframe,0,pendDesiredVel,full_lean,r,z']=0.013602984519031, ['keyframe,0,pendDesiredVel,full_lean,L1,x']=0.72777850080291, ['keyframe,0,pendDesiredVel,full_lean,L1,z']=0.0012751554227634, ['keyframe,0,pendDesiredVel,full_lean,R1,x']=0.75167607322418, ['keyframe,0,pendDesiredVel,full_lean,R1,z']=0.0042442928911984, ['keyframe,0,pendDesiredVel,full_lean,L2,x']=0.84822437118635, ['keyframe,0,pendDesiredVel,full_lean,L2,z']=0.0019809272886494, ['keyframe,0,pendDesiredVel,ignore,1,x']=0.95788833475772, ['keyframe,0,pendDesiredVel,ignore,1,z']=-0.0022933968172409, 
['keyframe,0,pendDesiredVel,ignore,0,x']=0.49133593616463, ['keyframe,0,pendDesiredVel,ignore,0,z']=-0.0056032730252848, ['keyframe,0,pendDesiredVel,full_lean,l,x']=0.65042956157686, ['keyframe,0,pendDesiredVel,full_lean,l,z']=-0.014746146581524, ['keyframe,0,pendDesiredVel,full_lean,r,x']=0.71622136677421, ['keyframe,0,pendDesiredVel,full_lean,r,z']=0.013179046009638, ['keyframe,0,pendDesiredVel,full_lean,L1,x']=0.72775339291649, ['keyframe,0,pendDesiredVel,full_lean,L1,z']=0.0014843659503535, ['keyframe,0,pendDesiredVel,full_lean,R1,x']=0.75168439081034, ['keyframe,0,pendDesiredVel,full_lean,R1,z']=0.0040984155592843, ['keyframe,0,pendDesiredVel,full_lean,L2,x']=0.84768850640149, ['keyframe,0,pendDesiredVel,full_lean,L2,z']=0.0019755580995616, ['keyframe,0,pendDesiredVel,ignore,1,x']=0.95664000871785, ['keyframe,0,pendDesiredVel,ignore,1,z']=-0.0023169285859117, 
['keyframe,0,pendDesiredVel,ignore,0,x']=0.48802705563885, ['keyframe,0,pendDesiredVel,ignore,0,z']=-0.013954315117401, ['keyframe,0,pendDesiredVel,full_lean,l,x']=0.6244062987454, ['keyframe,0,pendDesiredVel,full_lean,l,z']=0.010725430993183, ['keyframe,0,pendDesiredVel,full_lean,r,x']=0.73397785100005, ['keyframe,0,pendDesiredVel,full_lean,r,z']=-0.010850698481838, ['keyframe,0,pendDesiredVel,full_lean,L1,x']=0.75669671729001, ['keyframe,0,pendDesiredVel,full_lean,L1,z']=0.014076689497005, ['keyframe,0,pendDesiredVel,full_lean,R1,x']=0.76814774166426, ['keyframe,0,pendDesiredVel,full_lean,R1,z']=-0.0036664352612009, ['keyframe,0,pendDesiredVel,full_lean,L2,x']=0.81153560827322, ['keyframe,0,pendDesiredVel,full_lean,L2,z']=0.0011353640524627, ['keyframe,0,pendDesiredVel,ignore,1,x']=0.86151119657935, ['keyframe,0,pendDesiredVel,ignore,1,z']=-0.0040746042475397, 
['keyframe,0,pendDesiredVel,ignore,0,x']=0.32826329649416, ['keyframe,0,pendDesiredVel,ignore,0,z']=-0.020361337351578, ['keyframe,0,pendDesiredVel,full_lean,l,x']=0.78980578851882, ['keyframe,0,pendDesiredVel,full_lean,l,z']=0.012594991591796, ['keyframe,0,pendDesiredVel,full_lean,r,x']=0.82464935644462, ['keyframe,0,pendDesiredVel,full_lean,r,z']=-0.017044485891922, ['keyframe,0,pendDesiredVel,full_lean,L1,x']=0.72807473607511, ['keyframe,0,pendDesiredVel,full_lean,L1,z']=0.019158322886792, ['keyframe,0,pendDesiredVel,full_lean,R1,x']=0.66823463956661, ['keyframe,0,pendDesiredVel,full_lean,R1,z']=-0.01967797231025, ['keyframe,0,pendDesiredVel,full_lean,L2,x']=0.74525381662103, ['keyframe,0,pendDesiredVel,full_lean,L2,z']=0.029616694904189, ['keyframe,0,pendDesiredVel,ignore,1,x']=0.85394965549782, ['keyframe,0,pendDesiredVel,ignore,1,z']=0.0024231764523249, 
['keyframe,0,pendDesiredVel,ignore,0,x']=0.32812626917668, ['keyframe,0,pendDesiredVel,ignore,0,z']=-0.020362836187669, ['keyframe,0,pendDesiredVel,full_lean,l,x']=0.78974011156082, ['keyframe,0,pendDesiredVel,full_lean,l,z']=0.012621619635358, ['keyframe,0,pendDesiredVel,full_lean,r,x']=0.82453552568791, ['keyframe,0,pendDesiredVel,full_lean,r,z']=-0.017013549388808, ['keyframe,0,pendDesiredVel,full_lean,L1,x']=0.72801863505382, ['keyframe,0,pendDesiredVel,full_lean,L1,z']=0.019209591407334, ['keyframe,0,pendDesiredVel,full_lean,R1,x']=0.66823749390296, ['keyframe,0,pendDesiredVel,full_lean,R1,z']=-0.019651544477381, ['keyframe,0,pendDesiredVel,full_lean,L2,x']=0.74526914597701, ['keyframe,0,pendDesiredVel,full_lean,L2,z']=0.029643127314895, ['keyframe,0,pendDesiredVel,ignore,1,x']=0.85394883070678, ['keyframe,0,pendDesiredVel,ignore,1,z']=0.0024275502981052, 
['keyframe,0,pendDesiredVel,ignore,0,x']=0.32811222428954, ['keyframe,0,pendDesiredVel,ignore,0,z']=-0.020363146345834, ['keyframe,0,pendDesiredVel,full_lean,l,x']=0.78973364097969, ['keyframe,0,pendDesiredVel,full_lean,l,z']=0.012624292523184, ['keyframe,0,pendDesiredVel,full_lean,r,x']=0.82452380445559, ['keyframe,0,pendDesiredVel,full_lean,r,z']=-0.017010364754889, ['keyframe,0,pendDesiredVel,full_lean,L1,x']=0.72801296566233, ['keyframe,0,pendDesiredVel,full_lean,L1,z']=0.01921495997783, ['keyframe,0,pendDesiredVel,full_lean,R1,x']=0.66823800490074, ['keyframe,0,pendDesiredVel,full_lean,R1,z']=-0.0196487788972, ['keyframe,0,pendDesiredVel,full_lean,L2,x']=0.7452708599926, ['keyframe,0,pendDesiredVel,full_lean,L2,z']=0.029645930886528, ['keyframe,0,pendDesiredVel,ignore,1,x']=0.85394874547131, ['keyframe,0,pendDesiredVel,ignore,1,z']=0.0024280180154946, 
}
useCases.full_lean.pendOptimizationPath=
{
	firstFrames={1*4,21*4,41*4,61*4,81*4,101*4,121*4,139*4},
	segments={'ignore,0','full_lean,l','full_lean,r','full_lean,L1','full_lean,R1','full_lean,L2','ignore,1'},
}
--useCases.full_lean.measureOptCost=useCases.measureOptCost

useCases.full_lean.segmentations=
{
	full_lean={
		firstFrames={1*4,21*4,41*4,61*4,81*4,101*4,121*4,139*4},
		--        (blank)    L    R     L     R     L     (blank)
		names={             'l', 'r', 'L1', 'R1', 'L2',  },
		swingL={             0,   1,   0,    1,    0,    },
		swingR={             1,   0,   1,    0,    1,    },
		footRefL= "convertFromSwingL",
		footRefR= "convertFromSwingR",
		usePositionControl=false,
	},
}
useCases.full_lean.graphParam=
{
	full_lean={
		seg={            'l',  'r',  'L1', 'R1','L'},
		num_key={         3,    3,    3  ,  4,  4},
		key_first={       0,    0,    0  ,  0,  0},
		key_last={       'r', 'L1',  'R1', 'L', 'R1'},
	},
}
useCases.full_lean.graph=
{
	{
		"addInterpolatedSegment",
		grpName="full_lean",
		name="L",
		seg0={"full_lean", "L2"},
		seg1={"full_lean", "L1"},
		startWeight=0, endWeight=1
	},
	{"connectMulti", "full_lean", "l", "r", "L1", "R1", "L", "R1"},
	--{"initialSegment", "full_lean", "l"}
	{"initialSegment", "full_lean", "R1"}
}
useCases.full_lean.segNames=
{
	"l", "r", "L1", 'R1','L',
}

do 
	useCases.full_lean.controlParam={
		['useCases,full_lean,COMobjWeight']=0, 
		['useCases,full_lean,conservativeW']=1, 
		--['useCases,full_lean,contactMargin']=0.01, 
		['useCases,full_lean,dotMomentumScale']=0.3, 
		['useCases,full_lean,excludeRoot']=true, 
		['useCases,full_lean,headControlWeight']=0, 
		['useCases,full_lean,k_d_HEAD']=14, 
		['useCases,full_lean,k_p_HEAD']=0, 
		['useCases,full_lean,k_d_EE']=24, 
		['useCases,full_lean,k_d_momentum']=10, 
		['useCases,full_lean,k_p_EE']=120, 

		--['useCases,full_lean,maxPenetratingVel']=0, 
		['useCases,full_lean,momentumThr']=50, 
		['useCases,full_lean,noComvelDependentFootAdjustment']=true, 
		['useCases,full_lean,noIntersectionPrevenction']=true, 
		['useCases,full_lean,numericalDerivDmot']=true, 
		['useCases,full_lean,perClassContactMargin']=1, 
		['useCases,full_lean,turnGain']=10, 
		['useCases,full_lean,velMarginOffset']=0, 

		['useCases,full_lean,actuationType']      = useCases.full_lean.actuationType,
		['useCases,full_lean,k_p_ID']             = useCases.full_lean.k_p_ID,
		['useCases,full_lean,k_d_ID']             = useCases.full_lean.k_d_ID,
		['useCases,full_lean,ddqObjWeight']       = useCases.full_lean.ddqObjWeight,
		['useCases,full_lean,tauObjWeight']       = useCases.full_lean.tauObjWeight,
		['useCases,full_lean,ftObjWeight']        = useCases.full_lean.ftObjWeight,
		['useCases,full_lean,aObjWeight']         = useCases.full_lean.aObjWeight,
		['useCases,full_lean,lambdaObjWeight']    = useCases.full_lean.lambdaObjWeight,
		['useCases,full_lean,EEobjWeight']        = useCases.full_lean.EEobjWeight,
		['useCases,full_lean,EEobjWeightAngular'] = useCases.full_lean.EEobjWeightAngular,
		['useCases,full_lean,momentumWeight']     = useCases.full_lean.momentumWeight,
		['useCases,full_lean,tauMax']             = useCases.full_lean.tauMax,
		['useCases,full_lean,ftMax']              = useCases.full_lean.ftMax,
	}

	local function accumulate(cp_mod)
		local useCase=useCases.full_lean
		useCases.accumulate(useCase, cp_mod)
	end

	accumulate({
		----['keyframe,0,footRmod,full_lean,l,y']  = 0.,
		----['keyframe,1,footRmod,full_lean,l,y']  = 0.05,
		--['keyframe,0,footLmod,full_lean,r,y']  = 0.0,
		--['keyframe,1,footLmod,full_lean,r,y']  = 0.2,
		--['keyframe,0,footRmod,full_lean,L1,y'] = 0.0,
		--['keyframe,1,footRmod,full_lean,L1,y'] = 0.2,
		--['map,0,swingFootMod,full_lean,y']     = 0.1,
		--['map,1,swingFootMod,full_lean,y']     = 0.2,
		--['map,2,swingFootMod,full_lean,y']     = 0.1,

		----['keyframe,0,footLmod,full_lean,l,y']  = 0.,
		----['keyframe,1,footLmod,full_lean,l,y']  = 0.,
		--['keyframe,0,footRmod,full_lean,r,y']  = 0.,
		--['keyframe,1,footRmod,full_lean,r,y']  = -.1,
		--['keyframe,0,footLmod,full_lean,L1,y'] = 0.,
		--['keyframe,1,footLmod,full_lean,L1,y'] = -.1,
		--['map,0,spprtFootMod,full_lean,y']     = 0.,
		--['map,1,spprtFootMod,full_lean,y']     = 0.,
		--['map,2,spprtFootMod,full_lean,y']     = 0.,
	})
	useCases.unmapControlParam(useCases.full_lean)


-------------------------------------------------
if g_mode=='default' then
	--old weight
	----iter:175 bestfvever:7.4643352504611e-07 endSegW:10
	--accumulate({['map,0,swingFootMod,full_lean,x']= -0.10618654442692, ['map,0,swingFootMod,full_lean,y']= 0.04205615818675, ['map,0,swingFootMod,full_lean,z']= 0.26243445177334, ['map,1,swingFootMod,full_lean,x']= -0.80385885919627, ['map,1,swingFootMod,full_lean,y']= -1.1987545883134, ['map,1,swingFootMod,full_lean,z']= -0.068291123118659, ['map,2,swingFootMod,full_lean,x']= 0.43056618237193, ['map,2,swingFootMod,full_lean,y']= -0.053601362069685, ['map,2,swingFootMod,full_lean,z']= 0.32750623154267, ['map,0,spprtFootMod,full_lean,x']= 0.25719104690503, ['map,0,spprtFootMod,full_lean,y']= -0.77036092234635, ['map,0,spprtFootMod,full_lean,z']= 0.41808716283792, ['map,1,spprtFootMod,full_lean,x']= 0.3064211813428, ['map,1,spprtFootMod,full_lean,y']= -0.014773088524812, ['map,1,spprtFootMod,full_lean,z']= -0.33689099024384, ['map,2,spprtFootMod,full_lean,x']= 0.25286392529427, ['map,2,spprtFootMod,full_lean,y']= -0.40823926359621, ['map,2,spprtFootMod,full_lean,z']= 0.077781455526068, })
	----iter:184 bestfvever:7.3640582759011e-07
	--accumulate({['map,0,swingFootMod,full_lean,x']= -0.070554636379753, ['map,0,swingFootMod,full_lean,y']= 0.033602671096714, ['map,0,swingFootMod,full_lean,z']= 0.27874357670162, ['map,1,swingFootMod,full_lean,x']= -0.79651527872566, ['map,1,swingFootMod,full_lean,y']= -1.0708927339912, ['map,1,swingFootMod,full_lean,z']= 0.017346756251179, ['map,2,swingFootMod,full_lean,x']= 0.44974133439488, ['map,2,swingFootMod,full_lean,y']= -0.10667136848945, ['map,2,swingFootMod,full_lean,z']= 0.26127145497544, ['map,0,spprtFootMod,full_lean,x']= 0.24775193999987, ['map,0,spprtFootMod,full_lean,y']= -0.8347354087589, ['map,0,spprtFootMod,full_lean,z']= 0.43795073586878, ['map,1,spprtFootMod,full_lean,x']= 0.40436519960406, ['map,1,spprtFootMod,full_lean,y']= -0.064508691565249, ['map,1,spprtFootMod,full_lean,z']= -0.38034762213079, ['map,2,spprtFootMod,full_lean,x']= 0.14837710184579, ['map,2,spprtFootMod,full_lean,y']= -0.45649174109352, ['map,2,spprtFootMod,full_lean,z']= 0.082508196307027, })

	--old weight2
	----iter:128 bestfvever:6.3538854580386e-07
	--accumulate({['map,0,swingFootMod,full_lean,x']= 0.042113142534673, ['map,0,swingFootMod,full_lean,y']= -0.024404257269611, ['map,0,swingFootMod,full_lean,z']= 0.076098997854622, ['map,1,swingFootMod,full_lean,x']= -0.12253655556219, ['map,1,swingFootMod,full_lean,y']= 0.23820562688605, ['map,1,swingFootMod,full_lean,z']= 0.21201363052394, ['map,2,swingFootMod,full_lean,x']= 0.14368034572532, ['map,2,swingFootMod,full_lean,y']= -0.15395151142322, ['map,2,swingFootMod,full_lean,z']= 0.05454061449172, ['map,0,spprtFootMod,full_lean,x']= -0.12854397258473, ['map,0,spprtFootMod,full_lean,y']= 0.35419570461627, ['map,0,spprtFootMod,full_lean,z']= 0.012041745908916, ['map,1,spprtFootMod,full_lean,x']= 0.33053652562639, ['map,1,spprtFootMod,full_lean,y']= -0.20626389096917, ['map,1,spprtFootMod,full_lean,z']= 0.089878096364617, ['map,2,spprtFootMod,full_lean,x']= 0.13893340963462, ['map,2,spprtFootMod,full_lean,y']= -0.40581404346183, ['map,2,spprtFootMod,full_lean,z']= -0.2685994349737, })

	--old weight3
	----iter:154 bestfvever:2.7548638145877e-06
	--accumulate({['map,0,swingFootMod,full_lean,x']= 0.34222904302934, ['map,0,swingFootMod,full_lean,y']= -0.306118657495, ['map,0,swingFootMod,full_lean,z']= 0.10456383977294, ['map,1,swingFootMod,full_lean,x']= -0.026761127302385, ['map,1,swingFootMod,full_lean,y']= 0.11068170372664, ['map,1,swingFootMod,full_lean,z']= 0.073706815487607, ['map,2,swingFootMod,full_lean,x']= -0.36217170349684, ['map,2,swingFootMod,full_lean,y']= 0.88393449962055, ['map,2,swingFootMod,full_lean,z']= 0.16019871036092, ['map,0,spprtFootMod,full_lean,x']= 0.25613844419059, ['map,0,spprtFootMod,full_lean,y']= -0.66502129825607, ['map,0,spprtFootMod,full_lean,z']= 0.34634686256681, ['map,1,spprtFootMod,full_lean,x']= -0.22158569119105, ['map,1,spprtFootMod,full_lean,y']= 0.050974454522946, ['map,1,spprtFootMod,full_lean,z']= -0.018499356621321, ['map,2,spprtFootMod,full_lean,x']= 0.13777316456976, ['map,2,spprtFootMod,full_lean,y']= -0.26552985241061, ['map,2,spprtFootMod,full_lean,z']= -0.19074709852641, })
	
	--old weight3
	----iter:342 bestfvever:96818.711184637
	--accumulate({['map,0,swingFootMod,full_lean,x']= -0.094361647513022, ['map,0,swingFootMod,full_lean,y']= 0.04159006170082, ['map,0,swingFootMod,full_lean,z']= 0.10492087855801, ['map,1,swingFootMod,full_lean,x']= 0.044940915078446, ['map,1,swingFootMod,full_lean,y']= -0.078605078099797, ['map,1,swingFootMod,full_lean,z']= -0.13489207527479, ['map,2,swingFootMod,full_lean,x']= 0.095699894946873, ['map,2,swingFootMod,full_lean,y']= 0.037137835617642, ['map,2,swingFootMod,full_lean,z']= 0.16514601809474, ['map,0,spprtFootMod,full_lean,x']= 0.087674078484797, ['map,0,spprtFootMod,full_lean,y']= -0.040187328528842, ['map,0,spprtFootMod,full_lean,z']= -0.081818216895497, ['map,1,spprtFootMod,full_lean,x']= 0.057022945742481, ['map,1,spprtFootMod,full_lean,y']= -0.13862616480796, ['map,1,spprtFootMod,full_lean,z']= 0.00062858492554849, ['map,2,spprtFootMod,full_lean,x']= -0.16002173462388, ['map,2,spprtFootMod,full_lean,y']= -0.068260523688817, ['map,2,spprtFootMod,full_lean,z']= -0.14631350654502, })

	--old weight3
	----iter:640 bestfvever:139284.31970954
	--accumulate({['map,0,swingFootMod,full_lean,x']= 0.023297156331818, ['map,0,swingFootMod,full_lean,y']= 0.13705338731666, ['map,0,swingFootMod,full_lean,z']= 0.10586314609661, ['map,1,swingFootMod,full_lean,x']= 0.069173854452477, ['map,1,swingFootMod,full_lean,y']= -0.24229433891903, ['map,1,swingFootMod,full_lean,z']= 0.012560514842337, ['map,2,swingFootMod,full_lean,x']= -0.029281579163596, ['map,2,swingFootMod,full_lean,y']= 0.020141647340222, ['map,2,swingFootMod,full_lean,z']= 0.16332323233323, ['map,0,spprtFootMod,full_lean,x']= 0.058620140342197, ['map,0,spprtFootMod,full_lean,y']= -0.063185851280692, ['map,0,spprtFootMod,full_lean,z']= -0.074086446391857, ['map,1,spprtFootMod,full_lean,x']= -0.025364787311031, ['map,1,spprtFootMod,full_lean,y']= -0.057877677773419, ['map,1,spprtFootMod,full_lean,z']= -0.002319544299881, ['map,2,spprtFootMod,full_lean,x']= 0.11818481384138, ['map,2,spprtFootMod,full_lean,y']= -0.14056181808533, ['map,2,spprtFootMod,full_lean,z']= -0.20672056982301, })

	--iter:150 bestfvever:6.864652050195e-07
	accumulate({['map,0,swingFootMod,full_lean,x']= 0.33110401305774, ['map,0,swingFootMod,full_lean,y']= -0.17931479277263, ['map,0,swingFootMod,full_lean,z']= 0.079838719767522, ['map,1,swingFootMod,full_lean,x']= -0.13146642934476, ['map,1,swingFootMod,full_lean,y']= 0.096205464295965, ['map,1,swingFootMod,full_lean,z']= 0.12074585045023, ['map,2,swingFootMod,full_lean,x']= -0.039697529177841, ['map,2,swingFootMod,full_lean,y']= 0.22740441709716, ['map,2,swingFootMod,full_lean,z']= 0.10917566142782, ['map,0,spprtFootMod,full_lean,x']= 0.32169685753624, ['map,0,spprtFootMod,full_lean,y']= -0.49791002165323, ['map,0,spprtFootMod,full_lean,z']= 0.071036840535051, ['map,1,spprtFootMod,full_lean,x']= -0.12627181073495, ['map,1,spprtFootMod,full_lean,y']= -0.11467361369435, ['map,1,spprtFootMod,full_lean,z']= 0.20990597510445, ['map,2,spprtFootMod,full_lean,x']= 0.10990572225739, ['map,2,spprtFootMod,full_lean,y']= -0.3543482157689, ['map,2,spprtFootMod,full_lean,z']= 0.14322270835156, })

	useCases.full_lean.endSegW=20
	useCases.full_lean.prec=.05

	--iter:35 bestfvever:5.4637141267082e-07
	accumulate({['map,0,swingFootMod,full_lean,x']= 0.32487979492447, ['map,0,swingFootMod,full_lean,y']= -0.23294825751964, ['map,0,swingFootMod,full_lean,z']= 0.069087662930214, ['map,1,swingFootMod,full_lean,x']= -0.12310865140694, ['map,1,swingFootMod,full_lean,y']= 0.058416021380179, ['map,1,swingFootMod,full_lean,z']= 0.12349190150442, ['map,2,swingFootMod,full_lean,x']= -0.025840748230117, ['map,2,swingFootMod,full_lean,y']= 0.28114383265261, ['map,2,swingFootMod,full_lean,z']= 0.067821554241085, ['map,0,spprtFootMod,full_lean,x']= 0.36068569458692, ['map,0,spprtFootMod,full_lean,y']= -0.49068973402128, ['map,0,spprtFootMod,full_lean,z']= 0.048016484564426, ['map,1,spprtFootMod,full_lean,x']= -0.13574727033177, ['map,1,spprtFootMod,full_lean,y']= -0.085090503363895, ['map,1,spprtFootMod,full_lean,z']= 0.1761075838821, ['map,2,spprtFootMod,full_lean,x']= 0.074519715243042, ['map,2,spprtFootMod,full_lean,y']= -0.36535550194471, ['map,2,spprtFootMod,full_lean,z']= 0.10358960149049, })

elseif g_mode=='a_.1' then
	useCases.full_lean.aObjWeight = .1*useCases.full_lean.aObjWeight

	--old weight
	----iter:143 bestfvever:3.2451238328992e-06
	--accumulate({['map,0,swingFootMod,full_lean,x']= 0.18821230393842, ['map,0,swingFootMod,full_lean,y']= -1.890882830844, ['map,0,swingFootMod,full_lean,z']= -0.47076351925667, ['map,1,swingFootMod,full_lean,x']= 0.70936672781767, ['map,1,swingFootMod,full_lean,y']= 0.2152664396844, ['map,1,swingFootMod,full_lean,z']= 1.1449213214393, ['map,2,swingFootMod,full_lean,x']= -1.9146103214185, ['map,2,swingFootMod,full_lean,y']= -0.44104534300974, ['map,2,swingFootMod,full_lean,z']= -0.41502982384723, ['map,0,spprtFootMod,full_lean,x']= -0.15419271216273, ['map,0,spprtFootMod,full_lean,y']= -4.5293390949852, ['map,0,spprtFootMod,full_lean,z']= 0.75439473601462, ['map,1,spprtFootMod,full_lean,x']= -0.21839011006252, ['map,1,spprtFootMod,full_lean,y']= -2.08376474084, ['map,1,spprtFootMod,full_lean,z']= 0.58763413238968, ['map,2,spprtFootMod,full_lean,x']= 0.082198026239879, ['map,2,spprtFootMod,full_lean,y']= -0.0088062615982959, ['map,2,spprtFootMod,full_lean,z']= -0.069627674827376, })

	--old weight2
	----iter:125 bestfvever:4.7398681939635e-07
	--accumulate({['map,0,swingFootMod,full_lean,x']= -0.0098549538141145, ['map,0,swingFootMod,full_lean,y']= -0.63719029913722, ['map,0,swingFootMod,full_lean,z']= -0.057967728015552, ['map,1,swingFootMod,full_lean,x']= 0.045968122640709, ['map,1,swingFootMod,full_lean,y']= 0.12234941628592, ['map,1,swingFootMod,full_lean,z']= 0.3129348701908, ['map,2,swingFootMod,full_lean,x']= -0.073401056811354, ['map,2,swingFootMod,full_lean,y']= 0.50715001687001, ['map,2,swingFootMod,full_lean,z']= -0.1530515815834, ['map,0,spprtFootMod,full_lean,x']= -0.058746680712193, ['map,0,spprtFootMod,full_lean,y']= 0.12817633967519, ['map,0,spprtFootMod,full_lean,z']= -0.14639408785229, ['map,1,spprtFootMod,full_lean,x']= 0.18388750297216, ['map,1,spprtFootMod,full_lean,y']= -0.27525753424143, ['map,1,spprtFootMod,full_lean,z']= 0.1149289557827, ['map,2,spprtFootMod,full_lean,x']= 0.14713443015857, ['map,2,spprtFootMod,full_lean,y']= -0.18271151893077, ['map,2,spprtFootMod,full_lean,z']= -0.23338805814204, })

	--old weight3
	----iter:159 bestfvever:2.5507254606243e-06
	--accumulate({['map,0,swingFootMod,full_lean,x']= 0.33293229345364, ['map,0,swingFootMod,full_lean,y']= -0.61660875310404, ['map,0,swingFootMod,full_lean,z']= -0.21696754287911, ['map,1,swingFootMod,full_lean,x']= 0.18452599119934, ['map,1,swingFootMod,full_lean,y']= 0.69188543650103, ['map,1,swingFootMod,full_lean,z']= 0.070662949205605, ['map,2,swingFootMod,full_lean,x']= -0.38399276990352, ['map,2,swingFootMod,full_lean,y']= 0.30830070334841, ['map,2,swingFootMod,full_lean,z']= 0.056244095906004, ['map,0,spprtFootMod,full_lean,x']= -0.032398343113561, ['map,0,spprtFootMod,full_lean,y']= -0.61849119287608, ['map,0,spprtFootMod,full_lean,z']= 0.16530596757777, ['map,1,spprtFootMod,full_lean,x']= -0.14560383003746, ['map,1,spprtFootMod,full_lean,y']= 0.42049411889245, ['map,1,spprtFootMod,full_lean,z']= 0.1153689418207, ['map,2,spprtFootMod,full_lean,x']= -0.035661667193273, ['map,2,spprtFootMod,full_lean,y']= 0.26623574209862, ['map,2,spprtFootMod,full_lean,z']= -0.30632906682919, })

elseif g_mode=='a_.01' then
	useCases.full_lean.aObjWeight = .01*useCases.full_lean.aObjWeight

	--old weight
	----iter:171 bestfvever:4.0093361026335e-06
	--accumulate({['map,0,swingFootMod,full_lean,x']= 12.014600763752, ['map,0,swingFootMod,full_lean,y']= -21.302159639514, ['map,0,swingFootMod,full_lean,z']= -4.2401571337127, ['map,1,swingFootMod,full_lean,x']= 2.173443402791, ['map,1,swingFootMod,full_lean,y']= 0.48229484095089, ['map,1,swingFootMod,full_lean,z']= 2.692148228607, ['map,2,swingFootMod,full_lean,x']= -2.1069852355478, ['map,2,swingFootMod,full_lean,y']= -3.7948725840311, ['map,2,swingFootMod,full_lean,z']= -1.3473768830841, ['map,0,spprtFootMod,full_lean,x']= 4.6458355810483, ['map,0,spprtFootMod,full_lean,y']= -18.601554084925, ['map,0,spprtFootMod,full_lean,z']= 1.3565188273006, ['map,1,spprtFootMod,full_lean,x']= -8.1147348083967, ['map,1,spprtFootMod,full_lean,y']= -20.276818809381, ['map,1,spprtFootMod,full_lean,z']= -4.8681319719619, ['map,2,spprtFootMod,full_lean,x']= 6.8811834372198, ['map,2,spprtFootMod,full_lean,y']= -13.725602818599, ['map,2,spprtFootMod,full_lean,z']= -9.2880328830831, })

	--old weight2
	----iter:122 bestfvever:4.5014631570451e-07
	--accumulate({['map,0,swingFootMod,full_lean,x']= -0.031359821017244, ['map,0,swingFootMod,full_lean,y']= -0.46863656335098, ['map,0,swingFootMod,full_lean,z']= 0.15735704595813, ['map,1,swingFootMod,full_lean,x']= 0.034975423051117, ['map,1,swingFootMod,full_lean,y']= -0.09273363571049, ['map,1,swingFootMod,full_lean,z']= 0.15143543548863, ['map,2,swingFootMod,full_lean,x']= -0.052356557199296, ['map,2,swingFootMod,full_lean,y']= 0.45628107272789, ['map,2,swingFootMod,full_lean,z']= 0.014636557465041, ['map,0,spprtFootMod,full_lean,x']= -0.062037395438721, ['map,0,spprtFootMod,full_lean,y']= -0.0091173133463752, ['map,0,spprtFootMod,full_lean,z']= -0.063182091000505, ['map,1,spprtFootMod,full_lean,x']= 0.23387257820106, ['map,1,spprtFootMod,full_lean,y']= -0.015082786503932, ['map,1,spprtFootMod,full_lean,z']= 0.010686025744954, ['map,2,spprtFootMod,full_lean,x']= 0.056544189963544, ['map,2,spprtFootMod,full_lean,y']= -0.22015420652534, ['map,2,spprtFootMod,full_lean,z']= -0.19442689209764, })

	--old weight3
	----iter:152 bestfvever:2.7840217517045e-06
	--accumulate({['map,0,swingFootMod,full_lean,x']= 0.40444090264744, ['map,0,swingFootMod,full_lean,y']= -0.081524887131208, ['map,0,swingFootMod,full_lean,z']= -0.25085111794946, ['map,1,swingFootMod,full_lean,x']= 0.015139615315164, ['map,1,swingFootMod,full_lean,y']= 0.24963476459983, ['map,1,swingFootMod,full_lean,z']= 0.12189692301261, ['map,2,swingFootMod,full_lean,x']= -0.48434965692858, ['map,2,swingFootMod,full_lean,y']= 0.74519980458429, ['map,2,swingFootMod,full_lean,z']= 0.15601390721574, ['map,0,spprtFootMod,full_lean,x']= 0.26952433523352, ['map,0,spprtFootMod,full_lean,y']= -0.50770342788367, ['map,0,spprtFootMod,full_lean,z']= 0.32179113201203, ['map,1,spprtFootMod,full_lean,x']= -0.10383970738924, ['map,1,spprtFootMod,full_lean,y']= -0.016201104736596, ['map,1,spprtFootMod,full_lean,z']= 0.29386548170974, ['map,2,spprtFootMod,full_lean,x']= 0.6681165291257, ['map,2,spprtFootMod,full_lean,y']= -0.14532416048812, ['map,2,spprtFootMod,full_lean,z']= -0.30467756972391, })

elseif g_mode=='a_.05' then
	useCases.full_lean.aObjWeight = .05*useCases.full_lean.aObjWeight

end
-------------------------------------------------

useCases.full_lean.controlParam['useCases,full_lean,aObjWeight'] = useCases.full_lean.aObjWeight

useCases.unmapControlParam(useCases.full_lean)
end

