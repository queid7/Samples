package.path=package.path..";../Samples/ysscripts/lib/?.lua" --;"..package.path
require("utilfunc")

package.path=package.path..";../Samples/ysscripts/samples/?.lua" --;"..package.path
require("useCaseMuscle_gait1956_6steps")


useCases.g2592_gait130=deepCopyTable(useCases.gait1956_6steps)

useCases.g2592_gait130.optimizerMethod='Optimizer.methods.CMAes_ys'

--useCases.g2592_gait130.endSegW=10
useCases.g2592_gait130.endSegW=15
useCases.g2592_gait130.prec=.2
--useCases.g2592_gait130.endSegW=20
--useCases.g2592_gait130.prec=.05

useCases.g2592_gait130.modelName='g2592_gait130'
useCases.g2592_gait130.grpName='g2592_gait130'
useCases.g2592_gait130.grpNames={'g2592_gait130'}

-- nearest to human MET
useCases.g2592_gait130.ddqObjWeight=10*(25-7)
useCases.g2592_gait130.aObjWeight=5000*62
useCases.g2592_gait130.lambdaObjWeight=.0001
useCases.g2592_gait130.EEobjWeight=100
useCases.g2592_gait130.EEobjWeightAngular=10
useCases.g2592_gait130.momentumWeight=10

-- the initial pendControlParam and pendOptimizationPath can be automatically generated by using createInitialPendControlParam.lua
useCases.g2592_gait130.pendControlParam=
{
	['keyframe,0,pendDesiredVel,g2592_gait130,l,z']=0,['keyframe,0,pendDesiredVel,g2592_gait130,l,x']=1,['keyframe,0,pendDesiredVel,g2592_gait130,r,z']=0,['keyframe,0,pendDesiredVel,g2592_gait130,r,x']=1,['keyframe,0,pendDesiredVel,g2592_gait130,L1,z']=0,['keyframe,0,pendDesiredVel,g2592_gait130,L1,x']=1,['keyframe,0,pendDesiredVel,g2592_gait130,R1,z']=0,['keyframe,0,pendDesiredVel,g2592_gait130,R1,x']=1,['keyframe,0,pendDesiredVel,g2592_gait130,L2,z']=0,['keyframe,0,pendDesiredVel,g2592_gait130,L2,x']=1,['keyframe,0,pendDesiredVel,ignore,0,z']=0,['keyframe,0,pendDesiredVel,ignore,0,x']=1,['keyframe,0,pendDesiredVel,ignore,1,z']=0,['keyframe,0,pendDesiredVel,ignore,1,x']=1,
	['keyframe,0,pendDesiredVel,ignore,0,x']=0.3822577513421, ['keyframe,0,pendDesiredVel,ignore,0,z']=-0.13302046663322, ['keyframe,0,pendDesiredVel,g2592_gait130,l,x']=0.81919161945436, ['keyframe,0,pendDesiredVel,g2592_gait130,l,z']=0.020891619290028, ['keyframe,0,pendDesiredVel,g2592_gait130,r,x']=0.85645963709827, ['keyframe,0,pendDesiredVel,g2592_gait130,r,z']=0.0042139930721827, ['keyframe,0,pendDesiredVel,g2592_gait130,L1,x']=0.80063815312137, ['keyframe,0,pendDesiredVel,g2592_gait130,L1,z']=0.017048291951856, ['keyframe,0,pendDesiredVel,g2592_gait130,R1,x']=0.74033626108624, ['keyframe,0,pendDesiredVel,g2592_gait130,R1,z']=-0.021515013136843, ['keyframe,0,pendDesiredVel,g2592_gait130,L2,x']=0.81629708476258, ['keyframe,0,pendDesiredVel,g2592_gait130,L2,z']=0.037041726990177, ['keyframe,0,pendDesiredVel,ignore,1,x']=0.97413014015076, ['keyframe,0,pendDesiredVel,ignore,1,z']=0.0014023451571805, 
	['keyframe,0,pendDesiredVel,ignore,0,x']=0.3803614308801, ['keyframe,0,pendDesiredVel,ignore,0,z']=-0.13337215679808, ['keyframe,0,pendDesiredVel,g2592_gait130,l,x']=0.81823835336933, ['keyframe,0,pendDesiredVel,g2592_gait130,l,z']=0.020968794091569, ['keyframe,0,pendDesiredVel,g2592_gait130,r,x']=0.85519461802555, ['keyframe,0,pendDesiredVel,g2592_gait130,r,z']=0.0040674686792773, ['keyframe,0,pendDesiredVel,g2592_gait130,L1,x']=0.79965650056089, ['keyframe,0,pendDesiredVel,g2592_gait130,L1,z']=0.017043968865199, ['keyframe,0,pendDesiredVel,g2592_gait130,R1,x']=0.73974739936668, ['keyframe,0,pendDesiredVel,g2592_gait130,R1,z']=-0.021621125883322, ['keyframe,0,pendDesiredVel,g2592_gait130,L2,x']=0.81599617326314, ['keyframe,0,pendDesiredVel,g2592_gait130,L2,z']=0.03716512377183, ['keyframe,0,pendDesiredVel,ignore,1,x']=0.9740749869161, ['keyframe,0,pendDesiredVel,ignore,1,z']=0.0014081927122805, 
}
useCases.g2592_gait130.pendOptimizationPath=
{
	firstFrames={2,101,200,299,397,496,595,691},
	segments={'ignore,0','g2592_gait130,l','g2592_gait130,r','g2592_gait130,L1','g2592_gait130,R1','g2592_gait130,L2','ignore,1'},
}
--useCases.g2592_gait130.measureOptCost=useCases.measureOptCost

useCases.g2592_gait130.segmentations=
{
	g2592_gait130={
		firstFrames={2,101,200,299,397,496,595,691},
		--        (blank)    L    R     L     R     L     (blank)
		names={             'l', 'r', 'L1', 'R1', 'L2',  },
		swingL={             0,   1,   0,    1,    0,    },
		swingR={             1,   0,   1,    0,    1,    },
		footRefL= "convertFromSwingL",
		footRefR= "convertFromSwingR",
		usePositionControl=false,
	},
}
useCases.g2592_gait130.graphParam=
{
	g2592_gait130={
		seg={            'l',  'r',  'L1', 'R1','L'},
		num_key={         3,    3,    3  ,  4,  4},
		key_first={       0,    0,    0  ,  0,  0},
		key_last={       'r', 'L1',  'R1', 'L', 'R1'},
	},
}
useCases.g2592_gait130.graph=
{
	{
		"addInterpolatedSegment",
		grpName="g2592_gait130",
		name="L",
		seg0={"g2592_gait130", "L2"},
		seg1={"g2592_gait130", "L1"},
		startWeight=0, endWeight=1
	},
	{"connectMulti", "g2592_gait130", "l", "r", "L1", "R1", "L", "R1"},
	--{"initialSegment", "g2592_gait130", "l"}
	{"initialSegment", "g2592_gait130", "R1"}
}
useCases.g2592_gait130.segNames=
{
	"l", "r", "L1", 'R1','L',
}

do 
	useCases.g2592_gait130.controlParam={
		['useCases,g2592_gait130,COMobjWeight']=0, 
		['useCases,g2592_gait130,conservativeW']=1, 
		--['useCases,g2592_gait130,contactMargin']=0.01, 
		['useCases,g2592_gait130,dotMomentumScale']=0.3, 
		['useCases,g2592_gait130,excludeRoot']=true, 
		['useCases,g2592_gait130,headControlWeight']=0, 
		['useCases,g2592_gait130,k_d_HEAD']=14, 
		['useCases,g2592_gait130,k_p_HEAD']=0, 
		['useCases,g2592_gait130,k_d_EE']=24, 
		['useCases,g2592_gait130,k_d_momentum']=10, 
		['useCases,g2592_gait130,k_p_EE']=120, 

		--['useCases,g2592_gait130,maxPenetratingVel']=0, 
		['useCases,g2592_gait130,momentumThr']=50, 
		['useCases,g2592_gait130,noComvelDependentFootAdjustment']=true, 
		['useCases,g2592_gait130,noIntersectionPrevenction']=true, 
		['useCases,g2592_gait130,numericalDerivDmot']=true, 
		['useCases,g2592_gait130,perClassContactMargin']=1, 
		['useCases,g2592_gait130,turnGain']=10, 
		['useCases,g2592_gait130,velMarginOffset']=0, 

		['useCases,g2592_gait130,actuationType']      = useCases.g2592_gait130.actuationType,
		['useCases,g2592_gait130,k_p_ID']             = useCases.g2592_gait130.k_p_ID,
		['useCases,g2592_gait130,k_d_ID']             = useCases.g2592_gait130.k_d_ID,
		['useCases,g2592_gait130,ddqObjWeight']       = useCases.g2592_gait130.ddqObjWeight,
		['useCases,g2592_gait130,tauObjWeight']       = useCases.g2592_gait130.tauObjWeight,
		['useCases,g2592_gait130,ftObjWeight']        = useCases.g2592_gait130.ftObjWeight,
		['useCases,g2592_gait130,aObjWeight']         = useCases.g2592_gait130.aObjWeight,
		['useCases,g2592_gait130,lambdaObjWeight']    = useCases.g2592_gait130.lambdaObjWeight,
		['useCases,g2592_gait130,EEobjWeight']        = useCases.g2592_gait130.EEobjWeight,
		['useCases,g2592_gait130,EEobjWeightAngular'] = useCases.g2592_gait130.EEobjWeightAngular,
		['useCases,g2592_gait130,momentumWeight']     = useCases.g2592_gait130.momentumWeight,
		['useCases,g2592_gait130,tauMax']             = useCases.g2592_gait130.tauMax,
		['useCases,g2592_gait130,ftMax']              = useCases.g2592_gait130.ftMax,
	}

	local function accumulate(cp_mod)
		local useCase=useCases.g2592_gait130
		useCases.accumulate(useCase, cp_mod)
	end

	accumulate({
		----['keyframe,0,footRmod,g2592_gait130,l,y']  = 0.,
		--['keyframe,1,footRmod,g2592_gait130,l,y']  = 0.05,
		--['keyframe,0,footLmod,g2592_gait130,r,y']  = 0.05,
		--['keyframe,1,footLmod,g2592_gait130,r,y']  = 0.1,
		--['keyframe,0,footRmod,g2592_gait130,L1,y'] = 0.0,
		--['keyframe,1,footRmod,g2592_gait130,L1,y'] = 0.1,
		--['map,0,swingFootMod,g2592_gait130,y']     = 0.1,
		--['map,1,swingFootMod,g2592_gait130,y']     = 0.3,
		--['map,2,swingFootMod,g2592_gait130,y']     = 0.2,

		------['keyframe,0,footLmod,g2592_gait130,l,y']  = 0.,
		------['keyframe,1,footLmod,g2592_gait130,l,y']  = 0.,
		----['keyframe,0,footRmod,g2592_gait130,r,y']  = 0.,
		----['keyframe,1,footRmod,g2592_gait130,r,y']  = -.1,
		----['keyframe,0,footLmod,g2592_gait130,L1,y'] = 0.,
		----['keyframe,1,footLmod,g2592_gait130,L1,y'] = -.1,
		----['map,0,spprtFootMod,g2592_gait130,y']     = 0.,
		----['map,1,spprtFootMod,g2592_gait130,y']     = 0.,
		----['map,2,spprtFootMod,g2592_gait130,y']     = 0.,
	})
	useCases.unmapControlParam(useCases.g2592_gait130)


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

	if g_mode=='' then
	else

		if string.find(g_mode, 'cont_')~=nil then
			--iter:233 bestfvever:3.9107901094599e-07
			accumulate({['map,0,swingFootMod,g2592_gait130,x']= -0.19260809813539, ['map,0,swingFootMod,g2592_gait130,y']= -0.2678698527285, ['map,0,swingFootMod,g2592_gait130,z']= 0.39270879747744, ['map,1,swingFootMod,g2592_gait130,x']= 0.29956185318915, ['map,1,swingFootMod,g2592_gait130,y']= 0.084301191558227, ['map,1,swingFootMod,g2592_gait130,z']= -0.0033021297875873, ['map,2,swingFootMod,g2592_gait130,x']= -0.056554082472535, ['map,2,swingFootMod,g2592_gait130,y']= 0.062772658538821, ['map,2,swingFootMod,g2592_gait130,z']= 0.11418129072202, ['map,0,spprtFootMod,g2592_gait130,x']= 0.10207767062676, ['map,0,spprtFootMod,g2592_gait130,y']= -0.13220686186758, ['map,0,spprtFootMod,g2592_gait130,z']= 0.26029167304072, ['map,1,spprtFootMod,g2592_gait130,x']= 0.32699176281187, ['map,1,spprtFootMod,g2592_gait130,y']= 0.16514243689712, ['map,1,spprtFootMod,g2592_gait130,z']= 0.035345947692649, ['map,2,spprtFootMod,g2592_gait130,x']= -0.33451420741516, ['map,2,spprtFootMod,g2592_gait130,y']= -0.42767628178044, ['map,2,spprtFootMod,g2592_gait130,z']= 0.097795024242233, })
			useCases.g2592_gait130.prec=.1
		end

		if string.find(g_mode, 'a.1')~=nil then
			useCases.g2592_gait130.aObjWeight = .1*useCases.g2592_gait130.aObjWeight
		elseif string.find(g_mode, 'a1')~=nil then
			useCases.g2592_gait130.aObjWeight = 1*useCases.g2592_gait130.aObjWeight
		end

		if string.find(g_mode, 'tl.5')~=nil then
			useCases.g2592_gait130.tenLenRatio = .5
		elseif string.find(g_mode, 'tl.6')~=nil then
			useCases.g2592_gait130.tenLenRatio = .6
		elseif string.find(g_mode, 'tl.7')~=nil then
			useCases.g2592_gait130.tenLenRatio = .7
		elseif string.find(g_mode, 'tl.8')~=nil then
			useCases.g2592_gait130.tenLenRatio = .8
		elseif string.find(g_mode, 'tl.9')~=nil then
			useCases.g2592_gait130.tenLenRatio = .9
		end

		if string.find(g_mode, 'tl2.5')~=nil then
			useCases.g2592_gait130.tenLenRatio2 = .5
		elseif string.find(g_mode, 'tl2.6')~=nil then
			useCases.g2592_gait130.tenLenRatio2 = .6
		elseif string.find(g_mode, 'tl2.7')~=nil then
			useCases.g2592_gait130.tenLenRatio2 = .7
		elseif string.find(g_mode, 'tl2.8')~=nil then
			useCases.g2592_gait130.tenLenRatio2 = .8
		elseif string.find(g_mode, 'tl2.9')~=nil then
			useCases.g2592_gait130.tenLenRatio2 = .9
		end

		if string.find(g_mode, 'ol.5')~=nil then
			useCases.g2592_gait130.optLenRatio = .5
		elseif string.find(g_mode, 'ol.6')~=nil then
			useCases.g2592_gait130.optLenRatio = .6
		elseif string.find(g_mode, 'ol.7')~=nil then
			useCases.g2592_gait130.optLenRatio = .7
		elseif string.find(g_mode, 'ol.8')~=nil then
			useCases.g2592_gait130.optLenRatio = .8
		elseif string.find(g_mode, 'ol.9')~=nil then
			useCases.g2592_gait130.optLenRatio = .9
		end

		if string.find(g_mode, 'tlol.5')~=nil then
			useCases.g2592_gait130.tenLenRatio = .5
			useCases.g2592_gait130.optLenRatio = .5
		elseif string.find(g_mode, 'tlol.82')~=nil then
			useCases.g2592_gait130.tenLenRatio = .82
			useCases.g2592_gait130.optLenRatio = .82
		elseif string.find(g_mode, 'tlol.83')~=nil then
			useCases.g2592_gait130.tenLenRatio = .83
			useCases.g2592_gait130.optLenRatio = .83
		elseif string.find(g_mode, 'tlol.84')~=nil then
			useCases.g2592_gait130.tenLenRatio = .84
			useCases.g2592_gait130.optLenRatio = .84
		elseif string.find(g_mode, 'tlol.85')~=nil then
			useCases.g2592_gait130.tenLenRatio = .85
			useCases.g2592_gait130.optLenRatio = .85
		elseif string.find(g_mode, 'tlol.86')~=nil then
			useCases.g2592_gait130.tenLenRatio = .86
			useCases.g2592_gait130.optLenRatio = .86
		elseif string.find(g_mode, 'tlol.87')~=nil then
			useCases.g2592_gait130.tenLenRatio = .87
			useCases.g2592_gait130.optLenRatio = .87
		elseif string.find(g_mode, 'tlol.88')~=nil then
			useCases.g2592_gait130.tenLenRatio = .88
			useCases.g2592_gait130.optLenRatio = .88
		elseif string.find(g_mode, 'tlol.89')~=nil then
			useCases.g2592_gait130.tenLenRatio = .89
			useCases.g2592_gait130.optLenRatio = .89
		elseif string.find(g_mode, 'tlol.8.9')~=nil then
			useCases.g2592_gait130.tenLenRatio = .8
			useCases.g2592_gait130.optLenRatio = .9
		elseif string.find(g_mode, 'tlol.6')~=nil then
			useCases.g2592_gait130.tenLenRatio = .6
			useCases.g2592_gait130.optLenRatio = .6
		elseif string.find(g_mode, 'tlol.7')~=nil then
			useCases.g2592_gait130.tenLenRatio = .7
			useCases.g2592_gait130.optLenRatio = .7
		elseif string.find(g_mode, 'tlol.8')~=nil then
			useCases.g2592_gait130.tenLenRatio = .8
			useCases.g2592_gait130.optLenRatio = .8
		elseif string.find(g_mode, 'tlol.9')~=nil then
			useCases.g2592_gait130.tenLenRatio = .9
			useCases.g2592_gait130.optLenRatio = .9
		end

		if string.find(g_mode, 'tlol2.5')~=nil then
			useCases.g2592_gait130.tenLenRatio2 = .5
			useCases.g2592_gait130.optLenRatio2 = .5
		elseif string.find(g_mode, 'tlol2.82')~=nil then
			useCases.g2592_gait130.tenLenRatio2 = .82
			useCases.g2592_gait130.optLenRatio2 = .82
		elseif string.find(g_mode, 'tlol2.83')~=nil then
			useCases.g2592_gait130.tenLenRatio2 = .83
			useCases.g2592_gait130.optLenRatio2 = .83
		elseif string.find(g_mode, 'tlol2.84')~=nil then
			useCases.g2592_gait130.tenLenRatio2 = .84
			useCases.g2592_gait130.optLenRatio2 = .84
		elseif string.find(g_mode, 'tlol2.85')~=nil then
			useCases.g2592_gait130.tenLenRatio2 = .85
			useCases.g2592_gait130.optLenRatio2 = .85
		elseif string.find(g_mode, 'tlol2.86')~=nil then
			useCases.g2592_gait130.tenLenRatio2 = .86
			useCases.g2592_gait130.optLenRatio2 = .86
		elseif string.find(g_mode, 'tlol2.87')~=nil then
			useCases.g2592_gait130.tenLenRatio2 = .87
			useCases.g2592_gait130.optLenRatio2 = .87
		elseif string.find(g_mode, 'tlol2.88')~=nil then
			useCases.g2592_gait130.tenLenRatio2 = .88
			useCases.g2592_gait130.optLenRatio2 = .88
		elseif string.find(g_mode, 'tlol2.89')~=nil then
			useCases.g2592_gait130.tenLenRatio2 = .89
			useCases.g2592_gait130.optLenRatio2 = .89
		elseif string.find(g_mode, 'tlol2.8.9')~=nil then
			useCases.g2592_gait130.tenLenRatio2 = .8
			useCases.g2592_gait130.optLenRatio2 = .9
		elseif string.find(g_mode, 'tlol2.6')~=nil then
			useCases.g2592_gait130.tenLenRatio2 = .6
			useCases.g2592_gait130.optLenRatio2 = .6
		elseif string.find(g_mode, 'tlol2.7')~=nil then
			useCases.g2592_gait130.tenLenRatio2 = .7
			useCases.g2592_gait130.optLenRatio2 = .7
		elseif string.find(g_mode, 'tlol2.8')~=nil then
			useCases.g2592_gait130.tenLenRatio2 = .8
			useCases.g2592_gait130.optLenRatio2 = .8
		elseif string.find(g_mode, 'tlol2.9')~=nil then
			useCases.g2592_gait130.tenLenRatio2 = .9
			useCases.g2592_gait130.optLenRatio2 = .9
		end

		if string.find(g_mode, 'mf.2')~=nil then
			useCases.g2592_gait130.weakRatio = .2
		elseif string.find(g_mode, 'mf.1')~=nil then
			useCases.g2592_gait130.weakRatio = .1
		end

		if string.find(g_mode, 'hamst_psoas_tlol')~=nil then
			useCases.g2592_gait130.tenLenMuscles=concat_array(hamstrings, psoases)
			useCases.g2592_gait130.optLenMuscles=concat_array(hamstrings, psoases)

		elseif string.find(g_mode, 'biankpf_tl')~=nil then
			useCases.g2592_gait130.tenLenMuscles=concat_array(R_ankle_pf,  L_ankle_pf)

		elseif string.find(g_mode, 'biankpf_ol')~=nil then
			useCases.g2592_gait130.optLenMuscles=concat_array(R_ankle_pf,  L_ankle_pf)

		elseif string.find(g_mode, 'hamst_psoas_tl')~=nil then
			useCases.g2592_gait130.tenLenMuscles=concat_array(hamstrings, psoases)

		elseif string.find(g_mode, 'hamst_psoas_ol')~=nil then
			useCases.g2592_gait130.optLenMuscles=concat_array(hamstrings, psoases)

		elseif string.find(g_mode, 'hamst_tlol')~=nil then
			useCases.g2592_gait130.tenLenMuscles=hamstrings
			useCases.g2592_gait130.optLenMuscles=hamstrings

		elseif string.find(g_mode, 'hamst_tl')~=nil then
			useCases.g2592_gait130.tenLenMuscles=hamstrings

		elseif string.find(g_mode, 'hamst_ol')~=nil then
			useCases.g2592_gait130.optLenMuscles=hamstrings

		elseif string.find(g_mode, 'psoas_tlol')~=nil then
			useCases.g2592_gait130.tenLenMuscles=psoases
			useCases.g2592_gait130.optLenMuscles=psoases

		elseif string.find(g_mode, 'psoas_tl')~=nil then
			useCases.g2592_gait130.tenLenMuscles=psoases

		elseif string.find(g_mode, 'psoas_ol')~=nil then
			useCases.g2592_gait130.optLenMuscles=psoases
		end

		if string.find(g_mode, 'psoas_tl2')~=nil then
			useCases.g2592_gait130.tenLenMuscles2=psoases
		elseif string.find(g_mode, 'psoas_tlol2')~=nil then
			useCases.g2592_gait130.tenLenMuscles2=psoases
			useCases.g2592_gait130.optLenMuscles2=psoases
		end

		if string.find(g_mode, 'ankPF_mf')~=nil then
			useCases.g2592_gait130.weakenMuscles=concat_array(R_ankle_pf,  L_ankle_pf)

		elseif string.find(g_mode, 'ankPF')~=nil then
			useCases.g2592_gait130.weakenMuscles=concat_array(R_ankle_pf,  L_ankle_pf)
			if string.find(g_mode, 'ankPF.15')~=nil then
				useCases.g2592_gait130.weakRatio = .15
			elseif string.find(g_mode, 'ankPF.1')~=nil then
				useCases.g2592_gait130.weakRatio = .1
			elseif string.find(g_mode, 'ankPF.2')~=nil then
				useCases.g2592_gait130.weakRatio = .2
			elseif string.find(g_mode, 'ankPF.3')~=nil then
				useCases.g2592_gait130.weakRatio = .3
			end

		elseif string.find(g_mode, 'ankDF')~=nil then
			useCases.g2592_gait130.weakenMuscles=concat_array(R_ankle_df,  L_ankle_df)
			if string.find(g_mode, 'ankDF.1')~=nil then
				useCases.g2592_gait130.weakRatio = .1
			elseif string.find(g_mode, 'ankDF.01')~=nil then
				useCases.g2592_gait130.weakRatio = .01
			elseif string.find(g_mode, 'ankDF.05')~=nil then
				useCases.g2592_gait130.weakRatio = .05
			elseif string.find(g_mode, 'ankDF.2')~=nil then
				useCases.g2592_gait130.weakRatio = .2
			elseif string.find(g_mode, 'ankDF.3')~=nil then
				useCases.g2592_gait130.weakRatio = .3
			end

		elseif string.find(g_mode, 'ankLPF')~=nil then
			useCases.g2592_gait130.weakenMuscles=L_ankle_pf
			if string.find(g_mode, 'ankLPF.1')~=nil then
				useCases.g2592_gait130.weakRatio = .1
			elseif string.find(g_mode, 'ankLPF.01')~=nil then
				useCases.g2592_gait130.weakRatio = .01
			elseif string.find(g_mode, 'ankLPF.2')~=nil then
				useCases.g2592_gait130.weakRatio = .2
			end

		elseif string.find(g_mode, 'ankLDF')~=nil then
			useCases.g2592_gait130.weakenMuscles=L_ankle_df
			if string.find(g_mode, 'ankLDF.1')~=nil then
				useCases.g2592_gait130.weakRatio = .1
			elseif string.find(g_mode, 'ankLDF.01')~=nil then
				useCases.g2592_gait130.weakRatio = .01
			elseif string.find(g_mode, 'ankLDF.02')~=nil then
				useCases.g2592_gait130.weakRatio = .02
			elseif string.find(g_mode, 'ankLDF.03')~=nil then
				useCases.g2592_gait130.weakRatio = .03
			elseif string.find(g_mode, 'ankLDF.04')~=nil then
				useCases.g2592_gait130.weakRatio = .04
			elseif string.find(g_mode, 'ankLDF.05')~=nil then
				useCases.g2592_gait130.weakRatio = .05
			elseif string.find(g_mode, 'ankLDF.2')~=nil then
				useCases.g2592_gait130.weakRatio = .2
			elseif string.find(g_mode, 'ankLDF.3')~=nil then
				useCases.g2592_gait130.weakRatio = .3
			end

		elseif string.find(g_mode, 'hipAB')~=nil then
			useCases.g2592_gait130.weakenMuscles=concat_array(R_hip_abd, L_hip_abd)
			if string.find(g_mode, 'hipAB.2')~=nil then
				useCases.g2592_gait130.weakRatio = .2
			elseif string.find(g_mode, 'hipAB.15')~=nil then
				useCases.g2592_gait130.weakRatio = .15
			elseif string.find(g_mode, 'hipAB.3')~=nil then
				useCases.g2592_gait130.weakRatio = .3
			elseif string.find(g_mode, 'hipAB.4')~=nil then
				useCases.g2592_gait130.weakRatio = .4
			end

		elseif string.find(g_mode, 'hipLAB')~=nil then
			useCases.g2592_gait130.weakenMuscles=L_hip_abd
			if string.find(g_mode, 'hipLAB.2')~=nil then
				useCases.g2592_gait130.weakRatio = .2
			elseif string.find(g_mode, 'hipLAB.3')~=nil then
				useCases.g2592_gait130.weakRatio = .3
			end

		elseif string.find(g_mode, 'gluts')~=nil then
			useCases.g2592_gait130.weakenMuscles = concat_array(glut_meds,glut_mins)
			if string.find(g_mode, 'gluts.15')~=nil then
				useCases.g2592_gait130.weakRatio = .15
			elseif string.find(g_mode, 'gluts.1')~=nil then
				useCases.g2592_gait130.weakRatio = .1
			elseif string.find(g_mode, 'gluts.2')~=nil then
				useCases.g2592_gait130.weakRatio = .2
			elseif string.find(g_mode, 'gluts.3')~=nil then
				useCases.g2592_gait130.weakRatio = .3
			elseif string.find(g_mode, 'gluts.4')~=nil then
				useCases.g2592_gait130.weakRatio = .4
			elseif string.find(g_mode, 'gluts.5')~=nil then
				useCases.g2592_gait130.weakRatio = .5
			end

		elseif string.find(g_mode, 'lglut')~=nil then
			useCases.g2592_gait130.weakenMuscles = concat_array(lglut_meds,lglut_mins)
			if string.find(g_mode, 'lglut.1')~=nil then
				useCases.g2592_gait130.weakRatio = .1
			elseif string.find(g_mode, 'lglut.2')~=nil then
				useCases.g2592_gait130.weakRatio = .2
			elseif string.find(g_mode, 'lglut.3')~=nil then
				useCases.g2592_gait130.weakRatio = .3
			elseif string.find(g_mode, 'lglut.4')~=nil then
				useCases.g2592_gait130.weakRatio = .4
			elseif string.find(g_mode, 'lglut.5')~=nil then
				useCases.g2592_gait130.weakRatio = .5
			elseif string.find(g_mode, 'lglut.6')~=nil then
				useCases.g2592_gait130.weakRatio = .6
			end

		elseif string.find(g_mode, 'kneeEX')~=nil then
			useCases.g2592_gait130.weakenMuscles=concat_array(R_knee_ext,L_knee_ext)
			if string.find(g_mode, 'kneeEX.05')~=nil then
				useCases.g2592_gait130.weakRatio = .05
			elseif string.find(g_mode, 'kneeEX.15')~=nil then
				useCases.g2592_gait130.weakRatio = .15
			elseif string.find(g_mode, 'kneeEX.1')~=nil then
				useCases.g2592_gait130.weakRatio = .1
			elseif string.find(g_mode, 'kneeEX.2')~=nil then
				useCases.g2592_gait130.weakRatio = .2
			elseif string.find(g_mode, 'kneeEX.3')~=nil then
				useCases.g2592_gait130.weakRatio = .3
			end

		elseif string.find(g_mode, 'kneeLEX')~=nil then
			useCases.g2592_gait130.weakenMuscles=L_knee_ext
			if string.find(g_mode, 'kneeLEX.05')~=nil then
				useCases.g2592_gait130.weakRatio = .05
			elseif string.find(g_mode, 'kneeLEX.15')~=nil then
				useCases.g2592_gait130.weakRatio = .15
			elseif string.find(g_mode, 'kneeLEX.1')~=nil then
				useCases.g2592_gait130.weakRatio = .1
			elseif string.find(g_mode, 'kneeLEX.2')~=nil then
				useCases.g2592_gait130.weakRatio = .2
			end

		elseif string.find(g_mode, 'hipEX')~=nil then
			useCases.g2592_gait130.weakenMuscles=concat_array(R_hip_ext,L_hip_ext)
			if string.find(g_mode, 'hipEX.1')~=nil then
				useCases.g2592_gait130.weakRatio = .1
			elseif string.find(g_mode, 'hipEX.2')~=nil then
				useCases.g2592_gait130.weakRatio = .2
			elseif string.find(g_mode, 'hipEX.3')~=nil then
				useCases.g2592_gait130.weakRatio = .3
			end

		--elseif string.find(g_mode, 'hamst')~=nil then
			--useCases.g2592_gait130.weakenMuscles=hamstrings
			--if string.find(g_mode, 'hamst.1')~=nil then
				--useCases.g2592_gait130.weakRatio = .1
			--elseif string.find(g_mode, 'hamst.2')~=nil then
				--useCases.g2592_gait130.weakRatio = .2
			--elseif string.find(g_mode, 'hamst.3')~=nil then
				--useCases.g2592_gait130.weakRatio = .3
			--end

		elseif string.find(g_mode, 'rf')~=nil then
			useCases.g2592_gait130.weakenMuscles=rect_fems
			if string.find(g_mode, 'rf.1')~=nil then
				useCases.g2592_gait130.weakRatio = .1
			elseif string.find(g_mode, 'rf.2')~=nil then
				useCases.g2592_gait130.weakRatio = .2
				useCases.g2592_gait130.weakenMuscles=rect_fems
			end

		--elseif string.find(g_mode, 'psoas')~=nil then
			--useCases.g2592_gait130.weakenMuscles=psoases
			--if string.find(g_mode, 'psoas.1')~=nil then
				--useCases.g2592_gait130.weakRatio = .1
			--elseif string.find(g_mode, 'psoas.2')~=nil then
				--useCases.g2592_gait130.weakRatio = .2
			--end

		end

		if string.find(g_mode, 'efrtm5')~=nil then
			useCases.g2592_gait130.cmaEffortWeight = 1e-5
		elseif string.find(g_mode, 'efrtm4')~=nil then
			useCases.g2592_gait130.cmaEffortWeight = 1e-4
		elseif string.find(g_mode, 'efrtm3')~=nil then
			useCases.g2592_gait130.cmaEffortWeight = 1e-3
		elseif string.find(g_mode, 'efrtm6')~=nil then
			useCases.g2592_gait130.cmaEffortWeight = 1e-6
		end

		if string.find(g_mode, 'efcim5')~=nil then
			useCases.g2592_gait130.cmaEffortDistWeight = 1e-5
		elseif string.find(g_mode, 'efcim4')~=nil then
			useCases.g2592_gait130.cmaEffortDistWeight = 1e-4
		elseif string.find(g_mode, 'efcim3')~=nil then
			useCases.g2592_gait130.cmaEffortDistWeight = 1e-3
		elseif string.find(g_mode, 'efcim6')~=nil then
			useCases.g2592_gait130.cmaEffortDistWeight = 1e-6
		elseif string.find(g_mode, 'efcim2')~=nil then
			useCases.g2592_gait130.cmaEffortDistWeight = 1e-2
		elseif string.find(g_mode, 'efcim1')~=nil then
			useCases.g2592_gait130.cmaEffortDistWeight = 1e-1
		end

		if string.find(g_mode, 'lfcfm4')~=nil then
			useCases.g2592_gait130.cmaLFootCFWeight = 1e-4
		elseif string.find(g_mode, 'lfcfm5')~=nil then
			useCases.g2592_gait130.cmaLFootCFWeight = 1e-5
		elseif string.find(g_mode, 'lfcfm6')~=nil then
			useCases.g2592_gait130.cmaLFootCFWeight = 1e-6
		elseif string.find(g_mode, 'lfcfm2')~=nil then
			useCases.g2592_gait130.cmaLFootCFWeight = 1e-2
		elseif string.find(g_mode, 'lfcfm3')~=nil then
			useCases.g2592_gait130.cmaLFootCFWeight = 1e-3
		end

		if string.find(g_mode, 'lankpffom4')~=nil then
			useCases.g2592_gait130.cmaLAnkPFFoWeight = 1e-4
		elseif string.find(g_mode, 'lankpffom5')~=nil then
			useCases.g2592_gait130.cmaLAnkPFFoWeight = 1e-5
		elseif string.find(g_mode, 'lankpffom6')~=nil then
			useCases.g2592_gait130.cmaLAnkPFFoWeight = 1e-6
		elseif string.find(g_mode, 'lankpffom3')~=nil then
			useCases.g2592_gait130.cmaLAnkPFFoWeight = 1e-3
		end

		if string.find(g_mode, 'lrankpffom4')~=nil then
			useCases.g2592_gait130.cmaLAnkPFFoWeight = 1e-4
			useCases.g2592_gait130.cmaRAnkPFFoWeight = 1e-4
		elseif string.find(g_mode, 'lrankpffom5')~=nil then
			useCases.g2592_gait130.cmaLAnkPFFoWeight = 1e-5
			useCases.g2592_gait130.cmaRAnkPFFoWeight = 1e-5
		elseif string.find(g_mode, 'lrankpffom6')~=nil then
			useCases.g2592_gait130.cmaLAnkPFFoWeight = 1e-6
			useCases.g2592_gait130.cmaRAnkPFFoWeight = 1e-6
		elseif string.find(g_mode, 'lrankpffom3')~=nil then
			useCases.g2592_gait130.cmaLAnkPFFoWeight = 1e-3
			useCases.g2592_gait130.cmaRAnkPFFoWeight = 1e-3
		end

		if string.find(g_mode, 'clmpm5')~=nil then
			useCases.g2592_gait130.cmaPoseClamp = 1e-5
		elseif string.find(g_mode, 'clmpm4')~=nil then
			useCases.g2592_gait130.cmaPoseClamp = 1e-4
		elseif string.find(g_mode, 'clmpm3')~=nil then
			useCases.g2592_gait130.cmaPoseClamp = 1e-3
		elseif string.find(g_mode, 'clmpm6')~=nil then
			useCases.g2592_gait130.cmaPoseClamp = 1e-6
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

		if string.find(g_mode, 'a.1_hamst_psoas_tl.8_ankPF_mf.2_push')~=nil then
			--iter:179 bestfvever:1.6876442520237e-06
			accumulate({['map,0,swingFootMod,g2592_gait130,x']= -0.086404799999669, ['map,0,swingFootMod,g2592_gait130,y']= 0.19240951905766, ['map,0,swingFootMod,g2592_gait130,z']= 0.55579676273367, ['map,1,swingFootMod,g2592_gait130,x']= 0.66963538731804, ['map,1,swingFootMod,g2592_gait130,y']= 0.17855220520046, ['map,1,swingFootMod,g2592_gait130,z']= -0.07063979441948, ['map,2,swingFootMod,g2592_gait130,x']= -0.52133779009622, ['map,2,swingFootMod,g2592_gait130,y']= 0.27912539029751, ['map,2,swingFootMod,g2592_gait130,z']= 0.016619112095564, ['map,0,spprtFootMod,g2592_gait130,x']= 0.050424489613135, ['map,0,spprtFootMod,g2592_gait130,y']= -1.0259768713766, ['map,0,spprtFootMod,g2592_gait130,z']= -0.095664952002717, ['map,1,spprtFootMod,g2592_gait130,x']= -0.031728504928429, ['map,1,spprtFootMod,g2592_gait130,y']= -0.59243930537485, ['map,1,spprtFootMod,g2592_gait130,z']= -0.37463741424218, ['map,2,spprtFootMod,g2592_gait130,x']= -0.12630034785026, ['map,2,spprtFootMod,g2592_gait130,y']= -0.40255332181395, ['map,2,spprtFootMod,g2592_gait130,z']= 0.47607683649369, })

			useCases.g2592_gait130.impulse = {
				{startframe=100, dir=vector3(0,0,1), mag=magn, dur=.2},
				{startframe=300, dir=vector3(0,0,-1), mag=magn, dur=.2},
				{startframe=500, dir=vector3(1,0,0), mag=magn, dur=.2},
				{startframe=700, dir=vector3(-1,0,0), mag=magn, dur=.2},
				{startframe=900, dir=vector3(0,0,1), mag=magn, dur=.2},
				{startframe=1100, dir=vector3(0,0,-1), mag=magn, dur=.2},
			}
		elseif string.find(g_mode, 'push')~=nil then
			--iter:48 bestfvever:3.9292625218392e-07
			accumulate({['map,0,swingFootMod,g2592_gait130,x']= -0.19420595417091, ['map,0,swingFootMod,g2592_gait130,y']= -0.25949286568558, ['map,0,swingFootMod,g2592_gait130,z']= 0.40301463535033, ['map,1,swingFootMod,g2592_gait130,x']= 0.31613093992209, ['map,1,swingFootMod,g2592_gait130,y']= 0.087865249461949, ['map,1,swingFootMod,g2592_gait130,z']= -0.016804854132892, ['map,2,swingFootMod,g2592_gait130,x']= -0.068857072136694, ['map,2,swingFootMod,g2592_gait130,y']= 0.045070490813677, ['map,2,swingFootMod,g2592_gait130,z']= 0.13588722076236, ['map,0,spprtFootMod,g2592_gait130,x']= 0.11583753073716, ['map,0,spprtFootMod,g2592_gait130,y']= -0.15107580505513, ['map,0,spprtFootMod,g2592_gait130,z']= 0.24611417993075, ['map,1,spprtFootMod,g2592_gait130,x']= 0.33152751777068, ['map,1,spprtFootMod,g2592_gait130,y']= 0.14741010608262, ['map,1,spprtFootMod,g2592_gait130,z']= 0.020319149256821, ['map,2,spprtFootMod,g2592_gait130,x']= -0.32725465062968, ['map,2,spprtFootMod,g2592_gait130,y']= -0.4270487583634, ['map,2,spprtFootMod,g2592_gait130,z']= 0.12285781409532, })

			----iter:71 bestfvever:4.1712611663016e-07
			--accumulate({['map,0,swingFootMod,g2592_gait130,x']= -0.26752749797807, ['map,0,swingFootMod,g2592_gait130,y']= -0.24794282627597, ['map,0,swingFootMod,g2592_gait130,z']= 0.31283036770757, ['map,1,swingFootMod,g2592_gait130,x']= 0.19282405247122, ['map,1,swingFootMod,g2592_gait130,y']= 0.090001931244552, ['map,1,swingFootMod,g2592_gait130,z']= -0.01493562911679, ['map,2,swingFootMod,g2592_gait130,x']= -0.020852766778446, ['map,2,swingFootMod,g2592_gait130,y']= 0.17314236185439, ['map,2,swingFootMod,g2592_gait130,z']= 0.098640832297545, ['map,0,spprtFootMod,g2592_gait130,x']= 0.14985886882304, ['map,0,spprtFootMod,g2592_gait130,y']= -0.20653314412159, ['map,0,spprtFootMod,g2592_gait130,z']= 0.19864830966946, ['map,1,spprtFootMod,g2592_gait130,x']= 0.29093864494556, ['map,1,spprtFootMod,g2592_gait130,y']= 0.10355675994239, ['map,1,spprtFootMod,g2592_gait130,z']= 0.060057481334429, ['map,2,spprtFootMod,g2592_gait130,x']= -0.41248341306351, ['map,2,spprtFootMod,g2592_gait130,y']= -0.38677569900179, ['map,2,spprtFootMod,g2592_gait130,z']= 0.1095600491155, })

			useCases.g2592_gait130.impulse = {
				{startframe=100, dir=vector3(0,0,1), mag=magn, dur=.2},
				{startframe=300, dir=vector3(0,0,-1), mag=magn, dur=.2},
				{startframe=500, dir=vector3(1,0,0), mag=magn, dur=.2},
				{startframe=700, dir=vector3(-1,0,0), mag=magn, dur=.2},
				{startframe=900, dir=vector3(0,0,1), mag=magn, dur=.2},
				{startframe=1100, dir=vector3(0,0,-1), mag=magn, dur=.2},
			}
		end

		if string.find(g_mode, 'age30')~=nil then
			useCases.g2592_gait130.age = 30
		elseif string.find(g_mode, 'age50')~=nil then
			useCases.g2592_gait130.age = 50
		elseif string.find(g_mode, 'age70')~=nil then
			useCases.g2592_gait130.age = 70
		end

		if g_mode=='default' then
		elseif g_mode=='a.1' then
			--iter:132 bestfvever:9.7313103173022e-07
			accumulate({['map,0,swingFootMod,g2592_gait130,x']= -0.026152669254993, ['map,0,swingFootMod,g2592_gait130,y']= -0.35224275274898, ['map,0,swingFootMod,g2592_gait130,z']= -0.0060265196393804, ['map,1,swingFootMod,g2592_gait130,x']= 0.1049470739511, ['map,1,swingFootMod,g2592_gait130,y']= 0.13631837021047, ['map,1,swingFootMod,g2592_gait130,z']= 0.009395930062586, ['map,2,swingFootMod,g2592_gait130,x']= -0.16694679161458, ['map,2,swingFootMod,g2592_gait130,y']= 0.050646766165668, ['map,2,swingFootMod,g2592_gait130,z']= -0.00092102492681547, ['map,0,spprtFootMod,g2592_gait130,x']= -0.11323534477161, ['map,0,spprtFootMod,g2592_gait130,y']= 0.18437135674636, ['map,0,spprtFootMod,g2592_gait130,z']= 0.15328660371207, ['map,1,spprtFootMod,g2592_gait130,x']= -0.12462465220596, ['map,1,spprtFootMod,g2592_gait130,y']= -0.15763747002747, ['map,1,spprtFootMod,g2592_gait130,z']= 0.040168042969752, ['map,2,spprtFootMod,g2592_gait130,x']= 0.0840565444479, ['map,2,spprtFootMod,g2592_gait130,y']= -0.37735469252441, ['map,2,spprtFootMod,g2592_gait130,z']= 0.11386053079041, })

		elseif g_mode=='a.1_ankPF.2_efcim5' then
		elseif g_mode=='a.1_hamst_psoas_tl.8_ankPF_mf.2_efcim5' then
		elseif g_mode=='a.1_knee20_m3' then
			useCases.g2592_gait130.cmaLRKneeLimitWeight = 1e-3
			useCases.g2592_gait130.cmaLRKneeMaxExtAng = -20
		elseif g_mode=='a.1_knee20_hip0_m3' then
			useCases.g2592_gait130.cmaLRKneeLimitWeight = 1e-3
			useCases.g2592_gait130.cmaLRKneeMaxExtAng = -20
			useCases.g2592_gait130.cmaLRHipLimitWeight = useCases.g2592_gait130.cmaLRKneeLimitWeight
			useCases.g2592_gait130.cmaLRHipMaxExtAng = -0
		elseif g_mode=='a.1_knee20_hip0_m3_efrtm5' then
			useCases.g2592_gait130.cmaLRKneeLimitWeight = 1e-3
			useCases.g2592_gait130.cmaLRKneeMaxExtAng = -20
			useCases.g2592_gait130.cmaLRHipLimitWeight = useCases.g2592_gait130.cmaLRKneeLimitWeight
			useCases.g2592_gait130.cmaLRHipMaxExtAng = -0
		elseif g_mode=='a.1_back2x_knee20_hip0_m3_efrtm5' then
			useCases.g2592_gait130.cmaLRKneeLimitWeight = 1e-3
			useCases.g2592_gait130.cmaLRKneeMaxExtAng = -20
			useCases.g2592_gait130.cmaLRHipLimitWeight = useCases.g2592_gait130.cmaLRKneeLimitWeight
			useCases.g2592_gait130.cmaLRHipMaxExtAng = -0
			useCases.g2592_gait130.weakRatio = 2
			useCases.g2592_gait130.weakenMuscles={'ercspn_l','ercspn_r'}
			--iter:124 bestfvever:0.0081404223255896
			accumulate({['map,0,swingFootMod,g2592_gait130,x']= -0.28378925473852, ['map,0,swingFootMod,g2592_gait130,y']= -0.36810364453412, ['map,0,swingFootMod,g2592_gait130,z']= 0.14025013033968, ['map,1,swingFootMod,g2592_gait130,x']= 0.15884293972506, ['map,1,swingFootMod,g2592_gait130,y']= 0.50139238568134, ['map,1,swingFootMod,g2592_gait130,z']= 0.2604248553817, ['map,2,swingFootMod,g2592_gait130,x']= -0.32326650465749, ['map,2,swingFootMod,g2592_gait130,y']= 0.012569136398224, ['map,2,swingFootMod,g2592_gait130,z']= -0.066782405576107, ['map,0,spprtFootMod,g2592_gait130,x']= -0.039200241899173, ['map,0,spprtFootMod,g2592_gait130,y']= -0.31319598293774, ['map,0,spprtFootMod,g2592_gait130,z']= 0.39001288835427, ['map,1,spprtFootMod,g2592_gait130,x']= 0.0063545722652412, ['map,1,spprtFootMod,g2592_gait130,y']= 0.87484502459105, ['map,1,spprtFootMod,g2592_gait130,z']= 0.1350771588726, ['map,2,spprtFootMod,g2592_gait130,x']= -0.14975050828532, ['map,2,spprtFootMod,g2592_gait130,y']= -1.0772615953394, ['map,2,spprtFootMod,g2592_gait130,z']= 0.4383641511824, })


		end

	end
	-------------------------------------------------

	useCases.g2592_gait130.controlParam['useCases,g2592_gait130,aObjWeight'] = useCases.g2592_gait130.aObjWeight

	useCases.unmapControlParam(useCases.g2592_gait130)
end


