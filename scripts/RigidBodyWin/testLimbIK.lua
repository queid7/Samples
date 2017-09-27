require("config")
require("module")
require("common")

function ctor()
	this:create("Button", "button1");
	this:create("Button", "button2");
--	this:create("Value_Slider", "sliderx", "sliderx");
--	this:widget(0):sliderRange(0,0.5);
--	this:widget(0):sliderValue(0.1);
--	this:create("Value_Slider", "slidery", "slidery");
--	this:widget(0):sliderRange(0,0.5);
--	this:widget(0):sliderValue(0.1);
--	this:create("Value_Slider", "sliderz", "sliderz");
--	this:widget(0):sliderRange(-0.5,0.5);
--	this:widget(0):sliderValue(0.1);
	this:create("Value_Slider", "sliderx", "sliderx");
	this:widget(0):sliderRange(-100,100);
	this:widget(0):sliderValue(1);
	this:create("Value_Slider", "slidery", "slidery");
	this:widget(0):sliderRange(-100,100);
	this:widget(0):sliderValue(1);
	this:create("Value_Slider", "sliderz", "sliderz");
	this:widget(0):sliderRange(-100,100);
	this:widget(0):sliderValue(1);
	this:updateLayout();
	this:updateLayout();

	config_manip={
		"../Resource/mocap_manipulation/bvh_files/gf_0515/2/gf_skel2-1.wrl",
		'../Resource/mocap_manipulation/bvh_files/gf_0515/2/gf_skel2-1.dof',
		"LeftElbow", "LeftWrist", vector3(0,0.0,0),
		"RightElbow", "RightWrist", vector3(0,0.0,0),
		reversed=true,
		skinScale=1,
	}

	config_gymnist={
		"../Resource/motion/gymnist/gymnist.wrl",
		'../Resource/motion/gymnist/gymnist.dof',
		'lradius', 'lhand', vector3(0,0,0),
		'rradius', 'rhand', vector3(0,0,0),
		reversed=false,
		skinScale=100,
	}
	config_run={
		"../Resource/motion/justin_straight_run/justin_straight_run.wrl",
		"../Resource/motion/justin_straight_run/justin_straight_run.dof", 
		'ltibia', 'lfoot', vector3(0.000000,-0.053740,0.111624),
		'rtibia', 'rfoot', vector3(0.000000,-0.054795,0.112272),
		reversed=false,
		skinScale=100,
	}

	--config=config_manip
	config=config_run
    mLoader=MainLib.VRMLloader (config[1])
	mMotionDOFcontainer=MotionDOFcontainer(mLoader.dofInfo, config[2])
	mMotionDOF=mMotionDOFcontainer.mot

	-- in meter scale
	for i=0, mMotionDOF:rows()-1 do
		mMotionDOF:matView():set(i, 1, mMotionDOF:matView()(i,1)+0.07)
	end

	-- rendering is done in cm scale
	mSkin= RE.createVRMLskin(mLoader, false);
	local s=config.skinScale
	mSkin:scale(s,s,s); -- motion data often is in meter unit while visualization uses cm unit.
	mPose=vectorn()
	mPose:assign(mMotionDOF:row(0));
	mSkin:setPoseDOF(mPose);

	mEffectors=MotionUtil.Effectors()
	mEffectors:resize(2);
	lknee=mLoader:getBoneByName(config[3])
	mEffectors(0):init(mLoader:getBoneByName(config[4]), config[5])
	rknee=mLoader:getBoneByName(config[6]);
	mEffectors(1):init(mLoader:getBoneByName(config[7]), config[8])

	--mIK= MotionUtil.createFullbodyIkDOF_limbIK(mLoader.dofInfo, mEffectors, lknee, rknee, config.reversed);
	--mIK= MotionUtil.createFullbodyIk_MotionDOF_MultiTarget(mLoader.dofInfo, mEffectors);
	--mIK=MotionUtil.createFullbodyIkDOF_limbIK_straight(mLoader.dofInfo,mEffectors,lknee,rknee);
	
	mIK=LimbIKsolver(mLoader.dofInfo,mEffectors, CT.ivec(lknee:treeIndex(), rknee:treeIndex()), CT.vec(1,1))
	footPos=vector3N (2);

	dbg.namedDraw("Sphere", vector3(0,0,0), "origin", "red")
end

function onCallback(w, userData)
   if w:id()=="button1" then
	   print("button1\n");
   elseif w:id()=="button2" then
	   print("button2\n");
   elseif w:id()=="sliderx"or w:id()=="slidery"or w:id()=="sliderz" then
		local xx= this:findWidget('sliderx'):sliderValue()
		local yy= this:findWidget('slidery'):sliderValue()
		local zz= this:findWidget('sliderz'):sliderValue()
	   print(w:sliderValue())
	   mPose:assign(mMotionDOF:row(0));
	   mLoader:setPoseDOF(mPose);
	   -- local pos to global pos
	   local footOri=quaterN(2)
	   for i=0,1 do
		   local originalPos=mEffectors(i).bone:getFrame():toGlobalPos(mEffectors(i).localpos)
		   footPos(i):assign(originalPos);
		   footOri(i):assign(mEffectors(i).bone:getFrame().rotation)
		   dbg.namedDraw("Sphere", originalPos*config.skinScale, "x"..i)
	   end
	   footPos(0):radd(vector3(xx, yy,zz)/config.skinScale)
	   dbg.namedDraw("Sphere", footPos(0)*config.skinScale, "x0")
	   --mIK:IKsolve(mPose, footPos);
	   local importance=vectorn(2)
	   importance:setAllValue(1)
	   mIK:IKsolve3(mPose, MotionDOF.rootTransformation(mPose), footPos, footOri, importance)
	   mSkin:setPoseDOF(mPose);
   end
end

function dtor()
end

function frameMove(fElapsedTime)
end
