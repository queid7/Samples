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
	this:widget(0):sliderRange(0,50);
	this:widget(0):sliderValue(1);
	this:create("Value_Slider", "slidery", "slidery");
	this:widget(0):sliderRange(0,50);
	this:widget(0):sliderValue(1);
	this:create("Value_Slider", "sliderz", "sliderz");
	this:widget(0):sliderRange(-50,50);
	this:widget(0):sliderValue(1);
	this:updateLayout();
	this:updateLayout();

--	mLoader=MainLib.VRMLloader ("../Resource/motion/gymnist/gymnist.wrl")

    mLoader=MainLib.VRMLloader ("../Resource/mocap_manipulation/bvh_files/2/2_skel.wrl")
--	mMotionDOFcontainer=MotionDOFcontainer(mLoader.dofInfo, '../Resource/motion/gymnist/gymnist.dof')
	mMotionDOFcontainer=MotionDOFcontainer(mLoader.dofInfo, '../Resource/mocap_manipulation/bvh_files/2/2_skel.dof')
	mMotionDOF=mMotionDOFcontainer.mot

	-- in meter scale
	for i=0, mMotionDOF:rows()-1 do
		mMotionDOF:matView():set(i, 1, mMotionDOF:matView()(i,1)+0.07)
	end

	-- rendering is done in cm scale
	mSkin= RE.createVRMLskin(mLoader, false);
	mSkin:scale(1,1,1); -- motion data is in meter unit while visualization uses cm unit.
--	mSkin:scale(100,100,100); -- motion data is in meter unit while visualization uses cm unit.
	mPose=vectorn()
	mPose:assign(mMotionDOF:row(0));
	mSkin:setPoseDOF(mPose);

	mEffectors=MotionUtil.Effectors()
	mEffectors:resize(2);
	--mEffectors(0):init(mLoader:getBoneByName("lfoot"), vector3(0,-0.02,0.16))
	
--	mEffectors(0):init(mLoader:getBoneByName("lhand"), vector3(0,0.0,0))
--	mEffectors(1):init(mLoader:getBoneByName("rhand"), vector3(0,0.0,0))
	mEffectors(0):init(mLoader:getBoneByName("LeftWrist"), vector3(0,0.0,0))
	mEffectors(1):init(mLoader:getBoneByName("RightWrist"), vector3(0,0.0,0))


--	lknee=mLoader:getBoneByName("lradius");
--	rknee=mLoader:getBoneByName("rradius");
	lknee=mLoader:getBoneByName("LeftElbow");
	rknee=mLoader:getBoneByName("RightElbow");
	--mIK= MotionUtil.createFullbodyIkDOF_limbIK(mLoader.dofInfo, mEffectors, lknee, rknee, true);
	--mIK= MotionUtil.createFullbodyIk_MotionDOF_MultiTarget(mLoader.dofInfo, mEffectors);
	--mIK=MotionUtil.createFullbodyIkDOF_limbIK_straight(mLoader.dofInfo,mEffectors,lknee,rknee);
	local axisSign=vectorn(2)
	axisSign:setAllValue(2);
	mIK=COM_IKsolver(mLoader, mEffectors, CT.ivec(lknee:treeIndex(), rknee:treeIndex()), axisSign)
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
	   for i=0,1 do
		   local originalPos=mEffectors(i).bone:getFrame():toGlobalPos(mEffectors(i).localpos)
		   footPos(i):assign(originalPos);
		   dbg.namedDraw("Sphere", originalPos*100, "x"..i)
	   end
	   footPos(0):radd(vector3(xx, yy,zz))
	   dbg.namedDraw("Sphere", footPos(0)*100, "x0")
	   --mIK:IKsolve(mPose, footPos);
	   local conDelta=quaterN(2)
	   local importance=vectorn(2)
	   importance:setAllValue(1)
	   conDelta(0):assign(quater(1,0,0,0))
	   conDelta(1):assign(quater(1,0,0,0))
	   local desiredCOM=vector3(0,0,0)
	   local roottf=MotionDOF.rootTransformation(mPose)
	   local rotY=roottf.rotation:rotationY()
	   mIK:IKsolve(mPose, rotY, roottf, conDelta, footPos, importance, desiredCOM);
	   mSkin:setPoseDOF(mPose);
   end
end

function dtor()
end

function frameMove(fElapsedTime)
end
