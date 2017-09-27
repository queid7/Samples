require("config")
require("module")
require("common")

function ctor()
	this:create("Button", "button1", "start");
	this:create("Button", "button2", "pause");
	this:create("Button", "button3", "end");
--	this:create("Value_Slider", "slidery", "slidery");
--	this:widget(0):sliderRange(0,0.5);
--	this:widget(0):sliderValue(0.1);
	this:updateLayout();

	mLoader=MainLib.VRMLloader ("../Resource/motion/gymnist/gymnist.wrl")

	mMotionDOFcontainer=MotionDOFcontainer(mLoader.dofInfo, '../Resource/motion/gymnist/gymnist.dof')
	mMotionDOF=mMotionDOFcontainer.mot

	-- in meter scale
	for i=0, mMotionDOF:rows()-1 do
		mMotionDOF:matView():set(i, 1, mMotionDOF:matView()(i,1)+0.07)
	end

	-- rendering is done in cm scale
	mSkin= RE.createVRMLskin(mLoader, false);
	mSkin:scale(100,100,100); -- motion data is in meter unit while visualization uses cm unit.
	mSkin:setPoseDOF(mMotionDOF:row(0));

	mGraph={}
	mGraph.start={644, 821, next='walk'}
	mGraph.walk={821, 972, next='jump'}
	mGraph.jump={5644,6124, next='bkwalk'}
	mGraph.bkwalk={3525, 3666, next='bkwalk'}
	
	for v, seg in pairs(mGraph) do
		seg.mot=mMotionDOF:range(seg[1], seg[2]):copy()
	end
	
	mCurrSeg=mGraph.start
	mMotion=mCurrSeg.mot
	mDelta=0
	mTime=0
	mRun=0
	mPause=0
	--mNewMotion=MotionDOF(mLoader.dofInfo)
	--mNewMotion:stitch(mGraph.start.mot, mGraph.walk.mot)
	--mNewMotion2=MotionDOF(mLoader.dofInfo)
	--mNewMotion2:stitch(mNewMotion, mGraph.walk.mot)

	--mSkin:applyMotionDOF(mNewMotion2)
	--RE.motionPanel():motionWin():addSkin(mSkin)
	print('ctor')
end
function frameMove(fElapsedTime)
	mTime=mTime+fElapsedTime
	print(mTime)
	print(nextSeg)
	if mRun==1 then
	 	local currFrame=math.round(mTime*120)
	 	local lookAhead=20
	 	local preFrameSet=10
	 	if mMotion:numFrames()+mDelta<currFrame+lookAhead then
	 		local prevMotion=mMotion
	 		local nextSeg=mGraph[mCurrSeg.next].mot
	 			mMotion=MotionDOF(mLoader.dofInfo)
	 		if true then
	 			mMotion:changeLength(prevMotion:length()+nextSeg:length())
	 			mMotion:range(0, prevMotion:numFrames()-lookAhead):assign( prevMotion:range(0, prevMotion:numFrames()-lookAhead))
	 			mMotion:range(prevMotion:numFrames()-lookAhead, mMotion:numFrames()):stitch(
	 			prevMotion:range(prevMotion:numFrames()-lookAhead, prevMotion:numFrames()),
	 			nextSeg) 
	 		else 
	 			local prevLen=prevMotion:length() 
	 			local newLen=prevLen+nextSeg:length()
	 			local newActualLength=lookAhead+preFrameSet+nextSeg:length()
	 			mDelta=mDelta+newLen-newActualLength
				--numFrames = length + 1
				--데이터 크기 변경 : 23프레임 + 다음 모션
				mMotion:changeLength(newActualLength)
				--데이터 복사 b->a a:range(시작,끝-1):assign(b:range(시작,끝-1)
				mMotion:range(0, preFrameSet):assign( prevMotion:range(prevLen+1-lookAhead-preFrameSet, prevLen+1-lookAhead))
				--데이터 합치기 c=a+b  mMotion:range(c):stitch(a,b)  
				mMotion:range(preFrameSet, mMotion:numFrames()):stitch(
				prevMotion:range(prevMotion:length()-lookAhead, prevMotion:numFrames()),
	 			nextSeg)  
			end
		end
		mSkin:setPoseDOF(mMotion:row(currFrame-mDelta))
	end
end

function onCallback(w, userData)
   if w:id()=="button1" then
	mRun=1
	mTime=mPause
   elseif w:id()=="button2" then
	mPause=mTime
	mRun=0
   elseif w:id()=="button3" then
	mRun=0
	end
end

function dtor()
		print('dtor')
end

