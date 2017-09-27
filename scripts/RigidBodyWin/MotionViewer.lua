
require("config")
require("module")

motion_path="../Resource/motion/"


function scr(p) -- for commandline use, e.g. lua short.lua wrlv "ctor();scr()"
	p =p or 1
	if p==1 then
		onCallback(this:findWidget("load v"),0)
	end
end

function ctor()

   mEventReceiver=EVR()
   
--   this:create("Button", "load a predefined model", "load a predefined model", 0,3,0)


   this:create("Button", "load .V file", "load .V file")
   this:create("Button", "load .bvh file", "load .bvh/.mot file")
   this:create("Button", "load .amc file", "load .amc file")
   this:create("Button", "export .bvh file", "export .bvh file")
   this:create("Button", "scale 0.1 bvh file", "scale 0.1 bvh file")
   this:create("Button", "test", "test")
   this:create("Button", "changeMotionPath", "changeMotionPath")
   this:widget(0):buttonShortcut("FL_ALT+t")
   this:create("Button", "scale 0.1", "scale 0.1")
   this:create("Button", "scale 10", "scale 10")
   this:create("Button", "scale 100", "scale 100")
   this:create("Button", "change to Y up", "change to Y up")
   this:updateLayout()
   this:redraw()

   mObjectList=Ogre.ObjectList()

   RE.viewpoint():setFOVy(44.999999)
   RE.viewpoint().vpos:set(94.777964, 126.724047, 352.393547)
   RE.viewpoint().vat:set(-34.317428, 67.508947, -4.622992)
   RE.viewpoint():update()

   RE.renderer():fixedTimeStep(false)   
end


function onCallback(w, userData)
	if w:id()=="load .bvh file" then
	   local chosenFile=Fltk.chooseFile("Choose a BVH/MOT file to view", motion_path, "*.{bvh,mot}", false)
	   if chosenFile~="" then
		   mLoader=RE.motionLoader(chosenFile)
		   mSkin=RE.createSkin(mLoader)
		   mSkin:applyAnim(mLoader.mMotion)
		   RE.motionPanel():motionWin():detachSkin(mSkin)
		   RE.motionPanel():motionWin():addSkin(mSkin)
	   end
   elseif w:id()=="scale 0.1 bvh file" then
	   local chosenFile=Fltk.chooseFile("Choose a BVH/MOT file to view", motion_path, "*.{bvh,mot}", false)
	   if chosenFile~="" then
		   mLoader=RE.motionLoader(chosenFile)
		   mLoader.mMotion:scale(0.1)
		   local mot=mLoader.mMotion
		   MotionUtil.exportBVH(mot, string.sub(chosenFile, 1,-5).."_0.1.bvh", 0, mot:numFrames())
	   end
   elseif w:id()=="changeMotionPath" then
	   motion_path="../Resource/scripts/ui/RigidBodyWin"
   elseif w:id()=="load .amc file" then
	   local chosenFile=Fltk.chooseFile("Choose a ASF file to view", motion_path, "*.asf", false)
	   if chosenFile~="" then
		   mLoader=RE.motionLoader(chosenFile)
		   if mLoader.mMotion:numFrames()==0 then
			   local chosenFile=Fltk.chooseFile("Choose a AMC file to view", motion_path, "*.amc", false)
			   if chosenFile~="" then mLoader:loadAnimation(mLoader.mMotion, chosenFile) end
		   end
		   mSkin=RE.createSkin(mLoader)
		   mSkin:applyAnim(mLoader.mMotion)
		   RE.motionPanel():motionWin():detachSkin(mSkin)
		   RE.motionPanel():motionWin():addSkin(mSkin)
	   end
   elseif w:id()=="export .bvh file" then
	   local chosenFile=Fltk.chooseFile("Choose a BVH file to create", model_path, "*.bvh", true)
	   	local mot=mLoader.mMotion
		for i=1, mot:skeleton():numBone()-1 do
			local bone=mot:skeleton():bone(i)
			local trans=bone:getTranslationalChannels()
			local rot=bone:getRotationalChannels()
			if string.len(rot)~=0 then
				rot="ZXY"
			end

			if string.len(trans)~=0 then
				trans="XYZ"
			end

			bone:setChannels(trans, rot)
		end
	   MotionUtil.exportBVH(mLoader.mMotion, chosenFile, 0, mLoader.mMotion:numFrames())
   elseif w:id()=="load .V file" then
	   local chosenFile=Fltk.chooseFile("Choose a vsk file to view", motion_path, "*.vsk", false)
	   if chosenFile~="" then
		   mLoader=RE.motionLoader(chosenFile)
		   --if mLoader.mMotion:numFrames()==0 then
		   if true then
			   local fn, path=os.processFileName(chosenFile)
			   motion_path=path
			   local chosenFile=Fltk.chooseFile("Choose v file to view", path, "*.v", false)
			   if chosenFile~="" then mLoader:loadAnimation(mLoader.mMotion, chosenFile) end
		   end
		   mSkin=RE.createSkin(mLoader)
		   mSkin:applyAnim(mLoader.mMotion)
		   RE.motionPanel():motionWin():detachSkin(mSkin)
		   RE.motionPanel():motionWin():addSkin(mSkin)
	   end
   elseif w:id()=="test" then
	   mLoader=RE.motionLoader("../Resource/motion/Gymnist/Project 2/Capture day 1/Session 1/gymnist1.vsk")
	   --mLoader:loadAnimation(mLoader.mMotion, "../Resource/motion/Gymnist/Project 2/Capture day 1/Session 1/matless_ROM.V")
	   --mLoader:loadAnimation(mLoader.mMotion, "../Resource/motion/Gymnist/Project 2/Capture day 1/Session 1/matless_walk.V")
	   mLoader:loadAnimation(mLoader.mMotion, "../Resource/motion/Gymnist/Project 2/Capture day 1/Session 1/matless_roundoff 1.V")
	   --mLoader:loadAnimation(mLoader.mMotion, "../Resource/motion/test.v")
	   mLoader:printHierarchy()
	   mEventReceiver:loadMarkers(mLoader.markers,mLoader.frames, mLoader)
	   mSkin=RE.createSkin(mLoader)
	   mSkin:applyAnim(mLoader.mMotion)
	   RE.motionPanel():motionWin():detachSkin(mSkin)
	   RE.motionPanel():motionWin():addSkin(mSkin)
   elseif w:id()=="scale 100" then
	   mSkin:scale(100,100,100)
	   mSkin:setThickness(3)
   elseif w:id()=="scale 10" then
	   mSkin:scale(10,10,10)
	   mSkin:setThickness(3)
   elseif w:id()=="scale 0.1" then
	   mSkin:scale(0.1,0.1,0.1)
	   mSkin:setThickness(3)
   elseif w:id()=="change to Y up" then
	   local dofRot=quater(math.rad(-90), vector3(0,1,0))
	   *quater(math.rad(-90), vector3(1,0,0)) -- change to Y_up
	   mSkin:setRotation(dofRot)
   end

end

function dtorSub()
	-- if RE.motionPanelValid() then
	--    RE.motionPanel():motionWin():detachAllSkin()
	-- end

	if RE.motionPanelValid() then
		if mSkin then
			RE.motionPanel():motionWin():detachSkin(mSkin)
			mSkin=nil
		end
	end

	-- remove objects that are owned by LUA
	collectgarbage()
end

function dtor()
	dbg.finalize()
	dtorSub()
end
if EventReceiver then
	EVR=LUAclass(EventReceiver)
	function EVR:__init(graph)
		--EventReceiver.__init(self)
		self.currFrame=0
	end
else
	class 'EVR'
	function EVR:__init(graph)
	end
end
function EVR:loadMarkers(markers, bones, skel)
	self.traceManager=array:new()
	for i=1, markers:rows() do
		local markerdraw=TStrings()
		markerdraw:resize((markers:cols()/3)*2+ (bones:cols()/6)*2)

		local c=0
		local f=i-1
		for j=0, markers:cols()-1,3 do
			local nameid='m'..tostring(j/3)
			markerdraw:set(c, 'namedDraw_'..nameid)
			markerdraw:set(c+1, table.tostring2({'Sphere', vector3(markers(f,j), markers(f,j+1), markers(f,j+2)), nameid, 'red', 5}) )
			c=c+2
		end
		for j=0, bones:cols()-1,6 do
			local nameid='b'..skel:bone(j/6+1):name()
			markerdraw:set(c, 'namedDraw_'..nameid)
			local tf=transf()
			tf.translation:assign(vector3(bones(f,j+3), bones(f,j+4), bones(f,j+5)))
--			tf.rotation:setRotation("XYZ", vector3(bones(f,j), bones(f,j+1), bones(f,j+2))) -- EulerXYZ
			tf.rotation:setRotation( vector3(bones(f,j), bones(f,j+1), bones(f,j+2))) -- Rotation vector
			markerdraw:set(c+1, table.tostring2({'Axes', tf, nameid}))
			c=c+2
		end
		self.traceManager:pushBack(markerdraw)
	end
end
function EVR:loadTraceManager()
	self.traceManager=array:new()

	local binaryFile=util.BinaryFile()
	binaryFile:openRead("debug_plot.cf")
	local contactForce=matrixn()
	binaryFile:unpack(contactForce)
	local n=binaryFile:unpackInt()
	for i=1, n do
		local message=TStrings()
		binaryFile:unpack(message)
		self.traceManager:pushBack(message)
	end
	binaryFile:close()
end
function EVR:onFrameChanged(win, iframe)
	self.currFrame=iframe
	if self.traceManager then
		local message=self.traceManager[iframe+1]
		if message then
			dbg.eraseAllDrawn()
			RE.outputEraseAll(2)
			for i=0, message:size()-1,2 do
				RE.output2(message(i), message(i+1))
				if string.sub(message(i),1,10)== 'namedDraw_' then
					local tbl=table.fromstring2(message(i+1))
					dbg.namedDraw(unpack(tbl))
				end
			end
		end
	end
	if self.trajectory then
		if self.currFrame<self.trajectory:rows() then
			local curPos=self.trajectory:row(self.currFrame):toVector3(0)*100
			RE.viewpoint().vpos:assign(self.cameraInfo.vpos+curPos)
			RE.viewpoint().vat:assign(self.cameraInfo.vat+curPos)
			RE.viewpoint():update()     
		end
	end
end

function EVR:attachCamera()

	if mLoader~=nill then

		local discont=mMotionDOFcontainer.discontinuity
		local mMotionDOF=mMotionDOFcontainer.mot

		self.trajectory=matrixn(mMotionDOFcontainer:numFrames(),3)

		local segFinder=SegmentFinder(discont)

		for i=0, segFinder:numSegment()-1 do
			local s=segFinder:startFrame(i)
			local e=segFinder:endFrame(i)

			for f=s,e-1 do
				self.trajectory:row(f):setVec3(0, MotionDOF.rootTransformation(mMotionDOF:row(f)).translation)
				self.trajectory:row(f):set(1,0)
			end
			print("filtering",s,e)
			math.filter(self.trajectory:range(s,e,0, 3), 63)
		end

		self.cameraInfo={}
		local curPos=self.trajectory:row(self.currFrame):toVector3(0)*100
		self.cameraInfo.vpos=RE.viewpoint().vpos-curPos
		self.cameraInfo.vat=RE.viewpoint().vat-curPos
	end
end

function frameMove(fElapsedTime)
end

function _applyMotion(chosenFile, nodetach)

	if mLoader~=nill then

		mMotionDOFcontainer=MotionDOFcontainer(mLoader.dofInfo,chosenFile)
		mSkin:applyMotionDOF(mMotionDOFcontainer.mot)
		-- if not nodetach then
		-- 	 RE.motionPanel():motionWin():detachAllSkin()
		-- end
		RE.motionPanel():motionWin():detachSkin(mSkin)

		RE.motionPanel():motionWin():addSkin(mSkin)
	end
end
