
require("config")
require("module")
require("common")

package.projectPath='../Samples/classification/'
package.path=package.path..";../Samples/classification/lua/?.lua" --;"..package.path

package.path=package.path..";../Samples/ysscripts/samples/?.lua" --;"..package.path
--require("IPC_based/useCases")
require("useMuscles")

package.path=package.path..";../Samples/ysscripts/lib/?.lua" --;"..package.path
require("utilfunc")
require("OsModel")

--g_plotdata = true
g_plotdata = false

debugVis=true 
--function loadBVH(chosenFile)
	--local initialHeight=0
	--mMotionDOFcontainer=MotionDOFcontainer(mLoader.dofInfo)
	--local dof=_importBVH(chosenFile, initialHeight, 1.0)
	--mMotionDOFcontainer:resize(dof:rows())
	--mMotionDOFcontainer.mot:assign(dof)

	---- fix except root joint
	--mMotionDOFcontainer.mot:matView():sub(0,0,6,0):setAllValue(0)

	--_G.chosenMotFile=chosenFile
	--mSkin:applyMotionDOF(mMotionDOFcontainer.mot)
	--RE.motionPanel():motionWin():detachSkin(mSkin)
	--RE.motionPanel():motionWin():addSkin(mSkin)
	--useNearestSamplingWhenExportingBVH=true
--end

function ctor()
	RE.renderer():fixedTimeStep(true)   
	mEventReceiver=EVR()

	this:create("Button", "draw COM traj", "draw COM traj")
	this:create("Button", "attach camera", "attach camera")
	this:widget(0):buttonShortcut("FL_ALT+c")

	this:create("Check_Button", "draw skeleton", "draw skeleton")
	this:widget(0):checkButtonValue(0)

	this:updateLayout()
	this:redraw()

	mObjectList=Ogre.ObjectList()

	--do -- set viewpoint
		----RE.viewpoint():setFOVy(44.999999)
		----local vpos=vector3(94.777964, 126.724047, 352.393547)
		----local vat=vector3(-34.317428, 67.508947, -4.622992)
		--RE.viewpoint():setFOVy(40)
		--local vpos=vector3(294.777964, 126.724047, -352.393547)
		--local vat=vector3(234.317428, 67.508947, -4.622992)
		----RE.viewpoint().vpos:assign(vat+(vpos-vat)*1.5)
		--RE.viewpoint().vpos:assign(vpos)
		--RE.viewpoint().vat:assign(vat)
		--RE.viewpoint():update()
	--end

	do
		local wrlpath = ''
		local luamsclpath = ''
		local motpath = ''

		----wrlpath = '../Resource/motion/opensim/gait23_2geometry.wrl'
		----luamsclpath = '../Resource/motion/opensim/gait2354_render.luamscl'

		------------------------------------------------------------------
		---- chain1
		----wrlpath = '../Resource/motion/opensim/chain1.wrl'
		
		------------------------------------------------------------------
		---- chain2
		----wrlpath = '../Resource/motion/opensim/chain2_ball.wrl'
		----wrlpath = '../Resource/motion/opensim/chain2_hinge.wrl'
		
		----wrlpath = '../Resource/motion/opensim/chain2_ball_mscl4.wrl'
		----luamsclpath = '../Resource/motion/opensim/chain2_ball_mscl4.luamscl'
		
		--wrlpath = '../Resource/motion/opensim/chain2_ball_mscl1.wrl'
		--luamsclpath = '../Resource/motion/opensim/chain2_ball_mscl1.luamscl'
		
		----wrlpath = '../Resource/motion/opensim/chain2_hinge_mscl2_pathpoint.wrl'
		----luamsclpath = '../Resource/motion/opensim/chain2_hinge_mscl2_pathpoint.luamscl'
		
		----wrlpath = '../Resource/motion/opensim/chain3_hinge_mscl2_biarti.wrl'
		----luamsclpath = '../Resource/motion/opensim/chain3_hinge_mscl2_biarti.luamscl'
		
		----motpath = '../Resource/motion/opensim/chain2_rotY.bvh'
		--motpath = '../Resource/motion/opensim/chain2_rotZ.bvh'
		----motpath = '../Resource/motion/opensim/chain2_rotYZ.bvh'
		----motpath = '../Resource/motion/opensim/chain2_stop0.bvh'
		----motpath = '../Resource/motion/opensim/chain2_stop90.bvh'
		----motpath = '../Resource/motion/opensim/chain3_rotZ.bvh'
		
		--heightOffset = .5
		
		------------------------------------------------------------------
		---- gait* models
		
		----wrlpath = '../Resource/motion/opensim/gait1954_render.wrl'
		----luamsclpath = '../Resource/motion/opensim/gait1954_render.luamscl'

		--wrlpath = '../Resource/motion/opensim/gait1956_render.wrl'
		--luamsclpath = '../Resource/motion/opensim/gait1956_render.luamscl'

		----wrlpath = '../Resource/motion/opensim/gait2354_render.wrl'
		----luamsclpath = '../Resource/motion/opensim/gait2354_render.luamscl'
		
		----wrlpath = '../Resource/motion/opensim/gait2154_render.wrl'
		----luamsclpath = '../Resource/motion/opensim/gait2154_render.luamscl'

		----wrlpath = '../Resource/motion/opensim/gait2392_render.wrl'
		----luamsclpath = '../Resource/motion/opensim/gait2392_render.luamscl'

		----wrlpath = '../Resource/motion/opensim/FullBody_lee.wrl'
		----luamsclpath = '../Resource/motion/opensim/FullBody_lee.luamscl'

		----motpath = '../Resource/motion/opensim/gait23_6steps_120.bvh'
		----motpath = '../Resource/motion/opensim/gait23_6steps_120.dof'
		----motpath = '../Resource/motion/opensim/gait21_6steps_120.dof'
		----motpath = '../Resource/motion/opensim/gait19_6steps_120.dof'
		--motpath = '../Resource/motion/opensim/gait19_WalkTongTong00.dof'
		----motpath = '../Resource/motion/opensim/fullbody_WalkTongTong00.dof'
		------------------------------------------------------------------
	end

	model = scenarios.toModel(useCase.scenario)
	mOsim = OsModel(model.wrlpath, model.luamsclpath, model)
	local motpath = model.mot_file
	local initialHeight = model.initialHeight

	--mOsim = OsModel(wrlpath, luamsclpath)
	mOsim:setBoneForwardKinematics(mOsim.mLoader:fkSolver())

	if string.find(motpath, '.dof') then
		--mOsim:loadDOF(motpath)
		mOsim:loadDOF_repeat(motpath)
	elseif string.find(motpath, '.bvh') then
		mOsim:loadBVH(motpath,heightOffset)
	end

	for i=0, mOsim.mMotionDOFcontainer.mot:numFrames()-1 do
		mOsim.mMotionDOFcontainer.mot:row(i):set(1,mOsim.mMotionDOFcontainer.mot:row(i):get(1)+(initialHeight or model.initialHeight) )
	end

	mOsim:setIsometricFiberLengths()
 
	RE.motionPanel():motionWin():addSkin(mOsim.mSkin)

	-- camera
	RE.viewpoint().vpos:set(30, 100, 300)
	--RE.viewpoint().vpos:set(300, 100, 50)
	RE.viewpoint().vat:set(0,80,0)
	RE.viewpoint():update()

	if g_plotdata then
		local dataname = useCase.modelName..'__ref'
		mFile_l_mt = io.open(dataname..'__lmt.txt', 'w')
		mFile_angle = io.open(dataname..'__ang.txt', 'w')

		mFile_l_mt:write('frame\t')
		for m,msclName in ipairs(mOsim:getMuscleNames()) do
			mFile_l_mt:write(msclName..'.lmt'..'\t')
		end
		mFile_l_mt:write('\n')

		local angleNames={'hip_r.ang_flx','knee_r.ang_flx','ankle_r.ang_dor', 'pelvis.ang_roll'}
		mAngleIndices = {mOsim.rhipflexDOF, mOsim.rkneeDOF, mOsim.rankleDOF, -1}
		mFile_angle:write('frame\t')
		for m,angName in ipairs(angleNames) do
			mFile_angle:write(angName..'\t')
		end
		mFile_angle:write('\n')
	end
end


function onCallback(w, userData)
	if w:id()=="draw COM traj" then
		do -- test
			--local comtraj=calcCOMtrajectory(mLoader, mMotionDOFcontainer.mot, mMotionDOFcontainer.discontinuity)
			local comtraj=calcCOMtrajectory(mOsim.mLoader, mMotionDOFcontainer.mot, mMotionDOFcontainer.discontinuity)
			mObjectList:registerObject("comtraj", "LineList", "solidred", comtraj:matView()*100,0)
		end
	elseif w:id()=="attach camera" then
		mEventReceiver:attachCamera()
	end
end


function dtor()
	if g_plotdata then
		io.close(mFile_l_mt)
		io.close(mFile_angle)
	end

	--RE.motionPanel():motionWin():detachSkin(mSkin)
	RE.motionPanel():motionWin():detachSkin(mOsim.mSkin)
	mOsim=nil
	--mLoader=nil
	mObjectList=nil
	--mSkin=nil
	mMotionDOFcontainer=nil
	collectgarbage()
	dbg.finalize()
	--detachSkins()
end
if EventReceiver then
	--class 'EVR'(EventReceiver)
	EVR=LUAclass(EventReceiver)
	function EVR:__init(graph)
		--EventReceiver.__init(self)
		self.currFrame=0
		self.cameraInfo={}
	end
end

function EVR:onFrameChanged(win, iframe)
	self.currFrame=iframe
	if self.trajectory then
		if self.currFrame<self.trajectory:rows() then
			local mMotionDOF=mMotionDOFcontainer.mot
			local curPos=self.trajectory:row(self.currFrame):toVector3(0)*100
			local pPos=
			MotionDOF.rootTransformation(mMotionDOF:row(self.currFrame)).translation
			local currRot=
			self.trajectoryOri:row(self.currFrame):toQuater(0):rotationY()

			--dbg.draw('Line', pPos*100, pPos*100+rotate(vector3(0,0,100), currRot), 'prot')

			do
				RE.viewpoint().vpos:assign(self.cameraInfo.vpos+curPos)
				RE.viewpoint().vat:assign(self.cameraInfo.vat+curPos)
				RE.viewpoint():update()     
			end
		end
	end
	
	if mOsim.records[iframe]==nil then
		print(iframe, 'orig')
		mOsim.mLoader:setPoseDOF(mMotionDOFcontainer.mot:row(self.currFrame))

		mOsim:invalidatePPIndicesBasedInfo()
		mOsim:invalidatePPPoszBasedInfo()
		mOsim:integrateMuscleDynamics(1/30.)

		--print('frame', iframe)
		--local u = create_array(mOsim:getNumMuscles(), 1.)
		--local u = create_array(mOsim:getNumMuscles(), math.sin((iframe/10)%math.pi))
		--mOsim:setExcitations(u)
		
		--mOsim:drawMuscles(iframe, mObjectList)
		--mOsim:drawJoints(iframe, mObjectList)
		--mOsim:drawMuscleWithForces(iframe, mObjectList)
		
		if g_plotdata then
			mFile_l_mt:write(iframe..'\t')
			for m,l_mt in ipairs(mOsim:getTendonMuscleLengths()) do
				mFile_l_mt:write(l_mt..'\t')
			end
			mFile_l_mt:write('\n')

			--mFile_angle:write(iframe..'\t')
			--for j,angIndex in ipairs(mAngleIndices) do
				--local angValue = mMotionDOFcontainer.mot:row(self.currFrame)(angIndex)
				--if angIndex==mOsim.rkneeDOF then
					--angValue = -angValue
				--end
				--mFile_angle:write(math.deg(angValue)..'\t')
			--end
			--mFile_angle:write('\n')
			
			local posedof = vectorn()
			mOsim.bfk:getPoseDOFfromGlobal(posedof)
			mFile_angle:write(iframe..'\t')
			for j,angIndex in ipairs(mAngleIndices) do
				local angValue
				if angIndex==mOsim.rkneeDOF then
					angValue = -posedof(angIndex)
				elseif angIndex==-1 then	--pelvis_roll
					local rootTF=MotionDOF.rootTransformation(posedof)
					local rotY=rootTF.rotation:rotationY()
					local upward=vector3(0,1,0)
					upward:rotate(rootTF.rotation)
					local localRot=rotY:inverse()*rootTF.rotation
					local roll=localRot:rotationAngleAboutAxis(vector3(1,0,0))	    
					angValue = roll
				else
					angValue = posedof(angIndex)
				end
				mFile_angle:write(math.deg(angValue)..'\t')
			end
			mFile_angle:write('\n')

		end

	else
		print(iframe, 'rec')
		mOsim:drawRecordedScene(iframe, mObjectList)
	end
end

function EVR:attachCamera()

	--if mLoader~=nil then
	if mOsim~=nil then

		local discont=mMotionDOFcontainer.discontinuity
		local mMotionDOF=mMotionDOFcontainer.mot

		self.trajectory=matrixn(mMotionDOFcontainer:numFrames(),3)

		self.trajectoryOri=matrixn(mMotionDOFcontainer:numFrames(),4)
		local segFinder=SegmentFinder(discont)

		for i=0, segFinder:numSegment()-1 do
			local s=segFinder:startFrame(i)
			local e=segFinder:endFrame(i)

			for f=s,e-1 do
				self.trajectory:row(f):setVec3(0, MotionDOF.rootTransformation(mMotionDOF:row(f)).translation)
				self.trajectory:row(f):set(1,0)
				self.trajectoryOri:row(f):setQuater(0, MotionDOF.rootTransformation(mMotionDOF:row(f)).rotation:rotationY())
			end
			print("filtering",s,e)
			math.filter(self.trajectory:range(s,e,0, 3), 63)
			math.filter(self.trajectoryOri:range(s,e,0, 4), 63)
		end

		local curPos=self.trajectory:row(self.currFrame):toVector3(0)*100
		self.cameraInfo.vpos=RE.viewpoint().vpos-curPos
		self.cameraInfo.vat=RE.viewpoint().vat-curPos
		self.cameraInfo.dist=RE.viewpoint().vpos:distance(curPos)
		self.cameraInfo.refRot=self.trajectoryOri:row(self.currFrame):toQuater(0):rotationY()
	end
end

function frameMove(fElapsedTime)
end

--require('classifyLib')
