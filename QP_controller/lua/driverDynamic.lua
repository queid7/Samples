
require("config")

package.projectPath='../Samples/QP_controller/'
package.path=package.path..";../Samples/QP_controller/lua/?.lua" --;"..package.path
package.path=package.path..";../Samples/classification/lua/?.lua" --;"..package.path

require("IPC_based/common")

require("IPC_based/useCases")

require("subRoutines/motiongraph")

require("IPC_based/LocoGraph")


-- 최적화 루틴 사용법
-- trackingOptimizer.lua최적화 후 cartpoleanalysis2.lua로 타겟포즈 익스포트한다.
-- 그 후 useCase.lua적당히 고친 후 zmpcontrollerfullbody2.lua에서 불러온다.


iterativeIK={ drawDebugPose=false, nIter=5 }
--cleanVisualize=true
cleanVisualize=false

--dbg.startCount(66)
require("IPC_based/LocoSimulation")

-- following values are automatically connected to UI.

function ctor()
	local osm=RE.ogreSceneManager()
	if osm and osm:hasSceneNode("LightNode") then
		local lightnode=osm:getSceneNode("LightNode")
		lightnode:rotate(quater(math.rad(180), vector3(0,1,0)))
	end
  
	boolean_options={}
	boolean_options.attachCamera=useCase.attachCamera or false
	boolean_options.drawDisplacementMap=false
	boolean_options.useOrientationCorrection=false
	boolean_options.solveIK=true
	boolean_options.drawPredictedCOM=false
	boolean_options.drawPredictedZMP =true
	boolean_options.drawPelvis=false
	boolean_options.drawControlForce=true
	boolean_options.drawMatching=false

	float_options={}
	float_options.impulseMagnitude={ val=100, min=10, max=1000}
	float_options.impulseDuration={ val=0.2, min=1, max=100}
	float_options.desiredAngleX={ val=0, min=-1, max=1}
	float_options.desiredAngleZ={ val=0, min=-1, max=1}
	float_options.timescaleL={ val=0, min=-0.5, max=0.5}

	if cleanVisualize==false then
		RE.FltkRenderer():onCallback('OgreTraceManager')
	end
	mSynthesis	= OnlineLocoSynthesis:new()
	--  mSynthesis:__updateConstraints() -- I don't know why, but it needs to called again.
	-- mSynthesis:__updateConstraints() -- I don't know why, but it needs to called again.
	-- mSynthesis:__updateConstraints() -- I don't know why, but it needs to called again.

	this:create("Button", "stop", "stop")

	this:create("Choice", "type")
	this:widget(0):menuSize(4)
	this:widget(0):menuItem(0, "choose desired type")
	this:widget(0):menuItem(1, "walk", "w")
	this:widget(0):menuItem(2, "run", "r")
	this:widget(0):menuItem(3, "stand", "s")
   
	this:create("Value_Slider"	, "set desired speed X", "set desired speed X",1);
	this:widget(0):sliderRange(-5, 5);
	this:widget(0):sliderValue(0);

	this:create("Value_Slider"	, "set time scale", "set time scale",1);
	this:widget(0):sliderRange(0.5, 2);
	this:widget(0):sliderValue(1);
   
	this:create("Value_Slider"	, "set desired speed Z", "set desired speed Z",1);
	this:widget(0):sliderRange(-3, 4);
	this:widget(0):sliderValue(0);

	this:create("Value_Slider"	, "set desired turning speed", "set desired turning speed",1);
	this:widget(0):sliderRange(-0.9, 0.9);
	this:widget(0):sliderValue(0);

	this:create("Value_Slider"	, "set externalForce X", "set externalForce X",1);
	this:widget(0):sliderRange(0, 100);
	this:widget(0):sliderValue(0);

	this:create("Value_Slider"	, "set externalForce Z", "set externalForce Z",1);
	this:widget(0):sliderRange(0, 100);
	this:widget(0):sliderValue(0);

	this:create("Value_Slider"	, "debug", "debug",1);
	this:widget(0):sliderRange(0, 1);
	this:widget(0):sliderValue(1);


	for k, v in pairs(float_options) do
		this:create("Value_Slider"	, k, k,1)
		this:widget(0):sliderRange(v.min, v.max)
		this:widget(0):sliderValue(v.val)
	end

	this:create("Button"	, "impulse", "impulse",0)
	this:create("Button"	, "rotate view", "rotate view",0)
   
	this:create("Button", "center viewpoint", "center viewpoint",0)


	this:create("Check_Button"	, "show character", "show character",0)
	this:widget(0):checkButtonValue(true)


	for k, v in pairs(boolean_options) do

		this:create("Check_Button"	, k, k,0)
		this:widget(0):checkButtonValue(v)
	end


	--this:create("Check_Button"	, "orientation correction", "orientation correction",0);
	--this:widget(0):checkButtonValue(true)
	this:updateLayout()

	RE.viewpoint().vpos:set(0, 70, 300)
	RE.viewpoint().vat:set(0,60,0)
	RE.viewpoint():update()

	mCameraInfo={}
	local curPos=vector3(0,0,0)
	mCameraInfo.vpos=RE.viewpoint().vpos-curPos
	mCameraInfo.vat=RE.viewpoint().vat-curPos


	--util.FractionTimer.init()

end

function dtor()
   dbg.finalize()
   if mSynthesis then
	   mSynthesis:__finalize()
	   mSynthesis=nil
   end
end

function onCallback(w, userData)
   if w:id()=="stop" then
	   if mSynthesis then
		   mSynthesis:__finalize()
		   mSynthesis=nil
		   collectgarbage()
	   end

   elseif w:id()=="type" then
      print(w:menuValue())

      mSynthesis.desiredType=w:menuValue()
  elseif w:id()=='debug' then
	  mSynthesis.debugSlider=w:sliderValue()
   elseif w:id()=="set desired speed X" then
      mSynthesis.desiredSpeedX=w:sliderValue()
   elseif w:id()=="set desired speed Z" then
      mSynthesis.desiredSpeedZ=w:sliderValue()
   elseif w:id()=="set desired turning speed" then
      mSynthesis.desiredTurningSpeed=w:sliderValue()
   elseif w:id()=="set externalForce X" then
      mSynthesis.externalForce.x=w:sliderValue()
   elseif w:id()=="set externalForce Z" then
      mSynthesis.externalForce.z=w:sliderValue()
   elseif w:id()=="impulse" then
      mSynthesis.impulse=float_options.impulseDuration.val*model.simulationFrameRate
      mSynthesis.impulseDir=vector3(1,0,0)*float_options.impulseMagnitude.val
      mSynthesis.impulseGizmo=mSynthesis.objectList:registerEntity("arrow2", "arrow2.mesh")
      mSynthesis.impulseGizmo:setScale(2,2,2)
      RE.output("impulse", tostring(mSynthesis.impulse))
   elseif w:id()=="center viewpoint"then
      local center=mSynthesis.pendulum:calcCOMpos()
      RE.viewpoint().vat:assign(center*100)
      RE.viewpoint():update()
      
  elseif w:id()=="rotate view" then

	    if boolean_options.attachCamera then
			
			local vpos=mCameraInfo.vpos-mCameraInfo.vat
			vpos=rotate(vpos,quater(math.rad(10), vector3(0,1,0)))
			mCameraInfo.vpos:assign(mCameraInfo.vat+vpos)
		end
   elseif w:id()=="show character" then
      if mSynthesis and mSynthesis.skin2 then
	 mSynthesis.skin2:setVisible(w:checkButtonValue())
      end
   else
      for k, v in pairs(float_options) do
	 if w:id()==k then
	    float_options[k].val=w:sliderValue()
	    break
	 end
      end
      for k, v in pairs(boolean_options) do
	 if w:id()==k then
	    boolean_options[k]=w:checkButtonValue()
	    break
	 end
      end
   end
end

function renderOneFrame()
   noFrameMove=true
   RE.renderOneFrame(false)	
   noFrameMove=nil
end

--testExit=1
function frameMove(fElapsedTime)
   --	debug.debug()


   if mSynthesis~=nil and noFrameMove==nil then

      for rdd=1,renderDownSample do
	 for i=1,outputSuperSample do

		 --if mSynthesis.numFrames==400 then
			 --print("test profiler finished: C++ ", util.FractionTimer.stopInside(), "lua ", util.FractionTimer.stopOutside(),"ms\n")
		 --end
		if dbg_testmw then -- locoSimulation.lua
			local g_nf=mSynthesis.numFrames
			if preciseComparison and g_nf>=preciseComparison[1] and g_nf<preciseComparison[2] then
				g_debugOneStep=array:new()
				g_debugOneStepSaveFlag=true
			else
				g_debugOneStep=nil
			end
			mSynthesis:oneStep()
			if g_debugOneStep then
				util.saveTable(g_debugOneStep, "debugstates_comp/debugStates_oneStep1-"..g_nf..".tbl" )
				g_debugOneStepSaveFlag=false
			end
			mSynthesis:saveStates("debugstates_comp/debugStates1__"..g_nf..".tbl")
			if preciseComparisonSimul and g_nf>=preciseComparisonSimul[1] and g_nf<preciseComparisonSimul[2] then
				g_debugOneStep=array:new()
				g_debugOneStepFlag =true -- so that only the result of the first simulation frame is stored.
			else
				g_debugOneStep=nil
			end
			if g_nf==0 then
				saveDebugInfo(mSynthesis.simulator, "debugstates_comp/debugInfo_oneStep1.tbl")
			end
			mSynthesis:oneStepSimul()
			if g_nf==0 then
				saveDebugInfo(mSynthesis.simulator, "debugstates_comp/debugInfo_oneStepSimul1.tbl")
			end
			if g_debugOneStep then
				util.saveTable(g_debugOneStep, "debugstates_comp/debugStates_oneStepSimul1_"..g_nf..".tbl" )
			end
			mSynthesis:saveStates("debugstates_comp/debugStates1_"..g_nf..".tbl")
		else
			mSynthesis:oneStep()
			mSynthesis:oneStepSimul()
		end
	    if boolean_options.attachCamera then
	       local curPos= mSynthesis.pendulum:calcCOMpos()*100
	       curPos.y=0
	       RE.viewpoint().vpos:assign(mCameraInfo.vpos+curPos)
	       RE.viewpoint().vat:assign(mCameraInfo.vat+curPos)
	       RE.viewpoint():update()     
	    end

	    if error_feedback_method~=EFM.NONE then	 
	       mSynthesis:prepareNextStep()      
	   else
		   RE.output2("falldown?", "skipping prepareNextStep")
	    end
		mSynthesis:debugOut()

	    
	    if renderSuperSample and i~=outputSuperSample then
	       renderOneFrame()
	    end
	 end
      end

      print("frameMove", mSynthesis.numFrames)

   end
end

