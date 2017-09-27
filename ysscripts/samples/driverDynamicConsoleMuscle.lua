package.path=package.path..";../Samples/ysscripts/samples/?.lua" --;"..package.path
require("driverDynamicMuscle")

g_saveReplay = true
g_recordSimulation = true
g_plotdata = true


function ctor()

	model = scenarios.toModel(useCase.scenario)
	mOsim = OsModel(model.wrlpath, model.luamsclpath, model)
	
	mLoader = mOsim.mLoader

	print('original f_m_o')
	printtblh(mOsim.f_m_o)
	if useCase.f_m_o ~= nil then
		mOsim.f_m_o = useCase.f_m_o
		print('adapted f_m_o')
		printtblh(mOsim.f_m_o)
	end


	mSynthesis	= OnlineLocoSynthesis:new()

	mObjectList = Ogre.ObjectList()

	boolean_options={}
	boolean_options.attachCamera=useCase.attachCamera or false
	boolean_options.drawDisplacementMap=false
	boolean_options.useOrientationCorrection=false
	boolean_options.solveIK=true
	boolean_options.drawPredictedCOM=false
	boolean_options.drawPredictedZMP =false
	boolean_options.drawPelvis=false
	boolean_options.drawControlForce=false
	boolean_options.drawMatching=false
	
	useCases.unmapControlParam(useCase)

	mSynthesis:initializeControlParameters()
	mSynthesis:resetInitialState()
	mSynthesis:__updateConstraints()

	initDir()

	if g_recordSimulation then	
		initRecord()
	end

	if g_plotdata then
		initPlotData()
	end

end

function initDir()
	
	resultDir = "result/"
	if directory_exists(resultDir) == false then
		os.execute( 'mkdir ' .. resultDir )
	end

	dataname = useCase.modelName..'__'..g_mode
	dataDir = resultDir..dataname..'/'
	if directory_exists(dataDir) == false then
		os.execute( 'mkdir ' .. dataDir )
	end	

end

function initRecord()
	
	-- dcjo
	REF_TIME = "ref_time"
	POSEDOF = "poseDOF"
	JOINT_TORQUE = "joint_torque"
--	MUSCLE_ACTIVATION = "muscle_activation"
--	MUSCLE_FORCE = "muscle_force"

	rSim = {}
	rSim[REF_TIME] = {}
	rSim[POSEDOF] = {}
	rSim[JOINT_TORQUE] = {}
--	rSIM[MUSCLE_ACTIVATION] = {}
--	rSIM[MUSCLE_FORCE] = {}

end

function recordSimulation(synthesis, osim)

	local frame = synthesis.numFrames
	local refT = synthesis.outputGlobal:getRefTime()
	rSim[REF_TIME][frame] = refT

	local poseDOF = vectorn()
	osim.bfk:getPoseDOFfromGlobal(poseDOF)
	rSim[POSEDOF][frame] = poseDOF

	rSim[JOINT_TORQUE] = osim.jointTorques

end

function saveRecord()
	
	util.saveTable(rSim, dataDir..dataname..'.rsim')

end

function saveReplay()

	util.saveTable(mOsim.records, dataDir..dataname..'.replay')
	print("replay saved")

end


function initPlotData()

	mFiles = {}
	mFileNames = {'lmt', 'act', 'tor', 'dev', 'art', 'atq', 'ptq', 'ctq', 'mtq', 'ang', 'ang_s', 'pos', 'vel', 'grf', 'eng'}
	-- muscle
	mMsclFileNames = {'lmt', 'act'}
	-- joint
	-- mtq: muscle torque
	-- tor: torque, dev : assist device torque, art : artificial joint torque
	-- atq: active torque, ptq: passive torque, ctq: contact torque, mtq: muscle torque
	mJointFileNames = {'tor', 'dev', 'art', 'atq', 'ptq', 'ctq', 'mtq'}
	-- kinematic
	mKineFileNames = {'ang', 'ang_s', 'pos', 'vel'}
	-- grf
	mGrfFileNames = {'grf'}
	-- energy
	mEngFileNames = {'eng'}


	for i,name in ipairs(mFileNames) do
		mFiles[name] = io.open(dataDir..'/__'..name..'.txt', 'w')
	end

	local muscleNames = mOsim:getMuscleNames()
	muscleIndices = {}
	for i, mn in ipairs(muscleNames) do
		muscleIndices[i] = mOsim:getMuscleIndexWithName(mn)
	end
	for f, fileName in ipairs(mMsclFileNames) do
		mFiles[fileName]:write('frame\t')
		mFiles[fileName]:write('ref_time\t')
		for m, msclName in ipairs(muscleNames) do
			mFiles[fileName]:write(msclName..'\t')
		end
		mFiles[fileName]:write('\n')
	end

	local jointNames = mOsim:getJointNames()
	local channelNames = mOsim:getChannelNames()
	for f, fileName in ipairs(mJointFileNames) do
		mFiles[fileName]:write('frame\t')
		mFiles[fileName]:write('ref_time\t')
		for j, channelName in ipairs(channelNames) do
			print(channelName)
			print(fileName)
			mFiles[fileName]:write(channelName..'\t')
		end
		mFiles[fileName]:write('\n')
	end

	-- 'ang' : biomechnical angle of DOFs
--	local angleNames = {'hip_r.ang_flx', 'knee_r.ang_flx', 'ankle_r.ang_dor', 'pelvis.ang_roll'}
--	mAngleIndices = {mOsim.rhipflexDOF, mOsim.rkneeDOF, mOsim.rankleDOF, -1}
	local angleNames = {'hip_r.flx', 'hip_r.add', 'hip_r.int', 'knee_r.flx', 'ankle_r.dor', 'ankle_r.inv', 'hip_l.flx', 'hip_l.add', 'hip_l.int', 'knee_l.flx', 'ankle_l.dor', 'ankle_l.inv', 'pelvis.tilt', 'pelvis.list', 'pelvis.rot',}
	mAngleIndices = {mOsim.rhipflexDOF, mOsim.rhipflexDOF+1, mOsim.rhipflexDOF+2, mOsim.rkneeDOF, mOsim.rankleDOF, mOsim.rankleDOF+1, mOsim.lhipflexDOF, mOsim.lhipflexDOF+1, mOsim.lhipflexDOF+2, mOsim.lkneeDOF, mOsim.lankleDOF, mOsim.lankleDOF+1, -1, -2, -3}
	mAngleDirections = {1, 1, 1, -1, 1, 1, 1, -1, -1, -1, 1, -1, 0, 0, 0}
	fileName = 'ang'
	mFiles[fileName]:write('frame\t')
	mFiles[fileName]:write('ref_time\t')
	for a, angName in ipairs(angleNames) do
		mFiles[fileName]:write(angName..'\t')
	end
	mFiles[fileName]:write('\n')

	-- 'ang_s' : angle of DOFs
	-- channel [1:6] = DQ of ground pelvis
	-- treeindex of DOFs : [0:25], that of ground pelves : [0:6]
	local angleSNames = {}
	mAngleSIndices = {}
	fileName = 'ang_s'
	for i = 7, #channelNames do
		table.insert(angleSNames, channelNames[i])
		table.insert(mAngleSIndices, i)
	end
	mFiles[fileName]:write('frame\t')
	mFiles[fileName]:write('ref_time\t')
	for a, angName in ipairs(angleSNames) do
		mFiles[fileName]:write(angName..'\t')
	end
	mFiles[fileName]:write('\n')

	-- 'pos' : global position of joint
	fileName = 'pos'
	mFiles[fileName]:write('frame\t')
	mFiles[fileName]:write('ref_time\t')
	mFiles[fileName]:write('COM.x\t')
	mFiles[fileName]:write('COM.y\t')
	mFiles[fileName]:write('COM.z\t')
	for _, j in ipairs(jointNames) do
		mFiles[fileName]:write(j..'.x\t')
		mFiles[fileName]:write(j..'.y\t')
		mFiles[fileName]:write(j..'.z\t')
	end
	mFiles[fileName]:write('\n')

	fileName = 'vel'
	mFiles[fileName]:write('frame\t')
	mFiles[fileName]:write('ref_time\t')
	mFiles[fileName]:write('COM\t')
	mFiles[fileName]:write('COM_avg\t')
	mFiles[fileName]:write('gp\t')
	mFiles[fileName]:write('gp_avg\t')
	mFiles[fileName]:write('\n')

	local grfNames = {'rfoot.grf.x', 'rfoot.grf.y', 'rfoot.grf.z', 'lfoot.grf.x', 'lfoot.grf.y', 'lfoot.grf.z'}
	for f, fileName in ipairs(mGrfFileNames) do
		mFiles[fileName]:write('frame\t')
		mFiles[fileName]:write('ref_time\t')
		for g, grfName in ipairs(grfNames) do
			mFiles[fileName]:write(grfName..'\t')
		end
		mFiles[fileName]:write('\n')
	end

	fileName = 'eng'
	energyNames = {'met', 'avgmet', 'kcalrate', 'kcal', 'kcaldist'}
	mFiles[fileName]:write('frame\t')
	mFiles[fileName]:write('ref_time\t')
	for _, n in ipairs(energyNames) do
		mFiles[fileName]:write(n..'\t')
	end
	for _, m in ipairs(muscleNames) do
		for _, n in ipairs(energyNames) do
			mFiles[fileName]:write(m..'_'..n..'\t')
		end
	end
	mFiles[fileName]:write('\n')

end

function writeData(mFile, frame, data, refTime)

	mFile:write(frame..'\t')
	if refTime ~= nil then
		mFile:write(refTime..'\t')
	end
	for m, d in ipairs(data) do
		mFile:write(d..'\t')
	end
	mFile:write('\n')

end


function plotData()

	local frame = mSynthesis.numFrames
	local refTime = mSynthesis.outputGlobal:getRefTime()

	writeData(mFiles['lmt'], frame, mOsim:getTendonMuscleLengths(), refTime)
	writeData(mFiles['act'], frame, mOsim:getActivations(), refTime)

	-- joint index = tree index
	-- joint num = mOsim.mLoader:numBone() - 1
	-- mtq: muscle torque
	-- tor: torque, dev : assist device torque, art : artificial joint torque
	-- atq: active torque, ptq: passive torque, ctq: contact torque, mtq: muscle torque
	-- mJointFileNames = {'tor', 'dev', 'art', 'atq', 'ptq', 'ctq', 'mtq'}
	writeData(mFiles['tor'], frame, vecns2tbl(mOsim:getTotalTorques()), refTime)
	writeData(mFiles['dev'], frame, vecns2tbl(mOsim:getDeviceTorques()), refTime)
	if useCase.artificialJoints ~= nil then
		writeData(mFiles['art'], frame, vecns2tbl(mOsim:getArtificialTorques()), refTime)
	end
	writeData(mFiles['atq'], frame, vecns2tbl(mOsim:getActiveTorques()), refTime)
	writeData(mFiles['ptq'], frame, vecns2tbl(mOsim:getPassiveTorques()), refTime)
	writeData(mFiles['ctq'], frame, vecns2tbl(mOsim:getContactTorques()), refTime)
	writeData(mFiles['mtq'], frame, vecns2tbl(mOsim:getTotalMuscleTorques()), refTime)

	-- 'ang'
	local posedof = vectorn()
	mOsim.bfk:getPoseDOFfromGlobal(posedof)
	local angData = {}
	local rootTF = MotionDOF.rootTransformation(posedof)
	local rotY = rootTF.rotation:rotationY()
	local localRot = rotY:inverse()*rootTF.rotation
	for j, angIndex in ipairs(mAngleIndices) do
		local angValue
		if angIndex == -1 then -- pelvis roll, tilt
			angValue = localRot:rotationAngleAboutAxis(vector3(1,0,0))
		elseif angIndex == -2 then -- pelvis pitch, list
			angValue = localRot:rotationAngleAboutAxis(vector3(0,1,0))
		elseif angIndex == -3 then -- pelvis yaw, transverse rotation
			angValue = localRot:rotationAngleAboutAxis(vector3(0,0,1))
		else
			angValue = mAngleDirections[j] * posedof(angIndex)
		end
		angData[j] = angValue
	end
	writeData(mFiles['ang'], frame, angData, refTime)

	-- 'ang_s'
	local angSData = {}
	for j, angIndex in ipairs(mAngleSIndices) do
		angSData[j] = posedof(angIndex)
	end
	writeData(mFiles['ang_s'], frame, angSData, refTime)

	-- 'pos'
	local posData = {}
	local COM = mOsim.mLoader:calcCOM()
	table.insert(posData, COM.x)
	table.insert(posData, COM.y)
	table.insert(posData, COM.z)
	for i=1,mOsim:getNumJoints() do
		local pos = mOsim.bfk:globalFrame(i):toGlobalPos(vector3(0,0,0))
		table.insert(posData, pos.x)
		table.insert(posData, pos.y)
		table.insert(posData, pos.z)
	end
	writeData(mFiles['pos'], frame, posData, refTime)

	-- 'vel' km/h
	-- without y axis
	local velData = {}
	local gp_woy = mOsim.bfk:globalFrame(1):toGlobalPos(vector3(0,0,0))
	gp_woy.y = 0.
	local COM_woy = vector3(COM.x, 0.0, COM.z)
	if frame == 1 then
		COM_start = COM_woy
		gp_start = gp_woy
	elseif frame > 1 then
		table.insert(velData, (COM_woy-COM_prev):length()/(1/120.)*3.6)
		table.insert(velData, (COM_woy-COM_start):length()/(frame/120.)*3.6)
		table.insert(velData, (gp_woy-gp_prev):length()/(1/120.)*3.6)
		table.insert(velData, (gp_woy-gp_start):length()/(frame/120.)*3.6)
	end
	COM_prev = COM_woy
	gp_prev = gp_woy
	writeData(mFiles['vel'], frame, velData, refTime)

	-- 'grf'
	local rcf = mOsim:getRFootCF()
	local lcf = mOsim:getLFootCF()
	local grfData = {rcf.x, rcf.y, rcf.z, lcf.x, lcf.y, lcf.z}
	writeData(mFiles['grf'], frame, grfData, refTime)
	
	-- 'eng'
	local engData = {}
	if g_kcal == nil then g_kcal = 0.0 end
	if g_int_met == nil then g_int_met = 0.0 end
	if g_kcal_ml == nil then g_kcal_ml = {} end
	if g_int_met_ml == nil then g_int_met_ml = {} end
	local energyRate = mOsim:getMetabolicEnergyRate()
	local kcalrate = energyRate*(1./4.184)*0.001
	local met = (kcalrate*3600)/75.
	g_kcal = g_kcal + kcalrate*(1/120.)
	g_int_met = g_int_met + met
	table.insert(engData, met)
	table.insert(engData, g_int_met/frame)
	table.insert(engData, kcalrate)
	table.insert(engData, g_kcal)
	table.insert(engData, g_kcal/(gp_woy-gp_start):length())
	for i, mi in ipairs(muscleIndices) do
		local pM = mOsim:getPartialMetabolicEnergyRateOfMuscles({mi})
		if g_kcal_ml[i] == nil then g_kcal_ml[i] = 0.0 end
		if g_int_met_ml[i] == nil then g_int_met_ml[i] = 0.0 end
		local kcalrate = pM[5]*(1./4.184)*0.001
		local met = (kcalrate*3600)/75.
		g_kcal_ml[i] = g_kcal_ml[i] + kcalrate*(1/120.)
		g_int_met_ml[i] = g_int_met_ml[i] + met
		table.insert(engData, met)
		table.insert(engData, g_int_met_ml[i]/frame)
		table.insert(engData, kcalrate)
		table.insert(engData, g_kcal_ml[i])
		table.insert(engData, g_kcal_ml[i]/(gp_woy-gp_start):length())
	end
	writeData(mFiles['eng'], frame, engData, refTime)

end

function finalizePlotData()
	
	for f, fileName in ipairs(mFileNames) do
		io.close(mFiles[fileName])
	end

end

function dtor()
	if g_saveReplay then
		saveReplay()
	end

	if g_recordSimulation then
		saveRecord()
	end

	if g_plotdata then
		finalizePlotData()
	end

	dbg.finalize()
	if mSynthesis then
		mSynthesis:__finalize()
		mSynthesis = nil
	end
end

function frameMove(fElapsedTime)
	mSynthesis:oneStep()
	mSynthesis:oneStepSimul()
	print(mSynthesis.numFrames)

	if error_feedback_method~=EFM.NONE and mSynthesis.errorOccurred~=true then	 
		mSynthesis:prepareNextStep()      
	else
		print("simulation ends")
		dtor()	
		os.exit()
	end

	if g_saveReplay then
		mOsim:drawMuscles(mSynthesis.numFrames, mObjectList)
	end

	if g_recordSimulation then
		recordSimulation(mSynthesis, mOsim)
	end

	if g_plotdata then
		plotData()
	end

	if useCase.endFrame ~= nil then
		if mSynthesis.numFrames > useCase.endFrame then
			print("test ends")
			dtor()
			os.exit()
		end
	end

end

function EVR:onFrameChanged(win, iframe)
end
