require("config")

package.projectPath='../Samples/QP_controller/'
package.path=package.path..";../Samples/QP_controller/lua/?.lua" --;"..package.path
package.path=package.path..";../Samples/classification/lua/?.lua" --;"..package.path
package.path=package.path..";../Resource/classification/lua/?.lua" --;"..package.path
require('RagdollFall')
require('subRoutines/ConvexHull2D')
require("module")
require('subRoutines/QPservo')

servoMethod=ServoMethods.QP
timestep=1/120

-- only for QP servo
--useCase.k_p_ID=250
--useCase.k_d_ID=35
--useCase.tauObjWeight=0.0001
--useCase.lambdaObjWeight=0.0001
--useCase.ddqObjWeight=10000

-- ys
useCase.k_p_ID=250
useCase.k_d_ID=2.*math.sqrt(useCase.k_p_ID)
useCase.ddqObjWeight=10000000
--useCase.ddqObjWeight=1000000000000
useCase.ddqObjWeight_flight=10000
useCase.tauObjWeight=1
useCase.ftObjWeight=1
useCase.aObjWeight=10000000000
useCase.lambdaObjWeight=1

useCase.excludeRoot=true
useCase.excludeRootFlight=true

-- not changed
useCase.contactMargin=0.005
useCase.velMarginStrength=1
useCase.invAccConeCoef =0.1

-- main functions are in the RagdollFall.lua

--simulationMode=SimulationMode.PoseMaintain
--model_files.muscleModel=deepCopyTable(model_files.default)
--model_files.muscleModel.file_name=package.resourcePath.."gymnist/gymnist.wrl"
--model_files.muscleModel.mot_file=package.resourcePath.."gymnist/gymnist.dof"
--model_files.muscleModel.bones.chest="lowerback"
--model_files.muscleModel.bones.chest2="upperback"
--model_files.muscleModel.bones.head="Neck"
--model_files.muscleModel.totalmass=60
--model_files.muscleModel.start=0
--model_files.muscleModel.initialHeight=0.07
--model_files.muscleModel.frame_rate=120

--model=model_files.muscleModel
if true then
model_files.muscleModel=deepCopyTable(model_files.gymnist)
model_files.muscleModel.wrlpath=package.resourcePath.."opensim/gait23_2geometry.wrl"
model_files.muscleModel.file_name=package.resourcePath.."opensim/gait23_2geometry.wrl"
model_files.muscleModel.luamsclpath=package.resourcePath.."opensim/gait2354_render.luamscl"
model_files.muscleModel.mot_file=package.resourcePath.."opensim/gait23_6steps_120.dof"

model_files.muscleModel.bones={
	left_heel="ankle_l"
	,left_ball="mtp_l"
	,left_hip="hip_l"
	,left_knee="knee_l"
	,right_heel="ankle_r"
	,right_ball="mtp_r"
	,right_hip="hip_r"
	,right_knee="knee_r"
}

model_files.muscleModel.initialHeight=.5 -- meters
model=model_files.muscleModel

end
