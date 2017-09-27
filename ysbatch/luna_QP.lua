bindTargetClassification={
	classes={},
	modules={
		{
			namespace='Eigen',
			functions={[[
			double solve_quadprog( HessianQuadratic & problem, const matrixn & CE, const vectorn & ce0, const matrixn & CI, const vectorn & ci0, vectorn & x) @ solveQuadprog
			double solve_quadprog( HessianQuadratic & problem, const matrixn & CE, const vectorn & ce0, const matrixn & CI, const vectorn & ci0, vectorn & x, bool) @ solveQuadprog
			void solveLCP(const matrixn&  N, const vectorn& r, vectorn& g, vectorn & a);
			]]}
		}, 
	},
}
function generate()
	loadDefinitionDB(script_path..'/luna_baselib_db.lua')
	loadDefinitionDB(script_path..'/../../PhysicsLib/luna_physicslib_db.lua')
	buildDefinitionDB(bindTargetClassification)
	write(
	[[
	#include "stdafx.h"
//			double solve_quadprog_using_qpOASES(const matrixn & G,  const vectorn & g0,  const matrixn & CE, const vectorn & ce0,  const matrixn & CI, const vectorn & ci0, vectorn & x, bool g0_negative) ;
	]]);
	writeIncludeBlock()
	write('#include "../../MainLib/WrapperLua/luna.h"')
	write('#include "../../MainLib/WrapperLua/luna_baselib.h"')
	write('#include "../../PhysicsLib/luna_physics.h"')
	write('#include "quadprog.h"')
	writeHeader(bindTargetClassification)
	writeDefinitions(bindTargetClassification, 'Register_QP') -- input bindTarget can be non-overlapping subset of entire bindTarget 
	flushWritten(source_path..'/luna_QP.cpp') -- write to cpp file only when there exist modifications -> no-recompile.
end
