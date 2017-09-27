bindTargetClassification={
	classes={
		{
			name='Optimize',
			className='Optimize_lunawrapper',
			decl=true,
			isLuaInheritable=true,
			isExtendableFromLua=true,
			globalWrapperCode=[[
					class Optimize_lunawrapper: public Optimize, public luna_wrap_object
					{
						public:
						Optimize_lunawrapper() :Optimize() { }

						virtual double objectiveFunction(vectorn const& pos)
						{
							lunaStack l(_L);
							if(pushMemberFunc<Optimize_lunawrapper>(l,"_objectiveFunction")){
								l.push<vectorn>(pos);
								l.call(2,1);
								double out;
								l>>out;
								return out;
							} 
							return 0;
						}
					};
			]],
			ctor='()',
			memberFunctions=[[
			virtual double objectiveFunction(vectorn const& pos)
			void optimize(vectorn const& initialSolution)
			vectorn& getResult()
			void init(double stepSize, int ndim, double max_step, double grad_step, Optimize::Method & method)
			]]
		},
		{
			name='math.CMAwrap',
			cppname='CMAwrap',
			decl=true,
			ctors=[[
				(vectorn const& start_p, vectorn const& stdev, int populationSize, int mu);
				(vectorn const& start_p, vectorn const& stdev, int populationSize);
			]],
			memberFunctions=[[
			std::string testForTermination();	
			void samplePopulation();
			int numPopulation();
			int dim();
			vectornView getPopulation(int i);
			void setVal(int i, double eval);
			void resampleSingle(int i);
			void update();
			void getMean(vectorn& out);
			void getBest(vectorn& out);
			]]
		},
		{
			name='Optimize.Method'
		},
		{
			luaname='Optimize.LBFGS_METHOD',
			cppname='LBFGS_METHOD',
			super='Optimize.Method',
			ctor=[[
			()
			(double epsilon)
			]]
		}
	},
	modules={},
}
function generate()
	loadDefinitionDB(script_path..'/luna_baselib_db.lua')
	buildDefinitionDB(bindTargetClassification)
	write(
	[[
	#include "stdafx.h"
	]]);
	writeIncludeBlock()
	write([[
	#include "../../MainLib/WrapperLua/luna.h"
	#include "../../MainLib/WrapperLua/luna_baselib.h"
	#include "optimizer_lbfgs.hpp"
	#include "cma/CMAwrap.h"
	]])
	--write('#include "../../MainLib/WrapperLua/luna_mainlib.h"')
	writeHeader(bindTargetClassification)
	writeDefinitions(bindTargetClassification, 'Register_classification') -- input bindTarget can be non-overlapping subset of entire bindTarget 
	flushWritten(script_path..'/../../Samples/classification/luna_classification.cpp') -- write to cpp file only when there exist modifications -> no-recompile.
end
