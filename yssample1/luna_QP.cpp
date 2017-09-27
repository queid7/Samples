	#include "stdafx.h"
//			double solve_quadprog_using_qpOASES(const matrixn & G,  const vectorn & g0,  const matrixn & CE, const vectorn & ce0,  const matrixn & CI, const vectorn & ci0, vectorn & x, bool g0_negative) ;
	
		#include <stdio.h>
		#include <string.h>
		#include <string>
		extern "C"
		{
			#include "lua.h"
			#include "lualib.h"
			#include "lauxlib.h"
			//#include "luadebug.h"
		}
		
#include "../../MainLib/WrapperLua/luna.h"
#include "../../MainLib/WrapperLua/luna_baselib.h"
#include "../../PhysicsLib/luna_physics.h"
#include "quadprog.h"
#ifndef genlua_luna_QP_lua15473501_def12                      // 1209
#define genlua_luna_QP_lua15473501_def12                      // 1210
// declare all classes before including this file             // 1211
// e.g. class LMat; class LMatView; ....                      // 1212
// The forward declaration is not included here because luna_gen cannot distinguish struct, class, or namespace. // 1213
// : number denotes the line number of luna_gen.lua which generated that line // 1216
 class luna__interface__Eigen {
public:                                                       // 1233
    static const char moduleName[];                           // 1238
    typedef LunaModule<luna__interface__Eigen> luna_t;        // 1239
    static luna_RegType methods[];                            // 1240
};                                                            // 1255
#endif                                                        // 1260
 class impl_luna__interface__Eigen {
public:                                                       // 1291
    typedef LunaModule<luna__interface__Eigen> luna_t;        // 1293
// : number denotes the line number of luna_gen.lua that generated the sentence // 1298
  inline static bool _lg_typecheck_solveQuadprog_overload_1(lua_State *L)
  {                                                           // 1317
    if( lua_gettop(L)!=6) return false;                       // 564
    if( Luna<void>::get_uniqueid(L,1)!=55023510) return false; // HessianQuadratic // 572
    if( Luna<void>::get_uniqueid(L,2)!=23735758) return false; // matrixn // 572
    if( Luna<void>::get_uniqueid(L,3)!=10150210) return false; // vectorn // 572
    if( Luna<void>::get_uniqueid(L,4)!=23735758) return false; // matrixn // 572
    if( Luna<void>::get_uniqueid(L,5)!=10150210) return false; // vectorn // 572
    if( Luna<void>::get_uniqueid(L,6)!=10150210) return false; // vectorn // 572
    return true;
  }                                                           // 592
  inline static bool _lg_typecheck_solveQuadprog_overload_2(lua_State *L)
  {                                                           // 1317
    if( lua_gettop(L)!=7) return false;                       // 564
    if( Luna<void>::get_uniqueid(L,1)!=55023510) return false; // HessianQuadratic // 572
    if( Luna<void>::get_uniqueid(L,2)!=23735758) return false; // matrixn // 572
    if( Luna<void>::get_uniqueid(L,3)!=10150210) return false; // vectorn // 572
    if( Luna<void>::get_uniqueid(L,4)!=23735758) return false; // matrixn // 572
    if( Luna<void>::get_uniqueid(L,5)!=10150210) return false; // vectorn // 572
    if( Luna<void>::get_uniqueid(L,6)!=10150210) return false; // vectorn // 572
    if( lua_isboolean(L,7)==0) return false;                  // 584
    return true;
  }                                                           // 592
  inline static bool _lg_typecheck_solveLCP(lua_State *L)
  {                                                           // 1317
    if( lua_gettop(L)!=4) return false;                       // 564
    if( Luna<void>::get_uniqueid(L,1)!=23735758) return false; // matrixn // 572
    if( Luna<void>::get_uniqueid(L,2)!=10150210) return false; // vectorn // 572
    if( Luna<void>::get_uniqueid(L,3)!=10150210) return false; // vectorn // 572
    if( Luna<void>::get_uniqueid(L,4)!=10150210) return false; // vectorn // 572
    return true;
  }                                                           // 592
  static int _bind_solveQuadprog_overload_1(lua_State *L)
  {                                                           // 1350
    HessianQuadratic & problem=static_cast<HessianQuadratic &>(*Luna<HessianQuadratic >::check(L,1)); // 538
    const matrixn & CE=static_cast<matrixn &>(*Luna<matrixn >::check(L,2)); // 538
    const vectorn & ce0=static_cast<vectorn &>(*Luna<vectorn >::check(L,3)); // 538
    const matrixn & CI=static_cast<matrixn &>(*Luna<matrixn >::check(L,4)); // 538
    const vectorn & ci0=static_cast<vectorn &>(*Luna<vectorn >::check(L,5)); // 538
    vectorn & x=static_cast<vectorn &>(*Luna<vectorn >::check(L,6)); // 538
    try {                                                     // 328
    double ret=solve_quadprog( problem, CE, ce0, CI, ci0, x); // 329
    lua_pushnumber(L, ret);                                   // 330
    } 
    catch(std::exception& e) { luaL_error( L,e.what()); }
    catch(...) { luaL_error( L,"unknown_error");}
                                                              // 331
    return 1;                                                 // 332
  }                                                           // 362
  static int _bind_solveQuadprog_overload_2(lua_State *L)
  {                                                           // 1350
    HessianQuadratic & problem=static_cast<HessianQuadratic &>(*Luna<HessianQuadratic >::check(L,1)); // 538
    const matrixn & CE=static_cast<matrixn &>(*Luna<matrixn >::check(L,2)); // 538
    const vectorn & ce0=static_cast<vectorn &>(*Luna<vectorn >::check(L,3)); // 538
    const matrixn & CI=static_cast<matrixn &>(*Luna<matrixn >::check(L,4)); // 538
    const vectorn & ci0=static_cast<vectorn &>(*Luna<vectorn >::check(L,5)); // 538
    vectorn & x=static_cast<vectorn &>(*Luna<vectorn >::check(L,6)); // 538
    bool _arg7=(bool)lua_toboolean(L,7);                      // 549
    try {                                                     // 328
    double ret=solve_quadprog( problem, CE, ce0, CI, ci0, x, _arg7); // 329
    lua_pushnumber(L, ret);                                   // 330
    } 
    catch(std::exception& e) { luaL_error( L,e.what()); }
    catch(...) { luaL_error( L,"unknown_error");}
                                                              // 331
    return 1;                                                 // 332
  }                                                           // 362
  static int _bind_solveLCP(lua_State *L)
  {                                                           // 1350
    if (!_lg_typecheck_solveLCP(L)) { char msg[]="luna typecheck failed:\n  solveLCP(const matrixn & N,const vectorn & r,vectorn & g,vectorn & a,)"; puts(msg); luna_printStack(L); luaL_error(L, msg); }
                                                              // 518
    const matrixn & N=static_cast<matrixn &>(*Luna<matrixn >::check(L,1)); // 538
    const vectorn & r=static_cast<vectorn &>(*Luna<vectorn >::check(L,2)); // 538
    vectorn & g=static_cast<vectorn &>(*Luna<vectorn >::check(L,3)); // 538
    vectorn & a=static_cast<vectorn &>(*Luna<vectorn >::check(L,4)); // 538
    try {                                                     // 280
    solveLCP( N, r, g, a);                                    // 281
    } 
    catch(std::exception& e) { luaL_error( L,e.what()); }
    catch(...) { luaL_error( L,"unknown_error");}
                                                              // 282
    return 0;                                                 // 283
  }                                                           // 362
  static int _bind_solveQuadprog(lua_State *L)
  {                                                           // 221
    if (_lg_typecheck_solveQuadprog_overload_1(L)) return _bind_solveQuadprog_overload_1(L); // 224
    if (_lg_typecheck_solveQuadprog_overload_2(L)) return _bind_solveQuadprog_overload_2(L); // 224
    luaL_error(L, "solveQuadprog ( cannot find overloads:)\n(HessianQuadratic & problem,const matrixn & CE,const vectorn & ce0,const matrixn & CI,const vectorn & ci0,vectorn & x,)\n(HessianQuadratic & problem,const matrixn & CE,const vectorn & ce0,const matrixn & CI,const vectorn & ci0,vectorn & x,bool _arg7,)\n");
                                                              // 231
    return 0;                                                 // 232
  }                                                           // 233
}; // end of class impl_luna__interface__Eigen                // 1511
const char luna__interface__Eigen::moduleName[] = "_Eigen";   // 1539
luna_RegType luna__interface__Eigen::methods[] = {            // 1548
    {"solveLCP", &impl_luna__interface__Eigen::_bind_solveLCP}, // 1553
    {"solveQuadprog", &impl_luna__interface__Eigen::_bind_solveQuadprog}, // 1553
    {0,0}                                                     // 1556
};                                                            // 1557
void Register_QP(lua_State* L) {                              // 1564
    luna_dostring(L,"if __luna==nil then __luna={} end");     // 1565
    luna_dostring(L,"    if __luna.copyMethodsFrom==nil then\n        function __luna.copyMethodsFrom(methodsChild, methodsParent)\n            for k,v in pairs(methodsParent) do\n                if k~='__index' and k~='__newindex' and methodsChild[k]==nil then\n                    methodsChild[k]=v\n                end\n            end\n        end\n        function __luna.overwriteMethodsFrom(methodsChild, methodsParent)\n            for k,v in pairs(methodsParent) do\n                if k~='__index' and k~='__newindex' then\n                    if verbose then print('registering', k, methodsChild[k]) end\n                    methodsChild[k]=v\n                end\n            end\n        end\n    end\n    "); // 1566
   LunaModule<luna__interface__Eigen >::Register(L);          // 1605
    luna_dostring(L," \n                if Eigen==nil then \n                    Eigen={}\n                end \n                __luna.overwriteMethodsFrom(Eigen, __luna._Eigen)\n                "); // 1620
}                                                             // 1661