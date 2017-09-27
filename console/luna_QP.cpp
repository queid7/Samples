	#include "stdafx.h"
	
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
#ifdef USE_MPI
	#include <mpi.h>
	static  MPI_Status status;
#endif
	#include "../classification/cma/CMAwrap.h"

#ifndef genlua_luna_QP_lua15473501_def12                      // 1209
#define genlua_luna_QP_lua15473501_def12                      // 1210
// declare all classes before including this file             // 1211
// e.g. class LMat; class LMatView; ....                      // 1212
// The forward declaration is not included here because luna_gen cannot distinguish struct, class, or namespace. // 1213
// : number denotes the line number of luna_gen.lua which generated that line // 1216
class CMAwrap;                                                // 1220
template<>                                                    // 1231
 class LunaTraits<CMAwrap > {
public:                                                       // 1233
    static const char className[];                            // 1242
    static const int uniqueID;                                // 1243
    static luna_RegType methods[];                            // 1244
    static CMAwrap* _bind_ctor(lua_State *L);                 // 1246
    static void _bind_dtor(CMAwrap* obj);                     // 1247
    typedef CMAwrap base_t;                                   // 1249
};                                                            // 1255
 class luna__interface__Eigen {
public:                                                       // 1233
    static const char moduleName[];                           // 1238
    typedef LunaModule<luna__interface__Eigen> luna_t;        // 1239
    static luna_RegType methods[];                            // 1240
};                                                            // 1255
#if defined (USE_MPI)                                         // 1227
 class luna__interface__MPI {
public:                                                       // 1233
    static const char moduleName[];                           // 1238
    typedef LunaModule<luna__interface__MPI> luna_t;          // 1239
    static luna_RegType methods[];                            // 1240
};                                                            // 1255
#endif //defined (USE_MPI)                                    // 1257
#endif                                                        // 1260
template<>                                                    // 1288
 class impl_LunaTraits<CMAwrap > {
public:                                                       // 1291
    typedef Luna<CMAwrap > luna_t;                            // 1295
// : number denotes the line number of luna_gen.lua that generated the sentence // 1298
  inline static bool _lg_typecheck_ctor_overload_1(lua_State *L)
  {                                                           // 1307
    if( lua_gettop(L)!=4) return false;                       // 564
    if( Luna<void>::get_uniqueid(L,1)!=10150210) return false; // vectorn // 572
    if( Luna<void>::get_uniqueid(L,2)!=10150210) return false; // vectorn // 572
    if( lua_isnumber(L,3)==0) return false;                   // 574
    if( lua_isnumber(L,4)==0) return false;                   // 574
    return true;
  }                                                           // 592
  inline static bool _lg_typecheck_ctor_overload_2(lua_State *L)
  {                                                           // 1307
    if( lua_gettop(L)!=3) return false;                       // 564
    if( Luna<void>::get_uniqueid(L,1)!=10150210) return false; // vectorn // 572
    if( Luna<void>::get_uniqueid(L,2)!=10150210) return false; // vectorn // 572
    if( lua_isnumber(L,3)==0) return false;                   // 574
    return true;
  }                                                           // 592
  inline static bool _lg_typecheck_testForTermination(lua_State *L)
  {                                                           // 1317
    if( lua_gettop(L)!=1) return false;                       // 564
    if( Luna<void>::get_uniqueid(L,1)!=64707780) return false; // CMAwrap // 572
    return true;
  }                                                           // 592
  inline static bool _lg_typecheck_samplePopulation(lua_State *L)
  {                                                           // 1317
    if( lua_gettop(L)!=1) return false;                       // 564
    if( Luna<void>::get_uniqueid(L,1)!=64707780) return false; // CMAwrap // 572
    return true;
  }                                                           // 592
  inline static bool _lg_typecheck_numPopulation(lua_State *L)
  {                                                           // 1317
    if( lua_gettop(L)!=1) return false;                       // 564
    if( Luna<void>::get_uniqueid(L,1)!=64707780) return false; // CMAwrap // 572
    return true;
  }                                                           // 592
  inline static bool _lg_typecheck_dim(lua_State *L)
  {                                                           // 1317
    if( lua_gettop(L)!=1) return false;                       // 564
    if( Luna<void>::get_uniqueid(L,1)!=64707780) return false; // CMAwrap // 572
    return true;
  }                                                           // 592
  inline static bool _lg_typecheck_getPopulation(lua_State *L)
  {                                                           // 1317
    if( lua_gettop(L)!=2) return false;                       // 564
    if( Luna<void>::get_uniqueid(L,1)!=64707780) return false; // CMAwrap // 572
    if( lua_isnumber(L,2)==0) return false;                   // 574
    return true;
  }                                                           // 592
  inline static bool _lg_typecheck_setVal(lua_State *L)
  {                                                           // 1317
    if( lua_gettop(L)!=3) return false;                       // 564
    if( Luna<void>::get_uniqueid(L,1)!=64707780) return false; // CMAwrap // 572
    if( lua_isnumber(L,2)==0) return false;                   // 574
    if( lua_isnumber(L,3)==0) return false;                   // 574
    return true;
  }                                                           // 592
  inline static bool _lg_typecheck_resampleSingle(lua_State *L)
  {                                                           // 1317
    if( lua_gettop(L)!=2) return false;                       // 564
    if( Luna<void>::get_uniqueid(L,1)!=64707780) return false; // CMAwrap // 572
    if( lua_isnumber(L,2)==0) return false;                   // 574
    return true;
  }                                                           // 592
  inline static bool _lg_typecheck_update(lua_State *L)
  {                                                           // 1317
    if( lua_gettop(L)!=1) return false;                       // 564
    if( Luna<void>::get_uniqueid(L,1)!=64707780) return false; // CMAwrap // 572
    return true;
  }                                                           // 592
  inline static bool _lg_typecheck_getMean(lua_State *L)
  {                                                           // 1317
    if( lua_gettop(L)!=2) return false;                       // 564
    if( Luna<void>::get_uniqueid(L,1)!=64707780) return false; // CMAwrap // 572
    if( Luna<void>::get_uniqueid(L,2)!=10150210) return false; // vectorn // 572
    return true;
  }                                                           // 592
  inline static bool _lg_typecheck_getBest(lua_State *L)
  {                                                           // 1317
    if( lua_gettop(L)!=2) return false;                       // 564
    if( Luna<void>::get_uniqueid(L,1)!=64707780) return false; // CMAwrap // 572
    if( Luna<void>::get_uniqueid(L,2)!=10150210) return false; // vectorn // 572
    return true;
  }                                                           // 592
  inline static bool _lg_typecheck_getBestFvEver(lua_State *L)
  {                                                           // 1317
    if( lua_gettop(L)!=1) return false;                       // 564
    if( Luna<void>::get_uniqueid(L,1)!=64707780) return false; // CMAwrap // 572
    return true;
  }                                                           // 592
  inline static bool _lg_typecheck_getBestFvRecent(lua_State *L)
  {                                                           // 1317
    if( lua_gettop(L)!=1) return false;                       // 564
    if( Luna<void>::get_uniqueid(L,1)!=64707780) return false; // CMAwrap // 572
    return true;
  }                                                           // 592
  inline static bool _lg_typecheck_getBestSolEver(lua_State *L)
  {                                                           // 1317
    if( lua_gettop(L)!=2) return false;                       // 564
    if( Luna<void>::get_uniqueid(L,1)!=64707780) return false; // CMAwrap // 572
    if( Luna<void>::get_uniqueid(L,2)!=10150210) return false; // vectorn // 572
    return true;
  }                                                           // 592
  inline static bool _lg_typecheck_getBestSolRecent(lua_State *L)
  {                                                           // 1317
    if( lua_gettop(L)!=2) return false;                       // 564
    if( Luna<void>::get_uniqueid(L,1)!=64707780) return false; // CMAwrap // 572
    if( Luna<void>::get_uniqueid(L,2)!=10150210) return false; // vectorn // 572
    return true;
  }                                                           // 592
  inline static CMAwrap* _bind_ctor_overload_1(lua_State *L)
  {                                                           // 1329
    vectorn const & start_p=static_cast<vectorn &>(*Luna<vectorn >::check(L,1)); // 538
    vectorn const & stdev=static_cast<vectorn &>(*Luna<vectorn >::check(L,2)); // 538
    int populationSize=(int)lua_tonumber(L,3);                // 546
    int mu=(int)lua_tonumber(L,4);                            // 546
    return new CMAwrap( start_p, stdev, populationSize, mu);  // 1334
  }                                                           // 1335
  inline static CMAwrap* _bind_ctor_overload_2(lua_State *L)
  {                                                           // 1329
    vectorn const & start_p=static_cast<vectorn &>(*Luna<vectorn >::check(L,1)); // 538
    vectorn const & stdev=static_cast<vectorn &>(*Luna<vectorn >::check(L,2)); // 538
    int populationSize=(int)lua_tonumber(L,3);                // 546
    return new CMAwrap( start_p, stdev, populationSize);      // 1334
  }                                                           // 1335
  static CMAwrap* _bind_ctor(lua_State *L)
  {                                                           // 221
    if (_lg_typecheck_ctor_overload_1(L)) return _bind_ctor_overload_1(L); // 224
    if (_lg_typecheck_ctor_overload_2(L)) return _bind_ctor_overload_2(L); // 224
    luaL_error(L, "ctor ( cannot find overloads:)\n(vectorn const & start_p,vectorn const & stdev,int populationSize,int mu,)\n(vectorn const & start_p,vectorn const & stdev,int populationSize,)\n");
                                                              // 231
    return NULL;                                              // 232
  }                                                           // 233
  static int _bind_testForTermination(lua_State *L)
  {                                                           // 1350
    if (!_lg_typecheck_testForTermination(L)) { char msg[]="luna typecheck failed:\n  testForTermination(CMAwrap& self,)"; puts(msg); luna_printStack(L); luaL_error(L, msg); }
                                                              // 518
    CMAwrap& self=static_cast<CMAwrap &>(*Luna<CMAwrap >::check(L,1)); // 538
    try {                                                     // 334
    std ::string ret=self.testForTermination();               // 335
    lua_pushstring(L, ret.c_str());                           // 338
    } 
    catch(std::exception& e) { luaL_error( L,e.what()); }
    catch(...) { luaL_error( L,"unknown_error");}
                                                              // 342
    return 1;                                                 // 343
  }                                                           // 362
  static int _bind_samplePopulation(lua_State *L)
  {                                                           // 1350
    if (!_lg_typecheck_samplePopulation(L)) { char msg[]="luna typecheck failed:\n  samplePopulation(CMAwrap& self,)"; puts(msg); luna_printStack(L); luaL_error(L, msg); }
                                                              // 518
    CMAwrap& self=static_cast<CMAwrap &>(*Luna<CMAwrap >::check(L,1)); // 538
    try {                                                     // 280
    self.samplePopulation();                                  // 281
    } 
    catch(std::exception& e) { luaL_error( L,e.what()); }
    catch(...) { luaL_error( L,"unknown_error");}
                                                              // 282
    return 0;                                                 // 283
  }                                                           // 362
  static int _bind_numPopulation(lua_State *L)
  {                                                           // 1350
    if (!_lg_typecheck_numPopulation(L)) { char msg[]="luna typecheck failed:\n  numPopulation(CMAwrap& self,)"; puts(msg); luna_printStack(L); luaL_error(L, msg); }
                                                              // 518
    CMAwrap& self=static_cast<CMAwrap &>(*Luna<CMAwrap >::check(L,1)); // 538
    try {                                                     // 328
    int ret=self.numPopulation();                             // 329
    lua_pushnumber(L, ret);                                   // 330
    } 
    catch(std::exception& e) { luaL_error( L,e.what()); }
    catch(...) { luaL_error( L,"unknown_error");}
                                                              // 331
    return 1;                                                 // 332
  }                                                           // 362
  static int _bind_dim(lua_State *L)
  {                                                           // 1350
    if (!_lg_typecheck_dim(L)) { char msg[]="luna typecheck failed:\n  dim(CMAwrap& self,)"; puts(msg); luna_printStack(L); luaL_error(L, msg); }
                                                              // 518
    CMAwrap& self=static_cast<CMAwrap &>(*Luna<CMAwrap >::check(L,1)); // 538
    try {                                                     // 328
    int ret=self.dim();                                       // 329
    lua_pushnumber(L, ret);                                   // 330
    } 
    catch(std::exception& e) { luaL_error( L,e.what()); }
    catch(...) { luaL_error( L,"unknown_error");}
                                                              // 331
    return 1;                                                 // 332
  }                                                           // 362
  static int _bind_getPopulation(lua_State *L)
  {                                                           // 1350
    if (!_lg_typecheck_getPopulation(L)) { char msg[]="luna typecheck failed:\n  getPopulation(CMAwrap& self,int i,)"; puts(msg); luna_printStack(L); luaL_error(L, msg); }
                                                              // 518
    CMAwrap& self=static_cast<CMAwrap &>(*Luna<CMAwrap >::check(L,1)); // 538
    int i=(int)lua_tonumber(L,2);                             // 546
    try {                                                     // 316
    vectornView* ret=new vectornView(self.getPopulation( i)); // 321
    Luna<vectorn >::push(L,ret,true,"_vectornView");          // 322
    } 
    catch(std::exception& e) { luaL_error( L,e.what()); }
    catch(...) { luaL_error( L,"unknown_error");}
                                                              // 323
    return 1;                                                 // 326
  }                                                           // 362
  static int _bind_setVal(lua_State *L)
  {                                                           // 1350
    if (!_lg_typecheck_setVal(L)) { char msg[]="luna typecheck failed:\n  setVal(CMAwrap& self,int i,double eval,)"; puts(msg); luna_printStack(L); luaL_error(L, msg); }
                                                              // 518
    CMAwrap& self=static_cast<CMAwrap &>(*Luna<CMAwrap >::check(L,1)); // 538
    int i=(int)lua_tonumber(L,2);                             // 546
    double eval=(double)lua_tonumber(L,3);                    // 546
    try {                                                     // 280
    self.setVal( i, eval);                                    // 281
    } 
    catch(std::exception& e) { luaL_error( L,e.what()); }
    catch(...) { luaL_error( L,"unknown_error");}
                                                              // 282
    return 0;                                                 // 283
  }                                                           // 362
  static int _bind_resampleSingle(lua_State *L)
  {                                                           // 1350
    if (!_lg_typecheck_resampleSingle(L)) { char msg[]="luna typecheck failed:\n  resampleSingle(CMAwrap& self,int i,)"; puts(msg); luna_printStack(L); luaL_error(L, msg); }
                                                              // 518
    CMAwrap& self=static_cast<CMAwrap &>(*Luna<CMAwrap >::check(L,1)); // 538
    int i=(int)lua_tonumber(L,2);                             // 546
    try {                                                     // 280
    self.resampleSingle( i);                                  // 281
    } 
    catch(std::exception& e) { luaL_error( L,e.what()); }
    catch(...) { luaL_error( L,"unknown_error");}
                                                              // 282
    return 0;                                                 // 283
  }                                                           // 362
  static int _bind_update(lua_State *L)
  {                                                           // 1350
    if (!_lg_typecheck_update(L)) { char msg[]="luna typecheck failed:\n  update(CMAwrap& self,)"; puts(msg); luna_printStack(L); luaL_error(L, msg); }
                                                              // 518
    CMAwrap& self=static_cast<CMAwrap &>(*Luna<CMAwrap >::check(L,1)); // 538
    try {                                                     // 280
    self.update();                                            // 281
    } 
    catch(std::exception& e) { luaL_error( L,e.what()); }
    catch(...) { luaL_error( L,"unknown_error");}
                                                              // 282
    return 0;                                                 // 283
  }                                                           // 362
  static int _bind_getMean(lua_State *L)
  {                                                           // 1350
    if (!_lg_typecheck_getMean(L)) { char msg[]="luna typecheck failed:\n  getMean(CMAwrap& self,vectorn & out,)"; puts(msg); luna_printStack(L); luaL_error(L, msg); }
                                                              // 518
    CMAwrap& self=static_cast<CMAwrap &>(*Luna<CMAwrap >::check(L,1)); // 538
    vectorn & out=static_cast<vectorn &>(*Luna<vectorn >::check(L,2)); // 538
    try {                                                     // 280
    self.getMean( out);                                       // 281
    } 
    catch(std::exception& e) { luaL_error( L,e.what()); }
    catch(...) { luaL_error( L,"unknown_error");}
                                                              // 282
    return 0;                                                 // 283
  }                                                           // 362
  static int _bind_getBest(lua_State *L)
  {                                                           // 1350
    if (!_lg_typecheck_getBest(L)) { char msg[]="luna typecheck failed:\n  getBest(CMAwrap& self,vectorn & out,)"; puts(msg); luna_printStack(L); luaL_error(L, msg); }
                                                              // 518
    CMAwrap& self=static_cast<CMAwrap &>(*Luna<CMAwrap >::check(L,1)); // 538
    vectorn & out=static_cast<vectorn &>(*Luna<vectorn >::check(L,2)); // 538
    try {                                                     // 280
    self.getBest( out);                                       // 281
    } 
    catch(std::exception& e) { luaL_error( L,e.what()); }
    catch(...) { luaL_error( L,"unknown_error");}
                                                              // 282
    return 0;                                                 // 283
  }                                                           // 362
  static int _bind_getBestFvEver(lua_State *L)
  {                                                           // 1350
    if (!_lg_typecheck_getBestFvEver(L)) { char msg[]="luna typecheck failed:\n  getBestFvEver(CMAwrap& self,)"; puts(msg); luna_printStack(L); luaL_error(L, msg); }
                                                              // 518
    CMAwrap& self=static_cast<CMAwrap &>(*Luna<CMAwrap >::check(L,1)); // 538
    try {                                                     // 328
    double ret=self.getBestFvEver();                          // 329
    lua_pushnumber(L, ret);                                   // 330
    } 
    catch(std::exception& e) { luaL_error( L,e.what()); }
    catch(...) { luaL_error( L,"unknown_error");}
                                                              // 331
    return 1;                                                 // 332
  }                                                           // 362
  static int _bind_getBestFvRecent(lua_State *L)
  {                                                           // 1350
    if (!_lg_typecheck_getBestFvRecent(L)) { char msg[]="luna typecheck failed:\n  getBestFvRecent(CMAwrap& self,)"; puts(msg); luna_printStack(L); luaL_error(L, msg); }
                                                              // 518
    CMAwrap& self=static_cast<CMAwrap &>(*Luna<CMAwrap >::check(L,1)); // 538
    try {                                                     // 328
    double ret=self.getBestFvRecent();                        // 329
    lua_pushnumber(L, ret);                                   // 330
    } 
    catch(std::exception& e) { luaL_error( L,e.what()); }
    catch(...) { luaL_error( L,"unknown_error");}
                                                              // 331
    return 1;                                                 // 332
  }                                                           // 362
  static int _bind_getBestSolEver(lua_State *L)
  {                                                           // 1350
    if (!_lg_typecheck_getBestSolEver(L)) { char msg[]="luna typecheck failed:\n  getBestSolEver(CMAwrap& self,vectorn & out,)"; puts(msg); luna_printStack(L); luaL_error(L, msg); }
                                                              // 518
    CMAwrap& self=static_cast<CMAwrap &>(*Luna<CMAwrap >::check(L,1)); // 538
    vectorn & out=static_cast<vectorn &>(*Luna<vectorn >::check(L,2)); // 538
    try {                                                     // 280
    self.getBestSolEver( out);                                // 281
    } 
    catch(std::exception& e) { luaL_error( L,e.what()); }
    catch(...) { luaL_error( L,"unknown_error");}
                                                              // 282
    return 0;                                                 // 283
  }                                                           // 362
  static int _bind_getBestSolRecent(lua_State *L)
  {                                                           // 1350
    if (!_lg_typecheck_getBestSolRecent(L)) { char msg[]="luna typecheck failed:\n  getBestSolRecent(CMAwrap& self,vectorn & out,)"; puts(msg); luna_printStack(L); luaL_error(L, msg); }
                                                              // 518
    CMAwrap& self=static_cast<CMAwrap &>(*Luna<CMAwrap >::check(L,1)); // 538
    vectorn & out=static_cast<vectorn &>(*Luna<vectorn >::check(L,2)); // 538
    try {                                                     // 280
    self.getBestSolRecent( out);                              // 281
    } 
    catch(std::exception& e) { luaL_error( L,e.what()); }
    catch(...) { luaL_error( L,"unknown_error");}
                                                              // 282
    return 0;                                                 // 283
  }                                                           // 362
}; // end of class impl_LunaTraits<CMAwrap >                  // 1511
  CMAwrap* LunaTraits<CMAwrap >::_bind_ctor(lua_State *L)
  {                                                           // 1517
    return impl_LunaTraits<CMAwrap >::_bind_ctor(L);          // 1518
  }                                                           // 1519
  void LunaTraits<CMAwrap >::_bind_dtor(CMAwrap* obj){        // 1521
    delete obj;                                               // 1522
  }                                                           // 1523
const char LunaTraits<CMAwrap >::className[] = "math_CMAwrap"; // 1541
const int LunaTraits<CMAwrap >::uniqueID = 64707780;          // 1542
luna_RegType LunaTraits<CMAwrap >::methods[] = {              // 1548
    {"testForTermination", &impl_LunaTraits<CMAwrap >::_bind_testForTermination}, // 1553
    {"samplePopulation", &impl_LunaTraits<CMAwrap >::_bind_samplePopulation}, // 1553
    {"numPopulation", &impl_LunaTraits<CMAwrap >::_bind_numPopulation}, // 1553
    {"dim", &impl_LunaTraits<CMAwrap >::_bind_dim},           // 1553
    {"getPopulation", &impl_LunaTraits<CMAwrap >::_bind_getPopulation}, // 1553
    {"setVal", &impl_LunaTraits<CMAwrap >::_bind_setVal},     // 1553
    {"resampleSingle", &impl_LunaTraits<CMAwrap >::_bind_resampleSingle}, // 1553
    {"update", &impl_LunaTraits<CMAwrap >::_bind_update},     // 1553
    {"getMean", &impl_LunaTraits<CMAwrap >::_bind_getMean},   // 1553
    {"getBest", &impl_LunaTraits<CMAwrap >::_bind_getBest},   // 1553
    {"getBestFvEver", &impl_LunaTraits<CMAwrap >::_bind_getBestFvEver}, // 1553
    {"getBestFvRecent", &impl_LunaTraits<CMAwrap >::_bind_getBestFvRecent}, // 1553
    {"getBestSolEver", &impl_LunaTraits<CMAwrap >::_bind_getBestSolEver}, // 1553
    {"getBestSolRecent", &impl_LunaTraits<CMAwrap >::_bind_getBestSolRecent}, // 1553
    {0,0}                                                     // 1556
};                                                            // 1557
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
#if defined (USE_MPI)                                         // 1276
 class impl_luna__interface__MPI {
public:                                                       // 1291
    typedef LunaModule<luna__interface__MPI> luna_t;          // 1293
// : number denotes the line number of luna_gen.lua that generated the sentence // 1298
  inline static bool _lg_typecheck_rank(lua_State *L)
  {                                                           // 1317
    if( lua_gettop(L)!=0) return false;                       // 564
    return true;
  }                                                           // 592
  inline static bool _lg_typecheck_size(lua_State *L)
  {                                                           // 1317
    if( lua_gettop(L)!=0) return false;                       // 564
    return true;
  }                                                           // 592
  inline static bool _lg_typecheck_send(lua_State *L)
  {                                                           // 1317
    if( lua_gettop(L)!=2) return false;                       // 564
    if( lua_isstring(L,1)==0) return false;                   // 579
    if( lua_isnumber(L,2)==0) return false;                   // 574
    return true;
  }                                                           // 592
  inline static bool _lg_typecheck_receive(lua_State *L)
  {                                                           // 1317
    if( lua_gettop(L)!=1) return false;                       // 564
    if( lua_isnumber(L,1)==0) return false;                   // 574
    return true;
  }                                                           // 592
  inline static bool _lg_typecheck_source(lua_State *L)
  {                                                           // 1317
    if( lua_gettop(L)!=0) return false;                       // 564
    return true;
  }                                                           // 592
  inline static bool _lg_typecheck_test(lua_State *L)
  {                                                           // 1317
    if( lua_gettop(L)!=0) return false;                       // 564
    return true;
  }                                                           // 592
  inline static bool _lg_typecheck_test2(lua_State *L)
  {                                                           // 1317
    if( lua_gettop(L)!=0) return false;                       // 564
    return true;
  }                                                           // 592
#ifdef USE_MPI


  static int size()
  {
    int size, rc;
    rc = MPI_Comm_size(MPI_COMM_WORLD, &size);
    return size;
  }

  static int rank()
  {
    int rank, rc;
    rc = MPI_Comm_rank(MPI_COMM_WORLD, &rank);
    return rank;
  }

  static void send(const char* msg, int i)
  {
    int rc, tag=100;
    int len=strlen(msg);
    rc = MPI_Send(&len, 1, MPI_INT, i, tag, MPI_COMM_WORLD);
    rc = MPI_Send((void*)msg, len+1, MPI_CHAR, i, tag, MPI_COMM_WORLD);
  }

  static std::string receive(int i)
  {
    int rc, tag=100;
    int len;
    rc = MPI_Recv(&len, 1, MPI_INT, i, tag, MPI_COMM_WORLD, &status);
    TString temp;
    temp.reserve(len+1);
    rc = MPI_Recv((void*)&temp[0], len+1, MPI_CHAR, status.MPI_SOURCE, tag, MPI_COMM_WORLD, &status);
    temp.updateLength();
    return std::string(temp.ptr());
  }
 
  static int source()
  {
      return status.MPI_SOURCE;
  }
  static void test()
  {
    vectorn a(1);
    a.set(0,1);

    for(int i=0;i<1000; i++)
    {
        for(int j=0; j<1000; j++)
        {
            a(0)=a(0)+1;
        }
    }
  }
  static void test2()
  {
    int a=1;
    for(int i=0;i<1000; i++)
    {
        for(int j=0; j<1000; j++)
        {
            a=a+1;
        }
    }
  }

  #endif
                                                              // 1343
  static int _bind_rank(lua_State *L)
  {                                                           // 1350
    if (!_lg_typecheck_rank(L)) { char msg[]="luna typecheck failed:\n  rank()"; puts(msg); luna_printStack(L); luaL_error(L, msg); }
                                                              // 518
    try {                                                     // 328
    int ret=rank();                                           // 329
    lua_pushnumber(L, ret);                                   // 330
    } 
    catch(std::exception& e) { luaL_error( L,e.what()); }
    catch(...) { luaL_error( L,"unknown_error");}
                                                              // 331
    return 1;                                                 // 332
  }                                                           // 362
  static int _bind_size(lua_State *L)
  {                                                           // 1350
    if (!_lg_typecheck_size(L)) { char msg[]="luna typecheck failed:\n  size()"; puts(msg); luna_printStack(L); luaL_error(L, msg); }
                                                              // 518
    try {                                                     // 328
    int ret=size();                                           // 329
    lua_pushnumber(L, ret);                                   // 330
    } 
    catch(std::exception& e) { luaL_error( L,e.what()); }
    catch(...) { luaL_error( L,"unknown_error");}
                                                              // 331
    return 1;                                                 // 332
  }                                                           // 362
  static int _bind_send(lua_State *L)
  {                                                           // 1350
    if (!_lg_typecheck_send(L)) { char msg[]="luna typecheck failed:\n  send(const char * msg,int i,)"; puts(msg); luna_printStack(L); luaL_error(L, msg); }
                                                              // 518
    const char * msg=(const char *)lua_tostring(L,1);         // 541
    int i=(int)lua_tonumber(L,2);                             // 546
    try {                                                     // 280
    send( msg, i);                                            // 281
    } 
    catch(std::exception& e) { luaL_error( L,e.what()); }
    catch(...) { luaL_error( L,"unknown_error");}
                                                              // 282
    return 0;                                                 // 283
  }                                                           // 362
  static int _bind_receive(lua_State *L)
  {                                                           // 1350
    if (!_lg_typecheck_receive(L)) { char msg[]="luna typecheck failed:\n  receive(int i,)"; puts(msg); luna_printStack(L); luaL_error(L, msg); }
                                                              // 518
    int i=(int)lua_tonumber(L,1);                             // 546
    try {                                                     // 334
    std ::string ret=receive( i);                             // 335
    lua_pushstring(L, ret.c_str());                           // 338
    } 
    catch(std::exception& e) { luaL_error( L,e.what()); }
    catch(...) { luaL_error( L,"unknown_error");}
                                                              // 342
    return 1;                                                 // 343
  }                                                           // 362
  static int _bind_source(lua_State *L)
  {                                                           // 1350
    if (!_lg_typecheck_source(L)) { char msg[]="luna typecheck failed:\n  source()"; puts(msg); luna_printStack(L); luaL_error(L, msg); }
                                                              // 518
    try {                                                     // 328
    int ret=source();                                         // 329
    lua_pushnumber(L, ret);                                   // 330
    } 
    catch(std::exception& e) { luaL_error( L,e.what()); }
    catch(...) { luaL_error( L,"unknown_error");}
                                                              // 331
    return 1;                                                 // 332
  }                                                           // 362
  static int _bind_test(lua_State *L)
  {                                                           // 1350
    if (!_lg_typecheck_test(L)) { char msg[]="luna typecheck failed:\n  test()"; puts(msg); luna_printStack(L); luaL_error(L, msg); }
                                                              // 518
    try {                                                     // 280
    test();                                                   // 281
    } 
    catch(std::exception& e) { luaL_error( L,e.what()); }
    catch(...) { luaL_error( L,"unknown_error");}
                                                              // 282
    return 0;                                                 // 283
  }                                                           // 362
  static int _bind_test2(lua_State *L)
  {                                                           // 1350
    if (!_lg_typecheck_test2(L)) { char msg[]="luna typecheck failed:\n  test2()"; puts(msg); luna_printStack(L); luaL_error(L, msg); }
                                                              // 518
    try {                                                     // 280
    test2();                                                  // 281
    } 
    catch(std::exception& e) { luaL_error( L,e.what()); }
    catch(...) { luaL_error( L,"unknown_error");}
                                                              // 282
    return 0;                                                 // 283
  }                                                           // 362
}; // end of class impl_luna__interface__MPI                  // 1511
const char luna__interface__MPI::moduleName[] = "_MPI";       // 1539
luna_RegType luna__interface__MPI::methods[] = {              // 1548
    {"rank", &impl_luna__interface__MPI::_bind_rank},         // 1553
    {"size", &impl_luna__interface__MPI::_bind_size},         // 1553
    {"send", &impl_luna__interface__MPI::_bind_send},         // 1553
    {"receive", &impl_luna__interface__MPI::_bind_receive},   // 1553
    {"source", &impl_luna__interface__MPI::_bind_source},     // 1553
    {"test", &impl_luna__interface__MPI::_bind_test},         // 1553
    {"test2", &impl_luna__interface__MPI::_bind_test2},       // 1553
    {0,0}                                                     // 1556
};                                                            // 1557
#endif //defined (USE_MPI)                                    // 1560
void Register_QP(lua_State* L) {                              // 1564
    luna_dostring(L,"if __luna==nil then __luna={} end");     // 1565
    luna_dostring(L,"    if __luna.copyMethodsFrom==nil then\n        function __luna.copyMethodsFrom(methodsChild, methodsParent)\n            for k,v in pairs(methodsParent) do\n                if k~='__index' and k~='__newindex' and methodsChild[k]==nil then\n                    methodsChild[k]=v\n                end\n            end\n        end\n        function __luna.overwriteMethodsFrom(methodsChild, methodsParent)\n            for k,v in pairs(methodsParent) do\n                if k~='__index' and k~='__newindex' then\n                    if verbose then print('registering', k, methodsChild[k]) end\n                    methodsChild[k]=v\n                end\n            end\n        end\n    end\n    "); // 1566
    Luna<CMAwrap >::Register(L);                              // 1607
    luna_dostring(L, "if not math then math={} end math.CMAwrap=__luna.math_CMAwrap"); // 1614
    luna_dostring(L,"                __luna.math_CMAwrap.luna_class='h.CMAwrap'"); // 1615
   LunaModule<luna__interface__Eigen >::Register(L);          // 1605
    luna_dostring(L," \n                if Eigen==nil then \n                    Eigen={}\n                end \n                __luna.overwriteMethodsFrom(Eigen, __luna._Eigen)\n                "); // 1620
#if defined (USE_MPI)                                         // 1595
   LunaModule<luna__interface__MPI >::Register(L);            // 1605
    luna_dostring(L," \n                if MPI==nil then \n                    MPI={}\n                end \n                __luna.overwriteMethodsFrom(MPI, __luna._MPI)\n                "); // 1620
{
  std::stringstream stringStreams;// defining enums           // 1642
  stringStreams <<"MPI.ANY_SOURCE="<< MPI_ANY_SOURCE;         // 1646
  luna_dostring(L, stringStreams.str().c_str());
}                                                             // 1649
{
  std::stringstream stringStreams;// defining enums           // 1642
  stringStreams <<"MPI.ANY_TAG="<< MPI_ANY_TAG;               // 1646
  luna_dostring(L, stringStreams.str().c_str());
}                                                             // 1649
#endif //defined (USE_MPI)                                    // 1654
}                                                             // 1661