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
	#include "optimizer_lbfgs.hpp"
	#include "cma/CMAwrap.h"
	
#ifndef genlua_luna_classification_lua809941_def5             // 1179
#define genlua_luna_classification_lua809941_def5             // 1180
// declare all classes before including this file             // 1181
// e.g. class LMat; class LMatView; ....                      // 1182
// The forward declaration is not included here because luna_gen cannot distinguish struct, class, or namespace. // 1183
// : number denotes the line number of luna_gen.lua which generated that line // 1186
class Optimize_lunawrapper;                                   // 1190
class CMAwrap;                                                // 1190
template<>                                                    // 1201
 class LunaTraits<Optimize_lunawrapper > {
public:                                                       // 1203
    static const char className[];                            // 1212
    static const int uniqueID;                                // 1213
    static luna_RegType methods[];                            // 1214
    static Optimize_lunawrapper* _bind_ctor(lua_State *L);    // 1216
    static void _bind_dtor(Optimize_lunawrapper* obj);        // 1217
    typedef Optimize_lunawrapper base_t;                      // 1219
};                                                            // 1225
template<>                                                    // 1201
 class LunaTraits<CMAwrap > {
public:                                                       // 1203
    static const char className[];                            // 1212
    static const int uniqueID;                                // 1213
    static luna_RegType methods[];                            // 1214
    static CMAwrap* _bind_ctor(lua_State *L);                 // 1216
    static void _bind_dtor(CMAwrap* obj);                     // 1217
    typedef CMAwrap base_t;                                   // 1219
};                                                            // 1225
template<>                                                    // 1201
 class LunaTraits<Optimize ::Method > {
public:                                                       // 1203
    static const char className[];                            // 1212
    static const int uniqueID;                                // 1213
    static luna_RegType methods[];                            // 1214
    static Optimize ::Method* _bind_ctor(lua_State *L);       // 1216
    static void _bind_dtor(Optimize ::Method* obj);           // 1217
    typedef Optimize ::Method base_t;                         // 1219
};                                                            // 1225
template<>                                                    // 1201
 class LunaTraits<LBFGS_METHOD > {
public:                                                       // 1203
    static const char className[];                            // 1212
    static const int uniqueID;                                // 1213
    static luna_RegType methods[];                            // 1214
    static LBFGS_METHOD* _bind_ctor(lua_State *L);            // 1216
    static void _bind_dtor(LBFGS_METHOD* obj);                // 1217
    typedef Optimize ::Method base_t;                         // 1219
};                                                            // 1225
#endif                                                        // 1230
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
                                                              // 1254
template<>                                                    // 1258
 class impl_LunaTraits<Optimize_lunawrapper > {
public:                                                       // 1261
    typedef Luna<Optimize_lunawrapper > luna_t;               // 1265
// : number denotes the line number of luna_gen.lua that generated the sentence // 1268
  inline static bool _lg_typecheck_ctor(lua_State *L)
  {                                                           // 1277
    if( lua_gettop(L)!=0) return false;                       // 547
    return true;
  }                                                           // 575
  inline static bool _lg_typecheck_objectiveFunction(lua_State *L)
  {                                                           // 1287
    if( lua_gettop(L)!=2) return false;                       // 547
    if( Luna<void>::get_uniqueid(L,1)!=61759690) return false; // Optimize_lunawrapper // 555
    if( Luna<void>::get_uniqueid(L,2)!=10150210) return false; // vectorn // 555
    return true;
  }                                                           // 575
  inline static bool _lg_typecheck_optimize(lua_State *L)
  {                                                           // 1287
    if( lua_gettop(L)!=2) return false;                       // 547
    if( Luna<void>::get_uniqueid(L,1)!=61759690) return false; // Optimize_lunawrapper // 555
    if( Luna<void>::get_uniqueid(L,2)!=10150210) return false; // vectorn // 555
    return true;
  }                                                           // 575
  inline static bool _lg_typecheck_getResult(lua_State *L)
  {                                                           // 1287
    if( lua_gettop(L)!=1) return false;                       // 547
    if( Luna<void>::get_uniqueid(L,1)!=61759690) return false; // Optimize_lunawrapper // 555
    return true;
  }                                                           // 575
  inline static bool _lg_typecheck_init(lua_State *L)
  {                                                           // 1287
    if( lua_gettop(L)!=6) return false;                       // 547
    if( Luna<void>::get_uniqueid(L,1)!=61759690) return false; // Optimize_lunawrapper // 555
    if( lua_isnumber(L,2)==0) return false;                   // 557
    if( lua_isnumber(L,3)==0) return false;                   // 557
    if( lua_isnumber(L,4)==0) return false;                   // 557
    if( lua_isnumber(L,5)==0) return false;                   // 557
    if( Luna<void>::get_uniqueid(L,6)!=35711537) return false; // Optimize ::Method // 555
    return true;
  }                                                           // 575
  inline static Optimize_lunawrapper* _bind_ctor(lua_State *L)
  {                                                           // 1299
    if (!_lg_typecheck_ctor(L)) { char msg[]="luna typecheck failed:\n  ctor()"; puts(msg); luna_printStack(L); luaL_error(L, msg); }
                                                              // 507
    return new Optimize_lunawrapper();                        // 1304
  }                                                           // 1305
  static int _bind_objectiveFunction(lua_State *L)
  {                                                           // 1320
    if (!_lg_typecheck_objectiveFunction(L)) { char msg[]="luna typecheck failed:\n  objectiveFunction(Optimize_lunawrapper& self,vectorn const & pos,)"; puts(msg); luna_printStack(L); luaL_error(L, msg); }
                                                              // 507
    Optimize_lunawrapper& self=static_cast<Optimize_lunawrapper &>(*Luna<Optimize_lunawrapper >::check(L,1)); // 524
    vectorn const & pos=static_cast<vectorn &>(*Luna<vectorn >::check(L,2)); // 524
    try {                                                     // 319
    double ret=self.objectiveFunction( pos);                  // 320
    lua_pushnumber(L, ret);                                   // 321
    } 
    catch(std::exception& e) { luaL_error( L,e.what()); }
    catch(...) { luaL_error( L,"unknown_error");}
                                                              // 322
    return 1;                                                 // 323
  }                                                           // 353
  static int _bind_optimize(lua_State *L)
  {                                                           // 1320
    if (!_lg_typecheck_optimize(L)) { char msg[]="luna typecheck failed:\n  optimize(Optimize_lunawrapper& self,vectorn const & initialSolution,)"; puts(msg); luna_printStack(L); luaL_error(L, msg); }
                                                              // 507
    Optimize_lunawrapper& self=static_cast<Optimize_lunawrapper &>(*Luna<Optimize_lunawrapper >::check(L,1)); // 524
    vectorn const & initialSolution=static_cast<vectorn &>(*Luna<vectorn >::check(L,2)); // 524
    try {                                                     // 274
    self.optimize( initialSolution);                          // 275
    } 
    catch(std::exception& e) { luaL_error( L,e.what()); }
    catch(...) { luaL_error( L,"unknown_error");}
                                                              // 276
    return 0;                                                 // 277
  }                                                           // 353
  static int _bind_getResult(lua_State *L)
  {                                                           // 1320
    if (!_lg_typecheck_getResult(L)) { char msg[]="luna typecheck failed:\n  getResult(Optimize_lunawrapper& self,)"; puts(msg); luna_printStack(L); luaL_error(L, msg); }
                                                              // 507
    Optimize_lunawrapper& self=static_cast<Optimize_lunawrapper &>(*Luna<Optimize_lunawrapper >::check(L,1)); // 524
    try {                                                     // 300
    vectorn & ret=self.getResult();                           // 301
    Luna<vectorn >::push(L,&ret,false,"_vectorn");            // 302
    } 
    catch(std::exception& e) { luaL_error( L,e.what()); }
    catch(...) { luaL_error( L,"unknown_error");}
                                                              // 303
    return 1;                                                 // 317
  }                                                           // 353
  static int _bind_init(lua_State *L)
  {                                                           // 1320
    if (!_lg_typecheck_init(L)) { char msg[]="luna typecheck failed:\n  init(Optimize_lunawrapper& self,double stepSize,int ndim,double max_step,double grad_step,Optimize ::Method & method,)"; puts(msg); luna_printStack(L); luaL_error(L, msg); }
                                                              // 507
    Optimize_lunawrapper& self=static_cast<Optimize_lunawrapper &>(*Luna<Optimize_lunawrapper >::check(L,1)); // 524
    double stepSize=(double)lua_tonumber(L,2);                // 532
    int ndim=(int)lua_tonumber(L,3);                          // 532
    double max_step=(double)lua_tonumber(L,4);                // 532
    double grad_step=(double)lua_tonumber(L,5);               // 532
    Optimize ::Method & method=static_cast<Optimize ::Method &>(*Luna<Optimize ::Method >::check(L,6)); // 524
    try {                                                     // 274
    self.init( stepSize, ndim, max_step, grad_step, method);  // 275
    } 
    catch(std::exception& e) { luaL_error( L,e.what()); }
    catch(...) { luaL_error( L,"unknown_error");}
                                                              // 276
    return 0;                                                 // 277
  }                                                           // 353
            static int __index(lua_State* L)
            {                                                 // 1377
                    luna_t::userdataType* u=luna_t::checkRaw(L,1);
                    if (u->has_env){
                        lua_getfenv(L,1);
                        lua_pushvalue(L,2);
                        lua_gettable(L,-2);
                        if( !lua_isnil(L,-1))
                        return 1;
                    }
                                                              // 1408
                    int mt=lua_getmetatable(L, 1);
                    if(mt==0) luaL_error(L,"__index");//end
                    lua_pushstring(L, lua_tostring(L,2));
                    lua_rawget(L, -2);
                    return 1;
                                                              // 1419
            }                                                 // 1427
 
            static int __newindex(lua_State* L) {             // 1430
                    luna_t::userdataType* u=luna_t::checkRaw(L,1);
                    if(!u->has_env) {
                        lua_newtable(L);
                        lua_pushvalue(L,-1);
                        lua_setfenv(L,1);
                        u->has_env=1;
                    } else lua_getfenv(L,1);
                    lua_replace(L,1);
                    lua_settable(L,1);
                    return 0;
                                                              // 1458
            }                                                 // 1477
}; // end of class impl_LunaTraits<Optimize_lunawrapper >     // 1481
  Optimize_lunawrapper* LunaTraits<Optimize_lunawrapper >::_bind_ctor(lua_State *L)
  {                                                           // 1487
    return impl_LunaTraits<Optimize_lunawrapper >::_bind_ctor(L); // 1488
  }                                                           // 1489
  void LunaTraits<Optimize_lunawrapper >::_bind_dtor(Optimize_lunawrapper* obj){ // 1491
    delete obj;                                               // 1492
  }                                                           // 1493
const char LunaTraits<Optimize_lunawrapper >::className[] = "_Optimize"; // 1511
const int LunaTraits<Optimize_lunawrapper >::uniqueID = 61759690; // 1512
luna_RegType LunaTraits<Optimize_lunawrapper >::methods[] = { // 1518
    {"objectiveFunction", &impl_LunaTraits<Optimize_lunawrapper >::_bind_objectiveFunction}, // 1523
    {"optimize", &impl_LunaTraits<Optimize_lunawrapper >::_bind_optimize}, // 1523
    {"getResult", &impl_LunaTraits<Optimize_lunawrapper >::_bind_getResult}, // 1523
    {"init", &impl_LunaTraits<Optimize_lunawrapper >::_bind_init}, // 1523
    {"new_modified_T", &Luna<Optimize_lunawrapper >::new_modified_T}, // 1521
    {"__index", &impl_LunaTraits<Optimize_lunawrapper >::__index}, // 1523
    {"__newindex", &impl_LunaTraits<Optimize_lunawrapper >::__newindex}, // 1523
    {0,0}                                                     // 1526
};                                                            // 1527
template<>                                                    // 1258
 class impl_LunaTraits<CMAwrap > {
public:                                                       // 1261
    typedef Luna<CMAwrap > luna_t;                            // 1265
// : number denotes the line number of luna_gen.lua that generated the sentence // 1268
  inline static bool _lg_typecheck_ctor_overload_1(lua_State *L)
  {                                                           // 1277
    if( lua_gettop(L)!=4) return false;                       // 547
    if( Luna<void>::get_uniqueid(L,1)!=10150210) return false; // vectorn // 555
    if( Luna<void>::get_uniqueid(L,2)!=10150210) return false; // vectorn // 555
    if( lua_isnumber(L,3)==0) return false;                   // 557
    if( lua_isnumber(L,4)==0) return false;                   // 557
    return true;
  }                                                           // 575
  inline static bool _lg_typecheck_ctor_overload_2(lua_State *L)
  {                                                           // 1277
    if( lua_gettop(L)!=3) return false;                       // 547
    if( Luna<void>::get_uniqueid(L,1)!=10150210) return false; // vectorn // 555
    if( Luna<void>::get_uniqueid(L,2)!=10150210) return false; // vectorn // 555
    if( lua_isnumber(L,3)==0) return false;                   // 557
    return true;
  }                                                           // 575
  inline static bool _lg_typecheck_testForTermination(lua_State *L)
  {                                                           // 1287
    if( lua_gettop(L)!=1) return false;                       // 547
    if( Luna<void>::get_uniqueid(L,1)!=64707780) return false; // CMAwrap // 555
    return true;
  }                                                           // 575
  inline static bool _lg_typecheck_samplePopulation(lua_State *L)
  {                                                           // 1287
    if( lua_gettop(L)!=1) return false;                       // 547
    if( Luna<void>::get_uniqueid(L,1)!=64707780) return false; // CMAwrap // 555
    return true;
  }                                                           // 575
  inline static bool _lg_typecheck_numPopulation(lua_State *L)
  {                                                           // 1287
    if( lua_gettop(L)!=1) return false;                       // 547
    if( Luna<void>::get_uniqueid(L,1)!=64707780) return false; // CMAwrap // 555
    return true;
  }                                                           // 575
  inline static bool _lg_typecheck_dim(lua_State *L)
  {                                                           // 1287
    if( lua_gettop(L)!=1) return false;                       // 547
    if( Luna<void>::get_uniqueid(L,1)!=64707780) return false; // CMAwrap // 555
    return true;
  }                                                           // 575
  inline static bool _lg_typecheck_getPopulation(lua_State *L)
  {                                                           // 1287
    if( lua_gettop(L)!=2) return false;                       // 547
    if( Luna<void>::get_uniqueid(L,1)!=64707780) return false; // CMAwrap // 555
    if( lua_isnumber(L,2)==0) return false;                   // 557
    return true;
  }                                                           // 575
  inline static bool _lg_typecheck_setVal(lua_State *L)
  {                                                           // 1287
    if( lua_gettop(L)!=3) return false;                       // 547
    if( Luna<void>::get_uniqueid(L,1)!=64707780) return false; // CMAwrap // 555
    if( lua_isnumber(L,2)==0) return false;                   // 557
    if( lua_isnumber(L,3)==0) return false;                   // 557
    return true;
  }                                                           // 575
  inline static bool _lg_typecheck_resampleSingle(lua_State *L)
  {                                                           // 1287
    if( lua_gettop(L)!=2) return false;                       // 547
    if( Luna<void>::get_uniqueid(L,1)!=64707780) return false; // CMAwrap // 555
    if( lua_isnumber(L,2)==0) return false;                   // 557
    return true;
  }                                                           // 575
  inline static bool _lg_typecheck_update(lua_State *L)
  {                                                           // 1287
    if( lua_gettop(L)!=1) return false;                       // 547
    if( Luna<void>::get_uniqueid(L,1)!=64707780) return false; // CMAwrap // 555
    return true;
  }                                                           // 575
  inline static bool _lg_typecheck_getMean(lua_State *L)
  {                                                           // 1287
    if( lua_gettop(L)!=2) return false;                       // 547
    if( Luna<void>::get_uniqueid(L,1)!=64707780) return false; // CMAwrap // 555
    if( Luna<void>::get_uniqueid(L,2)!=10150210) return false; // vectorn // 555
    return true;
  }                                                           // 575
  inline static bool _lg_typecheck_getBest(lua_State *L)
  {                                                           // 1287
    if( lua_gettop(L)!=2) return false;                       // 547
    if( Luna<void>::get_uniqueid(L,1)!=64707780) return false; // CMAwrap // 555
    if( Luna<void>::get_uniqueid(L,2)!=10150210) return false; // vectorn // 555
    return true;
  }                                                           // 575
  inline static CMAwrap* _bind_ctor_overload_1(lua_State *L)
  {                                                           // 1299
    vectorn const & start_p=static_cast<vectorn &>(*Luna<vectorn >::check(L,1)); // 524
    vectorn const & stdev=static_cast<vectorn &>(*Luna<vectorn >::check(L,2)); // 524
    int populationSize=(int)lua_tonumber(L,3);                // 532
    int mu=(int)lua_tonumber(L,4);                            // 532
    return new CMAwrap( start_p, stdev, populationSize, mu);  // 1304
  }                                                           // 1305
  inline static CMAwrap* _bind_ctor_overload_2(lua_State *L)
  {                                                           // 1299
    vectorn const & start_p=static_cast<vectorn &>(*Luna<vectorn >::check(L,1)); // 524
    vectorn const & stdev=static_cast<vectorn &>(*Luna<vectorn >::check(L,2)); // 524
    int populationSize=(int)lua_tonumber(L,3);                // 532
    return new CMAwrap( start_p, stdev, populationSize);      // 1304
  }                                                           // 1305
  static CMAwrap* _bind_ctor(lua_State *L)
  {                                                           // 215
    if (_lg_typecheck_ctor_overload_1(L)) return _bind_ctor_overload_1(L); // 218
    if (_lg_typecheck_ctor_overload_2(L)) return _bind_ctor_overload_2(L); // 218
    luaL_error(L, "ctor ( cannot find overloads:)\n(vectorn const & start_p,vectorn const & stdev,int populationSize,int mu,)\n(vectorn const & start_p,vectorn const & stdev,int populationSize,)\n");
                                                              // 225
    return NULL;                                              // 226
  }                                                           // 227
  static int _bind_testForTermination(lua_State *L)
  {                                                           // 1320
    if (!_lg_typecheck_testForTermination(L)) { char msg[]="luna typecheck failed:\n  testForTermination(CMAwrap& self,)"; puts(msg); luna_printStack(L); luaL_error(L, msg); }
                                                              // 507
    CMAwrap& self=static_cast<CMAwrap &>(*Luna<CMAwrap >::check(L,1)); // 524
    try {                                                     // 325
    std ::string ret=self.testForTermination();               // 326
    lua_pushstring(L, ret.c_str());                           // 329
    } 
    catch(std::exception& e) { luaL_error( L,e.what()); }
    catch(...) { luaL_error( L,"unknown_error");}
                                                              // 333
    return 1;                                                 // 334
  }                                                           // 353
  static int _bind_samplePopulation(lua_State *L)
  {                                                           // 1320
    if (!_lg_typecheck_samplePopulation(L)) { char msg[]="luna typecheck failed:\n  samplePopulation(CMAwrap& self,)"; puts(msg); luna_printStack(L); luaL_error(L, msg); }
                                                              // 507
    CMAwrap& self=static_cast<CMAwrap &>(*Luna<CMAwrap >::check(L,1)); // 524
    try {                                                     // 274
    self.samplePopulation();                                  // 275
    } 
    catch(std::exception& e) { luaL_error( L,e.what()); }
    catch(...) { luaL_error( L,"unknown_error");}
                                                              // 276
    return 0;                                                 // 277
  }                                                           // 353
  static int _bind_numPopulation(lua_State *L)
  {                                                           // 1320
    if (!_lg_typecheck_numPopulation(L)) { char msg[]="luna typecheck failed:\n  numPopulation(CMAwrap& self,)"; puts(msg); luna_printStack(L); luaL_error(L, msg); }
                                                              // 507
    CMAwrap& self=static_cast<CMAwrap &>(*Luna<CMAwrap >::check(L,1)); // 524
    try {                                                     // 319
    int ret=self.numPopulation();                             // 320
    lua_pushnumber(L, ret);                                   // 321
    } 
    catch(std::exception& e) { luaL_error( L,e.what()); }
    catch(...) { luaL_error( L,"unknown_error");}
                                                              // 322
    return 1;                                                 // 323
  }                                                           // 353
  static int _bind_dim(lua_State *L)
  {                                                           // 1320
    if (!_lg_typecheck_dim(L)) { char msg[]="luna typecheck failed:\n  dim(CMAwrap& self,)"; puts(msg); luna_printStack(L); luaL_error(L, msg); }
                                                              // 507
    CMAwrap& self=static_cast<CMAwrap &>(*Luna<CMAwrap >::check(L,1)); // 524
    try {                                                     // 319
    int ret=self.dim();                                       // 320
    lua_pushnumber(L, ret);                                   // 321
    } 
    catch(std::exception& e) { luaL_error( L,e.what()); }
    catch(...) { luaL_error( L,"unknown_error");}
                                                              // 322
    return 1;                                                 // 323
  }                                                           // 353
  static int _bind_getPopulation(lua_State *L)
  {                                                           // 1320
    if (!_lg_typecheck_getPopulation(L)) { char msg[]="luna typecheck failed:\n  getPopulation(CMAwrap& self,int i,)"; puts(msg); luna_printStack(L); luaL_error(L, msg); }
                                                              // 507
    CMAwrap& self=static_cast<CMAwrap &>(*Luna<CMAwrap >::check(L,1)); // 524
    int i=(int)lua_tonumber(L,2);                             // 532
    try {                                                     // 307
    vectornView* ret=new vectornView(self.getPopulation( i)); // 312
    Luna<vectorn >::push(L,ret,true,"_vectornView");          // 313
    } 
    catch(std::exception& e) { luaL_error( L,e.what()); }
    catch(...) { luaL_error( L,"unknown_error");}
                                                              // 314
    return 1;                                                 // 317
  }                                                           // 353
  static int _bind_setVal(lua_State *L)
  {                                                           // 1320
    if (!_lg_typecheck_setVal(L)) { char msg[]="luna typecheck failed:\n  setVal(CMAwrap& self,int i,double eval,)"; puts(msg); luna_printStack(L); luaL_error(L, msg); }
                                                              // 507
    CMAwrap& self=static_cast<CMAwrap &>(*Luna<CMAwrap >::check(L,1)); // 524
    int i=(int)lua_tonumber(L,2);                             // 532
    double eval=(double)lua_tonumber(L,3);                    // 532
    try {                                                     // 274
    self.setVal( i, eval);                                    // 275
    } 
    catch(std::exception& e) { luaL_error( L,e.what()); }
    catch(...) { luaL_error( L,"unknown_error");}
                                                              // 276
    return 0;                                                 // 277
  }                                                           // 353
  static int _bind_resampleSingle(lua_State *L)
  {                                                           // 1320
    if (!_lg_typecheck_resampleSingle(L)) { char msg[]="luna typecheck failed:\n  resampleSingle(CMAwrap& self,int i,)"; puts(msg); luna_printStack(L); luaL_error(L, msg); }
                                                              // 507
    CMAwrap& self=static_cast<CMAwrap &>(*Luna<CMAwrap >::check(L,1)); // 524
    int i=(int)lua_tonumber(L,2);                             // 532
    try {                                                     // 274
    self.resampleSingle( i);                                  // 275
    } 
    catch(std::exception& e) { luaL_error( L,e.what()); }
    catch(...) { luaL_error( L,"unknown_error");}
                                                              // 276
    return 0;                                                 // 277
  }                                                           // 353
  static int _bind_update(lua_State *L)
  {                                                           // 1320
    if (!_lg_typecheck_update(L)) { char msg[]="luna typecheck failed:\n  update(CMAwrap& self,)"; puts(msg); luna_printStack(L); luaL_error(L, msg); }
                                                              // 507
    CMAwrap& self=static_cast<CMAwrap &>(*Luna<CMAwrap >::check(L,1)); // 524
    try {                                                     // 274
    self.update();                                            // 275
    } 
    catch(std::exception& e) { luaL_error( L,e.what()); }
    catch(...) { luaL_error( L,"unknown_error");}
                                                              // 276
    return 0;                                                 // 277
  }                                                           // 353
  static int _bind_getMean(lua_State *L)
  {                                                           // 1320
    if (!_lg_typecheck_getMean(L)) { char msg[]="luna typecheck failed:\n  getMean(CMAwrap& self,vectorn & out,)"; puts(msg); luna_printStack(L); luaL_error(L, msg); }
                                                              // 507
    CMAwrap& self=static_cast<CMAwrap &>(*Luna<CMAwrap >::check(L,1)); // 524
    vectorn & out=static_cast<vectorn &>(*Luna<vectorn >::check(L,2)); // 524
    try {                                                     // 274
    self.getMean( out);                                       // 275
    } 
    catch(std::exception& e) { luaL_error( L,e.what()); }
    catch(...) { luaL_error( L,"unknown_error");}
                                                              // 276
    return 0;                                                 // 277
  }                                                           // 353
  static int _bind_getBest(lua_State *L)
  {                                                           // 1320
    if (!_lg_typecheck_getBest(L)) { char msg[]="luna typecheck failed:\n  getBest(CMAwrap& self,vectorn & out,)"; puts(msg); luna_printStack(L); luaL_error(L, msg); }
                                                              // 507
    CMAwrap& self=static_cast<CMAwrap &>(*Luna<CMAwrap >::check(L,1)); // 524
    vectorn & out=static_cast<vectorn &>(*Luna<vectorn >::check(L,2)); // 524
    try {                                                     // 274
    self.getBest( out);                                       // 275
    } 
    catch(std::exception& e) { luaL_error( L,e.what()); }
    catch(...) { luaL_error( L,"unknown_error");}
                                                              // 276
    return 0;                                                 // 277
  }                                                           // 353
}; // end of class impl_LunaTraits<CMAwrap >                  // 1481
  CMAwrap* LunaTraits<CMAwrap >::_bind_ctor(lua_State *L)
  {                                                           // 1487
    return impl_LunaTraits<CMAwrap >::_bind_ctor(L);          // 1488
  }                                                           // 1489
  void LunaTraits<CMAwrap >::_bind_dtor(CMAwrap* obj){        // 1491
    delete obj;                                               // 1492
  }                                                           // 1493
const char LunaTraits<CMAwrap >::className[] = "math_CMAwrap"; // 1511
const int LunaTraits<CMAwrap >::uniqueID = 64707780;          // 1512
luna_RegType LunaTraits<CMAwrap >::methods[] = {              // 1518
    {"testForTermination", &impl_LunaTraits<CMAwrap >::_bind_testForTermination}, // 1523
    {"samplePopulation", &impl_LunaTraits<CMAwrap >::_bind_samplePopulation}, // 1523
    {"numPopulation", &impl_LunaTraits<CMAwrap >::_bind_numPopulation}, // 1523
    {"dim", &impl_LunaTraits<CMAwrap >::_bind_dim},           // 1523
    {"getPopulation", &impl_LunaTraits<CMAwrap >::_bind_getPopulation}, // 1523
    {"setVal", &impl_LunaTraits<CMAwrap >::_bind_setVal},     // 1523
    {"resampleSingle", &impl_LunaTraits<CMAwrap >::_bind_resampleSingle}, // 1523
    {"update", &impl_LunaTraits<CMAwrap >::_bind_update},     // 1523
    {"getMean", &impl_LunaTraits<CMAwrap >::_bind_getMean},   // 1523
    {"getBest", &impl_LunaTraits<CMAwrap >::_bind_getBest},   // 1523
    {0,0}                                                     // 1526
};                                                            // 1527
template<>                                                    // 1258
 class impl_LunaTraits<Optimize ::Method > {
public:                                                       // 1261
    typedef Luna<Optimize ::Method > luna_t;                  // 1265
// : number denotes the line number of luna_gen.lua that generated the sentence // 1268
}; // end of class impl_LunaTraits<Optimize ::Method >        // 1481
  Optimize ::Method* LunaTraits<Optimize ::Method >::_bind_ctor(lua_State *L)
  {                                                           // 1498
   std::cerr<<"undefined contructor of Optimize ::Method called\n"; // 1499
    return NULL;                                              // 1500
  }                                                           // 1501
  void LunaTraits<Optimize ::Method >::_bind_dtor(Optimize ::Method* obj){ // 1502
   delete obj;                                                // 1503
  }                                                           // 1504
const char LunaTraits<Optimize ::Method >::className[] = "Optimize_Method"; // 1511
const int LunaTraits<Optimize ::Method >::uniqueID = 35711537; // 1512
luna_RegType LunaTraits<Optimize ::Method >::methods[] = {    // 1518
    {0,0}                                                     // 1526
};                                                            // 1527
template<>                                                    // 1258
 class impl_LunaTraits<LBFGS_METHOD > {
public:                                                       // 1261
    typedef Luna<LBFGS_METHOD > luna_t;                       // 1265
// : number denotes the line number of luna_gen.lua that generated the sentence // 1268
  inline static bool _lg_typecheck_ctor_overload_1(lua_State *L)
  {                                                           // 1277
    if( lua_gettop(L)!=0) return false;                       // 547
    return true;
  }                                                           // 575
  inline static bool _lg_typecheck_ctor_overload_2(lua_State *L)
  {                                                           // 1277
    if( lua_gettop(L)!=1) return false;                       // 547
    if( lua_isnumber(L,1)==0) return false;                   // 557
    return true;
  }                                                           // 575
  inline static LBFGS_METHOD* _bind_ctor_overload_1(lua_State *L)
  {                                                           // 1299
    return new LBFGS_METHOD();                                // 1304
  }                                                           // 1305
  inline static LBFGS_METHOD* _bind_ctor_overload_2(lua_State *L)
  {                                                           // 1299
    double epsilon=(double)lua_tonumber(L,1);                 // 532
    return new LBFGS_METHOD( epsilon);                        // 1304
  }                                                           // 1305
  static LBFGS_METHOD* _bind_ctor(lua_State *L)
  {                                                           // 215
    if (_lg_typecheck_ctor_overload_1(L)) return _bind_ctor_overload_1(L); // 218
    if (_lg_typecheck_ctor_overload_2(L)) return _bind_ctor_overload_2(L); // 218
    luaL_error(L, "ctor ( cannot find overloads:)\n()\n(double epsilon,)\n");
                                                              // 225
    return NULL;                                              // 226
  }                                                           // 227
}; // end of class impl_LunaTraits<LBFGS_METHOD >             // 1481
  LBFGS_METHOD* LunaTraits<LBFGS_METHOD >::_bind_ctor(lua_State *L)
  {                                                           // 1487
    return impl_LunaTraits<LBFGS_METHOD >::_bind_ctor(L);     // 1488
  }                                                           // 1489
  void LunaTraits<LBFGS_METHOD >::_bind_dtor(LBFGS_METHOD* obj){ // 1491
    delete obj;                                               // 1492
  }                                                           // 1493
const char LunaTraits<LBFGS_METHOD >::className[] = "Optimize_LBFGS_METHOD"; // 1511
const int LunaTraits<LBFGS_METHOD >::uniqueID = 35711537;     // 1512
luna_RegType LunaTraits<LBFGS_METHOD >::methods[] = {         // 1518
    {0,0}                                                     // 1526
};                                                            // 1527
void Register_classification(lua_State* L) {                  // 1534
    luna_dostring(L,"if __luna==nil then __luna={} end");     // 1535
    luna_dostring(L,"    if __luna.copyMethodsFrom==nil then\n        function __luna.copyMethodsFrom(methodsChild, methodsParent)\n            for k,v in pairs(methodsParent) do\n                if k~='__index' and k~='__newindex' and methodsChild[k]==nil then\n                    methodsChild[k]=v\n                end\n            end\n        end\n        function __luna.overwriteMethodsFrom(methodsChild, methodsParent)\n            for k,v in pairs(methodsParent) do\n                if k~='__index' and k~='__newindex' then\n                    if verbose then print('registering', k, methodsChild[k]) end\n                    methodsChild[k]=v\n                end\n            end\n        end\n    end\n    "); // 1536
    Luna<Optimize_lunawrapper >::Register(L);                 // 1577
    luna_dostring(L, "Optimize=__luna._Optimize");            // 1597
    luna_dostring(L,"                __luna._Optimize.luna_class='Optimize'"); // 1598
    Luna<CMAwrap >::Register(L);                              // 1577
    luna_dostring(L, "if not math then math={} end math.CMAwrap=__luna.math_CMAwrap"); // 1584
    luna_dostring(L,"                __luna.math_CMAwrap.luna_class='h.CMAwrap'"); // 1585
    Luna<Optimize ::Method >::Register(L);                    // 1577
    luna_dostring(L, "if not Optimize then Optimize={} end Optimize.Method=__luna.Optimize_Method"); // 1584
    luna_dostring(L,"                __luna.Optimize_Method.luna_class='imize.Method'"); // 1585
    Luna<LBFGS_METHOD >::Register(L);                         // 1577
    luna_dostring(L, "if not Optimize then Optimize={} end Optimize.LBFGS_METHOD=__luna.Optimize_LBFGS_METHOD"); // 1584
    luna_dostring(L,"                __luna.Optimize_LBFGS_METHOD.luna_class='imize.LBFGS_METHOD'"); // 1585
    luna_dostring(L,"            __luna.copyMethodsFrom(__luna.Optimize_LBFGS_METHOD, __luna.Optimize_Method)"); // 1607
}                                                             // 1631