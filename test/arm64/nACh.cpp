/* Created by Language version: 7.7.0 */
/* NOT VECTORIZED */
#define NRN_VECTORIZED 0
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "mech_api.h"
#undef PI
#define nil 0
#define _pval pval
// clang-format off
#include "md1redef.h"
#include "section_fwd.hpp"
#include "nrniv_mf.h"
#include "md2redef.h"
#include "nrnconf.h"
// clang-format on
#include "neuron/cache/mechanism_range.hpp"
#include <vector>
using std::size_t;
static auto& std_cerr_stream = std::cerr;
static constexpr auto number_of_datum_variables = 3;
static constexpr auto number_of_floating_point_variables = 19;
namespace {
template <typename T>
using _nrn_mechanism_std_vector = std::vector<T>;
using _nrn_model_sorted_token = neuron::model_sorted_token;
using _nrn_mechanism_cache_range = neuron::cache::MechanismRange<number_of_floating_point_variables, number_of_datum_variables>;
using _nrn_mechanism_cache_instance = neuron::cache::MechanismInstance<number_of_floating_point_variables, number_of_datum_variables>;
using _nrn_non_owning_id_without_container = neuron::container::non_owning_identifier_without_container;
template <typename T>
using _nrn_mechanism_field = neuron::mechanism::field<T>;
template <typename... Args>
void _nrn_mechanism_register_data_fields(Args&&... args) {
  neuron::mechanism::register_data_fields(std::forward<Args>(args)...);
}
}
 
#if !NRNGPU
#undef exp
#define exp hoc_Exp
#if NRN_ENABLE_ARCH_INDEP_EXP_POW
#undef pow
#define pow hoc_pow
#endif
#endif
 
#define nrn_init _nrn_init__nACh
#define _nrn_initial _nrn_initial__nACh
#define nrn_cur _nrn_cur__nACh
#define _nrn_current _nrn_current__nACh
#define nrn_jacob _nrn_jacob__nACh
#define nrn_state _nrn_state__nACh
#define _net_receive _net_receive__nACh 
#define state state__nACh 
 
#define _threadargscomma_ /**/
#define _threadargsprotocomma_ /**/
#define _internalthreadargsprotocomma_ /**/
#define _threadargs_ /**/
#define _threadargsproto_ /**/
#define _internalthreadargsproto_ /**/
 	/*SUPPRESS 761*/
	/*SUPPRESS 762*/
	/*SUPPRESS 763*/
	/*SUPPRESS 765*/
	 extern double *hoc_getarg(int);
 
#define t nrn_threads->_t
#define dt nrn_threads->_dt
#define tau1 _ml->template fpfield<0>(_iml)
#define tau1_columnindex 0
#define tau2 _ml->template fpfield<1>(_iml)
#define tau2_columnindex 1
#define e _ml->template fpfield<2>(_iml)
#define e_columnindex 2
#define tau _ml->template fpfield<3>(_iml)
#define tau_columnindex 3
#define tauR _ml->template fpfield<4>(_iml)
#define tauR_columnindex 4
#define tauF _ml->template fpfield<5>(_iml)
#define tauF_columnindex 5
#define U _ml->template fpfield<6>(_iml)
#define U_columnindex 6
#define u0 _ml->template fpfield<7>(_iml)
#define u0_columnindex 7
#define failRate _ml->template fpfield<8>(_iml)
#define failRate_columnindex 8
#define i _ml->template fpfield<9>(_iml)
#define i_columnindex 9
#define A _ml->template fpfield<10>(_iml)
#define A_columnindex 10
#define B _ml->template fpfield<11>(_iml)
#define B_columnindex 11
#define g _ml->template fpfield<12>(_iml)
#define g_columnindex 12
#define factor _ml->template fpfield<13>(_iml)
#define factor_columnindex 13
#define x _ml->template fpfield<14>(_iml)
#define x_columnindex 14
#define DA _ml->template fpfield<15>(_iml)
#define DA_columnindex 15
#define DB _ml->template fpfield<16>(_iml)
#define DB_columnindex 16
#define _g _ml->template fpfield<17>(_iml)
#define _g_columnindex 17
#define _tsav _ml->template fpfield<18>(_iml)
#define _tsav_columnindex 18
#define _nd_area *_ml->dptr_field<0>(_iml)
 
 //RANDOM variables 
 #define release_probability	(nrnran123_State*)_ppvar[2].get<void*>()
 #define _p_release_probability _ppvar[2].literal_value<void*>()
 
 static _nrn_mechanism_cache_instance _ml_real{nullptr};
static _nrn_mechanism_cache_range *_ml{&_ml_real};
static size_t _iml{0};
static Datum *_ppvar;
 static int hoc_nrnpointerindex =  -1;
 /* external NEURON variables */
 /* declaration of user functions */
 static double _hoc_urand(void*);
 static int _mechtype;
extern void _nrn_cacheloop_reg(int, int);
extern void hoc_register_limits(int, HocParmLimits*);
extern void hoc_register_units(int, HocParmUnits*);
extern void nrn_promote(Prop*, int, int);
 
#define NMODL_TEXT 1
#if NMODL_TEXT
static void register_nmodl_text_and_filename(int mechtype);
#endif
 extern Prop* nrn_point_prop_;
 static int _pointtype;
 static void* _hoc_create_pnt(Object* _ho) { void* create_point_process(int, Object*);
 return create_point_process(_pointtype, _ho);
}
 static void _hoc_destroy_pnt(void*);
 static double _hoc_loc_pnt(void* _vptr) {double loc_point_process(int, void*);
 return loc_point_process(_pointtype, _vptr);
}
 static double _hoc_has_loc(void* _vptr) {double has_loc_point(void*);
 return has_loc_point(_vptr);
}
 static double _hoc_get_loc_pnt(void* _vptr) {
 double get_loc_point_process(void*); return (get_loc_point_process(_vptr));
}
 static void _hoc_setdata(void*);
 /* connect user functions to hoc names */
 static VoidFunc hoc_intfunc[] = {
 {0, 0}
};
 static Member_func _member_func[] = {
 {"loc", _hoc_loc_pnt},
 {"has_loc", _hoc_has_loc},
 {"get_loc", _hoc_get_loc_pnt},
 {"urand", _hoc_urand},
 {0, 0}
};
#define urand urand_nACh
 extern double urand( );
 /* declare global and static user variables */
 #define gind 0
 #define _gth 0
 /* some parameters have upper and lower limits */
 static HocParmLimits _hoc_parm_limits[] = {
 {"U", 0, 1},
 {"u0", 0, 1},
 {0, 0, 0}
};
 static HocParmUnits _hoc_parm_units[] = {
 {"tau1", "ms"},
 {"tau2", "ms"},
 {"e", "mV"},
 {"tau", "ms"},
 {"tauR", "ms"},
 {"tauF", "ms"},
 {"U", "1"},
 {"u0", "1"},
 {"A", "uS"},
 {"B", "uS"},
 {"i", "nA"},
 {0, 0}
};
 static double A0 = 0;
 static double B0 = 0;
 static double delta_t = 0.01;
 static double v = 0;
 /* connect global user variables to hoc */
 static DoubScal hoc_scdoub[] = {
 {0, 0}
};
 static DoubVec hoc_vdoub[] = {
 {0, 0, 0}
};
 static double _sav_indep;
 extern void _nrn_setdata_reg(int, void(*)(Prop*));
 static void _setdata(Prop* _prop) {
 neuron::legacy::set_globals_from_prop(_prop, _ml_real, _ml, _iml);
_ppvar = _nrn_mechanism_access_dparam(_prop);
 Node * _node = _nrn_mechanism_access_node(_prop);
v = _nrn_mechanism_access_voltage(_node);
 }
 static void _hoc_setdata(void* _vptr) { Prop* _prop;
 _prop = ((Point_process*)_vptr)->_prop;
   _setdata(_prop);
 }
 static void nrn_alloc(Prop*);
static void nrn_init(_nrn_model_sorted_token const&, NrnThread*, Memb_list*, int);
static void nrn_state(_nrn_model_sorted_token const&, NrnThread*, Memb_list*, int);
 static void nrn_cur(_nrn_model_sorted_token const&, NrnThread*, Memb_list*, int);
static void nrn_jacob(_nrn_model_sorted_token const&, NrnThread*, Memb_list*, int);
 static void _hoc_destroy_pnt(void* _vptr) {
   destroy_point_process(_vptr);
}
 
static int _ode_count(int);
static void _ode_map(Prop*, int, neuron::container::data_handle<double>*, neuron::container::data_handle<double>*, double*, int);
static void _ode_spec(_nrn_model_sorted_token const&, NrnThread*, Memb_list*, int);
static void _ode_matsol(_nrn_model_sorted_token const&, NrnThread*, Memb_list*, int);
 
#define _cvode_ieq _ppvar[3].literal_value<int>()
 static void _ode_matsol_instance1(_internalthreadargsproto_);
 /* connect range variables in _p that hoc is supposed to know about */
 static const char *_mechanism[] = {
 "7.7.0",
"nACh",
 "tau1",
 "tau2",
 "e",
 "tau",
 "tauR",
 "tauF",
 "U",
 "u0",
 "failRate",
 0,
 "i",
 0,
 "A",
 "B",
 0,
 0};
 
 /* Used by NrnProperty */
 static _nrn_mechanism_std_vector<double> _parm_default{
     0.15, /* tau1 */
     40, /* tau2 */
     0, /* e */
     100, /* tau */
     800, /* tauR */
     0, /* tauF */
     0.95, /* U */
     0, /* u0 */
     0, /* failRate */
 }; 
 
static void _mech_inst_destruct(Prop* _prop);
 
 
extern Prop* need_memb(Symbol*);
static void nrn_alloc(Prop* _prop) {
  Prop *prop_ion{};
  Datum *_ppvar{};
  if (nrn_point_prop_) {
    _nrn_mechanism_access_alloc_seq(_prop) = _nrn_mechanism_access_alloc_seq(nrn_point_prop_);
    _ppvar = _nrn_mechanism_access_dparam(nrn_point_prop_);
  } else {
   _ppvar = nrn_prop_datum_alloc(_mechtype, 4, _prop);
    _nrn_mechanism_access_dparam(_prop) = _ppvar;
     _nrn_mechanism_cache_instance _ml_real{_prop};
    auto* const _ml = &_ml_real;
    size_t const _iml{};
    assert(_nrn_mechanism_get_num_vars(_prop) == 19);
 	/*initialize range parameters*/
 	tau1 = _parm_default[0]; /* 0.15 */
 	tau2 = _parm_default[1]; /* 40 */
 	e = _parm_default[2]; /* 0 */
 	tau = _parm_default[3]; /* 100 */
 	tauR = _parm_default[4]; /* 800 */
 	tauF = _parm_default[5]; /* 0 */
 	U = _parm_default[6]; /* 0.95 */
 	u0 = _parm_default[7]; /* 0 */
 	failRate = _parm_default[8]; /* 0 */
  }
 	 assert(_nrn_mechanism_get_num_vars(_prop) == 19);
 	_nrn_mechanism_access_dparam(_prop) = _ppvar;
 	/*connect ionic variables to this model*/
 _p_release_probability = (void*)nrnran123_newstream();
 nrn_mech_inst_destruct[_mechtype] = _mech_inst_destruct;
 
}
 static void _initlists();
  /* some states have an absolute tolerance */
 static Symbol** _atollist;
 static HocStateTolerance _hoc_state_tol[] = {
 {0, 0}
};
 static void _net_receive(Point_process*, double*, double);
 static void _net_init(Point_process*, double*, double);
 extern Symbol* hoc_lookup(const char*);
extern void _nrn_thread_reg(int, int, void(*)(Datum*));
void _nrn_thread_table_reg(int, nrn_thread_table_check_t);
extern void hoc_register_tolerance(int, HocStateTolerance*, Symbol***);
extern void _cvode_abstol( Symbol**, double*, int);

 extern "C" void _nACh_reg() {
	int _vectorized = 0;
  _initlists();
 	_pointtype = point_register_mech(_mechanism,
	 nrn_alloc,nrn_cur, nrn_jacob, nrn_state, nrn_init,
	 hoc_nrnpointerindex, 0,
	 _hoc_create_pnt, _hoc_destroy_pnt, _member_func);
 _mechtype = nrn_get_mechtype(_mechanism[1]);
 hoc_register_parm_default(_mechtype, &_parm_default);
     _nrn_setdata_reg(_mechtype, _setdata);
 #if NMODL_TEXT
  register_nmodl_text_and_filename(_mechtype);
#endif
   _nrn_mechanism_register_data_fields(_mechtype,
                                       _nrn_mechanism_field<double>{"tau1"} /* 0 */,
                                       _nrn_mechanism_field<double>{"tau2"} /* 1 */,
                                       _nrn_mechanism_field<double>{"e"} /* 2 */,
                                       _nrn_mechanism_field<double>{"tau"} /* 3 */,
                                       _nrn_mechanism_field<double>{"tauR"} /* 4 */,
                                       _nrn_mechanism_field<double>{"tauF"} /* 5 */,
                                       _nrn_mechanism_field<double>{"U"} /* 6 */,
                                       _nrn_mechanism_field<double>{"u0"} /* 7 */,
                                       _nrn_mechanism_field<double>{"failRate"} /* 8 */,
                                       _nrn_mechanism_field<double>{"i"} /* 9 */,
                                       _nrn_mechanism_field<double>{"A"} /* 10 */,
                                       _nrn_mechanism_field<double>{"B"} /* 11 */,
                                       _nrn_mechanism_field<double>{"g"} /* 12 */,
                                       _nrn_mechanism_field<double>{"factor"} /* 13 */,
                                       _nrn_mechanism_field<double>{"x"} /* 14 */,
                                       _nrn_mechanism_field<double>{"DA"} /* 15 */,
                                       _nrn_mechanism_field<double>{"DB"} /* 16 */,
                                       _nrn_mechanism_field<double>{"_g"} /* 17 */,
                                       _nrn_mechanism_field<double>{"_tsav"} /* 18 */,
                                       _nrn_mechanism_field<double*>{"_nd_area", "area"} /* 0 */,
                                       _nrn_mechanism_field<Point_process*>{"_pntproc", "pntproc"} /* 1 */,
                                       _nrn_mechanism_field<void*>{"release_probability", "random"} /* 2 */,
                                       _nrn_mechanism_field<int>{"_cvode_ieq", "cvodeieq"} /* 3 */);
  hoc_register_prop_size(_mechtype, 19, 4);
  hoc_register_dparam_semantics(_mechtype, 0, "area");
  hoc_register_dparam_semantics(_mechtype, 1, "pntproc");
  hoc_register_dparam_semantics(_mechtype, 2, "random");
  hoc_register_dparam_semantics(_mechtype, 3, "cvodeieq");
 	hoc_register_cvode(_mechtype, _ode_count, _ode_map, _ode_spec, _ode_matsol);
 	hoc_register_tolerance(_mechtype, _hoc_state_tol, &_atollist);
 pnt_receive[_mechtype] = _net_receive;
 pnt_receive_init[_mechtype] = _net_init;
 pnt_receive_size[_mechtype] = 5;
 
    hoc_register_var(hoc_scdoub, hoc_vdoub, hoc_intfunc);
 	ivoc_help("help ?1 nACh /Users/peirui/BasalGangliaData/data/neurons/mechanisms/nACh.mod\n");
 hoc_register_limits(_mechtype, _hoc_parm_limits);
 hoc_register_units(_mechtype, _hoc_parm_units);
 }
static int _reset;
static const char *modelname = "";

static int error;
static int _ninits = 0;
static int _match_recurse=1;
static void _modl_cleanup(){ _match_recurse=1;}
 
static int _ode_spec1(_internalthreadargsproto_);
/*static int _ode_matsol1(_internalthreadargsproto_);*/
 static neuron::container::field_index _slist1[2], _dlist1[2];
 static int state(_internalthreadargsproto_);
 
/*CVODE*/
 static int _ode_spec1 () {_reset=0;
 {
   DA = - A / tau1 ;
   DB = - B / tau2 ;
   }
 return _reset;
}
 static int _ode_matsol1 () {
 DA = DA  / (1. - dt*( ( - 1.0 ) / tau1 )) ;
 DB = DB  / (1. - dt*( ( - 1.0 ) / tau2 )) ;
  return 0;
}
 /*END CVODE*/
 static int state () {_reset=0;
 {
    A = A + (1. - exp(dt*(( - 1.0 ) / tau1)))*(- ( 0.0 ) / ( ( - 1.0 ) / tau1 ) - A) ;
    B = B + (1. - exp(dt*(( - 1.0 ) / tau2)))*(- ( 0.0 ) / ( ( - 1.0 ) / tau2 ) - B) ;
   }
  return 0;
}
 
static void _net_receive (Point_process* _pnt, double* _args, double _lflag) 
{   neuron::legacy::set_globals_from_prop(_pnt->_prop, _ml_real, _ml, _iml);
    _ppvar = _nrn_mechanism_access_dparam(_pnt->_prop);
  if (_tsav > t){ hoc_execerror(hoc_object_name(_pnt->ob), ":Event arrived out of order. Must call ParallelContext.set_maxstep AFTER assigning minimum NetCon.delay");}
 _tsav = t; {
   double _lresult ;
 if ( _args[0] <= 0.0 ) {
     
/*VERBATIM*/
        return;
 }
   if ( urand ( _threadargs_ ) > failRate ) {
     _args[2] = _args[2] * exp ( - ( t - _args[4] ) / tauR ) ;
     _args[2] = _args[2] + ( _args[1] * ( exp ( - ( t - _args[4] ) / tau ) - exp ( - ( t - _args[4] ) / tauR ) ) / ( tau / tauR - 1.0 ) ) ;
     _args[1] = _args[1] * exp ( - ( t - _args[4] ) / tau ) ;
     x = 1.0 - _args[1] - _args[2] ;
     if ( tauF > 0.0 ) {
       _args[3] = _args[3] * exp ( - ( t - _args[4] ) / tauF ) ;
       _args[3] = _args[3] + U * ( 1.0 - _args[3] ) ;
       }
     else {
       _args[3] = U ;
       }
       if (nrn_netrec_state_adjust && !cvode_active_){
    /* discon state adjustment for cnexp case (rate uses no local variable) */
    double __state = A;
    double __primary = (A + _args[0] * factor * x * _args[3] / U) - __state;
     __primary += ( 1. - exp( 0.5*dt*( ( - 1.0 ) / tau1 ) ) )*( - ( 0.0 ) / ( ( - 1.0 ) / tau1 ) - __primary );
    A += __primary;
  } else {
 A = A + _args[0] * factor * x * _args[3] / U ;
       }
   if (nrn_netrec_state_adjust && !cvode_active_){
    /* discon state adjustment for cnexp case (rate uses no local variable) */
    double __state = B;
    double __primary = (B + _args[0] * factor * x * _args[3] / U) - __state;
     __primary += ( 1. - exp( 0.5*dt*( ( - 1.0 ) / tau2 ) ) )*( - ( 0.0 ) / ( ( - 1.0 ) / tau2 ) - __primary );
    B += __primary;
  } else {
 B = B + _args[0] * factor * x * _args[3] / U ;
       }
 _args[1] = _args[1] + x * _args[3] ;
     _args[4] = t ;
     }
   } }
 
static void _net_init(Point_process* _pnt, double* _args, double _lflag) {
       _ppvar = _nrn_mechanism_access_dparam(_pnt->_prop);
 _args[1] = 0.0 ;
   _args[2] = 0.0 ;
   _args[3] = u0 ;
   _args[4] = t ;
   }
 
double urand (  ) {
   double _lurand;
 _lurand = nrnran123_uniform ( release_probability ) ;
   
return _lurand;
 }
 
static double _hoc_urand(void* _vptr) {
 double _r;
    auto* const _pnt = static_cast<Point_process*>(_vptr);
  auto* const _p = _pnt->_prop;
  if (!_p) {
    hoc_execerror("POINT_PROCESS data instance not valid", NULL);
  }
   _setdata(_p);
 _r =  urand (  );
 return(_r);
}
 
static int _ode_count(int _type){ return 2;}
 
static void _ode_spec(_nrn_model_sorted_token const& _sorted_token, NrnThread* _nt, Memb_list* _ml_arg, int _type) {
      Node* _nd{};
  double _v{};
  int _cntml;
  _nrn_mechanism_cache_range _lmr{_sorted_token, *_nt, *_ml_arg, _type};
  _ml = &_lmr;
  _cntml = _ml_arg->_nodecount;
  Datum *_thread{_ml_arg->_thread};
  double* _globals = nullptr;
  if (gind != 0 && _thread != nullptr) { _globals = _thread[_gth].get<double*>(); }
  for (_iml = 0; _iml < _cntml; ++_iml) {
    _ppvar = _ml_arg->_pdata[_iml];
    _nd = _ml_arg->_nodelist[_iml];
    v = NODEV(_nd);
     _ode_spec1 ();
 }}
 
static void _ode_map(Prop* _prop, int _ieq, neuron::container::data_handle<double>* _pv, neuron::container::data_handle<double>* _pvdot, double* _atol, int _type) { 
  _ppvar = _nrn_mechanism_access_dparam(_prop);
  _cvode_ieq = _ieq;
  for (int _i=0; _i < 2; ++_i) {
    _pv[_i] = _nrn_mechanism_get_param_handle(_prop, _slist1[_i]);
    _pvdot[_i] = _nrn_mechanism_get_param_handle(_prop, _dlist1[_i]);
    _cvode_abstol(_atollist, _atol, _i);
  }
 }
 
static void _ode_matsol_instance1(_internalthreadargsproto_) {
 _ode_matsol1 ();
 }
 
static void _ode_matsol(_nrn_model_sorted_token const& _sorted_token, NrnThread* _nt, Memb_list* _ml_arg, int _type) {
      Node* _nd{};
  double _v{};
  int _cntml;
  _nrn_mechanism_cache_range _lmr{_sorted_token, *_nt, *_ml_arg, _type};
  _ml = &_lmr;
  _cntml = _ml_arg->_nodecount;
  Datum *_thread{_ml_arg->_thread};
  double* _globals = nullptr;
  if (gind != 0 && _thread != nullptr) { _globals = _thread[_gth].get<double*>(); }
  for (_iml = 0; _iml < _cntml; ++_iml) {
    _ppvar = _ml_arg->_pdata[_iml];
    _nd = _ml_arg->_nodelist[_iml];
    v = NODEV(_nd);
 _ode_matsol_instance1(_threadargs_);
 }}
 
static void _mech_inst_destruct(Prop* _prop) {
 Datum* _ppvar = _nrn_mechanism_access_dparam(_prop);
 nrnran123_deletestream(release_probability);
 }

static void initmodel() {
  int _i; double _save;_ninits++;
 _save = t;
 t = 0.0;
{
  A = A0;
  B = B0;
 {
   double _ltp ;
 A = 0.0 ;
   B = 0.0 ;
   _ltp = ( tau1 * tau2 ) / ( tau2 - tau1 ) * log ( tau2 / tau1 ) ;
   factor = - exp ( - _ltp / tau1 ) + exp ( - _ltp / tau2 ) ;
   factor = 1.0 / factor ;
   }
  _sav_indep = t; t = _save;

}
}

static void nrn_init(_nrn_model_sorted_token const& _sorted_token, NrnThread* _nt, Memb_list* _ml_arg, int _type){
Node *_nd; double _v; int* _ni; int _cntml;
_nrn_mechanism_cache_range _lmr{_sorted_token, *_nt, *_ml_arg, _type};
auto* const _vec_v = _nt->node_voltage_storage();
_ml = &_lmr;
_ni = _ml_arg->_nodeindices;
_cntml = _ml_arg->_nodecount;
for (_iml = 0; _iml < _cntml; ++_iml) {
 _ppvar = _ml_arg->_pdata[_iml];
 _tsav = -1e20;
   _v = _vec_v[_ni[_iml]];
 v = _v;
 initmodel();
}}

static double _nrn_current(double _v){double _current=0.;v=_v;{ {
   g = B - A ;
   i = g * ( v - e ) ;
   }
 _current += i;

} return _current;
}

static void nrn_cur(_nrn_model_sorted_token const& _sorted_token, NrnThread* _nt, Memb_list* _ml_arg, int _type){
_nrn_mechanism_cache_range _lmr{_sorted_token, *_nt, *_ml_arg, _type};
auto const _vec_rhs = _nt->node_rhs_storage();
auto const _vec_sav_rhs = _nt->node_sav_rhs_storage();
auto const _vec_v = _nt->node_voltage_storage();
Node *_nd; int* _ni; double _rhs, _v; int _cntml;
_ml = &_lmr;
_ni = _ml_arg->_nodeindices;
_cntml = _ml_arg->_nodecount;
for (_iml = 0; _iml < _cntml; ++_iml) {
 _ppvar = _ml_arg->_pdata[_iml];
   _v = _vec_v[_ni[_iml]];
 auto const _g_local = _nrn_current(_v + .001);
 	{ _rhs = _nrn_current(_v);
 	}
 _g = (_g_local - _rhs)/.001;
 _g *=  1.e2/(_nd_area);
 _rhs *= 1.e2/(_nd_area);
	 _vec_rhs[_ni[_iml]] -= _rhs;
 
}}

static void nrn_jacob(_nrn_model_sorted_token const& _sorted_token, NrnThread* _nt, Memb_list* _ml_arg, int _type) {
_nrn_mechanism_cache_range _lmr{_sorted_token, *_nt, *_ml_arg, _type};
auto const _vec_d = _nt->node_d_storage();
auto const _vec_sav_d = _nt->node_sav_d_storage();
auto* const _ml = &_lmr;
Node *_nd; int* _ni; int _iml, _cntml;
_ni = _ml_arg->_nodeindices;
_cntml = _ml_arg->_nodecount;
for (_iml = 0; _iml < _cntml; ++_iml) {
  _vec_d[_ni[_iml]] += _g;
 
}}

static void nrn_state(_nrn_model_sorted_token const& _sorted_token, NrnThread* _nt, Memb_list* _ml_arg, int _type){
Node *_nd; double _v = 0.0; int* _ni; int _cntml;
_nrn_mechanism_cache_range _lmr{_sorted_token, *_nt, *_ml_arg, _type};
auto* const _vec_v = _nt->node_voltage_storage();
_ml = &_lmr;
_ni = _ml_arg->_nodeindices;
_cntml = _ml_arg->_nodecount;
for (_iml = 0; _iml < _cntml; ++_iml) {
 _ppvar = _ml_arg->_pdata[_iml];
 _nd = _ml_arg->_nodelist[_iml];
   _v = _vec_v[_ni[_iml]];
 v=_v;
{
 { error =  state();
 if(error){
  std_cerr_stream << "at line 75 in file nACh.mod:\nBREAKPOINT {\n";
  std_cerr_stream << _ml << ' ' << _iml << '\n';
  abort_run(error);
}
 }}}

}

static void terminal(){}

static void _initlists() {
 int _i; static int _first = 1;
  if (!_first) return;
 _slist1[0] = {A_columnindex, 0};  _dlist1[0] = {DA_columnindex, 0};
 _slist1[1] = {B_columnindex, 0};  _dlist1[1] = {DB_columnindex, 0};
_first = 0;
}

#if NMODL_TEXT
static void register_nmodl_text_and_filename(int mech_type) {
    const char* nmodl_filename = "/Users/peirui/BasalGangliaData/data/neurons/mechanisms/nACh.mod";
    const char* nmodl_file_text = 
  "COMMENT\n"
  "Two state kinetic scheme synapse described by rise time tau1,\n"
  "and decay time constant tau2. The normalized peak conductance is 1.\n"
  "Decay time MUST be greater than rise time.\n"
  "\n"
  "The solution of A->G->bath with rate constants 1/tau1 and 1/tau2 is\n"
  " A = a*exp(-t/tau1) and\n"
  " G = a*tau2/(tau2-tau1)*(-exp(-t/tau1) + exp(-t/tau2))\n"
  "	where tau1 < tau2\n"
  "\n"
  "If tau2-tau1 is very small compared to tau1, this is an alphasynapse with time constant tau2.\n"
  "If tau1/tau2 is very small, this is single exponential decay with time constant tau2.\n"
  "\n"
  "The factor is evaluated in the initial block\n"
  "such that an event of weight 1 generates a\n"
  "peak conductance of 1.\n"
  "\n"
  "Because the solution is a sum of exponentials, the\n"
  "coupled equations can be solved as a pair of independent equations\n"
  "by the more efficient cnexp method.\n"
  "\n"
  "ENDCOMMENT\n"
  "\n"
  "NEURON {\n"
  "    POINT_PROCESS nACh\n"
  "    RANGE tau1, tau2, e, i, q\n"
  "    RANGE tau, tauR, tauF, U, u0\n"
  "    RANGE failRate\n"
  "    NONSPECIFIC_CURRENT i\n"
  "\n"
  "    RANDOM release_probability\n"
  "}\n"
  "\n"
  "UNITS {\n"
  "    (nA) = (nanoamp)\n"
  "    (mV) = (millivolt)\n"
  "    (uS) = (microsiemens)\n"
  "}\n"
  "\n"
  "PARAMETER {\n"
  "    : q = 2,\n"
  "    tau1= 0.15 (ms)\n"
  "    tau2 = 40 (ms)\n"
  "    e = 0 (mV)\n"
  "    tau = 100 (ms)\n"
  "    tauR = 800 (ms)\n"
  "    tauF = 0 (ms)\n"
  "    U = 0.95 (1) <0, 1>\n"
  "    u0 = 0 (1) <0, 1>\n"
  "    failRate = 0\n"
  "}\n"
  "\n"
  "ASSIGNED {\n"
  "    v (mV)\n"
  "    i (nA)\n"
  "    g (uS)\n"
  "    factor\n"
  "    x\n"
  "}\n"
  "\n"
  "STATE {\n"
  "    A (uS)\n"
  "    B (uS)\n"
  "}\n"
  "\n"
  "INITIAL {\n"
  "    LOCAL tp\n"
  "    A = 0\n"
  "    B = 0\n"
  "    tp = (tau1*tau2)/(tau2-tau1) * log(tau2/tau1)\n"
  "    factor = -exp(-tp/tau1) + exp(-tp/tau2)\n"
  "    factor = 1/factor\n"
  "}\n"
  "\n"
  "BREAKPOINT {\n"
  "    SOLVE state METHOD cnexp\n"
  "    g = B - A\n"
  "    i = g*(v - e)\n"
  "}\n"
  "\n"
  "DERIVATIVE state {\n"
  "    A' = -A/tau1\n"
  "    B' = -B/tau2\n"
  "}\n"
  "\n"
  "NET_RECEIVE(weight (uS), y, z, u, tsyn (ms)) {\n"
  "    LOCAL result\n"
  "    INITIAL {\n"
  "        y = 0\n"
  "        z = 0\n"
  "        u = u0\n"
  "        tsyn = t\n"
  "    }\n"
  "    if ( weight <= 0 ) {\n"
  "VERBATIM\n"
  "        return;\n"
  "ENDVERBATIM\n"
  "    }\n"
  "    if( urand() > failRate ){\n"
  "      z = z*exp(-(t-tsyn)/tauR)\n"
  "      z = z + (y*(exp(-(t-tsyn)/tau) - exp(-(t-tsyn)/tauR)) / (tau/tauR - 1) )\n"
  "      y = y*exp(-(t-tsyn)/tau)\n"
  "      x = 1-y-z\n"
  "      if (tauF > 0) {\n"
  "          u = u*exp(-(t-tsyn)/tauF)\n"
  "          u = u + U*(1-u)\n"
  "      } else {\n"
  "          u = U\n"
  "      }\n"
  "    A = A + weight*factor*x*u / U\n"
  "    B = B + weight*factor*x*u / U\n"
  "    y = y + x*u\n"
  "    tsyn = t\n"
  "    }\n"
  "}\n"
  "\n"
  "FUNCTION urand() {\n"
  "    urand = random_uniform(release_probability)\n"
  "}\n"
  "\n"
  "COMMENT\n"
  "(2025-10-08) NEURON 9.0+ compatibility. Replaced scop_random with the\n"
  "new RANDOM keyword.\n"
  "See: https://nrn.readthedocs.io/en/latest/nmodl/language/nmodl_neuron_extension.html#random\n"
  "\n"
  "ENDCOMMENT\n"
  ;
    hoc_reg_nmodl_filename(mech_type, nmodl_filename);
    hoc_reg_nmodl_text(mech_type, nmodl_file_text);
}
#endif
