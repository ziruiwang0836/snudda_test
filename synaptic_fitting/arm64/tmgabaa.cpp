/* Created by Language version: 7.7.0 */
/* VECTORIZED */
#define NRN_VECTORIZED 1
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
static constexpr auto number_of_datum_variables = 5;
static constexpr auto number_of_floating_point_variables = 31;
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
 
#define nrn_init _nrn_init__tmGabaA
#define _nrn_initial _nrn_initial__tmGabaA
#define nrn_cur _nrn_cur__tmGabaA
#define _nrn_current _nrn_current__tmGabaA
#define nrn_jacob _nrn_jacob__tmGabaA
#define nrn_state _nrn_state__tmGabaA
#define _net_receive _net_receive__tmGabaA 
#define state state__tmGabaA 
 
#define _threadargscomma_ _ml, _iml, _ppvar, _thread, _globals, _nt,
#define _threadargsprotocomma_ Memb_list* _ml, size_t _iml, Datum* _ppvar, Datum* _thread, double* _globals, NrnThread* _nt,
#define _internalthreadargsprotocomma_ _nrn_mechanism_cache_range* _ml, size_t _iml, Datum* _ppvar, Datum* _thread, double* _globals, NrnThread* _nt,
#define _threadargs_ _ml, _iml, _ppvar, _thread, _globals, _nt
#define _threadargsproto_ Memb_list* _ml, size_t _iml, Datum* _ppvar, Datum* _thread, double* _globals, NrnThread* _nt
#define _internalthreadargsproto_ _nrn_mechanism_cache_range* _ml, size_t _iml, Datum* _ppvar, Datum* _thread, double* _globals, NrnThread* _nt
 	/*SUPPRESS 761*/
	/*SUPPRESS 762*/
	/*SUPPRESS 763*/
	/*SUPPRESS 765*/
	 extern double *hoc_getarg(int);
 
#define t _nt->_t
#define dt _nt->_dt
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
#define mod_pka_g_min _ml->template fpfield<8>(_iml)
#define mod_pka_g_min_columnindex 8
#define mod_pka_g_max _ml->template fpfield<9>(_iml)
#define mod_pka_g_max_columnindex 9
#define mod_pka_g_half _ml->template fpfield<10>(_iml)
#define mod_pka_g_half_columnindex 10
#define mod_pka_g_slope _ml->template fpfield<11>(_iml)
#define mod_pka_g_slope_columnindex 11
#define mod_pka_fail_min _ml->template fpfield<12>(_iml)
#define mod_pka_fail_min_columnindex 12
#define mod_pka_fail_max _ml->template fpfield<13>(_iml)
#define mod_pka_fail_max_columnindex 13
#define mod_pka_fail_half _ml->template fpfield<14>(_iml)
#define mod_pka_fail_half_columnindex 14
#define mod_pka_fail_slope _ml->template fpfield<15>(_iml)
#define mod_pka_fail_slope_columnindex 15
#define failRate _ml->template fpfield<16>(_iml)
#define failRate_columnindex 16
#define i _ml->template fpfield<17>(_iml)
#define i_columnindex 17
#define modulation_factor _ml->template fpfield<18>(_iml)
#define modulation_factor_columnindex 18
#define modulation_factor_fail _ml->template fpfield<19>(_iml)
#define modulation_factor_fail_columnindex 19
#define A _ml->template fpfield<20>(_iml)
#define A_columnindex 20
#define B _ml->template fpfield<21>(_iml)
#define B_columnindex 21
#define g _ml->template fpfield<22>(_iml)
#define g_columnindex 22
#define factor _ml->template fpfield<23>(_iml)
#define factor_columnindex 23
#define x _ml->template fpfield<24>(_iml)
#define x_columnindex 24
#define PKAci _ml->template fpfield<25>(_iml)
#define PKAci_columnindex 25
#define DA _ml->template fpfield<26>(_iml)
#define DA_columnindex 26
#define DB _ml->template fpfield<27>(_iml)
#define DB_columnindex 27
#define v _ml->template fpfield<28>(_iml)
#define v_columnindex 28
#define _g _ml->template fpfield<29>(_iml)
#define _g_columnindex 29
#define _tsav _ml->template fpfield<30>(_iml)
#define _tsav_columnindex 30
#define _nd_area *_ml->dptr_field<0>(_iml)
#define _ion_PKAci *(_ml->dptr_field<2>(_iml))
#define _p_ion_PKAci static_cast<neuron::container::data_handle<double>>(_ppvar[2])
#define _ion_PKAco *(_ml->dptr_field<3>(_iml))
#define _p_ion_PKAco static_cast<neuron::container::data_handle<double>>(_ppvar[3])
 
 //RANDOM variables 
 #define release_probability	(nrnran123_State*)_ppvar[4].get<void*>()
 #define _p_release_probability _ppvar[4].literal_value<void*>()
 
 /* Thread safe. No static _ml, _iml or _ppvar. */
 static int hoc_nrnpointerindex =  -1;
 static _nrn_mechanism_std_vector<Datum> _extcall_thread;
 /* external NEURON variables */
 /* declaration of user functions */
 static double _hoc_modulation(void*);
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
 {"modulation", _hoc_modulation},
 {"urand", _hoc_urand},
 {0, 0}
};
#define modulation modulation_tmGabaA
#define urand urand_tmGabaA
 extern double modulation( _internalthreadargsprotocomma_ double , double , double , double , double );
 extern double urand( _internalthreadargsproto_ );
 /* declare global and static user variables */
 #define gind 0
 #define _gth 0
#define failRateModulationScaling failRateModulationScaling_tmGabaA
 double failRateModulationScaling = 0;
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
 {"mod_pka_g_min", "1"},
 {"mod_pka_g_max", "1"},
 {"mod_pka_g_half", "mM"},
 {"mod_pka_g_slope", "mM"},
 {"mod_pka_fail_min", "1"},
 {"mod_pka_fail_max", "1"},
 {"mod_pka_fail_half", "mM"},
 {"mod_pka_fail_slope", "mM"},
 {"A", "uS"},
 {"B", "uS"},
 {"i", "nA"},
 {"modulation_factor", "1"},
 {"modulation_factor_fail", "1"},
 {0, 0}
};
 static double A0 = 0;
 static double B0 = 0;
 static double delta_t = 0.01;
 /* connect global user variables to hoc */
 static DoubScal hoc_scdoub[] = {
 {"failRateModulationScaling_tmGabaA", &failRateModulationScaling_tmGabaA},
 {0, 0}
};
 static DoubVec hoc_vdoub[] = {
 {0, 0, 0}
};
 static double _sav_indep;
 extern void _nrn_setdata_reg(int, void(*)(Prop*));
 static void _setdata(Prop* _prop) {
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
 
#define _cvode_ieq _ppvar[5].literal_value<int>()
 static void _ode_matsol_instance1(_internalthreadargsproto_);
 /* connect range variables in _p that hoc is supposed to know about */
 static const char *_mechanism[] = {
 "7.7.0",
"tmGabaA",
 "tau1",
 "tau2",
 "e",
 "tau",
 "tauR",
 "tauF",
 "U",
 "u0",
 "mod_pka_g_min",
 "mod_pka_g_max",
 "mod_pka_g_half",
 "mod_pka_g_slope",
 "mod_pka_fail_min",
 "mod_pka_fail_max",
 "mod_pka_fail_half",
 "mod_pka_fail_slope",
 "failRate",
 0,
 "i",
 "modulation_factor",
 "modulation_factor_fail",
 0,
 "A",
 "B",
 0,
 0};
 static Symbol* _PKAc_sym;
 
 /* Used by NrnProperty */
 static _nrn_mechanism_std_vector<double> _parm_default{
     0.25, /* tau1 */
     3.75, /* tau2 */
     -60, /* e */
     3, /* tau */
     500, /* tauR */
     0, /* tauF */
     0.1, /* U */
     0, /* u0 */
     1, /* mod_pka_g_min */
     1, /* mod_pka_g_max */
     0.0001, /* mod_pka_g_half */
     0.01, /* mod_pka_g_slope */
     0, /* mod_pka_fail_min */
     0, /* mod_pka_fail_max */
     0.0001, /* mod_pka_fail_half */
     0.01, /* mod_pka_fail_slope */
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
   _ppvar = nrn_prop_datum_alloc(_mechtype, 6, _prop);
    _nrn_mechanism_access_dparam(_prop) = _ppvar;
     _nrn_mechanism_cache_instance _ml_real{_prop};
    auto* const _ml = &_ml_real;
    size_t const _iml{};
    assert(_nrn_mechanism_get_num_vars(_prop) == 31);
 	/*initialize range parameters*/
 	tau1 = _parm_default[0]; /* 0.25 */
 	tau2 = _parm_default[1]; /* 3.75 */
 	e = _parm_default[2]; /* -60 */
 	tau = _parm_default[3]; /* 3 */
 	tauR = _parm_default[4]; /* 500 */
 	tauF = _parm_default[5]; /* 0 */
 	U = _parm_default[6]; /* 0.1 */
 	u0 = _parm_default[7]; /* 0 */
 	mod_pka_g_min = _parm_default[8]; /* 1 */
 	mod_pka_g_max = _parm_default[9]; /* 1 */
 	mod_pka_g_half = _parm_default[10]; /* 0.0001 */
 	mod_pka_g_slope = _parm_default[11]; /* 0.01 */
 	mod_pka_fail_min = _parm_default[12]; /* 0 */
 	mod_pka_fail_max = _parm_default[13]; /* 0 */
 	mod_pka_fail_half = _parm_default[14]; /* 0.0001 */
 	mod_pka_fail_slope = _parm_default[15]; /* 0.01 */
 	failRate = _parm_default[16]; /* 0 */
  }
 	 assert(_nrn_mechanism_get_num_vars(_prop) == 31);
 	_nrn_mechanism_access_dparam(_prop) = _ppvar;
 	/*connect ionic variables to this model*/
 prop_ion = need_memb(_PKAc_sym);
 nrn_promote(prop_ion, 1, 0);
 	_ppvar[2] = _nrn_mechanism_get_param_handle(prop_ion, 1); /* PKAci */
 	_ppvar[3] = _nrn_mechanism_get_param_handle(prop_ion, 2); /* PKAco */
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

 extern "C" void _tmgabaa_reg() {
	int _vectorized = 1;
  _initlists();
 	ion_reg("PKAc", 0.0);
 	_PKAc_sym = hoc_lookup("PKAc_ion");
 	_pointtype = point_register_mech(_mechanism,
	 nrn_alloc,nrn_cur, nrn_jacob, nrn_state, nrn_init,
	 hoc_nrnpointerindex, 1,
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
                                       _nrn_mechanism_field<double>{"mod_pka_g_min"} /* 8 */,
                                       _nrn_mechanism_field<double>{"mod_pka_g_max"} /* 9 */,
                                       _nrn_mechanism_field<double>{"mod_pka_g_half"} /* 10 */,
                                       _nrn_mechanism_field<double>{"mod_pka_g_slope"} /* 11 */,
                                       _nrn_mechanism_field<double>{"mod_pka_fail_min"} /* 12 */,
                                       _nrn_mechanism_field<double>{"mod_pka_fail_max"} /* 13 */,
                                       _nrn_mechanism_field<double>{"mod_pka_fail_half"} /* 14 */,
                                       _nrn_mechanism_field<double>{"mod_pka_fail_slope"} /* 15 */,
                                       _nrn_mechanism_field<double>{"failRate"} /* 16 */,
                                       _nrn_mechanism_field<double>{"i"} /* 17 */,
                                       _nrn_mechanism_field<double>{"modulation_factor"} /* 18 */,
                                       _nrn_mechanism_field<double>{"modulation_factor_fail"} /* 19 */,
                                       _nrn_mechanism_field<double>{"A"} /* 20 */,
                                       _nrn_mechanism_field<double>{"B"} /* 21 */,
                                       _nrn_mechanism_field<double>{"g"} /* 22 */,
                                       _nrn_mechanism_field<double>{"factor"} /* 23 */,
                                       _nrn_mechanism_field<double>{"x"} /* 24 */,
                                       _nrn_mechanism_field<double>{"PKAci"} /* 25 */,
                                       _nrn_mechanism_field<double>{"DA"} /* 26 */,
                                       _nrn_mechanism_field<double>{"DB"} /* 27 */,
                                       _nrn_mechanism_field<double>{"v"} /* 28 */,
                                       _nrn_mechanism_field<double>{"_g"} /* 29 */,
                                       _nrn_mechanism_field<double>{"_tsav"} /* 30 */,
                                       _nrn_mechanism_field<double*>{"_nd_area", "area"} /* 0 */,
                                       _nrn_mechanism_field<Point_process*>{"_pntproc", "pntproc"} /* 1 */,
                                       _nrn_mechanism_field<double*>{"_ion_PKAci", "PKAc_ion"} /* 2 */,
                                       _nrn_mechanism_field<double*>{"_ion_PKAco", "PKAc_ion"} /* 3 */,
                                       _nrn_mechanism_field<void*>{"release_probability", "random"} /* 4 */,
                                       _nrn_mechanism_field<int>{"_cvode_ieq", "cvodeieq"} /* 5 */);
  hoc_register_prop_size(_mechtype, 31, 6);
  hoc_register_dparam_semantics(_mechtype, 0, "area");
  hoc_register_dparam_semantics(_mechtype, 1, "pntproc");
  hoc_register_dparam_semantics(_mechtype, 2, "PKAc_ion");
  hoc_register_dparam_semantics(_mechtype, 3, "PKAc_ion");
  hoc_register_dparam_semantics(_mechtype, 4, "random");
  hoc_register_dparam_semantics(_mechtype, 5, "cvodeieq");
 	hoc_register_cvode(_mechtype, _ode_count, _ode_map, _ode_spec, _ode_matsol);
 	hoc_register_tolerance(_mechtype, _hoc_state_tol, &_atollist);
 pnt_receive[_mechtype] = _net_receive;
 pnt_receive_init[_mechtype] = _net_init;
 pnt_receive_size[_mechtype] = 5;
 
    hoc_register_var(hoc_scdoub, hoc_vdoub, hoc_intfunc);
 	ivoc_help("help ?1 tmGabaA /Users/peirui/BasalGangliaData/data/neurons/mechanisms/tmgabaa.mod\n");
 hoc_register_limits(_mechtype, _hoc_parm_limits);
 hoc_register_units(_mechtype, _hoc_parm_units);
 }
static int _reset;
static const char *modelname = "GABA_A synapse with short-term plasticity";

static int error;
static int _ninits = 0;
static int _match_recurse=1;
static void _modl_cleanup(){ _match_recurse=1;}
 
static int _ode_spec1(_internalthreadargsproto_);
/*static int _ode_matsol1(_internalthreadargsproto_);*/
 static neuron::container::field_index _slist1[2], _dlist1[2];
 static int state(_internalthreadargsproto_);
 
/*CVODE*/
 static int _ode_spec1 (_internalthreadargsproto_) {int _reset = 0; {
   DA = - A / tau1 ;
   DB = - B / tau2 ;
   }
 return _reset;
}
 static int _ode_matsol1 (_internalthreadargsproto_) {
 DA = DA  / (1. - dt*( ( - 1.0 ) / tau1 )) ;
 DB = DB  / (1. - dt*( ( - 1.0 ) / tau2 )) ;
  return 0;
}
 /*END CVODE*/
 static int state (_internalthreadargsproto_) { {
    A = A + (1. - exp(dt*(( - 1.0 ) / tau1)))*(- ( 0.0 ) / ( ( - 1.0 ) / tau1 ) - A) ;
    B = B + (1. - exp(dt*(( - 1.0 ) / tau2)))*(- ( 0.0 ) / ( ( - 1.0 ) / tau2 ) - B) ;
   }
  return 0;
}
 
static void _net_receive (Point_process* _pnt, double* _args, double _lflag) 
{  Prop* _p; Datum* _ppvar; Datum* _thread; NrnThread* _nt;
   _nrn_mechanism_cache_instance _ml_real{_pnt->_prop};
  auto* const _ml = &_ml_real;
  size_t const _iml{};
   _thread = nullptr; double* _globals = nullptr; _nt = (NrnThread*)_pnt->_vnt;   _ppvar = _nrn_mechanism_access_dparam(_pnt->_prop);
  if (_tsav > t){ hoc_execerror(hoc_object_name(_pnt->ob), ":Event arrived out of order. Must call ParallelContext.set_maxstep AFTER assigning minimum NetCon.delay");}
 _tsav = t; {
   double _lresult ;
 if ( _args[0] <= 0.0 ) {
     
/*VERBATIM*/
        return;
 }
   if ( urand ( _threadargs_ ) > failRate * ( 1.0 + modulation_factor_fail ) ) {
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
     _nrn_mechanism_cache_instance _ml_real{_pnt->_prop};
  auto* const _ml = &_ml_real;
  size_t const _iml{};
  Datum* _ppvar = _nrn_mechanism_access_dparam(_pnt->_prop);
  Datum* _thread = nullptr;
  double* _globals = nullptr;
  NrnThread* _nt = (NrnThread*)_pnt->_vnt;
 _args[1] = 0.0 ;
   _args[2] = 0.0 ;
   _args[3] = u0 ;
   _args[4] = t ;
   }
 
double urand ( _internalthreadargsproto_ ) {
   double _lurand;
 _lurand = nrnran123_uniform ( release_probability ) ;
   
return _lurand;
 }
 
static double _hoc_urand(void* _vptr) {
 double _r;
 Datum* _ppvar; Datum* _thread; NrnThread* _nt;
   auto* const _pnt = static_cast<Point_process*>(_vptr);
  auto* const _p = _pnt->_prop;
  if (!_p) {
    hoc_execerror("POINT_PROCESS data instance not valid", NULL);
  }
   _nrn_mechanism_cache_instance _ml_real{_p};
  auto* const _ml = &_ml_real;
  size_t const _iml{};
  _ppvar = _nrn_mechanism_access_dparam(_p);
  _thread = _extcall_thread.data();
  double* _globals = nullptr;
  if (gind != 0 && _thread != nullptr) { _globals = _thread[_gth].get<double*>(); }
  _nt = static_cast<NrnThread*>(_pnt->_vnt);
 _r =  urand ( _threadargs_ );
 return(_r);
}
 
double modulation ( _internalthreadargsprotocomma_ double _lconc , double _lmod_min , double _lmod_max , double _lmod_half , double _lmod_slope ) {
   double _lmodulation;
 _lmodulation = _lmod_min + ( _lmod_max - _lmod_min ) / ( 1.0 + exp ( - ( _lconc - _lmod_half ) / _lmod_slope ) ) ;
   
return _lmodulation;
 }
 
static double _hoc_modulation(void* _vptr) {
 double _r;
 Datum* _ppvar; Datum* _thread; NrnThread* _nt;
   auto* const _pnt = static_cast<Point_process*>(_vptr);
  auto* const _p = _pnt->_prop;
  if (!_p) {
    hoc_execerror("POINT_PROCESS data instance not valid", NULL);
  }
   _nrn_mechanism_cache_instance _ml_real{_p};
  auto* const _ml = &_ml_real;
  size_t const _iml{};
  _ppvar = _nrn_mechanism_access_dparam(_p);
  _thread = _extcall_thread.data();
  double* _globals = nullptr;
  if (gind != 0 && _thread != nullptr) { _globals = _thread[_gth].get<double*>(); }
  _nt = static_cast<NrnThread*>(_pnt->_vnt);
 _r =  modulation ( _threadargscomma_ *getarg(1) , *getarg(2) , *getarg(3) , *getarg(4) , *getarg(5) );
 return(_r);
}
 
static int _ode_count(int _type){ return 2;}
 
static void _ode_spec(_nrn_model_sorted_token const& _sorted_token, NrnThread* _nt, Memb_list* _ml_arg, int _type) {
   Datum* _ppvar;
   size_t _iml;   _nrn_mechanism_cache_range* _ml;   Node* _nd{};
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
  PKAci = _ion_PKAci;
     _ode_spec1 (_threadargs_);
 }}
 
static void _ode_map(Prop* _prop, int _ieq, neuron::container::data_handle<double>* _pv, neuron::container::data_handle<double>* _pvdot, double* _atol, int _type) { 
  Datum* _ppvar;
  _ppvar = _nrn_mechanism_access_dparam(_prop);
  _cvode_ieq = _ieq;
  for (int _i=0; _i < 2; ++_i) {
    _pv[_i] = _nrn_mechanism_get_param_handle(_prop, _slist1[_i]);
    _pvdot[_i] = _nrn_mechanism_get_param_handle(_prop, _dlist1[_i]);
    _cvode_abstol(_atollist, _atol, _i);
  }
 }
 
static void _ode_matsol_instance1(_internalthreadargsproto_) {
 _ode_matsol1 (_threadargs_);
 }
 
static void _ode_matsol(_nrn_model_sorted_token const& _sorted_token, NrnThread* _nt, Memb_list* _ml_arg, int _type) {
   Datum* _ppvar;
   size_t _iml;   _nrn_mechanism_cache_range* _ml;   Node* _nd{};
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
  PKAci = _ion_PKAci;
 _ode_matsol_instance1(_threadargs_);
 }}
 
static void _mech_inst_destruct(Prop* _prop) {
 Datum* _ppvar = _nrn_mechanism_access_dparam(_prop);
 nrnran123_deletestream(release_probability);
 }

static void initmodel(_internalthreadargsproto_) {
  int _i; double _save;{
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
 
}
}

static void nrn_init(_nrn_model_sorted_token const& _sorted_token, NrnThread* _nt, Memb_list* _ml_arg, int _type){
_nrn_mechanism_cache_range _lmr{_sorted_token, *_nt, *_ml_arg, _type};
auto* const _vec_v = _nt->node_voltage_storage();
auto* const _ml = &_lmr;
Datum* _ppvar; Datum* _thread;
Node *_nd; double _v; int* _ni; int _iml, _cntml;
_ni = _ml_arg->_nodeindices;
_cntml = _ml_arg->_nodecount;
_thread = _ml_arg->_thread;
double* _globals = nullptr;
if (gind != 0 && _thread != nullptr) { _globals = _thread[_gth].get<double*>(); }
for (_iml = 0; _iml < _cntml; ++_iml) {
 _ppvar = _ml_arg->_pdata[_iml];
 _tsav = -1e20;
   _v = _vec_v[_ni[_iml]];
 v = _v;
  PKAci = _ion_PKAci;
 initmodel(_threadargs_);
}
}

static double _nrn_current(_internalthreadargsprotocomma_ double _v) {
double _current=0.; v=_v;
{ {
   modulation_factor = modulation ( _threadargscomma_ PKAci , mod_pka_g_min , mod_pka_g_max , mod_pka_g_half , mod_pka_g_slope ) ;
   modulation_factor_fail = modulation ( _threadargscomma_ PKAci , mod_pka_fail_min , mod_pka_fail_max , mod_pka_fail_half , mod_pka_fail_slope ) ;
   g = ( B - A ) * modulation_factor ;
   i = g * ( v - e ) ;
   }
 _current += i;

} return _current;
}

static void nrn_cur(_nrn_model_sorted_token const& _sorted_token, NrnThread* _nt, Memb_list* _ml_arg, int _type) {
_nrn_mechanism_cache_range _lmr{_sorted_token, *_nt, *_ml_arg, _type};
auto const _vec_rhs = _nt->node_rhs_storage();
auto const _vec_sav_rhs = _nt->node_sav_rhs_storage();
auto const _vec_v = _nt->node_voltage_storage();
auto* const _ml = &_lmr;
Datum* _ppvar; Datum* _thread;
Node *_nd; int* _ni; double _rhs, _v; int _iml, _cntml;
_ni = _ml_arg->_nodeindices;
_cntml = _ml_arg->_nodecount;
_thread = _ml_arg->_thread;
double* _globals = nullptr;
if (gind != 0 && _thread != nullptr) { _globals = _thread[_gth].get<double*>(); }
for (_iml = 0; _iml < _cntml; ++_iml) {
 _ppvar = _ml_arg->_pdata[_iml];
   _v = _vec_v[_ni[_iml]];
  PKAci = _ion_PKAci;
 auto const _g_local = _nrn_current(_threadargscomma_ _v + .001);
 	{ _rhs = _nrn_current(_threadargscomma_ _v);
 	}
 _g = (_g_local - _rhs)/.001;
 _g *=  1.e2/(_nd_area);
 _rhs *= 1.e2/(_nd_area);
	 _vec_rhs[_ni[_iml]] -= _rhs;
 
}
 
}

static void nrn_jacob(_nrn_model_sorted_token const& _sorted_token, NrnThread* _nt, Memb_list* _ml_arg, int _type) {
_nrn_mechanism_cache_range _lmr{_sorted_token, *_nt, *_ml_arg, _type};
auto const _vec_d = _nt->node_d_storage();
auto const _vec_sav_d = _nt->node_sav_d_storage();
auto* const _ml = &_lmr;
Datum* _ppvar; Datum* _thread;
Node *_nd; int* _ni; int _iml, _cntml;
_ni = _ml_arg->_nodeindices;
_cntml = _ml_arg->_nodecount;
_thread = _ml_arg->_thread;
double* _globals = nullptr;
if (gind != 0 && _thread != nullptr) { _globals = _thread[_gth].get<double*>(); }
for (_iml = 0; _iml < _cntml; ++_iml) {
  _vec_d[_ni[_iml]] += _g;
 
}
 
}

static void nrn_state(_nrn_model_sorted_token const& _sorted_token, NrnThread* _nt, Memb_list* _ml_arg, int _type) {
_nrn_mechanism_cache_range _lmr{_sorted_token, *_nt, *_ml_arg, _type};
auto* const _vec_v = _nt->node_voltage_storage();
auto* const _ml = &_lmr;
Datum* _ppvar; Datum* _thread;
Node *_nd; double _v = 0.0; int* _ni;
_ni = _ml_arg->_nodeindices;
size_t _cntml = _ml_arg->_nodecount;
_thread = _ml_arg->_thread;
double* _globals = nullptr;
if (gind != 0 && _thread != nullptr) { _globals = _thread[_gth].get<double*>(); }
for (size_t _iml = 0; _iml < _cntml; ++_iml) {
 _ppvar = _ml_arg->_pdata[_iml];
 _nd = _ml_arg->_nodelist[_iml];
   _v = _vec_v[_ni[_iml]];
 v=_v;
{
  PKAci = _ion_PKAci;
 {   state(_threadargs_);
  }}}

}

static void terminal(){}

static void _initlists(){
 int _i; static int _first = 1;
  if (!_first) return;
 _slist1[0] = {A_columnindex, 0};  _dlist1[0] = {DA_columnindex, 0};
 _slist1[1] = {B_columnindex, 0};  _dlist1[1] = {DB_columnindex, 0};
_first = 0;
}

#if NMODL_TEXT
static void register_nmodl_text_and_filename(int mech_type) {
    const char* nmodl_filename = "/Users/peirui/BasalGangliaData/data/neurons/mechanisms/tmgabaa.mod";
    const char* nmodl_file_text = 
  "TITLE GABA_A synapse with short-term plasticity\n"
  "\n"
  "\n"
  "NEURON {\n"
  "    THREADSAFE\n"
  "    POINT_PROCESS tmGabaA\n"
  "    RANGE tau1, tau2, e, i, q\n"
  "    RANGE tau, tauR, tauF, U, u0\n"
  "    RANGE failRate\n"
  "    NONSPECIFIC_CURRENT i\n"
  "\n"
  "    USEION PKAc READ PKAci VALENCE 0\n"
  "    RANGE mod_pka_g_min, mod_pka_g_max, mod_pka_g_half, mod_pka_g_slope\n"
  "    RANGE mod_pka_fail_min, mod_pka_fail_max, mod_pka_fail_half, mod_pka_fail_slope\n"
  "    RANGE modulation_factor, modulation_factor_fail\n"
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
  "    : q = 2, now included in tau1,tau2 parameters.\n"
  "    tau1= 0.25 (ms) : ORIG: 0.5ms\n"
  "    tau2 = 3.75 (ms)  : ORIG: 7.5ms, tau2 > tau1\n"
  "    e = -60 (mV)\n"
  "    tau = 3 (ms)\n"
  "    tauR = 500 (ms)  : tauR > tau\n"
  "    tauF = 0 (ms)    : tauF >= 0\n"
  "    U = 0.1 (1) <0, 1>\n"
  "    u0 = 0 (1) <0, 1>\n"
  "    mod_pka_g_min = 1 (1)\n"
  "    mod_pka_g_max = 1 (1)\n"
  "    mod_pka_g_half = 0.000100 (mM)\n"
  "    mod_pka_g_slope = 0.01 (mM)\n"
  "    mod_pka_fail_min = 0 (1)\n"
  "    mod_pka_fail_max = 0 (1)\n"
  "    mod_pka_fail_half = 0.000100 (mM)\n"
  "    mod_pka_fail_slope = 0.01 (mM)\n"
  "    failRate = 0\n"
  "    failRateModulationScaling = 0\n"
  "}\n"
  "\n"
  "ASSIGNED {\n"
  "    v (mV)\n"
  "    i (nA)\n"
  "    g (uS)\n"
  "    factor\n"
  "    x\n"
  "    PKAci (mM)\n"
  "\n"
  "    modulation_factor (1)\n"
  "    modulation_factor_fail (1)\n"
  "\n"
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
  "     SOLVE state METHOD cnexp\n"
  "     modulation_factor=modulation(PKAci, mod_pka_g_min, mod_pka_g_max, mod_pka_g_half, mod_pka_g_slope)\n"
  "     modulation_factor_fail=modulation(PKAci, mod_pka_fail_min, mod_pka_fail_max, mod_pka_fail_half, mod_pka_fail_slope)\n"
  "    g = (B - A)*modulation_factor\n"
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
  "    if( urand() > failRate*(1 + modulation_factor_fail)) {\n"
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
  "FUNCTION modulation(conc (mM), mod_min (1), mod_max (1), mod_half (mM), mod_slope (mM)) (1) {\n"
  "    : returns modulation factor\n"
  "    modulation = mod_min + (mod_max-mod_min) / (1 + exp(-(conc - mod_half)/mod_slope))\n"
  "}\n"
  "\n"
  "COMMENT\n"
  "(2025-10-08) NEURON 9.0+ compatibility. Replaced scop_random with the\n"
  "new RANDOM keyword.\n"
  "See: https://nrn.readthedocs.io/en/latest/nmodl/language/nmodl_neuron_extension.html#random\n"
  "\n"
  "(2019-11-25) Synaptic failure rate (failRate) added. Random factor, no\n"
  "reproducibility guaranteed in parallel sim.\n"
  "\n"
  "(2019-09-12) Set GABA reversal potential to -65mV\n"
  "\n"
  "(2019-08-21) Normalise activation by U, to make sure first activation has\n"
  "amplitude set by g\n"
  "\n"
  "(2019-06-05) Q-factor was calculated in INITAL block, which meant if\n"
  "the synapse was reinitalised then the time constants changed with each\n"
  "initalise. Updated: Johannes Hjorth, hjorth@kth.se\n"
  "\n"
  "(2025-04-02) Set GABA reversal potential to -60mV as per [4]\n"
  "\n"
  "Implementation of GABA_A synapse model with short-term facilitation\n"
  "and depression based on modified tmgsyn.mod [1] by Tsodyks et al [2].\n"
  "Choice of time constants follows [3].  NEURON implementation by Alexander\n"
  "Kozlov <akozlov@kth.se>.\n"
  "\n"
  "[1] tmgsyn.mod, ModelDB (https://senselab.med.yale.edu/ModelDB/),\n"
  "accession number 3815.\n"
  "\n"
  "[2] Tsodyks M, Uziel A, Markram H (2000) Synchrony generation in recurrent\n"
  "networks with frequency-dependent synapses. J Neurosci. 20(1):RC50.\n"
  "\n"
  "[3] Wolf JA, Moyer JT, Lazarewicz MT, Contreras D, Benoit-Marand M,\n"
  "O'Donnell P, Finkel LH (2005) NMDA/AMPA ratio impacts state transitions\n"
  "and entrainment to oscillations in a computational model of the nucleus\n"
  "accumbens medium spiny projection neuron. J Neurosci 25(40):9080-95.\n"
  "\n"
  "[4] Day M, Belal M, Surmeier WC, Melendez A, Wokosin D, Tkatch T,\n"
  "Clarke VRJ, Surmeier DJ (2024) GABAergic regulation of striatal spiny\n"
  "projection neurons depends upon their activity state. PLOS Biology.\n"
  "22(7):e3002752.\n"
  "ENDCOMMENT\n"
  ;
    hoc_reg_nmodl_filename(mech_type, nmodl_filename);
    hoc_reg_nmodl_text(mech_type, nmodl_file_text);
}
#endif
