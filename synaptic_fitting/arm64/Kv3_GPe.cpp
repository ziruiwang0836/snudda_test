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
static constexpr auto number_of_datum_variables = 3;
static constexpr auto number_of_floating_point_variables = 17;
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
 
#define nrn_init _nrn_init__Kv3
#define _nrn_initial _nrn_initial__Kv3
#define nrn_cur _nrn_cur__Kv3
#define _nrn_current _nrn_current__Kv3
#define nrn_jacob _nrn_jacob__Kv3
#define nrn_state _nrn_state__Kv3
#define _net_receive _net_receive__Kv3 
#define _f_settables _f_settables__Kv3 
#define settables settables__Kv3 
#define states states__Kv3 
 
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
#define gbar _ml->template fpfield<0>(_iml)
#define gbar_columnindex 0
#define tau_m0 _ml->template fpfield<1>(_iml)
#define tau_m0_columnindex 1
#define tau_h0 _ml->template fpfield<2>(_iml)
#define tau_h0_columnindex 2
#define exponent _ml->template fpfield<3>(_iml)
#define exponent_columnindex 3
#define i _ml->template fpfield<4>(_iml)
#define i_columnindex 4
#define m _ml->template fpfield<5>(_iml)
#define m_columnindex 5
#define h _ml->template fpfield<6>(_iml)
#define h_columnindex 6
#define ek _ml->template fpfield<7>(_iml)
#define ek_columnindex 7
#define Dm _ml->template fpfield<8>(_iml)
#define Dm_columnindex 8
#define Dh _ml->template fpfield<9>(_iml)
#define Dh_columnindex 9
#define ik _ml->template fpfield<10>(_iml)
#define ik_columnindex 10
#define minf _ml->template fpfield<11>(_iml)
#define minf_columnindex 11
#define taum _ml->template fpfield<12>(_iml)
#define taum_columnindex 12
#define hinf _ml->template fpfield<13>(_iml)
#define hinf_columnindex 13
#define tauh _ml->template fpfield<14>(_iml)
#define tauh_columnindex 14
#define v _ml->template fpfield<15>(_iml)
#define v_columnindex 15
#define _g _ml->template fpfield<16>(_iml)
#define _g_columnindex 16
#define _ion_ek *(_ml->dptr_field<0>(_iml))
#define _p_ion_ek static_cast<neuron::container::data_handle<double>>(_ppvar[0])
#define _ion_ik *(_ml->dptr_field<1>(_iml))
#define _p_ion_ik static_cast<neuron::container::data_handle<double>>(_ppvar[1])
#define _ion_dikdv *(_ml->dptr_field<2>(_iml))
 /* Thread safe. No static _ml, _iml or _ppvar. */
 static int hoc_nrnpointerindex =  -1;
 static _nrn_mechanism_std_vector<Datum> _extcall_thread;
 static Prop* _extcall_prop;
 /* _prop_id kind of shadows _extcall_prop to allow validity checking. */
 static _nrn_non_owning_id_without_container _prop_id{};
 /* external NEURON variables */
 /* declaration of user functions */
 static void _hoc_settables(void);
 static int _mechtype;
extern void _nrn_cacheloop_reg(int, int);
extern void hoc_register_limits(int, HocParmLimits*);
extern void hoc_register_units(int, HocParmUnits*);
extern void nrn_promote(Prop*, int, int);
 
#define NMODL_TEXT 1
#if NMODL_TEXT
static void register_nmodl_text_and_filename(int mechtype);
#endif
 static void _hoc_setdata();
 /* connect user functions to hoc names */
 static VoidFunc hoc_intfunc[] = {
 {"setdata_Kv3", _hoc_setdata},
 {"settables_Kv3", _hoc_settables},
 {0, 0}
};
 
/* Direct Python call wrappers to density mechanism functions.*/
 static double _npy_settables(Prop*);
 
static NPyDirectMechFunc npy_direct_func_proc[] = {
 {"settables", _npy_settables},
 {0, 0}
};
 /* declare global and static user variables */
 #define gind 0
 #define _gth 0
#define h0 h0_Kv3
 double h0 = 0.6;
#define k_h k_h_Kv3
 double k_h = -10;
#define k_m k_m_Kv3
 double k_m = 7.8;
#define phi_h phi_h_Kv3
 double phi_h = 0;
#define phi_m phi_m_Kv3
 double phi_m = -26;
#define sigma_h1 sigma_h1_Kv3
 double sigma_h1 = -10;
#define sigma_h0 sigma_h0_Kv3
 double sigma_h0 = 10;
#define sigma_m1 sigma_m1_Kv3
 double sigma_m1 = -12;
#define sigma_m0 sigma_m0_Kv3
 double sigma_m0 = 13;
#define tau_h1 tau_h1_Kv3
 double tau_h1 = 33;
#define theta_h theta_h_Kv3
 double theta_h = -20;
#define tau_m1 tau_m1_Kv3
 double tau_m1 = 14;
#define theta_m theta_m_Kv3
 double theta_m = -26;
#define usetable usetable_Kv3
 double usetable = 1;
 
static void _check_settables(_internalthreadargsproto_); 
static void _check_table_thread(_threadargsprotocomma_ int _type, _nrn_model_sorted_token const& _sorted_token) {
  if (gind != 0 && _thread != nullptr) { _globals = _thread[_gth].get<double*>(); } 
  _nrn_mechanism_cache_range _lmr{_sorted_token, *_nt, *_ml, _type};
  {
    auto* const _ml = &_lmr;
   _check_settables(_threadargs_);
   }
}
 /* some parameters have upper and lower limits */
 static HocParmLimits _hoc_parm_limits[] = {
 {"usetable_Kv3", 0, 1},
 {0, 0, 0}
};
 static HocParmUnits _hoc_parm_units[] = {
 {"theta_m_Kv3", "mV"},
 {"k_m_Kv3", "mV"},
 {"tau_m1_Kv3", "ms"},
 {"phi_m_Kv3", "mV"},
 {"sigma_m0_Kv3", "mV"},
 {"sigma_m1_Kv3", "mV"},
 {"theta_h_Kv3", "mV"},
 {"k_h_Kv3", "mV"},
 {"tau_h1_Kv3", "ms"},
 {"phi_h_Kv3", "mV"},
 {"sigma_h0_Kv3", "mV"},
 {"sigma_h1_Kv3", "mV"},
 {"gbar_Kv3", "mho/cm2"},
 {"tau_m0_Kv3", "ms"},
 {"tau_h0_Kv3", "ms"},
 {0, 0}
};
 static double delta_t = 0.01;
 static double m0 = 0;
 /* connect global user variables to hoc */
 static DoubScal hoc_scdoub[] = {
 {"theta_m_Kv3", &theta_m_Kv3},
 {"k_m_Kv3", &k_m_Kv3},
 {"tau_m1_Kv3", &tau_m1_Kv3},
 {"phi_m_Kv3", &phi_m_Kv3},
 {"sigma_m0_Kv3", &sigma_m0_Kv3},
 {"sigma_m1_Kv3", &sigma_m1_Kv3},
 {"h0_Kv3", &h0_Kv3},
 {"theta_h_Kv3", &theta_h_Kv3},
 {"k_h_Kv3", &k_h_Kv3},
 {"tau_h1_Kv3", &tau_h1_Kv3},
 {"phi_h_Kv3", &phi_h_Kv3},
 {"sigma_h0_Kv3", &sigma_h0_Kv3},
 {"sigma_h1_Kv3", &sigma_h1_Kv3},
 {"usetable_Kv3", &usetable_Kv3},
 {0, 0}
};
 static DoubVec hoc_vdoub[] = {
 {0, 0, 0}
};
 static double _sav_indep;
 extern void _nrn_setdata_reg(int, void(*)(Prop*));
 static void _setdata(Prop* _prop) {
 _extcall_prop = _prop;
 _prop_id = _nrn_get_prop_id(_prop);
 }
 static void _hoc_setdata() {
 Prop *_prop, *hoc_getdata_range(int);
 _prop = hoc_getdata_range(_mechtype);
   _setdata(_prop);
 hoc_retpushx(1.);
}
 static void nrn_alloc(Prop*);
static void nrn_init(_nrn_model_sorted_token const&, NrnThread*, Memb_list*, int);
static void nrn_state(_nrn_model_sorted_token const&, NrnThread*, Memb_list*, int);
 static void nrn_cur(_nrn_model_sorted_token const&, NrnThread*, Memb_list*, int);
static void nrn_jacob(_nrn_model_sorted_token const&, NrnThread*, Memb_list*, int);
 
static int _ode_count(int);
static void _ode_map(Prop*, int, neuron::container::data_handle<double>*, neuron::container::data_handle<double>*, double*, int);
static void _ode_spec(_nrn_model_sorted_token const&, NrnThread*, Memb_list*, int);
static void _ode_matsol(_nrn_model_sorted_token const&, NrnThread*, Memb_list*, int);
 
#define _cvode_ieq _ppvar[3].literal_value<int>()
 static void _ode_matsol_instance1(_internalthreadargsproto_);
 /* connect range variables in _p that hoc is supposed to know about */
 static const char *_mechanism[] = {
 "7.7.0",
"Kv3",
 "gbar_Kv3",
 "tau_m0_Kv3",
 "tau_h0_Kv3",
 "exponent_Kv3",
 0,
 "i_Kv3",
 0,
 "m_Kv3",
 "h_Kv3",
 0,
 0};
 static Symbol* _k_sym;
 
 /* Used by NrnProperty */
 static _nrn_mechanism_std_vector<double> _parm_default{
     0.001, /* gbar */
     0.1, /* tau_m0 */
     7, /* tau_h0 */
     4, /* exponent */
 }; 
 
 
extern Prop* need_memb(Symbol*);
static void nrn_alloc(Prop* _prop) {
  Prop *prop_ion{};
  Datum *_ppvar{};
   _ppvar = nrn_prop_datum_alloc(_mechtype, 4, _prop);
    _nrn_mechanism_access_dparam(_prop) = _ppvar;
     _nrn_mechanism_cache_instance _ml_real{_prop};
    auto* const _ml = &_ml_real;
    size_t const _iml{};
    assert(_nrn_mechanism_get_num_vars(_prop) == 17);
 	/*initialize range parameters*/
 	gbar = _parm_default[0]; /* 0.001 */
 	tau_m0 = _parm_default[1]; /* 0.1 */
 	tau_h0 = _parm_default[2]; /* 7 */
 	exponent = _parm_default[3]; /* 4 */
 	 assert(_nrn_mechanism_get_num_vars(_prop) == 17);
 	_nrn_mechanism_access_dparam(_prop) = _ppvar;
 	/*connect ionic variables to this model*/
 prop_ion = need_memb(_k_sym);
 nrn_promote(prop_ion, 0, 1);
 	_ppvar[0] = _nrn_mechanism_get_param_handle(prop_ion, 0); /* ek */
 	_ppvar[1] = _nrn_mechanism_get_param_handle(prop_ion, 3); /* ik */
 	_ppvar[2] = _nrn_mechanism_get_param_handle(prop_ion, 4); /* _ion_dikdv */
 
}
 static void _initlists();
  /* some states have an absolute tolerance */
 static Symbol** _atollist;
 static HocStateTolerance _hoc_state_tol[] = {
 {0, 0}
};
 extern Symbol* hoc_lookup(const char*);
extern void _nrn_thread_reg(int, int, void(*)(Datum*));
void _nrn_thread_table_reg(int, nrn_thread_table_check_t);
extern void hoc_register_tolerance(int, HocStateTolerance*, Symbol***);
extern void _cvode_abstol( Symbol**, double*, int);

 extern "C" void _Kv3_GPe_reg() {
	int _vectorized = 1;
  _initlists();
 	ion_reg("k", -10000.);
 	_k_sym = hoc_lookup("k_ion");
 	register_mech(_mechanism, nrn_alloc,nrn_cur, nrn_jacob, nrn_state, nrn_init, hoc_nrnpointerindex, 1);
 _mechtype = nrn_get_mechtype(_mechanism[1]);
 hoc_register_parm_default(_mechtype, &_parm_default);
         hoc_register_npy_direct(_mechtype, npy_direct_func_proc);
     _nrn_setdata_reg(_mechtype, _setdata);
     _nrn_thread_table_reg(_mechtype, _check_table_thread);
 #if NMODL_TEXT
  register_nmodl_text_and_filename(_mechtype);
#endif
   _nrn_mechanism_register_data_fields(_mechtype,
                                       _nrn_mechanism_field<double>{"gbar"} /* 0 */,
                                       _nrn_mechanism_field<double>{"tau_m0"} /* 1 */,
                                       _nrn_mechanism_field<double>{"tau_h0"} /* 2 */,
                                       _nrn_mechanism_field<double>{"exponent"} /* 3 */,
                                       _nrn_mechanism_field<double>{"i"} /* 4 */,
                                       _nrn_mechanism_field<double>{"m"} /* 5 */,
                                       _nrn_mechanism_field<double>{"h"} /* 6 */,
                                       _nrn_mechanism_field<double>{"ek"} /* 7 */,
                                       _nrn_mechanism_field<double>{"Dm"} /* 8 */,
                                       _nrn_mechanism_field<double>{"Dh"} /* 9 */,
                                       _nrn_mechanism_field<double>{"ik"} /* 10 */,
                                       _nrn_mechanism_field<double>{"minf"} /* 11 */,
                                       _nrn_mechanism_field<double>{"taum"} /* 12 */,
                                       _nrn_mechanism_field<double>{"hinf"} /* 13 */,
                                       _nrn_mechanism_field<double>{"tauh"} /* 14 */,
                                       _nrn_mechanism_field<double>{"v"} /* 15 */,
                                       _nrn_mechanism_field<double>{"_g"} /* 16 */,
                                       _nrn_mechanism_field<double*>{"_ion_ek", "k_ion"} /* 0 */,
                                       _nrn_mechanism_field<double*>{"_ion_ik", "k_ion"} /* 1 */,
                                       _nrn_mechanism_field<double*>{"_ion_dikdv", "k_ion"} /* 2 */,
                                       _nrn_mechanism_field<int>{"_cvode_ieq", "cvodeieq"} /* 3 */);
  hoc_register_prop_size(_mechtype, 17, 4);
  hoc_register_dparam_semantics(_mechtype, 0, "k_ion");
  hoc_register_dparam_semantics(_mechtype, 1, "k_ion");
  hoc_register_dparam_semantics(_mechtype, 2, "k_ion");
  hoc_register_dparam_semantics(_mechtype, 3, "cvodeieq");
 	hoc_register_cvode(_mechtype, _ode_count, _ode_map, _ode_spec, _ode_matsol);
 	hoc_register_tolerance(_mechtype, _hoc_state_tol, &_atollist);
 
    hoc_register_var(hoc_scdoub, hoc_vdoub, hoc_intfunc);
 	ivoc_help("help ?1 Kv3 /Users/peirui/BasalGangliaData/data/neurons/mechanisms/Kv3_GPe.mod\n");
 hoc_register_limits(_mechtype, _hoc_parm_limits);
 hoc_register_units(_mechtype, _hoc_parm_units);
 }
 static double *_t_minf;
 static double *_t_taum;
 static double *_t_hinf;
 static double *_t_tauh;
static int _reset;
static const char *modelname = "fast activated potassium Kv3 (Kv3.1/3.4) channel for GPe neuron";

static int error;
static int _ninits = 0;
static int _match_recurse=1;
static void _modl_cleanup(){ _match_recurse=1;}
static int _f_settables(_internalthreadargsprotocomma_ double);
static int settables(_internalthreadargsprotocomma_ double);
 
static int _ode_spec1(_internalthreadargsproto_);
/*static int _ode_matsol1(_internalthreadargsproto_);*/
 static void _n_settables(_internalthreadargsprotocomma_ double _lv);
 static neuron::container::field_index _slist1[2], _dlist1[2];
 static int states(_internalthreadargsproto_);
 
/*CVODE*/
 static int _ode_spec1 (_internalthreadargsproto_) {int _reset = 0; {
   settables ( _threadargscomma_ v ) ;
   Dm = ( minf - m ) / taum ;
   Dh = ( hinf - h ) / tauh ;
   }
 return _reset;
}
 static int _ode_matsol1 (_internalthreadargsproto_) {
 settables ( _threadargscomma_ v ) ;
 Dm = Dm  / (1. - dt*( ( ( ( - 1.0 ) ) ) / taum )) ;
 Dh = Dh  / (1. - dt*( ( ( ( - 1.0 ) ) ) / tauh )) ;
  return 0;
}
 /*END CVODE*/
 static int states (_internalthreadargsproto_) { {
   settables ( _threadargscomma_ v ) ;
    m = m + (1. - exp(dt*(( ( ( - 1.0 ) ) ) / taum)))*(- ( ( ( minf ) ) / taum ) / ( ( ( ( - 1.0 ) ) ) / taum ) - m) ;
    h = h + (1. - exp(dt*(( ( ( - 1.0 ) ) ) / tauh)))*(- ( ( ( hinf ) ) / tauh ) / ( ( ( ( - 1.0 ) ) ) / tauh ) - h) ;
   }
  return 0;
}
 static double _mfac_settables, _tmin_settables;
  static void _check_settables(_internalthreadargsproto_) {
  static int _maktable=1; int _i, _j, _ix = 0;
  double _xi, _tmax;
  if (!usetable) {return;}
  if (_maktable) { double _x, _dx; _maktable=0;
   _tmin_settables =  - 100.0 ;
   _tmax =  100.0 ;
   _dx = (_tmax - _tmin_settables)/400.; _mfac_settables = 1./_dx;
   for (_i=0, _x=_tmin_settables; _i < 401; _x += _dx, _i++) {
    _f_settables(_threadargscomma_ _x);
    _t_minf[_i] = minf;
    _t_taum[_i] = taum;
    _t_hinf[_i] = hinf;
    _t_tauh[_i] = tauh;
   }
  }
 }

 static int settables(_internalthreadargsprotocomma_ double _lv) { 
#if 0
_check_settables(_threadargs_);
#endif
 _n_settables(_threadargscomma_ _lv);
 return 0;
 }

 static void _n_settables(_internalthreadargsprotocomma_ double _lv){ int _i, _j;
 double _xi, _theta;
 if (!usetable) {
 _f_settables(_threadargscomma_ _lv); return; 
}
 _xi = _mfac_settables * (_lv - _tmin_settables);
 if (std::isnan(_xi)) {
  minf = _xi;
  taum = _xi;
  hinf = _xi;
  tauh = _xi;
  return;
 }
 if (_xi <= 0.) {
 minf = _t_minf[0];
 taum = _t_taum[0];
 hinf = _t_hinf[0];
 tauh = _t_tauh[0];
 return; }
 if (_xi >= 400.) {
 minf = _t_minf[400];
 taum = _t_taum[400];
 hinf = _t_hinf[400];
 tauh = _t_tauh[400];
 return; }
 _i = (int) _xi;
 _theta = _xi - (double)_i;
 minf = _t_minf[_i] + _theta*(_t_minf[_i+1] - _t_minf[_i]);
 taum = _t_taum[_i] + _theta*(_t_taum[_i+1] - _t_taum[_i]);
 hinf = _t_hinf[_i] + _theta*(_t_hinf[_i+1] - _t_hinf[_i]);
 tauh = _t_tauh[_i] + _theta*(_t_tauh[_i+1] - _t_tauh[_i]);
 }

 
static int  _f_settables ( _internalthreadargsprotocomma_ double _lv ) {
   minf = 1.0 / ( 1.0 + exp ( ( theta_m - _lv ) / k_m ) ) ;
   taum = tau_m0 + ( tau_m1 - tau_m0 ) / ( exp ( ( phi_m - _lv ) / sigma_m0 ) + exp ( ( phi_m - _lv ) / sigma_m1 ) ) ;
   hinf = h0 + ( 1.0 - h0 ) / ( 1.0 + exp ( ( theta_h - _lv ) / k_h ) ) ;
   tauh = tau_h0 + ( tau_h1 - tau_h0 ) / ( exp ( ( phi_h - _lv ) / sigma_h0 ) + exp ( ( phi_h - _lv ) / sigma_h1 ) ) ;
    return 0; }
 
static void _hoc_settables(void) {
  double _r;
 Datum* _ppvar; Datum* _thread; NrnThread* _nt;
 
  if(!_prop_id) {
    hoc_execerror("No data for settables_Kv3. Requires prior call to setdata_Kv3 and that the specified mechanism instance still be in existence.", NULL);
  }
  Prop* _local_prop = _extcall_prop;
  _nrn_mechanism_cache_instance _ml_real{_local_prop};
auto* const _ml = &_ml_real;
size_t const _iml{};
_ppvar = _local_prop ? _nrn_mechanism_access_dparam(_local_prop) : nullptr;
_thread = _extcall_thread.data();
double* _globals = nullptr;
if (gind != 0 && _thread != nullptr) { _globals = _thread[_gth].get<double*>(); }
_nt = nrn_threads;
 
#if 1
 _check_settables(_threadargs_);
#endif
 _r = 1.;
 settables ( _threadargscomma_ *getarg(1) );
 hoc_retpushx(_r);
}
 
static double _npy_settables(Prop* _prop) {
    double _r{0.0};
 Datum* _ppvar; Datum* _thread; NrnThread* _nt;
 _nrn_mechanism_cache_instance _ml_real{_prop};
auto* const _ml = &_ml_real;
size_t const _iml{};
_ppvar = _nrn_mechanism_access_dparam(_prop);
_thread = _extcall_thread.data();
double* _globals = nullptr;
if (gind != 0 && _thread != nullptr) { _globals = _thread[_gth].get<double*>(); }
_nt = nrn_threads;
 
#if 1
 _check_settables(_threadargs_);
#endif
 _r = 1.;
 settables ( _threadargscomma_ *getarg(1) );
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
  ek = _ion_ek;
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
  ek = _ion_ek;
 _ode_matsol_instance1(_threadargs_);
 }}

static void initmodel(_internalthreadargsproto_) {
  int _i; double _save;{
  h = h0;
  m = m0;
 {
   settables ( _threadargscomma_ v ) ;
   m = minf ;
   h = hinf ;
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

#if 0
 _check_settables(_threadargs_);
#endif
   _v = _vec_v[_ni[_iml]];
 v = _v;
  ek = _ion_ek;
 initmodel(_threadargs_);
 }
}

static double _nrn_current(_internalthreadargsprotocomma_ double _v) {
double _current=0.; v=_v;
{ {
   ik = gbar * pow( m , exponent ) * h * ( v - ek ) ;
   i = ik ;
   }
 _current += ik;

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
  ek = _ion_ek;
 auto const _g_local = _nrn_current(_threadargscomma_ _v + .001);
 	{ double _dik;
  _dik = ik;
 _rhs = _nrn_current(_threadargscomma_ _v);
  _ion_dikdv += (_dik - ik)/.001 ;
 	}
 _g = (_g_local - _rhs)/.001;
  _ion_ik += ik ;
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
  ek = _ion_ek;
 {   states(_threadargs_);
  } }}

}

static void terminal(){}

static void _initlists(){
 int _i; static int _first = 1;
  if (!_first) return;
 _slist1[0] = {m_columnindex, 0};  _dlist1[0] = {Dm_columnindex, 0};
 _slist1[1] = {h_columnindex, 0};  _dlist1[1] = {Dh_columnindex, 0};
   _t_minf = makevector(401*sizeof(double));
   _t_taum = makevector(401*sizeof(double));
   _t_hinf = makevector(401*sizeof(double));
   _t_tauh = makevector(401*sizeof(double));
_first = 0;
}

#if NMODL_TEXT
static void register_nmodl_text_and_filename(int mech_type) {
    const char* nmodl_filename = "/Users/peirui/BasalGangliaData/data/neurons/mechanisms/Kv3_GPe.mod";
    const char* nmodl_file_text = 
  "TITLE fast activated potassium Kv3 (Kv3.1/3.4) channel for GPe neuron\n"
  "\n"
  "COMMENT\n"
  " modeled by Gunay et al., 2008\n"
  " implemented in NEURON by Kitano, 2011\n"
  "ENDCOMMENT\n"
  "\n"
  "UNITS {\n"
  " (mV) = (millivolt)\n"
  " (mA) = (milliamp)\n"
  "}\n"
  "\n"
  "NEURON {\n"
  " SUFFIX Kv3\n"
  " USEION k READ ek WRITE ik\n"
  " RANGE tau_h0, tau_m0, exponent\n"
  " RANGE gbar, i\n"
  "}\n"
  "\n"
  "PARAMETER {\n"
  " v (mV)\n"
  " dt (ms)\n"
  " gbar  = 0.001 (mho/cm2)\n"
  " ek (mV)\n"
  "\n"
  " theta_m = -26.0 (mV)\n"
  " k_m = 7.8 (mV)\n"
  " tau_m0 = 0.1 (ms)\n"
  " tau_m1 = 14.0 (ms)\n"
  " phi_m = -26.0 (mV)\n"
  " sigma_m0 = 13.0 (mV)\n"
  " sigma_m1 = -12.0 (mV)\n"
  "\n"
  " h0 = 0.6\n"
  " theta_h = -20.0 (mV)\n"
  " k_h = -10.0 (mV)\n"
  " tau_h0 = 7.0 (ms)\n"
  " tau_h1 = 33.0 (ms)\n"
  " phi_h = 0.0 (mV)\n"
  " sigma_h0 = 10.0 (mV)\n"
  " sigma_h1 = -10.0 (mV)\n"
  " exponent = 4\n"
  "}\n"
  "\n"
  "STATE {\n"
  " m h\n"
  "}\n"
  "\n"
  "ASSIGNED { \n"
  " ik (mA/cm2)\n"
  " minf\n"
  " taum (ms)\n"
  " hinf\n"
  " tauh (ms)\n"
  " i\n"
  "}\n"
  "\n"
  "BREAKPOINT {\n"
  " SOLVE states METHOD cnexp\n"
  " ik  = gbar * m^exponent * h * (v-ek)\n"
  " i = ik\n"
  "}\n"
  "\n"
  "UNITSOFF\n"
  "\n"
  "INITIAL {\n"
  " settables(v)\n"
  " m = minf\n"
  " h = hinf\n"
  "}\n"
  "\n"
  "DERIVATIVE states {  \n"
  " settables(v)\n"
  " m' = (minf - m)/taum\n"
  " h' = (hinf - h)/tauh\n"
  "}\n"
  "\n"
  "PROCEDURE settables(v) {\n"
  "        TABLE minf, taum, hinf, tauh FROM -100 TO 100 WITH 400\n"
  "\n"
  "	minf = 1.0 / (1.0 + exp((theta_m - v)/k_m))\n"
  "	taum = tau_m0 + (tau_m1 - tau_m0)/(exp((phi_m - v)/sigma_m0) + exp((phi_m - v)/sigma_m1))\n"
  "	hinf = h0 + (1.0 - h0) / (1.0 + exp((theta_h - v)/k_h))\n"
  "	tauh = tau_h0 + (tau_h1 - tau_h0)/(exp((phi_h - v)/sigma_h0) + exp((phi_h - v)/sigma_h1))\n"
  "}\n"
  "\n"
  "UNITSON\n"
  ;
    hoc_reg_nmodl_filename(mech_type, nmodl_filename);
    hoc_reg_nmodl_text(mech_type, nmodl_file_text);
}
#endif
