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
static constexpr auto number_of_floating_point_variables = 27;
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
 
#define nrn_init _nrn_init__kaf_ms
#define _nrn_initial _nrn_initial__kaf_ms
#define nrn_cur _nrn_cur__kaf_ms
#define _nrn_current _nrn_current__kaf_ms
#define nrn_jacob _nrn_jacob__kaf_ms
#define nrn_state _nrn_state__kaf_ms
#define _net_receive _net_receive__kaf_ms 
#define rates rates__kaf_ms 
#define states states__kaf_ms 
 
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
#define q _ml->template fpfield<1>(_iml)
#define q_columnindex 1
#define mod_pka_g_min _ml->template fpfield<2>(_iml)
#define mod_pka_g_min_columnindex 2
#define mod_pka_g_max _ml->template fpfield<3>(_iml)
#define mod_pka_g_max_columnindex 3
#define mod_pka_g_half _ml->template fpfield<4>(_iml)
#define mod_pka_g_half_columnindex 4
#define mod_pka_g_hill _ml->template fpfield<5>(_iml)
#define mod_pka_g_hill_columnindex 5
#define mod_pka_shift_min _ml->template fpfield<6>(_iml)
#define mod_pka_shift_min_columnindex 6
#define mod_pka_shift_max _ml->template fpfield<7>(_iml)
#define mod_pka_shift_max_columnindex 7
#define mod_pka_shift_half _ml->template fpfield<8>(_iml)
#define mod_pka_shift_half_columnindex 8
#define mod_pka_shift_hill _ml->template fpfield<9>(_iml)
#define mod_pka_shift_hill_columnindex 9
#define ik _ml->template fpfield<10>(_iml)
#define ik_columnindex 10
#define gk _ml->template fpfield<11>(_iml)
#define gk_columnindex 11
#define modulation_factor_g _ml->template fpfield<12>(_iml)
#define modulation_factor_g_columnindex 12
#define modulation_factor_shift _ml->template fpfield<13>(_iml)
#define modulation_factor_shift_columnindex 13
#define modShift _ml->template fpfield<14>(_iml)
#define modShift_columnindex 14
#define m _ml->template fpfield<15>(_iml)
#define m_columnindex 15
#define h _ml->template fpfield<16>(_iml)
#define h_columnindex 16
#define ek _ml->template fpfield<17>(_iml)
#define ek_columnindex 17
#define minf _ml->template fpfield<18>(_iml)
#define minf_columnindex 18
#define mtau _ml->template fpfield<19>(_iml)
#define mtau_columnindex 19
#define hinf _ml->template fpfield<20>(_iml)
#define hinf_columnindex 20
#define htau _ml->template fpfield<21>(_iml)
#define htau_columnindex 21
#define PKAci _ml->template fpfield<22>(_iml)
#define PKAci_columnindex 22
#define Dm _ml->template fpfield<23>(_iml)
#define Dm_columnindex 23
#define Dh _ml->template fpfield<24>(_iml)
#define Dh_columnindex 24
#define v _ml->template fpfield<25>(_iml)
#define v_columnindex 25
#define _g _ml->template fpfield<26>(_iml)
#define _g_columnindex 26
#define _ion_ek *(_ml->dptr_field<0>(_iml))
#define _p_ion_ek static_cast<neuron::container::data_handle<double>>(_ppvar[0])
#define _ion_ik *(_ml->dptr_field<1>(_iml))
#define _p_ion_ik static_cast<neuron::container::data_handle<double>>(_ppvar[1])
#define _ion_dikdv *(_ml->dptr_field<2>(_iml))
#define _ion_PKAci *(_ml->dptr_field<3>(_iml))
#define _p_ion_PKAci static_cast<neuron::container::data_handle<double>>(_ppvar[3])
#define _ion_PKAco *(_ml->dptr_field<4>(_iml))
#define _p_ion_PKAco static_cast<neuron::container::data_handle<double>>(_ppvar[4])
 /* Thread safe. No static _ml, _iml or _ppvar. */
 static int hoc_nrnpointerindex =  -1;
 static _nrn_mechanism_std_vector<Datum> _extcall_thread;
 static Prop* _extcall_prop;
 /* _prop_id kind of shadows _extcall_prop to allow validity checking. */
 static _nrn_non_owning_id_without_container _prop_id{};
 /* external NEURON variables */
 /* declaration of user functions */
 static void _hoc_hill(void);
 static void _hoc_rates(void);
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
 {"setdata_kaf_ms", _hoc_setdata},
 {"hill_kaf_ms", _hoc_hill},
 {"rates_kaf_ms", _hoc_rates},
 {0, 0}
};
 
/* Direct Python call wrappers to density mechanism functions.*/
 static double _npy_hill(Prop*);
 static double _npy_rates(Prop*);
 
static NPyDirectMechFunc npy_direct_func_proc[] = {
 {"hill", _npy_hill},
 {"rates", _npy_rates},
 {0, 0}
};
#define hill hill_kaf_ms
 extern double hill( _internalthreadargsprotocomma_ double , double , double , double , double );
 /* declare global and static user variables */
 #define gind 0
 #define _gth 0
 /* some parameters have upper and lower limits */
 static HocParmLimits _hoc_parm_limits[] = {
 {0, 0, 0}
};
 static HocParmUnits _hoc_parm_units[] = {
 {"gbar_kaf_ms", "S/cm2"},
 {"mod_pka_g_min_kaf_ms", "1"},
 {"mod_pka_g_max_kaf_ms", "1"},
 {"mod_pka_g_half_kaf_ms", "mM"},
 {"mod_pka_g_hill_kaf_ms", "1"},
 {"mod_pka_shift_min_kaf_ms", "1"},
 {"mod_pka_shift_max_kaf_ms", "1"},
 {"mod_pka_shift_half_kaf_ms", "mM"},
 {"mod_pka_shift_hill_kaf_ms", "1"},
 {"ik_kaf_ms", "mA/cm2"},
 {"gk_kaf_ms", "S/cm2"},
 {"modulation_factor_g_kaf_ms", "1"},
 {"modulation_factor_shift_kaf_ms", "1"},
 {0, 0}
};
 static double delta_t = 0.01;
 static double h0 = 0;
 static double m0 = 0;
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
 
#define _cvode_ieq _ppvar[5].literal_value<int>()
 static void _ode_matsol_instance1(_internalthreadargsproto_);
 /* connect range variables in _p that hoc is supposed to know about */
 static const char *_mechanism[] = {
 "7.7.0",
"kaf_ms",
 "gbar_kaf_ms",
 "q_kaf_ms",
 "mod_pka_g_min_kaf_ms",
 "mod_pka_g_max_kaf_ms",
 "mod_pka_g_half_kaf_ms",
 "mod_pka_g_hill_kaf_ms",
 "mod_pka_shift_min_kaf_ms",
 "mod_pka_shift_max_kaf_ms",
 "mod_pka_shift_half_kaf_ms",
 "mod_pka_shift_hill_kaf_ms",
 0,
 "ik_kaf_ms",
 "gk_kaf_ms",
 "modulation_factor_g_kaf_ms",
 "modulation_factor_shift_kaf_ms",
 "modShift_kaf_ms",
 0,
 "m_kaf_ms",
 "h_kaf_ms",
 0,
 0};
 static Symbol* _k_sym;
 static Symbol* _PKAc_sym;
 
 /* Used by NrnProperty */
 static _nrn_mechanism_std_vector<double> _parm_default{
     0, /* gbar */
     1, /* q */
     1, /* mod_pka_g_min */
     1, /* mod_pka_g_max */
     0.0001, /* mod_pka_g_half */
     4, /* mod_pka_g_hill */
     0, /* mod_pka_shift_min */
     0, /* mod_pka_shift_max */
     0.0001, /* mod_pka_shift_half */
     4, /* mod_pka_shift_hill */
 }; 
 
 
extern Prop* need_memb(Symbol*);
static void nrn_alloc(Prop* _prop) {
  Prop *prop_ion{};
  Datum *_ppvar{};
   _ppvar = nrn_prop_datum_alloc(_mechtype, 6, _prop);
    _nrn_mechanism_access_dparam(_prop) = _ppvar;
     _nrn_mechanism_cache_instance _ml_real{_prop};
    auto* const _ml = &_ml_real;
    size_t const _iml{};
    assert(_nrn_mechanism_get_num_vars(_prop) == 27);
 	/*initialize range parameters*/
 	gbar = _parm_default[0]; /* 0 */
 	q = _parm_default[1]; /* 1 */
 	mod_pka_g_min = _parm_default[2]; /* 1 */
 	mod_pka_g_max = _parm_default[3]; /* 1 */
 	mod_pka_g_half = _parm_default[4]; /* 0.0001 */
 	mod_pka_g_hill = _parm_default[5]; /* 4 */
 	mod_pka_shift_min = _parm_default[6]; /* 0 */
 	mod_pka_shift_max = _parm_default[7]; /* 0 */
 	mod_pka_shift_half = _parm_default[8]; /* 0.0001 */
 	mod_pka_shift_hill = _parm_default[9]; /* 4 */
 	 assert(_nrn_mechanism_get_num_vars(_prop) == 27);
 	_nrn_mechanism_access_dparam(_prop) = _ppvar;
 	/*connect ionic variables to this model*/
 prop_ion = need_memb(_k_sym);
 nrn_promote(prop_ion, 0, 1);
 	_ppvar[0] = _nrn_mechanism_get_param_handle(prop_ion, 0); /* ek */
 	_ppvar[1] = _nrn_mechanism_get_param_handle(prop_ion, 3); /* ik */
 	_ppvar[2] = _nrn_mechanism_get_param_handle(prop_ion, 4); /* _ion_dikdv */
 prop_ion = need_memb(_PKAc_sym);
 nrn_promote(prop_ion, 1, 0);
 	_ppvar[3] = _nrn_mechanism_get_param_handle(prop_ion, 1); /* PKAci */
 	_ppvar[4] = _nrn_mechanism_get_param_handle(prop_ion, 2); /* PKAco */
 
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

 extern "C" void _kaf_ms_reg() {
	int _vectorized = 1;
  _initlists();
 	ion_reg("k", -10000.);
 	ion_reg("PKAc", 0.0);
 	_k_sym = hoc_lookup("k_ion");
 	_PKAc_sym = hoc_lookup("PKAc_ion");
 	register_mech(_mechanism, nrn_alloc,nrn_cur, nrn_jacob, nrn_state, nrn_init, hoc_nrnpointerindex, 1);
 _mechtype = nrn_get_mechtype(_mechanism[1]);
 hoc_register_parm_default(_mechtype, &_parm_default);
         hoc_register_npy_direct(_mechtype, npy_direct_func_proc);
     _nrn_setdata_reg(_mechtype, _setdata);
 #if NMODL_TEXT
  register_nmodl_text_and_filename(_mechtype);
#endif
   _nrn_mechanism_register_data_fields(_mechtype,
                                       _nrn_mechanism_field<double>{"gbar"} /* 0 */,
                                       _nrn_mechanism_field<double>{"q"} /* 1 */,
                                       _nrn_mechanism_field<double>{"mod_pka_g_min"} /* 2 */,
                                       _nrn_mechanism_field<double>{"mod_pka_g_max"} /* 3 */,
                                       _nrn_mechanism_field<double>{"mod_pka_g_half"} /* 4 */,
                                       _nrn_mechanism_field<double>{"mod_pka_g_hill"} /* 5 */,
                                       _nrn_mechanism_field<double>{"mod_pka_shift_min"} /* 6 */,
                                       _nrn_mechanism_field<double>{"mod_pka_shift_max"} /* 7 */,
                                       _nrn_mechanism_field<double>{"mod_pka_shift_half"} /* 8 */,
                                       _nrn_mechanism_field<double>{"mod_pka_shift_hill"} /* 9 */,
                                       _nrn_mechanism_field<double>{"ik"} /* 10 */,
                                       _nrn_mechanism_field<double>{"gk"} /* 11 */,
                                       _nrn_mechanism_field<double>{"modulation_factor_g"} /* 12 */,
                                       _nrn_mechanism_field<double>{"modulation_factor_shift"} /* 13 */,
                                       _nrn_mechanism_field<double>{"modShift"} /* 14 */,
                                       _nrn_mechanism_field<double>{"m"} /* 15 */,
                                       _nrn_mechanism_field<double>{"h"} /* 16 */,
                                       _nrn_mechanism_field<double>{"ek"} /* 17 */,
                                       _nrn_mechanism_field<double>{"minf"} /* 18 */,
                                       _nrn_mechanism_field<double>{"mtau"} /* 19 */,
                                       _nrn_mechanism_field<double>{"hinf"} /* 20 */,
                                       _nrn_mechanism_field<double>{"htau"} /* 21 */,
                                       _nrn_mechanism_field<double>{"PKAci"} /* 22 */,
                                       _nrn_mechanism_field<double>{"Dm"} /* 23 */,
                                       _nrn_mechanism_field<double>{"Dh"} /* 24 */,
                                       _nrn_mechanism_field<double>{"v"} /* 25 */,
                                       _nrn_mechanism_field<double>{"_g"} /* 26 */,
                                       _nrn_mechanism_field<double*>{"_ion_ek", "k_ion"} /* 0 */,
                                       _nrn_mechanism_field<double*>{"_ion_ik", "k_ion"} /* 1 */,
                                       _nrn_mechanism_field<double*>{"_ion_dikdv", "k_ion"} /* 2 */,
                                       _nrn_mechanism_field<double*>{"_ion_PKAci", "PKAc_ion"} /* 3 */,
                                       _nrn_mechanism_field<double*>{"_ion_PKAco", "PKAc_ion"} /* 4 */,
                                       _nrn_mechanism_field<int>{"_cvode_ieq", "cvodeieq"} /* 5 */);
  hoc_register_prop_size(_mechtype, 27, 6);
  hoc_register_dparam_semantics(_mechtype, 0, "k_ion");
  hoc_register_dparam_semantics(_mechtype, 1, "k_ion");
  hoc_register_dparam_semantics(_mechtype, 2, "k_ion");
  hoc_register_dparam_semantics(_mechtype, 3, "PKAc_ion");
  hoc_register_dparam_semantics(_mechtype, 4, "PKAc_ion");
  hoc_register_dparam_semantics(_mechtype, 5, "cvodeieq");
 	hoc_register_cvode(_mechtype, _ode_count, _ode_map, _ode_spec, _ode_matsol);
 	hoc_register_tolerance(_mechtype, _hoc_state_tol, &_atollist);
 
    hoc_register_var(hoc_scdoub, hoc_vdoub, hoc_intfunc);
 	ivoc_help("help ?1 kaf_ms /Users/peirui/BasalGangliaData/data/neurons/mechanisms/kaf_ms.mod\n");
 hoc_register_limits(_mechtype, _hoc_parm_limits);
 hoc_register_units(_mechtype, _hoc_parm_units);
 }
static int _reset;
static const char *modelname = "Fast A-type potassium current (Kv4.2)";

static int error;
static int _ninits = 0;
static int _match_recurse=1;
static void _modl_cleanup(){ _match_recurse=1;}
static int rates(_internalthreadargsproto_);
 
static int _ode_spec1(_internalthreadargsproto_);
/*static int _ode_matsol1(_internalthreadargsproto_);*/
 static neuron::container::field_index _slist1[2], _dlist1[2];
 static int states(_internalthreadargsproto_);
 
/*CVODE*/
 static int _ode_spec1 (_internalthreadargsproto_) {int _reset = 0; {
   rates ( _threadargs_ ) ;
   Dm = ( minf - m ) / mtau * q ;
   Dh = ( hinf - h ) / htau * q ;
   }
 return _reset;
}
 static int _ode_matsol1 (_internalthreadargsproto_) {
 rates ( _threadargs_ ) ;
 Dm = Dm  / (1. - dt*( ( ( ( ( - 1.0 ) ) ) / mtau )*( q ) )) ;
 Dh = Dh  / (1. - dt*( ( ( ( ( - 1.0 ) ) ) / htau )*( q ) )) ;
  return 0;
}
 /*END CVODE*/
 static int states (_internalthreadargsproto_) { {
   rates ( _threadargs_ ) ;
    m = m + (1. - exp(dt*(( ( ( ( - 1.0 ) ) ) / mtau )*( q ))))*(- ( ( ( ( minf ) ) / mtau )*( q ) ) / ( ( ( ( ( - 1.0 ) ) ) / mtau )*( q ) ) - m) ;
    h = h + (1. - exp(dt*(( ( ( ( - 1.0 ) ) ) / htau )*( q ))))*(- ( ( ( ( hinf ) ) / htau )*( q ) ) / ( ( ( ( ( - 1.0 ) ) ) / htau )*( q ) ) - h) ;
   }
  return 0;
}
 
static int  rates ( _internalthreadargsproto_ ) {
    minf = 1.0 / ( 1.0 + exp ( ( v - ( - 10.0 + modShift ) ) / ( - 17.7 ) ) ) ;
   mtau = 0.9 + 1.1 / ( 1.0 + exp ( ( v - ( - 30.0 ) ) / 10.0 ) ) ;
   hinf = 1.0 / ( 1.0 + exp ( ( v - ( - 75.6 ) ) / 11.8 ) ) ;
   htau = 14.0 ;
     return 0; }
 
static void _hoc_rates(void) {
  double _r;
 Datum* _ppvar; Datum* _thread; NrnThread* _nt;
 
  if(!_prop_id) {
    hoc_execerror("No data for rates_kaf_ms. Requires prior call to setdata_kaf_ms and that the specified mechanism instance still be in existence.", NULL);
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
 _r = 1.;
 rates ( _threadargs_ );
 hoc_retpushx(_r);
}
 
static double _npy_rates(Prop* _prop) {
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
 _r = 1.;
 rates ( _threadargs_ );
 return(_r);
}
 
double hill ( _internalthreadargsprotocomma_ double _lconc , double _lmod_min , double _lmod_max , double _lhalf_activation , double _lhill_coefficient ) {
   double _lhill;
  _lhill = _lmod_min + ( _lmod_max - _lmod_min ) * pow ( _lconc , _lhill_coefficient ) / ( pow ( _lconc , _lhill_coefficient ) + pow ( _lhalf_activation , _lhill_coefficient ) ) ;
    
return _lhill;
 }
 
static void _hoc_hill(void) {
  double _r;
 Datum* _ppvar; Datum* _thread; NrnThread* _nt;
 
  Prop* _local_prop = _prop_id ? _extcall_prop : nullptr;
  _nrn_mechanism_cache_instance _ml_real{_local_prop};
auto* const _ml = &_ml_real;
size_t const _iml{};
_ppvar = _local_prop ? _nrn_mechanism_access_dparam(_local_prop) : nullptr;
_thread = _extcall_thread.data();
double* _globals = nullptr;
if (gind != 0 && _thread != nullptr) { _globals = _thread[_gth].get<double*>(); }
_nt = nrn_threads;
 _r =  hill ( _threadargscomma_ *getarg(1) , *getarg(2) , *getarg(3) , *getarg(4) , *getarg(5) );
 hoc_retpushx(_r);
}
 
static double _npy_hill(Prop* _prop) {
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
 _r =  hill ( _threadargscomma_ *getarg(1) , *getarg(2) , *getarg(3) , *getarg(4) , *getarg(5) );
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
  ek = _ion_ek;
  PKAci = _ion_PKAci;
 _ode_matsol_instance1(_threadargs_);
 }}

static void initmodel(_internalthreadargsproto_) {
  int _i; double _save;{
  h = h0;
  m = m0;
 {
   rates ( _threadargs_ ) ;
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
   _v = _vec_v[_ni[_iml]];
 v = _v;
  ek = _ion_ek;
  PKAci = _ion_PKAci;
 initmodel(_threadargs_);
 }
}

static double _nrn_current(_internalthreadargsprotocomma_ double _v) {
double _current=0.; v=_v;
{ {
   modulation_factor_g = hill ( _threadargscomma_ PKAci , mod_pka_g_min , mod_pka_g_max , mod_pka_g_half , mod_pka_g_hill ) ;
   modulation_factor_shift = hill ( _threadargscomma_ PKAci , mod_pka_shift_min , mod_pka_shift_max , mod_pka_shift_half , mod_pka_shift_hill ) ;
   gk = gbar * m * m * h * modulation_factor_g ;
   modShift = modulation_factor_shift ;
   ik = gk * ( v - ek ) ;
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
  PKAci = _ion_PKAci;
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
  PKAci = _ion_PKAci;
 {   states(_threadargs_);
  } }}

}

static void terminal(){}

static void _initlists(){
 int _i; static int _first = 1;
  if (!_first) return;
 _slist1[0] = {m_columnindex, 0};  _dlist1[0] = {Dm_columnindex, 0};
 _slist1[1] = {h_columnindex, 0};  _dlist1[1] = {Dh_columnindex, 0};
_first = 0;
}

#if NMODL_TEXT
static void register_nmodl_text_and_filename(int mech_type) {
    const char* nmodl_filename = "/Users/peirui/BasalGangliaData/data/neurons/mechanisms/kaf_ms.mod";
    const char* nmodl_file_text = 
  "TITLE Fast A-type potassium current (Kv4.2)\n"
  "\n"
  "\n"
  "NEURON {\n"
  "    SUFFIX kaf_ms\n"
  "    USEION k READ ek WRITE ik\n"
  "    RANGE gbar, gk, ik, q\n"
  "\n"
  "    USEION PKAc READ PKAci VALENCE 0\n"
  "    RANGE mod_pka_g_min, mod_pka_g_max, mod_pka_g_half, mod_pka_g_hill\n"
  "    RANGE mod_pka_shift_min, mod_pka_shift_max, mod_pka_shift_half, mod_pka_shift_hill\n"
  "    RANGE modulation_factor_g, modulation_factor_shift\n"
  "			     \n"
  "    RANGE modShift\n"
  "}\n"
  "\n"
  "UNITS {\n"
  "    (S) = (siemens)\n"
  "    (mV) = (millivolt)\n"
  "    (mA) = (milliamp)\n"
  "}\n"
  "\n"
  "PARAMETER {\n"
  "    gbar = 0.0 (S/cm2) \n"
  "    q = 1	: room temperature (unspecified)\n"
  "    :q = 2	: body temperature 35 C (Du 2017)\n"
  "    :q = 3	: body temperature 35 C\n"
  "\n"
  "    mod_pka_g_min = 1 (1)\n"
  "    mod_pka_g_max = 1 (1)\n"
  "    mod_pka_g_half = 0.000100 (mM)\n"
  "    mod_pka_g_hill = 4 (1)\n"
  "\n"
  "    mod_pka_shift_min = 0 (1)\n"
  "    mod_pka_shift_max = 0 (1)\n"
  "    mod_pka_shift_half = 0.000100 (mM)\n"
  "    mod_pka_shift_hill = 4 (1)\n"
  "			   \n"
  "}\n"
  "\n"
  "ASSIGNED {\n"
  "    v (mV)\n"
  "    ek (mV)\n"
  "    ik (mA/cm2)\n"
  "    gk (S/cm2)\n"
  "    minf\n"
  "    mtau (ms)\n"
  "    hinf\n"
  "    htau (ms)\n"
  "    PKAci (mM)\n"
  "    modulation_factor_g (1)\n"
  "    modulation_factor_shift (1)    \n"
  "    modShift\n"
  "}\n"
  "\n"
  "STATE { m h }\n"
  "\n"
  "BREAKPOINT {\n"
  "     SOLVE states METHOD cnexp\n"
  "	   modulation_factor_g=hill(PKAci, mod_pka_g_min, mod_pka_g_max, mod_pka_g_half, mod_pka_g_hill)\n"
  "     modulation_factor_shift=hill(PKAci, mod_pka_shift_min, mod_pka_shift_max, mod_pka_shift_half, mod_pka_shift_hill)	   					 \n"
  "\n"
  "    : In Johanna's version gk depended on modDA, and modShift on modACh\n"
  "    gk = gbar*m*m*h*modulation_factor_g\n"
  "    modShift = modulation_factor_shift			     \n"
  "    ik = gk*(v-ek)\n"
  "}\n"
  "\n"
  "DERIVATIVE states {\n"
  "    rates()\n"
  "    m' = (minf-m)/mtau*q\n"
  "    h' = (hinf-h)/htau*q\n"
  "}\n"
  "\n"
  "INITIAL {\n"
  "    rates()\n"
  "    m = minf\n"
  "    h = hinf\n"
  "}\n"
  "\n"
  "PROCEDURE rates() {\n"
  "    UNITSOFF\n"
  "    minf = 1/(1+exp((v-(-10+modShift))/(-17.7)))\n"
  "    mtau = 0.9+1.1/(1+exp((v-(-30))/10))\n"
  "    hinf = 1/(1+exp((v-(-75.6))/11.8))\n"
  "    htau = 14\n"
  "    UNITSON\n"
  "\n"
  "    :Du 2017\n"
  "    :LOCAL alpha, beta, sum\n"
  "    :UNITSOFF\n"
  "    :alpha = 1.5/(1+exp((v-4)/(-17)))\n"
  "    :beta = 0.6/(1+exp((v-10)/9))\n"
  "    :sum = alpha+beta\n"
  "    :minf = alpha/sum\n"
  "    :mtau = 1/sum\n"
  "    :\n"
  "    :alpha = 0.105/(1+exp((v-(-121))/22))\n"
  "    :beta = 0.065/(1+exp((v-(-55))/(-11)))\n"
  "    :sum = alpha+beta\n"
  "    :hinf = alpha/sum\n"
  "    :htau = 1/sum\n"
  "    :UNITSON\n"
  "}\n"
  "\n"
  "FUNCTION hill(conc (mM),  mod_min (1), mod_max (1), half_activation (mM), hill_coefficient (1)) (1) {\n"
  "	UNITSOFF\n"
  "	hill = mod_min + (mod_max-mod_min) * pow(conc, hill_coefficient) / (pow(conc, hill_coefficient) + pow(half_activation, hill_coefficient))\n"
  "	UNITSON\n"
  "}\n"
  "\n"
  "\n"
  "COMMENT\n"
  "\n"
  "Original data by Tkatch et al (2000) [1]. Neostriatal neurons were acutely\n"
  "dissociated from young adult rats, age P28-P42.  Electrophysiological\n"
  "recordings were done at unspecified temperature (room temperature 20-22 C\n"
  "assumed). Potentials were not corrected for the liquid junction potential\n"
  "(estimated 1-2 mV).\n"
  "\n"
  "Activation m^1 matches experimental data [1, Fig.2C]. Activation time\n"
  "constants fit tabulated data [1, Fig.2B].  Slope of inactivation function\n"
  "fitted to the data [1, Fig.3B] with half inactivation potential -75.6\n"
  "mV. Temperature factor q between 1.5 [3] and 3 [2] was used for body\n"
  "temperature.  Conductance kinetics of m2h type is used [2], no corrections\n"
  "for m^2 applied. Later modification by Du [4] is close to this model.\n"
  "\n"
  "[1] Tkatch T, Baranauskas G, Surmeier DJ (2000) Kv4.2 mRNA abundance and\n"
  "A-type K(+) current amplitude are linearly related in basal ganglia and\n"
  "basal forebrain neurons. J Neurosci 20(2):579-88.\n"
  "\n"
  "[2] Wolf JA, Moyer JT, Lazarewicz MT, Contreras D, Benoit-Marand M,\n"
  "O'Donnell P, Finkel LH (2005) NMDA/AMPA ratio impacts state transitions\n"
  "and entrainment to oscillations in a computational model of the nucleus\n"
  "accumbens medium spiny projection neuron. J Neurosci 25(40):9080-95.\n"
  "\n"
  "[3]  Evans RC, Morera-Herreras T, Cui Y, Du K, Sheehan T, Kotaleski JH,\n"
  "Venance L, Blackwell KT (2012) The effects of NMDA subunit composition on\n"
  "calcium influx and spike timing-dependent plasticity in striatal medium\n"
  "spiny neurons. PLoS Comput Biol 8(4):e1002493.\n"
  "\n"
  "[4] Du K, Wu YW, Lindroos R, Liu Y, R\n"
  "\n"
  "zsa B, Katona G, Ding JB,\n"
  "Kotaleski JH (2017) Cell-type-specific inhibition of the dendritic\n"
  "plateau potential in striatal spiny projection neurons. Proc Natl Acad\n"
  "Sci USA 114:E7612-E7621.\n"
  "\n"
  "ENDCOMMENT\n"
  ;
    hoc_reg_nmodl_filename(mech_type, nmodl_filename);
    hoc_reg_nmodl_text(mech_type, nmodl_file_text);
}
#endif
