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
static constexpr auto number_of_datum_variables = 2;
static constexpr auto number_of_floating_point_variables = 16;
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
 
#define nrn_init _nrn_init__hd_lts
#define _nrn_initial _nrn_initial__hd_lts
#define nrn_cur _nrn_cur__hd_lts
#define _nrn_current _nrn_current__hd_lts
#define nrn_jacob _nrn_jacob__hd_lts
#define nrn_state _nrn_state__hd_lts
#define _net_receive _net_receive__hd_lts 
#define rate rate__hd_lts 
#define states states__hd_lts 
 
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
#define ghdbar _ml->template fpfield<0>(_iml)
#define ghdbar_columnindex 0
#define vhalfl _ml->template fpfield<1>(_iml)
#define vhalfl_columnindex 1
#define mod_pka_g_min _ml->template fpfield<2>(_iml)
#define mod_pka_g_min_columnindex 2
#define mod_pka_g_max _ml->template fpfield<3>(_iml)
#define mod_pka_g_max_columnindex 3
#define mod_pka_g_half _ml->template fpfield<4>(_iml)
#define mod_pka_g_half_columnindex 4
#define mod_pka_g_slope _ml->template fpfield<5>(_iml)
#define mod_pka_g_slope_columnindex 5
#define i _ml->template fpfield<6>(_iml)
#define i_columnindex 6
#define linf _ml->template fpfield<7>(_iml)
#define linf_columnindex 7
#define taul _ml->template fpfield<8>(_iml)
#define taul_columnindex 8
#define modulation_factor _ml->template fpfield<9>(_iml)
#define modulation_factor_columnindex 9
#define l _ml->template fpfield<10>(_iml)
#define l_columnindex 10
#define Dl _ml->template fpfield<11>(_iml)
#define Dl_columnindex 11
#define ghd _ml->template fpfield<12>(_iml)
#define ghd_columnindex 12
#define PKAci _ml->template fpfield<13>(_iml)
#define PKAci_columnindex 13
#define v _ml->template fpfield<14>(_iml)
#define v_columnindex 14
#define _g _ml->template fpfield<15>(_iml)
#define _g_columnindex 15
#define _ion_PKAci *(_ml->dptr_field<0>(_iml))
#define _p_ion_PKAci static_cast<neuron::container::data_handle<double>>(_ppvar[0])
#define _ion_PKAco *(_ml->dptr_field<1>(_iml))
#define _p_ion_PKAco static_cast<neuron::container::data_handle<double>>(_ppvar[1])
 /* Thread safe. No static _ml, _iml or _ppvar. */
 static int hoc_nrnpointerindex =  -1;
 static _nrn_mechanism_std_vector<Datum> _extcall_thread;
 static Prop* _extcall_prop;
 /* _prop_id kind of shadows _extcall_prop to allow validity checking. */
 static _nrn_non_owning_id_without_container _prop_id{};
 /* external NEURON variables */
 extern double celsius;
 /* declaration of user functions */
 static void _hoc_alpt(void);
 static void _hoc_bett(void);
 static void _hoc_modulation(void);
 static void _hoc_rate(void);
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
 {"setdata_hd_lts", _hoc_setdata},
 {"alpt_hd_lts", _hoc_alpt},
 {"bett_hd_lts", _hoc_bett},
 {"modulation_hd_lts", _hoc_modulation},
 {"rate_hd_lts", _hoc_rate},
 {0, 0}
};
 
/* Direct Python call wrappers to density mechanism functions.*/
 static double _npy_alpt(Prop*);
 static double _npy_bett(Prop*);
 static double _npy_modulation(Prop*);
 static double _npy_rate(Prop*);
 
static NPyDirectMechFunc npy_direct_func_proc[] = {
 {"alpt", _npy_alpt},
 {"bett", _npy_bett},
 {"modulation", _npy_modulation},
 {"rate", _npy_rate},
 {0, 0}
};
#define alpt alpt_hd_lts
#define bett bett_hd_lts
#define modulation modulation_hd_lts
 extern double alpt( _internalthreadargsprotocomma_ double );
 extern double bett( _internalthreadargsprotocomma_ double );
 extern double modulation( _internalthreadargsprotocomma_ double , double , double , double , double );
 /* declare global and static user variables */
 #define gind 0
 #define _gth 0
#define a0t a0t_hd_lts
 double a0t = 0.011;
#define ehd ehd_hd_lts
 double ehd = -30;
#define gmt gmt_hd_lts
 double gmt = 0.4;
#define kl kl_hd_lts
 double kl = -8;
#define qtl qtl_hd_lts
 double qtl = 1;
#define q10 q10_hd_lts
 double q10 = 4.5;
#define vhalft vhalft_hd_lts
 double vhalft = -75;
#define zetat zetat_hd_lts
 double zetat = 2.2;
 /* some parameters have upper and lower limits */
 static HocParmLimits _hoc_parm_limits[] = {
 {0, 0, 0}
};
 static HocParmUnits _hoc_parm_units[] = {
 {"ehd_hd_lts", "mV"},
 {"vhalft_hd_lts", "mV"},
 {"a0t_hd_lts", "/ms"},
 {"zetat_hd_lts", "1"},
 {"gmt_hd_lts", "1"},
 {"ghdbar_hd_lts", "mho/cm2"},
 {"vhalfl_hd_lts", "mV"},
 {"mod_pka_g_min_hd_lts", "1"},
 {"mod_pka_g_max_hd_lts", "1"},
 {"mod_pka_g_half_hd_lts", "mM"},
 {"mod_pka_g_slope_hd_lts", "mM"},
 {"i_hd_lts", "mA/cm2"},
 {"modulation_factor_hd_lts", "1"},
 {0, 0}
};
 static double delta_t = 0.01;
 static double l0 = 0;
 /* connect global user variables to hoc */
 static DoubScal hoc_scdoub[] = {
 {"ehd_hd_lts", &ehd_hd_lts},
 {"kl_hd_lts", &kl_hd_lts},
 {"vhalft_hd_lts", &vhalft_hd_lts},
 {"a0t_hd_lts", &a0t_hd_lts},
 {"zetat_hd_lts", &zetat_hd_lts},
 {"gmt_hd_lts", &gmt_hd_lts},
 {"q10_hd_lts", &q10_hd_lts},
 {"qtl_hd_lts", &qtl_hd_lts},
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
 
#define _cvode_ieq _ppvar[2].literal_value<int>()
 static void _ode_matsol_instance1(_internalthreadargsproto_);
 /* connect range variables in _p that hoc is supposed to know about */
 static const char *_mechanism[] = {
 "7.7.0",
"hd_lts",
 "ghdbar_hd_lts",
 "vhalfl_hd_lts",
 "mod_pka_g_min_hd_lts",
 "mod_pka_g_max_hd_lts",
 "mod_pka_g_half_hd_lts",
 "mod_pka_g_slope_hd_lts",
 0,
 "i_hd_lts",
 "linf_hd_lts",
 "taul_hd_lts",
 "modulation_factor_hd_lts",
 0,
 "l_hd_lts",
 0,
 0};
 static Symbol* _PKAc_sym;
 
 /* Used by NrnProperty */
 static _nrn_mechanism_std_vector<double> _parm_default{
     0.0001, /* ghdbar */
     -81, /* vhalfl */
     1, /* mod_pka_g_min */
     1, /* mod_pka_g_max */
     0.0001, /* mod_pka_g_half */
     0.01, /* mod_pka_g_slope */
 }; 
 
 
extern Prop* need_memb(Symbol*);
static void nrn_alloc(Prop* _prop) {
  Prop *prop_ion{};
  Datum *_ppvar{};
   _ppvar = nrn_prop_datum_alloc(_mechtype, 3, _prop);
    _nrn_mechanism_access_dparam(_prop) = _ppvar;
     _nrn_mechanism_cache_instance _ml_real{_prop};
    auto* const _ml = &_ml_real;
    size_t const _iml{};
    assert(_nrn_mechanism_get_num_vars(_prop) == 16);
 	/*initialize range parameters*/
 	ghdbar = _parm_default[0]; /* 0.0001 */
 	vhalfl = _parm_default[1]; /* -81 */
 	mod_pka_g_min = _parm_default[2]; /* 1 */
 	mod_pka_g_max = _parm_default[3]; /* 1 */
 	mod_pka_g_half = _parm_default[4]; /* 0.0001 */
 	mod_pka_g_slope = _parm_default[5]; /* 0.01 */
 	 assert(_nrn_mechanism_get_num_vars(_prop) == 16);
 	_nrn_mechanism_access_dparam(_prop) = _ppvar;
 	/*connect ionic variables to this model*/
 prop_ion = need_memb(_PKAc_sym);
 nrn_promote(prop_ion, 1, 0);
 	_ppvar[0] = _nrn_mechanism_get_param_handle(prop_ion, 1); /* PKAci */
 	_ppvar[1] = _nrn_mechanism_get_param_handle(prop_ion, 2); /* PKAco */
 
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

 extern "C" void _hd_lts_reg() {
	int _vectorized = 1;
  _initlists();
 	ion_reg("PKAc", 0.0);
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
                                       _nrn_mechanism_field<double>{"ghdbar"} /* 0 */,
                                       _nrn_mechanism_field<double>{"vhalfl"} /* 1 */,
                                       _nrn_mechanism_field<double>{"mod_pka_g_min"} /* 2 */,
                                       _nrn_mechanism_field<double>{"mod_pka_g_max"} /* 3 */,
                                       _nrn_mechanism_field<double>{"mod_pka_g_half"} /* 4 */,
                                       _nrn_mechanism_field<double>{"mod_pka_g_slope"} /* 5 */,
                                       _nrn_mechanism_field<double>{"i"} /* 6 */,
                                       _nrn_mechanism_field<double>{"linf"} /* 7 */,
                                       _nrn_mechanism_field<double>{"taul"} /* 8 */,
                                       _nrn_mechanism_field<double>{"modulation_factor"} /* 9 */,
                                       _nrn_mechanism_field<double>{"l"} /* 10 */,
                                       _nrn_mechanism_field<double>{"Dl"} /* 11 */,
                                       _nrn_mechanism_field<double>{"ghd"} /* 12 */,
                                       _nrn_mechanism_field<double>{"PKAci"} /* 13 */,
                                       _nrn_mechanism_field<double>{"v"} /* 14 */,
                                       _nrn_mechanism_field<double>{"_g"} /* 15 */,
                                       _nrn_mechanism_field<double*>{"_ion_PKAci", "PKAc_ion"} /* 0 */,
                                       _nrn_mechanism_field<double*>{"_ion_PKAco", "PKAc_ion"} /* 1 */,
                                       _nrn_mechanism_field<int>{"_cvode_ieq", "cvodeieq"} /* 2 */);
  hoc_register_prop_size(_mechtype, 16, 3);
  hoc_register_dparam_semantics(_mechtype, 0, "PKAc_ion");
  hoc_register_dparam_semantics(_mechtype, 1, "PKAc_ion");
  hoc_register_dparam_semantics(_mechtype, 2, "cvodeieq");
 	hoc_register_cvode(_mechtype, _ode_count, _ode_map, _ode_spec, _ode_matsol);
 	hoc_register_tolerance(_mechtype, _hoc_state_tol, &_atollist);
 
    hoc_register_var(hoc_scdoub, hoc_vdoub, hoc_intfunc);
 	ivoc_help("help ?1 hd_lts /Users/peirui/BasalGangliaData/data/neurons/mechanisms/hd_lts.mod\n");
 hoc_register_limits(_mechtype, _hoc_parm_limits);
 hoc_register_units(_mechtype, _hoc_parm_units);
 }
static int _reset;
static const char *modelname = "I-h channel from Magee 1998 for distal dendrites";

static int error;
static int _ninits = 0;
static int _match_recurse=1;
static void _modl_cleanup(){ _match_recurse=1;}
static int rate(_internalthreadargsprotocomma_ double);
 
static int _ode_spec1(_internalthreadargsproto_);
/*static int _ode_matsol1(_internalthreadargsproto_);*/
 static neuron::container::field_index _slist1[1], _dlist1[1];
 static int states(_internalthreadargsproto_);
 
double alpt ( _internalthreadargsprotocomma_ double _lv ) {
   double _lalpt;
 _lalpt = exp ( 0.0378 * zetat * ( _lv - vhalft ) ) ;
   
return _lalpt;
 }
 
static void _hoc_alpt(void) {
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
 _r =  alpt ( _threadargscomma_ *getarg(1) );
 hoc_retpushx(_r);
}
 
static double _npy_alpt(Prop* _prop) {
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
 _r =  alpt ( _threadargscomma_ *getarg(1) );
 return(_r);
}
 
double bett ( _internalthreadargsprotocomma_ double _lv ) {
   double _lbett;
 _lbett = exp ( 0.0378 * zetat * gmt * ( _lv - vhalft ) ) ;
   
return _lbett;
 }
 
static void _hoc_bett(void) {
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
 _r =  bett ( _threadargscomma_ *getarg(1) );
 hoc_retpushx(_r);
}
 
static double _npy_bett(Prop* _prop) {
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
 _r =  bett ( _threadargscomma_ *getarg(1) );
 return(_r);
}
 
/*CVODE*/
 static int _ode_spec1 (_internalthreadargsproto_) {int _reset = 0; {
   rate ( _threadargscomma_ v ) ;
   Dl = ( linf - l ) / taul ;
   }
 return _reset;
}
 static int _ode_matsol1 (_internalthreadargsproto_) {
 rate ( _threadargscomma_ v ) ;
 Dl = Dl  / (1. - dt*( ( ( ( - 1.0 ) ) ) / taul )) ;
  return 0;
}
 /*END CVODE*/
 static int states (_internalthreadargsproto_) { {
   rate ( _threadargscomma_ v ) ;
    l = l + (1. - exp(dt*(( ( ( - 1.0 ) ) ) / taul)))*(- ( ( ( linf ) ) / taul ) / ( ( ( ( - 1.0 ) ) ) / taul ) - l) ;
   }
  return 0;
}
 
static int  rate ( _internalthreadargsprotocomma_ double _lv ) {
   double _la , _lqt ;
 _lqt = pow( q10 , ( ( celsius - 33.0 ) / 10.0 ) ) ;
   _la = alpt ( _threadargscomma_ _lv ) ;
   linf = 1.0 / ( 1.0 + exp ( - ( _lv - vhalfl ) / kl ) ) ;
   taul = bett ( _threadargscomma_ _lv ) / ( qtl * _lqt * a0t * ( 1.0 + _la ) ) ;
    return 0; }
 
static void _hoc_rate(void) {
  double _r;
 Datum* _ppvar; Datum* _thread; NrnThread* _nt;
 
  if(!_prop_id) {
    hoc_execerror("No data for rate_hd_lts. Requires prior call to setdata_hd_lts and that the specified mechanism instance still be in existence.", NULL);
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
 rate ( _threadargscomma_ *getarg(1) );
 hoc_retpushx(_r);
}
 
static double _npy_rate(Prop* _prop) {
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
 rate ( _threadargscomma_ *getarg(1) );
 return(_r);
}
 
double modulation ( _internalthreadargsprotocomma_ double _lconc , double _lmod_min , double _lmod_max , double _lmod_half , double _lmod_slope ) {
   double _lmodulation;
 _lmodulation = _lmod_min + ( _lmod_max - _lmod_min ) / ( 1.0 + exp ( - ( _lconc - _lmod_half ) / _lmod_slope ) ) ;
   
return _lmodulation;
 }
 
static void _hoc_modulation(void) {
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
 _r =  modulation ( _threadargscomma_ *getarg(1) , *getarg(2) , *getarg(3) , *getarg(4) , *getarg(5) );
 hoc_retpushx(_r);
}
 
static double _npy_modulation(Prop* _prop) {
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
 _r =  modulation ( _threadargscomma_ *getarg(1) , *getarg(2) , *getarg(3) , *getarg(4) , *getarg(5) );
 return(_r);
}
 
static int _ode_count(int _type){ return 1;}
 
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
  for (int _i=0; _i < 1; ++_i) {
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

static void initmodel(_internalthreadargsproto_) {
  int _i; double _save;{
  l = l0;
 {
   rate ( _threadargscomma_ v ) ;
   l = linf ;
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
  PKAci = _ion_PKAci;
 initmodel(_threadargs_);
}
}

static double _nrn_current(_internalthreadargsprotocomma_ double _v) {
double _current=0.; v=_v;
{ {
   modulation_factor = modulation ( _threadargscomma_ PKAci , mod_pka_g_min , mod_pka_g_max , mod_pka_g_half , mod_pka_g_slope ) ;
   ghd = ghdbar * l * modulation_factor ;
   i = ghd * ( v - ehd ) ;
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
 {   states(_threadargs_);
  }}}

}

static void terminal(){}

static void _initlists(){
 int _i; static int _first = 1;
  if (!_first) return;
 _slist1[0] = {l_columnindex, 0};  _dlist1[0] = {Dl_columnindex, 0};
_first = 0;
}

#if NMODL_TEXT
static void register_nmodl_text_and_filename(int mech_type) {
    const char* nmodl_filename = "/Users/peirui/BasalGangliaData/data/neurons/mechanisms/hd_lts.mod";
    const char* nmodl_file_text = 
  "TITLE I-h channel from Magee 1998 for distal dendrites\n"
  "\n"
  "\n"
  "NEURON {\n"
  "	SUFFIX hd_lts\n"
  "	NONSPECIFIC_CURRENT i\n"
  "        RANGE ghdbar, vhalfl\n"
  "        RANGE linf,taul\n"
  "\n"
  "    USEION PKAc READ PKAci VALENCE 0\n"
  "    RANGE mod_pka_g_min, mod_pka_g_max, mod_pka_g_half, mod_pka_g_slope \n"
  "    RANGE modulation_factor			\n"
  "}\n"
  "\n"
  "UNITS {\n"
  "	(mA) = (milliamp)\n"
  "	(mV) = (millivolt)\n"
  "\n"
  "}\n"
  "\n"
  "PARAMETER {\n"
  "                        v 		        (mV)\n"
  "			ehd  = -30		(mV)        \n"
  "			celsius 	    (degC)\n"
  "			ghdbar=.0001 	(mho/cm2)\n"
  "			vhalfl=-81   	(mV)\n"
  "			kl=-8\n"
  "			vhalft=-75   	(mV)\n"
  "			a0t=0.011      	(/ms)\n"
  "		        zetat=2.2    	(1)\n"
  "			gmt=.4   	    (1)\n"
  "			q10=4.5\n"
  "		        qtl=1\n"
  "    mod_pka_g_min = 1 (1)\n"
  "    mod_pka_g_max = 1 (1)\n"
  "    mod_pka_g_half = 0.000100 (mM)\n"
  "    mod_pka_g_slope = 0.01 (mM)\n"
  "}\n"
  "\n"
  "\n"
  "\n"
  "STATE {\n"
  "    l\n"
  "}\n"
  "\n"
  "ASSIGNED {\n"
  "	i (mA/cm2)\n"
  "    linf      \n"
  "    taul\n"
  "    ghd\n"
  "    PKAci (mM)\n"
  "    modulation_factor (1)\n"
  "}\n"
  "\n"
  "INITIAL {\n"
  "	rate(v)\n"
  "	l=linf\n"
  "}\n"
  "\n"
  "\n"
  "BREAKPOINT {\n"
  "     SOLVE states METHOD cnexp\n"
  "     modulation_factor=modulation(PKAci, mod_pka_g_min, mod_pka_g_max, mod_pka_g_half, mod_pka_g_slope)	   \n"
  "	   \n"
  "     ghd = ghdbar*l*modulation_factor\n"
  "     i = ghd*(v-ehd)\n"
  "\n"
  "}\n"
  "\n"
  "\n"
  "FUNCTION alpt(v(mV)) {\n"
  "    alpt = exp(0.0378*zetat*(v-vhalft)) \n"
  "}\n"
  "\n"
  "FUNCTION bett(v(mV)) {\n"
  "    bett = exp(0.0378*zetat*gmt*(v-vhalft)) \n"
  "}\n"
  "\n"
  "DERIVATIVE states {     : exact when v held constant; integrates over dt step\n"
  "    rate(v)\n"
  "    l' =  (linf - l)/taul\n"
  "}\n"
  "\n"
  "PROCEDURE rate(v (mV)) { :callable from hoc\n"
  "    LOCAL a,qt\n"
  "    qt=q10^((celsius-33)/10)\n"
  "    a = alpt(v)\n"
  "    linf = 1/(1 + exp(-(v-vhalfl)/kl))\n"
  "    :linf = 1/(1+ alpl(v))\n"
  "    taul = bett(v)/(qtl*qt*a0t*(1+a))\n"
  "}\n"
  "\n"
  "\n"
  "\n"
  "FUNCTION modulation(conc (mM), mod_min (1), mod_max (1), mod_half (mM), mod_slope (mM)) (1) {\n"
  "    : returns modulation factor\n"
  "    modulation = mod_min + (mod_max-mod_min) / (1 + exp(-(conc - mod_half)/mod_slope))\n"
  "}\n"
  ;
    hoc_reg_nmodl_filename(mech_type, nmodl_filename);
    hoc_reg_nmodl_text(mech_type, nmodl_file_text);
}
#endif
