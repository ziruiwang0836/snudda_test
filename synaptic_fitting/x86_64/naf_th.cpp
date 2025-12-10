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
 
#define nrn_init _nrn_init__naf_th
#define _nrn_initial _nrn_initial__naf_th
#define nrn_cur _nrn_cur__naf_th
#define _nrn_current _nrn_current__naf_th
#define nrn_jacob _nrn_jacob__naf_th
#define nrn_state _nrn_state__naf_th
#define _net_receive _net_receive__naf_th 
#define rates rates__naf_th 
#define states states__naf_th 
 
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
#define ca_ratio _ml->template fpfield<2>(_iml)
#define ca_ratio_columnindex 2
#define mslope _ml->template fpfield<3>(_iml)
#define mslope_columnindex 3
#define mhalf _ml->template fpfield<4>(_iml)
#define mhalf_columnindex 4
#define hhalf _ml->template fpfield<5>(_iml)
#define hhalf_columnindex 5
#define use_hs _ml->template fpfield<6>(_iml)
#define use_hs_columnindex 6
#define vhalf_hs _ml->template fpfield<7>(_iml)
#define vhalf_hs_columnindex 7
#define slope_hs _ml->template fpfield<8>(_iml)
#define slope_hs_columnindex 8
#define tconst_hs _ml->template fpfield<9>(_iml)
#define tconst_hs_columnindex 9
#define tmin_hs _ml->template fpfield<10>(_iml)
#define tmin_hs_columnindex 10
#define thalf_hs _ml->template fpfield<11>(_iml)
#define thalf_hs_columnindex 11
#define g _ml->template fpfield<12>(_iml)
#define g_columnindex 12
#define i _ml->template fpfield<13>(_iml)
#define i_columnindex 13
#define m _ml->template fpfield<14>(_iml)
#define m_columnindex 14
#define h _ml->template fpfield<15>(_iml)
#define h_columnindex 15
#define hs _ml->template fpfield<16>(_iml)
#define hs_columnindex 16
#define ena _ml->template fpfield<17>(_iml)
#define ena_columnindex 17
#define ina _ml->template fpfield<18>(_iml)
#define ina_columnindex 18
#define ica _ml->template fpfield<19>(_iml)
#define ica_columnindex 19
#define minf _ml->template fpfield<20>(_iml)
#define minf_columnindex 20
#define mtau _ml->template fpfield<21>(_iml)
#define mtau_columnindex 21
#define hinf _ml->template fpfield<22>(_iml)
#define hinf_columnindex 22
#define htau _ml->template fpfield<23>(_iml)
#define htau_columnindex 23
#define hsinf _ml->template fpfield<24>(_iml)
#define hsinf_columnindex 24
#define hstau _ml->template fpfield<25>(_iml)
#define hstau_columnindex 25
#define Dm _ml->template fpfield<26>(_iml)
#define Dm_columnindex 26
#define Dh _ml->template fpfield<27>(_iml)
#define Dh_columnindex 27
#define Dhs _ml->template fpfield<28>(_iml)
#define Dhs_columnindex 28
#define v _ml->template fpfield<29>(_iml)
#define v_columnindex 29
#define _g _ml->template fpfield<30>(_iml)
#define _g_columnindex 30
#define _ion_ena *(_ml->dptr_field<0>(_iml))
#define _p_ion_ena static_cast<neuron::container::data_handle<double>>(_ppvar[0])
#define _ion_ina *(_ml->dptr_field<1>(_iml))
#define _p_ion_ina static_cast<neuron::container::data_handle<double>>(_ppvar[1])
#define _ion_dinadv *(_ml->dptr_field<2>(_iml))
#define _ion_ica *(_ml->dptr_field<3>(_iml))
#define _p_ion_ica static_cast<neuron::container::data_handle<double>>(_ppvar[3])
#define _ion_dicadv *(_ml->dptr_field<4>(_iml))
 /* Thread safe. No static _ml, _iml or _ppvar. */
 static int hoc_nrnpointerindex =  -1;
 static _nrn_mechanism_std_vector<Datum> _extcall_thread;
 static Prop* _extcall_prop;
 /* _prop_id kind of shadows _extcall_prop to allow validity checking. */
 static _nrn_non_owning_id_without_container _prop_id{};
 /* external NEURON variables */
 /* declaration of user functions */
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
 {"setdata_naf_th", _hoc_setdata},
 {"rates_naf_th", _hoc_rates},
 {0, 0}
};
 
/* Direct Python call wrappers to density mechanism functions.*/
 static double _npy_rates(Prop*);
 
static NPyDirectMechFunc npy_direct_func_proc[] = {
 {"rates", _npy_rates},
 {0, 0}
};
 /* declare global and static user variables */
 #define gind 0
 #define _gth 0
 /* some parameters have upper and lower limits */
 static HocParmLimits _hoc_parm_limits[] = {
 {0, 0, 0}
};
 static HocParmUnits _hoc_parm_units[] = {
 {"gbar_naf_th", "S/cm2"},
 {"g_naf_th", "S/cm2"},
 {"i_naf_th", "mA/cm2"},
 {0, 0}
};
 static double delta_t = 0.01;
 static double hs0 = 0;
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
"naf_th",
 "gbar_naf_th",
 "q_naf_th",
 "ca_ratio_naf_th",
 "mslope_naf_th",
 "mhalf_naf_th",
 "hhalf_naf_th",
 "use_hs_naf_th",
 "vhalf_hs_naf_th",
 "slope_hs_naf_th",
 "tconst_hs_naf_th",
 "tmin_hs_naf_th",
 "thalf_hs_naf_th",
 0,
 "g_naf_th",
 "i_naf_th",
 0,
 "m_naf_th",
 "h_naf_th",
 "hs_naf_th",
 0,
 0};
 static Symbol* _na_sym;
 static Symbol* _ca_sym;
 
 /* Used by NrnProperty */
 static _nrn_mechanism_std_vector<double> _parm_default{
     0, /* gbar */
     1.8, /* q */
     0.005, /* ca_ratio */
     10, /* mslope */
     -25, /* mhalf */
     -62, /* hhalf */
     1, /* use_hs */
     -54.8, /* vhalf_hs */
     1.57, /* slope_hs */
     160, /* tconst_hs */
     20, /* tmin_hs */
     47.2, /* thalf_hs */
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
    assert(_nrn_mechanism_get_num_vars(_prop) == 31);
 	/*initialize range parameters*/
 	gbar = _parm_default[0]; /* 0 */
 	q = _parm_default[1]; /* 1.8 */
 	ca_ratio = _parm_default[2]; /* 0.005 */
 	mslope = _parm_default[3]; /* 10 */
 	mhalf = _parm_default[4]; /* -25 */
 	hhalf = _parm_default[5]; /* -62 */
 	use_hs = _parm_default[6]; /* 1 */
 	vhalf_hs = _parm_default[7]; /* -54.8 */
 	slope_hs = _parm_default[8]; /* 1.57 */
 	tconst_hs = _parm_default[9]; /* 160 */
 	tmin_hs = _parm_default[10]; /* 20 */
 	thalf_hs = _parm_default[11]; /* 47.2 */
 	 assert(_nrn_mechanism_get_num_vars(_prop) == 31);
 	_nrn_mechanism_access_dparam(_prop) = _ppvar;
 	/*connect ionic variables to this model*/
 prop_ion = need_memb(_na_sym);
 nrn_promote(prop_ion, 0, 1);
 	_ppvar[0] = _nrn_mechanism_get_param_handle(prop_ion, 0); /* ena */
 	_ppvar[1] = _nrn_mechanism_get_param_handle(prop_ion, 3); /* ina */
 	_ppvar[2] = _nrn_mechanism_get_param_handle(prop_ion, 4); /* _ion_dinadv */
 prop_ion = need_memb(_ca_sym);
 	_ppvar[3] = _nrn_mechanism_get_param_handle(prop_ion, 3); /* ica */
 	_ppvar[4] = _nrn_mechanism_get_param_handle(prop_ion, 4); /* _ion_dicadv */
 
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

 extern "C" void _naf_th_reg() {
	int _vectorized = 1;
  _initlists();
 	ion_reg("na", -10000.);
 	ion_reg("ca", 2.0);
 	_na_sym = hoc_lookup("na_ion");
 	_ca_sym = hoc_lookup("ca_ion");
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
                                       _nrn_mechanism_field<double>{"ca_ratio"} /* 2 */,
                                       _nrn_mechanism_field<double>{"mslope"} /* 3 */,
                                       _nrn_mechanism_field<double>{"mhalf"} /* 4 */,
                                       _nrn_mechanism_field<double>{"hhalf"} /* 5 */,
                                       _nrn_mechanism_field<double>{"use_hs"} /* 6 */,
                                       _nrn_mechanism_field<double>{"vhalf_hs"} /* 7 */,
                                       _nrn_mechanism_field<double>{"slope_hs"} /* 8 */,
                                       _nrn_mechanism_field<double>{"tconst_hs"} /* 9 */,
                                       _nrn_mechanism_field<double>{"tmin_hs"} /* 10 */,
                                       _nrn_mechanism_field<double>{"thalf_hs"} /* 11 */,
                                       _nrn_mechanism_field<double>{"g"} /* 12 */,
                                       _nrn_mechanism_field<double>{"i"} /* 13 */,
                                       _nrn_mechanism_field<double>{"m"} /* 14 */,
                                       _nrn_mechanism_field<double>{"h"} /* 15 */,
                                       _nrn_mechanism_field<double>{"hs"} /* 16 */,
                                       _nrn_mechanism_field<double>{"ena"} /* 17 */,
                                       _nrn_mechanism_field<double>{"ina"} /* 18 */,
                                       _nrn_mechanism_field<double>{"ica"} /* 19 */,
                                       _nrn_mechanism_field<double>{"minf"} /* 20 */,
                                       _nrn_mechanism_field<double>{"mtau"} /* 21 */,
                                       _nrn_mechanism_field<double>{"hinf"} /* 22 */,
                                       _nrn_mechanism_field<double>{"htau"} /* 23 */,
                                       _nrn_mechanism_field<double>{"hsinf"} /* 24 */,
                                       _nrn_mechanism_field<double>{"hstau"} /* 25 */,
                                       _nrn_mechanism_field<double>{"Dm"} /* 26 */,
                                       _nrn_mechanism_field<double>{"Dh"} /* 27 */,
                                       _nrn_mechanism_field<double>{"Dhs"} /* 28 */,
                                       _nrn_mechanism_field<double>{"v"} /* 29 */,
                                       _nrn_mechanism_field<double>{"_g"} /* 30 */,
                                       _nrn_mechanism_field<double*>{"_ion_ena", "na_ion"} /* 0 */,
                                       _nrn_mechanism_field<double*>{"_ion_ina", "na_ion"} /* 1 */,
                                       _nrn_mechanism_field<double*>{"_ion_dinadv", "na_ion"} /* 2 */,
                                       _nrn_mechanism_field<double*>{"_ion_ica", "ca_ion"} /* 3 */,
                                       _nrn_mechanism_field<double*>{"_ion_dicadv", "ca_ion"} /* 4 */,
                                       _nrn_mechanism_field<int>{"_cvode_ieq", "cvodeieq"} /* 5 */);
  hoc_register_prop_size(_mechtype, 31, 6);
  hoc_register_dparam_semantics(_mechtype, 0, "na_ion");
  hoc_register_dparam_semantics(_mechtype, 1, "na_ion");
  hoc_register_dparam_semantics(_mechtype, 2, "na_ion");
  hoc_register_dparam_semantics(_mechtype, 3, "ca_ion");
  hoc_register_dparam_semantics(_mechtype, 4, "ca_ion");
  hoc_register_dparam_semantics(_mechtype, 5, "cvodeieq");
 	hoc_register_cvode(_mechtype, _ode_count, _ode_map, _ode_spec, _ode_matsol);
 	hoc_register_tolerance(_mechtype, _hoc_state_tol, &_atollist);
 
    hoc_register_var(hoc_scdoub, hoc_vdoub, hoc_intfunc);
 	ivoc_help("help ?1 naf_th /cfs/klemming/home/m/metog/BasalGangliaData/data/neurons/mechanisms/naf_th.mod\n");
 hoc_register_limits(_mechtype, _hoc_parm_limits);
 hoc_register_units(_mechtype, _hoc_parm_units);
 }
static int _reset;
static const char *modelname = "Fast transient sodium current";

static int error;
static int _ninits = 0;
static int _match_recurse=1;
static void _modl_cleanup(){ _match_recurse=1;}
static int rates(_internalthreadargsproto_);
 
static int _ode_spec1(_internalthreadargsproto_);
/*static int _ode_matsol1(_internalthreadargsproto_);*/
 static neuron::container::field_index _slist1[3], _dlist1[3];
 static int states(_internalthreadargsproto_);
 
/*CVODE*/
 static int _ode_spec1 (_internalthreadargsproto_) {int _reset = 0; {
   rates ( _threadargs_ ) ;
   Dm = ( minf - m ) / mtau * q ;
   Dh = ( hinf - h ) / htau * q ;
   if ( use_hs  == 1.0 ) {
     Dhs = ( hsinf - hs ) / hstau ;
     }
   }
 return _reset;
}
 static int _ode_matsol1 (_internalthreadargsproto_) {
 rates ( _threadargs_ ) ;
 Dm = Dm  / (1. - dt*( ( ( ( ( - 1.0 ) ) ) / mtau )*( q ) )) ;
 Dh = Dh  / (1. - dt*( ( ( ( ( - 1.0 ) ) ) / htau )*( q ) )) ;
 if ( use_hs  == 1.0 ) {
   Dhs = Dhs  / (1. - dt*( ( ( ( - 1.0 ) ) ) / hstau )) ;
   }
  return 0;
}
 /*END CVODE*/
 static int states (_internalthreadargsproto_) { {
   rates ( _threadargs_ ) ;
    m = m + (1. - exp(dt*(( ( ( ( - 1.0 ) ) ) / mtau )*( q ))))*(- ( ( ( ( minf ) ) / mtau )*( q ) ) / ( ( ( ( ( - 1.0 ) ) ) / mtau )*( q ) ) - m) ;
    h = h + (1. - exp(dt*(( ( ( ( - 1.0 ) ) ) / htau )*( q ))))*(- ( ( ( ( hinf ) ) / htau )*( q ) ) / ( ( ( ( ( - 1.0 ) ) ) / htau )*( q ) ) - h) ;
   if ( use_hs  == 1.0 ) {
      hs = hs + (1. - exp(dt*(( ( ( - 1.0 ) ) ) / hstau)))*(- ( ( ( hsinf ) ) / hstau ) / ( ( ( ( - 1.0 ) ) ) / hstau ) - hs) ;
     }
   }
  return 0;
}
 
static int  rates ( _internalthreadargsproto_ ) {
    minf = 1.0 / ( 1.0 + exp ( - ( v - mhalf ) / ( mslope ) ) ) ;
   mtau = 0.33 + 1.0 / ( exp ( ( v - ( - 62.0 ) ) / 14.0 ) + exp ( ( v - ( - 60.0 ) ) / ( - 17.0 ) ) ) ;
   hinf = 1.0 / ( 1.0 + exp ( ( v - hhalf ) / 6.0 ) ) ;
   htau = 0.6 + 1.0 / ( exp ( ( v - ( - 44.0 ) ) / 8.0 ) + exp ( ( v - ( - 99.0 ) ) / ( - 44.0 ) ) ) ;
   if ( use_hs  == 1.0 ) {
     hsinf = 1.0 / ( 1.0 + exp ( ( v - ( vhalf_hs ) ) / slope_hs ) ) ;
     hstau = tmin_hs + tconst_hs / ( 1.0 + exp ( v + thalf_hs ) ) ;
     }
     return 0; }
 
static void _hoc_rates(void) {
  double _r;
 Datum* _ppvar; Datum* _thread; NrnThread* _nt;
 
  if(!_prop_id) {
    hoc_execerror("No data for rates_naf_th. Requires prior call to setdata_naf_th and that the specified mechanism instance still be in existence.", NULL);
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
 
static int _ode_count(int _type){ return 3;}
 
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
  ena = _ion_ena;
     _ode_spec1 (_threadargs_);
   }}
 
static void _ode_map(Prop* _prop, int _ieq, neuron::container::data_handle<double>* _pv, neuron::container::data_handle<double>* _pvdot, double* _atol, int _type) { 
  Datum* _ppvar;
  _ppvar = _nrn_mechanism_access_dparam(_prop);
  _cvode_ieq = _ieq;
  for (int _i=0; _i < 3; ++_i) {
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
  ena = _ion_ena;
 _ode_matsol_instance1(_threadargs_);
 }}

static void initmodel(_internalthreadargsproto_) {
  int _i; double _save;{
  h = h0;
  hs = hs0;
  m = m0;
 {
   rates ( _threadargs_ ) ;
   m = minf ;
   h = hinf ;
   if ( use_hs  == 1.0 ) {
     hs = hsinf ;
     }
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
  ena = _ion_ena;
 initmodel(_threadargs_);
  }
}

static double _nrn_current(_internalthreadargsprotocomma_ double _v) {
double _current=0.; v=_v;
{ {
   double _litotal ;
 if ( use_hs  == 1.0 ) {
     g = gbar * m * m * m * h * hs ;
     }
   else {
     g = gbar * m * m * m * h ;
     }
   _litotal = g * ( v - ena ) ;
   ica = ca_ratio * _litotal ;
   ina = _litotal - ica ;
   i = ina + ica ;
   }
 _current += ina;
 _current += ica;

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
  ena = _ion_ena;
 auto const _g_local = _nrn_current(_threadargscomma_ _v + .001);
 	{ double _dica;
 double _dina;
  _dina = ina;
  _dica = ica;
 _rhs = _nrn_current(_threadargscomma_ _v);
  _ion_dinadv += (_dina - ina)/.001 ;
  _ion_dicadv += (_dica - ica)/.001 ;
 	}
 _g = (_g_local - _rhs)/.001;
  _ion_ina += ina ;
  _ion_ica += ica ;
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
  ena = _ion_ena;
 {   states(_threadargs_);
  }  }}

}

static void terminal(){}

static void _initlists(){
 int _i; static int _first = 1;
  if (!_first) return;
 _slist1[0] = {m_columnindex, 0};  _dlist1[0] = {Dm_columnindex, 0};
 _slist1[1] = {h_columnindex, 0};  _dlist1[1] = {Dh_columnindex, 0};
 _slist1[2] = {hs_columnindex, 0};  _dlist1[2] = {Dhs_columnindex, 0};
_first = 0;
}

#if NMODL_TEXT
static void register_nmodl_text_and_filename(int mech_type) {
    const char* nmodl_filename = "/cfs/klemming/home/m/metog/BasalGangliaData/data/neurons/mechanisms/naf_th.mod";
    const char* nmodl_file_text = 
  "TITLE Fast transient sodium current\n"
  "\n"
  "COMMENT\n"
  "\n"
  "original file decorator\n"
  "-----------------------\n"
  "Original data by Ogata and Tatebayashi (1990) [1]. Neostriatal neurons\n"
  "of medium size (putative medium spiny neurons) freshly isolated from\n"
  "the adult guinea pig brain (either sex, 200 g). Data compensated for\n"
  "the liquid junction potential (-13 mV). Experiments carried out at room\n"
  "temperature (22 C). Conductance fitted by m3h kinetics.\n"
  "\n"
  "Smooth fit of mtau and htau data [1] by Alexander Kozlov <akozlov@kth.se>\n"
  "assuming natural logarithm of tau values [1, Figs. 5 and 9] and\n"
  "temperature correction factor of 1.8 [2] as suggested by Robert Lindroos\n"
  "<robert.lindroos@ki.se>.\n"
  "\n"
  "[1] Ogata N, Tatebayashi H (1990) Sodium current kinetics in freshly\n"
  "isolated neostriatal neurones of the adult guinea pig. Pflugers Arch\n"
  "416(5):594-603.\n"
  "\n"
  "[2] Schwarz JR (1986) The effect of temperature on Na currents in rat\n"
  "myelinated nerve fibres. Pflugers Arch. 406(4):397-404.\n"
  "-----------------------------------------------------------------------\n"
  "\n"
  "-Jan -24: adding Ca permability of channel (RL)\n"
  "-Feb -24: slow inactivation gate added following dynamics of: \n"
  "    https://www.ncbi.nlm.nih.gov/pmc/articles/PMC4254877/ (RL)\n"
  "-April -24: time constant of slow inactivation updated from 160 -> 580 in accordance with: \n"
  "	https://pubmed.ncbi.nlm.nih.gov/25852980/\n"
  "	https://modeldb.science/183722?tab=2&file=YuCanavier2015/soma_bursting_ODE.m (RL)\n"
  "\n"
  "- earlier (2019-2020ish)\n"
  "\n"
  "Neuromodulation is added as functions:\n"
  "    \n"
  "    modulationDA = 1 + modDA*(maxModDA-1)*levelDA\n"
  "\n"
  "where:\n"
  "    \n"
  "    modDA  [0]: is a switch for turning modulation on or off {1/0}\n"
  "    maxModDA [1]: is the maximum modulation for this specific channel (read from the param file)\n"
  "                    e.g. 10% increase would correspond to a factor of 1.1 (100% +10%) {0-inf}\n"
  "    levelDA  [0]: is an additional parameter for scaling modulation. \n"
  "                Can be used simulate non static modulation by gradually changing the value from 0 to 1 {0-1}\n"
  "									\n"
  "	  Further neuromodulators can be added (assuming they are independent...) by for example:\n"
  "          modulationDA = 1 + modDA*(maxModDA-1)\n"
  "	  modulationACh = 1 + modACh*(maxModACh-1)\n"
  "	  ....\n"
  "\n"
  "	  etc. for other neuromodulators	   \n"
  "								     \n"
  "[] == default values\n"
  "{} == ranges\n"
  "\n"
  "ENDCOMMENT\n"
  "\n"
  "NEURON {\n"
  "    SUFFIX naf_th\n"
  "    USEION na READ ena WRITE ina\n"
  "    RANGE ca_ratio\n"
  "    USEION ca WRITE ica VALENCE 2\n"
  "    RANGE vhalf_hs, slope_hs, tconst_hs, tmin_hs, thalf_hs, hs, h, use_hs\n"
  "    RANGE mhalf, mslope, hhalf\n"
  "    RANGE gbar, g, q, i\n"
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
  "    :q = 1	: room temperature 22 C\n"
  "    q = 1.8	: body temperature 35 C\n"
  "    ca_ratio = 0.005\n"
  "    \n"
  "    mslope = 10\n"
  "    mhalf = -25\n"
  "    hhalf = -62\n"
  "    \n"
  "    use_hs = 1 : use slow inactivation gate if 1 (else don't use)\n"
  "    vhalf_hs = -54.8\n"
  "    slope_hs = 1.57\n"
  "    tconst_hs = 160\n"
  "    tmin_hs = 20\n"
  "    thalf_hs = 47.2\n"
  "}\n"
  "\n"
  "ASSIGNED {\n"
  "    v (mV)\n"
  "    ena (mV)\n"
  "    ina (mA/cm2)\n"
  "    ica (nA)\n"
  "    g (S/cm2)\n"
  "    minf\n"
  "    mtau (ms)\n"
  "    hinf\n"
  "    htau (ms)\n"
  "    hsinf\n"
  "    hstau (ms)\n"
  "    i (mA/cm2)\n"
  "}\n"
  "\n"
  "STATE { m h hs}\n"
  "\n"
  "BREAKPOINT {\n"
  "    LOCAL itotal\n"
  "    SOLVE states METHOD cnexp\n"
  "    if (use_hs == 1) {\n"
  "    	g = gbar*m*m*m*h*hs\n"
  "	} else {\n"
  "    	g = gbar*m*m*m*h\n"
  "	}\n"
  "    itotal = g*(v-ena)\n"
  "    ica = ca_ratio*itotal\n"
  "    ina = itotal - ica : should this be adjusted for valence?\n"
  "    i = ina + ica\n"
  "    \n"
  "}\n"
  "\n"
  "DERIVATIVE states {\n"
  "    rates()\n"
  "    m' = (minf-m)/mtau*q\n"
  "    h' = (hinf-h)/htau*q\n"
  "    if (use_hs == 1) {\n"
  "    	hs' = (hsinf-hs)/hstau  : q-factor not added since time constant changed anyway\n"
  "	}\n"
  "}\n"
  "\n"
  "INITIAL {\n"
  "    rates()\n"
  "    m = minf\n"
  "    h = hinf\n"
  "    if (use_hs == 1) {\n"
  "    	hs = hsinf\n"
  "	}\n"
  "}\n"
  "\n"
  "PROCEDURE rates() {\n"
  "    UNITSOFF\n"
  "    :minf = 1/(1+exp((v-(-25.5))/(-9.2)))\n"
  "    :mtau = 0.33+1/(exp((v-(-62))/14)+exp((v-(-60))/(-17)))\n"
  "    :hinf = 1/(1+exp((v-(-63.2))/6))\n"
  "    :htau = 0.6+1/(exp((v-(-44))/8)+exp((v-(-99))/(-44)))\n"
  "    minf = 1/(1+exp(-(v-mhalf)/(mslope)))\n"
  "    mtau = 0.33+1/(exp((v-(-62))/14)+exp((v-(-60))/(-17)))\n"
  "    hinf = 1/(1+exp((v-hhalf)/6))\n"
  "    htau = 0.6+1/(exp((v-(-44))/8)+exp((v-(-99))/(-44)))\n"
  "    if (use_hs == 1) {\n"
  "		hsinf = 1/(1+exp((v-(vhalf_hs))/slope_hs))\n"
  "		hstau = tmin_hs + tconst_hs/(1 + exp(v + thalf_hs))\n"
  "	}\n"
  "    UNITSON\n"
  "}\n"
  "\n"
  "\n"
  "\n"
  "\n"
  ;
    hoc_reg_nmodl_filename(mech_type, nmodl_filename);
    hoc_reg_nmodl_text(mech_type, nmodl_file_text);
}
#endif
