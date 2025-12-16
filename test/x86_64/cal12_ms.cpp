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
static constexpr auto number_of_datum_variables = 6;
static constexpr auto number_of_floating_point_variables = 21;
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
 
#define nrn_init _nrn_init__cal12_ms
#define _nrn_initial _nrn_initial__cal12_ms
#define nrn_cur _nrn_cur__cal12_ms
#define _nrn_current _nrn_current__cal12_ms
#define nrn_jacob _nrn_jacob__cal12_ms
#define nrn_state _nrn_state__cal12_ms
#define _net_receive _net_receive__cal12_ms 
#define rates rates__cal12_ms 
#define states states__cal12_ms 
 
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
#define pbar _ml->template fpfield<0>(_iml)
#define pbar_columnindex 0
#define mod_pka_p_min _ml->template fpfield<1>(_iml)
#define mod_pka_p_min_columnindex 1
#define mod_pka_p_max _ml->template fpfield<2>(_iml)
#define mod_pka_p_max_columnindex 2
#define mod_pka_p_half _ml->template fpfield<3>(_iml)
#define mod_pka_p_half_columnindex 3
#define mod_pka_p_hill _ml->template fpfield<4>(_iml)
#define mod_pka_p_hill_columnindex 4
#define ical _ml->template fpfield<5>(_iml)
#define ical_columnindex 5
#define modulation_factor _ml->template fpfield<6>(_iml)
#define modulation_factor_columnindex 6
#define m _ml->template fpfield<7>(_iml)
#define m_columnindex 7
#define h _ml->template fpfield<8>(_iml)
#define h_columnindex 8
#define ecal _ml->template fpfield<9>(_iml)
#define ecal_columnindex 9
#define cali _ml->template fpfield<10>(_iml)
#define cali_columnindex 10
#define calo _ml->template fpfield<11>(_iml)
#define calo_columnindex 11
#define minf _ml->template fpfield<12>(_iml)
#define minf_columnindex 12
#define mtau _ml->template fpfield<13>(_iml)
#define mtau_columnindex 13
#define hinf _ml->template fpfield<14>(_iml)
#define hinf_columnindex 14
#define htau _ml->template fpfield<15>(_iml)
#define htau_columnindex 15
#define PKAci _ml->template fpfield<16>(_iml)
#define PKAci_columnindex 16
#define Dm _ml->template fpfield<17>(_iml)
#define Dm_columnindex 17
#define Dh _ml->template fpfield<18>(_iml)
#define Dh_columnindex 18
#define v _ml->template fpfield<19>(_iml)
#define v_columnindex 19
#define _g _ml->template fpfield<20>(_iml)
#define _g_columnindex 20
#define _ion_cali *(_ml->dptr_field<0>(_iml))
#define _p_ion_cali static_cast<neuron::container::data_handle<double>>(_ppvar[0])
#define _ion_calo *(_ml->dptr_field<1>(_iml))
#define _p_ion_calo static_cast<neuron::container::data_handle<double>>(_ppvar[1])
#define _ion_ical *(_ml->dptr_field<2>(_iml))
#define _p_ion_ical static_cast<neuron::container::data_handle<double>>(_ppvar[2])
#define _ion_dicaldv *(_ml->dptr_field<3>(_iml))
#define _ion_PKAci *(_ml->dptr_field<4>(_iml))
#define _p_ion_PKAci static_cast<neuron::container::data_handle<double>>(_ppvar[4])
#define _ion_PKAco *(_ml->dptr_field<5>(_iml))
#define _p_ion_PKAco static_cast<neuron::container::data_handle<double>>(_ppvar[5])
 /* Thread safe. No static _ml, _iml or _ppvar. */
 static int hoc_nrnpointerindex =  -1;
 static _nrn_mechanism_std_vector<Datum> _extcall_thread;
 static Prop* _extcall_prop;
 /* _prop_id kind of shadows _extcall_prop to allow validity checking. */
 static _nrn_non_owning_id_without_container _prop_id{};
 /* external NEURON variables */
 extern double celsius;
 /* declaration of user functions */
 static void _hoc_efun(void);
 static void _hoc_ghk(void);
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
 {"setdata_cal12_ms", _hoc_setdata},
 {"efun_cal12_ms", _hoc_efun},
 {"ghk_cal12_ms", _hoc_ghk},
 {"hill_cal12_ms", _hoc_hill},
 {"rates_cal12_ms", _hoc_rates},
 {0, 0}
};
 
/* Direct Python call wrappers to density mechanism functions.*/
 static double _npy_efun(Prop*);
 static double _npy_ghk(Prop*);
 static double _npy_hill(Prop*);
 static double _npy_rates(Prop*);
 
static NPyDirectMechFunc npy_direct_func_proc[] = {
 {"efun", _npy_efun},
 {"ghk", _npy_ghk},
 {"hill", _npy_hill},
 {"rates", _npy_rates},
 {0, 0}
};
#define efun efun_cal12_ms
#define ghk ghk_cal12_ms
#define hill hill_cal12_ms
 extern double efun( _internalthreadargsprotocomma_ double );
 extern double ghk( _internalthreadargsprotocomma_ double , double , double );
 extern double hill( _internalthreadargsprotocomma_ double , double , double , double , double );
 /* declare global and static user variables */
 #define gind 0
 #define _gth 0
#define a a_cal12_ms
 double a = 0.17;
#define q q_cal12_ms
 double q = 2;
 /* some parameters have upper and lower limits */
 static HocParmLimits _hoc_parm_limits[] = {
 {0, 0, 0}
};
 static HocParmUnits _hoc_parm_units[] = {
 {"pbar_cal12_ms", "cm/s"},
 {"mod_pka_p_min_cal12_ms", "1"},
 {"mod_pka_p_max_cal12_ms", "1"},
 {"mod_pka_p_half_cal12_ms", "mM"},
 {"mod_pka_p_hill_cal12_ms", "1"},
 {"ical_cal12_ms", "mA/cm2"},
 {"modulation_factor_cal12_ms", "1"},
 {0, 0}
};
 static double delta_t = 0.01;
 static double h0 = 0;
 static double m0 = 0;
 /* connect global user variables to hoc */
 static DoubScal hoc_scdoub[] = {
 {"a_cal12_ms", &a_cal12_ms},
 {"q_cal12_ms", &q_cal12_ms},
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
 
#define _cvode_ieq _ppvar[6].literal_value<int>()
 static void _ode_matsol_instance1(_internalthreadargsproto_);
 /* connect range variables in _p that hoc is supposed to know about */
 static const char *_mechanism[] = {
 "7.7.0",
"cal12_ms",
 "pbar_cal12_ms",
 "mod_pka_p_min_cal12_ms",
 "mod_pka_p_max_cal12_ms",
 "mod_pka_p_half_cal12_ms",
 "mod_pka_p_hill_cal12_ms",
 0,
 "ical_cal12_ms",
 "modulation_factor_cal12_ms",
 0,
 "m_cal12_ms",
 "h_cal12_ms",
 0,
 0};
 static Symbol* _cal_sym;
 static Symbol* _PKAc_sym;
 
 /* Used by NrnProperty */
 static _nrn_mechanism_std_vector<double> _parm_default{
     0, /* pbar */
     1, /* mod_pka_p_min */
     1, /* mod_pka_p_max */
     0.0001, /* mod_pka_p_half */
     4, /* mod_pka_p_hill */
 }; 
 
 
extern Prop* need_memb(Symbol*);
static void nrn_alloc(Prop* _prop) {
  Prop *prop_ion{};
  Datum *_ppvar{};
   _ppvar = nrn_prop_datum_alloc(_mechtype, 7, _prop);
    _nrn_mechanism_access_dparam(_prop) = _ppvar;
     _nrn_mechanism_cache_instance _ml_real{_prop};
    auto* const _ml = &_ml_real;
    size_t const _iml{};
    assert(_nrn_mechanism_get_num_vars(_prop) == 21);
 	/*initialize range parameters*/
 	pbar = _parm_default[0]; /* 0 */
 	mod_pka_p_min = _parm_default[1]; /* 1 */
 	mod_pka_p_max = _parm_default[2]; /* 1 */
 	mod_pka_p_half = _parm_default[3]; /* 0.0001 */
 	mod_pka_p_hill = _parm_default[4]; /* 4 */
 	 assert(_nrn_mechanism_get_num_vars(_prop) == 21);
 	_nrn_mechanism_access_dparam(_prop) = _ppvar;
 	/*connect ionic variables to this model*/
 prop_ion = need_memb(_cal_sym);
 nrn_promote(prop_ion, 1, 0);
 	_ppvar[0] = _nrn_mechanism_get_param_handle(prop_ion, 1); /* cali */
 	_ppvar[1] = _nrn_mechanism_get_param_handle(prop_ion, 2); /* calo */
 	_ppvar[2] = _nrn_mechanism_get_param_handle(prop_ion, 3); /* ical */
 	_ppvar[3] = _nrn_mechanism_get_param_handle(prop_ion, 4); /* _ion_dicaldv */
 prop_ion = need_memb(_PKAc_sym);
 nrn_promote(prop_ion, 1, 0);
 	_ppvar[4] = _nrn_mechanism_get_param_handle(prop_ion, 1); /* PKAci */
 	_ppvar[5] = _nrn_mechanism_get_param_handle(prop_ion, 2); /* PKAco */
 
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

 extern "C" void _cal12_ms_reg() {
	int _vectorized = 1;
  _initlists();
 	ion_reg("cal", 2.0);
 	ion_reg("PKAc", 0.0);
 	_cal_sym = hoc_lookup("cal_ion");
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
                                       _nrn_mechanism_field<double>{"pbar"} /* 0 */,
                                       _nrn_mechanism_field<double>{"mod_pka_p_min"} /* 1 */,
                                       _nrn_mechanism_field<double>{"mod_pka_p_max"} /* 2 */,
                                       _nrn_mechanism_field<double>{"mod_pka_p_half"} /* 3 */,
                                       _nrn_mechanism_field<double>{"mod_pka_p_hill"} /* 4 */,
                                       _nrn_mechanism_field<double>{"ical"} /* 5 */,
                                       _nrn_mechanism_field<double>{"modulation_factor"} /* 6 */,
                                       _nrn_mechanism_field<double>{"m"} /* 7 */,
                                       _nrn_mechanism_field<double>{"h"} /* 8 */,
                                       _nrn_mechanism_field<double>{"ecal"} /* 9 */,
                                       _nrn_mechanism_field<double>{"cali"} /* 10 */,
                                       _nrn_mechanism_field<double>{"calo"} /* 11 */,
                                       _nrn_mechanism_field<double>{"minf"} /* 12 */,
                                       _nrn_mechanism_field<double>{"mtau"} /* 13 */,
                                       _nrn_mechanism_field<double>{"hinf"} /* 14 */,
                                       _nrn_mechanism_field<double>{"htau"} /* 15 */,
                                       _nrn_mechanism_field<double>{"PKAci"} /* 16 */,
                                       _nrn_mechanism_field<double>{"Dm"} /* 17 */,
                                       _nrn_mechanism_field<double>{"Dh"} /* 18 */,
                                       _nrn_mechanism_field<double>{"v"} /* 19 */,
                                       _nrn_mechanism_field<double>{"_g"} /* 20 */,
                                       _nrn_mechanism_field<double*>{"_ion_cali", "cal_ion"} /* 0 */,
                                       _nrn_mechanism_field<double*>{"_ion_calo", "cal_ion"} /* 1 */,
                                       _nrn_mechanism_field<double*>{"_ion_ical", "cal_ion"} /* 2 */,
                                       _nrn_mechanism_field<double*>{"_ion_dicaldv", "cal_ion"} /* 3 */,
                                       _nrn_mechanism_field<double*>{"_ion_PKAci", "PKAc_ion"} /* 4 */,
                                       _nrn_mechanism_field<double*>{"_ion_PKAco", "PKAc_ion"} /* 5 */,
                                       _nrn_mechanism_field<int>{"_cvode_ieq", "cvodeieq"} /* 6 */);
  hoc_register_prop_size(_mechtype, 21, 7);
  hoc_register_dparam_semantics(_mechtype, 0, "cal_ion");
  hoc_register_dparam_semantics(_mechtype, 1, "cal_ion");
  hoc_register_dparam_semantics(_mechtype, 2, "cal_ion");
  hoc_register_dparam_semantics(_mechtype, 3, "cal_ion");
  hoc_register_dparam_semantics(_mechtype, 4, "PKAc_ion");
  hoc_register_dparam_semantics(_mechtype, 5, "PKAc_ion");
  hoc_register_dparam_semantics(_mechtype, 6, "cvodeieq");
 	hoc_register_cvode(_mechtype, _ode_count, _ode_map, _ode_spec, _ode_matsol);
 	hoc_register_tolerance(_mechtype, _hoc_state_tol, &_atollist);
 
    hoc_register_var(hoc_scdoub, hoc_vdoub, hoc_intfunc);
 	ivoc_help("help ?1 cal12_ms /cfs/klemming/home/m/metog/BasalGangliaData/data/neurons/mechanisms/cal12_ms.mod\n");
 hoc_register_limits(_mechtype, _hoc_parm_limits);
 hoc_register_units(_mechtype, _hoc_parm_units);
 }
 static double FARADAY = 0x1.78e555060882cp+16;
 static double R = 0x1.0a1013e8990bep+3;
static int _reset;
static const char *modelname = "HVA L-type calcium current (Cav1.2)";

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
    minf = 1.0 / ( 1.0 + exp ( ( v - ( - 8.9 ) ) / ( - 6.7 ) ) ) ;
   mtau = 0.06 + 1.0 / ( exp ( ( v - 10.0 ) / 20.0 ) + exp ( ( v - ( - 17.0 ) ) / - 48.0 ) ) ;
   hinf = 1.0 / ( 1.0 + exp ( ( v - ( - 13.4 ) ) / 11.9 ) ) ;
   htau = 44.3 ;
     return 0; }
 
static void _hoc_rates(void) {
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
 
double ghk ( _internalthreadargsprotocomma_ double _lv , double _lci , double _lco ) {
   double _lghk;
 double _lz , _leci , _leco ;
 _lz = ( 1e-3 ) * 2.0 * FARADAY * _lv / ( R * ( celsius + 273.15 ) ) ;
   _leco = _lco * efun ( _threadargscomma_ _lz ) ;
   _leci = _lci * efun ( _threadargscomma_ - _lz ) ;
   _lghk = ( 1e-3 ) * 2.0 * FARADAY * ( _leci - _leco ) ;
   
return _lghk;
 }
 
static void _hoc_ghk(void) {
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
 _r =  ghk ( _threadargscomma_ *getarg(1) , *getarg(2) , *getarg(3) );
 hoc_retpushx(_r);
}
 
static double _npy_ghk(Prop* _prop) {
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
 _r =  ghk ( _threadargscomma_ *getarg(1) , *getarg(2) , *getarg(3) );
 return(_r);
}
 
double efun ( _internalthreadargsprotocomma_ double _lz ) {
   double _lefun;
 if ( fabs ( _lz ) < 1e-4 ) {
     _lefun = 1.0 - _lz / 2.0 ;
     }
   else {
     _lefun = _lz / ( exp ( _lz ) - 1.0 ) ;
     }
   
return _lefun;
 }
 
static void _hoc_efun(void) {
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
 _r =  efun ( _threadargscomma_ *getarg(1) );
 hoc_retpushx(_r);
}
 
static double _npy_efun(Prop* _prop) {
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
 _r =  efun ( _threadargscomma_ *getarg(1) );
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
  cali = _ion_cali;
  calo = _ion_calo;
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
  cali = _ion_cali;
  calo = _ion_calo;
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
  cali = _ion_cali;
  calo = _ion_calo;
  PKAci = _ion_PKAci;
 initmodel(_threadargs_);
 }
}

static double _nrn_current(_internalthreadargsprotocomma_ double _v) {
double _current=0.; v=_v;
{ {
   modulation_factor = hill ( _threadargscomma_ PKAci , mod_pka_p_min , mod_pka_p_max , mod_pka_p_half , mod_pka_p_hill ) ;
   ical = pbar * m * ( h * a + 1.0 - a ) * ghk ( _threadargscomma_ v , cali , calo ) * modulation_factor ;
   }
 _current += ical;

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
  cali = _ion_cali;
  calo = _ion_calo;
  PKAci = _ion_PKAci;
 auto const _g_local = _nrn_current(_threadargscomma_ _v + .001);
 	{ double _dical;
  _dical = ical;
 _rhs = _nrn_current(_threadargscomma_ _v);
  _ion_dicaldv += (_dical - ical)/.001 ;
 	}
 _g = (_g_local - _rhs)/.001;
  _ion_ical += ical ;
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
  cali = _ion_cali;
  calo = _ion_calo;
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
    const char* nmodl_filename = "/cfs/klemming/home/m/metog/BasalGangliaData/data/neurons/mechanisms/cal12_ms.mod";
    const char* nmodl_file_text = 
  "TITLE HVA L-type calcium current (Cav1.2)\n"
  "\n"
  "\n"
  "UNITS {\n"
  "    (mV) = (millivolt)\n"
  "    (mA) = (milliamp)\n"
  "    (S) = (siemens)\n"
  "    (molar) = (1/liter)\n"
  "    (mM) = (millimolar)\n"
  "    FARADAY = (faraday) (coulomb)\n"
  "    R = (k-mole) (joule/degC)\n"
  "}\n"
  "\n"
  "NEURON {\n"
  "    SUFFIX cal12_ms\n"
  "    USEION cal READ cali, calo WRITE ical VALENCE 2\n"
  "    RANGE pbar, ical\n"
  "\n"
  "    USEION PKAc READ PKAci VALENCE 0		     \n"
  "    RANGE mod_pka_p_min, mod_pka_p_max, mod_pka_p_half, mod_pka_p_hill\n"
  "    RANGE modulation_factor		     \n"
  "}\n"
  "\n"
  "PARAMETER {\n"
  "    pbar = 0.0 (cm/s)\n"
  "    a = 0.17\n"
  "    :q = 1	          : room temperature 22-25 C\n"
  "    q = 2	          : body temperature 35 C\n"
  "\n"
  "    mod_pka_p_min = 1 (1)\n"
  "    mod_pka_p_max = 1 (1)\n"
  "    mod_pka_p_half = 0.000100 (mM)\n"
  "    mod_pka_p_hill = 4 (1)\n"
  "						     \n"
  "} \n"
  "\n"
  "ASSIGNED { \n"
  "    v (mV)\n"
  "    ical (mA/cm2)\n"
  "    ecal (mV)\n"
  "    celsius (degC)\n"
  "    cali (mM)\n"
  "    calo (mM)\n"
  "    minf\n"
  "    mtau (ms)\n"
  "    hinf\n"
  "    htau (ms)\n"
  "    PKAci (mM)\n"
  "    modulation_factor (1)    \n"
  "}\n"
  "\n"
  "STATE { m h }\n"
  "\n"
  "BREAKPOINT {\n"
  "    SOLVE states METHOD cnexp\n"
  "    modulation_factor=hill(PKAci, mod_pka_p_min, mod_pka_p_max, mod_pka_p_half, mod_pka_p_hill)	   \n"
  "\n"
  "    ical = pbar*m*(h*a+1-a)*ghk(v, cali, calo)*modulation_factor\n"
  "}\n"
  "\n"
  "INITIAL {\n"
  "    rates()\n"
  "    m = minf\n"
  "    h = hinf\n"
  "}\n"
  "\n"
  "DERIVATIVE states { \n"
  "    rates()\n"
  "    m' = (minf-m)/mtau*q\n"
  "    h' = (hinf-h)/htau*q\n"
  "}\n"
  "\n"
  "PROCEDURE rates() {\n"
  "    UNITSOFF\n"
  "    minf = 1/(1+exp((v-(-8.9))/(-6.7)))\n"
  "    mtau = 0.06+1/(exp((v-10)/20)+exp((v-(-17))/-48))\n"
  "    hinf = 1/(1+exp((v-(-13.4))/11.9))\n"
  "    htau = 44.3\n"
  "    UNITSON\n"
  "}\n"
  "\n"
  "FUNCTION ghk(v (mV), ci (mM), co (mM)) (.001 coul/cm3) {\n"
  "    LOCAL z, eci, eco\n"
  "    z = (1e-3)*2*FARADAY*v/(R*(celsius+273.15))\n"
  "    eco = co*efun(z)\n"
  "    eci = ci*efun(-z)\n"
  "    ghk = (1e-3)*2*FARADAY*(eci-eco)\n"
  "}\n"
  "\n"
  "FUNCTION efun(z) {\n"
  "    if (fabs(z) < 1e-4) {\n"
  "        efun = 1-z/2\n"
  "    }else{\n"
  "        efun = z/(exp(z)-1)\n"
  "    }\n"
  "}\n"
  "\n"
  "FUNCTION hill(conc (mM),  mod_min (1), mod_max (1), half_activation (mM), hill_coefficient (1)) (1) {\n"
  "	hill = mod_min + (mod_max-mod_min) * pow(conc, hill_coefficient) / (pow(conc, hill_coefficient) + pow(half_activation, hill_coefficient))\n"
  "}\n"
  "\n"
  "\n"
  "COMMENT\n"
  "\n"
  "Activation curve was reconstructed for cultured NAc neurons from P5-P32\n"
  "Charles River rat pups [1].   Activation time constant is from the\n"
  "rodent neuron culture (both rat and mouse cells), room temperature 22-25\n"
  "C [2, Fig.15A]. Inactivation curve of CaL v1.3 current was taken from HEK\n"
  "cells [3, Fig.2 and p.819] at room temperature.\n"
  "\n"
  "Original NEURON model by Wolf (2005) [4] was modified by Alexander Kozlov\n"
  "<akozlov@csc.kth.se>. Kinetics of m1h type was used [5,6]. Activation\n"
  "time constant was refitted to avoid singularity.\n"
  "\n"
  "[1] Churchill D, Macvicar BA (1998) Biophysical and pharmacological\n"
  "characterization of voltage-dependent Ca2+ channels in neurons isolated\n"
  "from rat nucleus accumbens. J Neurophysiol 79(2):635-47.\n"
  "\n"
  "[2] Kasai H, Neher E (1992) Dihydropyridine-sensitive and\n"
  "omega-conotoxin-sensitive calcium channels in a mammalian\n"
  "neuroblastoma-glioma cell line. J Physiol 448:161-88.\n"
  "\n"
  "[3] Bell DC, Butcher AJ, Berrow NS, Page KM, Brust PF, Nesterova A,\n"
  "Stauderman KA, Seabrook GR, Nurnberg B, Dolphin AC (2001) Biophysical\n"
  "properties, pharmacology, and modulation of human, neuronal L-type\n"
  "(alpha(1D), Ca(V)1.3) voltage-dependent calcium currents. J Neurophysiol\n"
  "85:816-827.\n"
  "\n"
  "[4] Wolf JA, Moyer JT, Lazarewicz MT, Contreras D, Benoit-Marand M,\n"
  "O'Donnell P, Finkel LH (2005) NMDA/AMPA ratio impacts state transitions\n"
  "and entrainment to oscillations in a computational model of the nucleus\n"
  "accumbens medium spiny projection neuron. J Neurosci 25(40):9080-95.\n"
  "\n"
  "[5] Evans RC, Morera-Herreras T, Cui Y, Du K, Sheehan T, Kotaleski JH,\n"
  "Venance L, Blackwell KT (2012) The effects of NMDA subunit composition on\n"
  "calcium influx and spike timing-dependent plasticity in striatal medium\n"
  "spiny neurons. PLoS Comput Biol 8(4):e1002493.\n"
  "\n"
  "[6] Tuckwell HC (2012) Quantitative aspects of L-type Ca2+ currents. Prog\n"
  "Neurobiol 96(1):1-31.\n"
  "\n"
  "add justification for modulation\n"
  "\n"
  "ENDCOMMENT\n"
  ;
    hoc_reg_nmodl_filename(mech_type, nmodl_filename);
    hoc_reg_nmodl_text(mech_type, nmodl_file_text);
}
#endif
