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
static constexpr auto number_of_floating_point_variables = 26;
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
 
#define nrn_init _nrn_init__kv2_ch
#define _nrn_initial _nrn_initial__kv2_ch
#define nrn_cur _nrn_cur__kv2_ch
#define _nrn_current _nrn_current__kv2_ch
#define nrn_jacob _nrn_jacob__kv2_ch
#define nrn_state _nrn_state__kv2_ch
#define _net_receive _net_receive__kv2_ch 
#define kin kin__kv2_ch 
#define rates rates__kv2_ch 
 
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
#define g _ml->template fpfield<1>(_iml)
#define g_columnindex 1
#define ik _ml->template fpfield<2>(_iml)
#define ik_columnindex 2
#define an _ml->template fpfield<3>(_iml)
#define an_columnindex 3
#define bn _ml->template fpfield<4>(_iml)
#define bn_columnindex 4
#define c1 _ml->template fpfield<5>(_iml)
#define c1_columnindex 5
#define c2 _ml->template fpfield<6>(_iml)
#define c2_columnindex 6
#define c3 _ml->template fpfield<7>(_iml)
#define c3_columnindex 7
#define c4 _ml->template fpfield<8>(_iml)
#define c4_columnindex 8
#define o _ml->template fpfield<9>(_iml)
#define o_columnindex 9
#define ek _ml->template fpfield<10>(_iml)
#define ek_columnindex 10
#define kf1 _ml->template fpfield<11>(_iml)
#define kf1_columnindex 11
#define kb1 _ml->template fpfield<12>(_iml)
#define kb1_columnindex 12
#define kf2 _ml->template fpfield<13>(_iml)
#define kf2_columnindex 13
#define kb2 _ml->template fpfield<14>(_iml)
#define kb2_columnindex 14
#define kf3 _ml->template fpfield<15>(_iml)
#define kf3_columnindex 15
#define kb3 _ml->template fpfield<16>(_iml)
#define kb3_columnindex 16
#define kf4 _ml->template fpfield<17>(_iml)
#define kf4_columnindex 17
#define kb4 _ml->template fpfield<18>(_iml)
#define kb4_columnindex 18
#define Dc1 _ml->template fpfield<19>(_iml)
#define Dc1_columnindex 19
#define Dc2 _ml->template fpfield<20>(_iml)
#define Dc2_columnindex 20
#define Dc3 _ml->template fpfield<21>(_iml)
#define Dc3_columnindex 21
#define Dc4 _ml->template fpfield<22>(_iml)
#define Dc4_columnindex 22
#define Do _ml->template fpfield<23>(_iml)
#define Do_columnindex 23
#define v _ml->template fpfield<24>(_iml)
#define v_columnindex 24
#define _g _ml->template fpfield<25>(_iml)
#define _g_columnindex 25
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
 static void _hoc_ancalc(void);
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
 {"setdata_kv2_ch", _hoc_setdata},
 {"ancalc_kv2_ch", _hoc_ancalc},
 {"rates_kv2_ch", _hoc_rates},
 {0, 0}
};
 
/* Direct Python call wrappers to density mechanism functions.*/
 static double _npy_ancalc(Prop*);
 static double _npy_rates(Prop*);
 
static NPyDirectMechFunc npy_direct_func_proc[] = {
 {"ancalc", _npy_ancalc},
 {"rates", _npy_rates},
 {0, 0}
};
#define ancalc ancalc_kv2_ch
 extern double ancalc( _internalthreadargsprotocomma_ double );
 /* declare global and static user variables */
 #define gind 0
 #define _gth 0
 /* some parameters have upper and lower limits */
 static HocParmLimits _hoc_parm_limits[] = {
 {0, 0, 0}
};
 static HocParmUnits _hoc_parm_units[] = {
 {"gbar_kv2_ch", "S/cm2"},
 {"g_kv2_ch", "S/cm2"},
 {"ik_kv2_ch", "mA/cm2"},
 {"an_kv2_ch", "1/ms"},
 {"bn_kv2_ch", "1/ms"},
 {0, 0}
};
 static double c40 = 0;
 static double c30 = 0;
 static double c20 = 0;
 static double c10 = 0;
 static double delta_t = 0.01;
 static double o0 = 0;
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
 
#define _cvode_ieq _ppvar[3].literal_value<int>()
 static void _ode_matsol_instance1(_internalthreadargsproto_);
 /* connect range variables in _p that hoc is supposed to know about */
 static const char *_mechanism[] = {
 "7.7.0",
"kv2_ch",
 "gbar_kv2_ch",
 0,
 "g_kv2_ch",
 "ik_kv2_ch",
 "an_kv2_ch",
 "bn_kv2_ch",
 0,
 "c1_kv2_ch",
 "c2_kv2_ch",
 "c3_kv2_ch",
 "c4_kv2_ch",
 "o_kv2_ch",
 0,
 0};
 static Symbol* _k_sym;
 
 /* Used by NrnProperty */
 static _nrn_mechanism_std_vector<double> _parm_default{
     1, /* gbar */
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
    assert(_nrn_mechanism_get_num_vars(_prop) == 26);
 	/*initialize range parameters*/
 	gbar = _parm_default[0]; /* 1 */
 	 assert(_nrn_mechanism_get_num_vars(_prop) == 26);
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
 static void _thread_cleanup(Datum*);
 extern Symbol* hoc_lookup(const char*);
extern void _nrn_thread_reg(int, int, void(*)(Datum*));
void _nrn_thread_table_reg(int, nrn_thread_table_check_t);
extern void hoc_register_tolerance(int, HocStateTolerance*, Symbol***);
extern void _cvode_abstol( Symbol**, double*, int);

 extern "C" void _kv2_ch_reg() {
	int _vectorized = 1;
  _initlists();
 	ion_reg("k", -10000.);
 	_k_sym = hoc_lookup("k_ion");
 	register_mech(_mechanism, nrn_alloc,nrn_cur, nrn_jacob, nrn_state, nrn_init, hoc_nrnpointerindex, 3);
  _extcall_thread.resize(2);
 _mechtype = nrn_get_mechtype(_mechanism[1]);
 hoc_register_parm_default(_mechtype, &_parm_default);
         hoc_register_npy_direct(_mechtype, npy_direct_func_proc);
     _nrn_setdata_reg(_mechtype, _setdata);
     _nrn_thread_reg(_mechtype, 0, _thread_cleanup);
 #if NMODL_TEXT
  register_nmodl_text_and_filename(_mechtype);
#endif
   _nrn_mechanism_register_data_fields(_mechtype,
                                       _nrn_mechanism_field<double>{"gbar"} /* 0 */,
                                       _nrn_mechanism_field<double>{"g"} /* 1 */,
                                       _nrn_mechanism_field<double>{"ik"} /* 2 */,
                                       _nrn_mechanism_field<double>{"an"} /* 3 */,
                                       _nrn_mechanism_field<double>{"bn"} /* 4 */,
                                       _nrn_mechanism_field<double>{"c1"} /* 5 */,
                                       _nrn_mechanism_field<double>{"c2"} /* 6 */,
                                       _nrn_mechanism_field<double>{"c3"} /* 7 */,
                                       _nrn_mechanism_field<double>{"c4"} /* 8 */,
                                       _nrn_mechanism_field<double>{"o"} /* 9 */,
                                       _nrn_mechanism_field<double>{"ek"} /* 10 */,
                                       _nrn_mechanism_field<double>{"kf1"} /* 11 */,
                                       _nrn_mechanism_field<double>{"kb1"} /* 12 */,
                                       _nrn_mechanism_field<double>{"kf2"} /* 13 */,
                                       _nrn_mechanism_field<double>{"kb2"} /* 14 */,
                                       _nrn_mechanism_field<double>{"kf3"} /* 15 */,
                                       _nrn_mechanism_field<double>{"kb3"} /* 16 */,
                                       _nrn_mechanism_field<double>{"kf4"} /* 17 */,
                                       _nrn_mechanism_field<double>{"kb4"} /* 18 */,
                                       _nrn_mechanism_field<double>{"Dc1"} /* 19 */,
                                       _nrn_mechanism_field<double>{"Dc2"} /* 20 */,
                                       _nrn_mechanism_field<double>{"Dc3"} /* 21 */,
                                       _nrn_mechanism_field<double>{"Dc4"} /* 22 */,
                                       _nrn_mechanism_field<double>{"Do"} /* 23 */,
                                       _nrn_mechanism_field<double>{"v"} /* 24 */,
                                       _nrn_mechanism_field<double>{"_g"} /* 25 */,
                                       _nrn_mechanism_field<double*>{"_ion_ek", "k_ion"} /* 0 */,
                                       _nrn_mechanism_field<double*>{"_ion_ik", "k_ion"} /* 1 */,
                                       _nrn_mechanism_field<double*>{"_ion_dikdv", "k_ion"} /* 2 */,
                                       _nrn_mechanism_field<int>{"_cvode_ieq", "cvodeieq"} /* 3 */);
  hoc_register_prop_size(_mechtype, 26, 4);
  hoc_register_dparam_semantics(_mechtype, 0, "k_ion");
  hoc_register_dparam_semantics(_mechtype, 1, "k_ion");
  hoc_register_dparam_semantics(_mechtype, 2, "k_ion");
  hoc_register_dparam_semantics(_mechtype, 3, "cvodeieq");
 	hoc_register_cvode(_mechtype, _ode_count, _ode_map, _ode_spec, _ode_matsol);
 	hoc_register_tolerance(_mechtype, _hoc_state_tol, &_atollist);
 
    hoc_register_var(hoc_scdoub, hoc_vdoub, hoc_intfunc);
 	ivoc_help("help ?1 kv2_ch /Users/peirui/BasalGangliaData/data/neurons/mechanisms/kv2_ch.mod\n");
 hoc_register_limits(_mechtype, _hoc_parm_limits);
 hoc_register_units(_mechtype, _hoc_parm_units);
 }
static int _reset;
static const char *modelname = "";

static int error;
static int _ninits = 0;
static int _match_recurse=1;
static void _modl_cleanup(){ _match_recurse=1;}
static int rates(_internalthreadargsprotocomma_ double);
 
#define _MATELM1(_row,_col) *(_nrn_thread_getelm(static_cast<SparseObj*>(_so), _row + 1, _col + 1))
 
#define _RHS1(_arg) _rhs[_arg+1]
  
#define _linmat1  1
 static int _spth1 = 1;
 static int _cvspth1 = 0;
 
static int _ode_spec1(_internalthreadargsproto_);
/*static int _ode_matsol1(_internalthreadargsproto_);*/
 static neuron::container::field_index _slist1[5], _dlist1[5]; static double *_temp1;
 static int kin (void* _so, double* _rhs, _internalthreadargsproto_);
 
static int kin (void* _so, double* _rhs, _internalthreadargsproto_)
 {int _reset=0;
 {
   double b_flux, f_flux, _term; int _i;
 {int _i; double _dt1 = 1.0/dt;
for(_i=1;_i<5;_i++){
  	_RHS1(_i) = -_dt1*(_ml->data(_iml, _slist1[_i]) - _ml->data(_iml, _dlist1[_i]));
	_MATELM1(_i, _i) = _dt1;
      
} }
 rates ( _threadargscomma_ v ) ;
   /* ~ c4 <-> c3 ( kf1 , kb1 )*/
 f_flux =  kf1 * c4 ;
 b_flux =  kb1 * c3 ;
 _RHS1( 1) -= (f_flux - b_flux);
 _RHS1( 2) += (f_flux - b_flux);
 
 _term =  kf1 ;
 _MATELM1( 1 ,1)  += _term;
 _MATELM1( 2 ,1)  -= _term;
 _term =  kb1 ;
 _MATELM1( 1 ,2)  -= _term;
 _MATELM1( 2 ,2)  += _term;
 /*REACTION*/
  /* ~ c3 <-> c2 ( kf2 , kb2 )*/
 f_flux =  kf2 * c3 ;
 b_flux =  kb2 * c2 ;
 _RHS1( 2) -= (f_flux - b_flux);
 _RHS1( 3) += (f_flux - b_flux);
 
 _term =  kf2 ;
 _MATELM1( 2 ,2)  += _term;
 _MATELM1( 3 ,2)  -= _term;
 _term =  kb2 ;
 _MATELM1( 2 ,3)  -= _term;
 _MATELM1( 3 ,3)  += _term;
 /*REACTION*/
  /* ~ c2 <-> c1 ( kf3 , kb3 )*/
 f_flux =  kf3 * c2 ;
 b_flux =  kb3 * c1 ;
 _RHS1( 3) -= (f_flux - b_flux);
 _RHS1( 4) += (f_flux - b_flux);
 
 _term =  kf3 ;
 _MATELM1( 3 ,3)  += _term;
 _MATELM1( 4 ,3)  -= _term;
 _term =  kb3 ;
 _MATELM1( 3 ,4)  -= _term;
 _MATELM1( 4 ,4)  += _term;
 /*REACTION*/
  /* ~ c1 <-> o ( kf4 , kb4 )*/
 f_flux =  kf4 * c1 ;
 b_flux =  kb4 * o ;
 _RHS1( 4) -= (f_flux - b_flux);
 
 _term =  kf4 ;
 _MATELM1( 4 ,4)  += _term;
 _term =  kb4 ;
 _MATELM1( 4 ,0)  -= _term;
 /*REACTION*/
   /* c4 + c3 + c2 + c1 + o = 1.0 */
 _RHS1(0) =  1.0;
 _MATELM1(0, 0) = 1;
 _RHS1(0) -= o ;
 _MATELM1(0, 4) = 1;
 _RHS1(0) -= c1 ;
 _MATELM1(0, 3) = 1;
 _RHS1(0) -= c2 ;
 _MATELM1(0, 2) = 1;
 _RHS1(0) -= c3 ;
 _MATELM1(0, 1) = 1;
 _RHS1(0) -= c4 ;
 /*CONSERVATION*/
   } return _reset;
 }
 
static int  rates ( _internalthreadargsprotocomma_ double _lv ) {
   an = ancalc ( _threadargscomma_ _lv ) ;
   bn = ( 0.0051 ) / exp ( _lv / 22.02 ) ;
   kf1 = 4.0 * an ;
   kb1 = bn ;
   kf2 = 3.0 * an ;
   kb2 = 2.0 * bn ;
   kf3 = 2.0 * an ;
   kb3 = 3.0 * bn ;
   kf4 = an ;
   kb4 = 4.0 * bn ;
    return 0; }
 
static void _hoc_rates(void) {
  double _r;
 Datum* _ppvar; Datum* _thread; NrnThread* _nt;
 
  if(!_prop_id) {
    hoc_execerror("No data for rates_kv2_ch. Requires prior call to setdata_kv2_ch and that the specified mechanism instance still be in existence.", NULL);
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
 rates ( _threadargscomma_ *getarg(1) );
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
 rates ( _threadargscomma_ *getarg(1) );
 return(_r);
}
 
double ancalc ( _internalthreadargsprotocomma_ double _lvx ) {
   double _lancalc;
  if ( 80.0 < _lvx  && _lvx < 90.0 ) {
     _lancalc = 0.004186 * pow( _lvx , 2.0 ) - 0.40239 * _lvx + 11.34 ;
     }
   else {
     _lancalc = ( 51.743 - 0.612 * _lvx ) / ( exp ( ( - 84.54 + _lvx ) / - 11.84 ) - 1.0 ) ;
     }
    
return _lancalc;
 }
 
static void _hoc_ancalc(void) {
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
 _r =  ancalc ( _threadargscomma_ *getarg(1) );
 hoc_retpushx(_r);
}
 
static double _npy_ancalc(Prop* _prop) {
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
 _r =  ancalc ( _threadargscomma_ *getarg(1) );
 return(_r);
}
 
/*CVODE ode begin*/
 static int _ode_spec1(_internalthreadargsproto_) {
  int _reset=0;
  {
 double b_flux, f_flux, _term; int _i;
 {int _i; for(_i=0;_i<5;_i++) _ml->data(_iml, _dlist1[_i]) = 0.0;}
 rates ( _threadargscomma_ v ) ;
 /* ~ c4 <-> c3 ( kf1 , kb1 )*/
 f_flux =  kf1 * c4 ;
 b_flux =  kb1 * c3 ;
 Dc4 -= (f_flux - b_flux);
 Dc3 += (f_flux - b_flux);
 
 /*REACTION*/
  /* ~ c3 <-> c2 ( kf2 , kb2 )*/
 f_flux =  kf2 * c3 ;
 b_flux =  kb2 * c2 ;
 Dc3 -= (f_flux - b_flux);
 Dc2 += (f_flux - b_flux);
 
 /*REACTION*/
  /* ~ c2 <-> c1 ( kf3 , kb3 )*/
 f_flux =  kf3 * c2 ;
 b_flux =  kb3 * c1 ;
 Dc2 -= (f_flux - b_flux);
 Dc1 += (f_flux - b_flux);
 
 /*REACTION*/
  /* ~ c1 <-> o ( kf4 , kb4 )*/
 f_flux =  kf4 * c1 ;
 b_flux =  kb4 * o ;
 Dc1 -= (f_flux - b_flux);
 Do += (f_flux - b_flux);
 
 /*REACTION*/
   /* c4 + c3 + c2 + c1 + o = 1.0 */
 /*CONSERVATION*/
   } return _reset;
 }
 
/*CVODE matsol*/
 static int _ode_matsol1(void* _so, double* _rhs, _internalthreadargsproto_) {int _reset=0;{
 double b_flux, f_flux, _term; int _i;
   b_flux = f_flux = 0.;
 {int _i; double _dt1 = 1.0/dt;
for(_i=0;_i<5;_i++){
  	_RHS1(_i) = _dt1*(_ml->data(_iml, _dlist1[_i]));
	_MATELM1(_i, _i) = _dt1;
      
} }
 rates ( _threadargscomma_ v ) ;
 /* ~ c4 <-> c3 ( kf1 , kb1 )*/
 _term =  kf1 ;
 _MATELM1( 1 ,1)  += _term;
 _MATELM1( 2 ,1)  -= _term;
 _term =  kb1 ;
 _MATELM1( 1 ,2)  -= _term;
 _MATELM1( 2 ,2)  += _term;
 /*REACTION*/
  /* ~ c3 <-> c2 ( kf2 , kb2 )*/
 _term =  kf2 ;
 _MATELM1( 2 ,2)  += _term;
 _MATELM1( 3 ,2)  -= _term;
 _term =  kb2 ;
 _MATELM1( 2 ,3)  -= _term;
 _MATELM1( 3 ,3)  += _term;
 /*REACTION*/
  /* ~ c2 <-> c1 ( kf3 , kb3 )*/
 _term =  kf3 ;
 _MATELM1( 3 ,3)  += _term;
 _MATELM1( 4 ,3)  -= _term;
 _term =  kb3 ;
 _MATELM1( 3 ,4)  -= _term;
 _MATELM1( 4 ,4)  += _term;
 /*REACTION*/
  /* ~ c1 <-> o ( kf4 , kb4 )*/
 _term =  kf4 ;
 _MATELM1( 4 ,4)  += _term;
 _MATELM1( 0 ,4)  -= _term;
 _term =  kb4 ;
 _MATELM1( 4 ,0)  -= _term;
 _MATELM1( 0 ,0)  += _term;
 /*REACTION*/
   /* c4 + c3 + c2 + c1 + o = 1.0 */
 /*CONSERVATION*/
   } return _reset;
 }
 
/*CVODE end*/
 
static int _ode_count(int _type){ return 5;}
 
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
  for (int _i=0; _i < 5; ++_i) {
    _pv[_i] = _nrn_mechanism_get_param_handle(_prop, _slist1[_i]);
    _pvdot[_i] = _nrn_mechanism_get_param_handle(_prop, _dlist1[_i]);
    _cvode_abstol(_atollist, _atol, _i);
  }
 }
 
static void _ode_matsol_instance1(_internalthreadargsproto_) {
 _cvode_sparse_thread(&(_thread[_cvspth1].literal_value<void*>()), 5, _dlist1, neuron::scopmath::row_view{_ml, _iml}, _ode_matsol1, _threadargs_);
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
 
static void _thread_cleanup(Datum* _thread) {
   _nrn_destroy_sparseobj_thread(static_cast<SparseObj*>(_thread[_cvspth1].get<void*>()));
   _nrn_destroy_sparseobj_thread(static_cast<SparseObj*>(_thread[_spth1].get<void*>()));
 }

static void initmodel(_internalthreadargsproto_) {
  int _i; double _save;{
  c4 = c40;
  c3 = c30;
  c2 = c20;
  c1 = c10;
  o = o0;
 {
    _ss_sparse_thread(&(_thread[_spth1].literal_value<void*>()), 5, _slist1, _dlist1, neuron::scopmath::row_view{_ml, _iml}, &t, dt, kin, _linmat1, _threadargs_);
     if (secondorder) {
    int _i;
    for (_i = 0; _i < 5; ++_i) {
      _ml->data(_iml, _slist1[_i]) += dt*_ml->data(_iml, _dlist1[_i]);
    }}
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
 initmodel(_threadargs_);
 }
}

static double _nrn_current(_internalthreadargsprotocomma_ double _v) {
double _current=0.; v=_v;
{ {
   g = gbar * o ;
   ik = g * ( v - ek ) ;
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
double _dtsav = dt;
if (secondorder) { dt *= 0.5; }
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
 {  sparse_thread(&(_thread[_spth1].literal_value<void*>()), 5, _slist1, _dlist1, neuron::scopmath::row_view{_ml, _iml}, &t, dt, kin, _linmat1, _threadargs_);
     if (secondorder) {
    int _i;
    for (_i = 0; _i < 5; ++_i) {
      _ml->data(_iml, _slist1[_i]) += dt*_ml->data(_iml, _dlist1[_i]);
    }}
 } }}
 dt = _dtsav;
}

static void terminal(){}

static void _initlists(){
 int _i; static int _first = 1;
  if (!_first) return;
 _slist1[0] = {o_columnindex, 0};  _dlist1[0] = {Do_columnindex, 0};
 _slist1[1] = {c4_columnindex, 0};  _dlist1[1] = {Dc4_columnindex, 0};
 _slist1[2] = {c3_columnindex, 0};  _dlist1[2] = {Dc3_columnindex, 0};
 _slist1[3] = {c2_columnindex, 0};  _dlist1[3] = {Dc2_columnindex, 0};
 _slist1[4] = {c1_columnindex, 0};  _dlist1[4] = {Dc1_columnindex, 0};
_first = 0;
}

#if NMODL_TEXT
static void register_nmodl_text_and_filename(int mech_type) {
    const char* nmodl_filename = "/Users/peirui/BasalGangliaData/data/neurons/mechanisms/kv2_ch.mod";
    const char* nmodl_file_text = 
  ": KV2_CH.MOD\n"
  ": KV2 for Cholinergic Interneuron\n"
  "\n"
  "NEURON {\n"
  "	SUFFIX kv2_ch\n"
  "	USEION k READ ek WRITE ik\n"
  "	RANGE g, ik, an, bn, gbar\n"
  "}\n"
  "\n"
  "UNITS {\n"
  "	(mV) = (millivolt)\n"
  "	(S) = (siemens)\n"
  "	(mA) = (milliamp)\n"
  "}\n"
  "\n"
  "PARAMETER {\n"
  "	gbar = 1	(S/cm2)\n"
  "}\n"
  "\n"
  "ASSIGNED {\n"
  "	v	(mV)\n"
  "	ek	(mV)\n"
  "	g	(S/cm2)\n"
  "	ik	(mA/cm2)\n"
  "\n"
  "	an	(1/ms)\n"
  "	bn	(1/ms)\n"
  "	kf1	(1/ms)\n"
  "	kb1	(1/ms)\n"
  "	kf2	(1/ms)\n"
  "	kb2	(1/ms)\n"
  "	kf3	(1/ms)\n"
  "	kb3	(1/ms)\n"
  "	kf4	(1/ms)\n"
  "	kb4	(1/ms)\n"
  "}\n"
  "\n"
  "STATE {\n"
  "	c1\n"
  "	c2\n"
  "	c3\n"
  "	c4\n"
  "	o\n"
  "}\n"
  "\n"
  "BREAKPOINT {\n"
  "	SOLVE kin METHOD sparse\n"
  "	g = gbar*o\n"
  "	ik = g*(v-ek)\n"
  "}\n"
  "\n"
  "INITIAL {\n"
  "	SOLVE kin STEADYSTATE sparse\n"
  "}\n"
  "\n"
  "\n"
  "\n"
  "KINETIC kin{\n"
  "	rates(v)\n"
  "	~ c4 <-> c3     (kf1,kb1)\n"
  "	~ c3 <-> c2     (kf2,kb2)\n"
  "	~ c2 <-> c1     (kf3,kb3)\n"
  "	~ c1 <-> o      (kf4,kb4)\n"
  "	CONSERVE c4+c3+c2+c1+o=1\n"
  "}\n"
  "\n"
  "\n"
  "PROCEDURE rates(v(millivolt)) {\n"
  "		    : an = (51.743 (1/ms) - (0.612 (1/ms-mV))*v)/(exp((-84.54 (mV)+v)/-11.84 (mV)) - 1)\n"
  "        an = ancalc(v)								      \n"
  "	bn = (0.0051 (1/ms) )/exp(v/22.02 (mV))\n"
  "\n"
  "	kf1 = 4*an\n"
  "	kb1 = bn\n"
  "	kf2 = 3*an\n"
  "	kb2 = 2*bn\n"
  "	kf3 = 2*an\n"
  "	kb3 = 3*bn\n"
  "	kf4 = an\n"
  "	kb4 = 4*bn\n"
  "}\n"
  "\n"
  "FUNCTION ancalc(vx(millivolt)) {\n"
  "     UNITSOFF\n"
  "     if(80 < vx && vx < 90 ) {\n"
  "          ancalc = 0.004186*vx^2  -0.40239*vx + 11.34  \n"
  "       }\n"
  "     else {\n"
  "	  ancalc = (51.743 - 0.612*vx)/(exp((-84.54 +vx)/-11.84 ) - 1)     \n"
  "     }\n"
  "     UNITSON\n"
  "}     \n"
  ;
    hoc_reg_nmodl_filename(mech_type, nmodl_filename);
    hoc_reg_nmodl_text(mech_type, nmodl_file_text);
}
#endif
