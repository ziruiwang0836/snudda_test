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
static constexpr auto number_of_floating_point_variables = 12;
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
 
#define nrn_init _nrn_init__caldyn_ms
#define _nrn_initial _nrn_initial__caldyn_ms
#define nrn_cur _nrn_cur__caldyn_ms
#define _nrn_current _nrn_current__caldyn_ms
#define nrn_jacob _nrn_jacob__caldyn_ms
#define nrn_state _nrn_state__caldyn_ms
#define _net_receive _net_receive__caldyn_ms 
#define state state__caldyn_ms 
 
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
#define drive _ml->template fpfield<0>(_iml)
#define drive_columnindex 0
#define depth _ml->template fpfield<1>(_iml)
#define depth_columnindex 1
#define cainf _ml->template fpfield<2>(_iml)
#define cainf_columnindex 2
#define taur _ml->template fpfield<3>(_iml)
#define taur_columnindex 3
#define pump _ml->template fpfield<4>(_iml)
#define pump_columnindex 4
#define cali _ml->template fpfield<5>(_iml)
#define cali_columnindex 5
#define Dcali _ml->template fpfield<6>(_iml)
#define Dcali_columnindex 6
#define ical _ml->template fpfield<7>(_iml)
#define ical_columnindex 7
#define drive_channel _ml->template fpfield<8>(_iml)
#define drive_channel_columnindex 8
#define drive_pump _ml->template fpfield<9>(_iml)
#define drive_pump_columnindex 9
#define v _ml->template fpfield<10>(_iml)
#define v_columnindex 10
#define _g _ml->template fpfield<11>(_iml)
#define _g_columnindex 11
#define _ion_ical *(_ml->dptr_field<0>(_iml))
#define _p_ion_ical static_cast<neuron::container::data_handle<double>>(_ppvar[0])
#define _ion_cali *(_ml->dptr_field<1>(_iml))
#define _p_ion_cali static_cast<neuron::container::data_handle<double>>(_ppvar[1])
#define _ion_calo *(_ml->dptr_field<2>(_iml))
#define _p_ion_calo static_cast<neuron::container::data_handle<double>>(_ppvar[2])
#define _ion_cal_erev *_ml->dptr_field<3>(_iml)
#define _style_cal	*_ppvar[4].get<int*>()
 /* Thread safe. No static _ml, _iml or _ppvar. */
 static int hoc_nrnpointerindex =  -1;
 static _nrn_mechanism_std_vector<Datum> _extcall_thread;
 static Prop* _extcall_prop;
 /* _prop_id kind of shadows _extcall_prop to allow validity checking. */
 static _nrn_non_owning_id_without_container _prop_id{};
 /* external NEURON variables */
 /* declaration of user functions */
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
 {"setdata_caldyn_ms", _hoc_setdata},
 {0, 0}
};
 
/* Direct Python call wrappers to density mechanism functions.*/
 
static NPyDirectMechFunc npy_direct_func_proc[] = {
 {0, 0}
};
 /* declare global and static user variables */
 #define gind 0
 #define _gth 0
#define kd kd_caldyn_ms
 double kd = 0.0001;
#define kt kt_caldyn_ms
 double kt = 0.0001;
 /* some parameters have upper and lower limits */
 static HocParmLimits _hoc_parm_limits[] = {
 {0, 0, 0}
};
 static HocParmUnits _hoc_parm_units[] = {
 {"kt_caldyn_ms", "mM/ms"},
 {"kd_caldyn_ms", "mM"},
 {"drive_caldyn_ms", "1"},
 {"depth_caldyn_ms", "um"},
 {"cainf_caldyn_ms", "mM"},
 {"taur_caldyn_ms", "ms"},
 {0, 0}
};
 static double cali0 = 0;
 static double delta_t = 0.01;
 /* connect global user variables to hoc */
 static DoubScal hoc_scdoub[] = {
 {"kt_caldyn_ms", &kt_caldyn_ms},
 {"kd_caldyn_ms", &kd_caldyn_ms},
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
"caldyn_ms",
 "drive_caldyn_ms",
 "depth_caldyn_ms",
 "cainf_caldyn_ms",
 "taur_caldyn_ms",
 "pump_caldyn_ms",
 0,
 0,
 0,
 0};
 static Symbol* _cal_sym;
 
 /* Used by NrnProperty */
 static _nrn_mechanism_std_vector<double> _parm_default{
     10000, /* drive */
     0.2, /* depth */
     7e-05, /* cainf */
     43, /* taur */
     0.02, /* pump */
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
    assert(_nrn_mechanism_get_num_vars(_prop) == 12);
 	/*initialize range parameters*/
 	drive = _parm_default[0]; /* 10000 */
 	depth = _parm_default[1]; /* 0.2 */
 	cainf = _parm_default[2]; /* 7e-05 */
 	taur = _parm_default[3]; /* 43 */
 	pump = _parm_default[4]; /* 0.02 */
 	 assert(_nrn_mechanism_get_num_vars(_prop) == 12);
 	_nrn_mechanism_access_dparam(_prop) = _ppvar;
 	/*connect ionic variables to this model*/
 prop_ion = need_memb(_cal_sym);
 nrn_check_conc_write(_prop, prop_ion, 1);
 nrn_promote(prop_ion, 3, 0);
 	_ppvar[0] = _nrn_mechanism_get_param_handle(prop_ion, 3); /* ical */
 	_ppvar[1] = _nrn_mechanism_get_param_handle(prop_ion, 1); /* cali */
 	_ppvar[2] = _nrn_mechanism_get_param_handle(prop_ion, 2); /* calo */
 	_ppvar[3] = _nrn_mechanism_get_param_handle(prop_ion, 0); // erev cal
 	_ppvar[4] = {neuron::container::do_not_search, &(_nrn_mechanism_access_dparam(prop_ion)[0].literal_value<int>())}; /* iontype for cal */
 
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

 extern "C" void _caldyn_ms_reg() {
	int _vectorized = 1;
  _initlists();
 	ion_reg("cal", 2.0);
 	_cal_sym = hoc_lookup("cal_ion");
 	register_mech(_mechanism, nrn_alloc,nrn_cur, nrn_jacob, nrn_state, nrn_init, hoc_nrnpointerindex, 1);
 _mechtype = nrn_get_mechtype(_mechanism[1]);
 hoc_register_parm_default(_mechtype, &_parm_default);
         hoc_register_npy_direct(_mechtype, npy_direct_func_proc);
     _nrn_setdata_reg(_mechtype, _setdata);
 #if NMODL_TEXT
  register_nmodl_text_and_filename(_mechtype);
#endif
   _nrn_mechanism_register_data_fields(_mechtype,
                                       _nrn_mechanism_field<double>{"drive"} /* 0 */,
                                       _nrn_mechanism_field<double>{"depth"} /* 1 */,
                                       _nrn_mechanism_field<double>{"cainf"} /* 2 */,
                                       _nrn_mechanism_field<double>{"taur"} /* 3 */,
                                       _nrn_mechanism_field<double>{"pump"} /* 4 */,
                                       _nrn_mechanism_field<double>{"cali"} /* 5 */,
                                       _nrn_mechanism_field<double>{"Dcali"} /* 6 */,
                                       _nrn_mechanism_field<double>{"ical"} /* 7 */,
                                       _nrn_mechanism_field<double>{"drive_channel"} /* 8 */,
                                       _nrn_mechanism_field<double>{"drive_pump"} /* 9 */,
                                       _nrn_mechanism_field<double>{"v"} /* 10 */,
                                       _nrn_mechanism_field<double>{"_g"} /* 11 */,
                                       _nrn_mechanism_field<double*>{"_ion_ical", "cal_ion"} /* 0 */,
                                       _nrn_mechanism_field<double*>{"_ion_cali", "cal_ion"} /* 1 */,
                                       _nrn_mechanism_field<double*>{"_ion_calo", "cal_ion"} /* 2 */,
                                       _nrn_mechanism_field<double*>{"_ion_cal_erev", "cal_ion"} /* 3 */,
                                       _nrn_mechanism_field<int*>{"_style_cal", "#cal_ion"} /* 4 */,
                                       _nrn_mechanism_field<int>{"_cvode_ieq", "cvodeieq"} /* 5 */);
  hoc_register_prop_size(_mechtype, 12, 6);
  hoc_register_dparam_semantics(_mechtype, 0, "cal_ion");
  hoc_register_dparam_semantics(_mechtype, 1, "cal_ion");
  hoc_register_dparam_semantics(_mechtype, 2, "cal_ion");
  hoc_register_dparam_semantics(_mechtype, 3, "cal_ion");
  hoc_register_dparam_semantics(_mechtype, 4, "#cal_ion");
  hoc_register_dparam_semantics(_mechtype, 5, "cvodeieq");
 	nrn_writes_conc(_mechtype, 0);
 	hoc_register_cvode(_mechtype, _ode_count, _ode_map, _ode_spec, _ode_matsol);
 	hoc_register_tolerance(_mechtype, _hoc_state_tol, &_atollist);
 
    hoc_register_var(hoc_scdoub, hoc_vdoub, hoc_intfunc);
 	ivoc_help("help ?1 caldyn_ms /Users/peirui/BasalGangliaData/data/neurons/mechanisms/caldyn_ms.mod\n");
 hoc_register_limits(_mechtype, _hoc_parm_limits);
 hoc_register_units(_mechtype, _hoc_parm_units);
 }
 static double FARADAY = 0x1.78e555060882cp+16;
static int _reset;
static const char *modelname = "Calcium dynamics for L and T calcium pool";

static int error;
static int _ninits = 0;
static int _match_recurse=1;
static void _modl_cleanup(){ _match_recurse=1;}
 
static int _ode_spec1(_internalthreadargsproto_);
/*static int _ode_matsol1(_internalthreadargsproto_);*/
 static neuron::container::field_index _slist1[1], _dlist1[1];
 static int state(_internalthreadargsproto_);
 
/*CVODE*/
 static int _ode_spec1 (_internalthreadargsproto_) {int _reset = 0; {
   drive_channel = - drive * ical / ( 2.0 * FARADAY * depth ) ;
   drive_pump = - kt * ( cali - cainf ) / ( cali + kd ) ;
   if ( drive_channel <= 0. ) {
     drive_channel = 0. ;
     }
   Dcali = drive_channel + pump * drive_pump + ( cainf - cali ) / taur ;
   }
 return _reset;
}
 static int _ode_matsol1 (_internalthreadargsproto_) {
 drive_channel = - drive * ical / ( 2.0 * FARADAY * depth ) ;
 drive_pump = - kt * ( cali - cainf ) / ( cali + kd ) ;
 if ( drive_channel <= 0. ) {
   drive_channel = 0. ;
   }
 Dcali = Dcali  / (1. - dt*( ( ( ( - 1.0 ) ) ) / taur )) ;
  return 0;
}
 /*END CVODE*/
 static int state (_internalthreadargsproto_) { {
   drive_channel = - drive * ical / ( 2.0 * FARADAY * depth ) ;
   drive_pump = - kt * ( cali - cainf ) / ( cali + kd ) ;
   if ( drive_channel <= 0. ) {
     drive_channel = 0. ;
     }
    cali = cali + (1. - exp(dt*(( ( ( - 1.0 ) ) ) / taur)))*(- ( drive_channel + ( pump )*( drive_pump ) + ( ( cainf ) ) / taur ) / ( ( ( ( - 1.0 ) ) ) / taur ) - cali) ;
   }
  return 0;
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
  ical = _ion_ical;
  cali = _ion_cali;
  cali = _ion_cali;
     _ode_spec1 (_threadargs_);
  _ion_cali = cali;
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
 	_pv[0] = _p_ion_cali;
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
  ical = _ion_ical;
  cali = _ion_cali;
  cali = _ion_cali;
 _ode_matsol_instance1(_threadargs_);
 }}

static void initmodel(_internalthreadargsproto_) {
  int _i; double _save;{
 {
   cali = cainf ;
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
  ical = _ion_ical;
  cali = _ion_cali;
  cali = _ion_cali;
 initmodel(_threadargs_);
  _ion_cali = cali;
  nrn_wrote_conc(_cal_sym, _ion_cal_erev, _ion_cali, _ion_calo, _style_cal);
}
}

static double _nrn_current(_internalthreadargsprotocomma_ double _v) {
double _current=0.; v=_v;
{
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
  ical = _ion_ical;
  cali = _ion_cali;
  cali = _ion_cali;
 {   state(_threadargs_);
  } {
   }
  _ion_cali = cali;
}}

}

static void terminal(){}

static void _initlists(){
 int _i; static int _first = 1;
  if (!_first) return;
 _slist1[0] = {cali_columnindex, 0};  _dlist1[0] = {Dcali_columnindex, 0};
_first = 0;
}

#if NMODL_TEXT
static void register_nmodl_text_and_filename(int mech_type) {
    const char* nmodl_filename = "/Users/peirui/BasalGangliaData/data/neurons/mechanisms/caldyn_ms.mod";
    const char* nmodl_file_text = 
  "TITLE Calcium dynamics for L and T calcium pool\n"
  "\n"
  "NEURON {\n"
  "    SUFFIX caldyn_ms\n"
  "    USEION cal READ ical, cali WRITE cali VALENCE 2\n"
  "    RANGE pump, cainf, taur, drive, depth\n"
  "}\n"
  "\n"
  "UNITS {\n"
  "    (molar) = (1/liter) \n"
  "    (mM) = (millimolar)\n"
  "    (um) = (micron)\n"
  "    (mA) = (milliamp)\n"
  "    (msM) = (ms mM)\n"
  "    FARADAY = (faraday) (coulomb)\n"
  "}\n"
  "\n"
  "PARAMETER {\n"
  "    drive = 10000 (1)\n"
  "    depth = 0.2  (um)\n"
  "    cainf = 70e-6 (mM)\n"
  "    taur = 43 (ms)\n"
  "    kt = 1e-4 (mM/ms)\n"
  "    kd = 1e-4 (mM)\n"
  "    pump = 0.02\n"
  "}\n"
  "\n"
  "STATE { cali (mM) }\n"
  "\n"
  "INITIAL { cali = cainf }\n"
  "\n"
  "ASSIGNED {\n"
  "    ical (mA/cm2)\n"
  "    drive_channel (mM/ms)\n"
  "    drive_pump (mM/ms)\n"
  "}\n"
  "    \n"
  "BREAKPOINT {\n"
  "    SOLVE state METHOD cnexp\n"
  "}\n"
  "\n"
  "\n"
  "DERIVATIVE state { \n"
  "    \n"
  "    : force concentration to stay above cainf by only pumping if larger\n"
  "    drive_channel = -drive*ical/(2*FARADAY*depth)\n"
  "    drive_pump    = -kt*(cali-cainf)/(cali+kd)\n"
  "    \n"
  "    if (drive_channel <= 0.) { drive_channel = 0. }\n"
  "    \n"
  "    cali' = drive_channel + pump*drive_pump + (cainf-cali)/taur\n"
  "}\n"
  "\n"
  "COMMENT\n"
  "\n"
  "Original NEURON model by Wolf (2005) and Destexhe (1992).  Adaptation by\n"
  "Alexander Kozlov <akozlov@kth.se>. Updated by Robert Lindroos <robert.lindroos@ki.se>.\n"
  "\n"
  "Updates by RL:\n"
  "-cainf changed from 10 to 70 nM (sabatini et al., 2002 The Life Cycle of Ca 2+ Ions in Dendritic Spines)\n"
  "-pump updated to only be active if cai > cainf (neutralized by adding reversed entity) \n"
  "\n"
  "[1] Wolf JA, Moyer JT, Lazarewicz MT, Contreras D, Benoit-Marand M,\n"
  "O'Donnell P, Finkel LH (2005) NMDA/AMPA ratio impacts state transitions\n"
  "and entrainment to oscillations in a computational model of the nucleus\n"
  "accumbens medium spiny projection neuron. J Neurosci 25(40):9080-95.\n"
  "\n"
  "ENDCOMMENT\n"
  ;
    hoc_reg_nmodl_filename(mech_type, nmodl_filename);
    hoc_reg_nmodl_text(mech_type, nmodl_file_text);
}
#endif
