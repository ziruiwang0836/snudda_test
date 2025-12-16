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
static constexpr auto number_of_datum_variables = 5;
static constexpr auto number_of_floating_point_variables = 33;
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
 
#define nrn_init _nrn_init__kv4_ch
#define _nrn_initial _nrn_initial__kv4_ch
#define nrn_cur _nrn_cur__kv4_ch
#define _nrn_current _nrn_current__kv4_ch
#define nrn_jacob _nrn_jacob__kv4_ch
#define nrn_state _nrn_state__kv4_ch
#define _net_receive _net_receive__kv4_ch 
#define kin kin__kv4_ch 
#define rates rates__kv4_ch 
 
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
#define gbar _ml->template fpfield<0>(_iml)
#define gbar_columnindex 0
#define mod_pka_g_min _ml->template fpfield<1>(_iml)
#define mod_pka_g_min_columnindex 1
#define mod_pka_g_max _ml->template fpfield<2>(_iml)
#define mod_pka_g_max_columnindex 2
#define mod_pka_g_half _ml->template fpfield<3>(_iml)
#define mod_pka_g_half_columnindex 3
#define mod_pka_g_slope _ml->template fpfield<4>(_iml)
#define mod_pka_g_slope_columnindex 4
#define g _ml->template fpfield<5>(_iml)
#define g_columnindex 5
#define ik _ml->template fpfield<6>(_iml)
#define ik_columnindex 6
#define modulation_factor _ml->template fpfield<7>(_iml)
#define modulation_factor_columnindex 7
#define c1 _ml->template fpfield<8>(_iml)
#define c1_columnindex 8
#define c2 _ml->template fpfield<9>(_iml)
#define c2_columnindex 9
#define c3 _ml->template fpfield<10>(_iml)
#define c3_columnindex 10
#define c4 _ml->template fpfield<11>(_iml)
#define c4_columnindex 11
#define o _ml->template fpfield<12>(_iml)
#define o_columnindex 12
#define i1 _ml->template fpfield<13>(_iml)
#define i1_columnindex 13
#define i2 _ml->template fpfield<14>(_iml)
#define i2_columnindex 14
#define i3 _ml->template fpfield<15>(_iml)
#define i3_columnindex 15
#define i4 _ml->template fpfield<16>(_iml)
#define i4_columnindex 16
#define i5 _ml->template fpfield<17>(_iml)
#define i5_columnindex 17
#define is _ml->template fpfield<18>(_iml)
#define is_columnindex 18
#define ek _ml->template fpfield<19>(_iml)
#define ek_columnindex 19
#define PKAci _ml->template fpfield<20>(_iml)
#define PKAci_columnindex 20
#define Dc1 _ml->template fpfield<21>(_iml)
#define Dc1_columnindex 21
#define Dc2 _ml->template fpfield<22>(_iml)
#define Dc2_columnindex 22
#define Dc3 _ml->template fpfield<23>(_iml)
#define Dc3_columnindex 23
#define Dc4 _ml->template fpfield<24>(_iml)
#define Dc4_columnindex 24
#define Do _ml->template fpfield<25>(_iml)
#define Do_columnindex 25
#define Di1 _ml->template fpfield<26>(_iml)
#define Di1_columnindex 26
#define Di2 _ml->template fpfield<27>(_iml)
#define Di2_columnindex 27
#define Di3 _ml->template fpfield<28>(_iml)
#define Di3_columnindex 28
#define Di4 _ml->template fpfield<29>(_iml)
#define Di4_columnindex 29
#define Di5 _ml->template fpfield<30>(_iml)
#define Di5_columnindex 30
#define Dis _ml->template fpfield<31>(_iml)
#define Dis_columnindex 31
#define _g _ml->template fpfield<32>(_iml)
#define _g_columnindex 32
#define _ion_ek *(_ml->dptr_field<0>(_iml))
#define _p_ion_ek static_cast<neuron::container::data_handle<double>>(_ppvar[0])
#define _ion_ik *(_ml->dptr_field<1>(_iml))
#define _p_ion_ik static_cast<neuron::container::data_handle<double>>(_ppvar[1])
#define _ion_dikdv *(_ml->dptr_field<2>(_iml))
#define _ion_PKAci *(_ml->dptr_field<3>(_iml))
#define _p_ion_PKAci static_cast<neuron::container::data_handle<double>>(_ppvar[3])
#define _ion_PKAco *(_ml->dptr_field<4>(_iml))
#define _p_ion_PKAco static_cast<neuron::container::data_handle<double>>(_ppvar[4])
 static _nrn_mechanism_cache_instance _ml_real{nullptr};
static _nrn_mechanism_cache_range *_ml{&_ml_real};
static size_t _iml{0};
static Datum *_ppvar;
 static int hoc_nrnpointerindex =  -1;
 static Prop* _extcall_prop;
 /* _prop_id kind of shadows _extcall_prop to allow validity checking. */
 static _nrn_non_owning_id_without_container _prop_id{};
 /* external NEURON variables */
 extern double celsius;
 /* declaration of user functions */
 static void _hoc_modulation(void);
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
 {"setdata_kv4_ch", _hoc_setdata},
 {"modulation_kv4_ch", _hoc_modulation},
 {"rates_kv4_ch", _hoc_rates},
 {0, 0}
};
 
/* Direct Python call wrappers to density mechanism functions.*/
 static double _npy_modulation(Prop*);
 static double _npy_rates(Prop*);
 
static NPyDirectMechFunc npy_direct_func_proc[] = {
 {"modulation", _npy_modulation},
 {"rates", _npy_rates},
 {0, 0}
};
#define modulation modulation_kv4_ch
 extern double modulation( double , double , double , double , double );
 /* declare global and static user variables */
 #define gind 0
 #define _gth 0
#define am am_kv4_ch
 double am = 1;
#define a a_kv4_ch
 double a = 3;
#define alpha alpha_kv4_ch
 double alpha = 0;
#define bm bm_kv4_ch
 double bm = 7;
#define b b_kv4_ch
 double b = 40;
#define beta beta_kv4_ch
 double beta = 0;
#define ci ci_kv4_ch
 double ci = 0.2;
#define delta delta_kv4_ch
 double delta = 4;
#define gamma gamma_kv4_ch
 double gamma = 200;
#define isi5 isi5_kv4_ch
 double isi5 = 0.001;
#define i5is i5is_kv4_ch
 double i5is = 0.001;
#define io io_kv4_ch
 double io = 0.01;
#define ic ic_kv4_ch
 double ic = 500;
#define oi oi_kv4_ch
 double oi = 1e-09;
#define q10v q10v_kv4_ch
 double q10v = 3;
#define q10i q10i_kv4_ch
 double q10i = 3;
#define vhb vhb_kv4_ch
 double vhb = -30;
#define vha vha_kv4_ch
 double vha = -75;
#define vc vc_kv4_ch
 double vc = 10;
 /* some parameters have upper and lower limits */
 static HocParmLimits _hoc_parm_limits[] = {
 {0, 0, 0}
};
 static HocParmUnits _hoc_parm_units[] = {
 {"gamma_kv4_ch", "1/ms"},
 {"delta_kv4_ch", "1/ms"},
 {"ic_kv4_ch", "/ms"},
 {"oi_kv4_ch", "/ms"},
 {"io_kv4_ch", "/ms"},
 {"ci_kv4_ch", "/ms"},
 {"am_kv4_ch", "1/ms"},
 {"bm_kv4_ch", "1/ms"},
 {"vc_kv4_ch", "mV"},
 {"vha_kv4_ch", "mV"},
 {"vhb_kv4_ch", "mV"},
 {"i5is_kv4_ch", "/ms"},
 {"isi5_kv4_ch", "/ms"},
 {"alpha_kv4_ch", "/ms"},
 {"beta_kv4_ch", "/ms"},
 {"gbar_kv4_ch", "S/cm2"},
 {"mod_pka_g_min_kv4_ch", "1"},
 {"mod_pka_g_max_kv4_ch", "1"},
 {"mod_pka_g_half_kv4_ch", "mM"},
 {"mod_pka_g_slope_kv4_ch", "mM"},
 {"g_kv4_ch", "S/cm2"},
 {"ik_kv4_ch", "mA/cm2"},
 {"modulation_factor_kv4_ch", "1"},
 {0, 0}
};
 static double c40 = 0;
 static double c30 = 0;
 static double c20 = 0;
 static double c10 = 0;
 static double delta_t = 0.01;
 static double is0 = 0;
 static double i50 = 0;
 static double i40 = 0;
 static double i30 = 0;
 static double i20 = 0;
 static double i10 = 0;
 static double o0 = 0;
 static double v = 0;
 /* connect global user variables to hoc */
 static DoubScal hoc_scdoub[] = {
 {"gamma_kv4_ch", &gamma_kv4_ch},
 {"delta_kv4_ch", &delta_kv4_ch},
 {"a_kv4_ch", &a_kv4_ch},
 {"b_kv4_ch", &b_kv4_ch},
 {"ic_kv4_ch", &ic_kv4_ch},
 {"oi_kv4_ch", &oi_kv4_ch},
 {"io_kv4_ch", &io_kv4_ch},
 {"ci_kv4_ch", &ci_kv4_ch},
 {"am_kv4_ch", &am_kv4_ch},
 {"bm_kv4_ch", &bm_kv4_ch},
 {"vc_kv4_ch", &vc_kv4_ch},
 {"vha_kv4_ch", &vha_kv4_ch},
 {"vhb_kv4_ch", &vhb_kv4_ch},
 {"i5is_kv4_ch", &i5is_kv4_ch},
 {"isi5_kv4_ch", &isi5_kv4_ch},
 {"q10i_kv4_ch", &q10i_kv4_ch},
 {"q10v_kv4_ch", &q10v_kv4_ch},
 {"alpha_kv4_ch", &alpha_kv4_ch},
 {"beta_kv4_ch", &beta_kv4_ch},
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
 neuron::legacy::set_globals_from_prop(_prop, _ml_real, _ml, _iml);
_ppvar = _nrn_mechanism_access_dparam(_prop);
 Node * _node = _nrn_mechanism_access_node(_prop);
v = _nrn_mechanism_access_voltage(_node);
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
"kv4_ch",
 "gbar_kv4_ch",
 "mod_pka_g_min_kv4_ch",
 "mod_pka_g_max_kv4_ch",
 "mod_pka_g_half_kv4_ch",
 "mod_pka_g_slope_kv4_ch",
 0,
 "g_kv4_ch",
 "ik_kv4_ch",
 "modulation_factor_kv4_ch",
 0,
 "c1_kv4_ch",
 "c2_kv4_ch",
 "c3_kv4_ch",
 "c4_kv4_ch",
 "o_kv4_ch",
 "i1_kv4_ch",
 "i2_kv4_ch",
 "i3_kv4_ch",
 "i4_kv4_ch",
 "i5_kv4_ch",
 "is_kv4_ch",
 0,
 0};
 static Symbol* _k_sym;
 static Symbol* _PKAc_sym;
 
 /* Used by NrnProperty */
 static _nrn_mechanism_std_vector<double> _parm_default{
     1, /* gbar */
     1, /* mod_pka_g_min */
     1, /* mod_pka_g_max */
     0.0001, /* mod_pka_g_half */
     0.01, /* mod_pka_g_slope */
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
    assert(_nrn_mechanism_get_num_vars(_prop) == 33);
 	/*initialize range parameters*/
 	gbar = _parm_default[0]; /* 1 */
 	mod_pka_g_min = _parm_default[1]; /* 1 */
 	mod_pka_g_max = _parm_default[2]; /* 1 */
 	mod_pka_g_half = _parm_default[3]; /* 0.0001 */
 	mod_pka_g_slope = _parm_default[4]; /* 0.01 */
 	 assert(_nrn_mechanism_get_num_vars(_prop) == 33);
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

 extern "C" void _kv4_ch_reg() {
	int _vectorized = 0;
  _initlists();
 	ion_reg("k", -10000.);
 	ion_reg("PKAc", 0.0);
 	_k_sym = hoc_lookup("k_ion");
 	_PKAc_sym = hoc_lookup("PKAc_ion");
 	register_mech(_mechanism, nrn_alloc,nrn_cur, nrn_jacob, nrn_state, nrn_init, hoc_nrnpointerindex, 0);
 _mechtype = nrn_get_mechtype(_mechanism[1]);
 hoc_register_parm_default(_mechtype, &_parm_default);
         hoc_register_npy_direct(_mechtype, npy_direct_func_proc);
     _nrn_setdata_reg(_mechtype, _setdata);
 #if NMODL_TEXT
  register_nmodl_text_and_filename(_mechtype);
#endif
   _nrn_mechanism_register_data_fields(_mechtype,
                                       _nrn_mechanism_field<double>{"gbar"} /* 0 */,
                                       _nrn_mechanism_field<double>{"mod_pka_g_min"} /* 1 */,
                                       _nrn_mechanism_field<double>{"mod_pka_g_max"} /* 2 */,
                                       _nrn_mechanism_field<double>{"mod_pka_g_half"} /* 3 */,
                                       _nrn_mechanism_field<double>{"mod_pka_g_slope"} /* 4 */,
                                       _nrn_mechanism_field<double>{"g"} /* 5 */,
                                       _nrn_mechanism_field<double>{"ik"} /* 6 */,
                                       _nrn_mechanism_field<double>{"modulation_factor"} /* 7 */,
                                       _nrn_mechanism_field<double>{"c1"} /* 8 */,
                                       _nrn_mechanism_field<double>{"c2"} /* 9 */,
                                       _nrn_mechanism_field<double>{"c3"} /* 10 */,
                                       _nrn_mechanism_field<double>{"c4"} /* 11 */,
                                       _nrn_mechanism_field<double>{"o"} /* 12 */,
                                       _nrn_mechanism_field<double>{"i1"} /* 13 */,
                                       _nrn_mechanism_field<double>{"i2"} /* 14 */,
                                       _nrn_mechanism_field<double>{"i3"} /* 15 */,
                                       _nrn_mechanism_field<double>{"i4"} /* 16 */,
                                       _nrn_mechanism_field<double>{"i5"} /* 17 */,
                                       _nrn_mechanism_field<double>{"is"} /* 18 */,
                                       _nrn_mechanism_field<double>{"ek"} /* 19 */,
                                       _nrn_mechanism_field<double>{"PKAci"} /* 20 */,
                                       _nrn_mechanism_field<double>{"Dc1"} /* 21 */,
                                       _nrn_mechanism_field<double>{"Dc2"} /* 22 */,
                                       _nrn_mechanism_field<double>{"Dc3"} /* 23 */,
                                       _nrn_mechanism_field<double>{"Dc4"} /* 24 */,
                                       _nrn_mechanism_field<double>{"Do"} /* 25 */,
                                       _nrn_mechanism_field<double>{"Di1"} /* 26 */,
                                       _nrn_mechanism_field<double>{"Di2"} /* 27 */,
                                       _nrn_mechanism_field<double>{"Di3"} /* 28 */,
                                       _nrn_mechanism_field<double>{"Di4"} /* 29 */,
                                       _nrn_mechanism_field<double>{"Di5"} /* 30 */,
                                       _nrn_mechanism_field<double>{"Dis"} /* 31 */,
                                       _nrn_mechanism_field<double>{"_g"} /* 32 */,
                                       _nrn_mechanism_field<double*>{"_ion_ek", "k_ion"} /* 0 */,
                                       _nrn_mechanism_field<double*>{"_ion_ik", "k_ion"} /* 1 */,
                                       _nrn_mechanism_field<double*>{"_ion_dikdv", "k_ion"} /* 2 */,
                                       _nrn_mechanism_field<double*>{"_ion_PKAci", "PKAc_ion"} /* 3 */,
                                       _nrn_mechanism_field<double*>{"_ion_PKAco", "PKAc_ion"} /* 4 */,
                                       _nrn_mechanism_field<int>{"_cvode_ieq", "cvodeieq"} /* 5 */);
  hoc_register_prop_size(_mechtype, 33, 6);
  hoc_register_dparam_semantics(_mechtype, 0, "k_ion");
  hoc_register_dparam_semantics(_mechtype, 1, "k_ion");
  hoc_register_dparam_semantics(_mechtype, 2, "k_ion");
  hoc_register_dparam_semantics(_mechtype, 3, "PKAc_ion");
  hoc_register_dparam_semantics(_mechtype, 4, "PKAc_ion");
  hoc_register_dparam_semantics(_mechtype, 5, "cvodeieq");
 	hoc_register_cvode(_mechtype, _ode_count, _ode_map, _ode_spec, _ode_matsol);
 	hoc_register_tolerance(_mechtype, _hoc_state_tol, &_atollist);
 
    hoc_register_var(hoc_scdoub, hoc_vdoub, hoc_intfunc);
 	ivoc_help("help ?1 kv4_ch /cfs/klemming/home/m/metog/BasalGangliaData/data/neurons/mechanisms/kv4_ch.mod\n");
 hoc_register_limits(_mechtype, _hoc_parm_limits);
 hoc_register_units(_mechtype, _hoc_parm_units);
 }
static int _reset;
static const char *modelname = "";

static int error;
static int _ninits = 0;
static int _match_recurse=1;
static void _modl_cleanup(){ _match_recurse=1;}
static int rates(double);
 
#define _MATELM1(_row,_col)	*(_getelm(_row + 1, _col + 1))
 
#define _RHS1(_arg) _coef1[_arg + 1]
 static double *_coef1;
 
#define _linmat1  1
 static void* _sparseobj1;
 static void* _cvsparseobj1;
 
static int _ode_spec1(_internalthreadargsproto_);
/*static int _ode_matsol1(_internalthreadargsproto_);*/
 static neuron::container::field_index _slist1[11], _dlist1[11]; static double *_temp1;
 static int kin ();
 
static int kin ()
 {_reset=0;
 {
   double _lq10 ;
 double b_flux, f_flux, _term; int _i;
 {int _i; double _dt1 = 1.0/dt;
for(_i=1;_i<11;_i++){
  	_RHS1(_i) = -_dt1*(_ml->data(_iml, _slist1[_i]) - _ml->data(_iml, _dlist1[_i]));
	_MATELM1(_i, _i) = _dt1;
      
} }
 _lq10 = pow( q10i , ( ( celsius - 22.0 ) / 10.0 ) ) ;
   rates ( _threadargscomma_ v ) ;
   /* ~ c1 <-> c2 ( 3.0 * alpha , beta )*/
 f_flux =  3.0 * alpha * c1 ;
 b_flux =  beta * c2 ;
 _RHS1( 4) -= (f_flux - b_flux);
 _RHS1( 3) += (f_flux - b_flux);
 
 _term =  3.0 * alpha ;
 _MATELM1( 4 ,4)  += _term;
 _MATELM1( 3 ,4)  -= _term;
 _term =  beta ;
 _MATELM1( 4 ,3)  -= _term;
 _MATELM1( 3 ,3)  += _term;
 /*REACTION*/
  /* ~ c2 <-> c3 ( 2.0 * alpha , 2.0 * beta )*/
 f_flux =  2.0 * alpha * c2 ;
 b_flux =  2.0 * beta * c3 ;
 _RHS1( 3) -= (f_flux - b_flux);
 _RHS1( 2) += (f_flux - b_flux);
 
 _term =  2.0 * alpha ;
 _MATELM1( 3 ,3)  += _term;
 _MATELM1( 2 ,3)  -= _term;
 _term =  2.0 * beta ;
 _MATELM1( 3 ,2)  -= _term;
 _MATELM1( 2 ,2)  += _term;
 /*REACTION*/
  /* ~ c3 <-> c4 ( alpha , 3.0 * beta )*/
 f_flux =  alpha * c3 ;
 b_flux =  3.0 * beta * c4 ;
 _RHS1( 2) -= (f_flux - b_flux);
 _RHS1( 1) += (f_flux - b_flux);
 
 _term =  alpha ;
 _MATELM1( 2 ,2)  += _term;
 _MATELM1( 1 ,2)  -= _term;
 _term =  3.0 * beta ;
 _MATELM1( 2 ,1)  -= _term;
 _MATELM1( 1 ,1)  += _term;
 /*REACTION*/
  /* ~ c4 <-> o ( _lq10 * gamma , _lq10 * delta )*/
 f_flux =  _lq10 * gamma * c4 ;
 b_flux =  _lq10 * delta * o ;
 _RHS1( 1) -= (f_flux - b_flux);
 _RHS1( 10) += (f_flux - b_flux);
 
 _term =  _lq10 * gamma ;
 _MATELM1( 1 ,1)  += _term;
 _MATELM1( 10 ,1)  -= _term;
 _term =  _lq10 * delta ;
 _MATELM1( 1 ,10)  -= _term;
 _MATELM1( 10 ,10)  += _term;
 /*REACTION*/
  /* ~ i1 <-> i2 ( 3.0 * alpha * a , beta / b )*/
 f_flux =  3.0 * alpha * a * i1 ;
 b_flux =  beta / b * i2 ;
 _RHS1( 9) -= (f_flux - b_flux);
 _RHS1( 8) += (f_flux - b_flux);
 
 _term =  3.0 * alpha * a ;
 _MATELM1( 9 ,9)  += _term;
 _MATELM1( 8 ,9)  -= _term;
 _term =  beta / b ;
 _MATELM1( 9 ,8)  -= _term;
 _MATELM1( 8 ,8)  += _term;
 /*REACTION*/
  /* ~ i2 <-> i3 ( 2.0 * alpha * a , 2.0 * beta / b )*/
 f_flux =  2.0 * alpha * a * i2 ;
 b_flux =  2.0 * beta / b * i3 ;
 _RHS1( 8) -= (f_flux - b_flux);
 _RHS1( 7) += (f_flux - b_flux);
 
 _term =  2.0 * alpha * a ;
 _MATELM1( 8 ,8)  += _term;
 _MATELM1( 7 ,8)  -= _term;
 _term =  2.0 * beta / b ;
 _MATELM1( 8 ,7)  -= _term;
 _MATELM1( 7 ,7)  += _term;
 /*REACTION*/
  /* ~ i3 <-> i4 ( alpha * a , 3.0 * beta / b )*/
 f_flux =  alpha * a * i3 ;
 b_flux =  3.0 * beta / b * i4 ;
 _RHS1( 7) -= (f_flux - b_flux);
 _RHS1( 6) += (f_flux - b_flux);
 
 _term =  alpha * a ;
 _MATELM1( 7 ,7)  += _term;
 _MATELM1( 6 ,7)  -= _term;
 _term =  3.0 * beta / b ;
 _MATELM1( 7 ,6)  -= _term;
 _MATELM1( 6 ,6)  += _term;
 /*REACTION*/
  /* ~ i4 <-> i5 ( _lq10 * gamma , _lq10 * delta )*/
 f_flux =  _lq10 * gamma * i4 ;
 b_flux =  _lq10 * delta * i5 ;
 _RHS1( 6) -= (f_flux - b_flux);
 _RHS1( 5) += (f_flux - b_flux);
 
 _term =  _lq10 * gamma ;
 _MATELM1( 6 ,6)  += _term;
 _MATELM1( 5 ,6)  -= _term;
 _term =  _lq10 * delta ;
 _MATELM1( 6 ,5)  -= _term;
 _MATELM1( 5 ,5)  += _term;
 /*REACTION*/
  /* ~ i5 <-> is ( _lq10 * i5is , _lq10 * isi5 )*/
 f_flux =  _lq10 * i5is * i5 ;
 b_flux =  _lq10 * isi5 * is ;
 _RHS1( 5) -= (f_flux - b_flux);
 
 _term =  _lq10 * i5is ;
 _MATELM1( 5 ,5)  += _term;
 _term =  _lq10 * isi5 ;
 _MATELM1( 5 ,0)  -= _term;
 /*REACTION*/
  /* ~ i1 <-> c1 ( _lq10 * ic , _lq10 * ci )*/
 f_flux =  _lq10 * ic * i1 ;
 b_flux =  _lq10 * ci * c1 ;
 _RHS1( 9) -= (f_flux - b_flux);
 _RHS1( 4) += (f_flux - b_flux);
 
 _term =  _lq10 * ic ;
 _MATELM1( 9 ,9)  += _term;
 _MATELM1( 4 ,9)  -= _term;
 _term =  _lq10 * ci ;
 _MATELM1( 9 ,4)  -= _term;
 _MATELM1( 4 ,4)  += _term;
 /*REACTION*/
  /* ~ i2 <-> c2 ( _lq10 * ic / b , _lq10 * ci * a )*/
 f_flux =  _lq10 * ic / b * i2 ;
 b_flux =  _lq10 * ci * a * c2 ;
 _RHS1( 8) -= (f_flux - b_flux);
 _RHS1( 3) += (f_flux - b_flux);
 
 _term =  _lq10 * ic / b ;
 _MATELM1( 8 ,8)  += _term;
 _MATELM1( 3 ,8)  -= _term;
 _term =  _lq10 * ci * a ;
 _MATELM1( 8 ,3)  -= _term;
 _MATELM1( 3 ,3)  += _term;
 /*REACTION*/
  /* ~ i3 <-> c3 ( _lq10 * ic / pow( b , 2.0 ) , _lq10 * ci * pow( a , 2.0 ) )*/
 f_flux =  _lq10 * ic / pow( b , 2.0 ) * i3 ;
 b_flux =  _lq10 * ci * pow( a , 2.0 ) * c3 ;
 _RHS1( 7) -= (f_flux - b_flux);
 _RHS1( 2) += (f_flux - b_flux);
 
 _term =  _lq10 * ic / pow( b , 2.0 ) ;
 _MATELM1( 7 ,7)  += _term;
 _MATELM1( 2 ,7)  -= _term;
 _term =  _lq10 * ci * pow( a , 2.0 ) ;
 _MATELM1( 7 ,2)  -= _term;
 _MATELM1( 2 ,2)  += _term;
 /*REACTION*/
  /* ~ i4 <-> c4 ( _lq10 * ic / pow( b , 3.0 ) , _lq10 * ci * pow( a , 3.0 ) )*/
 f_flux =  _lq10 * ic / pow( b , 3.0 ) * i4 ;
 b_flux =  _lq10 * ci * pow( a , 3.0 ) * c4 ;
 _RHS1( 6) -= (f_flux - b_flux);
 _RHS1( 1) += (f_flux - b_flux);
 
 _term =  _lq10 * ic / pow( b , 3.0 ) ;
 _MATELM1( 6 ,6)  += _term;
 _MATELM1( 1 ,6)  -= _term;
 _term =  _lq10 * ci * pow( a , 3.0 ) ;
 _MATELM1( 6 ,1)  -= _term;
 _MATELM1( 1 ,1)  += _term;
 /*REACTION*/
  /* ~ i5 <-> o ( _lq10 * io , _lq10 * oi )*/
 f_flux =  _lq10 * io * i5 ;
 b_flux =  _lq10 * oi * o ;
 _RHS1( 5) -= (f_flux - b_flux);
 _RHS1( 10) += (f_flux - b_flux);
 
 _term =  _lq10 * io ;
 _MATELM1( 5 ,5)  += _term;
 _MATELM1( 10 ,5)  -= _term;
 _term =  _lq10 * oi ;
 _MATELM1( 5 ,10)  -= _term;
 _MATELM1( 10 ,10)  += _term;
 /*REACTION*/
   /* c1 + c2 + c3 + c4 + i1 + i2 + i3 + i4 + i5 + o + is = 1.0 */
 _RHS1(0) =  1.0;
 _MATELM1(0, 0) = 1;
 _RHS1(0) -= is ;
 _MATELM1(0, 10) = 1;
 _RHS1(0) -= o ;
 _MATELM1(0, 5) = 1;
 _RHS1(0) -= i5 ;
 _MATELM1(0, 6) = 1;
 _RHS1(0) -= i4 ;
 _MATELM1(0, 7) = 1;
 _RHS1(0) -= i3 ;
 _MATELM1(0, 8) = 1;
 _RHS1(0) -= i2 ;
 _MATELM1(0, 9) = 1;
 _RHS1(0) -= i1 ;
 _MATELM1(0, 1) = 1;
 _RHS1(0) -= c4 ;
 _MATELM1(0, 2) = 1;
 _RHS1(0) -= c3 ;
 _MATELM1(0, 3) = 1;
 _RHS1(0) -= c2 ;
 _MATELM1(0, 4) = 1;
 _RHS1(0) -= c1 ;
 /*CONSERVATION*/
   } return _reset;
 }
 
static int  rates (  double _lv ) {
   double _lq10 ;
 _lq10 = pow( q10v , ( ( celsius - 22.0 ) / 10.0 ) ) ;
   alpha = _lq10 * am * exp ( ( _lv - vha ) / vc ) ;
   beta = _lq10 * bm * exp ( ( _lv - vhb ) / - vc ) ;
    return 0; }
 
static void _hoc_rates(void) {
  double _r;
    _r = 1.;
 rates (  *getarg(1) );
 hoc_retpushx(_r);
}
 
static double _npy_rates(Prop* _prop) {
    double _r{0.0};
    neuron::legacy::set_globals_from_prop(_prop, _ml_real, _ml, _iml);
  _ppvar = _nrn_mechanism_access_dparam(_prop);
 _r = 1.;
 rates (  *getarg(1) );
 return(_r);
}
 
double modulation (  double _lconc , double _lmod_min , double _lmod_max , double _lmod_half , double _lmod_slope ) {
   double _lmodulation;
 _lmodulation = _lmod_min + ( _lmod_max - _lmod_min ) / ( 1.0 + exp ( - ( _lconc - _lmod_half ) / _lmod_slope ) ) ;
   
return _lmodulation;
 }
 
static void _hoc_modulation(void) {
  double _r;
    _r =  modulation (  *getarg(1) , *getarg(2) , *getarg(3) , *getarg(4) , *getarg(5) );
 hoc_retpushx(_r);
}
 
static double _npy_modulation(Prop* _prop) {
    double _r{0.0};
    neuron::legacy::set_globals_from_prop(_prop, _ml_real, _ml, _iml);
  _ppvar = _nrn_mechanism_access_dparam(_prop);
 _r =  modulation (  *getarg(1) , *getarg(2) , *getarg(3) , *getarg(4) , *getarg(5) );
 return(_r);
}
 
/*CVODE ode begin*/
 static int _ode_spec1() {_reset=0;{
 double _lq10 ;
 double b_flux, f_flux, _term; int _i;
 {int _i; for(_i=0;_i<11;_i++) _ml->data(_iml, _dlist1[_i]) = 0.0;}
 _lq10 = pow( q10i , ( ( celsius - 22.0 ) / 10.0 ) ) ;
 rates ( _threadargscomma_ v ) ;
 /* ~ c1 <-> c2 ( 3.0 * alpha , beta )*/
 f_flux =  3.0 * alpha * c1 ;
 b_flux =  beta * c2 ;
 Dc1 -= (f_flux - b_flux);
 Dc2 += (f_flux - b_flux);
 
 /*REACTION*/
  /* ~ c2 <-> c3 ( 2.0 * alpha , 2.0 * beta )*/
 f_flux =  2.0 * alpha * c2 ;
 b_flux =  2.0 * beta * c3 ;
 Dc2 -= (f_flux - b_flux);
 Dc3 += (f_flux - b_flux);
 
 /*REACTION*/
  /* ~ c3 <-> c4 ( alpha , 3.0 * beta )*/
 f_flux =  alpha * c3 ;
 b_flux =  3.0 * beta * c4 ;
 Dc3 -= (f_flux - b_flux);
 Dc4 += (f_flux - b_flux);
 
 /*REACTION*/
  /* ~ c4 <-> o ( _lq10 * gamma , _lq10 * delta )*/
 f_flux =  _lq10 * gamma * c4 ;
 b_flux =  _lq10 * delta * o ;
 Dc4 -= (f_flux - b_flux);
 Do += (f_flux - b_flux);
 
 /*REACTION*/
  /* ~ i1 <-> i2 ( 3.0 * alpha * a , beta / b )*/
 f_flux =  3.0 * alpha * a * i1 ;
 b_flux =  beta / b * i2 ;
 Di1 -= (f_flux - b_flux);
 Di2 += (f_flux - b_flux);
 
 /*REACTION*/
  /* ~ i2 <-> i3 ( 2.0 * alpha * a , 2.0 * beta / b )*/
 f_flux =  2.0 * alpha * a * i2 ;
 b_flux =  2.0 * beta / b * i3 ;
 Di2 -= (f_flux - b_flux);
 Di3 += (f_flux - b_flux);
 
 /*REACTION*/
  /* ~ i3 <-> i4 ( alpha * a , 3.0 * beta / b )*/
 f_flux =  alpha * a * i3 ;
 b_flux =  3.0 * beta / b * i4 ;
 Di3 -= (f_flux - b_flux);
 Di4 += (f_flux - b_flux);
 
 /*REACTION*/
  /* ~ i4 <-> i5 ( _lq10 * gamma , _lq10 * delta )*/
 f_flux =  _lq10 * gamma * i4 ;
 b_flux =  _lq10 * delta * i5 ;
 Di4 -= (f_flux - b_flux);
 Di5 += (f_flux - b_flux);
 
 /*REACTION*/
  /* ~ i5 <-> is ( _lq10 * i5is , _lq10 * isi5 )*/
 f_flux =  _lq10 * i5is * i5 ;
 b_flux =  _lq10 * isi5 * is ;
 Di5 -= (f_flux - b_flux);
 Dis += (f_flux - b_flux);
 
 /*REACTION*/
  /* ~ i1 <-> c1 ( _lq10 * ic , _lq10 * ci )*/
 f_flux =  _lq10 * ic * i1 ;
 b_flux =  _lq10 * ci * c1 ;
 Di1 -= (f_flux - b_flux);
 Dc1 += (f_flux - b_flux);
 
 /*REACTION*/
  /* ~ i2 <-> c2 ( _lq10 * ic / b , _lq10 * ci * a )*/
 f_flux =  _lq10 * ic / b * i2 ;
 b_flux =  _lq10 * ci * a * c2 ;
 Di2 -= (f_flux - b_flux);
 Dc2 += (f_flux - b_flux);
 
 /*REACTION*/
  /* ~ i3 <-> c3 ( _lq10 * ic / pow( b , 2.0 ) , _lq10 * ci * pow( a , 2.0 ) )*/
 f_flux =  _lq10 * ic / pow( b , 2.0 ) * i3 ;
 b_flux =  _lq10 * ci * pow( a , 2.0 ) * c3 ;
 Di3 -= (f_flux - b_flux);
 Dc3 += (f_flux - b_flux);
 
 /*REACTION*/
  /* ~ i4 <-> c4 ( _lq10 * ic / pow( b , 3.0 ) , _lq10 * ci * pow( a , 3.0 ) )*/
 f_flux =  _lq10 * ic / pow( b , 3.0 ) * i4 ;
 b_flux =  _lq10 * ci * pow( a , 3.0 ) * c4 ;
 Di4 -= (f_flux - b_flux);
 Dc4 += (f_flux - b_flux);
 
 /*REACTION*/
  /* ~ i5 <-> o ( _lq10 * io , _lq10 * oi )*/
 f_flux =  _lq10 * io * i5 ;
 b_flux =  _lq10 * oi * o ;
 Di5 -= (f_flux - b_flux);
 Do += (f_flux - b_flux);
 
 /*REACTION*/
   /* c1 + c2 + c3 + c4 + i1 + i2 + i3 + i4 + i5 + o + is = 1.0 */
 /*CONSERVATION*/
   } return _reset;
 }
 
/*CVODE matsol*/
 static int _ode_matsol1() {_reset=0;{
 double _lq10 ;
 double b_flux, f_flux, _term; int _i;
   b_flux = f_flux = 0.;
 {int _i; double _dt1 = 1.0/dt;
for(_i=0;_i<11;_i++){
  	_RHS1(_i) = _dt1*(_ml->data(_iml, _dlist1[_i]));
	_MATELM1(_i, _i) = _dt1;
      
} }
 _lq10 = pow( q10i , ( ( celsius - 22.0 ) / 10.0 ) ) ;
 rates ( _threadargscomma_ v ) ;
 /* ~ c1 <-> c2 ( 3.0 * alpha , beta )*/
 _term =  3.0 * alpha ;
 _MATELM1( 4 ,4)  += _term;
 _MATELM1( 3 ,4)  -= _term;
 _term =  beta ;
 _MATELM1( 4 ,3)  -= _term;
 _MATELM1( 3 ,3)  += _term;
 /*REACTION*/
  /* ~ c2 <-> c3 ( 2.0 * alpha , 2.0 * beta )*/
 _term =  2.0 * alpha ;
 _MATELM1( 3 ,3)  += _term;
 _MATELM1( 2 ,3)  -= _term;
 _term =  2.0 * beta ;
 _MATELM1( 3 ,2)  -= _term;
 _MATELM1( 2 ,2)  += _term;
 /*REACTION*/
  /* ~ c3 <-> c4 ( alpha , 3.0 * beta )*/
 _term =  alpha ;
 _MATELM1( 2 ,2)  += _term;
 _MATELM1( 1 ,2)  -= _term;
 _term =  3.0 * beta ;
 _MATELM1( 2 ,1)  -= _term;
 _MATELM1( 1 ,1)  += _term;
 /*REACTION*/
  /* ~ c4 <-> o ( _lq10 * gamma , _lq10 * delta )*/
 _term =  _lq10 * gamma ;
 _MATELM1( 1 ,1)  += _term;
 _MATELM1( 10 ,1)  -= _term;
 _term =  _lq10 * delta ;
 _MATELM1( 1 ,10)  -= _term;
 _MATELM1( 10 ,10)  += _term;
 /*REACTION*/
  /* ~ i1 <-> i2 ( 3.0 * alpha * a , beta / b )*/
 _term =  3.0 * alpha * a ;
 _MATELM1( 9 ,9)  += _term;
 _MATELM1( 8 ,9)  -= _term;
 _term =  beta / b ;
 _MATELM1( 9 ,8)  -= _term;
 _MATELM1( 8 ,8)  += _term;
 /*REACTION*/
  /* ~ i2 <-> i3 ( 2.0 * alpha * a , 2.0 * beta / b )*/
 _term =  2.0 * alpha * a ;
 _MATELM1( 8 ,8)  += _term;
 _MATELM1( 7 ,8)  -= _term;
 _term =  2.0 * beta / b ;
 _MATELM1( 8 ,7)  -= _term;
 _MATELM1( 7 ,7)  += _term;
 /*REACTION*/
  /* ~ i3 <-> i4 ( alpha * a , 3.0 * beta / b )*/
 _term =  alpha * a ;
 _MATELM1( 7 ,7)  += _term;
 _MATELM1( 6 ,7)  -= _term;
 _term =  3.0 * beta / b ;
 _MATELM1( 7 ,6)  -= _term;
 _MATELM1( 6 ,6)  += _term;
 /*REACTION*/
  /* ~ i4 <-> i5 ( _lq10 * gamma , _lq10 * delta )*/
 _term =  _lq10 * gamma ;
 _MATELM1( 6 ,6)  += _term;
 _MATELM1( 5 ,6)  -= _term;
 _term =  _lq10 * delta ;
 _MATELM1( 6 ,5)  -= _term;
 _MATELM1( 5 ,5)  += _term;
 /*REACTION*/
  /* ~ i5 <-> is ( _lq10 * i5is , _lq10 * isi5 )*/
 _term =  _lq10 * i5is ;
 _MATELM1( 5 ,5)  += _term;
 _MATELM1( 0 ,5)  -= _term;
 _term =  _lq10 * isi5 ;
 _MATELM1( 5 ,0)  -= _term;
 _MATELM1( 0 ,0)  += _term;
 /*REACTION*/
  /* ~ i1 <-> c1 ( _lq10 * ic , _lq10 * ci )*/
 _term =  _lq10 * ic ;
 _MATELM1( 9 ,9)  += _term;
 _MATELM1( 4 ,9)  -= _term;
 _term =  _lq10 * ci ;
 _MATELM1( 9 ,4)  -= _term;
 _MATELM1( 4 ,4)  += _term;
 /*REACTION*/
  /* ~ i2 <-> c2 ( _lq10 * ic / b , _lq10 * ci * a )*/
 _term =  _lq10 * ic / b ;
 _MATELM1( 8 ,8)  += _term;
 _MATELM1( 3 ,8)  -= _term;
 _term =  _lq10 * ci * a ;
 _MATELM1( 8 ,3)  -= _term;
 _MATELM1( 3 ,3)  += _term;
 /*REACTION*/
  /* ~ i3 <-> c3 ( _lq10 * ic / pow( b , 2.0 ) , _lq10 * ci * pow( a , 2.0 ) )*/
 _term =  _lq10 * ic / pow( b , 2.0 ) ;
 _MATELM1( 7 ,7)  += _term;
 _MATELM1( 2 ,7)  -= _term;
 _term =  _lq10 * ci * pow( a , 2.0 ) ;
 _MATELM1( 7 ,2)  -= _term;
 _MATELM1( 2 ,2)  += _term;
 /*REACTION*/
  /* ~ i4 <-> c4 ( _lq10 * ic / pow( b , 3.0 ) , _lq10 * ci * pow( a , 3.0 ) )*/
 _term =  _lq10 * ic / pow( b , 3.0 ) ;
 _MATELM1( 6 ,6)  += _term;
 _MATELM1( 1 ,6)  -= _term;
 _term =  _lq10 * ci * pow( a , 3.0 ) ;
 _MATELM1( 6 ,1)  -= _term;
 _MATELM1( 1 ,1)  += _term;
 /*REACTION*/
  /* ~ i5 <-> o ( _lq10 * io , _lq10 * oi )*/
 _term =  _lq10 * io ;
 _MATELM1( 5 ,5)  += _term;
 _MATELM1( 10 ,5)  -= _term;
 _term =  _lq10 * oi ;
 _MATELM1( 5 ,10)  -= _term;
 _MATELM1( 10 ,10)  += _term;
 /*REACTION*/
   /* c1 + c2 + c3 + c4 + i1 + i2 + i3 + i4 + i5 + o + is = 1.0 */
 /*CONSERVATION*/
   } return _reset;
 }
 
/*CVODE end*/
 
static int _ode_count(int _type){ return 11;}
 
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
  ek = _ion_ek;
  PKAci = _ion_PKAci;
     _ode_spec1 ();
  }}
 
static void _ode_map(Prop* _prop, int _ieq, neuron::container::data_handle<double>* _pv, neuron::container::data_handle<double>* _pvdot, double* _atol, int _type) { 
  _ppvar = _nrn_mechanism_access_dparam(_prop);
  _cvode_ieq = _ieq;
  for (int _i=0; _i < 11; ++_i) {
    _pv[_i] = _nrn_mechanism_get_param_handle(_prop, _slist1[_i]);
    _pvdot[_i] = _nrn_mechanism_get_param_handle(_prop, _dlist1[_i]);
    _cvode_abstol(_atollist, _atol, _i);
  }
 }
 
static void _ode_matsol_instance1(_internalthreadargsproto_) {
 _cvode_sparse(&_cvsparseobj1, 11, _dlist1, neuron::scopmath::row_view{_ml, _iml}, _ode_matsol1, &_coef1);
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
  ek = _ion_ek;
  PKAci = _ion_PKAci;
 _ode_matsol_instance1(_threadargs_);
 }}

static void initmodel() {
  int _i; double _save;_ninits++;
 _save = t;
 t = 0.0;
{
  c4 = c40;
  c3 = c30;
  c2 = c20;
  c1 = c10;
  is = is0;
  i5 = i50;
  i4 = i40;
  i3 = i30;
  i2 = i20;
  i1 = i10;
  o = o0;
 {
   error = _ss_sparse(&_sparseobj1, 11, _slist1, _dlist1, neuron::scopmath::row_view{_ml, _iml}, &t, dt, kin, &_coef1, _linmat1);
 if(error){
  std_cerr_stream << "at line 88 in file kv4_ch.mod:\nINITIAL {\n";
  std_cerr_stream << _ml << ' ' << _iml << '\n';
  abort_run(error);
}
    if (secondorder) {
    int _i;
    for (_i = 0; _i < 11; ++_i) {
      _ml->data(_iml, _slist1[_i]) += dt*_ml->data(_iml, _dlist1[_i]);
    }}
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
   _v = _vec_v[_ni[_iml]];
 v = _v;
  ek = _ion_ek;
  PKAci = _ion_PKAci;
 initmodel();
 }}

static double _nrn_current(double _v){double _current=0.;v=_v;{ {
   modulation_factor = modulation ( _threadargscomma_ PKAci , mod_pka_g_min , mod_pka_g_max , mod_pka_g_half , mod_pka_g_slope ) ;
   g = gbar * o * modulation_factor ;
   ik = g * ( v - ek ) ;
   }
 _current += ik;

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
  ek = _ion_ek;
  PKAci = _ion_PKAci;
 auto const _g_local = _nrn_current(_v + .001);
 	{ double _dik;
  _dik = ik;
 _rhs = _nrn_current(_v);
  _ion_dikdv += (_dik - ik)/.001 ;
 	}
 _g = (_g_local - _rhs)/.001;
  _ion_ik += ik ;
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
double _dtsav = dt;
if (secondorder) { dt *= 0.5; }
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
  ek = _ion_ek;
  PKAci = _ion_PKAci;
 { error = sparse(&_sparseobj1, 11, _slist1, _dlist1, neuron::scopmath::row_view{_ml, _iml}, &t, dt, kin, &_coef1, _linmat1);
 if(error){
  std_cerr_stream << "at line 81 in file kv4_ch.mod:\nBREAKPOINT {\n";
  std_cerr_stream << _ml << ' ' << _iml << '\n';
  abort_run(error);
}
    if (secondorder) {
    int _i;
    for (_i = 0; _i < 11; ++_i) {
      _ml->data(_iml, _slist1[_i]) += dt*_ml->data(_iml, _dlist1[_i]);
    }}
 } }}
 dt = _dtsav;
}

static void terminal(){}

static void _initlists() {
 int _i; static int _first = 1;
  if (!_first) return;
 _slist1[0] = {is_columnindex, 0};  _dlist1[0] = {Dis_columnindex, 0};
 _slist1[1] = {c4_columnindex, 0};  _dlist1[1] = {Dc4_columnindex, 0};
 _slist1[2] = {c3_columnindex, 0};  _dlist1[2] = {Dc3_columnindex, 0};
 _slist1[3] = {c2_columnindex, 0};  _dlist1[3] = {Dc2_columnindex, 0};
 _slist1[4] = {c1_columnindex, 0};  _dlist1[4] = {Dc1_columnindex, 0};
 _slist1[5] = {i5_columnindex, 0};  _dlist1[5] = {Di5_columnindex, 0};
 _slist1[6] = {i4_columnindex, 0};  _dlist1[6] = {Di4_columnindex, 0};
 _slist1[7] = {i3_columnindex, 0};  _dlist1[7] = {Di3_columnindex, 0};
 _slist1[8] = {i2_columnindex, 0};  _dlist1[8] = {Di2_columnindex, 0};
 _slist1[9] = {i1_columnindex, 0};  _dlist1[9] = {Di1_columnindex, 0};
 _slist1[10] = {o_columnindex, 0};  _dlist1[10] = {Do_columnindex, 0};
_first = 0;
}

#if NMODL_TEXT
static void register_nmodl_text_and_filename(int mech_type) {
    const char* nmodl_filename = "/cfs/klemming/home/m/metog/BasalGangliaData/data/neurons/mechanisms/kv4_ch.mod";
    const char* nmodl_file_text = 
  "COMMENT\n"
  "\n"
  "c1 - c2 - c3 - c4 - o\n"
  "|    |    |    |    |\n"
  "i1 - i2 - i3 - i4 - i5 - is\n"
  "\n"
  "ENDCOMMENT\n"
  "			      \n"
  "NEURON {\n"
  "	SUFFIX kv4_ch\n"
  "	USEION k READ ek WRITE ik\n"
  "	RANGE g, ik, gbar\n"
  "	GLOBAL alpha, beta\n"
  "	GLOBAL ci, ic, oi, io, a, b, am, bm, vc, gamma, delta, vha, vhb\n"
  "	GLOBAL i5is, isi5\n"
  "	GLOBAL q10i, q10v\n"
  "\n"
  "    USEION PKAc READ PKAci VALENCE 0\n"
  "    RANGE mod_pka_g_min, mod_pka_g_max, mod_pka_g_half, mod_pka_g_slope \n"
  "    RANGE modulation_factor\n"
  "			  \n"
  "}\n"
  "\n"
  "UNITS {\n"
  "	(mV) = (millivolt)\n"
  "	(mA) = (milliamp)\n"
  "	(S) = (siemens)\n"
  "}\n"
  "\n"
  "PARAMETER {\n"
  "	gbar = 1	(S/cm2)\n"
  "	gamma = 200	(1/ms)\n"
  "	delta = 4	(1/ms)\n"
  "	a = 3\n"
  "	b = 40\n"
  "	ic = 500	(/ms)\n"
  "	oi = 1e-9	(/ms)\n"
  "	io = .01	(/ms)\n"
  "	ci = .2		(/ms)\n"
  "	am = 1		(1/ms)\n"
  "	bm = 7		(1/ms)\n"
  "	vc = 10		(mV)\n"
  "	vha = -75	(mV)\n"
  "	vhb = -30	(mV)\n"
  "	i5is = .001	(/ms)\n"
  "	isi5 = .001	(/ms)\n"
  "	q10i = 3\n"
  "	q10v = 3\n"
  "	celsius		(degC)\n"
  "    mod_pka_g_min = 1 (1)\n"
  "    mod_pka_g_max = 1 (1)\n"
  "    mod_pka_g_half = 0.000100 (mM)\n"
  "    mod_pka_g_slope = 0.01 (mM)\n"
  "}\n"
  "\n"
  "ASSIGNED {\n"
  "	v	(mV)\n"
  "	ek	(mV)\n"
  "	g	(S/cm2)\n"
  "	ik	(mA/cm2)\n"
  "	alpha	(/ms)\n"
  "	beta	(/ms)\n"
  "    PKAci (mM)\n"
  "    modulation_factor (1)\n"
  "}\n"
  "\n"
  "STATE {\n"
  "	c1\n"
  "	c2\n"
  "	c3\n"
  "	c4\n"
  "	o\n"
  "	i1\n"
  "	i2\n"
  "	i3\n"
  "	i4\n"
  "	i5\n"
  "	is\n"
  "}\n"
  "\n"
  "BREAKPOINT {\n"
  "	SOLVE kin METHOD sparse\n"
  "    modulation_factor=modulation(PKAci, mod_pka_g_min, mod_pka_g_max, mod_pka_g_half, mod_pka_g_slope)	   \n"
  "	g = gbar*o*modulation_factor\n"
  "	ik = g*(v-ek)\n"
  "}\n"
  "\n"
  "INITIAL {\n"
  "	SOLVE kin STEADYSTATE sparse\n"
  "}\n"
  "\n"
  "KINETIC kin{LOCAL q10\n"
  "	q10 = q10i^((celsius - 22 (degC))/10 (degC))\n"
  "	rates(v)\n"
  "	~ c1 <-> c2 (3*alpha,beta)\n"
  "	~ c2 <-> c3 (2*alpha,2*beta)\n"
  "	~ c3 <-> c4 (alpha,3*beta)\n"
  "	~ c4 <-> o  (q10*gamma,q10*delta)\n"
  "\n"
  "	~ i1 <-> i2 (3*alpha*a,beta/b)\n"
  "	~ i2 <-> i3 (2*alpha*a,2*beta/b)\n"
  "	~ i3 <-> i4 (alpha*a,3*beta/b)\n"
  "	~ i4 <-> i5 (q10*gamma,q10*delta)\n"
  "	~ i5 <-> is (q10*i5is,q10*isi5)\n"
  "\n"
  "	~ i1 <-> c1 (q10*ic,q10*ci)\n"
  "	~ i2 <-> c2 (q10*ic/b,q10*ci*a)\n"
  "	~ i3 <-> c3 (q10*ic/b^2,q10*ci*a^2)\n"
  "	~ i4 <-> c4 (q10*ic/b^3,q10*ci*a^3)\n"
  "	~ i5 <-> o  (q10*io,q10*oi)\n"
  "\n"
  "	CONSERVE c1+c2+c3+c4+i1+i2+i3+i4+i5+o+is=1\n"
  "}\n"
  "\n"
  "PROCEDURE rates(v(millivolt)) {LOCAL q10\n"
  "	q10 = q10v^((celsius - 22 (degC))/10 (degC))\n"
  "	alpha = q10*am*exp((v-vha)/vc)\n"
  "	beta = q10*bm*exp((v-vhb)/-vc)\n"
  "}\n"
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
