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
static constexpr auto number_of_floating_point_variables = 24;
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
 
#define nrn_init _nrn_init__hcn12_ch
#define _nrn_initial _nrn_initial__hcn12_ch
#define nrn_cur _nrn_cur__hcn12_ch
#define _nrn_current _nrn_current__hcn12_ch
#define nrn_jacob _nrn_jacob__hcn12_ch
#define nrn_state _nrn_state__hcn12_ch
#define _net_receive _net_receive__hcn12_ch 
#define kin kin__hcn12_ch 
#define rates rates__hcn12_ch 
 
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
#define ehcn _ml->template fpfield<1>(_iml)
#define ehcn_columnindex 1
#define mod_pka_g_min _ml->template fpfield<2>(_iml)
#define mod_pka_g_min_columnindex 2
#define mod_pka_g_max _ml->template fpfield<3>(_iml)
#define mod_pka_g_max_columnindex 3
#define mod_pka_g_half _ml->template fpfield<4>(_iml)
#define mod_pka_g_half_columnindex 4
#define mod_pka_g_slope _ml->template fpfield<5>(_iml)
#define mod_pka_g_slope_columnindex 5
#define g _ml->template fpfield<6>(_iml)
#define g_columnindex 6
#define i _ml->template fpfield<7>(_iml)
#define i_columnindex 7
#define modulation_factor _ml->template fpfield<8>(_iml)
#define modulation_factor_columnindex 8
#define c _ml->template fpfield<9>(_iml)
#define c_columnindex 9
#define cac _ml->template fpfield<10>(_iml)
#define cac_columnindex 10
#define o _ml->template fpfield<11>(_iml)
#define o_columnindex 11
#define cao _ml->template fpfield<12>(_iml)
#define cao_columnindex 12
#define alpha _ml->template fpfield<13>(_iml)
#define alpha_columnindex 13
#define beta _ml->template fpfield<14>(_iml)
#define beta_columnindex 14
#define alphaa _ml->template fpfield<15>(_iml)
#define alphaa_columnindex 15
#define betaa _ml->template fpfield<16>(_iml)
#define betaa_columnindex 16
#define PKAci _ml->template fpfield<17>(_iml)
#define PKAci_columnindex 17
#define Dc _ml->template fpfield<18>(_iml)
#define Dc_columnindex 18
#define Dcac _ml->template fpfield<19>(_iml)
#define Dcac_columnindex 19
#define Do _ml->template fpfield<20>(_iml)
#define Do_columnindex 20
#define Dcao _ml->template fpfield<21>(_iml)
#define Dcao_columnindex 21
#define v _ml->template fpfield<22>(_iml)
#define v_columnindex 22
#define _g _ml->template fpfield<23>(_iml)
#define _g_columnindex 23
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
 {"setdata_hcn12_ch", _hoc_setdata},
 {"modulation_hcn12_ch", _hoc_modulation},
 {"rates_hcn12_ch", _hoc_rates},
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
#define modulation modulation_hcn12_ch
 extern double modulation( _internalthreadargsprotocomma_ double , double , double , double , double );
 /* declare global and static user variables */
 #define gind 0
 #define _gth 0
#define ai ai_hcn12_ch
 double ai = 1e-05;
#define aac aac_hcn12_ch
 double aac = -0.075;
#define aah aah_hcn12_ch
 double aah = -94.2;
#define aa0 aa0_hcn12_ch
 double aa0 = 0.0006;
#define ac ac_hcn12_ch
 double ac = -0.155;
#define ah ah_hcn12_ch
 double ah = -96;
#define a0 a0_hcn12_ch
 double a0 = 0.006;
#define bf bf_hcn12_ch
 double bf = 8.94;
#define b b_hcn12_ch
 double b = 80;
#define bac bac_hcn12_ch
 double bac = 0.144;
#define bah bah_hcn12_ch
 double bah = -35.5;
#define ba0 ba0_hcn12_ch
 double ba0 = 0.004;
#define bc bc_hcn12_ch
 double bc = 0.144;
#define bh bh_hcn12_ch
 double bh = -51.7;
#define b0 b0_hcn12_ch
 double b0 = 0.0008;
#define gca gca_hcn12_ch
 double gca = 1;
#define koff koff_hcn12_ch
 double koff = 4.5e-05;
#define kon kon_hcn12_ch
 double kon = 30;
#define q10a q10a_hcn12_ch
 double q10a = 1.5;
#define q10v q10v_hcn12_ch
 double q10v = 4;
#define shift shift_hcn12_ch
 double shift = -17;
 /* some parameters have upper and lower limits */
 static HocParmLimits _hoc_parm_limits[] = {
 {0, 0, 0}
};
 static HocParmUnits _hoc_parm_units[] = {
 {"a0_hcn12_ch", "/ms"},
 {"b0_hcn12_ch", "/ms"},
 {"ah_hcn12_ch", "mV"},
 {"bh_hcn12_ch", "mV"},
 {"ac_hcn12_ch", "/mV"},
 {"bc_hcn12_ch", "/mV"},
 {"aa0_hcn12_ch", "/ms"},
 {"ba0_hcn12_ch", "/ms"},
 {"aah_hcn12_ch", "mV"},
 {"bah_hcn12_ch", "mV"},
 {"aac_hcn12_ch", "/mV"},
 {"bac_hcn12_ch", "/mV"},
 {"kon_hcn12_ch", "/mM-ms"},
 {"koff_hcn12_ch", "/ms"},
 {"ai_hcn12_ch", "mM"},
 {"shift_hcn12_ch", "mV"},
 {"gbar_hcn12_ch", "S/cm2"},
 {"ehcn_hcn12_ch", "mV"},
 {"mod_pka_g_min_hcn12_ch", "1"},
 {"mod_pka_g_max_hcn12_ch", "1"},
 {"mod_pka_g_half_hcn12_ch", "mM"},
 {"mod_pka_g_slope_hcn12_ch", "mM"},
 {"g_hcn12_ch", "S/cm2"},
 {"i_hcn12_ch", "mA/cm2"},
 {"modulation_factor_hcn12_ch", "1"},
 {0, 0}
};
 static double cao0 = 0;
 static double cac0 = 0;
 static double c0 = 0;
 static double delta_t = 0.01;
 static double o0 = 0;
 /* connect global user variables to hoc */
 static DoubScal hoc_scdoub[] = {
 {"a0_hcn12_ch", &a0_hcn12_ch},
 {"b0_hcn12_ch", &b0_hcn12_ch},
 {"ah_hcn12_ch", &ah_hcn12_ch},
 {"bh_hcn12_ch", &bh_hcn12_ch},
 {"ac_hcn12_ch", &ac_hcn12_ch},
 {"bc_hcn12_ch", &bc_hcn12_ch},
 {"aa0_hcn12_ch", &aa0_hcn12_ch},
 {"ba0_hcn12_ch", &ba0_hcn12_ch},
 {"aah_hcn12_ch", &aah_hcn12_ch},
 {"bah_hcn12_ch", &bah_hcn12_ch},
 {"aac_hcn12_ch", &aac_hcn12_ch},
 {"bac_hcn12_ch", &bac_hcn12_ch},
 {"kon_hcn12_ch", &kon_hcn12_ch},
 {"koff_hcn12_ch", &koff_hcn12_ch},
 {"b_hcn12_ch", &b_hcn12_ch},
 {"bf_hcn12_ch", &bf_hcn12_ch},
 {"ai_hcn12_ch", &ai_hcn12_ch},
 {"gca_hcn12_ch", &gca_hcn12_ch},
 {"shift_hcn12_ch", &shift_hcn12_ch},
 {"q10v_hcn12_ch", &q10v_hcn12_ch},
 {"q10a_hcn12_ch", &q10a_hcn12_ch},
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
"hcn12_ch",
 "gbar_hcn12_ch",
 "ehcn_hcn12_ch",
 "mod_pka_g_min_hcn12_ch",
 "mod_pka_g_max_hcn12_ch",
 "mod_pka_g_half_hcn12_ch",
 "mod_pka_g_slope_hcn12_ch",
 0,
 "g_hcn12_ch",
 "i_hcn12_ch",
 "modulation_factor_hcn12_ch",
 0,
 "c_hcn12_ch",
 "cac_hcn12_ch",
 "o_hcn12_ch",
 "cao_hcn12_ch",
 0,
 0};
 static Symbol* _PKAc_sym;
 
 /* Used by NrnProperty */
 static _nrn_mechanism_std_vector<double> _parm_default{
     1, /* gbar */
     -20, /* ehcn */
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
    assert(_nrn_mechanism_get_num_vars(_prop) == 24);
 	/*initialize range parameters*/
 	gbar = _parm_default[0]; /* 1 */
 	ehcn = _parm_default[1]; /* -20 */
 	mod_pka_g_min = _parm_default[2]; /* 1 */
 	mod_pka_g_max = _parm_default[3]; /* 1 */
 	mod_pka_g_half = _parm_default[4]; /* 0.0001 */
 	mod_pka_g_slope = _parm_default[5]; /* 0.01 */
 	 assert(_nrn_mechanism_get_num_vars(_prop) == 24);
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
 static void _thread_cleanup(Datum*);
 extern Symbol* hoc_lookup(const char*);
extern void _nrn_thread_reg(int, int, void(*)(Datum*));
void _nrn_thread_table_reg(int, nrn_thread_table_check_t);
extern void hoc_register_tolerance(int, HocStateTolerance*, Symbol***);
extern void _cvode_abstol( Symbol**, double*, int);

 extern "C" void _hcn12_ch_reg() {
	int _vectorized = 1;
  _initlists();
 	ion_reg("PKAc", 0.0);
 	_PKAc_sym = hoc_lookup("PKAc_ion");
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
                                       _nrn_mechanism_field<double>{"ehcn"} /* 1 */,
                                       _nrn_mechanism_field<double>{"mod_pka_g_min"} /* 2 */,
                                       _nrn_mechanism_field<double>{"mod_pka_g_max"} /* 3 */,
                                       _nrn_mechanism_field<double>{"mod_pka_g_half"} /* 4 */,
                                       _nrn_mechanism_field<double>{"mod_pka_g_slope"} /* 5 */,
                                       _nrn_mechanism_field<double>{"g"} /* 6 */,
                                       _nrn_mechanism_field<double>{"i"} /* 7 */,
                                       _nrn_mechanism_field<double>{"modulation_factor"} /* 8 */,
                                       _nrn_mechanism_field<double>{"c"} /* 9 */,
                                       _nrn_mechanism_field<double>{"cac"} /* 10 */,
                                       _nrn_mechanism_field<double>{"o"} /* 11 */,
                                       _nrn_mechanism_field<double>{"cao"} /* 12 */,
                                       _nrn_mechanism_field<double>{"alpha"} /* 13 */,
                                       _nrn_mechanism_field<double>{"beta"} /* 14 */,
                                       _nrn_mechanism_field<double>{"alphaa"} /* 15 */,
                                       _nrn_mechanism_field<double>{"betaa"} /* 16 */,
                                       _nrn_mechanism_field<double>{"PKAci"} /* 17 */,
                                       _nrn_mechanism_field<double>{"Dc"} /* 18 */,
                                       _nrn_mechanism_field<double>{"Dcac"} /* 19 */,
                                       _nrn_mechanism_field<double>{"Do"} /* 20 */,
                                       _nrn_mechanism_field<double>{"Dcao"} /* 21 */,
                                       _nrn_mechanism_field<double>{"v"} /* 22 */,
                                       _nrn_mechanism_field<double>{"_g"} /* 23 */,
                                       _nrn_mechanism_field<double*>{"_ion_PKAci", "PKAc_ion"} /* 0 */,
                                       _nrn_mechanism_field<double*>{"_ion_PKAco", "PKAc_ion"} /* 1 */,
                                       _nrn_mechanism_field<int>{"_cvode_ieq", "cvodeieq"} /* 2 */);
  hoc_register_prop_size(_mechtype, 24, 3);
  hoc_register_dparam_semantics(_mechtype, 0, "PKAc_ion");
  hoc_register_dparam_semantics(_mechtype, 1, "PKAc_ion");
  hoc_register_dparam_semantics(_mechtype, 2, "cvodeieq");
 	hoc_register_cvode(_mechtype, _ode_count, _ode_map, _ode_spec, _ode_matsol);
 	hoc_register_tolerance(_mechtype, _hoc_state_tol, &_atollist);
 
    hoc_register_var(hoc_scdoub, hoc_vdoub, hoc_intfunc);
 	ivoc_help("help ?1 hcn12_ch /Users/peirui/BasalGangliaData/data/neurons/mechanisms/hcn12_ch.mod\n");
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
 static int _cvspth1 = 1;
 
static int _ode_spec1(_internalthreadargsproto_);
/*static int _ode_matsol1(_internalthreadargsproto_);*/
 
#define _MATELM1(_row,_col) *(_nrn_thread_getelm(static_cast<SparseObj*>(_so), _row + 1, _col + 1))
 
#define _RHS1(_arg) _rhs[_arg+1]
  
#define _linmat1  1
 static int _spth1 = 0;
 static neuron::container::field_index _slist1[4], _dlist1[4]; static double *_temp1;
 static int kin (void* _so, double* _rhs, _internalthreadargsproto_);
 
static int kin (void* _so, double* _rhs, _internalthreadargsproto_)
 {int _reset=0;
 {
   double _lqa ;
 double b_flux, f_flux, _term; int _i;
 {int _i; double _dt1 = 1.0/dt;
for(_i=1;_i<4;_i++){
  	_RHS1(_i) = -_dt1*(_ml->data(_iml, _slist1[_i]) - _ml->data(_iml, _dlist1[_i]));
	_MATELM1(_i, _i) = _dt1;
      
} }
 _lqa = pow( q10a , ( ( celsius - 22.0 ) / 10.0 ) ) ;
   rates ( _threadargscomma_ v ) ;
   /* ~ c <-> o ( alpha , beta )*/
 f_flux =  alpha * c ;
 b_flux =  beta * o ;
 _RHS1( 2) -= (f_flux - b_flux);
 _RHS1( 3) += (f_flux - b_flux);
 
 _term =  alpha ;
 _MATELM1( 2 ,2)  += _term;
 _MATELM1( 3 ,2)  -= _term;
 _term =  beta ;
 _MATELM1( 2 ,3)  -= _term;
 _MATELM1( 3 ,3)  += _term;
 /*REACTION*/
  /* ~ c <-> cac ( kon * _lqa * ai / bf , koff * _lqa * b / bf )*/
 f_flux =  kon * _lqa * ai / bf * c ;
 b_flux =  koff * _lqa * b / bf * cac ;
 _RHS1( 2) -= (f_flux - b_flux);
 _RHS1( 1) += (f_flux - b_flux);
 
 _term =  kon * _lqa * ai / bf ;
 _MATELM1( 2 ,2)  += _term;
 _MATELM1( 1 ,2)  -= _term;
 _term =  koff * _lqa * b / bf ;
 _MATELM1( 2 ,1)  -= _term;
 _MATELM1( 1 ,1)  += _term;
 /*REACTION*/
  /* ~ o <-> cao ( kon * _lqa * ai , koff * _lqa )*/
 f_flux =  kon * _lqa * ai * o ;
 b_flux =  koff * _lqa * cao ;
 _RHS1( 3) -= (f_flux - b_flux);
 
 _term =  kon * _lqa * ai ;
 _MATELM1( 3 ,3)  += _term;
 _term =  koff * _lqa ;
 _MATELM1( 3 ,0)  -= _term;
 /*REACTION*/
  /* ~ cac <-> cao ( alphaa , betaa )*/
 f_flux =  alphaa * cac ;
 b_flux =  betaa * cao ;
 _RHS1( 1) -= (f_flux - b_flux);
 
 _term =  alphaa ;
 _MATELM1( 1 ,1)  += _term;
 _term =  betaa ;
 _MATELM1( 1 ,0)  -= _term;
 /*REACTION*/
   /* c + cac + o + cao = 1.0 */
 _RHS1(0) =  1.0;
 _MATELM1(0, 0) = 1;
 _RHS1(0) -= cao ;
 _MATELM1(0, 3) = 1;
 _RHS1(0) -= o ;
 _MATELM1(0, 1) = 1;
 _RHS1(0) -= cac ;
 _MATELM1(0, 2) = 1;
 _RHS1(0) -= c ;
 /*CONSERVATION*/
   } return _reset;
 }
 
static int  rates ( _internalthreadargsprotocomma_ double _lv ) {
   double _lqv ;
 _lqv = pow( q10v , ( ( celsius - 22.0 ) / 10.0 ) ) ;
   if ( _lv > - 200.0 ) {
     alpha = a0 * _lqv / ( 1.0 + exp ( - ( _lv - ah - shift ) * ac ) ) ;
     beta = b0 * _lqv / ( 1.0 + exp ( - ( _lv - bh - shift ) * bc ) ) ;
     alphaa = aa0 * _lqv / ( 1.0 + exp ( - ( _lv - aah - shift ) * aac ) ) ;
     betaa = ba0 * _lqv / ( 1.0 + exp ( - ( _lv - bah - shift ) * bac ) ) ;
     }
   else {
     alpha = a0 * _lqv / ( 1.0 + exp ( - ( ( - 200.0 ) - ah - shift ) * ac ) ) ;
     beta = b0 * _lqv / ( 1.0 + exp ( - ( ( - 200.0 ) - bh - shift ) * bc ) ) ;
     alphaa = aa0 * _lqv / ( 1.0 + exp ( - ( ( - 200.0 ) - aah - shift ) * aac ) ) ;
     betaa = ba0 * _lqv / ( 1.0 + exp ( - ( ( - 200.0 ) - bah - shift ) * bac ) ) ;
     }
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
 
/*CVODE ode begin*/
 static int _ode_spec1(_internalthreadargsproto_) {
  int _reset=0;
  {
 double _lqa ;
 double b_flux, f_flux, _term; int _i;
 {int _i; for(_i=0;_i<4;_i++) _ml->data(_iml, _dlist1[_i]) = 0.0;}
 _lqa = pow( q10a , ( ( celsius - 22.0 ) / 10.0 ) ) ;
 rates ( _threadargscomma_ v ) ;
 /* ~ c <-> o ( alpha , beta )*/
 f_flux =  alpha * c ;
 b_flux =  beta * o ;
 Dc -= (f_flux - b_flux);
 Do += (f_flux - b_flux);
 
 /*REACTION*/
  /* ~ c <-> cac ( kon * _lqa * ai / bf , koff * _lqa * b / bf )*/
 f_flux =  kon * _lqa * ai / bf * c ;
 b_flux =  koff * _lqa * b / bf * cac ;
 Dc -= (f_flux - b_flux);
 Dcac += (f_flux - b_flux);
 
 /*REACTION*/
  /* ~ o <-> cao ( kon * _lqa * ai , koff * _lqa )*/
 f_flux =  kon * _lqa * ai * o ;
 b_flux =  koff * _lqa * cao ;
 Do -= (f_flux - b_flux);
 Dcao += (f_flux - b_flux);
 
 /*REACTION*/
  /* ~ cac <-> cao ( alphaa , betaa )*/
 f_flux =  alphaa * cac ;
 b_flux =  betaa * cao ;
 Dcac -= (f_flux - b_flux);
 Dcao += (f_flux - b_flux);
 
 /*REACTION*/
   /* c + cac + o + cao = 1.0 */
 /*CONSERVATION*/
   } return _reset;
 }
 
/*CVODE matsol*/
 static int _ode_matsol1(void* _so, double* _rhs, _internalthreadargsproto_) {int _reset=0;{
 double _lqa ;
 double b_flux, f_flux, _term; int _i;
   b_flux = f_flux = 0.;
 {int _i; double _dt1 = 1.0/dt;
for(_i=0;_i<4;_i++){
  	_RHS1(_i) = _dt1*(_ml->data(_iml, _dlist1[_i]));
	_MATELM1(_i, _i) = _dt1;
      
} }
 _lqa = pow( q10a , ( ( celsius - 22.0 ) / 10.0 ) ) ;
 rates ( _threadargscomma_ v ) ;
 /* ~ c <-> o ( alpha , beta )*/
 _term =  alpha ;
 _MATELM1( 2 ,2)  += _term;
 _MATELM1( 3 ,2)  -= _term;
 _term =  beta ;
 _MATELM1( 2 ,3)  -= _term;
 _MATELM1( 3 ,3)  += _term;
 /*REACTION*/
  /* ~ c <-> cac ( kon * _lqa * ai / bf , koff * _lqa * b / bf )*/
 _term =  kon * _lqa * ai / bf ;
 _MATELM1( 2 ,2)  += _term;
 _MATELM1( 1 ,2)  -= _term;
 _term =  koff * _lqa * b / bf ;
 _MATELM1( 2 ,1)  -= _term;
 _MATELM1( 1 ,1)  += _term;
 /*REACTION*/
  /* ~ o <-> cao ( kon * _lqa * ai , koff * _lqa )*/
 _term =  kon * _lqa * ai ;
 _MATELM1( 3 ,3)  += _term;
 _MATELM1( 0 ,3)  -= _term;
 _term =  koff * _lqa ;
 _MATELM1( 3 ,0)  -= _term;
 _MATELM1( 0 ,0)  += _term;
 /*REACTION*/
  /* ~ cac <-> cao ( alphaa , betaa )*/
 _term =  alphaa ;
 _MATELM1( 1 ,1)  += _term;
 _MATELM1( 0 ,1)  -= _term;
 _term =  betaa ;
 _MATELM1( 1 ,0)  -= _term;
 _MATELM1( 0 ,0)  += _term;
 /*REACTION*/
   /* c + cac + o + cao = 1.0 */
 /*CONSERVATION*/
   } return _reset;
 }
 
/*CVODE end*/
 
static int _ode_count(int _type){ return 4;}
 
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
  for (int _i=0; _i < 4; ++_i) {
    _pv[_i] = _nrn_mechanism_get_param_handle(_prop, _slist1[_i]);
    _pvdot[_i] = _nrn_mechanism_get_param_handle(_prop, _dlist1[_i]);
    _cvode_abstol(_atollist, _atol, _i);
  }
 }
 
static void _ode_matsol_instance1(_internalthreadargsproto_) {
 _cvode_sparse_thread(&(_thread[_cvspth1].literal_value<void*>()), 4, _dlist1, neuron::scopmath::row_view{_ml, _iml}, _ode_matsol1, _threadargs_);
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
 
static void _thread_cleanup(Datum* _thread) {
   _nrn_destroy_sparseobj_thread(static_cast<SparseObj*>(_thread[_spth1].get<void*>()));
   _nrn_destroy_sparseobj_thread(static_cast<SparseObj*>(_thread[_cvspth1].get<void*>()));
 }

static void initmodel(_internalthreadargsproto_) {
  int _i; double _save;{
  cao = cao0;
  cac = cac0;
  c = c0;
  o = o0;
 {
    _ss_sparse_thread(&(_thread[_spth1].literal_value<void*>()), 4, _slist1, _dlist1, neuron::scopmath::row_view{_ml, _iml}, &t, dt, kin, _linmat1, _threadargs_);
     if (secondorder) {
    int _i;
    for (_i = 0; _i < 4; ++_i) {
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
  PKAci = _ion_PKAci;
 initmodel(_threadargs_);
}
}

static double _nrn_current(_internalthreadargsprotocomma_ double _v) {
double _current=0.; v=_v;
{ {
   modulation_factor = modulation ( _threadargscomma_ PKAci , mod_pka_g_min , mod_pka_g_max , mod_pka_g_half , mod_pka_g_slope ) ;
   g = gbar * ( o + cao * gca ) * modulation_factor ;
   i = g * ( v - ehcn ) ;
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
  PKAci = _ion_PKAci;
 {  sparse_thread(&(_thread[_spth1].literal_value<void*>()), 4, _slist1, _dlist1, neuron::scopmath::row_view{_ml, _iml}, &t, dt, kin, _linmat1, _threadargs_);
     if (secondorder) {
    int _i;
    for (_i = 0; _i < 4; ++_i) {
      _ml->data(_iml, _slist1[_i]) += dt*_ml->data(_iml, _dlist1[_i]);
    }}
 }}}
 dt = _dtsav;
}

static void terminal(){}

static void _initlists(){
 int _i; static int _first = 1;
  if (!_first) return;
 _slist1[0] = {cao_columnindex, 0};  _dlist1[0] = {Dcao_columnindex, 0};
 _slist1[1] = {cac_columnindex, 0};  _dlist1[1] = {Dcac_columnindex, 0};
 _slist1[2] = {c_columnindex, 0};  _dlist1[2] = {Dc_columnindex, 0};
 _slist1[3] = {o_columnindex, 0};  _dlist1[3] = {Do_columnindex, 0};
_first = 0;
}

#if NMODL_TEXT
static void register_nmodl_text_and_filename(int mech_type) {
    const char* nmodl_filename = "/Users/peirui/BasalGangliaData/data/neurons/mechanisms/hcn12_ch.mod";
    const char* nmodl_file_text = 
  "NEURON {\n"
  "	SUFFIX hcn12_ch\n"
  "	NONSPECIFIC_CURRENT i\n"
  "	RANGE i, ehcn, g, gbar\n"
  "	GLOBAL a0, b0, ah, bh, ac, bc, aa0, ba0\n"
  "	GLOBAL aa0, ba0, aah, bah, aac, bac\n"
  "	GLOBAL kon, koff, b, bf, ai, gca, shift\n"
  "\n"
  "    USEION PKAc READ PKAci VALENCE 0\n"
  "    RANGE mod_pka_g_min, mod_pka_g_max, mod_pka_g_half, mod_pka_g_slope \n"
  "    RANGE modulation_factor\n"
  "					       \n"
  "}\n"
  "\n"
  "UNITS {\n"
  "	(mV) = (millivolt)\n"
  "	(molar) = (1/liter)\n"
  "	(mM) = (millimolar)\n"
  "	(mA) = (milliamp)\n"
  "	(S) = (siemens)\n"
  "}\n"
  "\n"
  "PARAMETER {\n"
  "	gbar    = 1		(S/cm2)\n"
  "	ehcn    = -20		(mV)\n"
  "	a0      = .006		(/ms)	: parameters for alpha and beta\n"
  "	b0      = .0008		(/ms)\n"
  "	ah      = -96		(mV)\n"
  "	bh      = -51.7		(mV)\n"
  "	ac      = -.155		(/mV)\n"
  "	bc      = .144		(/mV)\n"
  "	aa0     = .0006		(/ms)	: parameters for alphaa and betaa\n"
  "	ba0     = .004		(/ms)\n"
  "	aah     = -94.2		(mV)\n"
  "	bah     = -35.5		(mV)\n"
  "	aac     = -.075		(/mV)\n"
  "	bac     = .144		(/mV)\n"
  "	kon     = 30		(/mM-ms) : cyclic AMP binding parameters\n"
  "	koff    = 4.5e-05	(/ms)\n"
  "	b       = 80\n"
  "	bf      = 8.94\n"
  "	ai	= 1e-05		(mM)	:concentration cyclic AMP\n"
  "	gca     = 1			: relative conductance of the bound state\n"
  "	shift   = -17		(mV)	: shift in voltage dependence\n"
  "	q10v    = 4                     : q10 value from Magee 1998\n"
  "	q10a    = 1.5			: estimated q10 for the cAMP binding reaction\n"
  "	celsius			(degC)\n"
  "\n"
  "    mod_pka_g_min = 1 (1)\n"
  "    mod_pka_g_max = 1 (1)\n"
  "    mod_pka_g_half = 0.000100 (mM)\n"
  "    mod_pka_g_slope = 0.01 (mM)	\n"
  "\n"
  "	\n"
  "}\n"
  "\n"
  "ASSIGNED {\n"
  "	v	(mV)\n"
  "	g	(S/cm2)\n"
  "	i	(mA/cm2)\n"
  "	alpha	(/ms)\n"
  "	beta    (/ms)\n"
  "	alphaa	(/ms)\n"
  "	betaa	(/ms)\n"
  "\n"
  "	PKAci (mM)\n"
  "	modulation_factor (1)\n"
  "}\n"
  "\n"
  "STATE {\n"
  "	c\n"
  "	cac\n"
  "	o\n"
  "	cao\n"
  "}\n"
  "\n"
  "INITIAL {\n"
  "	SOLVE kin STEADYSTATE sparse\n"
  "}\n"
  "\n"
  "BREAKPOINT {\n"
  "     SOLVE kin METHOD sparse\n"
  "    modulation_factor=modulation(PKAci, mod_pka_g_min, mod_pka_g_max, mod_pka_g_half, mod_pka_g_slope)	   \n"
  "	   \n"
  "	g = gbar*(o + cao*gca)*modulation_factor\n"
  "	i = g*(v-ehcn)\n"
  "}\n"
  "\n"
  "KINETIC kin {\n"
  "	LOCAL qa\n"
  "	qa = q10a^((celsius-22 (degC))/10 (degC))\n"
  "	rates(v)\n"
  "	~ c <-> o       (alpha, beta)\n"
  "	~ c <-> cac     (kon*qa*ai/bf,koff*qa*b/bf)\n"
  "	~ o <-> cao     (kon*qa*ai, koff*qa)\n"
  "	~ cac <-> cao   (alphaa, betaa)\n"
  "	CONSERVE c + cac + o + cao = 1\n"
  "}\n"
  "\n"
  "PROCEDURE rates(v(mV)) {\n"
  "	LOCAL qv\n"
  "	qv = q10v^((celsius-22 (degC))/10 (degC))\n"
  "	if (v > -200) {\n"
  "		alpha = a0*qv / (1 + exp(-(v-ah-shift)*ac))\n"
  "		beta = b0*qv / (1 + exp(-(v-bh-shift)*bc))\n"
  "		alphaa = aa0*qv / (1 + exp(-(v-aah-shift)*aac))\n"
  "		betaa = ba0*qv / (1 + exp(-(v-bah-shift)*bac))\n"
  "	} else {\n"
  "		alpha = a0*qv / (1 + exp(-((-200)-ah-shift)*ac))\n"
  "		beta = b0*qv / (1 + exp(-((-200)-bh-shift)*bc))\n"
  "		alphaa = aa0*qv / (1 + exp(-((-200)-aah-shift)*aac))\n"
  "		betaa = ba0*qv / (1 + exp(-((-200)-bah-shift)*bac))\n"
  "	}\n"
  "}\n"
  "\n"
  "FUNCTION modulation(conc (mM), mod_min (1), mod_max (1), mod_half (mM), mod_slope (mM)) (1) {\n"
  "    : returns modulation factor\n"
  "    modulation = mod_min + (mod_max-mod_min) / (1 + exp(-(conc - mod_half)/mod_slope))\n"
  "}\n"
  "\n"
  "\n"
  "COMMENT\n"
  "\n"
  "Josh Held's adaptation to suit HCN1+2.  12/22/2003\n"
  "\n"
  "****\n"
  "Kinetic model of HCN2 channel gating from Wang et al 2002.\n"
  "\n"
  "In this model channel opening is coupled to a change in the affinity of the cyclic nucleotide binding domain for cAMP which is manifest as a shift in the activation curve toward more positive potentials.  This model explains the slow activation kinetics of Ih associated with low concentrations of cAMP.\n"
  "\n"
  "For further details email Matt Nolan at mfnolan@fido.cpmc.columbia.edu.\n"
  "\n"
  "Reference\n"
  "\n"
  "Wang J., Chen S., Nolan M.F. and Siegelbaum S.A. (2002). Activity-dependent regulation of HCN pacemaker channels by cyclicAMP: signalling through dynamic allosteric coupling. Neuron 36, 1-20.\n"
  "****\n"
  "\n"
  "ENDCOMMENT\n"
  ;
    hoc_reg_nmodl_filename(mech_type, nmodl_filename);
    hoc_reg_nmodl_text(mech_type, nmodl_file_text);
}
#endif
