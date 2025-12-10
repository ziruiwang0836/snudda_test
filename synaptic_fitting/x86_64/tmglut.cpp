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
static constexpr auto number_of_datum_variables = 7;
static constexpr auto number_of_floating_point_variables = 51;
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
 
#define nrn_init _nrn_init__tmGlut
#define _nrn_initial _nrn_initial__tmGlut
#define nrn_cur _nrn_cur__tmGlut
#define _nrn_current _nrn_current__tmGlut
#define nrn_jacob _nrn_jacob__tmGlut
#define nrn_state _nrn_state__tmGlut
#define _net_receive _net_receive__tmGlut 
#define state state__tmGlut 
 
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
#define tau1_ampa _ml->template fpfield<0>(_iml)
#define tau1_ampa_columnindex 0
#define tau2_ampa _ml->template fpfield<1>(_iml)
#define tau2_ampa_columnindex 1
#define tau1_nmda _ml->template fpfield<2>(_iml)
#define tau1_nmda_columnindex 2
#define tau2_nmda _ml->template fpfield<3>(_iml)
#define tau2_nmda_columnindex 3
#define nmda_ratio _ml->template fpfield<4>(_iml)
#define nmda_ratio_columnindex 4
#define e _ml->template fpfield<5>(_iml)
#define e_columnindex 5
#define tau _ml->template fpfield<6>(_iml)
#define tau_columnindex 6
#define tauR _ml->template fpfield<7>(_iml)
#define tauR_columnindex 7
#define tauF _ml->template fpfield<8>(_iml)
#define tauF_columnindex 8
#define U _ml->template fpfield<9>(_iml)
#define U_columnindex 9
#define u0 _ml->template fpfield<10>(_iml)
#define u0_columnindex 10
#define ca_ratio_ampa _ml->template fpfield<11>(_iml)
#define ca_ratio_ampa_columnindex 11
#define ca_ratio_nmda _ml->template fpfield<12>(_iml)
#define ca_ratio_nmda_columnindex 12
#define mg _ml->template fpfield<13>(_iml)
#define mg_columnindex 13
#define mod_pka_g_ampa_min _ml->template fpfield<14>(_iml)
#define mod_pka_g_ampa_min_columnindex 14
#define mod_pka_g_ampa_max _ml->template fpfield<15>(_iml)
#define mod_pka_g_ampa_max_columnindex 15
#define mod_pka_g_ampa_half _ml->template fpfield<16>(_iml)
#define mod_pka_g_ampa_half_columnindex 16
#define mod_pka_g_ampa_slope _ml->template fpfield<17>(_iml)
#define mod_pka_g_ampa_slope_columnindex 17
#define mod_pka_g_nmda_min _ml->template fpfield<18>(_iml)
#define mod_pka_g_nmda_min_columnindex 18
#define mod_pka_g_nmda_max _ml->template fpfield<19>(_iml)
#define mod_pka_g_nmda_max_columnindex 19
#define mod_pka_g_nmda_half _ml->template fpfield<20>(_iml)
#define mod_pka_g_nmda_half_columnindex 20
#define mod_pka_g_nmda_slope _ml->template fpfield<21>(_iml)
#define mod_pka_g_nmda_slope_columnindex 21
#define failRate _ml->template fpfield<22>(_iml)
#define failRate_columnindex 22
#define use_stp _ml->template fpfield<23>(_iml)
#define use_stp_columnindex 23
#define i _ml->template fpfield<24>(_iml)
#define i_columnindex 24
#define i_ampa _ml->template fpfield<25>(_iml)
#define i_ampa_columnindex 25
#define i_nmda _ml->template fpfield<26>(_iml)
#define i_nmda_columnindex 26
#define g _ml->template fpfield<27>(_iml)
#define g_columnindex 27
#define g_ampa _ml->template fpfield<28>(_iml)
#define g_ampa_columnindex 28
#define g_nmda _ml->template fpfield<29>(_iml)
#define g_nmda_columnindex 29
#define modulation_factor_ampa _ml->template fpfield<30>(_iml)
#define modulation_factor_ampa_columnindex 30
#define modulation_factor_nmda _ml->template fpfield<31>(_iml)
#define modulation_factor_nmda_columnindex 31
#define modulation_factor_fail _ml->template fpfield<32>(_iml)
#define modulation_factor_fail_columnindex 32
#define A_ampa _ml->template fpfield<33>(_iml)
#define A_ampa_columnindex 33
#define B_ampa _ml->template fpfield<34>(_iml)
#define B_ampa_columnindex 34
#define A_nmda _ml->template fpfield<35>(_iml)
#define A_nmda_columnindex 35
#define B_nmda _ml->template fpfield<36>(_iml)
#define B_nmda_columnindex 36
#define ical _ml->template fpfield<37>(_iml)
#define ical_columnindex 37
#define ical_ampa _ml->template fpfield<38>(_iml)
#define ical_ampa_columnindex 38
#define ical_nmda _ml->template fpfield<39>(_iml)
#define ical_nmda_columnindex 39
#define factor_ampa _ml->template fpfield<40>(_iml)
#define factor_ampa_columnindex 40
#define factor_nmda _ml->template fpfield<41>(_iml)
#define factor_nmda_columnindex 41
#define x _ml->template fpfield<42>(_iml)
#define x_columnindex 42
#define PKAci _ml->template fpfield<43>(_iml)
#define PKAci_columnindex 43
#define DA_ampa _ml->template fpfield<44>(_iml)
#define DA_ampa_columnindex 44
#define DB_ampa _ml->template fpfield<45>(_iml)
#define DB_ampa_columnindex 45
#define DA_nmda _ml->template fpfield<46>(_iml)
#define DA_nmda_columnindex 46
#define DB_nmda _ml->template fpfield<47>(_iml)
#define DB_nmda_columnindex 47
#define v _ml->template fpfield<48>(_iml)
#define v_columnindex 48
#define _g _ml->template fpfield<49>(_iml)
#define _g_columnindex 49
#define _tsav _ml->template fpfield<50>(_iml)
#define _tsav_columnindex 50
#define _nd_area *_ml->dptr_field<0>(_iml)
#define _ion_PKAci *(_ml->dptr_field<2>(_iml))
#define _p_ion_PKAci static_cast<neuron::container::data_handle<double>>(_ppvar[2])
#define _ion_PKAco *(_ml->dptr_field<3>(_iml))
#define _p_ion_PKAco static_cast<neuron::container::data_handle<double>>(_ppvar[3])
#define _ion_ical *(_ml->dptr_field<4>(_iml))
#define _p_ion_ical static_cast<neuron::container::data_handle<double>>(_ppvar[4])
#define _ion_dicaldv *(_ml->dptr_field<5>(_iml))
 
 //RANDOM variables 
 #define release_probability	(nrnran123_State*)_ppvar[6].get<void*>()
 #define _p_release_probability _ppvar[6].literal_value<void*>()
 
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
#define modulation modulation_tmGlut
#define urand urand_tmGlut
 extern double modulation( _internalthreadargsprotocomma_ double , double , double , double , double );
 extern double urand( _internalthreadargsproto_ );
 /* declare global and static user variables */
 #define gind 0
 #define _gth 0
#define failRateScaling failRateScaling_tmGlut
 double failRateScaling = 0;
#define mod_pka_fail_slope mod_pka_fail_slope_tmGlut
 double mod_pka_fail_slope = 0.01;
#define mod_pka_fail_half mod_pka_fail_half_tmGlut
 double mod_pka_fail_half = 0.0001;
#define mod_pka_fail_max mod_pka_fail_max_tmGlut
 double mod_pka_fail_max = 0;
#define mod_pka_fail_min mod_pka_fail_min_tmGlut
 double mod_pka_fail_min = 0;
 /* some parameters have upper and lower limits */
 static HocParmLimits _hoc_parm_limits[] = {
 {"U", 0, 1},
 {"u0", 0, 1},
 {0, 0, 0}
};
 static HocParmUnits _hoc_parm_units[] = {
 {"mod_pka_fail_min_tmGlut", "1"},
 {"mod_pka_fail_max_tmGlut", "1"},
 {"mod_pka_fail_half_tmGlut", "mM"},
 {"mod_pka_fail_slope_tmGlut", "mM"},
 {"tau1_ampa", "ms"},
 {"tau2_ampa", "ms"},
 {"tau1_nmda", "ms"},
 {"tau2_nmda", "ms"},
 {"nmda_ratio", "1"},
 {"e", "mV"},
 {"tau", "ms"},
 {"tauR", "ms"},
 {"tauF", "ms"},
 {"U", "1"},
 {"u0", "1"},
 {"mg", "mM"},
 {"mod_pka_g_ampa_min", "1"},
 {"mod_pka_g_ampa_max", "1"},
 {"mod_pka_g_ampa_half", "mM"},
 {"mod_pka_g_ampa_slope", "mM"},
 {"mod_pka_g_nmda_min", "1"},
 {"mod_pka_g_nmda_max", "1"},
 {"mod_pka_g_nmda_half", "mM"},
 {"mod_pka_g_nmda_slope", "mM"},
 {"A_ampa", "uS"},
 {"B_ampa", "uS"},
 {"A_nmda", "uS"},
 {"B_nmda", "uS"},
 {"i", "nA"},
 {"i_ampa", "nA"},
 {"i_nmda", "nA"},
 {"g", "uS"},
 {"g_ampa", "uS"},
 {"g_nmda", "uS"},
 {"modulation_factor_ampa", "1"},
 {"modulation_factor_nmda", "1"},
 {"modulation_factor_fail", "1"},
 {0, 0}
};
 static double A_nmda0 = 0;
 static double A_ampa0 = 0;
 static double B_nmda0 = 0;
 static double B_ampa0 = 0;
 static double delta_t = 0.01;
 /* connect global user variables to hoc */
 static DoubScal hoc_scdoub[] = {
 {"mod_pka_fail_min_tmGlut", &mod_pka_fail_min_tmGlut},
 {"mod_pka_fail_max_tmGlut", &mod_pka_fail_max_tmGlut},
 {"mod_pka_fail_half_tmGlut", &mod_pka_fail_half_tmGlut},
 {"mod_pka_fail_slope_tmGlut", &mod_pka_fail_slope_tmGlut},
 {"failRateScaling_tmGlut", &failRateScaling_tmGlut},
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
 
#define _cvode_ieq _ppvar[7].literal_value<int>()
 static void _ode_matsol_instance1(_internalthreadargsproto_);
 /* connect range variables in _p that hoc is supposed to know about */
 static const char *_mechanism[] = {
 "7.7.0",
"tmGlut",
 "tau1_ampa",
 "tau2_ampa",
 "tau1_nmda",
 "tau2_nmda",
 "nmda_ratio",
 "e",
 "tau",
 "tauR",
 "tauF",
 "U",
 "u0",
 "ca_ratio_ampa",
 "ca_ratio_nmda",
 "mg",
 "mod_pka_g_ampa_min",
 "mod_pka_g_ampa_max",
 "mod_pka_g_ampa_half",
 "mod_pka_g_ampa_slope",
 "mod_pka_g_nmda_min",
 "mod_pka_g_nmda_max",
 "mod_pka_g_nmda_half",
 "mod_pka_g_nmda_slope",
 "failRate",
 "use_stp",
 0,
 "i",
 "i_ampa",
 "i_nmda",
 "g",
 "g_ampa",
 "g_nmda",
 "modulation_factor_ampa",
 "modulation_factor_nmda",
 "modulation_factor_fail",
 0,
 "A_ampa",
 "B_ampa",
 "A_nmda",
 "B_nmda",
 0,
 0};
 static Symbol* _PKAc_sym;
 static Symbol* _cal_sym;
 
 /* Used by NrnProperty */
 static _nrn_mechanism_std_vector<double> _parm_default{
     1.1, /* tau1_ampa */
     5.75, /* tau2_ampa */
     2.76, /* tau1_nmda */
     115.5, /* tau2_nmda */
     0.5, /* nmda_ratio */
     0, /* e */
     3, /* tau */
     100, /* tauR */
     800, /* tauF */
     0.3, /* U */
     0, /* u0 */
     0.005, /* ca_ratio_ampa */
     0.1, /* ca_ratio_nmda */
     1, /* mg */
     1, /* mod_pka_g_ampa_min */
     1, /* mod_pka_g_ampa_max */
     0.0001, /* mod_pka_g_ampa_half */
     0.01, /* mod_pka_g_ampa_slope */
     1, /* mod_pka_g_nmda_min */
     1, /* mod_pka_g_nmda_max */
     0.0001, /* mod_pka_g_nmda_half */
     0.01, /* mod_pka_g_nmda_slope */
     0, /* failRate */
     1, /* use_stp */
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
   _ppvar = nrn_prop_datum_alloc(_mechtype, 8, _prop);
    _nrn_mechanism_access_dparam(_prop) = _ppvar;
     _nrn_mechanism_cache_instance _ml_real{_prop};
    auto* const _ml = &_ml_real;
    size_t const _iml{};
    assert(_nrn_mechanism_get_num_vars(_prop) == 51);
 	/*initialize range parameters*/
 	tau1_ampa = _parm_default[0]; /* 1.1 */
 	tau2_ampa = _parm_default[1]; /* 5.75 */
 	tau1_nmda = _parm_default[2]; /* 2.76 */
 	tau2_nmda = _parm_default[3]; /* 115.5 */
 	nmda_ratio = _parm_default[4]; /* 0.5 */
 	e = _parm_default[5]; /* 0 */
 	tau = _parm_default[6]; /* 3 */
 	tauR = _parm_default[7]; /* 100 */
 	tauF = _parm_default[8]; /* 800 */
 	U = _parm_default[9]; /* 0.3 */
 	u0 = _parm_default[10]; /* 0 */
 	ca_ratio_ampa = _parm_default[11]; /* 0.005 */
 	ca_ratio_nmda = _parm_default[12]; /* 0.1 */
 	mg = _parm_default[13]; /* 1 */
 	mod_pka_g_ampa_min = _parm_default[14]; /* 1 */
 	mod_pka_g_ampa_max = _parm_default[15]; /* 1 */
 	mod_pka_g_ampa_half = _parm_default[16]; /* 0.0001 */
 	mod_pka_g_ampa_slope = _parm_default[17]; /* 0.01 */
 	mod_pka_g_nmda_min = _parm_default[18]; /* 1 */
 	mod_pka_g_nmda_max = _parm_default[19]; /* 1 */
 	mod_pka_g_nmda_half = _parm_default[20]; /* 0.0001 */
 	mod_pka_g_nmda_slope = _parm_default[21]; /* 0.01 */
 	failRate = _parm_default[22]; /* 0 */
 	use_stp = _parm_default[23]; /* 1 */
  }
 	 assert(_nrn_mechanism_get_num_vars(_prop) == 51);
 	_nrn_mechanism_access_dparam(_prop) = _ppvar;
 	/*connect ionic variables to this model*/
 prop_ion = need_memb(_PKAc_sym);
 nrn_promote(prop_ion, 1, 0);
 	_ppvar[2] = _nrn_mechanism_get_param_handle(prop_ion, 1); /* PKAci */
 	_ppvar[3] = _nrn_mechanism_get_param_handle(prop_ion, 2); /* PKAco */
 prop_ion = need_memb(_cal_sym);
 	_ppvar[4] = _nrn_mechanism_get_param_handle(prop_ion, 3); /* ical */
 	_ppvar[5] = _nrn_mechanism_get_param_handle(prop_ion, 4); /* _ion_dicaldv */
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

 extern "C" void _tmglut_reg() {
	int _vectorized = 1;
  _initlists();
 	ion_reg("PKAc", 0.0);
 	ion_reg("cal", 2.0);
 	_PKAc_sym = hoc_lookup("PKAc_ion");
 	_cal_sym = hoc_lookup("cal_ion");
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
                                       _nrn_mechanism_field<double>{"tau1_ampa"} /* 0 */,
                                       _nrn_mechanism_field<double>{"tau2_ampa"} /* 1 */,
                                       _nrn_mechanism_field<double>{"tau1_nmda"} /* 2 */,
                                       _nrn_mechanism_field<double>{"tau2_nmda"} /* 3 */,
                                       _nrn_mechanism_field<double>{"nmda_ratio"} /* 4 */,
                                       _nrn_mechanism_field<double>{"e"} /* 5 */,
                                       _nrn_mechanism_field<double>{"tau"} /* 6 */,
                                       _nrn_mechanism_field<double>{"tauR"} /* 7 */,
                                       _nrn_mechanism_field<double>{"tauF"} /* 8 */,
                                       _nrn_mechanism_field<double>{"U"} /* 9 */,
                                       _nrn_mechanism_field<double>{"u0"} /* 10 */,
                                       _nrn_mechanism_field<double>{"ca_ratio_ampa"} /* 11 */,
                                       _nrn_mechanism_field<double>{"ca_ratio_nmda"} /* 12 */,
                                       _nrn_mechanism_field<double>{"mg"} /* 13 */,
                                       _nrn_mechanism_field<double>{"mod_pka_g_ampa_min"} /* 14 */,
                                       _nrn_mechanism_field<double>{"mod_pka_g_ampa_max"} /* 15 */,
                                       _nrn_mechanism_field<double>{"mod_pka_g_ampa_half"} /* 16 */,
                                       _nrn_mechanism_field<double>{"mod_pka_g_ampa_slope"} /* 17 */,
                                       _nrn_mechanism_field<double>{"mod_pka_g_nmda_min"} /* 18 */,
                                       _nrn_mechanism_field<double>{"mod_pka_g_nmda_max"} /* 19 */,
                                       _nrn_mechanism_field<double>{"mod_pka_g_nmda_half"} /* 20 */,
                                       _nrn_mechanism_field<double>{"mod_pka_g_nmda_slope"} /* 21 */,
                                       _nrn_mechanism_field<double>{"failRate"} /* 22 */,
                                       _nrn_mechanism_field<double>{"use_stp"} /* 23 */,
                                       _nrn_mechanism_field<double>{"i"} /* 24 */,
                                       _nrn_mechanism_field<double>{"i_ampa"} /* 25 */,
                                       _nrn_mechanism_field<double>{"i_nmda"} /* 26 */,
                                       _nrn_mechanism_field<double>{"g"} /* 27 */,
                                       _nrn_mechanism_field<double>{"g_ampa"} /* 28 */,
                                       _nrn_mechanism_field<double>{"g_nmda"} /* 29 */,
                                       _nrn_mechanism_field<double>{"modulation_factor_ampa"} /* 30 */,
                                       _nrn_mechanism_field<double>{"modulation_factor_nmda"} /* 31 */,
                                       _nrn_mechanism_field<double>{"modulation_factor_fail"} /* 32 */,
                                       _nrn_mechanism_field<double>{"A_ampa"} /* 33 */,
                                       _nrn_mechanism_field<double>{"B_ampa"} /* 34 */,
                                       _nrn_mechanism_field<double>{"A_nmda"} /* 35 */,
                                       _nrn_mechanism_field<double>{"B_nmda"} /* 36 */,
                                       _nrn_mechanism_field<double>{"ical"} /* 37 */,
                                       _nrn_mechanism_field<double>{"ical_ampa"} /* 38 */,
                                       _nrn_mechanism_field<double>{"ical_nmda"} /* 39 */,
                                       _nrn_mechanism_field<double>{"factor_ampa"} /* 40 */,
                                       _nrn_mechanism_field<double>{"factor_nmda"} /* 41 */,
                                       _nrn_mechanism_field<double>{"x"} /* 42 */,
                                       _nrn_mechanism_field<double>{"PKAci"} /* 43 */,
                                       _nrn_mechanism_field<double>{"DA_ampa"} /* 44 */,
                                       _nrn_mechanism_field<double>{"DB_ampa"} /* 45 */,
                                       _nrn_mechanism_field<double>{"DA_nmda"} /* 46 */,
                                       _nrn_mechanism_field<double>{"DB_nmda"} /* 47 */,
                                       _nrn_mechanism_field<double>{"v"} /* 48 */,
                                       _nrn_mechanism_field<double>{"_g"} /* 49 */,
                                       _nrn_mechanism_field<double>{"_tsav"} /* 50 */,
                                       _nrn_mechanism_field<double*>{"_nd_area", "area"} /* 0 */,
                                       _nrn_mechanism_field<Point_process*>{"_pntproc", "pntproc"} /* 1 */,
                                       _nrn_mechanism_field<double*>{"_ion_PKAci", "PKAc_ion"} /* 2 */,
                                       _nrn_mechanism_field<double*>{"_ion_PKAco", "PKAc_ion"} /* 3 */,
                                       _nrn_mechanism_field<double*>{"_ion_ical", "cal_ion"} /* 4 */,
                                       _nrn_mechanism_field<double*>{"_ion_dicaldv", "cal_ion"} /* 5 */,
                                       _nrn_mechanism_field<void*>{"release_probability", "random"} /* 6 */,
                                       _nrn_mechanism_field<int>{"_cvode_ieq", "cvodeieq"} /* 7 */);
  hoc_register_prop_size(_mechtype, 51, 8);
  hoc_register_dparam_semantics(_mechtype, 0, "area");
  hoc_register_dparam_semantics(_mechtype, 1, "pntproc");
  hoc_register_dparam_semantics(_mechtype, 2, "PKAc_ion");
  hoc_register_dparam_semantics(_mechtype, 3, "PKAc_ion");
  hoc_register_dparam_semantics(_mechtype, 4, "cal_ion");
  hoc_register_dparam_semantics(_mechtype, 5, "cal_ion");
  hoc_register_dparam_semantics(_mechtype, 6, "random");
  hoc_register_dparam_semantics(_mechtype, 7, "cvodeieq");
 	hoc_register_cvode(_mechtype, _ode_count, _ode_map, _ode_spec, _ode_matsol);
 	hoc_register_tolerance(_mechtype, _hoc_state_tol, &_atollist);
 pnt_receive[_mechtype] = _net_receive;
 pnt_receive_init[_mechtype] = _net_init;
 pnt_receive_size[_mechtype] = 5;
 
    hoc_register_var(hoc_scdoub, hoc_vdoub, hoc_intfunc);
 	ivoc_help("help ?1 tmGlut /cfs/klemming/home/m/metog/BasalGangliaData/data/neurons/mechanisms/tmglut.mod\n");
 hoc_register_limits(_mechtype, _hoc_parm_limits);
 hoc_register_units(_mechtype, _hoc_parm_units);
 }
static int _reset;
static const char *modelname = "Glutamatergic synapse with short-term plasticity (stp)";

static int error;
static int _ninits = 0;
static int _match_recurse=1;
static void _modl_cleanup(){ _match_recurse=1;}
 
static int _ode_spec1(_internalthreadargsproto_);
/*static int _ode_matsol1(_internalthreadargsproto_);*/
 static neuron::container::field_index _slist1[4], _dlist1[4];
 static int state(_internalthreadargsproto_);
 
/*CVODE*/
 static int _ode_spec1 (_internalthreadargsproto_) {int _reset = 0; {
   DA_ampa = - A_ampa / tau1_ampa ;
   DB_ampa = - B_ampa / tau2_ampa ;
   DA_nmda = - A_nmda / tau1_nmda ;
   DB_nmda = - B_nmda / tau2_nmda ;
   }
 return _reset;
}
 static int _ode_matsol1 (_internalthreadargsproto_) {
 DA_ampa = DA_ampa  / (1. - dt*( ( - 1.0 ) / tau1_ampa )) ;
 DB_ampa = DB_ampa  / (1. - dt*( ( - 1.0 ) / tau2_ampa )) ;
 DA_nmda = DA_nmda  / (1. - dt*( ( - 1.0 ) / tau1_nmda )) ;
 DB_nmda = DB_nmda  / (1. - dt*( ( - 1.0 ) / tau2_nmda )) ;
  return 0;
}
 /*END CVODE*/
 static int state (_internalthreadargsproto_) { {
    A_ampa = A_ampa + (1. - exp(dt*(( - 1.0 ) / tau1_ampa)))*(- ( 0.0 ) / ( ( - 1.0 ) / tau1_ampa ) - A_ampa) ;
    B_ampa = B_ampa + (1. - exp(dt*(( - 1.0 ) / tau2_ampa)))*(- ( 0.0 ) / ( ( - 1.0 ) / tau2_ampa ) - B_ampa) ;
    A_nmda = A_nmda + (1. - exp(dt*(( - 1.0 ) / tau1_nmda)))*(- ( 0.0 ) / ( ( - 1.0 ) / tau1_nmda ) - A_nmda) ;
    B_nmda = B_nmda + (1. - exp(dt*(( - 1.0 ) / tau2_nmda)))*(- ( 0.0 ) / ( ( - 1.0 ) / tau2_nmda ) - B_nmda) ;
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
   double _lweight_ampa , _lweight_nmda ;
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
     if ( use_stp > 0.0 ) {
       _lweight_ampa = _args[0] * x * _args[3] / U ;
       }
     else {
       _lweight_ampa = _args[0] ;
       }
     _lweight_nmda = _lweight_ampa * nmda_ratio ;
       if (nrn_netrec_state_adjust && !cvode_active_){
    /* discon state adjustment for cnexp case (rate uses no local variable) */
    double __state = A_ampa;
    double __primary = (A_ampa + _lweight_ampa * factor_ampa) - __state;
     __primary += ( 1. - exp( 0.5*dt*( ( - 1.0 ) / tau1_ampa ) ) )*( - ( 0.0 ) / ( ( - 1.0 ) / tau1_ampa ) - __primary );
    A_ampa += __primary;
  } else {
 A_ampa = A_ampa + _lweight_ampa * factor_ampa ;
       }
   if (nrn_netrec_state_adjust && !cvode_active_){
    /* discon state adjustment for cnexp case (rate uses no local variable) */
    double __state = B_ampa;
    double __primary = (B_ampa + _lweight_ampa * factor_ampa) - __state;
     __primary += ( 1. - exp( 0.5*dt*( ( - 1.0 ) / tau2_ampa ) ) )*( - ( 0.0 ) / ( ( - 1.0 ) / tau2_ampa ) - __primary );
    B_ampa += __primary;
  } else {
 B_ampa = B_ampa + _lweight_ampa * factor_ampa ;
       }
   if (nrn_netrec_state_adjust && !cvode_active_){
    /* discon state adjustment for cnexp case (rate uses no local variable) */
    double __state = A_nmda;
    double __primary = (A_nmda + _lweight_nmda * factor_nmda) - __state;
     __primary += ( 1. - exp( 0.5*dt*( ( - 1.0 ) / tau1_nmda ) ) )*( - ( 0.0 ) / ( ( - 1.0 ) / tau1_nmda ) - __primary );
    A_nmda += __primary;
  } else {
 A_nmda = A_nmda + _lweight_nmda * factor_nmda ;
       }
   if (nrn_netrec_state_adjust && !cvode_active_){
    /* discon state adjustment for cnexp case (rate uses no local variable) */
    double __state = B_nmda;
    double __primary = (B_nmda + _lweight_nmda * factor_nmda) - __state;
     __primary += ( 1. - exp( 0.5*dt*( ( - 1.0 ) / tau2_nmda ) ) )*( - ( 0.0 ) / ( ( - 1.0 ) / tau2_nmda ) - __primary );
    B_nmda += __primary;
  } else {
 B_nmda = B_nmda + _lweight_nmda * factor_nmda ;
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
  A_nmda = A_nmda0;
  A_ampa = A_ampa0;
  B_nmda = B_nmda0;
  B_ampa = B_ampa0;
 {
   double _ltp_ampa , _ltp_nmda ;
 A_ampa = 0.0 ;
   B_ampa = 0.0 ;
   _ltp_ampa = ( tau1_ampa * tau2_ampa ) / ( tau2_ampa - tau1_ampa ) * log ( tau2_ampa / tau1_ampa ) ;
   factor_ampa = - exp ( - _ltp_ampa / tau1_ampa ) + exp ( - _ltp_ampa / tau2_ampa ) ;
   factor_ampa = 1.0 / factor_ampa ;
   A_nmda = 0.0 ;
   B_nmda = 0.0 ;
   _ltp_nmda = ( tau1_nmda * tau2_nmda ) / ( tau2_nmda - tau1_nmda ) * log ( tau2_nmda / tau1_nmda ) ;
   factor_nmda = - exp ( - _ltp_nmda / tau1_nmda ) + exp ( - _ltp_nmda / tau2_nmda ) ;
   factor_nmda = 1.0 / factor_nmda ;
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
   double _litot_nmda , _litot_ampa , _lmggate ;
 modulation_factor_ampa = modulation ( _threadargscomma_ PKAci , mod_pka_g_ampa_min , mod_pka_g_ampa_max , mod_pka_g_ampa_half , mod_pka_g_ampa_slope ) ;
   modulation_factor_nmda = modulation ( _threadargscomma_ PKAci , mod_pka_g_nmda_min , mod_pka_g_nmda_max , mod_pka_g_nmda_half , mod_pka_g_nmda_slope ) ;
   modulation_factor_fail = modulation ( _threadargscomma_ PKAci , mod_pka_fail_min , mod_pka_fail_max , mod_pka_fail_half , mod_pka_fail_slope ) ;
   _lmggate = 1.0 / ( 1.0 + exp ( - 0.062 * v ) * ( mg / 3.57 ) ) ;
   g_nmda = ( B_nmda - A_nmda ) * modulation_factor_nmda ;
   _litot_nmda = g_nmda * ( v - e ) * _lmggate ;
   ical_nmda = ca_ratio_nmda * _litot_nmda ;
   i_nmda = _litot_nmda - ical_nmda ;
   g_ampa = ( B_ampa - A_ampa ) * modulation_factor_ampa ;
   _litot_ampa = g_ampa * ( v - e ) ;
   ical_ampa = ca_ratio_ampa * _litot_ampa ;
   i_ampa = _litot_ampa - ical_ampa ;
   ical = ical_nmda + ical_ampa ;
   g = g_ampa + g_nmda ;
   i = i_ampa + i_nmda ;
   }
 _current += i;
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
  PKAci = _ion_PKAci;
 auto const _g_local = _nrn_current(_threadargscomma_ _v + .001);
 	{ double _dical;
  _dical = ical;
 _rhs = _nrn_current(_threadargscomma_ _v);
  _ion_dicaldv += (_dical - ical)/.001 * 1.e2/ (_nd_area);
 	}
 _g = (_g_local - _rhs)/.001;
  _ion_ical += ical * 1.e2/ (_nd_area);
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
  } }}

}

static void terminal(){}

static void _initlists(){
 int _i; static int _first = 1;
  if (!_first) return;
 _slist1[0] = {A_ampa_columnindex, 0};  _dlist1[0] = {DA_ampa_columnindex, 0};
 _slist1[1] = {B_ampa_columnindex, 0};  _dlist1[1] = {DB_ampa_columnindex, 0};
 _slist1[2] = {A_nmda_columnindex, 0};  _dlist1[2] = {DA_nmda_columnindex, 0};
 _slist1[3] = {B_nmda_columnindex, 0};  _dlist1[3] = {DB_nmda_columnindex, 0};
_first = 0;
}

#if NMODL_TEXT
static void register_nmodl_text_and_filename(int mech_type) {
    const char* nmodl_filename = "/cfs/klemming/home/m/metog/BasalGangliaData/data/neurons/mechanisms/tmglut.mod";
    const char* nmodl_file_text = 
  "TITLE Glutamatergic synapse with short-term plasticity (stp)\n"
  "\n"
  "NEURON {\n"
  "    THREADSAFE\n"
  "    POINT_PROCESS tmGlut\n"
  "    RANGE tau1_ampa, tau2_ampa, tau1_nmda, tau2_nmda\n"
  "    RANGE g_ampa, g_nmda, i_ampa, i_nmda, nmda_ratio\n"
  "    RANGE e, g, i, q, mg\n"
  "    RANGE tau, tauR, tauF, U, u0\n"
  "    RANGE ca_ratio_ampa, ca_ratio_nmda, mggate, use_stp\n"
  "    RANGE failRate\n"
  "\n"
  "    USEION PKAc READ PKAci VALENCE 0\n"
  "    RANGE mod_pka_g_ampa_min, mod_pka_g_ampa_max, mod_pka_g_ampa_half, mod_pka_g_ampa_slope\n"
  "    RANGE mod_pka_g_nmda_min, mod_pka_g_nmda_max, mod_pka_g_nmda_half, mod_pka_g_nmda_slope\n"
  "    RANGE modulation_factor_ampa, modulation_factor_nmda, modulation_factor_fail\n"
  "\n"
  "    NONSPECIFIC_CURRENT i\n"
  "    USEION cal WRITE ical VALENCE 2\n"
  "\n"
  "    RANDOM release_probability\n"
  "}\n"
  "\n"
  "UNITS {\n"
  "    (nA) = (nanoamp)\n"
  "    (mV) = (millivolt)\n"
  "    (uS) = (microsiemens)\n"
  "    (mM) = (milli/liter)\n"
  "}\n"
  "\n"
  "PARAMETER {\n"
  "    : q = 2 --- We have manually corrected these\n"
  "    tau1_ampa= 1.1 (ms)     : ORIG 2.2, ampa same as in Wolf? not same as in Du et al 2017 (and glutamate.mod)\n"
  "    tau2_ampa = 5.75 (ms)  : ORIG 11.5 ms,  tau2 > tau1\n"
  "    tau1_nmda= 2.76  (ms)    : ORIG 5.52 ms Chapman et al 2003; table 1, adult rat (rise time, rt = 12.13. rt ~= 2.197*tau (wiki;rise time) -> tau = 12.13 / 2.197 ~= 5.52\n"
  "    tau2_nmda = 115.5 (ms)   : ORIG 231 ms, Chapman et al 2003; table 1, adult rat\n"
  "    nmda_ratio = 0.5 (1)\n"
  "    e = 0 (mV)\n"
  "    tau = 3 (ms)\n"
  "    tauR = 100 (ms)  : tauR > tau\n"
  "    tauF = 800 (ms)  : tauF >= 0\n"
  "    U = 0.3 (1) <0, 1>\n"
  "    u0 = 0 (1) <0, 1>\n"
  "    ca_ratio_ampa = 0.005\n"
  "    ca_ratio_nmda = 0.1\n"
  "    mg = 1 (mM)\n"
  "\n"
  "    mod_pka_g_ampa_min = 1 (1)\n"
  "    mod_pka_g_ampa_max = 1 (1)\n"
  "    mod_pka_g_ampa_half = 0.000100 (mM)\n"
  "    mod_pka_g_ampa_slope = 0.01 (mM)\n"
  "\n"
  "    mod_pka_g_nmda_min = 1 (1)\n"
  "    mod_pka_g_nmda_max = 1 (1)\n"
  "    mod_pka_g_nmda_half = 0.000100 (mM)\n"
  "    mod_pka_g_nmda_slope = 0.01 (mM)\n"
  "\n"
  "    mod_pka_fail_min = 0 (1)\n"
  "    mod_pka_fail_max = 0 (1)\n"
  "    mod_pka_fail_half = 0.000100 (mM)\n"
  "    mod_pka_fail_slope = 0.01 (mM)\n"
  "\n"
  "    failRateScaling = 0\n"
  "    failRate = 0\n"
  "    use_stp = 1     : to turn of use_stp -> use 0\n"
  "}\n"
  "\n"
  "ASSIGNED {\n"
  "    v (mV)\n"
  "    i (nA)\n"
  "    i_ampa (nA)\n"
  "    i_nmda (nA)\n"
  "    ical (nA)\n"
  "    ical_ampa (nA)\n"
  "    ical_nmda (nA)\n"
  "    g (uS)\n"
  "    g_ampa (uS)\n"
  "    g_nmda (uS)\n"
  "    factor_ampa\n"
  "    factor_nmda\n"
  "    x\n"
  "    PKAci (mM)\n"
  "    modulation_factor_ampa (1)\n"
  "    modulation_factor_nmda (1)\n"
  "    modulation_factor_fail (1)\n"
  "}\n"
  "\n"
  "STATE {\n"
  "    A_ampa (uS)\n"
  "    B_ampa (uS)\n"
  "    A_nmda (uS)\n"
  "    B_nmda (uS)\n"
  "}\n"
  "\n"
  "INITIAL {\n"
  "    LOCAL tp_ampa, tp_nmda\n"
  "    A_ampa = 0\n"
  "    B_ampa = 0\n"
  "\n"
  "    tp_ampa = (tau1_ampa*tau2_ampa)/(tau2_ampa-tau1_ampa) * log(tau2_ampa/tau1_ampa)\n"
  "    factor_ampa = -exp(-tp_ampa/tau1_ampa) + exp(-tp_ampa/tau2_ampa)\n"
  "    factor_ampa = 1/factor_ampa\n"
  "\n"
  "    A_nmda = 0\n"
  "    B_nmda = 0\n"
  "    tp_nmda = (tau1_nmda*tau2_nmda)/(tau2_nmda-tau1_nmda) * log(tau2_nmda/tau1_nmda)\n"
  "    factor_nmda = -exp(-tp_nmda/tau1_nmda) + exp(-tp_nmda/tau2_nmda)\n"
  "    factor_nmda = 1/factor_nmda\n"
  "}\n"
  "\n"
  "BREAKPOINT {\n"
  "    LOCAL itot_nmda, itot_ampa, mggate\n"
  "    SOLVE state METHOD cnexp\n"
  "\n"
  "    modulation_factor_ampa=modulation(PKAci, mod_pka_g_ampa_min, mod_pka_g_ampa_max, mod_pka_g_ampa_half, mod_pka_g_ampa_slope)\n"
  "    modulation_factor_nmda=modulation(PKAci, mod_pka_g_nmda_min, mod_pka_g_nmda_max, mod_pka_g_nmda_half, mod_pka_g_nmda_slope)\n"
  "    modulation_factor_fail=modulation(PKAci, mod_pka_fail_min, mod_pka_fail_max, mod_pka_fail_half, mod_pka_fail_slope)\n"
  "    : NMDA\n"
  "    mggate    = 1 / (1 + exp(-0.062 (/mV) * v) * (mg / 3.57 (mM)))\n"
  "    g_nmda    = (B_nmda - A_nmda) * modulation_factor_nmda\n"
  "    itot_nmda = g_nmda * (v - e) * mggate\n"
  "    ical_nmda = ca_ratio_nmda*itot_nmda\n"
  "    i_nmda    = itot_nmda - ical_nmda\n"
  "\n"
  "    : AMPA\n"
  "    g_ampa    = (B_ampa - A_ampa) * modulation_factor_ampa\n"
  "    itot_ampa = g_ampa*(v - e)\n"
  "    ical_ampa = ca_ratio_ampa*itot_ampa\n"
  "    i_ampa    = itot_ampa - ical_ampa\n"
  "\n"
  "    : total values\n"
  "    ical      = ical_nmda + ical_ampa\n"
  "    g         = g_ampa + g_nmda\n"
  "    i         = i_ampa + i_nmda\n"
  "\n"
  "    : printf(\"%g\\t%g\\t%g\\t%g\\t%g\\n\",tau1_ampa,B_ampa,A_ampa,B_nmda,A_nmda)\n"
  "    : printf(\"%g\\t%g\\t%g\\t%g\\t%g\\n\",v,g_nmda,g,i,ical)\n"
  "}\n"
  "\n"
  "DERIVATIVE state {\n"
  "    A_ampa' = -A_ampa/tau1_ampa\n"
  "    B_ampa' = -B_ampa/tau2_ampa\n"
  "    A_nmda' = -A_nmda/tau1_nmda\n"
  "    B_nmda' = -B_nmda/tau2_nmda\n"
  "}\n"
  "\n"
  "NET_RECEIVE(weight (uS), y, z, u, tsyn (ms)) {\n"
  "    LOCAL weight_ampa, weight_nmda\n"
  "    INITIAL {\n"
  "        y = 0\n"
  "        z = 0\n"
  "        u = u0\n"
  "        tsyn = t\n"
  "        : printf(\"t\\t t-tsyn\\t y\\t z\\t u\\n\")\n"
  "\n"
  "    }\n"
  "\n"
  "    if ( weight <= 0 ) {\n"
  "VERBATIM\n"
  "        return;\n"
  "ENDVERBATIM\n"
  "    }\n"
  "    if( urand() > failRate*(1 + modulation_factor_fail)) {\n"
  "\n"
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
  "\n"
  "      if (use_stp > 0) {\n"
  "    	   : We divide by U to normalise, so that g gives amplitude\n"
  "           : of first activation\n"
  "          weight_ampa = weight *x*u / U\n"
  "      } else {\n"
  "          weight_ampa = weight\n"
  "      }\n"
  "\n"
  "      weight_nmda = weight_ampa*nmda_ratio\n"
  "\n"
  "      A_ampa = A_ampa + weight_ampa*factor_ampa\n"
  "      B_ampa = B_ampa + weight_ampa*factor_ampa\n"
  "      A_nmda = A_nmda + weight_nmda*factor_nmda\n"
  "      B_nmda = B_nmda + weight_nmda*factor_nmda\n"
  "\n"
  "      y = y + x*u\n"
  "      : printf(\"** %g\\t%g\\t%g\\t%g\\t%g\\n\", t, t-tsyn, y, z, u)\n"
  "      tsyn = t\n"
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
  "\n"
  "\n"
  "COMMENT\n"
  "(2025-10-08) NEURON 9.0+ compatibility. Replaced scop_random with the\n"
  "new RANDOM keyword.\n"
  "See: https://nrn.readthedocs.io/en/latest/nmodl/language/nmodl_neuron_extension.html#random\n"
  "\n"
  "(2019-11-29) Synaptic failure rate (fail) added. Random factor, no\n"
  "reproducibility guaranteed in parallel sim.\n"
  "\n"
  "(2019-08-21) We normalise the activation by U, to make sure that g specifies\n"
  "             the conductance of the first actvation\n"
  "\n"
  "(2019-06-05) Q-factor was calculated in INITAL block, which meant if\n"
  "the synapse was reinitalised then the time constants changed with each\n"
  "initalise. Updated: Johannes Hjorth, hjorth@kth.se\n"
  "\n"
  "- updates by Robert Lindroos (robert.lindroos at ki.se):\n"
  "Missing line calculating Ca ratio of NMDA current fixed. The whole block were updated since\n"
  "plotting ratios for both nmda and ampa gave 0.\n"
  "- switch for turning of short term dynamics added. If used this synapse will summate.\n"
  "\n"
  "Implementation of glutamatergic synapse model with short-term facilitation\n"
  "and depression based on modified tmgsyn.mod [1] by Tsodyks et al [2].\n"
  "Choice of time constants and calcium current model follows [3].\n"
  "NEURON implementation by Alexander Kozlov <akozlov@kth.se>.\n"
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
  "ENDCOMMENT\n"
  ;
    hoc_reg_nmodl_filename(mech_type, nmodl_filename);
    hoc_reg_nmodl_text(mech_type, nmodl_file_text);
}
#endif
