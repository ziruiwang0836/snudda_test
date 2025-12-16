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
static constexpr auto number_of_floating_point_variables = 68;
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
 
#define nrn_init _nrn_init__na_ch
#define _nrn_initial _nrn_initial__na_ch
#define nrn_cur _nrn_cur__na_ch
#define _nrn_current _nrn_current__na_ch
#define nrn_jacob _nrn_jacob__na_ch
#define nrn_state _nrn_state__na_ch
#define _net_receive _net_receive__na_ch 
#define kin kin__na_ch 
#define rates rates__na_ch 
 
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
#define a0 _ml->template fpfield<1>(_iml)
#define a0_columnindex 1
#define vha _ml->template fpfield<2>(_iml)
#define vha_columnindex 2
#define vca _ml->template fpfield<3>(_iml)
#define vca_columnindex 3
#define b0 _ml->template fpfield<4>(_iml)
#define b0_columnindex 4
#define vhb _ml->template fpfield<5>(_iml)
#define vhb_columnindex 5
#define vcb _ml->template fpfield<6>(_iml)
#define vcb_columnindex 6
#define g0 _ml->template fpfield<7>(_iml)
#define g0_columnindex 7
#define d0 _ml->template fpfield<8>(_iml)
#define d0_columnindex 8
#define aS1 _ml->template fpfield<9>(_iml)
#define aS1_columnindex 9
#define aS2 _ml->template fpfield<10>(_iml)
#define aS2_columnindex 10
#define bS _ml->template fpfield<11>(_iml)
#define bS_columnindex 11
#define Con _ml->template fpfield<12>(_iml)
#define Con_columnindex 12
#define Coff _ml->template fpfield<13>(_iml)
#define Coff_columnindex 13
#define Oon _ml->template fpfield<14>(_iml)
#define Oon_columnindex 14
#define Ooff _ml->template fpfield<15>(_iml)
#define Ooff_columnindex 15
#define mod_pka_g_min _ml->template fpfield<16>(_iml)
#define mod_pka_g_min_columnindex 16
#define mod_pka_g_max _ml->template fpfield<17>(_iml)
#define mod_pka_g_max_columnindex 17
#define mod_pka_g_half _ml->template fpfield<18>(_iml)
#define mod_pka_g_half_columnindex 18
#define mod_pka_g_slope _ml->template fpfield<19>(_iml)
#define mod_pka_g_slope_columnindex 19
#define g _ml->template fpfield<20>(_iml)
#define g_columnindex 20
#define ina _ml->template fpfield<21>(_iml)
#define ina_columnindex 21
#define a _ml->template fpfield<22>(_iml)
#define a_columnindex 22
#define modulation_factor _ml->template fpfield<23>(_iml)
#define modulation_factor_columnindex 23
#define c1 _ml->template fpfield<24>(_iml)
#define c1_columnindex 24
#define c2 _ml->template fpfield<25>(_iml)
#define c2_columnindex 25
#define c3 _ml->template fpfield<26>(_iml)
#define c3_columnindex 26
#define c4 _ml->template fpfield<27>(_iml)
#define c4_columnindex 27
#define c5 _ml->template fpfield<28>(_iml)
#define c5_columnindex 28
#define ct _ml->template fpfield<29>(_iml)
#define ct_columnindex 29
#define o _ml->template fpfield<30>(_iml)
#define o_columnindex 30
#define i1 _ml->template fpfield<31>(_iml)
#define i1_columnindex 31
#define i2 _ml->template fpfield<32>(_iml)
#define i2_columnindex 32
#define i3 _ml->template fpfield<33>(_iml)
#define i3_columnindex 33
#define i4 _ml->template fpfield<34>(_iml)
#define i4_columnindex 34
#define i5 _ml->template fpfield<35>(_iml)
#define i5_columnindex 35
#define i6 _ml->template fpfield<36>(_iml)
#define i6_columnindex 36
#define ift _ml->template fpfield<37>(_iml)
#define ift_columnindex 37
#define is1 _ml->template fpfield<38>(_iml)
#define is1_columnindex 38
#define is2 _ml->template fpfield<39>(_iml)
#define is2_columnindex 39
#define ist _ml->template fpfield<40>(_iml)
#define ist_columnindex 40
#define it _ml->template fpfield<41>(_iml)
#define it_columnindex 41
#define ena _ml->template fpfield<42>(_iml)
#define ena_columnindex 42
#define alpha _ml->template fpfield<43>(_iml)
#define alpha_columnindex 43
#define beta _ml->template fpfield<44>(_iml)
#define beta_columnindex 44
#define gamma _ml->template fpfield<45>(_iml)
#define gamma_columnindex 45
#define delta _ml->template fpfield<46>(_iml)
#define delta_columnindex 46
#define PKAci _ml->template fpfield<47>(_iml)
#define PKAci_columnindex 47
#define Dc1 _ml->template fpfield<48>(_iml)
#define Dc1_columnindex 48
#define Dc2 _ml->template fpfield<49>(_iml)
#define Dc2_columnindex 49
#define Dc3 _ml->template fpfield<50>(_iml)
#define Dc3_columnindex 50
#define Dc4 _ml->template fpfield<51>(_iml)
#define Dc4_columnindex 51
#define Dc5 _ml->template fpfield<52>(_iml)
#define Dc5_columnindex 52
#define Dct _ml->template fpfield<53>(_iml)
#define Dct_columnindex 53
#define Do _ml->template fpfield<54>(_iml)
#define Do_columnindex 54
#define Di1 _ml->template fpfield<55>(_iml)
#define Di1_columnindex 55
#define Di2 _ml->template fpfield<56>(_iml)
#define Di2_columnindex 56
#define Di3 _ml->template fpfield<57>(_iml)
#define Di3_columnindex 57
#define Di4 _ml->template fpfield<58>(_iml)
#define Di4_columnindex 58
#define Di5 _ml->template fpfield<59>(_iml)
#define Di5_columnindex 59
#define Di6 _ml->template fpfield<60>(_iml)
#define Di6_columnindex 60
#define Dift _ml->template fpfield<61>(_iml)
#define Dift_columnindex 61
#define Dis1 _ml->template fpfield<62>(_iml)
#define Dis1_columnindex 62
#define Dis2 _ml->template fpfield<63>(_iml)
#define Dis2_columnindex 63
#define Dist _ml->template fpfield<64>(_iml)
#define Dist_columnindex 64
#define Dit _ml->template fpfield<65>(_iml)
#define Dit_columnindex 65
#define v _ml->template fpfield<66>(_iml)
#define v_columnindex 66
#define _g _ml->template fpfield<67>(_iml)
#define _g_columnindex 67
#define _ion_ena *(_ml->dptr_field<0>(_iml))
#define _p_ion_ena static_cast<neuron::container::data_handle<double>>(_ppvar[0])
#define _ion_ina *(_ml->dptr_field<1>(_iml))
#define _p_ion_ina static_cast<neuron::container::data_handle<double>>(_ppvar[1])
#define _ion_dinadv *(_ml->dptr_field<2>(_iml))
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
 {"setdata_na_ch", _hoc_setdata},
 {"modulation_na_ch", _hoc_modulation},
 {"rates_na_ch", _hoc_rates},
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
#define modulation modulation_na_ch
 extern double modulation( _internalthreadargsprotocomma_ double , double , double , double , double );
 /* declare global and static user variables */
 #define gind 0
 #define _gth 0
 /* some parameters have upper and lower limits */
 static HocParmLimits _hoc_parm_limits[] = {
 {0, 0, 0}
};
 static HocParmUnits _hoc_parm_units[] = {
 {"gbar_na_ch", "S/cm2"},
 {"a0_na_ch", "1/ms"},
 {"vha_na_ch", "mV"},
 {"vca_na_ch", "mV"},
 {"b0_na_ch", "1/ms"},
 {"vhb_na_ch", "mV"},
 {"vcb_na_ch", "mV"},
 {"g0_na_ch", "1/ms"},
 {"d0_na_ch", "1/ms"},
 {"aS1_na_ch", "1/ms"},
 {"aS2_na_ch", "1/ms"},
 {"bS_na_ch", "1/ms"},
 {"Con_na_ch", "1/ms"},
 {"Coff_na_ch", "1/ms"},
 {"Oon_na_ch", "1/ms"},
 {"Ooff_na_ch", "1/ms"},
 {"mod_pka_g_min_na_ch", "1"},
 {"mod_pka_g_max_na_ch", "1"},
 {"mod_pka_g_half_na_ch", "mM"},
 {"mod_pka_g_slope_na_ch", "mM"},
 {"g_na_ch", "S/cm2"},
 {"ina_na_ch", "mA/cm2"},
 {"modulation_factor_na_ch", "1"},
 {0, 0}
};
 static double ct0 = 0;
 static double c50 = 0;
 static double c40 = 0;
 static double c30 = 0;
 static double c20 = 0;
 static double c10 = 0;
 static double delta_t = 0.01;
 static double it0 = 0;
 static double ist0 = 0;
 static double is20 = 0;
 static double is10 = 0;
 static double ift0 = 0;
 static double i60 = 0;
 static double i50 = 0;
 static double i40 = 0;
 static double i30 = 0;
 static double i20 = 0;
 static double i10 = 0;
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
 
#define _cvode_ieq _ppvar[5].literal_value<int>()
 static void _ode_matsol_instance1(_internalthreadargsproto_);
 /* connect range variables in _p that hoc is supposed to know about */
 static const char *_mechanism[] = {
 "7.7.0",
"na_ch",
 "gbar_na_ch",
 "a0_na_ch",
 "vha_na_ch",
 "vca_na_ch",
 "b0_na_ch",
 "vhb_na_ch",
 "vcb_na_ch",
 "g0_na_ch",
 "d0_na_ch",
 "aS1_na_ch",
 "aS2_na_ch",
 "bS_na_ch",
 "Con_na_ch",
 "Coff_na_ch",
 "Oon_na_ch",
 "Ooff_na_ch",
 "mod_pka_g_min_na_ch",
 "mod_pka_g_max_na_ch",
 "mod_pka_g_half_na_ch",
 "mod_pka_g_slope_na_ch",
 0,
 "g_na_ch",
 "ina_na_ch",
 "a_na_ch",
 "modulation_factor_na_ch",
 0,
 "c1_na_ch",
 "c2_na_ch",
 "c3_na_ch",
 "c4_na_ch",
 "c5_na_ch",
 "ct_na_ch",
 "o_na_ch",
 "i1_na_ch",
 "i2_na_ch",
 "i3_na_ch",
 "i4_na_ch",
 "i5_na_ch",
 "i6_na_ch",
 "ift_na_ch",
 "is1_na_ch",
 "is2_na_ch",
 "ist_na_ch",
 "it_na_ch",
 0,
 0};
 static Symbol* _na_sym;
 static Symbol* _PKAc_sym;
 
 /* Used by NrnProperty */
 static _nrn_mechanism_std_vector<double> _parm_default{
     1, /* gbar */
     37, /* a0 */
     45, /* vha */
     40, /* vca */
     10, /* b0 */
     -50, /* vhb */
     -20, /* vcb */
     40, /* g0 */
     30, /* d0 */
     0.0025, /* aS1 */
     0.0002, /* aS2 */
     0.00017, /* bS */
     0.001, /* Con */
     0.1, /* Coff */
     0.7, /* Oon */
     0.01, /* Ooff */
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
    assert(_nrn_mechanism_get_num_vars(_prop) == 68);
 	/*initialize range parameters*/
 	gbar = _parm_default[0]; /* 1 */
 	a0 = _parm_default[1]; /* 37 */
 	vha = _parm_default[2]; /* 45 */
 	vca = _parm_default[3]; /* 40 */
 	b0 = _parm_default[4]; /* 10 */
 	vhb = _parm_default[5]; /* -50 */
 	vcb = _parm_default[6]; /* -20 */
 	g0 = _parm_default[7]; /* 40 */
 	d0 = _parm_default[8]; /* 30 */
 	aS1 = _parm_default[9]; /* 0.0025 */
 	aS2 = _parm_default[10]; /* 0.0002 */
 	bS = _parm_default[11]; /* 0.00017 */
 	Con = _parm_default[12]; /* 0.001 */
 	Coff = _parm_default[13]; /* 0.1 */
 	Oon = _parm_default[14]; /* 0.7 */
 	Ooff = _parm_default[15]; /* 0.01 */
 	mod_pka_g_min = _parm_default[16]; /* 1 */
 	mod_pka_g_max = _parm_default[17]; /* 1 */
 	mod_pka_g_half = _parm_default[18]; /* 0.0001 */
 	mod_pka_g_slope = _parm_default[19]; /* 0.01 */
 	 assert(_nrn_mechanism_get_num_vars(_prop) == 68);
 	_nrn_mechanism_access_dparam(_prop) = _ppvar;
 	/*connect ionic variables to this model*/
 prop_ion = need_memb(_na_sym);
 nrn_promote(prop_ion, 0, 1);
 	_ppvar[0] = _nrn_mechanism_get_param_handle(prop_ion, 0); /* ena */
 	_ppvar[1] = _nrn_mechanism_get_param_handle(prop_ion, 3); /* ina */
 	_ppvar[2] = _nrn_mechanism_get_param_handle(prop_ion, 4); /* _ion_dinadv */
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
 static void _thread_cleanup(Datum*);
 extern Symbol* hoc_lookup(const char*);
extern void _nrn_thread_reg(int, int, void(*)(Datum*));
void _nrn_thread_table_reg(int, nrn_thread_table_check_t);
extern void hoc_register_tolerance(int, HocStateTolerance*, Symbol***);
extern void _cvode_abstol( Symbol**, double*, int);

 extern "C" void _na_ch_reg() {
	int _vectorized = 1;
  _initlists();
 	ion_reg("na", -10000.);
 	ion_reg("PKAc", 0.0);
 	_na_sym = hoc_lookup("na_ion");
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
                                       _nrn_mechanism_field<double>{"a0"} /* 1 */,
                                       _nrn_mechanism_field<double>{"vha"} /* 2 */,
                                       _nrn_mechanism_field<double>{"vca"} /* 3 */,
                                       _nrn_mechanism_field<double>{"b0"} /* 4 */,
                                       _nrn_mechanism_field<double>{"vhb"} /* 5 */,
                                       _nrn_mechanism_field<double>{"vcb"} /* 6 */,
                                       _nrn_mechanism_field<double>{"g0"} /* 7 */,
                                       _nrn_mechanism_field<double>{"d0"} /* 8 */,
                                       _nrn_mechanism_field<double>{"aS1"} /* 9 */,
                                       _nrn_mechanism_field<double>{"aS2"} /* 10 */,
                                       _nrn_mechanism_field<double>{"bS"} /* 11 */,
                                       _nrn_mechanism_field<double>{"Con"} /* 12 */,
                                       _nrn_mechanism_field<double>{"Coff"} /* 13 */,
                                       _nrn_mechanism_field<double>{"Oon"} /* 14 */,
                                       _nrn_mechanism_field<double>{"Ooff"} /* 15 */,
                                       _nrn_mechanism_field<double>{"mod_pka_g_min"} /* 16 */,
                                       _nrn_mechanism_field<double>{"mod_pka_g_max"} /* 17 */,
                                       _nrn_mechanism_field<double>{"mod_pka_g_half"} /* 18 */,
                                       _nrn_mechanism_field<double>{"mod_pka_g_slope"} /* 19 */,
                                       _nrn_mechanism_field<double>{"g"} /* 20 */,
                                       _nrn_mechanism_field<double>{"ina"} /* 21 */,
                                       _nrn_mechanism_field<double>{"a"} /* 22 */,
                                       _nrn_mechanism_field<double>{"modulation_factor"} /* 23 */,
                                       _nrn_mechanism_field<double>{"c1"} /* 24 */,
                                       _nrn_mechanism_field<double>{"c2"} /* 25 */,
                                       _nrn_mechanism_field<double>{"c3"} /* 26 */,
                                       _nrn_mechanism_field<double>{"c4"} /* 27 */,
                                       _nrn_mechanism_field<double>{"c5"} /* 28 */,
                                       _nrn_mechanism_field<double>{"ct"} /* 29 */,
                                       _nrn_mechanism_field<double>{"o"} /* 30 */,
                                       _nrn_mechanism_field<double>{"i1"} /* 31 */,
                                       _nrn_mechanism_field<double>{"i2"} /* 32 */,
                                       _nrn_mechanism_field<double>{"i3"} /* 33 */,
                                       _nrn_mechanism_field<double>{"i4"} /* 34 */,
                                       _nrn_mechanism_field<double>{"i5"} /* 35 */,
                                       _nrn_mechanism_field<double>{"i6"} /* 36 */,
                                       _nrn_mechanism_field<double>{"ift"} /* 37 */,
                                       _nrn_mechanism_field<double>{"is1"} /* 38 */,
                                       _nrn_mechanism_field<double>{"is2"} /* 39 */,
                                       _nrn_mechanism_field<double>{"ist"} /* 40 */,
                                       _nrn_mechanism_field<double>{"it"} /* 41 */,
                                       _nrn_mechanism_field<double>{"ena"} /* 42 */,
                                       _nrn_mechanism_field<double>{"alpha"} /* 43 */,
                                       _nrn_mechanism_field<double>{"beta"} /* 44 */,
                                       _nrn_mechanism_field<double>{"gamma"} /* 45 */,
                                       _nrn_mechanism_field<double>{"delta"} /* 46 */,
                                       _nrn_mechanism_field<double>{"PKAci"} /* 47 */,
                                       _nrn_mechanism_field<double>{"Dc1"} /* 48 */,
                                       _nrn_mechanism_field<double>{"Dc2"} /* 49 */,
                                       _nrn_mechanism_field<double>{"Dc3"} /* 50 */,
                                       _nrn_mechanism_field<double>{"Dc4"} /* 51 */,
                                       _nrn_mechanism_field<double>{"Dc5"} /* 52 */,
                                       _nrn_mechanism_field<double>{"Dct"} /* 53 */,
                                       _nrn_mechanism_field<double>{"Do"} /* 54 */,
                                       _nrn_mechanism_field<double>{"Di1"} /* 55 */,
                                       _nrn_mechanism_field<double>{"Di2"} /* 56 */,
                                       _nrn_mechanism_field<double>{"Di3"} /* 57 */,
                                       _nrn_mechanism_field<double>{"Di4"} /* 58 */,
                                       _nrn_mechanism_field<double>{"Di5"} /* 59 */,
                                       _nrn_mechanism_field<double>{"Di6"} /* 60 */,
                                       _nrn_mechanism_field<double>{"Dift"} /* 61 */,
                                       _nrn_mechanism_field<double>{"Dis1"} /* 62 */,
                                       _nrn_mechanism_field<double>{"Dis2"} /* 63 */,
                                       _nrn_mechanism_field<double>{"Dist"} /* 64 */,
                                       _nrn_mechanism_field<double>{"Dit"} /* 65 */,
                                       _nrn_mechanism_field<double>{"v"} /* 66 */,
                                       _nrn_mechanism_field<double>{"_g"} /* 67 */,
                                       _nrn_mechanism_field<double*>{"_ion_ena", "na_ion"} /* 0 */,
                                       _nrn_mechanism_field<double*>{"_ion_ina", "na_ion"} /* 1 */,
                                       _nrn_mechanism_field<double*>{"_ion_dinadv", "na_ion"} /* 2 */,
                                       _nrn_mechanism_field<double*>{"_ion_PKAci", "PKAc_ion"} /* 3 */,
                                       _nrn_mechanism_field<double*>{"_ion_PKAco", "PKAc_ion"} /* 4 */,
                                       _nrn_mechanism_field<int>{"_cvode_ieq", "cvodeieq"} /* 5 */);
  hoc_register_prop_size(_mechtype, 68, 6);
  hoc_register_dparam_semantics(_mechtype, 0, "na_ion");
  hoc_register_dparam_semantics(_mechtype, 1, "na_ion");
  hoc_register_dparam_semantics(_mechtype, 2, "na_ion");
  hoc_register_dparam_semantics(_mechtype, 3, "PKAc_ion");
  hoc_register_dparam_semantics(_mechtype, 4, "PKAc_ion");
  hoc_register_dparam_semantics(_mechtype, 5, "cvodeieq");
 	hoc_register_cvode(_mechtype, _ode_count, _ode_map, _ode_spec, _ode_matsol);
 	hoc_register_tolerance(_mechtype, _hoc_state_tol, &_atollist);
 
    hoc_register_var(hoc_scdoub, hoc_vdoub, hoc_intfunc);
 	ivoc_help("help ?1 na_ch /cfs/klemming/home/m/metog/BasalGangliaData/data/neurons/mechanisms/na_ch.mod\n");
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
 static neuron::container::field_index _slist1[14], _dlist1[14]; static double *_temp1;
 static int kin (void* _so, double* _rhs, _internalthreadargsproto_);
 
static int kin (void* _so, double* _rhs, _internalthreadargsproto_)
 {int _reset=0;
 {
   double b_flux, f_flux, _term; int _i;
 {int _i; double _dt1 = 1.0/dt;
for(_i=1;_i<14;_i++){
  	_RHS1(_i) = -_dt1*(_ml->data(_iml, _slist1[_i]) - _ml->data(_iml, _dlist1[_i]));
	_MATELM1(_i, _i) = _dt1;
      
} }
 rates ( _threadargscomma_ v ) ;
   /* ~ c1 <-> c2 ( 4.0 * alpha , beta )*/
 f_flux =  4.0 * alpha * c1 ;
 b_flux =  beta * c2 ;
 _RHS1( 5) -= (f_flux - b_flux);
 _RHS1( 4) += (f_flux - b_flux);
 
 _term =  4.0 * alpha ;
 _MATELM1( 5 ,5)  += _term;
 _MATELM1( 4 ,5)  -= _term;
 _term =  beta ;
 _MATELM1( 5 ,4)  -= _term;
 _MATELM1( 4 ,4)  += _term;
 /*REACTION*/
  /* ~ c2 <-> c3 ( 3.0 * alpha , 2.0 * beta )*/
 f_flux =  3.0 * alpha * c2 ;
 b_flux =  2.0 * beta * c3 ;
 _RHS1( 4) -= (f_flux - b_flux);
 _RHS1( 3) += (f_flux - b_flux);
 
 _term =  3.0 * alpha ;
 _MATELM1( 4 ,4)  += _term;
 _MATELM1( 3 ,4)  -= _term;
 _term =  2.0 * beta ;
 _MATELM1( 4 ,3)  -= _term;
 _MATELM1( 3 ,3)  += _term;
 /*REACTION*/
  /* ~ c3 <-> c4 ( 2.0 * alpha , 3.0 * beta )*/
 f_flux =  2.0 * alpha * c3 ;
 b_flux =  3.0 * beta * c4 ;
 _RHS1( 3) -= (f_flux - b_flux);
 _RHS1( 2) += (f_flux - b_flux);
 
 _term =  2.0 * alpha ;
 _MATELM1( 3 ,3)  += _term;
 _MATELM1( 2 ,3)  -= _term;
 _term =  3.0 * beta ;
 _MATELM1( 3 ,2)  -= _term;
 _MATELM1( 2 ,2)  += _term;
 /*REACTION*/
  /* ~ c4 <-> c5 ( alpha , 4.0 * beta )*/
 f_flux =  alpha * c4 ;
 b_flux =  4.0 * beta * c5 ;
 _RHS1( 2) -= (f_flux - b_flux);
 _RHS1( 1) += (f_flux - b_flux);
 
 _term =  alpha ;
 _MATELM1( 2 ,2)  += _term;
 _MATELM1( 1 ,2)  -= _term;
 _term =  4.0 * beta ;
 _MATELM1( 2 ,1)  -= _term;
 _MATELM1( 1 ,1)  += _term;
 /*REACTION*/
  /* ~ c5 <-> o ( gamma , delta )*/
 f_flux =  gamma * c5 ;
 b_flux =  delta * o ;
 _RHS1( 1) -= (f_flux - b_flux);
 
 _term =  gamma ;
 _MATELM1( 1 ,1)  += _term;
 _term =  delta ;
 _MATELM1( 1 ,0)  -= _term;
 /*REACTION*/
  /* ~ o <-> is1 ( aS1 , bS )*/
 f_flux =  aS1 * o ;
 b_flux =  bS * is1 ;
 _RHS1( 7) += (f_flux - b_flux);
 
 _term =  aS1 ;
 _MATELM1( 7 ,0)  -= _term;
 _term =  bS ;
 _MATELM1( 7 ,7)  += _term;
 /*REACTION*/
  /* ~ i1 <-> i2 ( 4.0 * alpha * a , beta / a )*/
 f_flux =  4.0 * alpha * a * i1 ;
 b_flux =  beta / a * i2 ;
 _RHS1( 13) -= (f_flux - b_flux);
 _RHS1( 12) += (f_flux - b_flux);
 
 _term =  4.0 * alpha * a ;
 _MATELM1( 13 ,13)  += _term;
 _MATELM1( 12 ,13)  -= _term;
 _term =  beta / a ;
 _MATELM1( 13 ,12)  -= _term;
 _MATELM1( 12 ,12)  += _term;
 /*REACTION*/
  /* ~ i2 <-> i3 ( 3.0 * alpha * a , 2.0 * beta / a )*/
 f_flux =  3.0 * alpha * a * i2 ;
 b_flux =  2.0 * beta / a * i3 ;
 _RHS1( 12) -= (f_flux - b_flux);
 _RHS1( 11) += (f_flux - b_flux);
 
 _term =  3.0 * alpha * a ;
 _MATELM1( 12 ,12)  += _term;
 _MATELM1( 11 ,12)  -= _term;
 _term =  2.0 * beta / a ;
 _MATELM1( 12 ,11)  -= _term;
 _MATELM1( 11 ,11)  += _term;
 /*REACTION*/
  /* ~ i3 <-> i4 ( 2.0 * alpha * a , 3.0 * beta / a )*/
 f_flux =  2.0 * alpha * a * i3 ;
 b_flux =  3.0 * beta / a * i4 ;
 _RHS1( 11) -= (f_flux - b_flux);
 _RHS1( 10) += (f_flux - b_flux);
 
 _term =  2.0 * alpha * a ;
 _MATELM1( 11 ,11)  += _term;
 _MATELM1( 10 ,11)  -= _term;
 _term =  3.0 * beta / a ;
 _MATELM1( 11 ,10)  -= _term;
 _MATELM1( 10 ,10)  += _term;
 /*REACTION*/
  /* ~ i4 <-> i5 ( alpha * a , 4.0 * beta / a )*/
 f_flux =  alpha * a * i4 ;
 b_flux =  4.0 * beta / a * i5 ;
 _RHS1( 10) -= (f_flux - b_flux);
 _RHS1( 9) += (f_flux - b_flux);
 
 _term =  alpha * a ;
 _MATELM1( 10 ,10)  += _term;
 _MATELM1( 9 ,10)  -= _term;
 _term =  4.0 * beta / a ;
 _MATELM1( 10 ,9)  -= _term;
 _MATELM1( 9 ,9)  += _term;
 /*REACTION*/
  /* ~ i5 <-> i6 ( gamma , delta )*/
 f_flux =  gamma * i5 ;
 b_flux =  delta * i6 ;
 _RHS1( 9) -= (f_flux - b_flux);
 _RHS1( 8) += (f_flux - b_flux);
 
 _term =  gamma ;
 _MATELM1( 9 ,9)  += _term;
 _MATELM1( 8 ,9)  -= _term;
 _term =  delta ;
 _MATELM1( 9 ,8)  -= _term;
 _MATELM1( 8 ,8)  += _term;
 /*REACTION*/
  /* ~ i6 <-> is2 ( aS2 , bS )*/
 f_flux =  aS2 * i6 ;
 b_flux =  bS * is2 ;
 _RHS1( 8) -= (f_flux - b_flux);
 _RHS1( 6) += (f_flux - b_flux);
 
 _term =  aS2 ;
 _MATELM1( 8 ,8)  += _term;
 _MATELM1( 6 ,8)  -= _term;
 _term =  bS ;
 _MATELM1( 8 ,6)  -= _term;
 _MATELM1( 6 ,6)  += _term;
 /*REACTION*/
  /* ~ c1 <-> i1 ( Con , Coff )*/
 f_flux =  Con * c1 ;
 b_flux =  Coff * i1 ;
 _RHS1( 5) -= (f_flux - b_flux);
 _RHS1( 13) += (f_flux - b_flux);
 
 _term =  Con ;
 _MATELM1( 5 ,5)  += _term;
 _MATELM1( 13 ,5)  -= _term;
 _term =  Coff ;
 _MATELM1( 5 ,13)  -= _term;
 _MATELM1( 13 ,13)  += _term;
 /*REACTION*/
  /* ~ c2 <-> i2 ( Con * a , Coff / a )*/
 f_flux =  Con * a * c2 ;
 b_flux =  Coff / a * i2 ;
 _RHS1( 4) -= (f_flux - b_flux);
 _RHS1( 12) += (f_flux - b_flux);
 
 _term =  Con * a ;
 _MATELM1( 4 ,4)  += _term;
 _MATELM1( 12 ,4)  -= _term;
 _term =  Coff / a ;
 _MATELM1( 4 ,12)  -= _term;
 _MATELM1( 12 ,12)  += _term;
 /*REACTION*/
  /* ~ c3 <-> i3 ( Con * pow( a , 2.0 ) , Coff / pow( a , 2.0 ) )*/
 f_flux =  Con * pow( a , 2.0 ) * c3 ;
 b_flux =  Coff / pow( a , 2.0 ) * i3 ;
 _RHS1( 3) -= (f_flux - b_flux);
 _RHS1( 11) += (f_flux - b_flux);
 
 _term =  Con * pow( a , 2.0 ) ;
 _MATELM1( 3 ,3)  += _term;
 _MATELM1( 11 ,3)  -= _term;
 _term =  Coff / pow( a , 2.0 ) ;
 _MATELM1( 3 ,11)  -= _term;
 _MATELM1( 11 ,11)  += _term;
 /*REACTION*/
  /* ~ c4 <-> i4 ( Con * pow( a , 3.0 ) , Coff / pow( a , 3.0 ) )*/
 f_flux =  Con * pow( a , 3.0 ) * c4 ;
 b_flux =  Coff / pow( a , 3.0 ) * i4 ;
 _RHS1( 2) -= (f_flux - b_flux);
 _RHS1( 10) += (f_flux - b_flux);
 
 _term =  Con * pow( a , 3.0 ) ;
 _MATELM1( 2 ,2)  += _term;
 _MATELM1( 10 ,2)  -= _term;
 _term =  Coff / pow( a , 3.0 ) ;
 _MATELM1( 2 ,10)  -= _term;
 _MATELM1( 10 ,10)  += _term;
 /*REACTION*/
  /* ~ c5 <-> i5 ( Con * pow( a , 4.0 ) , Coff / pow( a , 4.0 ) )*/
 f_flux =  Con * pow( a , 4.0 ) * c5 ;
 b_flux =  Coff / pow( a , 4.0 ) * i5 ;
 _RHS1( 1) -= (f_flux - b_flux);
 _RHS1( 9) += (f_flux - b_flux);
 
 _term =  Con * pow( a , 4.0 ) ;
 _MATELM1( 1 ,1)  += _term;
 _MATELM1( 9 ,1)  -= _term;
 _term =  Coff / pow( a , 4.0 ) ;
 _MATELM1( 1 ,9)  -= _term;
 _MATELM1( 9 ,9)  += _term;
 /*REACTION*/
  /* ~ o <-> i6 ( Oon , Ooff )*/
 f_flux =  Oon * o ;
 b_flux =  Ooff * i6 ;
 _RHS1( 8) += (f_flux - b_flux);
 
 _term =  Oon ;
 _MATELM1( 8 ,0)  -= _term;
 _term =  Ooff ;
 _MATELM1( 8 ,8)  += _term;
 /*REACTION*/
   /* c1 + c2 + c3 + c4 + c5 + i1 + i2 + i3 + i4 + i5 + i6 + is1 + is2 + o = 1.0 */
 _RHS1(0) =  1.0;
 _MATELM1(0, 0) = 1;
 _RHS1(0) -= o ;
 _MATELM1(0, 6) = 1;
 _RHS1(0) -= is2 ;
 _MATELM1(0, 7) = 1;
 _RHS1(0) -= is1 ;
 _MATELM1(0, 8) = 1;
 _RHS1(0) -= i6 ;
 _MATELM1(0, 9) = 1;
 _RHS1(0) -= i5 ;
 _MATELM1(0, 10) = 1;
 _RHS1(0) -= i4 ;
 _MATELM1(0, 11) = 1;
 _RHS1(0) -= i3 ;
 _MATELM1(0, 12) = 1;
 _RHS1(0) -= i2 ;
 _MATELM1(0, 13) = 1;
 _RHS1(0) -= i1 ;
 _MATELM1(0, 1) = 1;
 _RHS1(0) -= c5 ;
 _MATELM1(0, 2) = 1;
 _RHS1(0) -= c4 ;
 _MATELM1(0, 3) = 1;
 _RHS1(0) -= c3 ;
 _MATELM1(0, 4) = 1;
 _RHS1(0) -= c2 ;
 _MATELM1(0, 5) = 1;
 _RHS1(0) -= c1 ;
 /*CONSERVATION*/
   } return _reset;
 }
 
static int  rates ( _internalthreadargsprotocomma_ double _lv ) {
   alpha = a0 * exp ( ( _lv - vha ) / vca ) ;
   beta = b0 * exp ( ( _lv - vhb ) / vcb ) ;
   gamma = g0 ;
   delta = d0 ;
   a = pow( ( ( Coff / Con ) / ( Ooff / Oon ) ) , ( 1.0 / 8.0 ) ) ;
    return 0; }
 
static void _hoc_rates(void) {
  double _r;
 Datum* _ppvar; Datum* _thread; NrnThread* _nt;
 
  if(!_prop_id) {
    hoc_execerror("No data for rates_na_ch. Requires prior call to setdata_na_ch and that the specified mechanism instance still be in existence.", NULL);
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
 double b_flux, f_flux, _term; int _i;
 {int _i; for(_i=0;_i<14;_i++) _ml->data(_iml, _dlist1[_i]) = 0.0;}
 rates ( _threadargscomma_ v ) ;
 /* ~ c1 <-> c2 ( 4.0 * alpha , beta )*/
 f_flux =  4.0 * alpha * c1 ;
 b_flux =  beta * c2 ;
 Dc1 -= (f_flux - b_flux);
 Dc2 += (f_flux - b_flux);
 
 /*REACTION*/
  /* ~ c2 <-> c3 ( 3.0 * alpha , 2.0 * beta )*/
 f_flux =  3.0 * alpha * c2 ;
 b_flux =  2.0 * beta * c3 ;
 Dc2 -= (f_flux - b_flux);
 Dc3 += (f_flux - b_flux);
 
 /*REACTION*/
  /* ~ c3 <-> c4 ( 2.0 * alpha , 3.0 * beta )*/
 f_flux =  2.0 * alpha * c3 ;
 b_flux =  3.0 * beta * c4 ;
 Dc3 -= (f_flux - b_flux);
 Dc4 += (f_flux - b_flux);
 
 /*REACTION*/
  /* ~ c4 <-> c5 ( alpha , 4.0 * beta )*/
 f_flux =  alpha * c4 ;
 b_flux =  4.0 * beta * c5 ;
 Dc4 -= (f_flux - b_flux);
 Dc5 += (f_flux - b_flux);
 
 /*REACTION*/
  /* ~ c5 <-> o ( gamma , delta )*/
 f_flux =  gamma * c5 ;
 b_flux =  delta * o ;
 Dc5 -= (f_flux - b_flux);
 Do += (f_flux - b_flux);
 
 /*REACTION*/
  /* ~ o <-> is1 ( aS1 , bS )*/
 f_flux =  aS1 * o ;
 b_flux =  bS * is1 ;
 Do -= (f_flux - b_flux);
 Dis1 += (f_flux - b_flux);
 
 /*REACTION*/
  /* ~ i1 <-> i2 ( 4.0 * alpha * a , beta / a )*/
 f_flux =  4.0 * alpha * a * i1 ;
 b_flux =  beta / a * i2 ;
 Di1 -= (f_flux - b_flux);
 Di2 += (f_flux - b_flux);
 
 /*REACTION*/
  /* ~ i2 <-> i3 ( 3.0 * alpha * a , 2.0 * beta / a )*/
 f_flux =  3.0 * alpha * a * i2 ;
 b_flux =  2.0 * beta / a * i3 ;
 Di2 -= (f_flux - b_flux);
 Di3 += (f_flux - b_flux);
 
 /*REACTION*/
  /* ~ i3 <-> i4 ( 2.0 * alpha * a , 3.0 * beta / a )*/
 f_flux =  2.0 * alpha * a * i3 ;
 b_flux =  3.0 * beta / a * i4 ;
 Di3 -= (f_flux - b_flux);
 Di4 += (f_flux - b_flux);
 
 /*REACTION*/
  /* ~ i4 <-> i5 ( alpha * a , 4.0 * beta / a )*/
 f_flux =  alpha * a * i4 ;
 b_flux =  4.0 * beta / a * i5 ;
 Di4 -= (f_flux - b_flux);
 Di5 += (f_flux - b_flux);
 
 /*REACTION*/
  /* ~ i5 <-> i6 ( gamma , delta )*/
 f_flux =  gamma * i5 ;
 b_flux =  delta * i6 ;
 Di5 -= (f_flux - b_flux);
 Di6 += (f_flux - b_flux);
 
 /*REACTION*/
  /* ~ i6 <-> is2 ( aS2 , bS )*/
 f_flux =  aS2 * i6 ;
 b_flux =  bS * is2 ;
 Di6 -= (f_flux - b_flux);
 Dis2 += (f_flux - b_flux);
 
 /*REACTION*/
  /* ~ c1 <-> i1 ( Con , Coff )*/
 f_flux =  Con * c1 ;
 b_flux =  Coff * i1 ;
 Dc1 -= (f_flux - b_flux);
 Di1 += (f_flux - b_flux);
 
 /*REACTION*/
  /* ~ c2 <-> i2 ( Con * a , Coff / a )*/
 f_flux =  Con * a * c2 ;
 b_flux =  Coff / a * i2 ;
 Dc2 -= (f_flux - b_flux);
 Di2 += (f_flux - b_flux);
 
 /*REACTION*/
  /* ~ c3 <-> i3 ( Con * pow( a , 2.0 ) , Coff / pow( a , 2.0 ) )*/
 f_flux =  Con * pow( a , 2.0 ) * c3 ;
 b_flux =  Coff / pow( a , 2.0 ) * i3 ;
 Dc3 -= (f_flux - b_flux);
 Di3 += (f_flux - b_flux);
 
 /*REACTION*/
  /* ~ c4 <-> i4 ( Con * pow( a , 3.0 ) , Coff / pow( a , 3.0 ) )*/
 f_flux =  Con * pow( a , 3.0 ) * c4 ;
 b_flux =  Coff / pow( a , 3.0 ) * i4 ;
 Dc4 -= (f_flux - b_flux);
 Di4 += (f_flux - b_flux);
 
 /*REACTION*/
  /* ~ c5 <-> i5 ( Con * pow( a , 4.0 ) , Coff / pow( a , 4.0 ) )*/
 f_flux =  Con * pow( a , 4.0 ) * c5 ;
 b_flux =  Coff / pow( a , 4.0 ) * i5 ;
 Dc5 -= (f_flux - b_flux);
 Di5 += (f_flux - b_flux);
 
 /*REACTION*/
  /* ~ o <-> i6 ( Oon , Ooff )*/
 f_flux =  Oon * o ;
 b_flux =  Ooff * i6 ;
 Do -= (f_flux - b_flux);
 Di6 += (f_flux - b_flux);
 
 /*REACTION*/
   /* c1 + c2 + c3 + c4 + c5 + i1 + i2 + i3 + i4 + i5 + i6 + is1 + is2 + o = 1.0 */
 /*CONSERVATION*/
   } return _reset;
 }
 
/*CVODE matsol*/
 static int _ode_matsol1(void* _so, double* _rhs, _internalthreadargsproto_) {int _reset=0;{
 double b_flux, f_flux, _term; int _i;
   b_flux = f_flux = 0.;
 {int _i; double _dt1 = 1.0/dt;
for(_i=0;_i<14;_i++){
  	_RHS1(_i) = _dt1*(_ml->data(_iml, _dlist1[_i]));
	_MATELM1(_i, _i) = _dt1;
      
} }
 rates ( _threadargscomma_ v ) ;
 /* ~ c1 <-> c2 ( 4.0 * alpha , beta )*/
 _term =  4.0 * alpha ;
 _MATELM1( 5 ,5)  += _term;
 _MATELM1( 4 ,5)  -= _term;
 _term =  beta ;
 _MATELM1( 5 ,4)  -= _term;
 _MATELM1( 4 ,4)  += _term;
 /*REACTION*/
  /* ~ c2 <-> c3 ( 3.0 * alpha , 2.0 * beta )*/
 _term =  3.0 * alpha ;
 _MATELM1( 4 ,4)  += _term;
 _MATELM1( 3 ,4)  -= _term;
 _term =  2.0 * beta ;
 _MATELM1( 4 ,3)  -= _term;
 _MATELM1( 3 ,3)  += _term;
 /*REACTION*/
  /* ~ c3 <-> c4 ( 2.0 * alpha , 3.0 * beta )*/
 _term =  2.0 * alpha ;
 _MATELM1( 3 ,3)  += _term;
 _MATELM1( 2 ,3)  -= _term;
 _term =  3.0 * beta ;
 _MATELM1( 3 ,2)  -= _term;
 _MATELM1( 2 ,2)  += _term;
 /*REACTION*/
  /* ~ c4 <-> c5 ( alpha , 4.0 * beta )*/
 _term =  alpha ;
 _MATELM1( 2 ,2)  += _term;
 _MATELM1( 1 ,2)  -= _term;
 _term =  4.0 * beta ;
 _MATELM1( 2 ,1)  -= _term;
 _MATELM1( 1 ,1)  += _term;
 /*REACTION*/
  /* ~ c5 <-> o ( gamma , delta )*/
 _term =  gamma ;
 _MATELM1( 1 ,1)  += _term;
 _MATELM1( 0 ,1)  -= _term;
 _term =  delta ;
 _MATELM1( 1 ,0)  -= _term;
 _MATELM1( 0 ,0)  += _term;
 /*REACTION*/
  /* ~ o <-> is1 ( aS1 , bS )*/
 _term =  aS1 ;
 _MATELM1( 0 ,0)  += _term;
 _MATELM1( 7 ,0)  -= _term;
 _term =  bS ;
 _MATELM1( 0 ,7)  -= _term;
 _MATELM1( 7 ,7)  += _term;
 /*REACTION*/
  /* ~ i1 <-> i2 ( 4.0 * alpha * a , beta / a )*/
 _term =  4.0 * alpha * a ;
 _MATELM1( 13 ,13)  += _term;
 _MATELM1( 12 ,13)  -= _term;
 _term =  beta / a ;
 _MATELM1( 13 ,12)  -= _term;
 _MATELM1( 12 ,12)  += _term;
 /*REACTION*/
  /* ~ i2 <-> i3 ( 3.0 * alpha * a , 2.0 * beta / a )*/
 _term =  3.0 * alpha * a ;
 _MATELM1( 12 ,12)  += _term;
 _MATELM1( 11 ,12)  -= _term;
 _term =  2.0 * beta / a ;
 _MATELM1( 12 ,11)  -= _term;
 _MATELM1( 11 ,11)  += _term;
 /*REACTION*/
  /* ~ i3 <-> i4 ( 2.0 * alpha * a , 3.0 * beta / a )*/
 _term =  2.0 * alpha * a ;
 _MATELM1( 11 ,11)  += _term;
 _MATELM1( 10 ,11)  -= _term;
 _term =  3.0 * beta / a ;
 _MATELM1( 11 ,10)  -= _term;
 _MATELM1( 10 ,10)  += _term;
 /*REACTION*/
  /* ~ i4 <-> i5 ( alpha * a , 4.0 * beta / a )*/
 _term =  alpha * a ;
 _MATELM1( 10 ,10)  += _term;
 _MATELM1( 9 ,10)  -= _term;
 _term =  4.0 * beta / a ;
 _MATELM1( 10 ,9)  -= _term;
 _MATELM1( 9 ,9)  += _term;
 /*REACTION*/
  /* ~ i5 <-> i6 ( gamma , delta )*/
 _term =  gamma ;
 _MATELM1( 9 ,9)  += _term;
 _MATELM1( 8 ,9)  -= _term;
 _term =  delta ;
 _MATELM1( 9 ,8)  -= _term;
 _MATELM1( 8 ,8)  += _term;
 /*REACTION*/
  /* ~ i6 <-> is2 ( aS2 , bS )*/
 _term =  aS2 ;
 _MATELM1( 8 ,8)  += _term;
 _MATELM1( 6 ,8)  -= _term;
 _term =  bS ;
 _MATELM1( 8 ,6)  -= _term;
 _MATELM1( 6 ,6)  += _term;
 /*REACTION*/
  /* ~ c1 <-> i1 ( Con , Coff )*/
 _term =  Con ;
 _MATELM1( 5 ,5)  += _term;
 _MATELM1( 13 ,5)  -= _term;
 _term =  Coff ;
 _MATELM1( 5 ,13)  -= _term;
 _MATELM1( 13 ,13)  += _term;
 /*REACTION*/
  /* ~ c2 <-> i2 ( Con * a , Coff / a )*/
 _term =  Con * a ;
 _MATELM1( 4 ,4)  += _term;
 _MATELM1( 12 ,4)  -= _term;
 _term =  Coff / a ;
 _MATELM1( 4 ,12)  -= _term;
 _MATELM1( 12 ,12)  += _term;
 /*REACTION*/
  /* ~ c3 <-> i3 ( Con * pow( a , 2.0 ) , Coff / pow( a , 2.0 ) )*/
 _term =  Con * pow( a , 2.0 ) ;
 _MATELM1( 3 ,3)  += _term;
 _MATELM1( 11 ,3)  -= _term;
 _term =  Coff / pow( a , 2.0 ) ;
 _MATELM1( 3 ,11)  -= _term;
 _MATELM1( 11 ,11)  += _term;
 /*REACTION*/
  /* ~ c4 <-> i4 ( Con * pow( a , 3.0 ) , Coff / pow( a , 3.0 ) )*/
 _term =  Con * pow( a , 3.0 ) ;
 _MATELM1( 2 ,2)  += _term;
 _MATELM1( 10 ,2)  -= _term;
 _term =  Coff / pow( a , 3.0 ) ;
 _MATELM1( 2 ,10)  -= _term;
 _MATELM1( 10 ,10)  += _term;
 /*REACTION*/
  /* ~ c5 <-> i5 ( Con * pow( a , 4.0 ) , Coff / pow( a , 4.0 ) )*/
 _term =  Con * pow( a , 4.0 ) ;
 _MATELM1( 1 ,1)  += _term;
 _MATELM1( 9 ,1)  -= _term;
 _term =  Coff / pow( a , 4.0 ) ;
 _MATELM1( 1 ,9)  -= _term;
 _MATELM1( 9 ,9)  += _term;
 /*REACTION*/
  /* ~ o <-> i6 ( Oon , Ooff )*/
 _term =  Oon ;
 _MATELM1( 0 ,0)  += _term;
 _MATELM1( 8 ,0)  -= _term;
 _term =  Ooff ;
 _MATELM1( 0 ,8)  -= _term;
 _MATELM1( 8 ,8)  += _term;
 /*REACTION*/
   /* c1 + c2 + c3 + c4 + c5 + i1 + i2 + i3 + i4 + i5 + i6 + is1 + is2 + o = 1.0 */
 /*CONSERVATION*/
   } return _reset;
 }
 
/*CVODE end*/
 
static int _ode_count(int _type){ return 14;}
 
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
  PKAci = _ion_PKAci;
     _ode_spec1 (_threadargs_);
  }}
 
static void _ode_map(Prop* _prop, int _ieq, neuron::container::data_handle<double>* _pv, neuron::container::data_handle<double>* _pvdot, double* _atol, int _type) { 
  Datum* _ppvar;
  _ppvar = _nrn_mechanism_access_dparam(_prop);
  _cvode_ieq = _ieq;
  for (int _i=0; _i < 14; ++_i) {
    _pv[_i] = _nrn_mechanism_get_param_handle(_prop, _slist1[_i]);
    _pvdot[_i] = _nrn_mechanism_get_param_handle(_prop, _dlist1[_i]);
    _cvode_abstol(_atollist, _atol, _i);
  }
 }
 
static void _ode_matsol_instance1(_internalthreadargsproto_) {
 _cvode_sparse_thread(&(_thread[_cvspth1].literal_value<void*>()), 14, _dlist1, neuron::scopmath::row_view{_ml, _iml}, _ode_matsol1, _threadargs_);
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
  PKAci = _ion_PKAci;
 _ode_matsol_instance1(_threadargs_);
 }}
 
static void _thread_cleanup(Datum* _thread) {
   _nrn_destroy_sparseobj_thread(static_cast<SparseObj*>(_thread[_cvspth1].get<void*>()));
   _nrn_destroy_sparseobj_thread(static_cast<SparseObj*>(_thread[_spth1].get<void*>()));
 }

static void initmodel(_internalthreadargsproto_) {
  int _i; double _save;{
  ct = ct0;
  c5 = c50;
  c4 = c40;
  c3 = c30;
  c2 = c20;
  c1 = c10;
  it = it0;
  ist = ist0;
  is2 = is20;
  is1 = is10;
  ift = ift0;
  i6 = i60;
  i5 = i50;
  i4 = i40;
  i3 = i30;
  i2 = i20;
  i1 = i10;
  o = o0;
 {
    _ss_sparse_thread(&(_thread[_spth1].literal_value<void*>()), 14, _slist1, _dlist1, neuron::scopmath::row_view{_ml, _iml}, &t, dt, kin, _linmat1, _threadargs_);
     if (secondorder) {
    int _i;
    for (_i = 0; _i < 14; ++_i) {
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
  ena = _ion_ena;
  PKAci = _ion_PKAci;
 initmodel(_threadargs_);
 }
}

static double _nrn_current(_internalthreadargsprotocomma_ double _v) {
double _current=0.; v=_v;
{ {
   modulation_factor = modulation ( _threadargscomma_ PKAci , mod_pka_g_min , mod_pka_g_max , mod_pka_g_half , mod_pka_g_slope ) ;
   g = gbar * o * modulation_factor ;
   ina = g * ( v - ena ) ;
   ct = c1 + c2 + c3 + c4 + c5 ;
   ift = i1 + i2 + i3 + i4 + i5 + i6 ;
   ist = is1 + is2 ;
   it = ift + ist ;
   }
 _current += ina;

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
  PKAci = _ion_PKAci;
 auto const _g_local = _nrn_current(_threadargscomma_ _v + .001);
 	{ double _dina;
  _dina = ina;
 _rhs = _nrn_current(_threadargscomma_ _v);
  _ion_dinadv += (_dina - ina)/.001 ;
 	}
 _g = (_g_local - _rhs)/.001;
  _ion_ina += ina ;
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
  ena = _ion_ena;
  PKAci = _ion_PKAci;
 {  sparse_thread(&(_thread[_spth1].literal_value<void*>()), 14, _slist1, _dlist1, neuron::scopmath::row_view{_ml, _iml}, &t, dt, kin, _linmat1, _threadargs_);
     if (secondorder) {
    int _i;
    for (_i = 0; _i < 14; ++_i) {
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
 _slist1[1] = {c5_columnindex, 0};  _dlist1[1] = {Dc5_columnindex, 0};
 _slist1[2] = {c4_columnindex, 0};  _dlist1[2] = {Dc4_columnindex, 0};
 _slist1[3] = {c3_columnindex, 0};  _dlist1[3] = {Dc3_columnindex, 0};
 _slist1[4] = {c2_columnindex, 0};  _dlist1[4] = {Dc2_columnindex, 0};
 _slist1[5] = {c1_columnindex, 0};  _dlist1[5] = {Dc1_columnindex, 0};
 _slist1[6] = {is2_columnindex, 0};  _dlist1[6] = {Dis2_columnindex, 0};
 _slist1[7] = {is1_columnindex, 0};  _dlist1[7] = {Dis1_columnindex, 0};
 _slist1[8] = {i6_columnindex, 0};  _dlist1[8] = {Di6_columnindex, 0};
 _slist1[9] = {i5_columnindex, 0};  _dlist1[9] = {Di5_columnindex, 0};
 _slist1[10] = {i4_columnindex, 0};  _dlist1[10] = {Di4_columnindex, 0};
 _slist1[11] = {i3_columnindex, 0};  _dlist1[11] = {Di3_columnindex, 0};
 _slist1[12] = {i2_columnindex, 0};  _dlist1[12] = {Di2_columnindex, 0};
 _slist1[13] = {i1_columnindex, 0};  _dlist1[13] = {Di1_columnindex, 0};
_first = 0;
}

#if NMODL_TEXT
static void register_nmodl_text_and_filename(int mech_type) {
    const char* nmodl_filename = "/cfs/klemming/home/m/metog/BasalGangliaData/data/neurons/mechanisms/na_ch.mod";
    const char* nmodl_file_text = 
  "COMMENT\n"
  "NA_CH.MOD\n"
  "\n"
  "c1 - c2 - c3 - c4 - c5 - o  - is1\n"
  "|    |    |    |    |    |\n"
  "i1 - i2 - i3 - i4 - i5 - i6 - is2\n"
  "\n"
  "SLOW\n"
  "\n"
  "6/18/2003\n"
  "    \n"
  "ENDCOMMENT\n"
  "\n"
  "\n"
  "\n"
  "NEURON {\n"
  "	SUFFIX na_ch\n"
  "	USEION na READ ena WRITE ina\n"
  "	RANGE g, ina, gbar, a\n"
  "	RANGE Con, Coff, Oon, Ooff\n"
  "	RANGE a0, vha, vca\n"
  "	RANGE b0, vhb, vcb\n"
  "	RANGE g0\n"
  "	RANGE d0\n"
  "	RANGE aS1, aS2, bS\n"
  "    USEION PKAc READ PKAci VALENCE 0\n"
  "    RANGE mod_pka_g_min, mod_pka_g_max, mod_pka_g_half, mod_pka_g_slope \n"
  "    RANGE modulation_factor\n"
  "\n"
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
  "\n"
  "	a0 = 37		(1/ms)	: alpha\n"
  "	vha  = 45	(mV)\n"
  "	vca = 40	(mV)\n"
  "\n"
  "	b0 = 10		(1/ms)	: beta\n"
  "	vhb = -50	(mV)\n"
  "	vcb = -20	(mV)\n"
  "\n"
  "	g0 = 40		(1/ms)	: gamma\n"
  "\n"
  "	d0 = 30		(1/ms)	: delta\n"
  "\n"
  "	aS1 = 0.0025	(1/ms)\n"
  "	aS2 = 0.0002	(1/ms)\n"
  "	bS = 0.00017	(1/ms)\n"
  "\n"
  "	Con = 0.001	(1/ms)\n"
  "	Coff = 0.1	(1/ms)\n"
  "	Oon = .7	(1/ms)\n"
  "	Ooff = 0.01	(1/ms)\n"
  "\n"
  "    mod_pka_g_min = 1 (1)\n"
  "    mod_pka_g_max = 1 (1)\n"
  "    mod_pka_g_half = 0.000100 (mM)\n"
  "    mod_pka_g_slope = 0.01 (mM)			\n"
  "}\n"
  "\n"
  "ASSIGNED {\n"
  "	v	(mV)\n"
  "	ena	(mV)\n"
  "	g	(S/cm2)\n"
  "	ina	(mA/cm2)\n"
  "	alpha	(1/ms)\n"
  "	beta	(1/ms)\n"
  "	gamma	(1/ms)\n"
  "	delta	(1/ms)\n"
  "	a\n"
  "    PKAci (mM)\n"
  "    modulation_factor (1)\n"
  "}\n"
  "\n"
  "STATE {\n"
  "	c1  : closed\n"
  "	c2\n"
  "	c3\n"
  "	c4\n"
  "	c5\n"
  "	ct  : total closed\n"
  "	o   : open\n"
  "	i1  : fast inactivated\n"
  "	i2\n"
  "	i3\n"
  "	i4\n"
  "	i5\n"
  "	i6\n"
  "	ift : total fast inactivated\n"
  "	is1 : slow inactivated\n"
  "	is2\n"
  "	ist : total slow inactivated\n"
  "	it  : total inactivated\n"
  "}\n"
  "\n"
  "BREAKPOINT {\n"
  "     SOLVE kin METHOD sparse\n"
  "    modulation_factor=modulation(PKAci, mod_pka_g_min, mod_pka_g_max, mod_pka_g_half, mod_pka_g_slope)	   \n"
  "	   \n"
  "	g = gbar*o*modulation_factor\n"
  "	ina = g*(v-ena)\n"
  "	ct = c1 + c2 + c3 + c4 + c5\n"
  "	ift = i1 + i2 + i3 + i4 + i5 + i6\n"
  "	ist = is1 + is2\n"
  "	it = ift + ist\n"
  "}\n"
  "\n"
  "INITIAL {\n"
  "	SOLVE kin STEADYSTATE sparse\n"
  "}\n"
  "\n"
  "KINETIC kin{\n"
  "	rates(v)\n"
  "\n"
  "	~ c1 <-> c2 (4*alpha, beta)\n"
  "	~ c2 <-> c3 (3*alpha, 2*beta)\n"
  "	~ c3 <-> c4 (2*alpha, 3*beta)\n"
  "	~ c4 <-> c5 (alpha, 4*beta)\n"
  "	~ c5 <-> o  (gamma, delta)\n"
  "	~ o <-> is1 (aS1, bS)\n"
  "\n"
  "	~ i1 <-> i2 (4*alpha*a, beta/a)\n"
  "	~ i2 <-> i3 (3*alpha*a, 2*beta/a)\n"
  "	~ i3 <-> i4 (2*alpha*a, 3*beta/a)\n"
  "	~ i4 <-> i5 (alpha*a, 4*beta/a)\n"
  "	~ i5 <-> i6 (gamma, delta)\n"
  "	~ i6 <-> is2 (aS2, bS)\n"
  "\n"
  "	~ c1 <-> i1 (Con, Coff)\n"
  "	~ c2 <-> i2 (Con*a, Coff/a)\n"
  "	~ c3 <-> i3 (Con*a^2, Coff/a^2)\n"
  "	~ c4 <-> i4 (Con*a^3, Coff/a^3)\n"
  "	~ c5 <-> i5 (Con*a^4, Coff/a^4)\n"
  "	~ o <-> i6  (Oon, Ooff)\n"
  "\n"
  "	CONSERVE c1+c2+c3+c4+c5+i1+i2+i3+i4+i5+i6+is1+is2+o=1\n"
  "\n"
  "}\n"
  "\n"
  "PROCEDURE rates(v(millivolt)) {\n"
  "	alpha = a0*exp((v-vha)/vca)\n"
  "	beta = b0*exp((v-vhb)/vcb)\n"
  "	gamma = g0\n"
  "	delta = d0\n"
  "\n"
  "	a = ((Coff/Con)/(Ooff/Oon))^(1/8)\n"
  "}\n"
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
