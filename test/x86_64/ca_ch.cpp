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
static constexpr auto number_of_datum_variables = 8;
static constexpr auto number_of_floating_point_variables = 17;
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
 
#define nrn_init _nrn_init__ca_ch
#define _nrn_initial _nrn_initial__ca_ch
#define nrn_cur _nrn_cur__ca_ch
#define _nrn_current _nrn_current__ca_ch
#define nrn_jacob _nrn_jacob__ca_ch
#define nrn_state _nrn_state__ca_ch
#define _net_receive _net_receive__ca_ch 
#define coord coord__ca_ch 
#define state state__ca_ch 
 
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
#define test _ml->template fpfield<0>(_iml)
#define test_columnindex 0
#define ipump _ml->template fpfield<1>(_iml)
#define ipump_columnindex 1
#define last_ipump _ml->template fpfield<2>(_iml)
#define last_ipump_columnindex 2
#define ca _ml->template data_array<3, 4>(_iml)
#define ca_columnindex 3
#define CaBuffer _ml->template data_array<4, 4>(_iml)
#define CaBuffer_columnindex 4
#define Buffer _ml->template data_array<5, 4>(_iml)
#define Buffer_columnindex 5
#define pump _ml->template fpfield<6>(_iml)
#define pump_columnindex 6
#define pumpca _ml->template fpfield<7>(_iml)
#define pumpca_columnindex 7
#define cao _ml->template fpfield<8>(_iml)
#define cao_columnindex 8
#define ica _ml->template fpfield<9>(_iml)
#define ica_columnindex 9
#define cai _ml->template fpfield<10>(_iml)
#define cai_columnindex 10
#define Dca _ml->template data_array<11, 4>(_iml)
#define Dca_columnindex 11
#define DCaBuffer _ml->template data_array<12, 4>(_iml)
#define DCaBuffer_columnindex 12
#define DBuffer _ml->template data_array<13, 4>(_iml)
#define DBuffer_columnindex 13
#define Dpump _ml->template fpfield<14>(_iml)
#define Dpump_columnindex 14
#define Dpumpca _ml->template fpfield<15>(_iml)
#define Dpumpca_columnindex 15
#define _g _ml->template fpfield<16>(_iml)
#define _g_columnindex 16
#define _ion_cao *(_ml->dptr_field<0>(_iml))
#define _p_ion_cao static_cast<neuron::container::data_handle<double>>(_ppvar[0])
#define _ion_cai *(_ml->dptr_field<1>(_iml))
#define _p_ion_cai static_cast<neuron::container::data_handle<double>>(_ppvar[1])
#define _ion_ica *(_ml->dptr_field<2>(_iml))
#define _p_ion_ica static_cast<neuron::container::data_handle<double>>(_ppvar[2])
#define _ion_dicadv *(_ml->dptr_field<3>(_iml))
#define _ion_ca_erev *_ml->dptr_field<4>(_iml)
#define _style_ca	*_ppvar[5].get<int*>()
#define diam	(*(_ml->dptr_field<6>(_iml)))
#define area	(*(_ml->dptr_field<7>(_iml)))
 static _nrn_mechanism_cache_instance _ml_real{nullptr};
static _nrn_mechanism_cache_range *_ml{&_ml_real};
static size_t _iml{0};
static Datum *_ppvar;
 static int hoc_nrnpointerindex =  -1;
 static Prop* _extcall_prop;
 /* _prop_id kind of shadows _extcall_prop to allow validity checking. */
 static _nrn_non_owning_id_without_container _prop_id{};
 /* external NEURON variables */
 /* declaration of user functions */
 static void _hoc_coord(void);
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
 {"setdata_ca_ch", _hoc_setdata},
 {"coord_ca_ch", _hoc_coord},
 {0, 0}
};
 
/* Direct Python call wrappers to density mechanism functions.*/
 static double _npy_coord(Prop*);
 
static NPyDirectMechFunc npy_direct_func_proc[] = {
 {"coord", _npy_coord},
 {0, 0}
};
 /* declare global and static user variables */
 #define gind 0
 #define _gth 0
#define Buffer0 Buffer0_ca_ch
 double Buffer0 = 0;
#define DFree DFree_ca_ch
 double DFree = 0.6;
#define k4 k4_ca_ch
 double k4 = 5e+06;
#define k3 k3_ca_ch
 double k3 = 1e+10;
#define k2 k2_ca_ch
 double k2 = 5e+08;
#define k1 k1_ca_ch
 double k1 = 1e+10;
#define k2buf k2buf_ca_ch
 double k2buf = 0.5;
#define k1buf k1buf_ca_ch
 double k1buf = 500;
#define totbuf totbuf_ca_ch
 double totbuf = 1.2;
#define totpump totpump_ca_ch
 double totpump = 0.2;
#define vol vol_ca_ch
 double vol[4];
 /* some parameters have upper and lower limits */
 static HocParmLimits _hoc_parm_limits[] = {
 {0, 0, 0}
};
 static HocParmUnits _hoc_parm_units[] = {
 {"DFree_ca_ch", "um2/ms"},
 {"k1buf_ca_ch", "/mM-ms"},
 {"k2buf_ca_ch", "/ms"},
 {"k1_ca_ch", "um3/s"},
 {"k2_ca_ch", "/s"},
 {"k3_ca_ch", "/s"},
 {"k4_ca_ch", "um3/s"},
 {"totpump_ca_ch", "mol/cm2"},
 {"totbuf_ca_ch", "mM"},
 {"vol_ca_ch", "1"},
 {"Buffer0_ca_ch", "mM"},
 {"ca_ca_ch", "mM"},
 {"CaBuffer_ca_ch", "mM"},
 {"Buffer_ca_ch", "mM"},
 {"pump_ca_ch", "mol/cm2"},
 {"pumpca_ca_ch", "mol/cm2"},
 {"ipump_ca_ch", "mA/cm2"},
 {"last_ipump_ca_ch", "mA/cm2"},
 {0, 0}
};
 static double CaBuffer0 = 0;
 static double ca0 = 0;
 static double delta_t = 0.01;
 static double pumpca0 = 0;
 static double pump0 = 0;
 static double v = 0;
 /* connect global user variables to hoc */
 static DoubScal hoc_scdoub[] = {
 {"DFree_ca_ch", &DFree_ca_ch},
 {"k1buf_ca_ch", &k1buf_ca_ch},
 {"k2buf_ca_ch", &k2buf_ca_ch},
 {"k1_ca_ch", &k1_ca_ch},
 {"k2_ca_ch", &k2_ca_ch},
 {"k3_ca_ch", &k3_ca_ch},
 {"k4_ca_ch", &k4_ca_ch},
 {"totpump_ca_ch", &totpump_ca_ch},
 {"totbuf_ca_ch", &totbuf_ca_ch},
 {"Buffer0_ca_ch", &Buffer0_ca_ch},
 {0, 0}
};
 static DoubVec hoc_vdoub[] = {
 {"vol_ca_ch", vol_ca_ch, 4},
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
 
#define _cvode_ieq _ppvar[8].literal_value<int>()
 static void _ode_synonym(_nrn_model_sorted_token const&, NrnThread&, Memb_list&, int);
 static void _ode_matsol_instance1(_internalthreadargsproto_);
 /* connect range variables in _p that hoc is supposed to know about */
 static const char *_mechanism[] = {
 "7.7.0",
"ca_ch",
 0,
 "test_ca_ch",
 "ipump_ca_ch",
 "last_ipump_ca_ch",
 0,
 "ca_ca_ch[4]",
 "CaBuffer_ca_ch[4]",
 "Buffer_ca_ch[4]",
 "pump_ca_ch",
 "pumpca_ca_ch",
 0,
 0};
 static Symbol* _morphology_sym;
 extern Node* nrn_alloc_node_;
 static Symbol* _ca_sym;
 
 /* Used by NrnProperty */
 static _nrn_mechanism_std_vector<double> _parm_default{
 }; 
 
 
extern Prop* need_memb(Symbol*);
static void nrn_alloc(Prop* _prop) {
  Prop *prop_ion{};
  Datum *_ppvar{};
   _ppvar = nrn_prop_datum_alloc(_mechtype, 9, _prop);
    _nrn_mechanism_access_dparam(_prop) = _ppvar;
     _nrn_mechanism_cache_instance _ml_real{_prop};
    auto* const _ml = &_ml_real;
    size_t const _iml{};
    assert(_nrn_mechanism_get_num_vars(_prop) == 17);
 	/*initialize range parameters*/
 	 assert(_nrn_mechanism_get_num_vars(_prop) == 17);
 	_nrn_mechanism_access_dparam(_prop) = _ppvar;
 	/*connect ionic variables to this model*/
 prop_ion = need_memb(_morphology_sym);
 	_ppvar[6] = _nrn_mechanism_get_param_handle(prop_ion, 0); /* diam */
 	_ppvar[7] = _nrn_mechanism_get_area_handle(nrn_alloc_node_);
 prop_ion = need_memb(_ca_sym);
 nrn_check_conc_write(_prop, prop_ion, 1);
 nrn_promote(prop_ion, 3, 0);
 	_ppvar[0] = _nrn_mechanism_get_param_handle(prop_ion, 2); /* cao */
 	_ppvar[1] = _nrn_mechanism_get_param_handle(prop_ion, 1); /* cai */
 	_ppvar[2] = _nrn_mechanism_get_param_handle(prop_ion, 3); /* ica */
 	_ppvar[3] = _nrn_mechanism_get_param_handle(prop_ion, 4); /* _ion_dicadv */
 	_ppvar[4] = _nrn_mechanism_get_param_handle(prop_ion, 0); // erev ca
 	_ppvar[5] = {neuron::container::do_not_search, &(_nrn_mechanism_access_dparam(prop_ion)[0].literal_value<int>())}; /* iontype for ca */
 
}
 static void _initlists();
  /* some states have an absolute tolerance */
 static Symbol** _atollist;
 static HocStateTolerance _hoc_state_tol[] = {
 {"ca_ca_ch", 1e-05},
 {"pump_ca_ch", 0.001},
 {"pumpca_ca_ch", 1e-15},
 {0, 0}
};
 extern Symbol* hoc_lookup(const char*);
extern void _nrn_thread_reg(int, int, void(*)(Datum*));
void _nrn_thread_table_reg(int, nrn_thread_table_check_t);
extern void hoc_register_tolerance(int, HocStateTolerance*, Symbol***);
extern void _cvode_abstol( Symbol**, double*, int);

 extern "C" void _ca_ch_reg() {
	int _vectorized = 0;
  _initlists();
 	ion_reg("ca", -10000.);
 	_morphology_sym = hoc_lookup("morphology");
 	_ca_sym = hoc_lookup("ca_ion");
 	register_mech(_mechanism, nrn_alloc,nrn_cur, nrn_jacob, nrn_state, nrn_init, hoc_nrnpointerindex, 0);
 _mechtype = nrn_get_mechtype(_mechanism[1]);
 hoc_register_parm_default(_mechtype, &_parm_default);
         hoc_register_npy_direct(_mechtype, npy_direct_func_proc);
     _nrn_setdata_reg(_mechtype, _setdata);
 #if NMODL_TEXT
  register_nmodl_text_and_filename(_mechtype);
#endif
   _nrn_mechanism_register_data_fields(_mechtype,
                                       _nrn_mechanism_field<double>{"test"} /* 0 */,
                                       _nrn_mechanism_field<double>{"ipump"} /* 1 */,
                                       _nrn_mechanism_field<double>{"last_ipump"} /* 2 */,
                                       _nrn_mechanism_field<double>{"ca", 4} /* 3 */,
                                       _nrn_mechanism_field<double>{"CaBuffer", 4} /* 4 */,
                                       _nrn_mechanism_field<double>{"Buffer", 4} /* 5 */,
                                       _nrn_mechanism_field<double>{"pump"} /* 6 */,
                                       _nrn_mechanism_field<double>{"pumpca"} /* 7 */,
                                       _nrn_mechanism_field<double>{"cao"} /* 8 */,
                                       _nrn_mechanism_field<double>{"ica"} /* 9 */,
                                       _nrn_mechanism_field<double>{"cai"} /* 10 */,
                                       _nrn_mechanism_field<double>{"Dca", 4} /* 11 */,
                                       _nrn_mechanism_field<double>{"DCaBuffer", 4} /* 12 */,
                                       _nrn_mechanism_field<double>{"DBuffer", 4} /* 13 */,
                                       _nrn_mechanism_field<double>{"Dpump"} /* 14 */,
                                       _nrn_mechanism_field<double>{"Dpumpca"} /* 15 */,
                                       _nrn_mechanism_field<double>{"_g"} /* 16 */,
                                       _nrn_mechanism_field<double*>{"_ion_cao", "ca_ion"} /* 0 */,
                                       _nrn_mechanism_field<double*>{"_ion_cai", "ca_ion"} /* 1 */,
                                       _nrn_mechanism_field<double*>{"_ion_ica", "ca_ion"} /* 2 */,
                                       _nrn_mechanism_field<double*>{"_ion_dicadv", "ca_ion"} /* 3 */,
                                       _nrn_mechanism_field<double*>{"_ion_ca_erev", "ca_ion"} /* 4 */,
                                       _nrn_mechanism_field<int*>{"_style_ca", "#ca_ion"} /* 5 */,
                                       _nrn_mechanism_field<double*>{"diam", "diam"} /* 6 */,
                                       _nrn_mechanism_field<double*>{"area", "area"} /* 7 */,
                                       _nrn_mechanism_field<int>{"_cvode_ieq", "cvodeieq"} /* 8 */);
  hoc_register_prop_size(_mechtype, 35, 9);
  hoc_register_dparam_semantics(_mechtype, 0, "ca_ion");
  hoc_register_dparam_semantics(_mechtype, 1, "ca_ion");
  hoc_register_dparam_semantics(_mechtype, 2, "ca_ion");
  hoc_register_dparam_semantics(_mechtype, 3, "ca_ion");
  hoc_register_dparam_semantics(_mechtype, 4, "ca_ion");
  hoc_register_dparam_semantics(_mechtype, 5, "#ca_ion");
  hoc_register_dparam_semantics(_mechtype, 8, "cvodeieq");
  hoc_register_dparam_semantics(_mechtype, 6, "diam");
  hoc_register_dparam_semantics(_mechtype, 7, "area");
 	nrn_writes_conc(_mechtype, 0);
 	hoc_register_cvode(_mechtype, _ode_count, _ode_map, _ode_spec, _ode_matsol);
 	hoc_register_tolerance(_mechtype, _hoc_state_tol, &_atollist);
 	hoc_register_synonym(_mechtype, _ode_synonym);
 
    hoc_register_var(hoc_scdoub, hoc_vdoub, hoc_intfunc);
 	ivoc_help("help ?1 ca_ch /cfs/klemming/home/m/metog/BasalGangliaData/data/neurons/mechanisms/ca_ch.mod\n");
 hoc_register_limits(_mechtype, _hoc_parm_limits);
 hoc_register_units(_mechtype, _hoc_parm_units);
 }
 static double FARADAY = 0x1.34c0c8b92a9b7p+3;
 static double PI = 0x1.921fb54442d18p+1;
 static double volo = 1;
 static double _zcoord_done ;
 static double _zfrat [ 4 ] ;
 static double _zdsq , _zdsqvol ;
static int _reset;
static const char *modelname = "Calcium ion accumulation and diffusion";

static int error;
static int _ninits = 0;
static int _match_recurse=1;
static void _modl_cleanup(){ _match_recurse=1;}
static int coord();
 
#define _MATELM1(_row,_col)	*(_getelm(_row + 1, _col + 1))
 
#define _RHS1(_arg) _coef1[_arg + 1]
 static double *_coef1;
 
#define _linmat1  0
 static void* _sparseobj1;
 static void* _cvsparseobj1;
 
static int _ode_spec1(_internalthreadargsproto_);
/*static int _ode_matsol1(_internalthreadargsproto_);*/
 static neuron::container::field_index _slist1[14], _dlist1[14]; static double *_temp1;
 static int state ();
 
static int  coord (  ) {
   double _lr , _ldr2 ;
 _lr = 1.0 / 2.0 ;
   _ldr2 = _lr / ( 4.0 - 1.0 ) / 2.0 ;
   vol [ 0 ] = 0.0 ;
   _zfrat [ 0 ] = 2.0 * _lr ;
   {int  _li ;for ( _li = 0 ; _li <= 4 - 2 ; _li ++ ) {
     vol [ _li ] = vol [ _li ] + PI * ( _lr - _ldr2 / 2.0 ) * 2.0 * _ldr2 ;
     _lr = _lr - _ldr2 ;
     _zfrat [ _li + 1 ] = 2.0 * PI * _lr / ( 2.0 * _ldr2 ) ;
     _lr = _lr - _ldr2 ;
     vol [ _li + 1 ] = PI * ( _lr + _ldr2 / 2.0 ) * 2.0 * _ldr2 ;
     } }
    return 0; }
 
static void _hoc_coord(void) {
  double _r;
    _r = 1.;
 coord (  );
 hoc_retpushx(_r);
}
 
static double _npy_coord(Prop* _prop) {
    double _r{0.0};
    neuron::legacy::set_globals_from_prop(_prop, _ml_real, _ml, _iml);
  _ppvar = _nrn_mechanism_access_dparam(_prop);
 _r = 1.;
 coord (  );
 return(_r);
}
 
static int state ()
 {_reset=0;
 {
   double b_flux, f_flux, _term; int _i;
 {int _i; double _dt1 = 1.0/dt;
for(_i=0;_i<14;_i++){
  	_RHS1(_i) = -_dt1*(_ml->data(_iml, _slist1[_i]) - _ml->data(_iml, _dlist1[_i]));
	_MATELM1(_i, _i) = _dt1;
      
}  
_RHS1(12) *= ( ( 1.e10 ) * area) ;
_MATELM1(12, 12) *= ( ( 1.e10 ) * area); 
_RHS1(13) *= ( ( 1.e10 ) * area) ;
_MATELM1(13, 13) *= ( ( 1.e10 ) * area);  
for (_i=0; _i < 4; _i++) {
  	_RHS1(_i + 0) *= ( diam * diam * vol [ ((int) _i ) ] * 1.0) ;
_MATELM1(_i + 0, _i + 0) *= ( diam * diam * vol [ ((int) _i ) ] * 1.0);  } 
for (_i=0; _i < 4; _i++) {
  	_RHS1(_i + 4) *= ( diam * diam * vol [ ((int) _i ) ] * 1.0) ;
_MATELM1(_i + 4, _i + 4) *= ( diam * diam * vol [ ((int) _i ) ] * 1.0);  } 
for (_i=0; _i < 4; _i++) {
  	_RHS1(_i + 8) *= ( diam * diam * vol [ ((int) _i ) ] * 1.0) ;
_MATELM1(_i + 8, _i + 8) *= ( diam * diam * vol [ ((int) _i ) ] * 1.0);  } }
 /* COMPARTMENT _li , diam * diam * vol [ ((int) _i ) ] * 1.0 {
     ca CaBuffer Buffer }
   */
 /* COMPARTMENT ( 1.e10 ) * area {
     pump pumpca }
   */
 /* COMPARTMENT ( 1.e15 ) * volo {
     }
   */
 /* ~ ca [ 0 ] < < ( - ( ica - last_ipump ) * PI * diam * _zfrat [ 0 ] * 1.0 / ( 2.0 * FARADAY ) )*/
 f_flux = b_flux = 0.;
 _RHS1( 8 +  0) += (b_flux =   ( - ( ica - last_ipump ) * PI * diam * _zfrat [ 0 ] * 1.0 / ( 2.0 * FARADAY ) ) );
 /*FLUX*/
  {int  _li ;for ( _li = 0 ; _li <= 4 - 2 ; _li ++ ) {
     /* ~ ca [ _li ] <-> ca [ _li + 1 ] ( DFree * _zfrat [ _li + 1 ] * 1.0 , DFree * _zfrat [ _li + 1 ] * 1.0 )*/
 f_flux =  DFree * _zfrat [ _li + 1 ] * 1.0 * ca [ _li] ;
 b_flux =  DFree * _zfrat [ _li + 1 ] * 1.0 * ca [ _li + 1] ;
 _RHS1( 8 +  _li) -= (f_flux - b_flux);
 _RHS1( 8 +  _li + 1) += (f_flux - b_flux);
 
 _term =  DFree * _zfrat [ _li + 1 ] * 1.0 ;
 _MATELM1( 8 +  _li ,8 +  _li)  += _term;
 _MATELM1( 8 +  _li + 1 ,8 +  _li)  -= _term;
 _term =  DFree * _zfrat [ _li + 1 ] * 1.0 ;
 _MATELM1( 8 +  _li ,8 +  _li + 1)  -= _term;
 _MATELM1( 8 +  _li + 1 ,8 +  _li + 1)  += _term;
 /*REACTION*/
  } }
   _zdsq = diam * diam * 1.0 ;
   {int  _li ;for ( _li = 0 ; _li <= 4 - 1 ; _li ++ ) {
     _zdsqvol = _zdsq * vol [ _li ] ;
     /* ~ ca [ _li ] + Buffer [ _li ] <-> CaBuffer [ _li ] ( k1buf * _zdsqvol , k2buf * _zdsqvol )*/
 f_flux =  k1buf * _zdsqvol * Buffer [ _li] * ca [ _li] ;
 b_flux =  k2buf * _zdsqvol * CaBuffer [ _li] ;
 _RHS1( 0 +  _li) -= (f_flux - b_flux);
 _RHS1( 8 +  _li) -= (f_flux - b_flux);
 _RHS1( 4 +  _li) += (f_flux - b_flux);
 
 _term =  k1buf * _zdsqvol * ca [ _li] ;
 _MATELM1( 0 +  _li ,0 +  _li)  += _term;
 _MATELM1( 8 +  _li ,0 +  _li)  += _term;
 _MATELM1( 4 +  _li ,0 +  _li)  -= _term;
 _term =  k1buf * _zdsqvol * Buffer [ _li] ;
 _MATELM1( 0 +  _li ,8 +  _li)  += _term;
 _MATELM1( 8 +  _li ,8 +  _li)  += _term;
 _MATELM1( 4 +  _li ,8 +  _li)  -= _term;
 _term =  k2buf * _zdsqvol ;
 _MATELM1( 0 +  _li ,4 +  _li)  -= _term;
 _MATELM1( 8 +  _li ,4 +  _li)  -= _term;
 _MATELM1( 4 +  _li ,4 +  _li)  += _term;
 /*REACTION*/
  } }
   /* ~ ca [ 0 ] + pump <-> pumpca ( ( 1.e-11 ) * k1 * area , ( 1.e7 ) * k2 * area )*/
 f_flux =  ( 1.e-11 ) * k1 * area * pump * ca [ 0] ;
 b_flux =  ( 1.e7 ) * k2 * area * pumpca ;
 _RHS1( 13) -= (f_flux - b_flux);
 _RHS1( 8 +  0) -= (f_flux - b_flux);
 _RHS1( 12) += (f_flux - b_flux);
 
 _term =  ( 1.e-11 ) * k1 * area * ca [ 0] ;
 _MATELM1( 13 ,13)  += _term;
 _MATELM1( 8 +  0 ,13)  += _term;
 _MATELM1( 12 ,13)  -= _term;
 _term =  ( 1.e-11 ) * k1 * area * pump ;
 _MATELM1( 13 ,8 +  0)  += _term;
 _MATELM1( 8 +  0 ,8 +  0)  += _term;
 _MATELM1( 12 ,8 +  0)  -= _term;
 _term =  ( 1.e7 ) * k2 * area ;
 _MATELM1( 13 ,12)  -= _term;
 _MATELM1( 8 +  0 ,12)  -= _term;
 _MATELM1( 12 ,12)  += _term;
 /*REACTION*/
  /* ~ pumpca <-> pump + cao ( ( 1.e7 ) * k3 * area , ( 1.e-11 ) * k4 * area )*/
 f_flux =  ( 1.e7 ) * k3 * area * pumpca ;
 b_flux =  ( 1.e-11 ) * k4 * area * cao * pump ;
 _RHS1( 12) -= (f_flux - b_flux);
 _RHS1( 13) += (f_flux - b_flux);
 
 _term =  ( 1.e7 ) * k3 * area ;
 _MATELM1( 12 ,12)  += _term;
 _MATELM1( 13 ,12)  -= _term;
 _term =  ( 1.e-11 ) * k4 * area * cao ;
 _MATELM1( 12 ,13)  -= _term;
 _MATELM1( 13 ,13)  += _term;
 /*REACTION*/
  ipump = 2.0 * FARADAY * ( f_flux - b_flux ) / area ;
   cai = ca [ 0 ] ;
     } return _reset;
 }
 
/*CVODE ode begin*/
 static int _ode_spec1() {_reset=0;{
 double b_flux, f_flux, _term; int _i;
 {int _i; for(_i=0;_i<14;_i++) _ml->data(_iml, _dlist1[_i]) = 0.0;}
 /* COMPARTMENT _li , diam * diam * vol [ ((int) _i ) ] * 1.0 {
   ca CaBuffer Buffer }
 */
 /* COMPARTMENT ( 1.e10 ) * area {
   pump pumpca }
 */
 /* COMPARTMENT ( 1.e15 ) * volo {
   }
 */
 /* ~ ca [ 0 ] < < ( - ( ica - last_ipump ) * PI * diam * _zfrat [ 0 ] * 1.0 / ( 2.0 * FARADAY ) )*/
 f_flux = b_flux = 0.;
 Dca [ 0] += (b_flux =   ( - ( ica - last_ipump ) * PI * diam * _zfrat [ 0 ] * 1.0 / ( 2.0 * FARADAY ) ) );
 /*FLUX*/
  {int  _li ;for ( _li = 0 ; _li <= 4 - 2 ; _li ++ ) {
   /* ~ ca [ _li ] <-> ca [ _li + 1 ] ( DFree * _zfrat [ _li + 1 ] * 1.0 , DFree * _zfrat [ _li + 1 ] * 1.0 )*/
 f_flux =  DFree * _zfrat [ _li + 1 ] * 1.0 * ca [ _li] ;
 b_flux =  DFree * _zfrat [ _li + 1 ] * 1.0 * ca [ _li + 1] ;
 Dca [ _li] -= (f_flux - b_flux);
 Dca [ _li + 1] += (f_flux - b_flux);
 
 /*REACTION*/
  } }
 _zdsq = diam * diam * 1.0 ;
 {int  _li ;for ( _li = 0 ; _li <= 4 - 1 ; _li ++ ) {
   _zdsqvol = _zdsq * vol [ _li ] ;
   /* ~ ca [ _li ] + Buffer [ _li ] <-> CaBuffer [ _li ] ( k1buf * _zdsqvol , k2buf * _zdsqvol )*/
 f_flux =  k1buf * _zdsqvol * Buffer [ _li] * ca [ _li] ;
 b_flux =  k2buf * _zdsqvol * CaBuffer [ _li] ;
 DBuffer [ _li] -= (f_flux - b_flux);
 Dca [ _li] -= (f_flux - b_flux);
 DCaBuffer [ _li] += (f_flux - b_flux);
 
 /*REACTION*/
  } }
 /* ~ ca [ 0 ] + pump <-> pumpca ( ( 1.e-11 ) * k1 * area , ( 1.e7 ) * k2 * area )*/
 f_flux =  ( 1.e-11 ) * k1 * area * pump * ca [ 0] ;
 b_flux =  ( 1.e7 ) * k2 * area * pumpca ;
 Dpump -= (f_flux - b_flux);
 Dca [ 0] -= (f_flux - b_flux);
 Dpumpca += (f_flux - b_flux);
 
 /*REACTION*/
  /* ~ pumpca <-> pump + cao ( ( 1.e7 ) * k3 * area , ( 1.e-11 ) * k4 * area )*/
 f_flux =  ( 1.e7 ) * k3 * area * pumpca ;
 b_flux =  ( 1.e-11 ) * k4 * area * cao * pump ;
 Dpumpca -= (f_flux - b_flux);
 Dpump += (f_flux - b_flux);
 
 /*REACTION*/
  ipump = 2.0 * FARADAY * ( f_flux - b_flux ) / area ;
 cai = ca [ 0 ] ;
 for (_i=0; _i < 4; _i++) { _ml->data(_iml, _dlist1[_i + 0]) /= ( diam * diam * vol [ ((int) _i ) ] * 1.0);}
 for (_i=0; _i < 4; _i++) { _ml->data(_iml, _dlist1[_i + 4]) /= ( diam * diam * vol [ ((int) _i ) ] * 1.0);}
 for (_i=0; _i < 4; _i++) { _ml->data(_iml, _dlist1[_i + 8]) /= ( diam * diam * vol [ ((int) _i ) ] * 1.0);}
 _ml->data(_iml, _dlist1[12]) /= ( ( 1.e10 ) * area);
 _ml->data(_iml, _dlist1[13]) /= ( ( 1.e10 ) * area);
   } return _reset;
 }
 
/*CVODE matsol*/
 static int _ode_matsol1() {_reset=0;{
 double b_flux, f_flux, _term; int _i;
   b_flux = f_flux = 0.;
 {int _i; double _dt1 = 1.0/dt;
for(_i=0;_i<14;_i++){
  	_RHS1(_i) = _dt1*(_ml->data(_iml, _dlist1[_i]));
	_MATELM1(_i, _i) = _dt1;
      
}  
_RHS1(12) *= ( ( 1.e10 ) * area) ;
_MATELM1(12, 12) *= ( ( 1.e10 ) * area); 
_RHS1(13) *= ( ( 1.e10 ) * area) ;
_MATELM1(13, 13) *= ( ( 1.e10 ) * area);  
for (_i=0; _i < 4; _i++) {
  	_RHS1(_i + 0) *= ( diam * diam * vol [ ((int) _i ) ] * 1.0) ;
_MATELM1(_i + 0, _i + 0) *= ( diam * diam * vol [ ((int) _i ) ] * 1.0);  } 
for (_i=0; _i < 4; _i++) {
  	_RHS1(_i + 4) *= ( diam * diam * vol [ ((int) _i ) ] * 1.0) ;
_MATELM1(_i + 4, _i + 4) *= ( diam * diam * vol [ ((int) _i ) ] * 1.0);  } 
for (_i=0; _i < 4; _i++) {
  	_RHS1(_i + 8) *= ( diam * diam * vol [ ((int) _i ) ] * 1.0) ;
_MATELM1(_i + 8, _i + 8) *= ( diam * diam * vol [ ((int) _i ) ] * 1.0);  } }
 /* COMPARTMENT _li , diam * diam * vol [ ((int) _i ) ] * 1.0 {
 ca CaBuffer Buffer }
 */
 /* COMPARTMENT ( 1.e10 ) * area {
 pump pumpca }
 */
 /* COMPARTMENT ( 1.e15 ) * volo {
 }
 */
 /* ~ ca [ 0 ] < < ( - ( ica - last_ipump ) * PI * diam * _zfrat [ 0 ] * 1.0 / ( 2.0 * FARADAY ) )*/
 /*FLUX*/
  {int  _li ;for ( _li = 0 ; _li <= 4 - 2 ; _li ++ ) {
 /* ~ ca [ _li ] <-> ca [ _li + 1 ] ( DFree * _zfrat [ _li + 1 ] * 1.0 , DFree * _zfrat [ _li + 1 ] * 1.0 )*/
 _term =  DFree * _zfrat [ _li + 1 ] * 1.0 ;
 _MATELM1( 8 +  _li ,8 +  _li)  += _term;
 _MATELM1( 8 +  _li + 1 ,8 +  _li)  -= _term;
 _term =  DFree * _zfrat [ _li + 1 ] * 1.0 ;
 _MATELM1( 8 +  _li ,8 +  _li + 1)  -= _term;
 _MATELM1( 8 +  _li + 1 ,8 +  _li + 1)  += _term;
 /*REACTION*/
  } }
 _zdsq = diam * diam * 1.0 ;
 {int  _li ;for ( _li = 0 ; _li <= 4 - 1 ; _li ++ ) {
 _zdsqvol = _zdsq * vol [ _li ] ;
 /* ~ ca [ _li ] + Buffer [ _li ] <-> CaBuffer [ _li ] ( k1buf * _zdsqvol , k2buf * _zdsqvol )*/
 _term =  k1buf * _zdsqvol * ca [ _li] ;
 _MATELM1( 0 +  _li ,0 +  _li)  += _term;
 _MATELM1( 8 +  _li ,0 +  _li)  += _term;
 _MATELM1( 4 +  _li ,0 +  _li)  -= _term;
 _term =  k1buf * _zdsqvol * Buffer [ _li] ;
 _MATELM1( 0 +  _li ,8 +  _li)  += _term;
 _MATELM1( 8 +  _li ,8 +  _li)  += _term;
 _MATELM1( 4 +  _li ,8 +  _li)  -= _term;
 _term =  k2buf * _zdsqvol ;
 _MATELM1( 0 +  _li ,4 +  _li)  -= _term;
 _MATELM1( 8 +  _li ,4 +  _li)  -= _term;
 _MATELM1( 4 +  _li ,4 +  _li)  += _term;
 /*REACTION*/
  } }
 /* ~ ca [ 0 ] + pump <-> pumpca ( ( 1.e-11 ) * k1 * area , ( 1.e7 ) * k2 * area )*/
 _term =  ( 1.e-11 ) * k1 * area * ca [ 0] ;
 _MATELM1( 13 ,13)  += _term;
 _MATELM1( 8 +  0 ,13)  += _term;
 _MATELM1( 12 ,13)  -= _term;
 _term =  ( 1.e-11 ) * k1 * area * pump ;
 _MATELM1( 13 ,8 +  0)  += _term;
 _MATELM1( 8 +  0 ,8 +  0)  += _term;
 _MATELM1( 12 ,8 +  0)  -= _term;
 _term =  ( 1.e7 ) * k2 * area ;
 _MATELM1( 13 ,12)  -= _term;
 _MATELM1( 8 +  0 ,12)  -= _term;
 _MATELM1( 12 ,12)  += _term;
 /*REACTION*/
  /* ~ pumpca <-> pump + cao ( ( 1.e7 ) * k3 * area , ( 1.e-11 ) * k4 * area )*/
 _term =  ( 1.e7 ) * k3 * area ;
 _MATELM1( 12 ,12)  += _term;
 _MATELM1( 13 ,12)  -= _term;
 _term =  ( 1.e-11 ) * k4 * area * cao ;
 _MATELM1( 12 ,13)  -= _term;
 _MATELM1( 13 ,13)  += _term;
 cai = ca [ 0 ] ;
   } return _reset;
 }
 
/*CVODE end*/
 
static int _ode_count(int _type){ return 14;}
 
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
  cao = _ion_cao;
  cai = _ion_cai;
  ica = _ion_ica;
  cai = _ion_cai;
     _ode_spec1 ();
  _ion_cai = cai;
  }}
 
static void _ode_map(Prop* _prop, int _ieq, neuron::container::data_handle<double>* _pv, neuron::container::data_handle<double>* _pvdot, double* _atol, int _type) { 
  _ppvar = _nrn_mechanism_access_dparam(_prop);
  _cvode_ieq = _ieq;
  for (int _i=0; _i < 14; ++_i) {
    _pv[_i] = _nrn_mechanism_get_param_handle(_prop, _slist1[_i]);
    _pvdot[_i] = _nrn_mechanism_get_param_handle(_prop, _dlist1[_i]);
    _cvode_abstol(_atollist, _atol, _i);
  }
 }
 static void _ode_synonym(_nrn_model_sorted_token const& _sorted_token, NrnThread& _nt, Memb_list& _ml_arg, int _type) {
 _nrn_mechanism_cache_range _lmr{_sorted_token, _nt, _ml_arg, _type};
auto* const _ml = &_lmr;
auto const _cnt = _ml_arg._nodecount;
for (int _iml = 0; _iml < _cnt; ++_iml) {
  Datum* _ppvar = _ml_arg._pdata[_iml];
 _ion_cai =  ca [ 0 ] ;
   }
}
 
static void _ode_matsol_instance1(_internalthreadargsproto_) {
 _cvode_sparse(&_cvsparseobj1, 14, _dlist1, neuron::scopmath::row_view{_ml, _iml}, _ode_matsol1, &_coef1);
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
  cao = _ion_cao;
  cai = _ion_cai;
  ica = _ion_ica;
  cai = _ion_cai;
 _ode_matsol_instance1(_threadargs_);
 }}

static void initmodel() {
  int _i; double _save;_ninits++;
 _save = t;
 t = 0.0;
{
 for (_i=0; _i<4; _i++) Buffer[_i] = Buffer0;
 for (_i=0; _i<4; _i++) CaBuffer[_i] = CaBuffer0;
 for (_i=0; _i<4; _i++) ca[_i] = ca0;
  pumpca = pumpca0;
  pump = pump0;
 {
   if ( _zcoord_done  == 0.0 ) {
     _zcoord_done = 1.0 ;
     coord ( _threadargs_ ) ;
     }
   {int  _li ;for ( _li = 0 ; _li <= 4 - 1 ; _li ++ ) {
     ca [ _li ] = cai ;
     } }
   ipump = 0.0 ;
   pump = totpump ;
   pumpca = ( 1e-18 ) * pump * cao * k4 / k3 ;
   {int  _li ;for ( _li = 0 ; _li <= 4 - 1 ; _li ++ ) {
     ca [ _li ] = cai ;
     CaBuffer [ _li ] = ( totbuf * ca [ _li ] ) / ( k2buf / k1buf + ca [ _li ] ) ;
     Buffer [ _li ] = totbuf - CaBuffer [ _li ] ;
     } }
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
  cao = _ion_cao;
  cai = _ion_cai;
  ica = _ion_ica;
  cai = _ion_cai;
 initmodel();
  _ion_cai = cai;
   nrn_wrote_conc(_ca_sym, _ion_ca_erev, _ion_cai, _ion_cao, _style_ca);
}}

static double _nrn_current(double _v){double _current=0.;v=_v;{ {
   last_ipump = ipump ;
   ica = ipump ;
   test = 0.0 ;
   }
 _current += ica;

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
  cao = _ion_cao;
  cai = _ion_cai;
  ica = _ion_ica;
  cai = _ion_cai;
if (_nt->_vcv) { _ode_spec1(); }
 auto const _g_local = _nrn_current(_v + .001);
 	{ double _dica;
  _dica = ica;
 _rhs = _nrn_current(_v);
  _ion_dicadv += (_dica - ica)/.001 ;
 	}
 _g = (_g_local - _rhs)/.001;
  _ion_cai = cai;
  _ion_ica += ica ;
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
  cao = _ion_cao;
  cai = _ion_cai;
  ica = _ion_ica;
  cai = _ion_cai;
 { error = sparse(&_sparseobj1, 14, _slist1, _dlist1, neuron::scopmath::row_view{_ml, _iml}, &t, dt, state, &_coef1, _linmat1);
 if(error){
  std_cerr_stream << "at line 67 in file ca_ch.mod:\nBREAKPOINT {\n";
  std_cerr_stream << _ml << ' ' << _iml << '\n';
  abort_run(error);
}
    if (secondorder) {
    int _i;
    for (_i = 0; _i < 14; ++_i) {
      _ml->data(_iml, _slist1[_i]) += dt*_ml->data(_iml, _dlist1[_i]);
    }}
 }  _ion_cai = cai;
 }}
 dt = _dtsav;
}

static void terminal(){}

static void _initlists() {
 int _i; static int _first = 1;
  if (!_first) return;
 for(_i=0;_i<4;_i++){_slist1[0+_i] = {Buffer_columnindex, _i};  _dlist1[0+_i] = {DBuffer_columnindex, _i};}
 for(_i=0;_i<4;_i++){_slist1[4+_i] = {CaBuffer_columnindex, _i};  _dlist1[4+_i] = {DCaBuffer_columnindex, _i};}
 for(_i=0;_i<4;_i++){_slist1[8+_i] = {ca_columnindex, _i};  _dlist1[8+_i] = {Dca_columnindex, _i};}
 _slist1[12] = {pumpca_columnindex, 0};  _dlist1[12] = {Dpumpca_columnindex, 0};
 _slist1[13] = {pump_columnindex, 0};  _dlist1[13] = {Dpump_columnindex, 0};
_first = 0;
}

#if NMODL_TEXT
static void register_nmodl_text_and_filename(int mech_type) {
    const char* nmodl_filename = "/cfs/klemming/home/m/metog/BasalGangliaData/data/neurons/mechanisms/ca_ch.mod";
    const char* nmodl_file_text = 
  ":Migliore file Modify by Maciej Lazarewicz (mailto:mlazarew@gmu.edu) May/16/2001\n"
  "\n"
  "TITLE Calcium ion accumulation and diffusion\n"
  ": The internal coordinate system is set up in PROCEDURE coord_cadifus()\n"
  ": and must be executed before computing the concentrations.\n"
  ": The scale factors set up in this procedure do not have to be recomputed\n"
  ": when diam1 or DFree are changed.\n"
  ": The amount of calcium in an annulus is ca[i]*diam1^2*vol[i] with\n"
  ": ca[0] being the second order correct concentration at the exact edge\n"
  ": and ca[NANN-1] being the concentration at the exact center\n"
  "\n"
  "NEURON {\n"
  "	SUFFIX ca_ch\n"
  "	USEION ca READ cao, cai, ica WRITE cai, ica\n"
  "	RANGE ipump,last_ipump,test\n"
  "	GLOBAL DFree, k1buf, k2buf, k1, k2, k3, k4, totpump, totbuf\n"
  "	GLOBAL vol, Buffer0\n"
  "}\n"
  "\n"
  "DEFINE NANN  4\n"
  "\n"
  "UNITS {\n"
  "        (mol)   = (1)\n"
  "	(molar) = (1/liter)\n"
  "	(mM)	= (millimolar)\n"
  "	(um)	= (micron)\n"
  "	(mA)	= (milliamp)\n"
  "	FARADAY = (faraday)	(10000 coulomb)\n"
  "	PI	= (pi) 		(1)\n"
  "}\n"
  "\n"
  "PARAMETER {\n"
  "	DFree	= .6	(um2/ms)\n"
  "	diam 	= 1	(um)\n"
  "	cao		(mM)\n"
  "	ica		(mA/cm2)\n"
  "	k1buf 	= 500	(/mM-ms)\n"
  "	k2buf 	= 0.5	(/ms)\n"
  "        k1	= 1.e10 (um3/s)	\n"
  "        k2	= 50.e7 (/s)	: k1*50.e-3\n"
  "        k3	= 1.e10 (/s)	: k1\n"
  "        k4	= 5.e6	(um3/s)	: k1*5.e-4\n"
  "	totpump	= 0.2	(mol/cm2)\n"
  "	totbuf	= 1.2	(mM)\n"
  "} \n"
  "\n"
  "CONSTANT { volo=1  (liter)}\n"
  "\n"
  "ASSIGNED {\n"
  "	area		(um2)\n"
  "	test\n"
  "	cai		(mM)\n"
  "	vol[NANN]	(1)	: gets extra cm2 when multiplied by diam^2\n"
  "	ipump           (mA/cm2)\n"
  "	last_ipump	(mA/cm2)\n"
  "}\n"
  "\n"
  "STATE {\n"
  "	ca[NANN]	(mM) <1.e-5> : ca[0] is equivalent to cai\n"
  "	CaBuffer[NANN]  (mM)\n"
  "	Buffer[NANN]    (mM)\n"
  "        pump            (mol/cm2) <1.e-3>\n"
  "        pumpca          (mol/cm2) <1.e-15>\n"
  "\n"
  "}\n"
  "\n"
  "BREAKPOINT {\n"
  "	SOLVE state METHOD sparse\n"
  "	last_ipump=ipump\n"
  "	ica = ipump\n"
  "	test = 0\n"
  "}\n"
  "\n"
  "LOCAL coord_done\n"
  "\n"
  "INITIAL {\n"
  "	if (coord_done == 0) {\n"
  "		coord_done = 1\n"
  "		coord()\n"
  "	}\n"
  "	: note Buffer gets set to Buffer0 automatically\n"
  "	: and CaBuffer gets set to 0 (Default value of CaBuffer0) as well\n"
  "	FROM i=0 TO NANN-1 {\n"
  "		ca[i] = cai\n"
  "	}\n"
  " \n"
  "       	ipump 	= 0\n"
  "        pump 	= totpump\n"
  "        pumpca 	= (1e-18)*pump*cao*k4/k3\n"
  "\n"
  "        FROM i=0 TO NANN-1 {\n"
  "               	ca[i] = cai\n"
  "		CaBuffer[i] =(totbuf*ca[i])/(k2buf/k1buf+ca[i])\n"
  "		Buffer[i] = totbuf - CaBuffer[i]	\n"
  "	}\n"
  "}\n"
  "\n"
  "LOCAL frat[NANN] 	: gets extra cm when multiplied by diam\n"
  "\n"
  "PROCEDURE coord() {\n"
  "	LOCAL r, dr2\n"
  "	: cylindrical coordinate system  with constant annuli thickness to\n"
  "	: center of cell. Note however that the first annulus is half thickness\n"
  "	: so that the concentration is second order correct spatially at\n"
  "	: the membrane or exact edge of the cell.\n"
  "	: note ca[0] is at edge of cell\n"
  "	:      ca[NANN-1] is at center of cell\n"
  "	r = 1/2					:starts at edge (half diam)\n"
  "	dr2 = r/(NANN-1)/2			:half thickness of annulus\n"
  "	vol[0] = 0\n"
  "	frat[0] = 2*r\n"
  "	FROM i=0 TO NANN-2 {\n"
  "		vol[i] = vol[i] + PI*(r-dr2/2)*2*dr2	:interior half\n"
  "		r = r - dr2\n"
  "		frat[i+1] = 2*PI*r/(2*dr2)	:exterior edge of annulus\n"
  "					: divided by distance between centers\n"
  "		r = r - dr2\n"
  "		vol[i+1] = PI*(r+dr2/2)*2*dr2	:outer half of annulus\n"
  "	}\n"
  "}\n"
  "\n"
  "LOCAL dsq, dsqvol : can't define local variable in KINETIC block or use\n"
  "		:  in COMPARTMENT\n"
  "KINETIC state {\n"
  "	COMPARTMENT i, diam*diam*vol[i]*1(um) {ca CaBuffer Buffer}\n"
  "        COMPARTMENT (1.e10)*area {pump pumpca}\n"
  "        COMPARTMENT (1.e15)*volo {cao}\n"
  "\n"
  "	~ ca[0] << (-(ica-last_ipump)*PI*diam*frat[0]*1(um)/(2*FARADAY))\n"
  "\n"
  "	FROM i=0 TO NANN-2 {\n"
  "		~ ca[i] <-> ca[i+1] 	(DFree*frat[i+1]*1(um), DFree*frat[i+1]*1(um))\n"
  "	}\n"
  "\n"
  "	dsq = diam*diam*1(um)\n"
  "	FROM i=0 TO NANN-1 {\n"
  "		dsqvol = dsq*vol[i]\n"
  "		~ ca[i] + Buffer[i] <-> CaBuffer[i] (k1buf*dsqvol,k2buf*dsqvol)\n"
  "	}\n"
  "\n"
  "        ~ca[0] + pump <-> pumpca 	((1.e-11)*k1*area, (1.e7)*k2*area)\n"
  "        ~pumpca       <-> pump + cao 	((1.e7)*k3*area, (1.e-11)*k4*area)\n"
  "\n"
  "        ipump = 2*FARADAY*(f_flux-b_flux)/area\n"
  "\n"
  "	cai = ca[0]\n"
  "}\n"
  "	\n"
  "COMMENT\n"
  "At this time, conductances (and channel states and currents are\n"
  "calculated at the midpoint of a dt interval.  Membrane potential and\n"
  "concentrations are calculated at the edges of a dt interval.  With\n"
  "secondorder=2 everything turns out to be second order correct.\n"
  "\n"
  "From model:\n"
  "Maurice N, Mercer J, Chan CS, Hernandez-Lopez S, Held J, Tkatch T, Surmeier DJ. \n"
  "D2 dopamine receptor-mediated modulation of voltage-dependent Na+ channels \n"
  "reduces autonomous activity in striatal cholinergic interneurons. \n"
  "J Neurosci. 2004 Nov 17;24(46):10289-301. doi: 10.1523/JNEUROSCI.2155-04.2004. \n"
  "PMID: 15548642; PMCID: PMC6730305.\n"
  "\n"
  "ENDCOMMENT\n"
  "\n"
  "\n"
  ;
    hoc_reg_nmodl_filename(mech_type, nmodl_filename);
    hoc_reg_nmodl_text(mech_type, nmodl_file_text);
}
#endif
