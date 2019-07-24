#include <roki/rk_chain.h>

#define RK_WARN_LINK_DUP    "%s: name of a link duplicated."

#define RK_WARN_CHAIN_EMPTY "empty chain specified."

#define ZTK_TAG_RKMOTOR "motor"



/* read a 6D spatial vector from a ZTK format processor. */
zVec6D *zVec6DFromZTK(zVec6D *v, ZTK *ztk)
{
  v->e[0] = ZTKDouble(ztk);
  v->e[1] = ZTKDouble(ztk);
  v->e[2] = ZTKDouble(ztk);
  v->e[3] = ZTKDouble(ztk);
  v->e[4] = ZTKDouble(ztk);
  v->e[5] = ZTKDouble(ztk);
  return v;
}

/* read a 3x3 matrix from a ZTK format processor. */
zMat3D *zMat3DFromZTK(zMat3D *m, ZTK *ztk)
{
  m->e[0][0] = ZTKDouble(ztk);
  m->e[1][0] = ZTKDouble(ztk);
  m->e[2][0] = ZTKDouble(ztk);
  m->e[0][1] = ZTKDouble(ztk);
  m->e[1][1] = ZTKDouble(ztk);
  m->e[2][1] = ZTKDouble(ztk);
  m->e[0][2] = ZTKDouble(ztk);
  m->e[1][2] = ZTKDouble(ztk);
  m->e[2][2] = ZTKDouble(ztk);
  return m;
}

/* read a 3D frame from a ZTK format processor. */
zFrame3D *zFrame3DFromZTK(zFrame3D *f, ZTK *ztk)
{
  zFrame3DAtt(f)->e[0][0] = ZTKDouble(ztk);
  zFrame3DAtt(f)->e[1][0] = ZTKDouble(ztk);
  zFrame3DAtt(f)->e[2][0] = ZTKDouble(ztk);
  zFrame3DPos(f)->e[0]    = ZTKDouble(ztk);
  zFrame3DAtt(f)->e[0][1] = ZTKDouble(ztk);
  zFrame3DAtt(f)->e[1][1] = ZTKDouble(ztk);
  zFrame3DAtt(f)->e[2][1] = ZTKDouble(ztk);
  zFrame3DPos(f)->e[1]    = ZTKDouble(ztk);
  zFrame3DAtt(f)->e[0][2] = ZTKDouble(ztk);
  zFrame3DAtt(f)->e[1][2] = ZTKDouble(ztk);
  zFrame3DAtt(f)->e[2][2] = ZTKDouble(ztk);
  zFrame3DPos(f)->e[2]    = ZTKDouble(ztk);
  return f;
}

/* read DH parameters from a ZTK format processor. */
zFrame3D *zFrame3DDHFromZTK(zFrame3D *f, ZTK *ztk)
{
  double a, alpha, d, theta;

  a = ZTKDouble(ztk);
  alpha = zDeg2Rad( ZTKDouble(ztk) );
  d = ZTKDouble(ztk);
  theta = zDeg2Rad( ZTKDouble(ztk) );
  return zFrame3DFromDH( f, a, alpha, d, theta );
}




/* DC motor */

static void *_rkMotorDCMotorConstantFromZTK(void *obj, int i, void *arg, ZTK *ztk){
  ((rkMotorPrpDC*)((rkMotor*)obj)->prp)->k = ZTKDouble(ztk);
  return obj;
}
static void *_rkMotorDCAdmittanceFromZTK(void *obj, int i, void *arg, ZTK *ztk){
  ((rkMotorPrpDC*)((rkMotor*)obj)->prp)->admit = ZTKDouble(ztk);
  return obj;
}
static void *_rkMotorDCMaxVoltageFromZTK(void *obj, int i, void *arg, ZTK *ztk){
  ((rkMotorPrpDC*)((rkMotor*)obj)->prp)->maxvol = ZTKDouble(ztk);
  return obj;
}
static void *_rkMotorDCMinVoltageFromZTK(void *obj, int i, void *arg, ZTK *ztk){
  ((rkMotorPrpDC*)((rkMotor*)obj)->prp)->minvol = ZTKDouble(ztk);
  return obj;
}
static void *_rkMotorDCGearRatioFromZTK(void *obj, int i, void *arg, ZTK *ztk){
  ((rkMotorPrpDC*)((rkMotor*)obj)->prp)->decratio = ZTKDouble(ztk);
  return obj;
}
static void *_rkMotorDCRotorInertiaFromZTK(void *obj, int i, void *arg, ZTK *ztk){
  ((rkMotorPrpDC*)((rkMotor*)obj)->prp)->inertia = ZTKDouble(ztk);
  return obj;
}
static void *_rkMotorDCGearInertiaFromZTK(void *obj, int i, void *arg, ZTK *ztk){
  ((rkMotorPrpDC*)((rkMotor*)obj)->prp)->inertia_gear = ZTKDouble(ztk);
  return obj;
}
static void *_rkMotorDCCompKFromZTK(void *obj, int i, void *arg, ZTK *ztk){
  ((rkMotorPrpDC*)((rkMotor*)obj)->prp)->_comp_k = ZTKDouble(ztk);
  return obj;
}
static void *_rkMotorDCCompLFromZTK(void *obj, int i, void *arg, ZTK *ztk){
  ((rkMotorPrpDC*)((rkMotor*)obj)->prp)->_comp_l = ZTKDouble(ztk);
  return obj;
}

static void _rkMotorDCMotorConstantFPrint(FILE *fp, int i, void *obj){
  fprintf( fp, "%.10g\n", ((rkMotorPrpDC*)((rkMotor*)obj)->prp)->k );
}
static void _rkMotorDCAdmittanceFPrint(FILE *fp, int i, void *obj){
  fprintf( fp, "%.10g\n", ((rkMotorPrpDC*)((rkMotor*)obj)->prp)->admit );
}
static void _rkMotorDCMaxVoltageFPrint(FILE *fp, int i, void *obj){
  fprintf( fp, "%.10g\n", ((rkMotorPrpDC*)((rkMotor*)obj)->prp)->maxvol );
}
static void _rkMotorDCMinVoltageFPrint(FILE *fp, int i, void *obj){
  fprintf( fp, "%.10g\n", ((rkMotorPrpDC*)((rkMotor*)obj)->prp)->minvol );
}
static void _rkMotorDCGearRatioFPrint(FILE *fp, int i, void *obj){
  fprintf( fp, "%.10g\n", ((rkMotorPrpDC*)((rkMotor*)obj)->prp)->decratio );
}
static void _rkMotorDCRotorInertiaFPrint(FILE *fp, int i, void *obj){
  fprintf( fp, "%.10g\n", ((rkMotorPrpDC*)((rkMotor*)obj)->prp)->inertia );
}
static void _rkMotorDCGearInertiaFPrint(FILE *fp, int i, void *obj){
  fprintf( fp, "%.10g\n", ((rkMotorPrpDC*)((rkMotor*)obj)->prp)->inertia_gear );
}
static void _rkMotorDCCompKFPrint(FILE *fp, int i, void *obj){
  fprintf( fp, "%.10g\n", ((rkMotorPrpDC*)((rkMotor*)obj)->prp)->_comp_k );
}
static void _rkMotorDCCompLFPrint(FILE *fp, int i, void *obj){
  fprintf( fp, "%.10g\n", ((rkMotorPrpDC*)((rkMotor*)obj)->prp)->_comp_l );
}

static ZTKPrp __ztk_prp_rkmotor_dc[] = {
  { "motorconstant", 1, _rkMotorDCMotorConstantFromZTK, _rkMotorDCMotorConstantFPrint },
  { "admittance", 1, _rkMotorDCAdmittanceFromZTK, _rkMotorDCAdmittanceFPrint },
  { "maxvoltage", 1, _rkMotorDCMaxVoltageFromZTK, _rkMotorDCMaxVoltageFPrint },
  { "minvoltage", 1, _rkMotorDCMinVoltageFromZTK, _rkMotorDCMinVoltageFPrint },
  { "gearratio", 1, _rkMotorDCGearRatioFromZTK, _rkMotorDCGearRatioFPrint },
  { "rotorinertia", 1, _rkMotorDCRotorInertiaFromZTK, _rkMotorDCRotorInertiaFPrint },
  { "gearinertia", 1, _rkMotorDCGearInertiaFromZTK, _rkMotorDCGearInertiaFPrint },
  { "compk", 1, _rkMotorDCCompKFromZTK, _rkMotorDCCompKFPrint },
  { "compl", 1, _rkMotorDCCompLFromZTK, _rkMotorDCCompLFPrint },
};

bool rkMotorDCRegZTK(ZTK *ztk, char *tag)
{
  return ZTKDefRegPrp( ztk, tag, __ztk_prp_rkmotor_dc );
}

rkMotor *rkMotorDCFromZTK(rkMotor *motor, ZTK *ztk)
{
  return ZTKEncodeKey( motor, NULL, ztk, __ztk_prp_rkmotor_dc );
}

void rkMotorDCFPrint(FILE *fp, rkMotor *motor)
{
  ZTKPrpKeyFPrint( fp, motor, __ztk_prp_rkmotor_dc );
}


/* direct torque motor */

static void *_rkMotorTrqMaxFromZTK(void *obj, int i, void *arg, ZTK *ztk){
  ((rkMotorPrpTRQ *)((rkMotor*)obj)->prp)->max = ZTKDouble(ztk);
  return obj;
}
static void *_rkMotorTrqMinFromZTK(void *obj, int i, void *arg, ZTK *ztk){
  ((rkMotorPrpTRQ *)((rkMotor*)obj)->prp)->min = ZTKDouble(ztk);
  return obj;
}

static void _rkMotorTrqMaxFPrint(FILE *fp, int i, void *obj){
  fprintf( fp, "%.10g\n", ((rkMotorPrpTRQ *)((rkMotor*)obj)->prp)->max );
}
static void _rkMotorTrqMinFPrint(FILE *fp, int i, void *obj){
  fprintf( fp, "%.10g\n", ((rkMotorPrpTRQ *)((rkMotor*)obj)->prp)->min );
}

static ZTKPrp __ztk_prp_rkmotor_trq[] = {
  { "max", 1, _rkMotorTrqMaxFromZTK, _rkMotorTrqMaxFPrint },
  { "min", 1, _rkMotorTrqMinFromZTK, _rkMotorTrqMinFPrint },
};

bool rkMotorTrqRegZTK(ZTK *ztk, char *tag)
{
  return ZTKDefRegPrp( ztk, tag, __ztk_prp_rkmotor_trq );
}

rkMotor *rkMotorTrqFromZTK(rkMotor *motor, ZTK *ztk)
{
  return ZTKEncodeKey( motor, NULL, ztk, __ztk_prp_rkmotor_trq );
}

void rkMotorTrqFPrint(FILE *fp, rkMotor *motor)
{
  ZTKPrpKeyFPrint( fp, motor, __ztk_prp_rkmotor_trq );
}




static void *_rkMotorNameFromZTK(void *obj, int i, void *arg, ZTK *ztk){
  zNameSet( (rkMotor*)obj, ZTKVal(ztk) );
  return zNamePtr((rkMotor*)obj) ? obj : NULL;
}
static void *_rkMotorTypeFromZTK(void *obj, int i, void *arg, ZTK *ztk){
  rkMotorType((rkMotor*)obj) = rkMotorTypeFromStr( ZTKVal(ztk) );
  return obj;
}

static void _rkMotorNameFPrint(FILE *fp, int i, void *obj){
  fprintf( fp, "%s\n", zName( (rkMotor*)obj ) );
}
static void _rkMotorTypeFPrint(FILE *fp, int i, void *obj){
  fprintf( fp, "%s\n", rkMotorTypeExpr( rkMotorType((rkMotor*)obj) ) );
}

static ZTKPrp __ztk_prp_rkmotor[] = {
  { "name", 1, _rkMotorNameFromZTK, _rkMotorNameFPrint },
  { "type", 1, _rkMotorTypeFromZTK, _rkMotorTypeFPrint },
};

bool rkMotorRegZTK(ZTK *ztk)
{
  return ZTKDefRegPrp( ztk, ZTK_TAG_RKMOTOR, __ztk_prp_rkmotor ) &&
         rkMotorTrqRegZTK( ztk, ZTK_TAG_RKMOTOR ) &&
         rkMotorDCRegZTK( ztk, ZTK_TAG_RKMOTOR );
}

rkMotor *rkMotorFromZTK(rkMotor *motor, ZTK *ztk)
{
  rkMotorInit( motor );
  if( !ZTKEncodeKey( motor, NULL, ztk, __ztk_prp_rkmotor ) ) return NULL;
  if( !rkMotorCreate( motor, rkMotorType(motor) ) ) return NULL;
  /* obviously, the following is a volatile implementation. */
  switch( rkMotorType(motor) ){
  case RK_MOTOR_TRQ: return rkMotorTrqFromZTK( motor, ztk );
  case RK_MOTOR_DC: return rkMotorDCFromZTK( motor, ztk );
  default: ;
  }
  return motor;
}

void rkMotorFPrint(FILE *fp, rkMotor *motor)
{
  if( !motor ) return;
  ZTKPrpKeyFPrint( fp, motor, __ztk_prp_rkmotor );
  /* obviously, the following is a volatile implementation. */
  switch( rkMotorType(motor) ){
  case RK_MOTOR_TRQ: rkMotorTrqFPrint( fp, motor ); break;
  case RK_MOTOR_DC: rkMotorDCFPrint( fp, motor ); break;
  default: ;
  }
  fprintf( fp, "\n" );
}




rkMotor *rkMotorArrayFind(rkMotorArray *marray, char *name, rkJoint *joint)
{
  rkMotor *mp;
  zNameFind( zArrayBuf(marray), zArraySize(marray), name, mp );
  if( !mp ){
    ZRUNERROR( RK_ERR_MOTOR_UNKNOWN, name );
    return NULL;
  }
  if( rkMotorSize(mp) != rkJointSize(joint) ){
    ZRUNERROR( RK_ERR_JOINT_SIZMISMATCH );
    return NULL;
  }
  return mp;
}





#define rkJointPrpFromZTK(joint, motorarray, ztk, ztkprp) \
  ( ZTKEncodeKey( joint, motorarray, ztk, ztkprp ) ? joint : NULL )



rkJoint *rkJointFixedFromZTK(rkJoint *joint, rkMotorArray *motorarray, ZTK *ztk)
{
  return joint;
}

void rkJointFixedFPrint(FILE *fp, rkJoint *joint){}



static void _rkJointLimDisRevol(void *prp, double *testval, double *limval){
  /* copied from rk_joint_revol.c */
  /* to be deleted */
  double angle;
  angle = zPhaseNormalize( *testval );
  *limval = zLimit( angle, ((rkJointPrpRevol*)prp)->min, ((rkJointPrpRevol*)prp)->max );
}

static void _rkJointSetDisRevol(void *prp, double *val){
  /* copied from rk_joint_revol.c */
  /* to be deleted */
  _rkJointLimDisRevol( prp, val, &((rkJointPrpRevol*)prp)->dis );
  zSinCos( ((rkJointPrpRevol*)prp)->dis, &((rkJointPrpRevol*)prp)->_s, &((rkJointPrpRevol*)prp)->_c );
}

static void *_rkJointRevolDisFromZTK(void *obj, int i, void *arg, ZTK *ztk){
  double val;
  val = zDeg2Rad( ZTKDouble(ztk) );
  _rkJointSetDisRevol( ((rkJoint*)obj)->prp, &val );
  return obj;
}
static void *_rkJointRevolMinFromZTK(void *obj, int i, void *arg, ZTK *ztk){
  ((rkJointPrpRevol *)((rkJoint*)obj)->prp)->min = zDeg2Rad(ZTKDouble(ztk));
  return obj;
}
static void *_rkJointRevolMaxFromZTK(void *obj, int i, void *arg, ZTK *ztk){
  ((rkJointPrpRevol *)((rkJoint*)obj)->prp)->max = zDeg2Rad(ZTKDouble(ztk));
  return obj;
}
static void *_rkJointRevolStiffnessFromZTK(void *obj, int i, void *arg, ZTK *ztk){
  ((rkJointPrpRevol *)((rkJoint*)obj)->prp)->stiff = ZTKDouble(ztk);
  return obj;
}
static void *_rkJointRevolViscosityFromZTK(void *obj, int i, void *arg, ZTK *ztk){
  ((rkJointPrpRevol *)((rkJoint*)obj)->prp)->viscos = ZTKDouble(ztk);
  return obj;
}
static void *_rkJointRevolCoulombFromZTK(void *obj, int i, void *arg, ZTK *ztk){
  ((rkJointPrpRevol *)((rkJoint*)obj)->prp)->coulomb = ZTKDouble(ztk);
  return obj;
}
static void *_rkJointRevolStaticFricFromZTK(void *obj, int i, void *arg, ZTK *ztk){
  ((rkJointPrpRevol *)((rkJoint*)obj)->prp)->sf = ZTKDouble(ztk);
  return obj;
}
static void *_rkJointRevolMotorFromZTK(void *obj, int i, void *arg, ZTK *ztk){
  rkMotor *mp;
  if( !( mp = rkMotorArrayFind( arg, ZTKVal(ztk), obj ) ) ) return NULL;
  return rkMotorClone( mp, &((rkJointPrpRevol *)((rkJoint*)obj)->prp)->m ) ? obj : NULL;
}

static void _rkJointRevolDisFPrint(FILE *fp, int i, void *obj){
  fprintf( fp, "%.10g\n", zRad2Deg(((rkJointPrpRevol *)obj)->dis) );
}
static void _rkJointRevolMinFPrint(FILE *fp, int i, void *obj){
  fprintf( fp, "%.10g\n", zRad2Deg(((rkJointPrpRevol *)obj)->min) );
}
static void _rkJointRevolMaxFPrint(FILE *fp, int i, void *obj){
  fprintf( fp, "%.10g\n", zRad2Deg(((rkJointPrpRevol *)obj)->max) );
}
static void _rkJointRevolStiffnessFPrint(FILE *fp, int i, void *obj){
  fprintf( fp, "%.10g\n", ((rkJointPrpRevol *)obj)->stiff );
}
static void _rkJointRevolViscosityFPrint(FILE *fp, int i, void *obj){
  fprintf( fp, "%.10g\n", ((rkJointPrpRevol *)obj)->viscos );
}
static void _rkJointRevolCoulombFPrint(FILE *fp, int i, void *obj){
  fprintf( fp, "%.10g\n", ((rkJointPrpRevol *)obj)->coulomb );
}
static void _rkJointRevolStaticFricFPrint(FILE *fp, int i, void *obj){
  fprintf( fp, "%.10g\n", ((rkJointPrpRevol *)obj)->sf );
}
static void _rkJointRevolMotorFPrint(FILE *fp, int i, void *obj){
  fprintf( fp, "%s\n", zName(&((rkJointPrpRevol *)obj)->m) );
}

static ZTKPrp __ztk_prp_rkjoint_revol[] = {
  { "dis", 1, _rkJointRevolDisFromZTK, _rkJointRevolDisFPrint },
  { "min", 1, _rkJointRevolMinFromZTK, _rkJointRevolMinFPrint },
  { "max", 1, _rkJointRevolMaxFromZTK, _rkJointRevolMaxFPrint },
  { "stiffness", 1, _rkJointRevolStiffnessFromZTK, _rkJointRevolStiffnessFPrint },
  { "viscosity", 1, _rkJointRevolViscosityFromZTK, _rkJointRevolViscosityFPrint },
  { "coulomb", 1, _rkJointRevolCoulombFromZTK, _rkJointRevolCoulombFPrint },
  { "staticfriction", 1, _rkJointRevolStaticFricFromZTK, _rkJointRevolStaticFricFPrint },
  { "motor", 1, _rkJointRevolMotorFromZTK, _rkJointRevolMotorFPrint },
};

bool rkJointRevolRegZTK(ZTK *ztk, char *tag)
{
  return ZTKDefRegPrp( ztk, tag, __ztk_prp_rkjoint_revol ) ? true : false;
}

rkJoint *rkJointRevolFromZTK(rkJoint *joint, rkMotorArray *motorarray, ZTK *ztk)
{
  return rkJointPrpFromZTK( joint, motorarray, ztk, __ztk_prp_rkjoint_revol );
}

void rkJointRevolFPrint(FILE *fp, rkJoint *joint)
{
  ZTKPrpKeyFPrint( fp, joint->prp, __ztk_prp_rkjoint_revol );
}





static void _rkJointLimDisPrism(void *prp, double *testval, double *limval){
  /* copied from rk_joint_prism.c */
  /* to be deleted */
  *limval = zLimit( *testval, ((rkJointPrpPrism*)prp)->min, ((rkJointPrpPrism*)prp)->max );
}
static void _rkJointSetDisPrism(void *prp, double *val){
  /* copied from rk_joint_prism.c */
  /* to be deleted */
  _rkJointLimDisPrism( prp, val, &((rkJointPrpPrism*)prp)->dis );
}


static void *_rkJointPrismDisFromZTK(void *obj, int i, void *arg, ZTK *ztk){
  double val;
  val = zDeg2Rad( ZTKDouble(ztk) );
  _rkJointSetDisPrism( ((rkJoint*)obj)->prp, &val );
  return obj;
}
static void *_rkJointPrismMinFromZTK(void *obj, int i, void *arg, ZTK *ztk){
  ((rkJointPrpPrism *)((rkJoint*)obj)->prp)->min = zDeg2Rad(ZTKDouble(ztk));
  return obj;
}
static void *_rkJointPrismMaxFromZTK(void *obj, int i, void *arg, ZTK *ztk){
  ((rkJointPrpPrism *)((rkJoint*)obj)->prp)->max = zDeg2Rad(ZTKDouble(ztk));
  return obj;
}
static void *_rkJointPrismStiffnessFromZTK(void *obj, int i, void *arg, ZTK *ztk){
  ((rkJointPrpPrism *)((rkJoint*)obj)->prp)->stiff = zDeg2Rad(ZTKDouble(ztk));
  return obj;
}
static void *_rkJointPrismViscosityFromZTK(void *obj, int i, void *arg, ZTK *ztk){
  ((rkJointPrpPrism *)((rkJoint*)obj)->prp)->viscos = zDeg2Rad(ZTKDouble(ztk));
  return obj;
}
static void *_rkJointPrismCoulombFromZTK(void *obj, int i, void *arg, ZTK *ztk){
  ((rkJointPrpPrism *)((rkJoint*)obj)->prp)->coulomb = zDeg2Rad(ZTKDouble(ztk));
  return obj;
}
static void *_rkJointPrismStaticFricFromZTK(void *obj, int i, void *arg, ZTK *ztk){
  ((rkJointPrpPrism *)((rkJoint*)obj)->prp)->sf = zDeg2Rad(ZTKDouble(ztk));
  return obj;
}
static void *_rkJointPrismMotorFromZTK(void *obj, int i, void *arg, ZTK *ztk){
  rkMotor *mp;
  if( !( mp = rkMotorArrayFind( arg, ZTKVal(ztk), (rkJoint*)obj ) ) ) return NULL;
  return rkMotorClone( mp, &((rkJointPrpPrism *)((rkJoint*)obj)->prp)->m ) ? obj : NULL;
}

static void _rkJointPrismDisFPrint(FILE *fp, int i, void *obj){
  fprintf( fp, "%.10g\n", ((rkJointPrpPrism *)obj)->dis );
}
static void _rkJointPrismMinFPrint(FILE *fp, int i, void *obj){
  fprintf( fp, "%.10g\n", ((rkJointPrpPrism *)obj)->min );
}
static void _rkJointPrismMaxFPrint(FILE *fp, int i, void *obj){
  fprintf( fp, "%.10g\n", ((rkJointPrpPrism *)obj)->max );
}
static void _rkJointPrismStiffnessFPrint(FILE *fp, int i, void *obj){
  fprintf( fp, "%.10g\n", ((rkJointPrpPrism *)obj)->stiff );
}
static void _rkJointPrismViscosityFPrint(FILE *fp, int i, void *obj){
  fprintf( fp, "%.10g\n", ((rkJointPrpPrism *)obj)->viscos );
}
static void _rkJointPrismCoulombFPrint(FILE *fp, int i, void *obj){
  fprintf( fp, "%.10g\n", ((rkJointPrpPrism *)obj)->coulomb );
}
static void _rkJointPrismStaticFricFPrint(FILE *fp, int i, void *obj){
  fprintf( fp, "%.10g\n", ((rkJointPrpPrism *)obj)->sf );
}
static void _rkJointPrismMotorFPrint(FILE *fp, int i, void *obj){
  fprintf( fp, "%s\n", zName(&((rkJointPrpPrism *)obj)->m) );
}

static ZTKPrp __ztk_prp_rkjoint_prism[] = {
  { "dis", 1, _rkJointPrismDisFromZTK, _rkJointPrismDisFPrint },
  { "min", 1, _rkJointPrismMinFromZTK, _rkJointPrismMinFPrint },
  { "max", 1, _rkJointPrismMaxFromZTK, _rkJointPrismMaxFPrint },
  { "stiffness", 1, _rkJointPrismStiffnessFromZTK, _rkJointPrismStiffnessFPrint },
  { "viscosity", 1, _rkJointPrismViscosityFromZTK, _rkJointPrismViscosityFPrint },
  { "coulomb", 1, _rkJointPrismCoulombFromZTK, _rkJointPrismCoulombFPrint },
  { "staticfriction", 1, _rkJointPrismStaticFricFromZTK, _rkJointPrismStaticFricFPrint },
  { "motor", 1, _rkJointPrismMotorFromZTK, _rkJointPrismMotorFPrint },
};

bool rkJointPrismRegZTK(ZTK *ztk, char *tag)
{
  return ZTKDefRegPrp( ztk, tag, __ztk_prp_rkjoint_prism ) ? true : false;
}

rkJoint *rkJointPrismFromZTK(rkJoint *joint, rkMotorArray *motorarray, ZTK *ztk)
{
  return rkJointPrpFromZTK( joint, motorarray, ztk, __ztk_prp_rkjoint_prism );
}

void rkJointPrismFPrint(FILE *fp, rkJoint *joint)
{
  ZTKPrpKeyFPrint( fp, joint->prp, __ztk_prp_rkjoint_prism );
}



/* limit joint displacement */
static void _rkJointLimDisCylin(void *prp, double *testval, double *limval){
  /* copied from rk_joint_cylin.c */
  /* to be deleted */
  double angle;
  /* 0: prismatic */
  limval[0] = zLimit( testval[0], ((rkJointPrpCylin*)prp)->min[0], ((rkJointPrpCylin*)prp)->max[0] );
  /* 1: revolutional */
  angle = zPhaseNormalize( testval[1] );
  limval[1] = zLimit( angle, ((rkJointPrpCylin*)prp)->min[1], ((rkJointPrpCylin*)prp)->max[1] );
}
/* joint displacement set function */
static void _rkJointSetDisCylin(void *prp, double *val){
  /* copied from rk_joint_cylin.c */
  /* to be deleted */
  _rkJointLimDisCylin( prp, val, ((rkJointPrpCylin*)prp)->dis );
  zSinCos( ((rkJointPrpCylin*)prp)->dis[1], &((rkJointPrpCylin*)prp)->_s, &((rkJointPrpCylin*)prp)->_c );
}

static void *_rkJointCylinDisFromZTK(void *obj, int i, void *arg, ZTK *ztk){
  double dis[2];
  dis[0] = ZTKDouble(ztk);
  dis[1] = zDeg2Rad(ZTKDouble(ztk));
  _rkJointSetDisCylin( ((rkJoint*)obj)->prp, dis );
  return obj;
}
static void *_rkJointCylinMinFromZTK(void *obj, int i, void *arg, ZTK *ztk){
  ((rkJointPrpCylin*)((rkJoint*)obj)->prp)->min[0] = ZTKDouble(ztk);
  ((rkJointPrpCylin*)((rkJoint*)obj)->prp)->min[1] = zDeg2Rad(ZTKDouble(ztk));
  return obj;
}
static void *_rkJointCylinMaxFromZTK(void *obj, int i, void *arg, ZTK *ztk){
  ((rkJointPrpCylin*)((rkJoint*)obj)->prp)->max[0] = ZTKDouble(ztk);
  ((rkJointPrpCylin*)((rkJoint*)obj)->prp)->max[1] = zDeg2Rad(ZTKDouble(ztk));
  return obj;
}
static void *_rkJointCylinStiffFromZTK(void *obj, int i, void *arg, ZTK *ztk){
  ((rkJointPrpCylin*)((rkJoint*)obj)->prp)->stiff[0] = ZTKDouble(ztk);
  ((rkJointPrpCylin*)((rkJoint*)obj)->prp)->stiff[1] = ZTKDouble(ztk);
  return obj;
}
static void *_rkJointCylinViscosFromZTK(void *obj, int i, void *arg, ZTK *ztk){
  ((rkJointPrpCylin*)((rkJoint*)obj)->prp)->viscos[0] = ZTKDouble(ztk);
  ((rkJointPrpCylin*)((rkJoint*)obj)->prp)->viscos[1] = ZTKDouble(ztk);
  return obj;
}
static void *_rkJointCylinCoulombFromZTK(void *obj, int i, void *arg, ZTK *ztk){
  ((rkJointPrpCylin*)((rkJoint*)obj)->prp)->coulomb[0] = ZTKDouble(ztk);
  ((rkJointPrpCylin*)((rkJoint*)obj)->prp)->coulomb[1] = ZTKDouble(ztk);
  return obj;
}
static void *_rkJointCylinStaticFrictionFromZTK(void *obj, int i, void *arg, ZTK *ztk){
  ((rkJointPrpCylin*)((rkJoint*)obj)->prp)->sf[0] = ZTKDouble(ztk);
  ((rkJointPrpCylin*)((rkJoint*)obj)->prp)->sf[1] = ZTKDouble(ztk);
  return obj;
}
static void *_rkJointCylinMotorFromZTK(void *obj, int i, void *arg, ZTK *ztk){
  rkMotor *mp;
  if( !( mp = rkMotorArrayFind( arg, ZTKVal(ztk), (rkJoint*)obj ) ) ) return NULL;
  return rkMotorClone( mp, &((rkJointPrpCylin *)((rkJoint*)obj)->prp)->m ) ? obj : NULL;
}

static void _rkJointCylinDisFPrint(FILE *fp, int i, void *obj){
  fprintf( fp, "%.10g %.10g\n",
    ((rkJointPrpCylin *)((rkJoint*)obj)->prp)->dis[0],
    zRad2Deg(((rkJointPrpCylin *)((rkJoint*)obj)->prp)->dis[1]) );
}
static void _rkJointCylinMinFPrint(FILE *fp, int i, void *obj){
  fprintf( fp, "%.10g %.10g\n",
    ((rkJointPrpCylin*)((rkJoint*)obj)->prp)->min[0],
    zRad2Deg(((rkJointPrpCylin*)((rkJoint*)obj)->prp)->min[1]) );
}
static void _rkJointCylinMaxFPrint(FILE *fp, int i, void *obj){
  fprintf( fp, "%.10g %.10g\n",
    ((rkJointPrpCylin*)((rkJoint*)obj)->prp)->max[0],
    zRad2Deg(((rkJointPrpCylin*)((rkJoint*)obj)->prp)->max[1]) );
}
static void _rkJointCylinStiffFPrint(FILE *fp, int i, void *obj){
  fprintf( fp, "%.10g %.10g\n",
    ((rkJointPrpCylin*)((rkJoint*)obj)->prp)->stiff[0],
    ((rkJointPrpCylin*)((rkJoint*)obj)->prp)->stiff[1] );
}
static void _rkJointCylinViscosFPrint(FILE *fp, int i, void *obj){
  fprintf( fp, "%.10g %.10g\n",
    ((rkJointPrpCylin*)((rkJoint*)obj)->prp)->viscos[0],
    ((rkJointPrpCylin*)((rkJoint*)obj)->prp)->viscos[1] );
}
static void _rkJointCylinCoulombFPrint(FILE *fp, int i, void *obj){
  fprintf( fp, "%.10g %.10g\n",
    ((rkJointPrpCylin*)((rkJoint*)obj)->prp)->coulomb[0],
    ((rkJointPrpCylin*)((rkJoint*)obj)->prp)->coulomb[1] );
}
static void _rkJointCylinStaticFrictionFPrint(FILE *fp, int i, void *obj){
  fprintf( fp, "%.10g %.10g\n",
    ((rkJointPrpCylin*)((rkJoint*)obj)->prp)->sf[0],
    ((rkJointPrpCylin*)((rkJoint*)obj)->prp)->sf[1] );
}
static void _rkJointCylinMotorFPrint(FILE *fp, int i, void *obj){
  fprintf( fp, "%s\n", zName(&((rkJointPrpCylin *)obj)->m) );
}

static ZTKPrp __ztk_prp_rkjoint_cylin[] = {
  { "dis", 1, _rkJointCylinDisFromZTK, _rkJointCylinDisFPrint },
  { "min", 1, _rkJointCylinMinFromZTK, _rkJointCylinMinFPrint },
  { "max", 1, _rkJointCylinMaxFromZTK, _rkJointCylinMaxFPrint },
  { "stiffness", 1, _rkJointCylinStiffFromZTK, _rkJointCylinStiffFPrint },
  { "viscosity", 1, _rkJointCylinViscosFromZTK, _rkJointCylinViscosFPrint },
  { "coulomb", 1, _rkJointCylinCoulombFromZTK, _rkJointCylinCoulombFPrint },
  { "staticfriction", 1, _rkJointCylinStaticFrictionFromZTK, _rkJointCylinStaticFrictionFPrint },
  { "motor", 1, _rkJointCylinMotorFromZTK, _rkJointCylinMotorFPrint },
};

bool rkJointCylinRegZTK(ZTK *ztk, char *tag)
{
  return ZTKDefRegPrp( ztk, tag, __ztk_prp_rkjoint_cylin ) ? true : false;
}

rkJoint *rkJointCylinFromZTK(rkJoint *joint, rkMotorArray *motorarray, ZTK *ztk)
{
  return rkJointPrpFromZTK( joint, motorarray, ztk, __ztk_prp_rkjoint_cylin );
}

void rkJointCylinFPrint(FILE *fp, rkJoint *joint)
{
  ZTKPrpKeyFPrint( fp, joint->prp, __ztk_prp_rkjoint_cylin );
}




/* limit joint displacement */
static double _rkJointLimDis1Hooke(void *prp, int i, double testval){
  /* copied from rk_joint_hooke.c */
  /* to be deleted */
  testval = zPhaseNormalize( testval );
  return zLimit( testval, ((rkJointPrpHooke*)prp)->min[i], ((rkJointPrpHooke*)prp)->max[i] );
}
/* joint displacement set function */
static void _rkJointSetDis1Hooke(void *prp, int i, double val){
  /* copied from rk_joint_hooke.c */
  /* to be deleted */
  ((rkJointPrpHooke*)prp)->dis[i] = _rkJointLimDis1Hooke( prp, i, val );
  zSinCos( ((rkJointPrpHooke*)prp)->dis[i], &((rkJointPrpHooke*)prp)->_s[i], &((rkJointPrpHooke*)prp)->_c[i] );
}

static void *_rkJointHookeDisFromZTK(void *obj, int i, void *arg, ZTK *ztk){
  _rkJointSetDis1Hooke( ((rkJoint*)obj)->prp, 0, zDeg2Rad(ZTKDouble(ztk)) );
  _rkJointSetDis1Hooke( ((rkJoint*)obj)->prp, 1, zDeg2Rad(ZTKDouble(ztk)) );
  return obj;
}
static void *_rkJointHookeMinFromZTK(void *obj, int i, void *arg, ZTK *ztk){
  ((rkJointPrpHooke*)((rkJoint*)obj)->prp)->min[0] = zDeg2Rad(ZTKDouble(ztk));
  ((rkJointPrpHooke*)((rkJoint*)obj)->prp)->min[1] = zDeg2Rad(ZTKDouble(ztk));
  return obj;
}
static void *_rkJointHookeMaxFromZTK(void *obj, int i, void *arg, ZTK *ztk){
  ((rkJointPrpHooke*)((rkJoint*)obj)->prp)->max[0] = zDeg2Rad(ZTKDouble(ztk));
  ((rkJointPrpHooke*)((rkJoint*)obj)->prp)->max[1] = zDeg2Rad(ZTKDouble(ztk));
  return obj;
}
static void *_rkJointHookeStiffFromZTK(void *obj, int i, void *arg, ZTK *ztk){
  ((rkJointPrpHooke*)((rkJoint*)obj)->prp)->stiff[0] = ZTKDouble(ztk);
  ((rkJointPrpHooke*)((rkJoint*)obj)->prp)->stiff[1] = ZTKDouble(ztk);
  return obj;
}
static void *_rkJointHookeViscosFromZTK(void *obj, int i, void *arg, ZTK *ztk){
  ((rkJointPrpHooke*)((rkJoint*)obj)->prp)->viscos[0] = ZTKDouble(ztk);
  ((rkJointPrpHooke*)((rkJoint*)obj)->prp)->viscos[1] = ZTKDouble(ztk);
  return obj;
}
static void *_rkJointHookeCoulombFromZTK(void *obj, int i, void *arg, ZTK *ztk){
  ((rkJointPrpHooke*)((rkJoint*)obj)->prp)->coulomb[0] = ZTKDouble(ztk);
  ((rkJointPrpHooke*)((rkJoint*)obj)->prp)->coulomb[1] = ZTKDouble(ztk);
  return obj;
}
static void *_rkJointHookeStaticFrictionFromZTK(void *obj, int i, void *arg, ZTK *ztk){
  ((rkJointPrpHooke*)((rkJoint*)obj)->prp)->sf[0] = ZTKDouble(ztk);
  ((rkJointPrpHooke*)((rkJoint*)obj)->prp)->sf[1] = ZTKDouble(ztk);
  return obj;
}
static void *_rkJointHookeMotorFromZTK(void *obj, int i, void *arg, ZTK *ztk){
  rkMotor *mp;
  if( !( mp = rkMotorArrayFind( arg, ZTKVal(ztk), (rkJoint*)obj ) ) ) return NULL;
  return rkMotorClone( mp, &((rkJointPrpHooke *)((rkJoint*)obj)->prp)->m ) ? obj : NULL;
}

static void _rkJointHookeDisFPrint(FILE *fp, int i, void *obj){
  fprintf( fp, "%.10g %.10g\n",
    zRad2Deg(((rkJointPrpHooke *)((rkJoint*)obj)->prp)->dis[0]),
    zRad2Deg(((rkJointPrpHooke *)((rkJoint*)obj)->prp)->dis[1]) );
}
static void _rkJointHookeMinFPrint(FILE *fp, int i, void *obj){
  fprintf( fp, "%.10g %.10g\n",
    zRad2Deg(((rkJointPrpHooke*)((rkJoint*)obj)->prp)->min[0]),
    zRad2Deg(((rkJointPrpHooke*)((rkJoint*)obj)->prp)->min[1]) );
}
static void _rkJointHookeMaxFPrint(FILE *fp, int i, void *obj){
  fprintf( fp, "%.10g %.10g\n",
    zRad2Deg(((rkJointPrpHooke*)((rkJoint*)obj)->prp)->max[0]),
    zRad2Deg(((rkJointPrpHooke*)((rkJoint*)obj)->prp)->max[1]) );
}
static void _rkJointHookeStiffFPrint(FILE *fp, int i, void *obj){
  fprintf( fp, "%.10g %.10g\n",
    ((rkJointPrpHooke*)((rkJoint*)obj)->prp)->stiff[0],
    ((rkJointPrpHooke*)((rkJoint*)obj)->prp)->stiff[1] );
}
static void _rkJointHookeViscosFPrint(FILE *fp, int i, void *obj){
  fprintf( fp, "%.10g %.10g\n",
    ((rkJointPrpHooke*)((rkJoint*)obj)->prp)->viscos[0],
    ((rkJointPrpHooke*)((rkJoint*)obj)->prp)->viscos[1] );
}
static void _rkJointHookeCoulombFPrint(FILE *fp, int i, void *obj){
  fprintf( fp, "%.10g %.10g\n",
    ((rkJointPrpHooke*)((rkJoint*)obj)->prp)->coulomb[0],
    ((rkJointPrpHooke*)((rkJoint*)obj)->prp)->coulomb[1] );
}
static void _rkJointHookeStaticFrictionFPrint(FILE *fp, int i, void *obj){
  fprintf( fp, "%.10g %.10g\n",
    ((rkJointPrpHooke*)((rkJoint*)obj)->prp)->sf[0],
    ((rkJointPrpHooke*)((rkJoint*)obj)->prp)->sf[1] );
}
static void _rkJointHookeMotorFPrint(FILE *fp, int i, void *obj){
  fprintf( fp, "%s\n", zName(&((rkJointPrpHooke *)obj)->m) );
}

static ZTKPrp __ztk_prp_rkjoint_hooke[] = {
  { "dis", 1, _rkJointHookeDisFromZTK, _rkJointHookeDisFPrint },
  { "min", 1, _rkJointHookeMinFromZTK, _rkJointHookeMinFPrint },
  { "max", 1, _rkJointHookeMaxFromZTK, _rkJointHookeMaxFPrint },
  { "stiffness", 1, _rkJointHookeStiffFromZTK, _rkJointHookeStiffFPrint },
  { "viscosity", 1, _rkJointHookeViscosFromZTK, _rkJointHookeViscosFPrint },
  { "coulomb", 1, _rkJointHookeCoulombFromZTK, _rkJointHookeCoulombFPrint },
  { "staticfriction", 1, _rkJointHookeStaticFrictionFromZTK, _rkJointHookeStaticFrictionFPrint },
  { "motor", 1, _rkJointHookeMotorFromZTK, _rkJointHookeMotorFPrint },
};

bool rkJointHookeRegZTK(ZTK *ztk, char *tag)
{
  return ZTKDefRegPrp( ztk, tag, __ztk_prp_rkjoint_hooke ) ? true : false;
}

rkJoint *rkJointHookeFromZTK(rkJoint *joint, rkMotorArray *motorarray, ZTK *ztk)
{
  return rkJointPrpFromZTK( joint, motorarray, ztk, __ztk_prp_rkjoint_hooke );
}

void rkJointHookeFPrint(FILE *fp, rkJoint *joint)
{
  ZTKPrpKeyFPrint( fp, joint->prp, __ztk_prp_rkjoint_hooke );
}



/* limit joint displacement */
static void _rkJointLimDisSpher(void *prp, double *testval, double *limval){
  zVec3DCopy( (zVec3D*)testval, (zVec3D*)limval );
}
/* joint displacement set function */
static void _rkJointSetDisSpher(void *prp, double *val){
  _rkJointLimDisSpher( prp, val, ((rkJointPrpSpher*)prp)->aa.e );
  zMat3DFromAA( &((rkJointPrpSpher*)prp)->_att, &((rkJointPrpSpher*)prp)->aa );
}

static void *_rkJointSpherDisFromZTK(void *obj, int i, void *arg, ZTK *ztk){
  zVec3D aa;
  zVec3DFromZTK( &aa, ztk );
  _rkJointSetDisSpher( ((rkJoint*)obj)->prp, aa.e );
  return obj;
}
static void *_rkJointSpherMotorFromZTK(void *obj, int i, void *arg, ZTK *ztk){
  rkMotor *mp;
  if( !( mp = rkMotorArrayFind( arg, ZTKVal(ztk), (rkJoint*)obj ) ) ) return NULL;
  return rkMotorClone( mp, &((rkJointPrpSpher *)((rkJoint*)obj)->prp)->m ) ? obj : NULL;
}

static void _rkJointSpherDisFPrint(FILE *fp, int i, void *obj){
  zVec3DFPrint( fp, &((rkJointPrpSpher*)((rkJoint*)obj)->prp)->aa );
}
static void _rkJointSpherMotorFPrint(FILE *fp, int i, void *obj){
  fprintf( fp, "%s\n", zName(&((rkJointPrpSpher *)obj)->m) );
}

static ZTKPrp __ztk_prp_rkjoint_spher[] = {
  { "dis", 1, _rkJointSpherDisFromZTK, _rkJointSpherDisFPrint },
  { "motor", 1, _rkJointSpherMotorFromZTK, _rkJointSpherMotorFPrint },
};

bool rkJointSpherRegZTK(ZTK *ztk, char *tag)
{
  return ZTKDefRegPrp( ztk, tag, __ztk_prp_rkjoint_spher ) ? true : false;
}

rkJoint *rkJointSpherFromZTK(rkJoint *joint, rkMotorArray *motorarray, ZTK *ztk)
{
  return rkJointPrpFromZTK( joint, motorarray, ztk, __ztk_prp_rkjoint_spher );
}

void rkJointSpherFPrint(FILE *fp, rkJoint *joint)
{
  ZTKPrpKeyFPrint( fp, joint->prp, __ztk_prp_rkjoint_spher );
}



static void _rkJointSetDisFloat(void *prp, double *val){
  /* modified a function in rk_joint_float.c */
  /* to replace the original */
  zVec6DCopy( (zVec6D*)val, ((rkJointPrpFloat*)prp)->dis.e );
  zMat3DFromAA( &((rkJointPrpFloat*)prp)->_att, zVec6DAng(&((rkJointPrpFloat*)prp)->dis) );
}

static void *_rkJointFloatDisFromZTK(void *obj, int i, void *arg, ZTK *ztk){
  zVec6D dis;
  zVec6DFromZTK( &dis, ztk );
  _rkJointSetDisFloat( obj, dis.e );
  return obj;
}

static void _rkJointFloatDisFPrint(FILE *fp, int i, void *obj){
  zVec6DDataNLFPrint( fp, &((rkJointPrpFloat*)obj)->dis );
}

static ZTKPrp __ztk_prp_rkjoint_float[] = {
  { "dis", 1, _rkJointFloatDisFromZTK, _rkJointFloatDisFPrint },
};

bool rkJointFloatRegZTK(ZTK *ztk, char *tag)
{
  return ZTKDefRegPrp( ztk, tag, __ztk_prp_rkjoint_float ) ? true : false;
}

rkJoint *rkJointFloatFromZTK(rkJoint *joint, rkMotorArray *motorarray, ZTK *ztk)
{
  return rkJointPrpFromZTK( joint, motorarray, ztk, __ztk_prp_rkjoint_float );
}

void rkJointFloatFPrint(FILE *fp, rkJoint *joint)
{
  ZTKPrpKeyFPrint( fp, joint->prp, __ztk_prp_rkjoint_float );
}



static void _rkJointSetDisBrFloat(void *prp, double *val){
  /* modified a function in rk_joint_float.c */
  /* to replace the original */
  zVec6DCopy( (zVec6D*)val, ((rkJointPrpFloat*)prp)->dis.e );
  zMat3DFromAA( &((rkJointPrpFloat*)prp)->_att, zVec6DAng(&((rkJointPrpFloat*)prp)->dis) );
}

static void *_rkJointBrFloatDisFromZTK(void *obj, int i, void *arg, ZTK *ztk){
  zVec6D dis;
  zVec6DFromZTK( &dis, ztk );
  _rkJointSetDisBrFloat( (rkJointPrpBrFloat *)obj, dis.e );
  return obj;
}
static void *_rkJointBrFloatForceThFromZTK(void *obj, int i, void *arg, ZTK *ztk){
  ((rkJointPrpBrFloat *)obj)->ep_f = ZTKDouble(ztk);
  return obj;
}
static void *_rkJointBrFloatTorqueThFromZTK(void *obj, int i, void *arg, ZTK *ztk){
  ((rkJointPrpBrFloat *)obj)->ep_t = ZTKDouble(ztk);
  return obj;
}

static void _rkJointBrFloatDisFPrint(FILE *fp, int i, void *obj){
  zVec6DDataNLFPrint( fp, &((rkJointPrpFloat*)obj)->dis );
}
static void _rkJointBrFloatForceThFPrint(FILE *fp, int i, void *obj){
  fprintf( fp, "%.10g\n", ((rkJointPrpBrFloat *)obj)->ep_f );
}
static void _rkJointBrFloatTorqueThFPrint(FILE *fp, int i, void *obj){
  fprintf( fp, "%.10g\n", ((rkJointPrpBrFloat *)obj)->ep_t );
}

static ZTKPrp __ztk_prp_rkjoint_brfloat[] = {
  { "dis", 1, _rkJointBrFloatDisFromZTK, _rkJointBrFloatDisFPrint },
  { "forcethreshold", 1, _rkJointBrFloatForceThFromZTK, _rkJointBrFloatForceThFPrint },
  { "torquethreshold", 1, _rkJointBrFloatTorqueThFromZTK, _rkJointBrFloatTorqueThFPrint },
};

bool rkJointBrFloatRegZTK(ZTK *ztk, char *tag)
{
  return ZTKDefRegPrp( ztk, tag, __ztk_prp_rkjoint_brfloat ) ? true : false;
}

rkJoint *rkJointBrFloatFromZTK(rkJoint *joint, rkMotorArray *motorarray, ZTK *ztk)
{
  return rkJointPrpFromZTK( joint, motorarray, ztk, __ztk_prp_rkjoint_brfloat );
}

void rkJointBrFloatFPrint(FILE *fp, rkJoint *joint)
{
  ZTKPrpKeyFPrint( fp, joint->prp, __ztk_prp_rkjoint_brfloat );
}



rkJoint *rkJointFromZTK(rkJoint *joint, rkMotorArray *motorarray, ZTK *ztk)
{
  /* obviously, the following is a volatile implementation. */
  switch( rkJointType(joint) ){
  case RK_JOINT_FIXED: rkJointFixedFromZTK( joint, motorarray, ztk ); break;
  case RK_JOINT_REVOL: rkJointRevolFromZTK( joint, motorarray, ztk ); break;
  case RK_JOINT_PRISM: rkJointPrismFromZTK( joint, motorarray, ztk ); break;
  case RK_JOINT_CYLIN: rkJointCylinFromZTK( joint, motorarray, ztk ); break;
  case RK_JOINT_HOOKE: rkJointHookeFromZTK( joint, motorarray, ztk ); break;
  case RK_JOINT_SPHER: rkJointSpherFromZTK( joint, motorarray, ztk ); break;
  case RK_JOINT_FLOAT: rkJointFloatFromZTK( joint, motorarray, ztk ); break;
  case RK_JOINT_BRFLOAT: rkJointBrFloatFromZTK( joint, motorarray, ztk ); break;
  default: ;
  }
  return joint;
}

void _rkJointFPrint(FILE *fp, rkJoint *joint)
{
  /* obviously, the following is a volatile implementation. */
  switch( rkJointType(joint) ){
  case RK_JOINT_FIXED: rkJointFixedFPrint( fp, joint ); break;
  case RK_JOINT_REVOL: rkJointRevolFPrint( fp, joint ); break;
  case RK_JOINT_PRISM: rkJointPrismFPrint( fp, joint ); break;
  case RK_JOINT_CYLIN: rkJointCylinFPrint( fp, joint ); break;
  case RK_JOINT_HOOKE: rkJointHookeFPrint( fp, joint ); break;
  case RK_JOINT_SPHER: rkJointSpherFPrint( fp, joint ); break;
  case RK_JOINT_FLOAT: rkJointFloatFPrint( fp, joint ); break;
  case RK_JOINT_BRFLOAT: rkJointBrFloatFPrint( fp, joint ); break;
  default: ;
  }
}



#define ZTK_TAG_RKLINK "link"

typedef struct{
  rkLink *linkbuf;
  int nl;
  zShape3D *shapebuf;
  int ns;
} _rkLinkRefPrp;

static void *_rkLinkNameFromZTK(void *obj, int i, void *arg, ZTK *ztk){
  rkLink *link;
  zNameFind( ((_rkLinkRefPrp*)arg)->linkbuf, ((_rkLinkRefPrp*)arg)->nl, ZTKVal(ztk), link );
  if( link ){
    ZRUNWARN( RK_WARN_LINK_DUP, ZTKVal(ztk) );
    return NULL;
  }
  zNameSet( (rkLink*)obj, ZTKVal(ztk) );
  return zNamePtr((rkLink*)obj) ? obj : NULL;
}
static void *_rkLinkJointTypeFromZTK(void *obj, int i, void *arg, ZTK *ztk){
  return rkJointCreate( rkLinkJoint((rkLink*)obj), rkJointTypeFromStr(ZTKVal(ztk)) ) ? obj : NULL;
}
static void *_rkLinkMassFromZTK(void *obj, int i, void *arg, ZTK *ztk){
  rkLinkSetMass( (rkLink*)obj, ZTKDouble(ztk) );
  return obj;
}
static void *_rkLinkStuffFromZTK(void *obj, int i, void *arg, ZTK *ztk){
  return rkLinkSetStuff( (rkLink*)obj, ZTKVal(ztk) ) ? obj : NULL;
}
static void *_rkLinkCOMFromZTK(void *obj, int i, void *arg, ZTK *ztk){
  zVec3DFromZTK( rkLinkCOM((rkLink*)obj), ztk );
  return obj;
}
static void *_rkLinkInertiaFromZTK(void *obj, int i, void *arg, ZTK *ztk){
  zMat3DFromZTK( rkLinkInertia((rkLink*)obj), ztk );
  return obj;
}
static void *_rkLinkPosFromZTK(void *obj, int i, void *arg, ZTK *ztk){
  zVec3DFromZTK( rkLinkOrgPos((rkLink*)obj), ztk );
  return obj;
}
static void *_rkLinkAttFromZTK(void *obj, int i, void *arg, ZTK *ztk){
  zMat3DFromZTK( rkLinkOrgAtt((rkLink*)obj), ztk );
  return obj;
}
static void *_rkLinkRotFromZTK(void *obj, int i, void *arg, ZTK *ztk){
  zVec3D axis;
  double angle;
  zVec3DFromZTK( &axis, ztk );
  angle = zDeg2Rad( ZTKDouble( ztk ) );
  if( zVec3DNormalizeDRC( &axis ) > 0 ){
    zVec3DMulDRC( &axis, angle );
    zMat3DRot( rkLinkOrgAtt((rkLink*)obj), &axis, rkLinkOrgAtt((rkLink*)obj) );
  }
  return obj;
}
static void *_rkLinkFrameFromZTK(void *obj, int i, void *arg, ZTK *ztk){
  zFrame3DFromZTK( rkLinkOrgFrame((rkLink*)obj), ztk );
  return obj;
}
static void *_rkLinkDHFromZTK(void *obj, int i, void *arg, ZTK *ztk){
  zFrame3DDHFromZTK( rkLinkOrgFrame((rkLink*)obj), ztk );
  return obj;
}
static void *_rkLinkParentFromZTK(void *obj, int i, void *arg, ZTK *ztk){
  rkLink *parent;
  zNameFind( ((_rkLinkRefPrp*)arg)->linkbuf, ((_rkLinkRefPrp*)arg)->nl, ZTKVal(ztk), parent );
  if( !parent ){
    ZRUNERROR( RK_ERR_LINK_UNKNOWN, ZTKVal(ztk) );
    return NULL;
  }
  rkLinkAddChild( parent, (rkLink*)obj );
  return obj;
}
static void *_rkLinkShapeFromZTK(void *obj, int i, void *arg, ZTK *ztk){
  zShape3D *sp;
  zNameFind( ((_rkLinkRefPrp*)arg)->shapebuf, ((_rkLinkRefPrp*)arg)->ns, ZTKVal(ztk), sp );
  if( !sp ){
    ZRUNERROR( RK_ERR_SHAPE_UNKNOWN, ZTKVal(ztk) );
    return NULL;
  }
  return rkLinkShapePush( (rkLink*)obj, sp ) ? obj : NULL;
}

static void _rkLinkNameFPrint(FILE *fp, int i, void *obj){
  fprintf( fp, "%s\n", zName((rkLink*)obj) );
}
static void _rkLinkJointTypeFPrint(FILE *fp, int i, void *obj){
  fprintf( fp, "%s\n", rkJointTypeExpr(rkLinkJointType((rkLink*)obj)) );
}
static void _rkLinkMassFPrint(FILE *fp, int i, void *obj){
  fprintf( fp, "%.10g\n", rkLinkMass((rkLink*)obj) );
}
static void _rkLinkStuffFPrint(FILE *fp, int i, void *obj){
  fprintf( fp, "%s\n", rkLinkStuff((rkLink*)obj) );
}
static void _rkLinkCOMFPrint(FILE *fp, int i, void *obj){
  zVec3DFPrint( fp, rkLinkCOM((rkLink*)obj) );
}
static void _rkLinkInertiaFPrint(FILE *fp, int i, void *obj){
  zMat3DFPrint( fp, rkLinkInertia((rkLink*)obj) );
}
static void _rkLinkPosFPrint(FILE *fp, int i, void *obj){
  zVec3DFPrint( fp, rkLinkOrgPos((rkLink*)obj) );
}
static void _rkLinkAttFPrint(FILE *fp, int i, void *obj){
  zMat3DFPrint( fp, rkLinkOrgAtt((rkLink*)obj) );
}

static ZTKPrp __ztk_prp_rklink[] = {
  { "name", 1, _rkLinkNameFromZTK, _rkLinkNameFPrint },
  { "jointtype", 1, _rkLinkJointTypeFromZTK, _rkLinkJointTypeFPrint },
  { "mass", 1, _rkLinkMassFromZTK, _rkLinkMassFPrint },
  { "stuff", 1, _rkLinkStuffFromZTK, _rkLinkStuffFPrint },
  { "COM", 1, _rkLinkCOMFromZTK, _rkLinkCOMFPrint },
  { "inertia", 1, _rkLinkInertiaFromZTK, _rkLinkInertiaFPrint },
  { "pos", 1, _rkLinkPosFromZTK, _rkLinkPosFPrint },
  { "att", 1, _rkLinkAttFromZTK, _rkLinkAttFPrint },
  { "rot", -1, _rkLinkRotFromZTK, NULL },
  { "frame", 1, _rkLinkFrameFromZTK, NULL },
  { "DH", 1, _rkLinkDHFromZTK, NULL },
  { "parent", 1, _rkLinkParentFromZTK, NULL },
  { "shape", -1, _rkLinkShapeFromZTK, NULL },
};

bool rkLinkRegZTK(ZTK *ztk)
{
  return rkJointRevolRegZTK( ztk, ZTK_TAG_RKLINK ) &&
         rkJointPrismRegZTK( ztk, ZTK_TAG_RKLINK ) &&
         rkJointCylinRegZTK( ztk, ZTK_TAG_RKLINK ) &&
         rkJointHookeRegZTK( ztk, ZTK_TAG_RKLINK ) &&
         rkJointSpherRegZTK( ztk, ZTK_TAG_RKLINK ) &&
         rkJointFloatRegZTK( ztk, ZTK_TAG_RKLINK ) &&
         rkJointBrFloatRegZTK( ztk, ZTK_TAG_RKLINK ) &&
         ZTKDefRegPrp( ztk, ZTK_TAG_RKLINK, __ztk_prp_rklink ) ? true : false;
}

rkLink *rkLinkFromZTK(rkLink *link, rkLink *linkbuf, int nl, zShape3D *shapebuf, int ns, rkMotorArray *motorarray, ZTK *ztk)
{
  _rkLinkRefPrp prp;

  rkLinkInit( link );
  prp.linkbuf = linkbuf;
  prp.nl = nl;
  prp.shapebuf = shapebuf;
  prp.ns = ns;
  if( !ZTKEncodeKey( link, &prp, ztk, __ztk_prp_rklink ) ) return NULL;
  rkJointFromZTK( rkLinkJoint(link), motorarray, ztk );
  return link;
}

void rkLinkFPrint(FILE *fp, rkLink *link)
{
  zShapeListCell *cp;

  ZTKPrpKeyFPrint( fp, link, __ztk_prp_rklink );
  _rkJointFPrint( fp, rkLinkJoint(link) );
  if( !rkLinkShapeIsEmpty(link) )
    zListForEach( rkLinkShapeList(link), cp )
      fprintf( fp, "shape: %s\n", zName( zShapeListCellShape(cp) ) );
  if( rkLinkParent(link) )
    fprintf( fp, "parent: %s\n", zName(rkLinkParent(link)) );
  fprintf( fp, "\n" );
}




#define ZTK_TAG_RKCHAIN "chain"

static void *_rkChainNameFromZTK(void *obj, int i, void *arg, ZTK *ztk){
  return zNameSet( (rkChain*)obj, ZTKVal(ztk) ) ? obj : NULL;
}

static void _rkChainNameFPrint(FILE *fp, int i, void *obj){
  fprintf( fp, "%s\n", zName((rkChain*)obj) );
}

static ZTKPrp __ztk_prp_rkchain_chain[] = {
  { "name", 1, _rkChainNameFromZTK, _rkChainNameFPrint },
};


static void *_rkChainChainFromZTK(void *obj, int i, void *arg, ZTK *ztk){
  return ZTKEncodeKey( obj, arg, ztk, __ztk_prp_rkchain_chain );
}
static void *_rkChainMotorFromZTK(void *obj, int i, void *arg, ZTK *ztk){
  return rkMotorFromZTK( zArrayElemNC(rkChainMotor((rkChain*)obj),i), ztk );
}
static void *_rkChainLinkFromZTK(void *obj, int i, void *arg, ZTK *ztk){
  zShape3D *shapearray = NULL;
  int ns = 0;
  if( rkChainShape((rkChain*)obj) ){
    shapearray = zMShape3DShapeBuf(rkChainShape((rkChain*)obj));
    ns = zMShape3DShapeNum(rkChainShape((rkChain*)obj));
  }
  return rkLinkFromZTK( rkChainLink((rkChain*)obj,i),
    rkChainRoot((rkChain*)obj), rkChainNum((rkChain*)obj),
    shapearray, ns, rkChainMotor((rkChain*)obj), ztk ) ? obj : NULL;
}

static void _rkChainChainFPrint(FILE *fp, int i, void *obj){
  ZTKPrpKeyFPrint( fp, obj, __ztk_prp_rkchain_chain );
}

static ZTKPrp __ztk_prp_rkchain[] = {
  { ZTK_TAG_RKCHAIN, 1, _rkChainChainFromZTK, _rkChainChainFPrint },
  { ZTK_TAG_OPTIC, -1, NULL, NULL },
  { ZTK_TAG_SHAPE, -1, NULL, NULL },
  { ZTK_TAG_RKMOTOR, -1, _rkChainMotorFromZTK, NULL },
  { ZTK_TAG_RKLINK, -1, _rkChainLinkFromZTK, NULL },
};

bool rkChainRegZTK(ZTK *ztk)
{
  return ZTKDefRegPrp( ztk, ZTK_TAG_RKCHAIN, __ztk_prp_rkchain_chain ) &&
         zMShape3DRegZTK( ztk ) &&
         rkMotorRegZTK( ztk ) &&
         rkLinkRegZTK( ztk ) ? true : false;
}

rkChain *rkChainFromZTK(rkChain *chain, ZTK *ztk)
{
  int num_motor, num_link;

  if( ( num_motor = ZTKCountTag( ztk, ZTK_TAG_RKMOTOR ) ) > 0 ){
    if( !( rkChainMotor(chain) = zAlloc( rkMotorArray, 1 ) ) ){
      ZALLOCERROR();
      return NULL;
    }
    zArrayAlloc( rkChainMotor(chain), rkMotor, num_motor );
    if( zArraySize(rkChainMotor(chain)) != num_motor ) return NULL;
  }
  num_link = ZTKCountTag( ztk, ZTK_TAG_RKLINK );
  zArrayAlloc( &chain->link, rkLink, num_link );
  if( rkChainNum(chain) != num_link ) return NULL;
  if( rkChainNum(chain) == 0 ){
    ZRUNWARN( RK_WARN_CHAIN_EMPTY );
    return NULL;
  }
  ZTKEncodeTag( chain, NULL, ztk, __ztk_prp_rkchain );
  return chain;
}

rkChain *rkChainReadZTK(rkChain *chain, char filename[])
{
  ZTK ztk;

  ZTKInit( &ztk );
  rkChainRegZTK( &ztk );
  ZTKParse( &ztk, filename );
  /* read shapes */
  rkChainInit( chain );
  if( ZTKCountTag( &ztk, ZTK_TAG_SHAPE ) > 0 ){
    if( !( rkChainShape(chain) = zAlloc( zMShape3D, 1 ) ) ){
      ZALLOCERROR();
      return NULL;
    }
    if( !zMShape3DFromZTK( rkChainShape(chain), &ztk ) ) return NULL;
  }
  /* read motors and links */
  chain = rkChainFromZTK( chain, &ztk );
  ZTKDestroy( &ztk );
  return chain;
}

void _rkChainFPrint(FILE *fp, rkChain *chain)
{
  register int i;

  ZTKPrpTagFPrint( fp, chain, __ztk_prp_rkchain );
  fprintf( fp, "\n" );
  if( rkChainShape(chain) )
    zMShape3DFPrint( fp, rkChainShape(chain) );
  if( rkChainMotor(chain) )
    rkMotorArrayFPrint( fp, rkChainMotor(chain) ); /* this is not good. rkMotorArray class should be redesigned. */
  for( i=0; i<rkChainNum(chain); i++ ){
    fprintf( fp, "[%s]\n", ZTK_TAG_RKLINK );
    rkLinkFPrint( fp, rkChainLink(chain,i) );
  }
}

rkChain *rkChainReadInitZTK(rkChain *chain, char filename[])
{
  /* dummy */
  return chain;
}

int main(int argc, char *argv[])
{
  rkChain chain;

  if( argc <= 1 ) return 0;
  rkChainReadZTK( &chain, argv[1] );
  rkChainReadInitZTK( &chain, argv[1] );
  _rkChainFPrint( stdout, &chain );
  rkChainDestroy( &chain );
  return 0;
}
