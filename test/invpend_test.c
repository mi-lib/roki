/* this code is to check if the inverse dynamics, angular momentum and
   kinematic energy computation are correct, comparing with the
   analytical solutions on a cylindrical inverted pendulum.
 */
#include <roki/roki.h>

/* the following parameters must be consistent with invpend.zkc */
#define H  0.3
#define L1 0.49
#define L2 0.195

#define I1 0.075
#define I2 0.0011403
#define M2 0.085

#define DT 0.001

zVec6D *invpend_vel(rkChain *ip, double t1, double t2, double dt1, double dt2, zVec6D *v)
{
  double s1, c1, s2, c2;

  zSinCos( t1, &s1, &c1 );
  zSinCos( t2, &s2, &c2 );
  return zVec6DCreate( v,
   -L1*dt1*s1+L2*(dt2*s1*c2+dt1*c1*s2),
    L1*dt1*c1-L2*(dt2*c1*c2-dt1*s1*s2),
             -L2* dt2   *s2,
    dt2*c1, dt2*s1, dt1 );
}

zVec6D *invpend_acc(rkChain *ip, double t1, double t2, double dt1, double dt2, double ddt1, double ddt2, zVec6D *a)
{
  double s1, c1, s2, c2;

  zSinCos( t1, &s1, &c1 );
  zSinCos( t2, &s2, &c2 );
  return zVec6DCreate( a,
    (-L1*s1+L2*c1*s2)*ddt1+L2*s1*c2*ddt2+(-L1*c1-L2*s1*s2)*dt1*dt1+2*L2*c1*c2*dt1*dt2-L2*s1*s2*dt2*dt2,
    ( L1*c1+L2*s1*s2)*ddt1-L2*c1*c2*ddt2+(-L1*s1+L2*c1*s2)*dt1*dt1+2*L2*s1*c2*dt1*dt2+L2*c1*s2*dt2*dt2,
     RK_G - L2*s2*ddt2 - L2*c2*dt2*dt2,
    c1*ddt2-s1*dt1*dt2, s1*ddt2+c1*dt1*dt2, ddt1 );
}

zVec3D *invpend_am(rkChain *ip, double t1, double t2, double dt1, double dt2, zVec3D *am)
{
  double s1, c1, s2, c2;

  zSinCos( t1, &s1, &c1 );
  zSinCos( t2, &s2, &c2 );
  return zVec3DCreate( am,
   -(M2*(H*(L1*c1+L2*s1*s2)+L1*L2*c1*c2+L2*L2*s1*s2*c2)+I2*s1*s2*c2)*dt1 + (M2*(H*L2*c1*c2-L1*L2*s1*s2+L2*L2*c1)+I2*c1)*dt2,
    (M2*(H*(-L1*s1+L2*c1*s2)-L1*L2*s1*c2+L2*L2*c1*s2*c2)+I2*c1*s2*c2)*dt1 + (M2*(H*L2*s1*c2+L1*L2*c1*s2+L2*L2*s1)+I2*s1)*dt2,
    (I1+M2*(L1*L1+L2*L2*s2*s2)+I2*s2*s2)*dt1 - M2*L1*L2*c2*dt2 );
}

double invpend_ke(rkChain *ip, double t1, double t2, double dt1, double dt2)
{
  double s1, c1, s2, c2;

  zSinCos( t1, &s1, &c1 );
  zSinCos( t2, &s2, &c2 );
  return 0.5*(I1+M2*L1*L1+(I2+M2*L2*L2)*s2*s2)*dt1*dt1
               - M2*L1*L2*c2*dt1*dt2
       + 0.5*(I2+M2*L2*L2)*dt2*dt2;
}

void invpend_trq(rkChain *ip, double t1, double t2, double dt1, double dt2, double ddt1, double ddt2, double *u1, double *u2)
{
  double s1, c1, s2, c2;

  zSinCos( t1, &s1, &c1 );
  zSinCos( t2, &s2, &c2 );
  *u1 = (I1+M2*L1*L1+(M2*L2*L2+I2)*s2*s2)*ddt1 - M2*L1*L2*c2*ddt2 + 2*(M2*L2*L2+I2)*s2*c2*dt1*dt2 + M2*L1*L2*s2*dt2*dt2;
  *u2 = -M2*L1*L2*c2*ddt1 + (M2*L2*L2+I2)*ddt2 - (M2*L2*L2+I2)*s2*c2*dt1*dt1 - M2*RK_G*L2*s2;
}

void invpend_set_joint_state(rkChain *ip, zVec t, zVec dt, zVec ddt)
{
  zVecSetElem( t, 0, zRandF(-zPI,zPI) );
  zVecSetElem( t, 1, zRandF(-zPI,zPI) );
  zVecSetElem( dt, 0, zRandF(-zPI,zPI) * 0.1 / DT );
  zVecSetElem( dt, 1, zRandF(-zPI,zPI) * 0.1 / DT );
  zVecSetElem( ddt, 0, zRandF(-zPI,zPI) * zSqr(0.1/DT) );
  zVecSetElem( ddt, 1, zRandF(-zPI,zPI) * zSqr(0.1/DT) );
  rkChainFK( ip, t );
  rkChainID( ip, dt, ddt );
}

void assert_invpend(rkChain *invpend, zVec t, zVec dt, zVec ddt)
{
  zVec6D v1, v2, a1, a2, ve6, ae6;
  zVec3D am1, am2, e3;
  double k1, k2, u11, u12, u21, u22;

  invpend_set_joint_state( invpend, t, dt, ddt );

  /* *** velocity test *** */
  /* analytical computation */
  invpend_vel( invpend, zVecElem(t,0), zVecElem(t,1), zVecElem(dt,0), zVecElem(dt,1), &v1 );
  /* - recursive computation */
  zMulMatVec3D( rkChainLinkWldAtt(invpend,2), rkChainLinkCOMVel(invpend,2), zVec6DLin(&v2) );
  zMulMatVec3D( rkChainLinkWldAtt(invpend,2), rkChainLinkAngVel(invpend,2), zVec6DAng(&v2) );
  zAssert( velocity test, zVec6DIsTiny( zVec6DSub( &v1, &v2, &ve6 ) ) );

  /* *** acceleration test *** */
  /* analytical computation */
  invpend_acc( invpend, zVecElem(t,0), zVecElem(t,1), zVecElem(dt,0), zVecElem(dt,1), zVecElem(ddt,0), zVecElem(ddt,1), &a1 );
  /* recursive computation */
  zMulMatVec3D( rkChainLinkWldAtt(invpend,2), rkChainLinkCOMAcc(invpend,2), zVec6DLin(&a2) );
  zMulMatVec3D( rkChainLinkWldAtt(invpend,2), rkChainLinkAngAcc(invpend,2), zVec6DAng(&a2) );
  zAssert( acceleration test, zVec6DIsTol( zVec6DSub( &a1, &a2, &ae6 ), zTOL*15 ) );

  /* *** angular momentum test *** */
  /* analytical computation */
  invpend_am( invpend, zVecElem(t,0), zVecElem(t,1), zVecElem(dt,0), zVecElem(dt,1), &am1 );
  /* recursive computation */
  rkChainAM( invpend, ZVEC3DZERO, &am2 );
  zAssert( angular momentum, zVec3DIsTiny( zVec3DSub( &am1, &am2, &e3 ) ) );

  /* *** kinematic energy test *** */
  /* analytical computation */
  k1 = invpend_ke( invpend, zVecElem(t,0), zVecElem(t,1), zVecElem(dt,0), zVecElem(dt,1) );
  /* recursive computation */
  k2 = rkChainKE( invpend );
  zAssert( kinematic energy, zIsTiny( k1 - k2 ) );

  /* *** torque test *** */
  /* analytical computation */
  invpend_trq( invpend, zVecElem(t,0), zVecElem(t,1), zVecElem(dt,0), zVecElem(dt,1), zVecElem(ddt,0), zVecElem(ddt,1), &u11, &u12 );
  /* recursive computation */
  rkChainLinkGetJointTrq( invpend, 1, &u21 );
  rkChainLinkGetJointTrq( invpend, 2, &u22 );
  zAssert( joint torque, zIsTiny( u11 - u21 ) && zIsTiny( u12 - u22 ) );
}

int main(void)
{
  rkChain invpend;
  zVec t, dt, ddt;

  zRandInit();
  rkChainReadFile( &invpend, "invpend.zkc" );
  t   = zVecAlloc( rkChainJointSize(&invpend) );
  dt  = zVecAlloc( rkChainJointSize(&invpend) );
  ddt = zVecAlloc( rkChainJointSize(&invpend) );

  assert_invpend( &invpend, t, dt, ddt );

  rkChainDestroy( &invpend );
  zVecFreeAO( 3, t, dt, ddt );
  return 0;
}
