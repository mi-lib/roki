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

zVec6D *invpend_vel(rkChain *ip, double q1, double q2, double dq1, double dq2, zVec6D *v)
{
  double s1, c1, s2, c2;

  zSinCos( q1, &s1, &c1 );
  zSinCos( q2, &s2, &c2 );
  return zVec6DCreate( v,
   -L1*dq1*s1+L2*(dq2*s1*c2+dq1*c1*s2),
    L1*dq1*c1-L2*(dq2*c1*c2-dq1*s1*s2),
             -L2* dq2   *s2,
    dq2*c1, dq2*s1, dq1 );
}

zVec6D *invpend_acc(rkChain *ip, double q1, double q2, double dq1, double dq2, double ddq1, double ddq2, zVec6D *a)
{
  double s1, c1, s2, c2;

  zSinCos( q1, &s1, &c1 );
  zSinCos( q2, &s2, &c2 );
  return zVec6DCreate( a,
    (-L1*s1+L2*c1*s2)*ddq1+L2*s1*c2*ddq2+(-L1*c1-L2*s1*s2)*dq1*dq1+2*L2*c1*c2*dq1*dq2-L2*s1*s2*dq2*dq2,
    ( L1*c1+L2*s1*s2)*ddq1-L2*c1*c2*ddq2+(-L1*s1+L2*c1*s2)*dq1*dq1+2*L2*s1*c2*dq1*dq2+L2*c1*s2*dq2*dq2,
     RK_G - L2*s2*ddq2 - L2*c2*dq2*dq2,
    c1*ddq2-s1*dq1*dq2, s1*ddq2+c1*dq1*dq2, ddq1 );
}

zVec3D *invpend_am(rkChain *ip, double q1, double q2, double dq1, double dq2, zVec3D *am)
{
  double s1, c1, s2, c2;

  zSinCos( q1, &s1, &c1 );
  zSinCos( q2, &s2, &c2 );
  return zVec3DCreate( am,
   -(M2*(H*(L1*c1+L2*s1*s2)+L1*L2*c1*c2+L2*L2*s1*s2*c2)+I2*s1*s2*c2)*dq1 + (M2*(H*L2*c1*c2-L1*L2*s1*s2+L2*L2*c1)+I2*c1)*dq2,
    (M2*(H*(-L1*s1+L2*c1*s2)-L1*L2*s1*c2+L2*L2*c1*s2*c2)+I2*c1*s2*c2)*dq1 + (M2*(H*L2*s1*c2+L1*L2*c1*s2+L2*L2*s1)+I2*s1)*dq2,
    (I1+M2*(L1*L1+L2*L2*s2*s2)+I2*s2*s2)*dq1 - M2*L1*L2*c2*dq2 );
}

double invpend_ke(rkChain *ip, double q1, double q2, double dq1, double dq2)
{
  double s1, c1, s2, c2;

  zSinCos( q1, &s1, &c1 );
  zSinCos( q2, &s2, &c2 );
  return 0.5*(I1+M2*L1*L1+(I2+M2*L2*L2)*s2*s2)*dq1*dq1
               - M2*L1*L2*c2*dq1*dq2
       + 0.5*(I2+M2*L2*L2)*dq2*dq2;
}

void invpend_trq(rkChain *ip, double q1, double q2, double dq1, double dq2, double ddq1, double ddq2, double *u1, double *u2)
{
  double s1, c1, s2, c2;

  zSinCos( q1, &s1, &c1 );
  zSinCos( q2, &s2, &c2 );
  *u1 = (I1+M2*L1*L1+(M2*L2*L2+I2)*s2*s2)*ddq1 - M2*L1*L2*c2*ddq2 + 2*(M2*L2*L2+I2)*s2*c2*dq1*dq2 + M2*L1*L2*s2*dq2*dq2;
  *u2 = -M2*L1*L2*c2*ddq1 + (M2*L2*L2+I2)*ddq2 - (M2*L2*L2+I2)*s2*c2*dq1*dq1 - M2*RK_G*L2*s2;
}

void invpend_set_joint_state(rkChain *ip, zVec q, zVec dq, zVec ddq)
{
  zVecSetElem( q, 0, zRandF(-zPI,zPI) );
  zVecSetElem( q, 1, zRandF(-zPI,zPI) );
  zVecSetElem( dq, 0, zRandF(-zPI,zPI) * 0.1 / DT );
  zVecSetElem( dq, 1, zRandF(-zPI,zPI) * 0.1 / DT );
  zVecSetElem( ddq, 0, zRandF(-zPI,zPI) * zSqr(0.1/DT) );
  zVecSetElem( ddq, 1, zRandF(-zPI,zPI) * zSqr(0.1/DT) );
  rkChainFK( ip, q );
  rkChainID( ip, dq, ddq );
}

void assert_invpend(rkChain *invpend, zVec q, zVec dq, zVec ddq)
{
  zVec6D v1, v2, a1, a2, ve6, ae6;
  zVec3D am1, am2, e3;
  double k1, k2, u11, u12, u21, u22;

  invpend_set_joint_state( invpend, q, dq, ddq );

  /* *** velocity test *** */
  /* analytical computation */
  invpend_vel( invpend, zVecElem(q,0), zVecElem(q,1), zVecElem(dq,0), zVecElem(dq,1), &v1 );
  /* - recursive computation */
  zMulMat3DVec3D( rkChainLinkWldAtt(invpend,2), rkChainLinkCOMVel(invpend,2), zVec6DLin(&v2) );
  zMulMat3DVec3D( rkChainLinkWldAtt(invpend,2), rkChainLinkAngVel(invpend,2), zVec6DAng(&v2) );
  zAssert( velocity test, zVec6DIsTiny( zVec6DSub( &v1, &v2, &ve6 ) ) );

  /* *** acceleration test *** */
  /* analytical computation */
  invpend_acc( invpend, zVecElem(q,0), zVecElem(q,1), zVecElem(dq,0), zVecElem(dq,1), zVecElem(ddq,0), zVecElem(ddq,1), &a1 );
  /* recursive computation */
  zMulMat3DVec3D( rkChainLinkWldAtt(invpend,2), rkChainLinkCOMAcc(invpend,2), zVec6DLin(&a2) );
  zMulMat3DVec3D( rkChainLinkWldAtt(invpend,2), rkChainLinkAngAcc(invpend,2), zVec6DAng(&a2) );
  zAssert( acceleration test, zVec6DIsTol( zVec6DSub( &a1, &a2, &ae6 ), zTOL*15 ) );

  /* *** angular momentum test *** */
  /* analytical computation */
  invpend_am( invpend, zVecElem(q,0), zVecElem(q,1), zVecElem(dq,0), zVecElem(dq,1), &am1 );
  /* recursive computation */
  rkChainAM( invpend, ZVEC3DZERO, &am2 );
  zAssert( angular momentum, zVec3DIsTiny( zVec3DSub( &am1, &am2, &e3 ) ) );

  /* *** kinematic energy test *** */
  /* analytical computation */
  k1 = invpend_ke( invpend, zVecElem(q,0), zVecElem(q,1), zVecElem(dq,0), zVecElem(dq,1) );
  /* recursive computation */
  k2 = rkChainKE( invpend );
  zAssert( kinematic energy, zIsTiny( k1 - k2 ) );

  /* *** torque test *** */
  /* analytical computation */
  invpend_trq( invpend, zVecElem(q,0), zVecElem(q,1), zVecElem(dq,0), zVecElem(dq,1), zVecElem(ddq,0), zVecElem(ddq,1), &u11, &u12 );
  /* recursive computation */
  rkChainLinkGetJointTrq( invpend, 1, &u21 );
  rkChainLinkGetJointTrq( invpend, 2, &u22 );
  zAssert( joint torque, zIsTiny( u11 - u21 ) && zIsTiny( u12 - u22 ) );
}

int main(void)
{
  rkChain invpend;
  zVec q, dq, ddq;

  zRandInit();
  rkChainReadZTK( &invpend, "invpend.ztk" );
  q   = zVecAlloc( rkChainJointSize(&invpend) );
  dq  = zVecAlloc( rkChainJointSize(&invpend) );
  ddq = zVecAlloc( rkChainJointSize(&invpend) );

  assert_invpend( &invpend, q, dq, ddq );

  rkChainDestroy( &invpend );
  zVecFreeAO( 3, q, dq, ddq );
  return 0;
}
