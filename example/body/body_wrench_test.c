#include <roki/rk_body.h>

void init_mp(rkBody *body)
{
  rkBodyInit( body );
  /* 0.1x0.2x0.4[m^3] recutangular solid block
     with 1000[kg/m^3] density */
  rkBodySetMass( body, 0.1*0.2*0.4*1000 );
  zMat3DCreate( rkBodyInertia(body),
    (zSqr(0.2)+zSqr(0.4))*rkBodyMass(body)/12, 0, 0,
    0, (zSqr(0.4)+zSqr(0.1))*rkBodyMass(body)/12, 0,
    0, 0, (zSqr(0.1)+zSqr(0.2))*rkBodyMass(body)/12 );
  zVec3DCreate( rkBodyCOM(body), 0.05, 0.1, 0.2 );
}

void output(rkBody *body, zVec6D *f)
{
  zVec3DDataWrite( rkBodyCOMAcc(body) );
  zVec3DDataWrite( zVec6DLin(f) );
  zVec3DDataWrite( rkBodyAngAcc(body) );
  zVec3DDataWrite( zVec6DAng(f) );
}

#define DIV 1000
#define T   10.0

void pattern1(rkBody *body, double t)
{
  /* p=0.5at^2, w=0 */
  double x, y, z;
  double vx, vy, vz;
  double ax, ay, az;

  ax = 1; vx = ax * t; x = 0.5*ax*t*t;
  ay = 2; vy = ay * t; y = 0.5*ay*t*t;
  az = 3; vz = az * t; z = 0.5*az*t*t;
  zVec3DCreate( rkBodyPos(body), x, y, z );
  zMat3DIdent( rkBodyAtt(body) );
  zVec6DCreate( rkBodyVel(body), vx, vy, vz, 0, 0, 0 );
  zVec6DCreate( rkBodyAcc(body), ax, ay, az, 0, 0, 0 );
#if 0
  zVec6DClear( f ); /* dummy. to be theoretical value. */
#endif
}

void pattern2(rkBody *body, double t)
{
  /* p=0, w=v */
  double x, y, z;
  double vx, vy, vz;
  zVec3D aa;

  vx = 1; x = vx*t;
  vy = 1; y = vy*t;
  vz = 1; z = vz*t;
  zVec3DClear( rkBodyPos(body) );
  zVec3DCreate( &aa, x, y, z );
  zMat3DAA( rkBodyAtt(body), &aa );
  zVec6DCreate( rkBodyVel(body), 0, 0, 0, vx, vy, vz );
  zVec6DCreate( rkBodyAcc(body), 0, 0, 0, 0, 0, 0 );
}

void pattern3(rkBody *body, double t)
{
  /* p=vt, w=v */
  double x, y, z;
  double vx, vy, vz;
  zVec3D aa;

  vx = 1; x = vx*t;
  vy = 1; y = vy*t;
  vz = 1; z = vz*t;
  zVec3DCreate( rkBodyPos(body), 0.1*x, 0.1*y, 0.1*z );
  zVec3DCreate( &aa, x, y, z );
  zMat3DAA( rkBodyAtt(body), &aa );
  zVec6DCreate( rkBodyVel(body), 0.1*vx, 0.1*vy, 0.1*vz, vx, vy, vz );
  zVec6DCreate( rkBodyAcc(body), 0, 0, 0, 0, 0, 0 );
}

void pattern4(rkBody *body, double t)
{
  /* p=0, w=at */
  double x, y, z;
  double vx, vy, vz;
  double ax, ay, az;
  zVec3D aa;

  ax = 0.1; vx = ax * t; x = 0.5*ax*t*t;
  ay = 0.1; vy = ay * t; y = 0.5*ay*t*t;
  az = 0.1; vz = az * t; z = 0.5*az*t*t;
  zVec3DClear( rkBodyPos(body) );
  zVec3DCreate( &aa, x, y, z );
  zMat3DAA( rkBodyAtt(body), &aa );
  zVec6DCreate( rkBodyVel(body), 0, 0, 0, vx, vy, vz );
  zVec6DCreate( rkBodyAcc(body), 0, 0, 0, ax, ay, az );
}

void test(void (* pat)(rkBody*,double), rkBody *body)
{
  register int i;
  double dt;
  zVec6D w;

  dt = T / DIV;
  for( i=0; i<DIV; i++ ){
    pat( body, dt*i );
    rkBodyUpdateCOM( body );
    rkBodyUpdateCOMRate( body );
    rkBodyNetWrench( body, &w );
    output( body, &w );
  }
}

int main(int argc, char *argv[])
{
  rkBody body;

  init_mp( &body );
  if( argc > 1 )
    switch( atoi(argv[1]) ){
    case 2: test( pattern2, &body ); goto TERMINATE;
    case 3: test( pattern3, &body ); goto TERMINATE;
    case 4: test( pattern4, &body ); goto TERMINATE;
    default: ;
    }
  test( pattern1, &body );
 TERMINATE:
  rkBodyDestroy( &body );
  return 0;
}
