#include <roki/rk_body.h>

void init_mp(rkBody *body)
{
  /* 0.1x0.2x0.4[m^3] recutangular solid block
     with 1000[kg/m^3] density */
  rkBodySetMass( body, 0.1*0.2*0.4*1000 );
  zMat3DCreate( rkBodyInertia(body),
    (zSqr(0.2)+zSqr(0.4))*rkBodyMass(body)/12, 0, 0,
    0, (zSqr(0.4)+zSqr(0.1))*rkBodyMass(body)/12, 0,
    0, 0, (zSqr(0.1)+zSqr(0.2))*rkBodyMass(body)/12 );
  zVec3DCreate( rkBodyCOM(body), 0.05, 0.1, 0.2 );
  /* intial motion */
  zVec3DCreate( rkBodyAngVel(body), 0, 0, 1 );
  zVec3DOuterProd(rkBodyAngVel(body), rkBodyCOM(body), rkBodyLinVel(body) );
}

#define DIV 100

int main(void)
{
  rkBody body;
  register int i;
  double theta;
  zMat3D ident;
  zVec3D am, am_com, aa;

  rkBodyInit( &body );
  init_mp( &body );
  zMat3DIdent( &ident );
  for( i=0; i<DIV; i++ ){
    theta = 2 * zPI * i / DIV;
    zMat3DRotYaw( &ident, theta, rkBodyAtt(&body) );
    rkBodyAM( &body, rkBodyCOM(&body), &am_com );
    rkBodyAM( &body, Z_ZEROVEC3D, &am );
    zVec3DDataWrite( &am );
    zVec3DDataWrite( &am_com );
  }
  rkBodyDestroy( &body );
  return 0;
}
