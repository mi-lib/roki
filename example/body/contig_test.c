#include <roki/rk_body.h>

void init_shape(zMShape3D *ms)
{
  zMShape3DReadZTK( ms, "../model/cube.ztk" );
}

void init_mp(rkBody *body, zMShape3D *ms)
{
  int i;

  rkBodyInit( body );
  for( i=0; i<zMShape3DShapeNum(ms); i++ )
    rkBodyShapePush( body, zMShape3DShape(ms,i) );
}

#define N 100

int main(int argc, char *argv[])
{
  rkBody body;
  zMShape3D ms;
  zVec3D p, *cv1, *cv2, *dp;
  zShape3D sc;
  double d1, d2;
  int i;
  FILE *fp1, *fp2;

  zRandInit();
  init_shape( &ms );
  init_mp( &body, &ms );
  zFrame3DFromZYX( rkBodyFrame(&body),
    zRandF(-1,1), zRandF(-1,1), zRandF(-1,1),
    zRandF(-1,1), zRandF(-1,1), zRandF(-1,1) );
  /* for assertion */
  zShape3DClone( zMShape3DShape(&ms,0), &sc, NULL );
  zShape3DXform( zMShape3DShape(&ms,0), rkBodyFrame(&body), &sc );
  fp1 = fopen( "p", "w" );
  fp2 = fopen( "cp", "w" );
  for( i=0; i<N; i++ ){
    zVec3DCreate( &p, zRandF(-2,2), zRandF(-2,2), zRandF(-2,2) );
    cv1 = rkBodyContigVert( &body, &p, &d1 );
    cv2 = zShape3DContigVert( &sc, &p, &d2 );
    dp = (zVec3D*)( (long)zShape3DVertBuf(zMShape3DShape(&ms,0)) + (long)cv2 - (long)zShape3DVertBuf(&sc) );
    if( cv1 != dp ) ZRUNERROR( "might be false" );
    zVec3DValueNLFPrint( fp1, &p );
    zVec3DValueNLFPrint( fp1, cv2 );
    fprintf( fp1, "\n\n" );
    zVec3DValueNLFPrint( fp2, cv2 );
  }
  fclose( fp1 );
  fclose( fp2 );
  zShape3DDestroy( &sc );
  rkBodyDestroy( &body );
  zMShape3DDestroy( &ms );
  return 0;
}
