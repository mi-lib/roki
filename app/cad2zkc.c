#include <roki/rk_chain.h>

typedef struct __cad2zkc_t{
  Z_NAMED_CLASS
  struct __cad2zkc_t *parent;
  zFrame3D f;
  rkMP mp;
} cad2zkc_t;

zListClass( cad2zkc_list_t, cad2zkc_list_cell_t, cad2zkc_t );

cad2zkc_t *cad2zkc_pickup(cad2zkc_list_t *list, char *name)
{
  cad2zkc_list_cell_t *p;

  if( !strcmp( name, "none" ) ) return NULL;
  zListForEach( list, p )
    if( !strcmp( zName(&p->data), name ) ) return &p->data;
  ZRUNERROR( "%s not found", name );
  return NULL;
}

bool cad2zkc_mirror(cad2zkc_list_t *list, cad2zkc_t *data, FILE *fin, char buf[], int size)
{
  cad2zkc_t *p;

  zFToken( fin, buf, size );
  if( !( p = cad2zkc_pickup( list, buf ) ) ) return false;
  zFToken( fin, buf, size );
  data->parent = cad2zkc_pickup( list, buf );

  zVec3DCreate( zFrame3DPos(&data->f),
    zFrame3DPos(&p->f)->e[zX],
   -zFrame3DPos(&p->f)->e[zY],
    zFrame3DPos(&p->f)->e[zZ] );
  zMat3DCreate( zFrame3DAtt(&data->f),
    zFrame3DAtt(&p->f)->e[0][0],
    zFrame3DAtt(&p->f)->e[1][0],
   -zFrame3DAtt(&p->f)->e[2][0],
   -zFrame3DAtt(&p->f)->e[0][1],
   -zFrame3DAtt(&p->f)->e[1][1],
    zFrame3DAtt(&p->f)->e[2][1],
    zFrame3DAtt(&p->f)->e[0][2],
    zFrame3DAtt(&p->f)->e[1][2],
   -zFrame3DAtt(&p->f)->e[2][2] );
  rkMPSetMass( &data->mp, rkMPMass(&p->mp) );
  zVec3DCreate( rkMPCOM(&data->mp),
    rkMPCOMElem(&p->mp,zX),-rkMPCOMElem(&p->mp,zY), rkMPCOMElem(&p->mp,zZ) );
  zMat3DCreate( rkMPInertia(&data->mp),
    rkMPInertiaElem(&p->mp,0,0),
   -rkMPInertiaElem(&p->mp,0,1),
    rkMPInertiaElem(&p->mp,0,2),
   -rkMPInertiaElem(&p->mp,1,0),
    rkMPInertiaElem(&p->mp,1,1),
   -rkMPInertiaElem(&p->mp,1,2),
    rkMPInertiaElem(&p->mp,2,0),
   -rkMPInertiaElem(&p->mp,2,1),
    rkMPInertiaElem(&p->mp,2,2) );
  return true;
}

bool cad2zkc_list_create(cad2zkc_list_t *list, FILE *fin)
{
  char buf[BUFSIZ];
  cad2zkc_list_cell_t *cell;
  double ixx[6];
  int i;

  zListInit( list );
  while( 1 ){
    if( !zFToken( fin, buf, BUFSIZ ) ) break;
    if( !( cell = zAlloc( cad2zkc_list_cell_t, 1 ) ) )
      goto ALLOCERROR;
    zListInsertHead( list, cell );
    if( !zNameSet( &cell->data, buf ) )
      goto ALLOCERROR;
    zFToken( fin, buf, BUFSIZ );
    if( !strcmp( buf, "mirror" ) ){
      if( !cad2zkc_mirror( list, &cell->data, fin, buf, BUFSIZ ) )
        goto ALLOCERROR;
      continue;
    }
    cell->data.parent = cad2zkc_pickup( list, buf );
    zVec3DFRead( fin, zFrame3DPos(&cell->data.f) );
    zMat3DFRead( fin, zFrame3DAtt(&cell->data.f) );
    rkMPSetMass( &cell->data.mp, zFDouble(fin) );
    zVec3DFRead( fin, rkMPCOM(&cell->data.mp) );
    for( i=0; i<6; i++ )
      ixx[i] = zFDouble( fin );
    zMat3DCreate( rkMPInertia(&cell->data.mp),
      ixx[0], ixx[3], ixx[4],
      ixx[3], ixx[1], ixx[5],
      ixx[4], ixx[5], ixx[2] );
  }
  return true;

 ALLOCERROR:
  ZALLOCERROR();
  return false;
}

void cad2zkc_list_destroy(cad2zkc_list_t *list)
{
  cad2zkc_list_cell_t *cell;

  while( !zListIsEmpty( list ) ){
    zListDeleteHead( list, &cell );
    zNameDestroy( &cell->data );
    zFree( cell );
  }
}

void cad2zkc_conv_one(cad2zkc_t *data)
{
  zFrame3D f;
  rkMP mp;
  zMat3D ri;

  printf( "[link]\nname: %s\n", zName(data) );
  if( !data->parent ){
    printf( "jointtype: fix\n" );
    rkMPWrite( &data->mp );
    printf( "frame: " );
    zFrame3DWrite( &data->f );
    zEndl();
    return;
  }
  zFrame3DXfer( &data->parent->f, &data->f, &f );
  rkMPSetMass( &mp, rkMPMass(&data->mp) );
  zXfer3DInv( &data->f, rkMPCOM(&data->mp), rkMPCOM(&mp) );
  zMulMatTMat3D( zFrame3DAtt(&data->f), rkMPInertia(&data->mp), &ri );
  zMulMatMat3D( &ri, zFrame3DAtt(&data->f), rkMPInertia(&mp) );
  printf( "jointtype: revolute\n" );
  rkMPWrite( &mp );
  printf( "frame: " );
  zFrame3DWrite( &f );
  printf( "parent: %s\n\n", zName(data->parent) );
}

void cad2zkc_conv(cad2zkc_list_t *list)
{
  cad2zkc_list_cell_t *cell;

  zListForEach( list, cell )
    cad2zkc_conv_one( &cell->data );
}

int main(int argc, char *argv[])
{
  FILE *fin;
  cad2zkc_list_t list;

  if( argc > 1 ){
    if( !( fin = fopen( argv[1], "r" ) ) ){
      ZOPENERROR( argv[1] );
      exit( 1 );
    }
  } else{
    fin = stdin;
  }
  cad2zkc_list_create( &list, fin );
  cad2zkc_conv( &list );
  cad2zkc_list_destroy( &list );
  if( fin != stdin )
    fclose( fin );
  return 0;
}
