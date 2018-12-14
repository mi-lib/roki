#include <roki/rk_chain.h>

static rkChain chain;

enum{
  RK_MP_MFILE=0,
  RK_MP_VFILE,
  RK_MP_HELP,
  RK_MP_INVALID
};
zOption option[] = {
  { "m", "model", "<.zkc file>", "chain model file", NULL, false },
  { "v", "v", "<.zv file>", "chain pose file", NULL, false },
  { "h", "help", NULL, "show this message", NULL, false },
  { NULL, NULL, NULL, NULL, NULL, false },
};

void rk_mpUsage(void)
{
  eprintf( "Usage: rk_mp [options]\n" );
  eprintf( "<options>\n" );
  zOptionHelp( option );
  exit( 0 );
}

bool rk_mpInitFK(rkChain *chain)
{
  zVec v;

  if( !( v = zVecReadFile( option[RK_MP_VFILE].arg ) ) ){
    ZALLOCERROR();
    return false;
  }
  rkChainFK( chain, v );
  zVecFree( v );
  return true;
}

bool rk_mpCommandArgs(int argc, char *argv[])
{
  zStrList arglist;
  char *mfile, *vfile;

  if( argc <= 1 ) rk_mpUsage();
  zOptionRead( option, argv, &arglist );
  if( option[RK_MP_HELP].flag ) rk_mpUsage();
  zStrListGetPtr( &arglist, 2, &mfile, &vfile );
  if( mfile ){
    option[RK_MP_MFILE].flag = true;
    option[RK_MP_MFILE].arg  = mfile;
  }
  if( !option[RK_MP_MFILE].flag ){
    ZRUNERROR( "kinematic chain model not assigned" );
    return false;
  }
  if( vfile ){
    option[RK_MP_VFILE].flag = true;
    option[RK_MP_VFILE].arg  = vfile;
  }
  if( !rkChainReadFile( &chain, option[RK_MP_MFILE].arg ) )
    return 1;
  if( option[RK_MP_VFILE].flag )
    if( !rk_mpInitFK( &chain ) ) return 1;
  zStrListDestroy( &arglist, false );
  return true;
}

zListClass(int_list_t, int_list_cell_t, int);
zIndex rk_mpLinkList(rkChain *chain)
{
  int_list_t list;
  int_list_cell_t *cp;
  zIndex index;
  int i;

  zListInit( &list );
  for( i=0; i<rkChainNum(chain); i++ )
    printf( "[%2d] %s\n", i, zName(rkChainLink(chain,i)) );
  while( 1 ){
    printf( "enter link ID> " );
    if( scanf( "%d", &i ) != 1 ) break;
    if( i < 0 || !rkChainLink( chain, i ) ) break;
    if( !( cp = zAlloc( int_list_cell_t, 1 ) ) ){
      ZALLOCERROR();
      break;
    }
    cp->data = i;
    zListInsertHead( &list, cp );
  }
  if( zListIsEmpty( &list ) ) return NULL;
  if( !( index = zIndexCreate( zListNum(&list) ) ) )
    ZALLOCERROR();
  else
    for( cp=zListTail(&list), i=0; i<zArrayNum(index);
         i++, cp=zListCellNext(cp) )
      zIndexSetElem( index, i, cp->data );
  zListDestroy( int_list_cell_t, &list );
  return index;
}

zMat3D *rk_mpShiftInertia(zMat3D *i, double m, zVec3D *s)
{
  zMat3D tmp;
  double xx, xy, yy, yz, zz, zx;

  xx = zSqr(s->e[zX]);
  yy = zSqr(s->e[zY]);
  zz = zSqr(s->e[zZ]);
  xy = s->e[zX] * s->e[zY];
  yz = s->e[zY] * s->e[zZ];
  zx = s->e[zZ] * s->e[zX];
  zMat3DCreate( &tmp,
    yy+zz,   -xy,    -zx,
      -xy, zz+xx,    -yz,
      -zx,   -yz,  xx+yy );
  return zMat3DCatDRC( i, m, &tmp );
}

zMat3D *rk_mpCatInertia(zMat3D *is, zMat3D *att, double m, zVec3D *s, zMat3D *i)
{
  zMat3D tmp;

  zRotMat3D( att, is, &tmp );
  rk_mpShiftInertia( &tmp, m, s );
  return zMat3DAddDRC( i, &tmp );
}

void rk_mpCalc(rkChain *chain, zIndex index, rkMP *mp)
{
  rkLink *l;
  zVec3D rc;
  int i;

  rkMPMass(mp) = 0;
  zVec3DClear( rkMPCOM(mp) );
  zMat3DClear( rkMPInertia(mp) );

  for( i=0; i<zArrayNum(index); i++ ){
    l = rkChainLink( chain, zIndexElem(index,i) );
    /* mass */
    rkMPMass(mp) += rkLinkMass(l);
    /* COM */
    zVec3DCatDRC( rkMPCOM(mp), rkLinkMass(l), rkLinkWldCOM(l) );
    /* inertia tensor */
    rk_mpCatInertia( rkLinkInertia(l), rkLinkWldAtt(l), rkLinkMass(l), rkLinkWldCOM(l), rkMPInertia(mp) );
  }

  if( !zIsTiny( rkMPMass(mp) ) )
    zVec3DDivDRC( rkMPCOM(mp), rkMPMass(mp) );
  zVec3DRev( rkMPCOM(mp), &rc );
  rk_mpShiftInertia( rkMPInertia(mp),-rkMPMass(mp), &rc );
}

int main(int argc, char *argv[])
{
  zIndex index;
  rkMP mp;

  if( !rk_mpCommandArgs( argc, argv+1 ) ) return 1;
  if( !( index = rk_mpLinkList( &chain ) ) )
    return 1;

  rk_mpCalc( &chain, index, &mp );
  rkMPWrite( &mp );

  zIndexFree( index );
  rkChainDestroy( &chain );
  return 0;
}
