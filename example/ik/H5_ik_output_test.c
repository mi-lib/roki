#include <roki/rk_chain.h>

static void _rkChainIKConfJointFPrintZTK(FILE *fp, rkChain *chain){
  zVec6D dis;
  int i;
  bool dis_is_zero;

  for( i=0; i<rkChainLinkNum(chain); i++ ){
    if( rkChainLinkJointDOF(chain,i) == 0 ) continue;
    zVec6DZero( &dis );
    rkChainLinkJointGetDis( chain, i, dis.e );
    dis_is_zero = _zVec6DIsTiny( &dis );
    if( chain->_ik->joint_is_enabled[i] || ( !chain->_ik->joint_is_enabled[i] && !dis_is_zero ) ){
      fprintf( fp, "joint: %s\t%g", zName(rkChainLink(chain,i)), chain->_ik->joint_weight[i] );
      if( !dis_is_zero ){
        fprintf( fp, " " );
        rkChainLinkJoint(chain,i)->com->_dis_fprintZTK( fp, i, rkChainLinkJoint(chain,i) );
      } else
        fprintf( fp, "\n" );
    }
  }
}

static void _rkIKConstraintFPrintZTK(FILE *fp, rkChain *chain){
  rkIKCell *cp;

  zListForEach( &chain->_ik->_c_list, cp ){
    fprintf( fp, "constraint: %s %s", zName(&cp->data), cp->data.constraint->typestr );
    fprintf( fp, "\n" );
  }
}




/* print the inverse kinematics configuration of a kinematic chain out to the current position of a file. */
void rkChainIKConfFPrintZTK(FILE *fp, rkChain *chain)
{
  fprintf( fp, "[%s]\n", ZTK_TAG_RKIK );
  _rkChainIKConfJointFPrintZTK( fp, chain );
  _rkIKConstraintFPrintZTK( fp, chain );
  fprintf( fp, "\n" );
}

/* write the inverse kinematics configuration of a kinematic chain to a file in ZTK format. */
bool rkChainIKConfWriteZTK(rkChain *chain, const char *filename)
{
  FILE *fp;

  if( !( fp = zOpenZTKFile( (char *)filename, "w" ) ) ) return false;
  rkChainIKConfFPrintZTK( fp, chain );
  fclose( fp );
  return true;
}



#define H5_ZTK "../model/H5.ztk"

int main(int argc, char *argv[])
{
  rkChain robot;

  rkChainReadZTK( &robot, H5_ZTK );
  rkChainIKConfReadZTK( &robot, H5_ZTK );
  rkChainIKConfWriteZTK( &robot, "hoge" );
  rkChainDestroy( &robot );
  return 0;
}
