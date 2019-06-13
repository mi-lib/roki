#include <roki/rk_ik.h>

int main(int argc, char *argv[])
{
  rkIKSeq seq;

  rkIKSeqScan( &seq );
  rkIKSeqPrint( &seq );
  rkIKSeqFree( &seq );
  return 0;
}
