#include <roki/rk_ik.h>

int main(int argc, char *argv[])
{
  rkIKSeq seq;

  rkIKSeqRead( &seq );
  rkIKSeqWrite( &seq );
  rkIKSeqFree( &seq );
  return 0;
}
