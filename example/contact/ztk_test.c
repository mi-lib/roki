#include <roki/roki.h>

int main(void)
{
  rkContactInfoArray carray;

  rkContactInfoArrayScanZTK( &carray, "contact.ztk" );
  rkContactInfoArrayFPrint( stdout, &carray );
  rkContactInfoArrayDestroy( &carray );
  return 0;
}
