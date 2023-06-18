#include <roki/roki.h>

int main(void)
{
  rkContactInfoArray carray;

  rkContactInfoArrayReadZTK( &carray, "contact.ztk" );
  rkContactInfoArrayFPrintZTK( stdout, &carray );
  rkContactInfoArrayDestroy( &carray );
  return 0;
}
