#include <roki/roki.h>

#define KEY_ALM "alminum"
#define KEY_URE "urethan"

void assert_assoc(void)
{
  rkContactInfo ci1, ci2;

  rkContactInfoRigidCreate( &ci1, 10, 0.1, 1.0, 3.0, KEY_ALM, KEY_ALM );
  rkContactInfoElasticCreate( &ci2, 100, 10, 1.0, 2.0, KEY_ALM, KEY_URE );

  zAssert( rkContactInfoAssoc,
    rkContactInfoAssoc( &ci1, KEY_ALM, KEY_ALM ) &&
    rkContactInfoAssoc( &ci2, KEY_ALM, KEY_URE ) &&
    !rkContactInfoAssoc( &ci1, KEY_ALM, KEY_URE ) &&
    !rkContactInfoAssoc( &ci2, KEY_ALM, KEY_ALM ) );

  rkContactInfoDestroy( &ci1 );
  rkContactInfoDestroy( &ci2 );
}

void assert_pool(void)
{
  rkContactInfoPool cip;

  eprintf( "(duplicate error expected.)\n" );
  zAssert( rkContactInfoPoolReadFile, rkContactInfoPoolReadFile( &cip, "test" ) );
  zAssert( rkContactInfoPoolAssoc,
    rkContactInfoPoolAssoc( &cip, "stf1", "stf2" ) &&
    rkContactInfoPoolAssoc( &cip, "stf3", "stf4" ) &&
    !rkContactInfoPoolAssoc( &cip, "stf1", "stf1" ) &&
    !rkContactInfoPoolAssoc( &cip, "stf1", "stf4" ) );
  zAssert( rkContactInfoPoolAssocType,
    rkContactInfoPoolAssocType( &cip, "stf1", "stf2", RK_CONTACT_ELASTIC ) &&
    rkContactInfoPoolAssocType( &cip, "stf1", "stf2", RK_CONTACT_RIGID ) &&
    rkContactInfoPoolAssocType( &cip, "stf3", "stf4", RK_CONTACT_RIGID ) &&
    !rkContactInfoPoolAssocType( &cip, "stf3", "stf4", RK_CONTACT_ELASTIC ) );
  rkContactInfoPoolDestroy( &cip );
}

int main(void)
{
  assert_assoc();
  assert_pool();
  return EXIT_SUCCESS;
}
