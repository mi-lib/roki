/* RoKi - Robot Kinetics library
 * Copyright (C) 1998 Tomomichi Sugihara (Zhidao)
 *
 * rk_force - wrench (six-axis force vector)
 */

#include <roki/rk_force.h>

/* ********************************************************** */
/* CLASS: rkWrench & rkWrenchList
 * wrench list class
 * ********************************************************** */

/* rkWrenchListPop
 * - pop to wrench list.
 */
rkWrench *rkWrenchListPop(rkWrenchList *wl)
{
  rkWrench *cell;

  if( zListIsEmpty( wl ) ) return NULL;
  zListDeleteTail( wl, &cell );
  return cell;
}

/* rkWrenchXfer
 * - convert a wrench.
 */
zVec6D *rkWrenchXfer(rkWrench *cell, zVec6D *w)
{
  return zVec6DAngShift( rkWrenchW(cell), rkWrenchPos(cell), w );
}

/* rkWrenchListNet
 * - calculate net wrench of a list.
 */
zVec6D *rkWrenchListNet(rkWrenchList *list, zVec6D *w)
{
  rkWrench *cell;
  zVec6D n;

  zVec6DClear( w );
  zListForEach( list, cell ){
    rkWrenchXfer( cell, &n );
    zVec6DAddDRC( w, &n );
  }
  return w;
}

/* rkWrenchFWrite
 * - output wrench to file.
 */
void rkWrenchFWrite(FILE *fp, rkWrench *cell)
{
  if( !cell ) return;

  fprintf( fp, " force: " );
  zVec3DFWrite( fp, rkWrenchForce( cell ) );
  fprintf( fp, " torque: " );
  zVec3DFWrite( fp, rkWrenchTorque( cell ) );
  fprintf( fp, " point of action: " );
  zVec3DFWrite( fp, rkWrenchPos( cell ) );
}
