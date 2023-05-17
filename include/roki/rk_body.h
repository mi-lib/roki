/* RoKi - Robot Kinetics library
 * Copyright (C) 1998 Tomomichi Sugihara (Zhidao)
 *
 * rk_body - body with mass property
 */

#ifndef __RK_BODY_H__
#define __RK_BODY_H__

#include <roki/rk_g3d.h>
#include <roki/rk_contact.h>
#include <roki/rk_force.h>

__BEGIN_DECLS

/* ********************************************************** */
/* CLASS: rkMP
 * mass property class
 * ********************************************************** */

ZDEF_STRUCT( rkMP ){
  /* mass property */
  double mass;
  zVec3D com;
  zMat3D inertia;
#ifdef __cplusplus
  rkMP();
  double Mass() const;
  zVec3D &COM();
  zMat3D &Inertia();
  double setMass(double);
  zVec3D &setCOM(zVec3D &);
  zMat3D &setInertia(zMat3D &);
  rkMP *copy(rkMP &);
  void zero();
  friend rkMP operator*(zFrame3D &f, rkMP &src);
#endif /* __cplusplus */
};

#define rkMPMass(mp)           (mp)->mass
#define rkMPCOM(mp)            ( &(mp)->com )
#define rkMPInertia(mp)        ( &(mp)->inertia )

#define rkMPCOMElem(mp,i)       rkMPCOM(mp)->e[i]
#define rkMPInertiaElem(mp,i,j) rkMPInertia(mp)->e[j][i]

#define rkMPSetMass(mp,m)      ( rkMPMass(mp) = (m) )
#define rkMPSetCOM(mp,c)       zVec3DCopy( c, rkMPCOM(mp) )
#define rkMPSetInertia(mp,i)   zMat3DCopy( i, rkMPInertia(mp) )

#define rkMPCopy(src,dst)      ( *(dst) = *(src) )

/*! \brief clear mass property. */
#define rkMPZero(mp) do{\
  rkMPSetMass( (mp), 0 );\
  zVec3DZero( rkMPCOM(mp) );\
  zMat3DZero( rkMPInertia(mp) );\
} while(0)

/*! \brief convert mass properties in [g,mm] to that in [kg,m]. */
__ROKI_EXPORT rkMP *rkMPgmm2kgm(rkMP *mp);

/*! \brief shift inertia tensor by an offset 3D vector. */
#define rkMPShiftInertia(mp,r,i) zMat3DCatVec3DDoubleOuterProd( rkMPInertia(mp), -rkMPMass(mp), (r), (i) )

/*! \brief transform mass properties to that with respect to a frame.
 *
 * rkMPXform() transforms a set of mass properties \a src to that
 * with respect to a frame \a f.
 * The mass of \a dest will be the same with that of \a src.
 * The COM of \a src will be transformed to that of \a dest by \a f.
 * The inertia tensor of \a src will be rotated to that of \a dest
 * with respect to the attitude of \a f.
 *
 * rkMPXformInv() transforms \a src to that with respect to the
 * inverse of a frame \a f.
 *
 * \return \a dest
 */
__ROKI_EXPORT rkMP *rkMPXform(rkMP *src, zFrame3D *f, rkMP *dest);
__ROKI_EXPORT rkMP *rkMPXformInv(rkMP *src, zFrame3D *f, rkMP *dest);

/*! \brief combine two mass property sets in the same frame.
 */
__ROKI_EXPORT rkMP *rkMPCombine(rkMP *mp1, rkMP *mp2, rkMP *mp);

/* \brief convert inertia tensor to that about the origin.
 */
__ROKI_EXPORT zMat3D *rkMPOrgInertia(rkMP *mp, zMat3D *i);

/* \brief compute the inertial ellipsoid from a mass property set.
 */
__ROKI_EXPORT zEllips3D *rkMPInertiaEllips(rkMP *mp, zEllips3D *ie);

/*! \brief print mass property.
 *
 * rkMPFPrint() prints a set of mass properties \a mp out to
 * the current position of a file \a fp in the following format.
 *
 *  mass: <m>
 *  COM:  { <x>, <y>, <z> }
 *  inertia: {
 *    <ixx>, <ixy>, <izx>,
 *    <iyx>, <iyy>, <iyz>,
 *    <izx>, <iyz>, <izz>
 *  }
 *
 * rkMPPrint() prints a set of mass properties \a mp out to the
 * standard output in the same format with the above.
 * \return
 * rkMPFPrint() and rkMPPrint() return no value.
 */
__ROKI_EXPORT void rkMPFPrint(FILE *fp, rkMP *mp);
#define rkMPPrint(mp) rkMPFPrint( stdout, mp )

#ifdef __cplusplus
inline rkMP::rkMP(){ rkMPZero( this ); }
inline double rkMP::Mass() const { return rkMPMass( this ); }
inline zVec3D &rkMP::COM(){ return *rkMPCOM( this ); }
inline zMat3D &rkMP::Inertia(){ return *rkMPInertia( this ); }
inline double rkMP::setMass(double m){ return rkMPSetMass( this, m ); }
inline zVec3D &rkMP::setCOM(zVec3D &p){ rkMPSetCOM( this, &p ); return COM(); }
inline zMat3D &rkMP::setInertia(zMat3D &i){ rkMPSetInertia( this, &i ); return Inertia(); }
inline rkMP *rkMP::copy(rkMP &src){ rkMPCopy( &src, this ); return this; }
inline void rkMP::zero(){ rkMPZero( this ); }
inline rkMP operator*(zFrame3D &f, rkMP &src){
  rkMP dest;
  rkMPXform( &src, &f, &dest );
  return dest;
}
#endif /* __cplusplus */

/* ********************************************************** */
/* CLASS: rkBody
 * rigid body class
 * ********************************************************** */

ZDEF_STRUCT( rkBody ){
  rkMP mp;           /*!< \brief mass property */
  zFrame3D frame;    /*!< \brief absolute transformation frame */
  zVec6D vel;        /*!< \brief velocity */
  zVec6D acc;        /*!< \brief acceleration */
  zVec3D com;        /*!< \brief center of mass (COM) */
  zVec3D comvel;     /*!< \brief COM velocity */
  zVec3D comacc;     /*!< \brief COM acceleration */
  rkWrenchList extw; /*!< \brief external wrench with respect to body frame */
  zShapeList shapelist; /*!< \brief shapes */
  char *stuff;          /*!< \brief stuff identifier */
};

#define rkBodyMP(b)           ( &(b)->mp )
#define rkBodyMass(b)         rkMPMass( rkBodyMP(b) )
#define rkBodyCOM(b)          rkMPCOM( rkBodyMP(b) )
#define rkBodyInertia(b)      rkMPInertia( rkBodyMP(b) )
#define rkBodyFrame(b)        ( &(b)->frame )
#define rkBodyPos(b)          zFrame3DPos( rkBodyFrame(b) )
#define rkBodyAtt(b)          zFrame3DAtt( rkBodyFrame(b) )
#define rkBodyVel(b)          ( &(b)->vel )
#define rkBodyAcc(b)          ( &(b)->acc )
#define rkBodyLinVel(b)       zVec6DLin( rkBodyVel(b) )
#define rkBodyLinAcc(b)       zVec6DLin( rkBodyAcc(b) )
#define rkBodyAngVel(b)       zVec6DAng( rkBodyVel(b) )
#define rkBodyAngAcc(b)       zVec6DAng( rkBodyAcc(b) )
#define rkBodyWldCOM(b)       ( &(b)->com )
#define rkBodyCOMVel(b)       ( &(b)->comvel )
#define rkBodyCOMAcc(b)       ( &(b)->comacc )

#define rkBodyExtWrench(b)    ( &(b)->extw )
#define rkBodyShapeList(b)    ( &(b)->shapelist )
#define rkBodyShapeNum(b)     zListSize( rkBodyShapeList(b) )
#define rkBodyShapeIsEmpty(b) zListIsEmpty( rkBodyShapeList(b) )

#define rkBodySetMass(b,m)    rkMPSetMass( rkBodyMP(b), m )
#define rkBodySetCOM(b,c)     rkMPSetCOM( rkBodyMP(b), c )
#define rkBodySetInertia(b,i) rkMPSetInertia( rkBodyMP(b), i )
#define rkBodySetFrame(b,f)   zFrame3DCopy( f, rkBodyFrame(b) )
#define rkBodySetPos(b,p)     zFrame3DSetPos( rkBodyFrame(b), p )
#define rkBodySetAtt(b,r)     zFrame3DSetAtt( rkBodyFrame(b), r )
#define rkBodySetVel(b,v)     zVec6DCopy( v, rkBodyVel(b) )
#define rkBodySetAcc(b,a)     zVec6DCopy( a, rkBodyAcc(b) )
#define rkBodySetLinVel(b,v)  zVec6DSetLin( rkBodyVel(b), v )
#define rkBodySetLinAcc(b,a)  zVec6DSetLin( rkBodyAcc(b), a )
#define rkBodySetAngVel(b,v)  zVec6DSetAng( rkBodyVel(b), v )
#define rkBodySetAngAcc(b,a)  zVec6DSetAng( rkBodyAcc(b), a )
#define rkBodySetWldCOM(b,c)  zVec3DCopy( c, rkBodyWldCOM(b) )
#define rkBodySetCOMVel(b,v)  zVec3DCopy( v, rkBodyCOMVel(b) )
#define rkBodySetCOMAcc(b,a)  zVec3DCopy( a, rkBodyCOMAcc(b) )

#define rkBodyStuff(b)        (b)->stuff
#define rkBodySetStuff(b,m)   ( rkBodyStuff(b) = zStrClone(m) )
#define rkBodyStuffDestroy(b) zFree( rkBodyStuff(b) )

/*! \brief initialize and destroy body object.
 *
 * rkBodyInit() initializes a body object \a body.
 *
 * rkBodyDestroy() destroys a body object \a body by freeing the
 * memory allocated for its name and external forces.
 * \return
 * rkBodyInit() and rkBodyDestroy() return no value.
 */
__ROKI_EXPORT void rkBodyInit(rkBody *body);
__ROKI_EXPORT void rkBodyDestroy(rkBody *body);

/*! \brief clone a body.
 *
 * rkBodyClone() clones a body \a org, namely, copies its mass-property,
 * external wrench, material stuff and multishape, to another \a cln.
 *
 * The multishapes associated with \a org and \a cln are pointed by
 * \a so and \sc, respectively. It is supposed that the orders of the
 * shapes of \a org and \a cln are the same in \a so and \sc. Namely,
 * if the k-th shape of \a so is attached with \a org, the k-th shape
 * of \a sc is supposed to be attached with \a cln.
 * \return cln
 */
__ROKI_EXPORT rkBody *rkBodyClone(rkBody *org, rkBody *cln, zMShape3D *so, zMShape3D *sc);

/*! \brief zero velocity and acceleration of a body.
 *
 * rkBodyZeroRate() zeroes velocity and acceleration of a body.
 * \return
 * rkBodyZeroRate() returns no value.
 */
__ROKI_EXPORT void rkBodyZeroRate(rkBody *body);

/*! \brief copy state of a body.
 *
 * rkBodyCopyState() copies state of a body \a src to that of another
 * \a dst. The state includes frame, velocity, acceleration, and the
 * position, velocity and acceleration of the center of mass.
 * \return dst
 */
__ROKI_EXPORT rkBody *rkBodyCopyState(rkBody *src, rkBody *dst);

/*! \brief combine two bodies.
 *
 * rkBodyCombine() combines mass properties of the two bodies \a b1
 * and \a b2 to one body \a b which is denoted in a frame \a f.
 * \return b
 */
__ROKI_EXPORT rkBody *rkBodyCombine(rkBody *b1, rkBody *b2, zFrame3D *f, rkBody *b);

/*! \brief combine a body directly to another.
 *
 * rkBodyCombineDRC() combines mass properties of a given body \a sb
 * directly to another \a b.
 * \return b
 */
__ROKI_EXPORT rkBody *rkBodyCombineDRC(rkBody *b, rkBody *sb);

/* \brief compute the inertial ellipsoid from a rigid body.
 */
#define rkBodyInertiaEllips(b,e) rkMPInertiaEllips( rkBodyMP(b), e )

/*! \brief update body COM state.
 *
 * rkBodyUpdateCOM() updates the COM position of a body \a body by
 * transforming the local COM position of \a body to that with respect
 * to the world frame. The result is stored into the internal member
 * of \a body, which can be referred by rkBodyWldCOM().
 *
 * rkBodyUpdateCOMRate() updates the COM velocity and acceleration of
 * a body object \a body with respect to the inertial frame by using
 * information of the velocity and acceleration of the original point
 * of a body object \a body and its local COM position. The results are
 * also stored into the internal members, which can be referred by
 * rkBodyCOMVel() and rkBodyCOMAcc().
 * \return
 * rkBodyUpdateCOM() returns a pointer to the updated COM position
 * vector of \a body.
 *
 * rkBodyUpdateCOMRate() returns no value.
 */
__ROKI_EXPORT zVec3D *rkBodyUpdateCOM(rkBody *body);
__ROKI_EXPORT void rkBodyUpdateCOMVel(rkBody *body);
__ROKI_EXPORT void rkBodyUpdateCOMAcc(rkBody *body);
__ROKI_EXPORT void rkBodyUpdateCOMRate(rkBody *body);

/*! \brief push and pop external force applied to body.
 *
 * rkBodyExtForcePush() pushes a new external force list cell \a f to
 * the list on a body object \a b.
 *
 * rkBodyExtForcePop() pops the latest external force list cell from
 * the list on a body object \a b.
 *
 * rkBodyExtForceDestroy() destroys the external force list on a body
 * object \a b by freeing all cells.
 * \notes
 * When the external force list on \a b includes statically-allocated
 * cells, rkBodyExtForceDestroy() causes segmentation fault.
 * \return
 * rkBodyExtForcePush() returns a pointer to the cell pushed.
 * rkBodyExtForcePop() returns a pointer to the cell poped.
 * rkBodyExtForceDestroy() returns no value.
 */
#define rkBodyExtWrenchPush(b,f)  rkWrenchListPush( rkBodyExtWrench(b), f )
#define rkBodyExtWrenchPop(b)     rkWrenchListPop( rkBodyExtWrench(b) )
#define rkBodyExtWrenchDestroy(b) rkWrenchListDestroy( rkBodyExtWrench(b) )

/*! \brief calculate total external wrench acting to a body.
 *
 * rkBodyNetExtWrench() calculates the net external wrench acting to a
 * body \a b by summing up individual external forces in the force list.
 * The result is put into \a w.
 * \return
 * rkBodyNetExtWrench() returns a pointer \a w.
 */
#define rkBodyNetExtWrench(b,w) rkWrenchListNet( rkBodyExtWrench(b), w )

/*! \brief net wrench exerted on a body.
 *
 * rkBodyNetWrench() calculates the net wrench (six-axis force) applied to
 * a body \a body based on the Newton-Euler's equation of motion.
 * \notes
 * It is assumed that the orientation of absolute velocity and acceleration
 * of the body \a body is all with respect to the local frame of \a body
 * itself. Consequently, the result net wrench is also with respect to the
 * local frame in terms of orientation.
 */
__ROKI_EXPORT zVec6D *rkBodyNetWrench(rkBody *body, zVec6D *w);

/*! \brief angular momentum and kinematic energy of body.
 *
 * rkBodyAM() calculates the angular momentum of a body object \a b
 * around a point \a p and stores the result into \a am. Both \a p and
 * \a am are with respect to the body frame.
 *
 * rkBodyKE() calculates the kinematic energy originating from linear
 * and angular velocity of a body object \a b.
 * \return
 * rkBodyAM() returns a pointer \a am.
 * rkBodyKE() returns the value calculated.
 */
__ROKI_EXPORT zVec3D *rkBodyAM(rkBody *b, zVec3D *p, zVec3D *am);
__ROKI_EXPORT double rkBodyKE(rkBody *b);

/*! \brief push and pop of shape attached to body.
 *
 * rkBodyShapePush() pushes a new shape \a shape to the shape list of
 * a body object \a b.
 *
 * rkBodyShapePop() pops the last shape of the list attached to a body
 * object \a b.
 *
 * rkBodyShapeDestroy() destroys the shape list of a body object \a b
 * by freeing all cells.
 * \notes
 * When the shape list of the body \a b includes statically-allocated
 * cells, rkBodyShapeDestroy() causes segmentation fault.
 * \return
 * rkBodyShapePush() returns a pointer to the cell pushed.
 * rkBodyShapePop() returns a pointer to the shape poped.
 * rkBodyShapeDestroy() returns no value.
 */
#define rkBodyShapePush(b,s)  zShapeListPush( rkBodyShapeList(b), s )
#define rkBodyShapePop(b)     zShapeListPop( rkBodyShapeList(b) )
#define rkBodyShapeDestroy(b) zShapeListDestroy( rkBodyShapeList(b) )

/*! \brief contiguous vertex of a body to a point.
 */
__ROKI_EXPORT zVec3D *rkBodyContigVert(rkBody *body, zVec3D *p, double *d);

/*! \brief compute volume of a body. */
__ROKI_EXPORT double rkBodyShapeVolume(rkBody *body);
/*! \brief compute mass property of a body. */
__ROKI_EXPORT rkMP *rkBodyShapeMP(rkBody *body, double density, rkMP *mp);

__END_DECLS

#endif /* __RK_BODY_H__ */
