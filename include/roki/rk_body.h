/* RoKi - Robot Kinetics library
 * Copyright (C) 1998 Tomomichi Sugihara (Zhidao)
 *
 * rk_body - body with mass property
 */

#ifndef __RK_BODY_H__
#define __RK_BODY_H__

#include <roki/rk_g3d.h>
#include <roki/rk_contact.h>

__BEGIN_DECLS

/* ********************************************************** */
/* CLASS: rkMP
 * mass property class
 * ********************************************************** */

ZDEF_STRUCT( __ROKI_CLASS_EXPORT, rkMP ){
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

#define rkMPCopy(src,dest)     ( *(dest) = *(src) )

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

__END_DECLS

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

__BEGIN_DECLS

ZDEF_STRUCT( __ROKI_CLASS_EXPORT, rkBody ){
  rkMP mp;              /*!< \brief mass property */
  zFrame3D frame;       /*!< \brief absolute transformation frame */
  zVec6D vel;           /*!< \brief velocity */
  zVec6D acc;           /*!< \brief acceleration */
  zVec3D com;           /*!< \brief center of mass (COM) */
  zVec3D comvel;        /*!< \brief COM velocity */
  zVec3D comacc;        /*!< \brief COM acceleration */
  zVec6D extwrench;     /*!< \brief external wrench with respect to body frame */
  zShapeList shapelist; /*!< \brief shapes */
  char *stuff;          /*!< \brief stuff identifier */
#ifdef __cplusplus
  rkBody();
  ~rkBody();
#endif /* __cplusplus */
};

#define rkBodyMP(body)           ( &(body)->mp )
#define rkBodyMass(body)         rkMPMass( rkBodyMP(body) )
#define rkBodyCOM(body)          rkMPCOM( rkBodyMP(body) )
#define rkBodyInertia(body)      rkMPInertia( rkBodyMP(body) )
#define rkBodyFrame(body)        ( &(body)->frame )
#define rkBodyPos(body)          zFrame3DPos( rkBodyFrame(body) )
#define rkBodyAtt(body)          zFrame3DAtt( rkBodyFrame(body) )
#define rkBodyVel(body)          ( &(body)->vel )
#define rkBodyAcc(body)          ( &(body)->acc )
#define rkBodyLinVel(body)       zVec6DLin( rkBodyVel(body) )
#define rkBodyLinAcc(body)       zVec6DLin( rkBodyAcc(body) )
#define rkBodyAngVel(body)       zVec6DAng( rkBodyVel(body) )
#define rkBodyAngAcc(body)       zVec6DAng( rkBodyAcc(body) )
#define rkBodyWldCOM(body)       ( &(body)->com )
#define rkBodyCOMVel(body)       ( &(body)->comvel )
#define rkBodyCOMAcc(body)       ( &(body)->comacc )

#define rkBodyExtWrench(body)    ( &(body)->extwrench )
#define rkBodyShapeList(body)    ( &(body)->shapelist )
#define rkBodyShapeNum(body)     zListSize( rkBodyShapeList(body) )
#define rkBodyShapeIsEmpty(body) zListIsEmpty( rkBodyShapeList(body) )

#define rkBodySetMass(body,m)    rkMPSetMass( rkBodyMP(body), m )
#define rkBodySetCOM(body,c)     rkMPSetCOM( rkBodyMP(body), c )
#define rkBodySetInertia(body,i) rkMPSetInertia( rkBodyMP(body), i )
#define rkBodySetFrame(body,f)   zFrame3DCopy( f, rkBodyFrame(body) )
#define rkBodySetPos(body,p)     zFrame3DSetPos( rkBodyFrame(body), p )
#define rkBodySetAtt(body,r)     zFrame3DSetAtt( rkBodyFrame(body), r )
#define rkBodySetVel(body,v)     zVec6DCopy( v, rkBodyVel(body) )
#define rkBodySetAcc(body,a)     zVec6DCopy( a, rkBodyAcc(body) )
#define rkBodySetLinVel(body,v)  zVec6DSetLin( rkBodyVel(body), v )
#define rkBodySetLinAcc(body,a)  zVec6DSetLin( rkBodyAcc(body), a )
#define rkBodySetAngVel(body,v)  zVec6DSetAng( rkBodyVel(body), v )
#define rkBodySetAngAcc(body,a)  zVec6DSetAng( rkBodyAcc(body), a )
#define rkBodySetWldCOM(body,c)  zVec3DCopy( c, rkBodyWldCOM(body) )
#define rkBodySetCOMVel(body,v)  zVec3DCopy( v, rkBodyCOMVel(body) )
#define rkBodySetCOMAcc(body,a)  zVec3DCopy( a, rkBodyCOMAcc(body) )

#define rkBodyStuff(body)        (body)->stuff
#define rkBodySetStuff(body,m)   ( rkBodyStuff(body) = zStrClone(m) )
#define rkBodyStuffDestroy(body) zFree( rkBodyStuff(body) )

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
 * material stuff and multishape, to another \a cln.
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
 * rkBodyCopyState() copies state of a body \a src to that of another \a dest. The state includes
 * frame, velocity, acceleration, and the position, velocity and acceleration of the center of mass.
 * \retval dest
 */
__ROKI_EXPORT rkBody *rkBodyCopyState(rkBody *src, rkBody *dest);

/*! \brief combine two bodies.
 *
 * rkBodyCombine() combines mass properties of the two bodies \a body1 and \a body2 to one body \a body
 * which is denoted in a frame \a frame.
 * \retval body
 */
__ROKI_EXPORT rkBody *rkBodyCombine(rkBody *body1, rkBody *body2, zFrame3D *frame, rkBody *body);

/*! \brief combine a body directly to another.
 *
 * rkBodyCombineDRC() combines mass properties of a given body \a sb
 * directly to another \a b.
 * \retval body
 */
__ROKI_EXPORT rkBody *rkBodyCombineDRC(rkBody *body, rkBody *subbody);

/* \brief compute the inertial ellipsoid from a rigid body.
 */
#define rkBodyInertiaEllips(body,e) rkMPInertiaEllips( rkBodyMP(body), e )

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

/*! \brief set and add external wrench or force applied to a body.
 *
 * rkBodySetExtWrench() sets the external wrench of a body \a body for \a wrench.
 * rkBodyAddExtWrench() adds the external wrench \a wrench to a body \a body.
 * rkBodyZeroExtWrench() zeroes the external wrench of a body \a body.
 *
 * rkBodySetExtForce() sets the external force acting at \a pos of a body \a body for \a force.
 * rkBodyAddExtForce() adds the external force \a force acting at \a pos to a body \a body.
 * \return
 * rkBodySetExtWrench(), rkBodyAddExtWrench(), and rkBodyZeroExtWrench() are macros.
 * See rk_body.h.
 * rkBodySetExtForce() and rkBodyAddExtForce() return a pointer to the external wrench, which is
 * a member of \a body,
 */
#define rkBodySetExtWrench(body,wrench) zVec6DCopy( wrench, rkBodyExtWrench(body) )
#define rkBodyAddExtWrench(body,wrench) zVec6DAddDRC( rkBodyExtWrench(body), wrench )
#define rkBodyZeroExtWrench(body)       rkBodySetExtWrench( body, ZVEC6DZERO )
__ZEO_EXPORT zVec6D *rkBodySetExtForce(rkBody *body, zVec3D *force, zVec3D *pos);
__ZEO_EXPORT zVec6D *rkBodyAddExtForce(rkBody *body, zVec3D *force, zVec3D *pos);

/*! \brief inertial wrench of a body.
 *
 * rkBodyInertialWrench() computes the inertial wrench (six-axis force) of a body \a body.
 * The result is put into \a wrench.
 * \notes
 * The orientation of the resulted wrench \a wrench is with respect to the body frame.
 * \return
 * rkBodyInertialWrench() returns a pointer \a wrench.
 */
__ROKI_EXPORT zVec6D *rkBodyInertialWrench(rkBody *body, zVec6D *wrench);

/*! \brief linear and angular momentum and kinematic energy of body.
 *
 * rkBodyLinearMomentum() calculates the linear momentum of a body object \a body, and stores the result
 * into \a momentum. \a momentum is with respect to the body frame.
 *
 * rkBodyAngularMomentum() calculates the angular momentum of a body object \a body around a point \a pos,
 * and stores the result into \a am. Both \a pos and \a am are with respect to the body frame.
 *
 * rkBodyKineticEnergy() calculates the kinetic energy originating from linear and angular velocities
 * of a body object \a body.
 * \return
 * rkBodyLinearMomentum() returns a pointer \a momenum.
 * rkBodyAngularMomentum() returns a pointer \a am.
 * rkBodyKineticEnergy() returns the value calculated.
 */
__ROKI_EXPORT zVec3D *rkBodyLinearMomentum(const rkBody *body, zVec3D *momentum);
__ROKI_EXPORT zVec3D *rkBodyAngularMomentum(const rkBody *body, const zVec3D *pos, zVec3D *am);
__ROKI_EXPORT double rkBodyKineticEnergy(const rkBody *body);

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
#define rkBodyShapePush(body,shape) zShapeListPush( rkBodyShapeList(body), shape )
#define rkBodyShapePop(body)        zShapeListPop( rkBodyShapeList(body) )
#define rkBodyShapeDestroy(body)    zShapeListDestroy( rkBodyShapeList(body) )

/*! \brief contiguous vertex of a body to a point.
 */
__ROKI_EXPORT zVec3D *rkBodyContigVert(rkBody *body, zVec3D *p, double *d);

/*! \brief compute volume of a body. */
__ROKI_EXPORT double rkBodyShapeVolume(rkBody *body);
/*! \brief compute mass property of a body. */
__ROKI_EXPORT rkMP *rkBodyShapeMP(rkBody *body, double density, rkMP *mp);

__END_DECLS

#ifdef __cplusplus
inline rkBody::rkBody(){ rkBodyInit( this ); }
inline rkBody::~rkBody(){ rkBodyDestroy( this ); }
#endif /* __cplusplus */

#endif /* __RK_BODY_H__ */
