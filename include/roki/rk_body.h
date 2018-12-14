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

typedef struct _rkMP{
  /* mass property */
  double mass;
  zVec3D com;
  zMat3D inertia;
} rkMP;

#define rkMPMass(mp)           (mp)->mass
#define rkMPCOM(mp)            ( &(mp)->com )
#define rkMPInertia(mp)        ( &(mp)->inertia )

#define rkMPCOMElem(mp,i)       rkMPCOM(mp)->e[i]
#define rkMPInertiaElem(mp,i,j) rkMPInertia(mp)->e[j][i]

#define rkMPSetMass(mp,m)      ( rkMPMass(mp) = (m) )
#define rkMPSetCOM(mp,c)       zVec3DCopy( c, rkMPCOM(mp) )
#define rkMPSetInertia(mp,i)   zMat3DCopy( i, rkMPInertia(mp) )

#define rkMPCopy(src,dst)      ( *(dst) = *(src) )

/*! \brief transfer a mass property set to that with respect to a frame.
 *
 * rkMPXfer() transfers a mass property set \a src to that
 * with respect to a given frame \a f.
 * The mass of \a dest will be the same with that of \a src.
 * The COM of \a src will be transfered to that of \a dest
 * by \a f.
 * The inertia tensor of \a src will be rotated to that of
 * \a dest with respect to the attitude of \a f.
 * \return \a dest
 */
__EXPORT rkMP *rkMPXfer(rkMP *src, zFrame3D *f, rkMP *dest);

/*! \brief combine two mass property sets in the same frame.
 */
__EXPORT rkMP *rkMPCombine(rkMP *mp1, rkMP *mp2, rkMP *mp);

/* \brief convert inertia tensor to that about the origin.
 */
__EXPORT zMat3D *rkMPOrgInertia(rkMP *mp, zMat3D *i);

/* \brief compute the inertial ellipsoid from a mass property set.
 */
__EXPORT zEllips3D *rkMPInertiaEllips(rkMP *mp, zEllips3D *ie);

/*! \brief output mass property.
 *
 * rkMPFWrite() outputs a set of mass properties \a mp to the current
 * position of a file \a fp in the following format.
 *
 *  mass: <m>
 *  COM:  { <x>, <y>, <z> }
 *  inertia: {
 *    <ixx>, <ixy>, <izx>,
 *    <iyx>, <iyy>, <iyz>,
 *    <izx>, <iyz>, <izz>
 *  }
 *
 * rkMPWrite() outputs a set of mass properties \a mp to the standard
 * output in the same format with the above.
 * \return
 * rkMPFWrite() and rkMPWrite() return no value.
 */
__EXPORT void rkMPFWrite(FILE *fp, rkMP *mp);
#define rkMPWrite(mp) rkMPFWrite( stdout, mp )

/* ********************************************************** */
/* CLASS: rkBody
 * rigid body class
 * ********************************************************** */

typedef struct{
  rkMP mp;           /*!< \brief mass property */
  zFrame3D frame;    /*!< \brief absolute transformation */
  zVec6D vel;        /*!< \brief velocity */
  zVec6D acc;        /*!< \brief acceleration */
  zVec3D com;        /*!< \brief center of mass (COM) */
  zVec3D comvel;     /*!< \brief COM velocity */
  zVec3D comacc;     /*!< \brief COM acceleration */
  rkWrenchList extw; /*!< \brief external wrench with respect to body frame */
  zShapeList shapelist; /*!< \brief shapes */
  char *stuff;          /*!< \brief stuff identifier */
} rkBody;

#define rkBodyMP(b)           ( &(b)->mp )
#define rkBodyMass(b)         rkMPMass( &(b)->mp )
#define rkBodyCOM(b)          rkMPCOM( &(b)->mp )
#define rkBodyInertia(b)      rkMPInertia( &(b)->mp )
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
#define rkBodyShapeNum(b)     zListNum( rkBodyShapeList(b) )
#define rkBodyShapeIsEmpty(b) zListIsEmpty( rkBodyShapeList(b) )

#define rkBodySetMass(b,m)    rkMPSetMass( &(b)->mp, m )
#define rkBodySetCOM(b,c)     rkMPSetCOM( &(b)->mp, c )
#define rkBodySetInertia(b,i) rkMPSetInertia( &(b)->mp, i )
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
 * rkBodyInit() initializes a body object \a b by cleaning up all
 * internal properties.
 *
 * rkBodyDestroy() destroys a body object \a b by freeing the memory
 * space allocated for its name and external forces.
 * \return
 * rkBodyInit() and rkBodyDestroy() return no value.
 */
__EXPORT void rkBodyInit(rkBody *b);
__EXPORT void rkBodyDestroy(rkBody *b);

__EXPORT rkBody *rkBodyClone(rkBody *org, rkBody *cln, zMShape3D *so, zMShape3D *sc);

__EXPORT rkBody *rkBodyCopyState(rkBody *src, rkBody *dst);

/*! \brief combine two bodies.
 *
 * rkBodyCombine() combines mass properties of the two bodies
 * \a b1 and \a b2 to one body \a b which is denoted in a frame
 * \a f.
 * \return b
 */
__EXPORT rkBody *rkBodyCombine(rkBody *b1, rkBody *b2, zFrame3D *f, rkBody *b);

/*! \brief combine a body directly to another.
 *
 * rkBodyCombineDRC() combines mass properties of a given body
 * \a sb directly to another \a b.
 * \return b
 */
__EXPORT rkBody *rkBodyCombineDRC(rkBody *b, rkBody *sb);

/* \brief compute the inertial ellipsoid from a rigid body.
 */
#define rkBodyInertiaEllips(b,e) rkMPInertiaEllips( rkBodyMP(b), e )

/*! \brief update body COM state.
 *
 * rkBodyUpdateCOM() updates the COM position of a body \a body by
 * transferring the local COM position of \a body to that with respect
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
__EXPORT zVec3D *rkBodyUpdateCOM(rkBody *body);
__EXPORT void rkBodyUpdateCOMVel(rkBody *body);
__EXPORT void rkBodyUpdateCOMAcc(rkBody *body);
__EXPORT void rkBodyUpdateCOMRate(rkBody *body);

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

/*! \brief calculate total 6D external force acting to a body.
 *
 * rkBodyCalcExtForce6D() calculates the total 6D external force
 * equivalent to the sumation of forces contained in the force list on
 * a body object \a b. The result is put into \a f.
 * \return
 * rkBodyCalcExtForce6D() returns a pointer \a f.
 */
#define rkBodyCalcExtWrench(b,f) rkWrenchListNet( rkBodyExtWrench(b), f )

/*! \brief net wrench exerted on body.
 *
 * rkBodyNetForce() computes the net six-axis force applied to a body object
 * \a body based on Newton-Euler's equation of motion.
 * \notes
 * It is assumed that the orientation of absolute velocity and acceleration
 * of the body \a body is all with respect to the local frame of \a body
 * itself. Consequently, the result net force-moment is also with respect
 * to the local frame in terms of orientation.
 */
__EXPORT zVec6D *rkBodyNetWrench(rkBody *body, zVec6D *w);

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
__EXPORT zVec3D *rkBodyAM(rkBody *b, zVec3D *p, zVec3D *am);
__EXPORT double rkBodyKE(rkBody *b);

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
__EXPORT zVec3D *rkBodyContigVert(rkBody *body, zVec3D *p, double *d);

#define RK_BODY_TAG "body"

__END_DECLS

#endif /* __RK_BODY_H__ */
