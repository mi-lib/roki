/* RoKi - Robot Kinetics library
 * Copyright (C) 1998 Tomomichi Sugihara (Zhidao)
 *
 * rk_contact - physical properties of contact
 */

#ifndef __RK_CONTACT_H__
#define __RK_CONTACT_H__

#include <zeda/zeda.h>
#include <roki/rk_errmsg.h>

__BEGIN_DECLS

/* to be varied. */
typedef enum{ RK_CONTACT_RIGID, RK_CONTACT_ELASTIC } rkContactType;

typedef enum{ RK_CONTACT_UNCERTAIN, RK_CONTACT_SF, RK_CONTACT_KF } rkContactFricType;

/* ********************************************************** */
/* CLASS: rkContactInfo
 * contact model class
 * ********************************************************** */

typedef struct{
  char *__stf[2];        /* stuff binding keys */
  rkContactType type;    /* contact type (elastic/rigid model) */
  union{
    struct{
      double compensation; /* compensation coefficient */
      double relaxation;   /* relaxation coefficient   */
    } rigid_param;
    struct{
      double elasticity; /* elasticity coefficient */
      double viscosity;  /* viscosity coefficient */
    } elastic_param;
  } param;
  double static_fc;      /* static friction coefficient */
  double kinetic_fc;     /* kinetic friction coefficient */
} rkContactInfo;

#define rkContactInfoType(c)      (c)->type
#define rkContactInfoK(c)         (c)->param.rigid_param.compensation
#define rkContactInfoL(c)         (c)->param.rigid_param.relaxation
#define rkContactInfoSF(c)        (c)->static_fc
#define rkContactInfoKF(c)        (c)->kinetic_fc
#define rkContactInfoE(c)         (c)->param.elastic_param.elasticity
#define rkContactInfoV(c)         (c)->param.elastic_param.viscosity

#define rkContactInfoSetType(c,t) ( rkContactInfoType(c) = (t) )
#define rkContactInfoSetK(c,k)    ( rkContactInfoK(c) = (k) )
#define rkContactInfoSetL(c,l)    ( rkContactInfoL(c) = (l) )
#define rkContactInfoSetSF(c,f)   ( rkContactInfoSF(c) = (f) )
#define rkContactInfoSetKF(c,f)   ( rkContactInfoKF(c) = (f) )
#define rkContactInfoSetE(c,e)    ( rkContactInfoE(c) = (e) )
#define rkContactInfoSetV(c,v)    ( rkContactInfoV(c) = (v) )

/*! \brief initialize, create and destroy contact model.
 *
 * rkContactInfoInit() sets all parameter of \a ci for 0.
 *
 * rkContactInfoRigidCreate() creates the contact model \a ci as
 * a rigid contact model. \a sf is a static friction coefficient,
 * \a kf is a kinetic friction coefficient and \a r is a rebound
 * coefficient.
 * rkContactInfoElasticCreate() creates the contact model \a ci
 * as an elastic contact model. \a e is a elasticity coefficient,
 * \a v is a viscosity coefficient and \a kf is a kinetic friction
 * coefficient.
 * For both of these functions, \a stf1 and \a stf2 are the key
 * strings to associate \a ci to a pair of stuff.
 *
 * rkContactInfoDestroy() destroys the instance \a ci.
 * \return
 * rkContactInfoInit(), rkContactInfoRigidCreate() and
 * rkContactInfoElasticCreate() return a pointer to \a ci.
 *
 * rkContactInfoDestroy() returns no value.
 */
#define rkContactInfoInit(c) memset( (c), 0, sizeof(rkContactInfo) )
__EXPORT rkContactInfo *rkContactInfoRigidCreate(rkContactInfo *ci, double k, double l, double sf, double kf, char *stf1, char *stf2);
__EXPORT rkContactInfo *rkContactInfoElasticCreate(rkContactInfo *ci, double e, double v, double sf, double kf, char *stf1, char *stf2);
#define rkContactInfoDestroy(c) do{\
  zFree( (c)->__stf[0] );\
  zFree( (c)->__stf[1] );\
  rkContactInfoInit(c);\
} while(0)

/*! \brief associate contact info to a pair of keys
 *
 * rkContactInfoAssoc() checks if the contact information set
 * \a ci is associated by a pair of keys \a stf1 and \a stf2.
 * \return
 * If \a ci matches the pair of \a stf1 and \a stf2, it returns
 * \a ci. Otherwise, the null pointer is returned.
 */
__EXPORT rkContactInfo *rkContactInfoAssoc(rkContactInfo *ci, char *stf1, char *stf2);

#define ZTK_TAG_RKCONTACTINFO "contact"

/*! \brief print information of a contact model out to the current position of a ZTK file.
 *
 * rkContactInfoFPrint() prints information of a contact
 * model \a ci out to the current position of a file \a fp.
 * An acceptable data file format is as follows.
 *
 *  staticfriction   <value>  <- static friction coefficient
 *  kineticfriction  <value>  <- kinetic friction coefficient
 *  rebound          <value>  <- rebound coefficient
 *  elasticity       <value>  <- elasticity coefficient
 *  viscosity        <value>  <- viscosity coefficient
 *
 * rebound-staticfriction group and elasticity-viscosity
 * group are exclusive with each other. When the function finds
 * one keyword in the former group, the contact type is
 * automatically set for 'rigid', while one in the latter is
 * found, the type is set for 'elastic'.
 * If any inconsistency exists, the last found keyword has
 * priority. kineticfriction is common for the two groups.
 * \return
 * rkContactInfoFPrintZTK() returns no value.
 */
__EXPORT void rkContactInfoFPrintZTK(FILE *fp, rkContactInfo *ci);

/* ********************************************************** */
/* CLASS: rkContactInfoArray
 * array class of contact information set
 * ********************************************************** */

zArrayClass( rkContactInfoArray, rkContactInfo );

/*! \brief destroy contact info array.
 *
 * rkContactInfoArrayDestroy() destroys the internal objects and
 * array of the contact information array \a ci.
 * \return
 * rkContactInfoArrayDestroy() returns no value.
 */
__EXPORT void rkContactInfoArrayDestroy(rkContactInfoArray *ci);

/*! \brief associate contact info to a pair of keys
 *
 * rkContactInfoArrayAssoc() finds the contact information set from
 * the information array \a ci by a pair of keys \a stf1 and \a stf2.
 * \return
 * The contact information set with which the pair of \a stf1 and
 * \a stf2 is associated is returned. Unless any of the information
 * set in \a ci matches the pair, the null pointer is returned.
 */
__EXPORT rkContactInfo *rkContactInfoArrayAssoc(rkContactInfoArray *ci, char *stf1, char *stf2);
__EXPORT rkContactInfo *rkContactInfoArrayAssocType(rkContactInfoArray *ci, char *stf1, char *stf2, char type);

/* ZTK */

/* scan contact information array from a ZTK format processor. */
__EXPORT rkContactInfoArray *rkContactInfoArrayFromZTK(rkContactInfoArray *carray, ZTK *ztk);
/* print contact information array out to the current position of a ZTK format file. */
__EXPORT void rkContactInfoArrayFPrintZTK(FILE *fp, rkContactInfoArray *ci);

/* read contact information array from a ZTK file. */
__EXPORT rkContactInfoArray *rkContactInfoArrayReadZTK(rkContactInfoArray *carray, char filename[]);
/* write contact information array to a ZTK file. */
__EXPORT bool rkContactInfoArrayWriteZTK(rkContactInfoArray *ci, char filename[]);

__END_DECLS

#endif /* __RK_CONTACT_H__ */
