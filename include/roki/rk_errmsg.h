/* RoKi - Robot Kinetics library
 * Copyright (C) 1998 Tomomichi Sugihara (Zhidao)
 *
 * rk_errmsg - error messages.
 */

#ifndef __RK_ERRMSG_H__
#define __RK_ERRMSG_H__

/* NOTE: never include this header file in user programs. */

/* warning message */

#define RK_WARN_TOOLNG_NAME      "too long name, truncated to %s"
#define RK_WARN_TWOFOLDNAME      "twofolded name %s exists"

#define RK_WARN_CONTACT_DUPKEY   "duplicate contact information with keys %s and %s"
#define RK_WARN_CONTACT_EMPTY    "empty set of contact information specified."

#define RK_WARN_LINK_NULL        "null link specified."
#define RK_WARN_LINK_DUP         "%s: name of a link duplicated."

#define RK_WARN_LINK_DUP_MASS_DENS  "duplicate specification of mass and density. mass specification ignored"
#define RK_WARN_LINK_NO_MASS_DENS   "neither mass nor density are specified"
#define RK_WARN_LINK_TOO_SMALL_VOL  "too small volume of a link specified"
#define RK_WARN_LINK_TOO_SMALL_DENS "too small density of a link specified"
#define RK_WARN_LINK_TOO_SMALL_MASS "too small mass of a link specified"
#define RK_WARN_LINK_EMPTY_SHAPE    "cannot compute mass properties of an empty shape"

#define RK_WARN_CHAIN_NULL         "null chain specified."
#define RK_WARN_CHAIN_EMPTY        "empty chain specified."
#define RK_WARN_CHAIN_SHAPE_ONLY   "create a floating link to assign shape(s)"

#define RK_WARN_IK_CONSTRAINT_ALREADY_REGISTERED "constraint %s already registered, replaced"

/* error messages */

#define RK_ERR_CONTACT_UNBOUND     "unbound contact info"

#define RK_ERR_MOTOR_UNKNOWN       "%s: unknown motor"
#define RK_ERR_MOTOR_UNKNOWNTYPE   "%s: unknown motor type"
#define RK_ERR_MOTOR_FAILED        "cannot create motor instance"
#define RK_ERR_MOTOR_OUTOFRANGE    "motor identifier out of range %d/%d"
#define RK_ERR_MOTOR_UNNAMED       "unnamed motor exists"

#define RK_ERR_JOINT_INVTYPE       "invalid joint type specified - %d"
#define RK_ERR_JOINT_FAILED        "cannot create joint instance"
#define RK_ERR_JOINT_DOFMISMATCH   "unmatched joint/motor DOFs %d/%d"

#define RK_ERR_MAT_VEC_SIZMISMATCH "unmatched matrix/vector size with joint size"

#define RK_ERR_CHAIN_INVSHAPE      "invalid model file"

#define RK_ERR_LINK_MANY           "too many links defined"
#define RK_ERR_LINK_INVDSC         "invalid description for a link"
#define RK_ERR_LINK_UNNAMED        "unnamed link exists"
#define RK_ERR_LINK_INVID          "invalid link #%d specified"
#define RK_ERR_LINK_UNKNOWN        "%s: unknown link"

#define RK_ERR_SHAPE_UNKNOWN       "%s: unknown shape"

#define RK_ERR_IK_CONSTRAINT_NOTFOUND   "constraint %s of the inverse kinematics not found"
#define RK_ERR_IK_CONSTRAINT_PREDEFINED "constraint %s of the inverse kinematics predefined"
#define RK_ERR_IK_CELL_IS_NULL     "null pointer specified for IK cell"
#define RK_ERR_IK_CELL_NOTFOUND    "IK cell %s not found"

#define RK_ERR_FATAL               "fatal error! - please report to the author"

#endif /* __RK_ERRMSG_H__ */
