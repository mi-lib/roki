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

#define RK_WARN_IK_CELL_NOTFOUND "IK cell not found"

/* error messages */

#define RK_ERR_CONTACT_UNBOUND   "unbound contact info"

#define RK_ERR_MOTOR_UNKNOWN     "motor %s not found"
#define RK_ERR_MOTOR_INVTYPE     "invalid motor type specified - %d"
#define RK_ERR_MOTOR_FAILED      "cannot create motor instance"
#define RK_ERR_MOTOR_OUTOFRANGE  "motor identifier out of range %d/%d"
#define RK_ERR_MOTOR_UNNAMED     "unnamed motor exists"

#define RK_ERR_JOINT_INVTYPE     "invalid joint type specified - %d"
#define RK_ERR_JOINT_FAILED      "cannot create joint instance"
#define RK_ERR_JOINT_SIZMISMATCH "unmatched motor size"

#define RK_ERR_CHAIN_INVSHAPE    "invalid model file"

#define RK_ERR_LINK_MANY         "too many links defined"
#define RK_ERR_LINK_INVDSC       "invalid description for a link"
#define RK_ERR_LINK_UNNAMED      "unnamed link exists"

#define RK_ERR_LINK_INVID        "invalid link #%d specified"
#define RK_ERR_LINK_UNKNOWN      "link %s not found"
#define RK_ERR_SHAPE_UNKNOWN     "shape %s not found"

#define RK_ERR_IK_UNKNOWN        "unknown constraint type %s"

#define RK_ERR_FATAL             "fatal error! - please report to the author"

#endif /* __RK_ERRMSG_H__ */
