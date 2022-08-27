#include <roki/roki.h>

/* this sample will succeed when the model have trq controled motors */
int main()
{
    rkChain chain;
    
    rkChainReadZTK(&chain, "../model/arm_2DoF_trq.ztk");
    rkChainABIAlloc(&chain);
    zVec dis = zVecAlloc(rkChainJointSize(&chain));
    zVec vel = zVecAlloc(rkChainJointSize(&chain));
    zVec acc = zVecAlloc(rkChainJointSize(&chain));
    zVec expected = zVecAlloc(rkChainJointSize(&chain));
    zVec actual = zVecAlloc(rkChainJointSize(&chain));
    zVec err = zVecAlloc(rkChainJointSize(&chain));
    zVecRandUniform(dis, 10, -10);
    zVecRandUniform(vel, 10, -10);
    zVecRandUniform(expected, 10, -10);

    /* FD */
    /* joint accelaration calculated by abi method */
    rkChainSetJointMotorSetInputAll(&chain, expected);
    zVecPrint(rkChainGetJointTrqAll(&chain, actual));
    rkChainABI(&chain, dis, vel, acc);

    /* ID */
    /* joint trq calculated by inverse dynamics */
    rkChainFK(&chain, dis);
    rkChainID(&chain, vel, acc);
    rkChainGetJointTrqAll(&chain, actual);

    zVecSub(actual, expected, err);
    if (zVecIsTiny(err))
        printf("success\n");
    else
        printf("failed\n");

    rkChainABIDestroy(&chain);
    rkChainDestroy(&chain);

    return 0;
}