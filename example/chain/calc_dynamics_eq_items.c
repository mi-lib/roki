#include <roki/roki.h>

zMat zMatFromzMat3D(zMat m, zMat3D *m3)
{
    register int r, c;
    if (zMatColSize(m) != 3)
        return NULL;
    if (zMatRowSize(m) != 3)
        return NULL;
    for (r = 0; r < 3; r++)
        for (c = 0; c < 3; c++)
            zMatSetElem(m, r, c, m3->e[c][r]);
    return m;
}

/* inertia matrix */
zMat rkChainCalcInertiaMatrix(rkChain *chain, zMat inertia)
{
    register int i;
    zMat tmpInertia = zMatAlloc(rkChainJointSize(chain), rkChainJointSize(chain));
    zMat jacobi = zMatAlloc(3, rkChainJointSize(chain));
    zMat tmpJacobi = zMatAlloc(3, rkChainJointSize(chain));
    zMat jacobiT = zMatAlloc(rkChainJointSize(chain), 3); /* transposed jacobian */
    zMat mp = zMatAlloc(3, 3);                            /* mass property */
    zMat3D mp3d;                                          /* mass property as zMat3D */

    /* initialize */
    zMatZero(inertia);
    for (i = 0; i < rkChainLinkNum(chain); i++)
    {
        /* calculate linear component */
        rkChainLinkWldLinJacobi(chain, i, rkChainLinkCOM(chain, i), jacobi);
        zMatT(jacobi, jacobiT);
        zMatIdent(mp);
        zMatMulDRC(mp, rkChainLinkMass(chain, i));
        zMulMatMat(mp, jacobi, tmpJacobi);
        zMulMatMat(jacobiT, tmpJacobi, tmpInertia);
        zMatAddDRC(inertia, tmpInertia);
        /* calculate angular component */
        rkChainLinkWldAngJacobi(chain, i, jacobi);
        zMatT(jacobi, jacobiT);
        rkLinkWldInertia(rkChainLink(chain, i), &mp3d);
        zMatFromzMat3D(mp, &mp3d);
        zMulMatMat(mp, jacobi, tmpJacobi);
        zMulMatMat(jacobiT, tmpJacobi, tmpInertia);
        zMatAddDRC(inertia, tmpInertia);
    }
    zMatFreeAO(5, tmpInertia, jacobi, tmpJacobi, jacobiT, mp);
    return inertia;
}

/* solve inverse dynamics of a kinematic chain. */
zVec rkChainCalcBias(rkChain *c, zVec dis, zVec vel, zVec bias)
{
    zVec zero = zVecAlloc(rkChainJointSize(c));
    zVecZero(zero);
    rkChainFK(c, dis);
    rkChainSetJointRateAll(c, vel, zero);
    rkChainUpdateID(c);
    zVecFree(zero);
    return rkChainGetJointTrqAll(c, bias);
}

int main()
{
    rkChain chain;
    bool isSucceeded = true;
    rkChainReadZTK(&chain, "../model/mighty.ztk");
    zMat inertia = zMatAlloc(rkChainJointSize(&chain), rkChainJointSize(&chain));
    zVec dis = zVecAlloc(rkChainJointSize(&chain));
    zVec vel = zVecAlloc(rkChainJointSize(&chain));
    zVec bias = zVecAlloc(rkChainJointSize(&chain));
    zVec tmp = zVecAlloc(rkChainJointSize(&chain));

    /* create random state */
    zVec min = zVecAlloc(rkChainJointSize(&chain));
    zVec max = zVecAlloc(rkChainJointSize(&chain));
    zVecSetAll(min, -10.0);
    zVecSetAll(max, 10.0);
    zVecRand(dis, min, max);
    zVecRand(vel, min, max);
    rkChainFK(&chain, dis);
    rkChainSetJointVelAll(&chain, vel);
    rkChainUpdateVel(&chain);

    /* calc dynamics equation items*/
    rkChainCalcInertiaMatrix(&chain, inertia);
    rkChainCalcBias(&chain, dis, vel, bias);

    zMulMatVec(inertia, vel, tmp);
    double actual = zVecInnerProd(tmp, vel) * 0.5;
    double expected = rkChainKE(&chain);
    if (!zIsTol(actual - expected, zTOL))
    {
        isSucceeded = false;
    }
    if (isSucceeded)
    {
        printf("calc inertia matrix: succeeded!\n");
    }
    else
    {
        printf("failed...\n");
    }

    /* calc trq */
    zVec acc = zVecAlloc(rkChainJointSize(&chain));
    zVec idTrq = zVecAlloc(rkChainJointSize(&chain));
    zVec eqTrq = zVecAlloc(rkChainJointSize(&chain)); // calc from inertia matrix and bias
    rkChainID(&chain, vel, acc);
    rkChainGetJointTrqAll(&chain, idTrq);
    zMulMatVec(inertia, acc, eqTrq);
    zVecAddDRC(eqTrq, bias);
    zVecPrint(eqTrq);
    zVecPrint(idTrq);
    int i;
    for (i = 0; i < zVecSize(idTrq); i++)
    {
        if (!zIsTol(zVecElem(idTrq, i) - zVecElem(eqTrq, i), zTOL))
        {
            isSucceeded = false;
        }
    }
    if (isSucceeded)
    {
        printf("calc bias: succeeded!\n");
    }
    else
    {
        printf("failed...\n");
    }

    zMatFree(inertia);
    zVecFreeAO(9, dis, vel, bias, tmp, min, max, acc, idTrq, eqTrq);
    rkChainDestroy(&chain);

    return 0;
}