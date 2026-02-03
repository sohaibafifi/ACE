/*
 * This file is part of the constraint solver ACE (AbsCon Essence).
 *
 * Copyright (c) 2021. All rights reserved.
 * Christophe Lecoutre, CRIL, Univ. Artois and CNRS.
 *
 * Licensed under the MIT License.
 * See LICENSE file in the project root for full license information.
 */

package optimization.linearization;

import org.ojalgo.optimisation.Expression;

import constraints.Constraint;
import constraints.intension.Primitive2;
import constraints.intension.Primitive2.PrimitiveBinaryNoCst.*;
import constraints.intension.Primitive2.PrimitiveBinaryVariant1.Add2.*;
import constraints.intension.Primitive2.PrimitiveBinaryVariant1.Sub2.*;
import constraints.intension.Primitive2.PrimitiveBinaryVariant1.Dist2.*;
import constraints.intension.Primitive2.PrimitiveBinaryVariant2.Dist2b.Dist2bEQ;
import variables.Variable;

/**
 * Linearizer for binary Primitive (Intension) constraints.
 *
 * Many Intension constraints are inherently linear:
 * - Add2: x + y op k
 * - Sub2: x - y op k
 * - Neg2EQ: x = -y (i.e., x + y = 0)
 * - Or2: x OR y for binary variables (x + y >= 1)
 * - Dist2: |x - y| op k (split into two linear constraints)
 *
 * This linearizer captures these cases for an exact or near-exact LP relaxation.
 */
public class PrimitiveLinearizer implements ConstraintLinearizer {

    @Override
    public boolean canLinearize(Constraint c) {
        return c instanceof Add2LE
            || c instanceof Add2GE
            || c instanceof Add2EQ
            || c instanceof Sub2LE
            || c instanceof Sub2GE
            || c instanceof Sub2EQ
            || c instanceof Neg2EQ
            || c instanceof Or2
            || c instanceof Dist2LE
            || c instanceof Dist2GE
            || c instanceof Dist2EQ
            || c instanceof Dist2bEQ
            || c instanceof Disjonctive;
    }

    @Override
    public boolean linearize(Constraint c, LinearizationContext ctx) {
        if (c instanceof Add2LE) {
            return addAdd2LE((Primitive2) c, ctx);
        } else if (c instanceof Add2GE) {
            return addAdd2GE((Primitive2) c, ctx);
        } else if (c instanceof Add2EQ) {
            return addAdd2EQ((Primitive2) c, ctx);
        } else if (c instanceof Sub2LE) {
            return addSub2LE((Primitive2) c, ctx);
        } else if (c instanceof Sub2GE) {
            return addSub2GE((Primitive2) c, ctx);
        } else if (c instanceof Sub2EQ) {
            return addSub2EQ((Primitive2) c, ctx);
        } else if (c instanceof Neg2EQ) {
            return addNeg2EQ((Primitive2) c, ctx);
        } else if (c instanceof Or2) {
            return addOr2((Primitive2) c, ctx);
        } else if (c instanceof Dist2LE) {
            return addDist2LE((Primitive2) c, ctx);
        } else if (c instanceof Dist2GE) {
            return addDist2GE((Primitive2) c, ctx);
        } else if (c instanceof Dist2EQ) {
            return addDist2EQ((Primitive2) c, ctx);
        } else if (c instanceof Dist2bEQ) {
            return addDist2bEQ((Primitive2) c, ctx);
        } else if (c instanceof Disjonctive) {
            return addDisjonctive((Disjonctive) c, ctx);
        }
        return false;
    }

    // Helper to get k from Primitive2 via reflection
    private int getK(Primitive2 p) {
        try {
            java.lang.reflect.Field kField = Primitive2.class.getDeclaredField("k");
            kField.setAccessible(true);
            return (int) kField.get(p);
        } catch (Exception e) {
            return 0;
        }
    }

    /**
     * x + y <= k
     */
    private boolean addAdd2LE(Primitive2 ctr, LinearizationContext ctx) {
        Variable x = ctr.scp[0];
        Variable y = ctr.scp[1];
        int k = getK(ctr);

        Expression expr = ctx.addExpression("add2LE_" + ctr.num);
        expr.set(ctx.getLpVar(x), 1);
        expr.set(ctx.getLpVar(y), 1);
        expr.upper(k);
        return true;
    }

    /**
     * x + y >= k
     */
    private boolean addAdd2GE(Primitive2 ctr, LinearizationContext ctx) {
        Variable x = ctr.scp[0];
        Variable y = ctr.scp[1];
        int k = getK(ctr);

        Expression expr = ctx.addExpression("add2GE_" + ctr.num);
        expr.set(ctx.getLpVar(x), 1);
        expr.set(ctx.getLpVar(y), 1);
        expr.lower(k);
        return true;
    }

    /**
     * x + y = k
     */
    private boolean addAdd2EQ(Primitive2 ctr, LinearizationContext ctx) {
        Variable x = ctr.scp[0];
        Variable y = ctr.scp[1];
        int k = getK(ctr);

        Expression expr = ctx.addExpression("add2EQ_" + ctr.num);
        expr.set(ctx.getLpVar(x), 1);
        expr.set(ctx.getLpVar(y), 1);
        expr.level(k);
        return true;
    }

    /**
     * x - y <= k
     */
    private boolean addSub2LE(Primitive2 ctr, LinearizationContext ctx) {
        Variable x = ctr.scp[0];
        Variable y = ctr.scp[1];
        int k = getK(ctr);

        Expression expr = ctx.addExpression("sub2LE_" + ctr.num);
        expr.set(ctx.getLpVar(x), 1);
        expr.set(ctx.getLpVar(y), -1);
        expr.upper(k);
        return true;
    }

    /**
     * x - y >= k
     */
    private boolean addSub2GE(Primitive2 ctr, LinearizationContext ctx) {
        Variable x = ctr.scp[0];
        Variable y = ctr.scp[1];
        int k = getK(ctr);

        Expression expr = ctx.addExpression("sub2GE_" + ctr.num);
        expr.set(ctx.getLpVar(x), 1);
        expr.set(ctx.getLpVar(y), -1);
        expr.lower(k);
        return true;
    }

    /**
     * x - y = k
     */
    private boolean addSub2EQ(Primitive2 ctr, LinearizationContext ctx) {
        Variable x = ctr.scp[0];
        Variable y = ctr.scp[1];
        int k = getK(ctr);

        Expression expr = ctx.addExpression("sub2EQ_" + ctr.num);
        expr.set(ctx.getLpVar(x), 1);
        expr.set(ctx.getLpVar(y), -1);
        expr.level(k);
        return true;
    }

    /**
     * x = -y  =>  x + y = 0
     */
    private boolean addNeg2EQ(Primitive2 ctr, LinearizationContext ctx) {
        Variable x = ctr.scp[0];
        Variable y = ctr.scp[1];

        Expression expr = ctx.addExpression("neg2EQ_" + ctr.num);
        expr.set(ctx.getLpVar(x), 1);
        expr.set(ctx.getLpVar(y), 1);
        expr.level(0);
        return true;
    }

    /**
     * x OR y (binary 0/1 variables)  =>  x + y >= 1
     */
    private boolean addOr2(Primitive2 ctr, LinearizationContext ctx) {
        Variable x = ctr.scp[0];
        Variable y = ctr.scp[1];

        Expression expr = ctx.addExpression("or2_" + ctr.num);
        expr.set(ctx.getLpVar(x), 1);
        expr.set(ctx.getLpVar(y), 1);
        expr.lower(1);
        return true;
    }

    /**
     * |x - y| <= k  =>  -k <= x - y <= k
     * Split into: x - y <= k AND y - x <= k
     */
    private boolean addDist2LE(Primitive2 ctr, LinearizationContext ctx) {
        Variable x = ctr.scp[0];
        Variable y = ctr.scp[1];
        int k = getK(ctr);

        // x - y <= k
        Expression expr1 = ctx.addExpression("dist2LE_pos_" + ctr.num);
        expr1.set(ctx.getLpVar(x), 1);
        expr1.set(ctx.getLpVar(y), -1);
        expr1.upper(k);

        // y - x <= k  =>  x - y >= -k
        Expression expr2 = ctx.addExpression("dist2LE_neg_" + ctr.num);
        expr2.set(ctx.getLpVar(x), 1);
        expr2.set(ctx.getLpVar(y), -1);
        expr2.lower(-k);

        return true;
    }

    /**
     * |x - y| >= k
     * This is disjunctive: (x - y >= k) OR (y - x >= k)
     * Uses big-M relaxation with auxiliary binary.
     */
    private boolean addDist2GE(Primitive2 ctr, LinearizationContext ctx) {
        Variable x = ctr.scp[0];
        Variable y = ctr.scp[1];
        int k = getK(ctr);

        // Compute big-M values
        double Mpos = k - (x.dom.firstValue() - y.dom.lastValue()); // for x - y >= k
        double Mneg = k - (y.dom.firstValue() - x.dom.lastValue()); // for y - x >= k

        if (Mpos <= 0 || Mneg <= 0) {
            // One branch is always satisfied
            return true;
        }

        // Create auxiliary binary b: b=1 means "x - y >= k" is the active branch
        org.ojalgo.optimisation.Variable b = org.ojalgo.optimisation.Variable
            .make("dist2GE_" + ctr.num + "_b").lower(0).upper(1);
        ctx.addVariable(b);

        // x - y >= k - M*(1-b)  =>  x - y + M*b >= k - M + M = k
        // Actually: x - y >= k when b=1, relaxed when b=0
        // x - y - M*(1-b) >= k - M  =>  x - y + M*b >= k
        Expression expr1 = ctx.addExpression("dist2GE_pos_" + ctr.num);
        expr1.set(ctx.getLpVar(x), 1);
        expr1.set(ctx.getLpVar(y), -1);
        expr1.set(b, Mpos);
        expr1.lower(k);

        // y - x >= k when b=0, i.e., y - x + M*b >= k
        // Actually we need: y - x >= k - M*b
        // y - x + M*b >= k
        Expression expr2 = ctx.addExpression("dist2GE_neg_" + ctr.num);
        expr2.set(ctx.getLpVar(y), 1);
        expr2.set(ctx.getLpVar(x), -1);
        expr2.set(b, -Mneg);
        expr2.lower(k - Mneg);

        return true;
    }

    /**
     * |x - y| = k  =>  (x - y = k) OR (x - y = -k)
     * LP relaxation: -k <= x - y <= k (valid but weak)
     */
    private boolean addDist2EQ(Primitive2 ctr, LinearizationContext ctx) {
        // Use the LE relaxation as a valid approximation
        return addDist2LE(ctr, ctx);
    }

    /**
     * x = |y - k|: means x >= 0 and (x = y - k OR x = k - y)
     * LP relaxation: x >= y - k AND x >= k - y (equivalent to x >= |y - k|)
     */
    private boolean addDist2bEQ(Primitive2 ctr, LinearizationContext ctx) {
        Variable x = ctr.scp[0];
        Variable y = ctr.scp[1];
        int k = getK(ctr);

        // x >= y - k  =>  x - y >= -k
        Expression expr1 = ctx.addExpression("dist2bEQ_pos_" + ctr.num);
        expr1.set(ctx.getLpVar(x), 1);
        expr1.set(ctx.getLpVar(y), -1);
        expr1.lower(-k);

        // x >= k - y  =>  x + y >= k
        Expression expr2 = ctx.addExpression("dist2bEQ_neg_" + ctr.num);
        expr2.set(ctx.getLpVar(x), 1);
        expr2.set(ctx.getLpVar(y), 1);
        expr2.lower(k);

        return true;
    }

    /**
     * Disjunctive constraint: (x + wx <= y) OR (y + wy <= x)
     * Uses big-M formulation.
     */
    private boolean addDisjonctive(Disjonctive ctr, LinearizationContext ctx) {
        Variable x = ctr.scp[0];
        Variable y = ctr.scp[1];

        // Get wx, wy via reflection
        int wx, wy;
        try {
            java.lang.reflect.Field wxField = Disjonctive.class.getDeclaredField("wx");
            java.lang.reflect.Field wyField = Disjonctive.class.getDeclaredField("wy");
            wxField.setAccessible(true);
            wyField.setAccessible(true);
            wx = (int) wxField.get(ctr);
            wy = (int) wyField.get(ctr);
        } catch (Exception e) {
            return false;
        }

        // Big-M values
        double M1 = x.dom.lastValue() + wx - y.dom.firstValue();
        double M2 = y.dom.lastValue() + wy - x.dom.firstValue();

        // Create auxiliary binary b: b=1 means x + wx <= y
        org.ojalgo.optimisation.Variable b = org.ojalgo.optimisation.Variable
            .make("disj_" + ctr.num + "_b").lower(0).upper(1);
        ctx.addVariable(b);

        // x + wx <= y + M*(1-b)  =>  x - y <= -wx + M - M*b  =>  x - y + M*b <= M - wx
        Expression expr1 = ctx.addExpression("disj_xy_" + ctr.num);
        expr1.set(ctx.getLpVar(x), 1);
        expr1.set(ctx.getLpVar(y), -1);
        expr1.set(b, M1);
        expr1.upper(M1 - wx);

        // y + wy <= x + M*b  =>  y - x <= -wy + M*b  =>  y - x - M*b <= -wy
        Expression expr2 = ctx.addExpression("disj_yx_" + ctr.num);
        expr2.set(ctx.getLpVar(y), 1);
        expr2.set(ctx.getLpVar(x), -1);
        expr2.set(b, -M2);
        expr2.upper(-wy);

        return true;
    }
}
