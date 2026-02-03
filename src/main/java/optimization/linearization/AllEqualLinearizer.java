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
import constraints.global.AllEqual;
import variables.Variable;

/**
 * Linearizer for AllEqual constraints.
 *
 * AllEqual(x_0, x_1, ..., x_n) ensures all variables take the same value.
 * LP formulation: x_i = x_0 for all i > 0
 *
 * This is an exact linearization - no relaxation needed.
 */
public class AllEqualLinearizer implements ConstraintLinearizer {

    @Override
    public boolean canLinearize(Constraint c) {
        return c instanceof AllEqual;
    }

    @Override
    public boolean linearize(Constraint c, LinearizationContext ctx) {
        AllEqual ctr = (AllEqual) c;
        Variable[] scp = ctr.scp;

        if (scp.length < 2) {
            return true; // Trivially satisfied
        }

        // x_i = x_0 for all i > 0  =>  x_i - x_0 = 0
        Variable x0 = scp[0];
        for (int i = 1; i < scp.length; i++) {
            Variable xi = scp[i];
            Expression expr = ctx.addExpression("allEq_" + ctr.num + "_" + i);
            expr.set(ctx.getLpVar(xi), 1);
            expr.set(ctx.getLpVar(x0), -1);
            expr.level(0); // xi - x0 = 0
        }
        return true;
    }
}
