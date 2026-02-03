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
import org.ojalgo.optimisation.Variable;

import constraints.Constraint;
import constraints.global.Count;
import variables.Domain;

/**
 * Linearizer for Count constraints (ExactlyK, AtMostK, AtLeastK, etc.).
 *
 * For binary {0,1} variables counting value 0 or 1:
 * - Direct linearization as sum constraints
 *
 * For general integer variables:
 * - Uses reification with auxiliary binary variables to encode (x_i = value)
 */
public class CountLinearizer implements ConstraintLinearizer {

    @Override
    public boolean canLinearize(Constraint c) {
        return c instanceof Count.CountCst.ExactlyK
            || c instanceof Count.CountCst.AtMostK
            || c instanceof Count.CountCst.AtMost1
            || c instanceof Count.CountCst.AtLeastK
            || c instanceof Count.CountCst.AtLeast1;
    }

    @Override
    public boolean linearize(Constraint c, LinearizationContext ctx) {
        if (c instanceof Count.CountCst.ExactlyK) {
            return addCountConstraint((Count.CountCst) c, "==", ctx);
        } else if (c instanceof Count.CountCst.AtMostK || c instanceof Count.CountCst.AtMost1) {
            return addCountConstraint((Count.CountCst) c, "<=", ctx);
        } else if (c instanceof Count.CountCst.AtLeastK || c instanceof Count.CountCst.AtLeast1) {
            return addCountConstraint((Count.CountCst) c, ">=", ctx);
        }
        return false;
    }

    /**
     * Adds a Count constraint (ExactlyK, AtMostK, AtLeastK) to the LP model.
     *
     * For binary variables {0,1} counting value 0 or 1:
     * - value=1: sum(x_i) op k
     * - value=0: sum(x_i) op (n-k), with flipped inequality
     *
     * For general variables, uses big-M reification:
     * - Introduces auxiliary binaries b_lt, b_eq, b_gt for each variable
     * - b_lt + b_eq + b_gt = 1 (exactly one region)
     * - LP constraints ensure b_eq=1 only when x_i = value
     * - Count constraint: sum(b_eq_i) op k
     *
     * @param countCtr the count constraint
     * @param op the operator ("==", "<=", or ">=")
     * @return true if constraint was added
     */
    private boolean addCountConstraint(Count.CountCst countCtr, String op, LinearizationContext ctx) {
        variables.Variable[] list = countCtr.scp;
        int value = countCtr.getValue();
        int k = countCtr.getK();
        int n = list.length;

        // Check if all variables are binary {0,1}
        boolean allBinary = true;
        for (variables.Variable var : list) {
            Domain dom = var.dom;
            if (dom.firstValue() < 0 || dom.lastValue() > 1) {
                allBinary = false;
                break;
            }
        }

        if (allBinary && (value == 0 || value == 1)) {
            // Fast path for binary variables
            return addBinaryCountConstraint(countCtr, list, value, k, n, op, ctx);
        }

        // General case: reification for non-binary variables
        return addReifiedCountConstraint(countCtr, list, value, k, op, ctx);
    }

    /**
     * Fast path: direct linearization for binary {0,1} variables.
     */
    private boolean addBinaryCountConstraint(Count.CountCst countCtr, variables.Variable[] list,
            int value, int k, int n, String op, LinearizationContext ctx) {

        Expression expr = ctx.addExpression("count_" + countCtr.num);
        for (variables.Variable var : list) {
            expr.set(ctx.getLpVar(var), 1);
        }

        long limit;
        String adjustedOp = op;
        if (value == 1) {
            limit = k;
        } else {
            // value == 0: count(x=0) op k  <=>  count(x=1) flipped_op (n-k)
            limit = n - k;
            if (op.equals("<=")) adjustedOp = ">=";
            else if (op.equals(">=")) adjustedOp = "<=";
        }

        switch (adjustedOp) {
            case "<=":
                expr.upper(limit);
                break;
            case ">=":
                expr.lower(limit);
                break;
            case "==":
                expr.level(limit);
                break;
        }

        return true;
    }

    /**
     * General case: reification using auxiliary binary variables.
     *
     * For each variable x_i with domain [min, max], introduces:
     * - b_lt: 1 if x_i < value
     * - b_eq: 1 if x_i = value
     * - b_gt: 1 if x_i > value
     *
     * Uses big-M constraints to link b_eq to x_i = value.
     */
    private boolean addReifiedCountConstraint(Count.CountCst countCtr, variables.Variable[] list,
            int value, int k, String op, LinearizationContext ctx) {

        Expression countExpr = ctx.addExpression("count_" + countCtr.num);

        for (int i = 0; i < list.length; i++) {
            variables.Variable var = list[i];
            Domain dom = var.dom;
            double min = dom.firstValue();
            double max = dom.lastValue();

            // Create auxiliary binary variables for reification
            Variable bLt = Variable.make("count_" + countCtr.num + "_lt_" + i).lower(0).upper(1);
            Variable bEq = Variable.make("count_" + countCtr.num + "_eq_" + i).lower(0).upper(1);
            Variable bGt = Variable.make("count_" + countCtr.num + "_gt_" + i).lower(0).upper(1);
            ctx.addVariable(bLt);
            ctx.addVariable(bEq);
            ctx.addVariable(bGt);

            // Exactly one of {lt, eq, gt} must hold: b_lt + b_eq + b_gt = 1
            Expression splitExpr = ctx.addExpression("count_split_" + countCtr.num + "_" + i);
            splitExpr.set(bLt, 1);
            splitExpr.set(bEq, 1);
            splitExpr.set(bGt, 1);
            splitExpr.level(1);

            // Big-M values for each region
            double mLt = max - value + 1;
            double mGt = value - min + 1;
            double mEqUp = max - value;
            double mEqLo = value - min;

            // b_lt=1 => x_i <= value-1: x_i + mLt*b_lt <= value-1 + mLt
            Expression ltExpr = ctx.addExpression("count_lt_" + countCtr.num + "_" + i);
            ltExpr.set(ctx.getLpVar(var), 1);
            ltExpr.set(bLt, mLt);
            ltExpr.upper(value - 1 + mLt);

            // b_gt=1 => x_i >= value+1: x_i - mGt*b_gt >= value+1 - mGt
            Expression gtExpr = ctx.addExpression("count_gt_" + countCtr.num + "_" + i);
            gtExpr.set(ctx.getLpVar(var), 1);
            gtExpr.set(bGt, -mGt);
            gtExpr.lower(value + 1 - mGt);

            // b_eq=1 => x_i <= value: x_i + mEqUp*b_eq <= value + mEqUp
            Expression eqUpperExpr = ctx.addExpression("count_eq_up_" + countCtr.num + "_" + i);
            eqUpperExpr.set(ctx.getLpVar(var), 1);
            eqUpperExpr.set(bEq, mEqUp);
            eqUpperExpr.upper(value + mEqUp);

            // b_eq=1 => x_i >= value: x_i - mEqLo*b_eq >= value - mEqLo
            Expression eqLowerExpr = ctx.addExpression("count_eq_lo_" + countCtr.num + "_" + i);
            eqLowerExpr.set(ctx.getLpVar(var), 1);
            eqLowerExpr.set(bEq, -mEqLo);
            eqLowerExpr.lower(value - mEqLo);

            // Add b_eq to the count expression
            countExpr.set(bEq, 1);
        }

        // Apply the count constraint on sum of b_eq variables
        switch (op) {
            case "<=":
                countExpr.upper(k);
                break;
            case ">=":
                countExpr.lower(k);
                break;
            case "==":
                countExpr.level(k);
                break;
        }

        return true;
    }
}
