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

import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;

import org.ojalgo.optimisation.Expression;

import constraints.Constraint;
import constraints.global.Sum;
import variables.Variable;

/**
 * Linearizer for Sum constraints (SumSimple and SumWeighted variants).
 *
 * Handles:
 * - SumSimpleLE, SumSimpleGE, SumSimpleEQ: sum(x_i) op limit
 * - SumWeightedLE, SumWeightedGE, SumWeightedEQ: sum(c_i * x_i) op limit
 *
 * Also generates knapsack cover cuts for eligible weighted sum constraints
 * with binary variables, which strengthens the LP relaxation.
 */
public class SumLinearizer implements ConstraintLinearizer {

    @Override
    public boolean canLinearize(Constraint c) {
        return c instanceof Sum.SumSimple.SumSimpleLE
            || c instanceof Sum.SumSimple.SumSimpleGE
            || c instanceof Sum.SumSimple.SumSimpleEQ
            || c instanceof Sum.SumWeighted.SumWeightedLE
            || c instanceof Sum.SumWeighted.SumWeightedGE
            || c instanceof Sum.SumWeighted.SumWeightedEQ;
    }

    @Override
    public boolean linearize(Constraint c, LinearizationContext ctx) {
        if (c instanceof Sum.SumSimple.SumSimpleLE) {
            addSumSimpleConstraint((Sum.SumSimple) c, "<=", ctx);
            return true;
        } else if (c instanceof Sum.SumSimple.SumSimpleGE) {
            addSumSimpleConstraint((Sum.SumSimple) c, ">=", ctx);
            return true;
        } else if (c instanceof Sum.SumSimple.SumSimpleEQ) {
            addSumSimpleConstraint((Sum.SumSimple) c, "==", ctx);
            return true;
        } else if (c instanceof Sum.SumWeighted.SumWeightedLE) {
            addSumWeightedConstraint((Sum.SumWeighted) c, "<=", ctx);
            return true;
        } else if (c instanceof Sum.SumWeighted.SumWeightedGE) {
            addSumWeightedConstraint((Sum.SumWeighted) c, ">=", ctx);
            return true;
        } else if (c instanceof Sum.SumWeighted.SumWeightedEQ) {
            addSumWeightedConstraint((Sum.SumWeighted) c, "==", ctx);
            return true;
        }
        return false;
    }

    /**
     * Adds a simple sum constraint (sum of variables OP limit).
     */
    private void addSumSimpleConstraint(Sum.SumSimple sumCtr, String op, LinearizationContext ctx) {
        Variable[] scp = sumCtr.scp;
        long limit = sumCtr.limit();

        Expression expr = ctx.addExpression("sum_" + sumCtr.num);
        for (Variable var : scp) {
            expr.set(ctx.getLpVar(var), 1);
        }

        switch (op) {
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
    }

    /**
     * Adds a weighted sum constraint (sum of coeff*vars OP limit).
     */
    private void addSumWeightedConstraint(Sum.SumWeighted sumCtr, String op, LinearizationContext ctx) {
        Variable[] scp = sumCtr.scp;
        int[] coeffs = sumCtr.icoeffs;
        long limit = sumCtr.limit();

        Expression expr = ctx.addExpression("wsum_" + sumCtr.num);
        for (int i = 0; i < scp.length; i++) {
            expr.set(ctx.getLpVar(scp[i]), coeffs[i]);
        }

        switch (op) {
            case "<=":
                expr.upper(limit);
                addKnapsackCoverCutIfRelevant(sumCtr, ctx);
                break;
            case ">=":
                expr.lower(limit);
                break;
            case "==":
                expr.level(limit);
                break;
        }
    }

    /**
     * Adds one static minimal-cover cut for eligible 0/1 knapsack constraints.
     * This is a light-weight strengthening pass.
     *
     * A cover cut for knapsack sum(a_i * x_i) <= b identifies a minimal set C
     * of items whose total weight exceeds b, then adds: sum_{i in C} x_i <= |C| - 1
     */
    private void addKnapsackCoverCutIfRelevant(Sum.SumWeighted sumCtr, LinearizationContext ctx) {
        Variable[] scp = sumCtr.scp;
        int[] coeffs = sumCtr.icoeffs;
        long rhs = sumCtr.limit();

        if (rhs < 0 || scp.length <= 1) {
            return;
        }

        // Check all variables are binary with positive coefficients
        long total = 0;
        for (int i = 0; i < scp.length; i++) {
            if (coeffs[i] <= 0 || !ctx.isBinary01Domain(scp[i].dom)) {
                return;
            }
            total += coeffs[i];
        }
        if (total <= rhs) {
            return;
        }

        // Build cover greedily by selecting largest coefficients first
        List<Integer> ordered = new ArrayList<>();
        for (int i = 0; i < scp.length; i++) {
            ordered.add(i);
        }
        ordered.sort(Comparator.comparingInt((Integer i) -> coeffs[i]).reversed());

        List<Integer> cover = new ArrayList<>();
        long coverWeight = 0;
        for (int i : ordered) {
            cover.add(i);
            coverWeight += coeffs[i];
            if (coverWeight > rhs) {
                break;
            }
        }
        if (coverWeight <= rhs || cover.size() <= 1) {
            return;
        }

        // Minimize the cover greedily
        boolean changed;
        do {
            changed = false;
            for (int p = 0; p < cover.size(); p++) {
                int i = cover.get(p);
                if (coverWeight - coeffs[i] > rhs) {
                    coverWeight -= coeffs[i];
                    cover.remove(p);
                    changed = true;
                    break;
                }
            }
        } while (changed && cover.size() > 1);

        if (cover.size() <= 1) {
            return;
        }

        // Add cover cut: sum_{i in cover} x_i <= |cover| - 1
        Expression cut = ctx.addExpression("cover_" + sumCtr.num + "_" + ctx.nextCoverCutId());
        for (int i : cover) {
            cut.set(ctx.getLpVar(scp[i]), 1);
        }
        cut.upper(cover.size() - 1);
    }
}
