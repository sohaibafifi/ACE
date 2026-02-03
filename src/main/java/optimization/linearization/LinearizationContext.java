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

import org.ojalgo.optimisation.ExpressionsBasedModel;
import org.ojalgo.optimisation.Variable;
import org.ojalgo.optimisation.Expression;

import problem.Problem;
import variables.Domain;

/**
 * Context object passed to ConstraintLinearizers during linearization.
 * Provides access to the LP model, variables, and helper methods.
 *
 * This class acts as a facade, shielding linearizers from the details of
 * how the LP model is structured and providing convenient utility methods.
 */
public class LinearizationContext {

    private final ExpressionsBasedModel model;
    private final Variable[] lpVars;
    private final Problem problem;
    private int coverCutCount;

    /**
     * Creates a new linearization context.
     *
     * @param model the ojAlgo expressions-based model
     * @param lpVars LP variables corresponding to CP variables (indexed by var.num)
     * @param problem the CP problem being relaxed
     */
    public LinearizationContext(ExpressionsBasedModel model, Variable[] lpVars, Problem problem) {
        this.model = model;
        this.lpVars = lpVars;
        this.problem = problem;
        this.coverCutCount = 0;
    }

    /**
     * Add a new named expression (constraint) to the LP model.
     *
     * @param name the name for the expression
     * @return the created expression
     */
    public Expression addExpression(String name) {
        return model.addExpression(name);
    }

    /**
     * Add a new LP variable to the model (e.g., for reification).
     *
     * @param var the variable to add
     */
    public void addVariable(Variable var) {
        model.addVariable(var);
    }

    /**
     * Get the LP variable corresponding to a CP variable.
     *
     * @param cpVar the CP variable
     * @return the corresponding LP variable
     */
    public Variable getLpVar(variables.Variable cpVar) {
        return lpVars[cpVar.num];
    }

    /**
     * Get the LP variable by index.
     *
     * @param index the variable index
     * @return the LP variable at that index
     */
    public Variable getLpVar(int index) {
        return lpVars[index];
    }

    /**
     * Check if a domain represents a binary {0,1} variable.
     *
     * @param dom the domain to check
     * @return true if domain is contained in {0,1}
     */
    public boolean isBinary01Domain(Domain dom) {
        return dom.firstValue() >= 0 && dom.lastValue() <= 1;
    }

    /**
     * Get the underlying problem.
     *
     * @return the CP problem
     */
    public Problem getProblem() {
        return problem;
    }

    /**
     * Increment and return the cover cut counter.
     * Used for generating unique names for knapsack cover cuts.
     *
     * @return the next cover cut number
     */
    public int nextCoverCutId() {
        return coverCutCount++;
    }

    /**
     * Get the total number of cover cuts generated.
     *
     * @return the cover cut count
     */
    public int getCoverCutCount() {
        return coverCutCount;
    }
}
