/*
 * This file is part of the constraint solver ACE (AbsCon Essence). 
 *
 * Copyright (c) 2021. All rights reserved.
 * Christophe Lecoutre, CRIL, Univ. Artois and CNRS. 
 * 
 * Licensed under the MIT License.
 * See LICENSE file in the project root for full license information.
 */

package optimization;

import org.ojalgo.optimisation.ExpressionsBasedModel;
import org.ojalgo.optimisation.Variable;
import org.ojalgo.optimisation.Optimisation;

import org.ojalgo.optimisation.solver.cplex.SolverCPLEX;
import problem.Problem;
import constraints.Constraint;
import constraints.global.Sum;
import constraints.global.Count;
import variables.Domain;
import utility.Kit;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;


/**
 * LP Relaxation for computing lower/upper bounds to enable optimality detection.
 * This is a simple implementation that only handles
 * linear constraints (Sum constraints) and relaxes variable integrality.
 *
 * The model is built once and variable domains are updated at each node to avoid
 * reconstructing the entire LP model repeatedly.
 */
public class LPRelaxation {

    private final Problem problem;
    private ExpressionsBasedModel model;
    private Variable[] lpVars;  // LP variables corresponding to CP variables
    private boolean modelBuilt = false;  // Track if model structure has been built
    private org.ojalgo.optimisation.Expression boundConstraint;  // Constraint for incumbent bound (enables pruning)
    private final long lpTimeoutMs;
    private int generatedCoverCuts;


    /**
     * Creates an LP relaxation for the given problem.
     *
     * @param problem the problem to create LP relaxation for
     */
    public LPRelaxation(Problem problem) {
        this.problem = problem;
        this.lpTimeoutMs = problem.head.control.optimization.lpTimeoutMs;
    }

    /**
     * Build LP model structure (variables and constraints) once.
     * This creates continuous variables and adds linear constraints.
     * Variable bounds are set from initial domains and can be updated later via updateDomains().
     */
    public void buildModel() {
        if (modelBuilt) {
            // Model already built, just update domains
            updateDomains();
            return;
        }

        generatedCoverCuts = 0;

        model = new ExpressionsBasedModel();
        variables.Variable[] cpVars = problem.variables;
        lpVars = new Variable[cpVars.length];

        // Create LP variables with initial domain bounds
        for (int i = 0; i < cpVars.length; i++) {
            Domain dom = cpVars[i].dom;
            // Use current domain bounds
            double lowerBound = dom.size() > 0 ? dom.firstValue() : 0;
            double upperBound = dom.size() > 0 ? dom.lastValue() : 0;

            lpVars[i] = Variable.make("x" + i)
                .lower(lowerBound)
                .upper(upperBound);
            model.addVariable(lpVars[i]);
        }

        // Add linear constraints (Sum constraints for now)
        Constraint clb = problem.optimizer != null ? (Constraint) problem.optimizer.clb : null;
        Constraint cub = problem.optimizer != null ? (Constraint) problem.optimizer.cub : null;

        int linearConstraints = 0;
        int nonLinearConstraints = 0;

        for (Constraint c : problem.constraints) {
            if (c.ignored) {
                continue;
            }

            // Skip the optimizer's bound constraints
            if (c == clb || c == cub) {
                continue;
            }

            if (addConstraintIfLinear(c)) {
                linearConstraints++;
            } else {
                nonLinearConstraints++;
            }
        }

        Kit.log.config("LP model: " + linearConstraints + " linear constraints, " + nonLinearConstraints + " non-linear (relaxed)");
        Kit.log.config("LP cuts: " + generatedCoverCuts);

        // Set objective (from Optimizer)
        setObjective();

        // Add bound constraint for pruning (objective <= bestKnown for minimization, >= for maximization)
        addBoundConstraint();

        // Configure solver options (once)
        configureSolver();

        modelBuilt = true;
    }

    /**
     * Adds a constraint on the objective to enable pruning based on incumbent solution.
     * For minimization: objective <= maxBound (best known solution)
     * For maximization: objective >= minBound (best known solution)
     */
    private void addBoundConstraint() {
        if (problem.optimizer == null || !objectiveSet) {
            return;
        }

        // Create a constraint that mirrors the objective expression
        Optimizable objCtr = problem.optimizer.ctr;
        boundConstraint = model.addExpression("incumbent_bound");

        if (objCtr instanceof Sum.SumSimple.SumSimpleLE || objCtr instanceof Sum.SumSimple.SumSimpleGE) {
            Sum.SumSimple sumCtr = (Sum.SumSimple) objCtr;
            for (variables.Variable var : sumCtr.scp) {
                boundConstraint.set(lpVars[var.num], 1);
            }
        } else if (objCtr instanceof Sum.SumWeighted.SumWeightedLE || objCtr instanceof Sum.SumWeighted.SumWeightedGE) {
            Sum.SumWeighted sumCtr = (Sum.SumWeighted) objCtr;
            int[] coeffs = sumCtr.icoeffs;
            for (int i = 0; i < sumCtr.scp.length; i++) {
                boundConstraint.set(lpVars[sumCtr.scp[i].num], coeffs[i]);
            }
        } else if (objCtr instanceof ObjectiveVariable) {
            ObjectiveVariable objVar = (ObjectiveVariable) objCtr;
            boundConstraint.set(lpVars[objVar.x.num], 1);
        }

        // Set initial bound from optimizer
        if (problem.optimizer.minimization) {
            // For minimization, we want objective <= maxBound (best known - 1)
            boundConstraint.upper(problem.optimizer.maxBound);
        } else {
            // For maximization, we want objective >= minBound (best known + 1)
            boundConstraint.lower(problem.optimizer.minBound);
        }
    }

    /**
     * Update the bound constraint when a new incumbent solution is found.
     * Called by Optimizer when a better solution is discovered.
     *
     * @param newBound the new bound value (maxBound for minimization, minBound for maximization)
     */
    public void updateBound(long newBound) {
        if (boundConstraint == null) {
            return;
        }

        if (problem.optimizer.minimization) {
            boundConstraint.upper(newBound);
        } else {
            boundConstraint.lower(newBound);
        }
    }

    /**
     * Update LP variable bounds from current CP variable domains.
     * This is called at each search node instead of rebuilding the entire model.
     */
    public void updateDomains() {
        if (model == null || lpVars == null) {
            return;
        }

        variables.Variable[] cpVars = problem.variables;
        for (int i = 0; i < cpVars.length; i++) {
            Domain dom = cpVars[i].dom;
            double lowerBound = dom.size() > 0 ? dom.firstValue() : 0;
            double upperBound = dom.size() > 0 ? dom.lastValue() : 0;

            lpVars[i].lower(lowerBound);
            lpVars[i].upper(upperBound);
        }
    }

    /**
     * Adds a constraint to the LP model if it's a linear constraint we can handle.
     * Currently handles: SumSimpleLE, SumSimpleGE, SumWeightedLE, SumWeightedGE, SumWeightedEQ
     * Future support: AllDifferent (via assignment problem relaxation)
     * Non-linear constraints (Element, etc.) are relaxed away (ignored).
     * 
     * @param c the constraint to potentially add
     * @return true if constraint was added, false otherwise
     */
    private boolean addConstraintIfLinear(Constraint c) {
        if (c instanceof Sum.SumSimple.SumSimpleLE) {
            Sum.SumSimple.SumSimpleLE sumCtr = (Sum.SumSimple.SumSimpleLE) c;
            addSumSimpleConstraint(sumCtr, "<=");
            return true;
        } else if (c instanceof Sum.SumSimple.SumSimpleGE) {
            Sum.SumSimple.SumSimpleGE sumCtr = (Sum.SumSimple.SumSimpleGE) c;
            addSumSimpleConstraint(sumCtr, ">=");
            return true;
        } else if (c instanceof Sum.SumWeighted.SumWeightedLE) {
            Sum.SumWeighted.SumWeightedLE sumCtr = (Sum.SumWeighted.SumWeightedLE) c;
            addSumWeightedConstraint(sumCtr, "<=");
            return true;
        } else if (c instanceof Sum.SumWeighted.SumWeightedGE) {
            Sum.SumWeighted.SumWeightedGE sumCtr = (Sum.SumWeighted.SumWeightedGE) c;
            addSumWeightedConstraint(sumCtr, ">=");
            return true;
        } else if (c instanceof Sum.SumWeighted.SumWeightedEQ) {
            Sum.SumWeighted.SumWeightedEQ sumCtr = (Sum.SumWeighted.SumWeightedEQ) c;
            addSumWeightedConstraint(sumCtr, "==");
            return true;
        } else if (c instanceof Sum.SumSimple.SumSimpleEQ) {
            Sum.SumSimple.SumSimpleEQ sumCtr = (Sum.SumSimple.SumSimpleEQ) c;
            addSumSimpleConstraint(sumCtr, "==");
            return true;
        }
        else if (c instanceof Count.CountCst.ExactlyK) {
            // ExactlyK: exactly k variables = value
            // For binary vars {0,1}: if value=1, sum=k; if value=0, sum=n-k
            return addCountConstraint((Count.CountCst) c, "==");
        } else if (c instanceof Count.CountCst.AtMostK || c instanceof Count.CountCst.AtMost1) {
            // AtMostK: at most k variables = value
            // For binary vars {0,1}: if value=1, sum<=k; if value=0, sum>=n-k
            return addCountConstraint((Count.CountCst) c, "<=");
        } else if (c instanceof Count.CountCst.AtLeastK || c instanceof Count.CountCst.AtLeast1) {
            // AtLeastK: at least k variables = value
            // For binary vars {0,1}: if value=1, sum>=k; if value=0, sum<=n-k
            return addCountConstraint((Count.CountCst) c, ">=");
        }
        // TODO: Add support for more constraints

        // Future enhancement: Add AllDifferent constraint support
        // else if (c instanceof constraints.global.AllDifferent) {
        //     addAllDifferentConstraint((constraints.global.AllDifferent) c);
        // }
        
        // Other constraint types are simply ignored (relaxed away)
        // This includes: Element, Cardinality, etc.
        return false;
    }

    /**
     * Adds a simple sum constraint (sum of variables OP limit).
     * 
     * @param sumCtr the sum constraint
     * @param op the operator ("<=", ">=", or "==")
     */
    private void addSumSimpleConstraint(Sum.SumSimple sumCtr, String op) {
        variables.Variable[] scp = sumCtr.scp;
        long limit = sumCtr.limit();
        
        // Build expression: sum of LP variables
        org.ojalgo.optimisation.Expression expr = model.addExpression("sum_" + sumCtr.num);
        for (variables.Variable var : scp) {
            expr.set(lpVars[var.num], 1);
        }
        
        // Set constraint based on operator
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
     * 
     * @param sumCtr the weighted sum constraint
     * @param op the operator ("<=", ">=", or "==")
     */
    private void addSumWeightedConstraint(Sum.SumWeighted sumCtr, String op) {
        variables.Variable[] scp = sumCtr.scp;
        int[] coeffs = sumCtr.icoeffs;
        long limit = sumCtr.limit();
        
        // Build expression: weighted sum of LP variables
        org.ojalgo.optimisation.Expression expr = model.addExpression("wsum_" + sumCtr.num);
        for (int i = 0; i < scp.length; i++) {
            expr.set(lpVars[scp[i].num], coeffs[i]);
        }
        
        // Set constraint based on operator
        switch (op) {
            case "<=":
                expr.upper(limit);
                addKnapsackCoverCutIfRelevant(sumCtr);
                break;
            case ">=":
                expr.lower(limit);
                break;
            case "==":
                expr.level(limit);
                break;
        }
    }

    private boolean isBinary01Domain(Domain dom) {
        return dom.firstValue() >= 0 && dom.lastValue() <= 1;
    }

    /**
     * Adds one static minimal-cover cut for eligible 0/1 knapsack constraints.
     * This is a light-weight strengthening pass.
     */
    private void addKnapsackCoverCutIfRelevant(Sum.SumWeighted sumCtr) {
        variables.Variable[] scp = sumCtr.scp;
        int[] coeffs = sumCtr.icoeffs;
        long rhs = sumCtr.limit();

        if (rhs < 0 || scp.length <= 1) {
            return;
        }

        long total = 0;
        for (int i = 0; i < scp.length; i++) {
            if (coeffs[i] <= 0 || !isBinary01Domain(scp[i].dom)) {
                return;
            }
            total += coeffs[i];
        }
        if (total <= rhs) {
            return;
        }

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

        // Minimalise the cover greedily.
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

        org.ojalgo.optimisation.Expression cut = model.addExpression("cover_" + sumCtr.num + "_" + generatedCoverCuts++);
        for (int i : cover) {
            cut.set(lpVars[scp[i].num], 1);
        }
        cut.upper(cover.size() - 1);
    }

    /**
     * Adds a Count constraint (ExactlyK, AtMostK, AtLeastK) to the LP model.
     * TODO: move to Count class?
     * @param countCtr the count constraint
     * @param op the original operator ("==", "<=", or ">=") based on count type
     * @return true if constraint was added, false if not applicable
     */
    private boolean addCountConstraint(Count.CountCst countCtr, String op) {
        // Get the list of variables and the value being counted
        variables.Variable[] list = countCtr.scp;  // scp contains the list
        int value = countCtr.getValue();
        int k = countCtr.getK();
        int n = list.length;

        boolean allBinary = true;
        for (variables.Variable var : list) {
            Domain dom = var.dom;
            if (dom.firstValue() < 0 || dom.lastValue() > 1) {
                allBinary = false;
                break;
            }
        }

        if (allBinary && (value == 0 || value == 1)) {
            // Fast path for binary variables {0,1}
            org.ojalgo.optimisation.Expression expr = model.addExpression("count_" + countCtr.num);
            for (variables.Variable var : list) {
                expr.set(lpVars[var.num], 1);
            }

            long limit;
            String adjustedOp = op;
            if (value == 1) {
                limit = k;
            } else {  // value == 0
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

        // General case: reification for non-binary variables
        org.ojalgo.optimisation.Expression countExpr = model.addExpression("count_" + countCtr.num);

        for (int i = 0; i < list.length; i++) {
            variables.Variable var = list[i];
            Domain dom = var.dom;
            double min = dom.firstValue();
            double max = dom.lastValue();

            Variable bLt = Variable.make("count_" + countCtr.num + "_lt_" + i).lower(0).upper(1);
            Variable bEq = Variable.make("count_" + countCtr.num + "_eq_" + i).lower(0).upper(1);
            Variable bGt = Variable.make("count_" + countCtr.num + "_gt_" + i).lower(0).upper(1);
            model.addVariable(bLt);
            model.addVariable(bEq);
            model.addVariable(bGt);

            org.ojalgo.optimisation.Expression splitExpr = model.addExpression("count_split_" + countCtr.num + "_" + i);
            splitExpr.set(bLt, 1);
            splitExpr.set(bEq, 1);
            splitExpr.set(bGt, 1);
            splitExpr.level(1);

            double mLt = max - value + 1;
            double mGt = value - min + 1;
            double mEqUp = max - value;
            double mEqLo = value - min;

            org.ojalgo.optimisation.Expression ltExpr = model.addExpression("count_lt_" + countCtr.num + "_" + i);
            ltExpr.set(lpVars[var.num], 1);
            ltExpr.set(bLt, mLt);
            ltExpr.upper(value - 1 + mLt);

            org.ojalgo.optimisation.Expression gtExpr = model.addExpression("count_gt_" + countCtr.num + "_" + i);
            gtExpr.set(lpVars[var.num], 1);
            gtExpr.set(bGt, -mGt);
            gtExpr.lower(value + 1 - mGt);

            org.ojalgo.optimisation.Expression eqUpperExpr = model.addExpression("count_eq_up_" + countCtr.num + "_" + i);
            eqUpperExpr.set(lpVars[var.num], 1);
            eqUpperExpr.set(bEq, mEqUp);
            eqUpperExpr.upper(value + mEqUp);

            org.ojalgo.optimisation.Expression eqLowerExpr = model.addExpression("count_eq_lo_" + countCtr.num + "_" + i);
            eqLowerExpr.set(lpVars[var.num], 1);
            eqLowerExpr.set(bEq, -mEqLo);
            eqLowerExpr.lower(value - mEqLo);

            countExpr.set(bEq, 1);
        }

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

    /**
     * Flag indicating if a valid objective was set
     */
    private boolean objectiveSet = false;
    
    /**
     * Sets the objective function based on the optimizer constraint.
     * The optimizer's main constraint (clb or cub) defines the objective.
     */
    private void setObjective() {
        objectiveSet = false;
        
        if (problem.optimizer == null) {
            return;
        }
        
        Optimizable objCtr = problem.optimizer.ctr;
        
        if (objCtr instanceof Sum.SumSimple.SumSimpleLE || objCtr instanceof Sum.SumSimple.SumSimpleGE) {
            Sum.SumSimple sumCtr = (Sum.SumSimple) objCtr;
            variables.Variable[] scp = sumCtr.scp;
            
            // Build objective expression
            org.ojalgo.optimisation.Expression objExpr = model.addExpression("objective");
            for (variables.Variable var : scp) {
                objExpr.set(lpVars[var.num], 1);
            }
            
            // Set as objective (minimize or maximize based on optimizer type)
            objExpr.weight(1);
            objectiveSet = true;
            
        } else if (objCtr instanceof Sum.SumWeighted.SumWeightedLE || objCtr instanceof Sum.SumWeighted.SumWeightedGE) {
            Sum.SumWeighted sumCtr = (Sum.SumWeighted) objCtr;
            variables.Variable[] scp = sumCtr.scp;
            int[] coeffs = sumCtr.icoeffs;
            
            // Build objective expression
            org.ojalgo.optimisation.Expression objExpr = model.addExpression("objective");
            for (int i = 0; i < scp.length; i++) {
                objExpr.set(lpVars[scp[i].num], coeffs[i]);
            }
            
            // Set as objective
            objExpr.weight(1);
            objectiveSet = true;
            
        } else if (objCtr instanceof ObjectiveVariable) {
            // Handle objective variable case
            ObjectiveVariable objVar = (ObjectiveVariable) objCtr;
            int varNum = objVar.x.num;
            
            org.ojalgo.optimisation.Expression objExpr = model.addExpression("objective");
            objExpr.set(lpVars[varNum], 1);
            objExpr.weight(1);
            objectiveSet = true;
        }
        // If none of the above match, objectiveSet stays false
        // solve() will return null in that case
    }

    /**
     * Configure solver options. Called once during model building.
     */
    private void configureSolver() {
        ExpressionsBasedModel.Integration<SolverCPLEX> integration = SolverCPLEX.INTEGRATION;
        model.options.setConfigurator(integration);
        if (problem.head.control.general.verbose > 0) {
            model.options.progress(SolverCPLEX.class);
            if (problem.head.control.general.verbose > 1) {
                model.options.debug(SolverCPLEX.class);
            }
        }
        // Set time limit to prevent LP from taking too long
        // Abort LP solve when timeout is reached.
        if (lpTimeoutMs > 0L) {
            model.options.time_abort = lpTimeoutMs;
        }

        model.options.sparse = Boolean.TRUE;
        // Relax integrality constraints (make all variables continuous)
        model.relax();
    }

    /**
     * Solve the LP relaxation and return the bound value.
     * The model is reused across calls - only variable bounds are updated via updateDomains().
     *
     * @return the LP bound value, or null if LP is infeasible/unbounded or objective not set
     */
    public Double solve() {
        if (model == null || !objectiveSet) {
            return null;
        }

        try {
            long startTime = System.currentTimeMillis();

            // Minimize or maximize based on optimizer type
            Optimisation.Result result;
            if (problem.optimizer.minimization) {
                result = model.minimise();
            } else {
                result = model.maximise();
            }

            long elapsed = System.currentTimeMillis() - startTime;
            if (problem.head.control.general.verbose > 0)
                Kit.log.config("LP solve time: " + elapsed + "ms, state: " + result.getState() + ((result.getState().isOptimal() ||  result.getState().isFeasible() || result.getState().isUnexplored()) ?  ", value: " + result.getValue() : ""));

            if (result.getState().isOptimal()) {
                return result.getValue();
            }

            if (result.getState().isFeasible() || result.getState().isUnexplored()) {
                // Not optimal but we have information - return the best bound available
                // For LP, when not optimal due to time limit, we return the current objective
                // which serves as a valid bound (though possibly weaker than the true optimal)
                // Note: For MIP, CPLEX would provide getBestObjValue() for the dual bound,
                // but for pure LP relaxation, the current value is our best estimate
                // return result.getValue();
                return null;
            }

            // LP infeasible means the current search branch is infeasible
            return null;

        } catch (Exception e) {
            Kit.log.config("LP solver error: " + e.getMessage() + " (" + e.getClass().getSimpleName() + ")");
            // In case of any LP solver error, return null (no bound)
            return null;
        }
    }

    /**
     * Check if LP relaxation is viable for this problem.
     * LP is not viable if the objective can't be modeled (e.g., SumViewWeighted).
     * 
     * @return true if LP can provide useful bounds, false otherwise
     */
    public boolean isViable() {
        return objectiveSet;
    }
}
