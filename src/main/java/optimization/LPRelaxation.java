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

import problem.Problem;
import constraints.Constraint;
import constraints.global.Sum;
import constraints.global.Count;
import variables.Domain;
import utility.Kit;

/**
 * LP Relaxation for computing lower/upper bounds to enable optimality detection.
 * This is a simple implementation that only handles
 * linear constraints (Sum constraints) and relaxes variable integrality.
 */
public class LPRelaxation {
    
    private final Problem problem;
    private ExpressionsBasedModel model;
    private Variable[] lpVars;  // LP variables corresponding to CP variables

    /**
     * Creates an LP relaxation for the given problem.
     * 
     * @param problem the problem to create LP relaxation for
     */
    public LPRelaxation(Problem problem) {
        this.problem = problem;
    }

    /**
     * Build LP model from current domains and linear constraints.
     * This creates continuous variables with bounds from current domains
     * and adds linear constraints (Sum constraints only).
     */
    public void buildModel() {
        model = new ExpressionsBasedModel();
        variables.Variable[] cpVars = problem.variables;
        lpVars = new Variable[cpVars.length];

        // Create LP variables with current domain bounds
        for (int i = 0; i < cpVars.length; i++) {
            Domain dom = cpVars[i].dom;
            // Use current domain bounds (tightened during search)
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

        // Kit.log.config("LP model: " + linearConstraints + " linear constraints, " + nonLinearConstraints + " non-linear (relaxed)");

        // Set objective (from Optimizer)
        setObjective();
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
        } else if (c instanceof Count.CountCst.ExactlyK) {
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

            if (adjustedOp.equals("<=")) {
                expr.upper(limit);
            } else if (adjustedOp.equals(">=")) {
                expr.lower(limit);
            } else if (adjustedOp.equals("==")) {
                expr.level(limit);
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
     * Solve the LP relaxation and return the bound value.
     * 
     * @return the LP bound value, or null if LP is infeasible/unbounded or objective not set
     */
    public Double solve() {
        if (model == null || !objectiveSet) {
            return null;
        }
        
        try {
            long startTime = System.currentTimeMillis();
            
            // Set time limit to prevent LP from taking too long ?
            // model.options.time_abort = 1_000L;  // milliseconds
            // model.options.time_suffice = 500L;  // stop if good enough solution found

            // Minimize or maximize based on optimizer type
            Optimisation.Result result;
            if (problem.optimizer.minimization) {
                result = model.minimise();
            } else {
                result = model.maximise();
            }
            
            long elapsed = System.currentTimeMillis() - startTime;
            // Kit.log.config("LP solve time: " + elapsed + "ms, state: " + result.getState());
            
            if (result.getState().isOptimal()) {
                Double value = result.getValue();
                freeMemory();  // Free LP model memory after root solve
                return value;
            }

            if (result.getState().isFeasible()){
                // TODO : return LB/UB even if not optimal ?
            }
            
            // LP infeasible means problem is infeasible
            freeMemory();  // Free LP model memory
            return null;
            
        } catch (Exception e) {
            // In case of any LP solver error, return null (no bound)
            freeMemory();  // Free LP model memory
            return null;
        }
    }
    
    /**
     * Free LP model memory to allow garbage collection.
     * Called after root node LP solve since LP is only computed once.
     */
    public void freeMemory() {
        model = null;
        lpVars = null;
        System.gc(); // Force garbage collection
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
