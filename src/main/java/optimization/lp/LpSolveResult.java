/*
 * This file is part of the constraint solver ACE (AbsCon Essence).
 *
 * Copyright (c) 2021. All rights reserved.
 * Christophe Lecoutre, CRIL, Univ. Artois and CNRS.
 *
 * Licensed under the MIT License.
 * See LICENSE file in the project root for full license information.
 */

package optimization.lp;

public final class LpSolveResult {

	public final LpStatus status;
	public final double objectiveValue;
	public final double objectiveBound;
	public final double[] variableValues;
	public final double[] reducedCosts;

	public LpSolveResult(LpStatus status, double objectiveValue, double objectiveBound, double[] variableValues, double[] reducedCosts) {
		this.status = status;
		this.objectiveValue = objectiveValue;
		this.objectiveBound = objectiveBound;
		this.variableValues = variableValues;
		this.reducedCosts = reducedCosts;
	}

	public static LpSolveResult invalid() {
		return new LpSolveResult(LpStatus.INVALID, Double.NaN, Double.NaN, null, null);
	}

	public static LpSolveResult failed() {
		return new LpSolveResult(LpStatus.FAILED, Double.NaN, Double.NaN, null, null);
	}

	public static LpSolveResult infeasible() {
		return new LpSolveResult(LpStatus.INFEASIBLE, Double.NaN, Double.NaN, null, null);
	}

	public boolean hasObjectiveBound() {
		return Double.isFinite(objectiveBound);
	}

	public boolean hasReducedCosts() {
		return reducedCosts != null;
	}

	public boolean isInfeasible() {
		return status != null && status.isInfeasible();
	}
}
