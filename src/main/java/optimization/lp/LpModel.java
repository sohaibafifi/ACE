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

public interface LpModel {

	LpExpression addExpression(String name);

	void addVariable(LpVariable var);

	LpVariable newVariable(String name, double lower, double upper);

	void setMinimization(boolean minimization);

	void setTimeLimitMs(long timeLimitMs);

	void setSparse(boolean sparse);

	void relax();

	void prepare();

	void reset();

	LpSolveResult solve();

	int variableCount();
}
