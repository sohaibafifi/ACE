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

public interface LpExpression {

	void set(LpVariable var, double coefficient);

	void lower(double lower);

	void upper(double upper);

	void level(double level);

	void weight(double weight);
}
