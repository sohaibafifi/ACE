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

import constraints.Constraint;

/**
 * Interface for constraint linearizers that convert CP constraints to LP expressions.
 * Each implementation handles a specific family of constraints.
 */
public interface ConstraintLinearizer {

    /**
     * Check if this linearizer can handle the given constraint.
     *
     * @param c the constraint to check
     * @return true if this linearizer can linearize the constraint
     */
    boolean canLinearize(Constraint c);

    /**
     * Linearize the constraint into LP expressions.
     * Called only after canLinearize() returns true.
     *
     * @param c the constraint to linearize
     * @param ctx the linearization context providing access to LP model and variables
     * @return true if constraint was added to the model, false if skipped
     */
    boolean linearize(Constraint c, LinearizationContext ctx);
}
