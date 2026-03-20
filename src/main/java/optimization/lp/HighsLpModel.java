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

import java.lang.ref.Cleaner;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;

public final class HighsLpModel implements LpModel {

	private static final Cleaner CLEANER = Cleaner.create();
	private static final double HIGHS_INF = 1e30;

	private final Object highs;
	private final Cleaner.Cleanable cleanable;
	private final List<HighsLpVariable> variables;
	private final List<HighsLpExpression> expressions;
	private final Set<Integer> dirtyBoundIndexes;

	private boolean minimization = true;
	private final boolean integerVariables;
	private long timeLimitMs = 0L;
	private boolean structureDirty = true;
	private boolean optionsDirty = true;

	public HighsLpModel(boolean integerVariables) {
		this.highs = HighsCApi.create();
		this.cleanable = CLEANER.register(this, new DestroyAction(highs));
		this.variables = new ArrayList<>();
		this.expressions = new ArrayList<>();
		this.dirtyBoundIndexes = new LinkedHashSet<>();
		this.integerVariables = integerVariables;
	}

	@Override
	public LpExpression addExpression(String name) {
		HighsLpExpression expression = new HighsLpExpression(name);
		expressions.add(expression);
		structureDirty = true;
		return expression;
	}

	@Override
	public void addVariable(LpVariable var) {
		HighsLpVariable variable = unwrap(var);
		if (variable.owner != this)
			throw new IllegalArgumentException("LP variable belongs to a different model");
		if (variable.index >= 0)
			return;
		variable.index = variables.size();
		variables.add(variable);
		structureDirty = true;
	}

	@Override
	public LpVariable newVariable(String name, double lower, double upper) {
		return new HighsLpVariable(this, name, lower, upper, integerVariables);
	}

	@Override
	public void setMinimization(boolean minimization) {
		if (this.minimization != minimization) {
			this.minimization = minimization;
			structureDirty = true;
		}
	}

	@Override
	public void setTimeLimitMs(long timeLimitMs) {
		if (this.timeLimitMs != timeLimitMs) {
			this.timeLimitMs = timeLimitMs;
			optionsDirty = true;
		}
	}

	@Override
	public void setSparse(boolean sparse) {
		// HiGHS always receives a sparse matrix here.
	}

	@Override
	public void relax() {
		// Variables are modeled as continuous by construction.
	}

	@Override
	public void prepare() {
		// Nothing to prepare eagerly: the native model is synchronized on solve().
	}

	@Override
	public void reset() {
		// Bound changes are applied lazily on the next solve().
	}

	@Override
	public LpSolveResult solve() {
		applyOptions();
		if (structureDirty)
			rebuildNativeModel();
		else
			applyDirtyBounds();

		long runStatus = HighsCApi.run(highs);
		long modelStatus = HighsCApi.getModelStatus(highs);
		LpStatus status = mapStatus(runStatus, modelStatus);

		try (HighsCApi.NativeArena arena = HighsCApi.openArena()) {
			double objectiveValue = HighsCApi.getDoubleInfoValue(highs, arena, "objective_function_value", Double.NaN);
			int numCol = variables.size();
			int numRow = constraintExpressions().size();
			long primalSolutionStatus = HighsCApi.getIntInfoValue(highs, arena, "primal_solution_status", HighsCApi.SOLUTION_STATUS_NONE);
			long dualSolutionStatus = HighsCApi.getIntInfoValue(highs, arena, "dual_solution_status", HighsCApi.SOLUTION_STATUS_NONE);
			double mipDualBound = integerVariables ? HighsCApi.getDoubleInfoValue(highs, arena, "mip_dual_bound", Double.NaN) : Double.NaN;

			double[] variableValues = null;
			double[] colDuals = null;
			double[] rowValues = null;
			double[] rowDuals = null;
			if (primalSolutionStatus != HighsCApi.SOLUTION_STATUS_NONE || dualSolutionStatus != HighsCApi.SOLUTION_STATUS_NONE) {
				Object colValueSeg = allocateDoubles(arena, numCol);
				Object colDualSeg = allocateDoubles(arena, numCol);
				Object rowValueSeg = allocateDoubles(arena, numRow);
				Object rowDualSeg = allocateDoubles(arena, numRow);
				long solutionStatus = HighsCApi.getSolution(highs, colValueSeg, colDualSeg, rowValueSeg, rowDualSeg);
				if (!HighsCApi.isOkOrWarning(solutionStatus))
					return LpSolveResult.failed();
				if (primalSolutionStatus != HighsCApi.SOLUTION_STATUS_NONE) {
					variableValues = readDoubles(colValueSeg, numCol);
					rowValues = readDoubles(rowValueSeg, numRow);
				}
				if (dualSolutionStatus != HighsCApi.SOLUTION_STATUS_NONE) {
					colDuals = readDoubles(colDualSeg, numCol);
					rowDuals = readDoubles(rowDualSeg, numRow);
				}
			}

			double objectiveBound = Double.NaN;
			if (integerVariables) {
				if (Double.isFinite(mipDualBound) && Math.abs(mipDualBound) < HIGHS_INF)
					objectiveBound = mipDualBound;
			} else if (dualSolutionStatus == HighsCApi.SOLUTION_STATUS_FEASIBLE && variableValues != null && rowValues != null && colDuals != null && rowDuals != null)
				objectiveBound = computeDualObjectiveValue(variableValues, colDuals, rowValues, rowDuals);
			if (status.isOptimal() && !Double.isFinite(objectiveBound))
				objectiveBound = objectiveValue;

			double[] reducedCosts = status.isOptimal() && colDuals != null ? colDuals : null;
			return new LpSolveResult(status, objectiveValue, objectiveBound, variableValues, reducedCosts);
		}
	}

	@Override
	public int variableCount() {
		return variables.size();
	}

	private void applyOptions() {
		if (!optionsDirty)
			return;
		try (HighsCApi.NativeArena arena = HighsCApi.openArena()) {
			HighsCApi.setBoolOption(highs, arena, "output_flag", false);
			HighsCApi.setDoubleOption(highs, arena, "infinite_bound", HIGHS_INF);
			HighsCApi.setDoubleOption(highs, arena, "infinite_cost", HIGHS_INF);
			HighsCApi.setDoubleOption(highs, arena, "time_limit", timeLimitMs > 0L ? timeLimitMs / 1000d : HIGHS_INF);
		}
		optionsDirty = false;
	}

	private void rebuildNativeModel() {
		List<HighsLpExpression> rows = constraintExpressions();
		int numCol = variables.size();
		int numRow = rows.size();

		double[] colCost = new double[numCol];
		for (HighsLpExpression expression : expressions) {
			if (expression.weight == 0d)
				continue;
			for (Map.Entry<Integer, Double> entry : expression.coefficients.entrySet())
				colCost[entry.getKey()] += expression.weight * entry.getValue();
		}

		double[] colLower = new double[numCol];
		double[] colUpper = new double[numCol];
		for (int i = 0; i < numCol; i++) {
			HighsLpVariable variable = variables.get(i);
			colLower[i] = toHighsLower(variable.lower);
			colUpper[i] = toHighsUpper(variable.upper);
		}

		double[] rowLower = new double[numRow];
		double[] rowUpper = new double[numRow];
		@SuppressWarnings("unchecked")
		List<Integer>[] columnRows = new List[numCol];
		@SuppressWarnings("unchecked")
		List<Double>[] columnValues = new List[numCol];
		for (int i = 0; i < numCol; i++) {
			columnRows[i] = new ArrayList<>();
			columnValues[i] = new ArrayList<>();
		}

		for (int row = 0; row < numRow; row++) {
			HighsLpExpression expression = rows.get(row);
			rowLower[row] = expression.hasLower ? toHighsLower(expression.lower) : -HIGHS_INF;
			rowUpper[row] = expression.hasUpper ? toHighsUpper(expression.upper) : HIGHS_INF;
			for (Map.Entry<Integer, Double> entry : expression.coefficients.entrySet()) {
				columnRows[entry.getKey()].add(row);
				columnValues[entry.getKey()].add(entry.getValue());
			}
		}

		long[] aStart = new long[numCol];
		int numNz = 0;
		for (int col = 0; col < numCol; col++) {
			aStart[col] = numNz;
			numNz += columnRows[col].size();
		}

		long[] aIndex = new long[numNz];
		double[] aValue = new double[numNz];
		int position = 0;
		for (int col = 0; col < numCol; col++) {
			List<Integer> rowsForColumn = columnRows[col];
			List<Double> valuesForColumn = columnValues[col];
			for (int i = 0; i < rowsForColumn.size(); i++) {
				aIndex[position] = rowsForColumn.get(i).longValue();
				aValue[position] = valuesForColumn.get(i).doubleValue();
				position++;
			}
		}

		try (HighsCApi.NativeArena arena = HighsCApi.openArena()) {
			Object colCostSeg = allocateDoubles(arena, colCost);
			Object colLowerSeg = allocateDoubles(arena, colLower);
			Object colUpperSeg = allocateDoubles(arena, colUpper);
			Object rowLowerSeg = allocateDoubles(arena, rowLower);
			Object rowUpperSeg = allocateDoubles(arena, rowUpper);
			Object aStartSeg = allocateLongs(arena, aStart);
			Object aIndexSeg = numNz == 0 ? HighsCApi.nullSegment() : allocateLongs(arena, aIndex);
			Object aValueSeg = numNz == 0 ? HighsCApi.nullSegment() : allocateDoubles(arena, aValue);
			Object integralitySeg = integerVariables ? allocateLongs(arena, integralityVector()) : HighsCApi.nullSegment();

			long clearStatus = HighsCApi.clearModel(highs);
			if (!HighsCApi.isOkOrWarning(clearStatus))
				throw new IllegalStateException("HiGHS clearModel failed with status " + clearStatus);

			long passStatus = integerVariables
					? HighsCApi.passMip(highs, numCol, numRow, numNz, HighsCApi.MATRIX_FORMAT_COLWISE,
							minimization ? HighsCApi.OBJ_SENSE_MINIMIZE : HighsCApi.OBJ_SENSE_MAXIMIZE, 0d, colCostSeg, colLowerSeg, colUpperSeg, rowLowerSeg,
							rowUpperSeg, aStartSeg, aIndexSeg, aValueSeg, integralitySeg)
					: HighsCApi.passLp(highs, numCol, numRow, numNz, HighsCApi.MATRIX_FORMAT_COLWISE,
							minimization ? HighsCApi.OBJ_SENSE_MINIMIZE : HighsCApi.OBJ_SENSE_MAXIMIZE, 0d, colCostSeg, colLowerSeg, colUpperSeg, rowLowerSeg,
							rowUpperSeg, aStartSeg, aIndexSeg, aValueSeg);
			if (!HighsCApi.isOkOrWarning(passStatus))
				throw new IllegalStateException("HiGHS passModel failed with status " + passStatus);
		}

		structureDirty = false;
		dirtyBoundIndexes.clear();
	}

	private void applyDirtyBounds() {
		if (dirtyBoundIndexes.isEmpty())
			return;
		for (Integer index : dirtyBoundIndexes) {
			HighsLpVariable variable = variables.get(index.intValue());
			long status = HighsCApi.changeColBounds(highs, variable.index, toHighsLower(variable.lower), toHighsUpper(variable.upper));
			if (!HighsCApi.isOkOrWarning(status))
				throw new IllegalStateException("HiGHS changeColBounds failed with status " + status + " on column " + variable.index);
		}
		dirtyBoundIndexes.clear();
	}

	private List<HighsLpExpression> constraintExpressions() {
		List<HighsLpExpression> rows = new ArrayList<>();
		for (HighsLpExpression expression : expressions) {
			if (expression.hasLower || expression.hasUpper)
				rows.add(expression);
		}
		return rows;
	}

	private void markStructureDirty() {
		structureDirty = true;
	}

	private void markBoundsDirty(int index) {
		if (!structureDirty)
			dirtyBoundIndexes.add(index);
	}

	private static double toHighsLower(double value) {
		return Double.isFinite(value) ? value : -HIGHS_INF;
	}

	private static double toHighsUpper(double value) {
		return Double.isFinite(value) ? value : HIGHS_INF;
	}

	private long[] integralityVector() {
		long[] integrality = new long[variables.size()];
		for (int i = 0; i < variables.size(); i++)
			integrality[i] = variables.get(i).integer ? HighsCApi.VAR_TYPE_INTEGER : HighsCApi.VAR_TYPE_CONTINUOUS;
		return integrality;
	}

	private double computeDualObjectiveValue(double[] colValues, double[] colDuals, double[] rowValues, double[] rowDuals) {
		double dualObjective = 0d;
		for (int i = 0; i < variables.size(); i++)
			dualObjective += chooseBound(colValues[i], variables.get(i).lower, variables.get(i).upper) * colDuals[i];
		List<HighsLpExpression> rows = constraintExpressions();
		for (int i = 0; i < rows.size(); i++) {
			HighsLpExpression row = rows.get(i);
			double lower = row.hasLower ? row.lower : -HIGHS_INF;
			double upper = row.hasUpper ? row.upper : HIGHS_INF;
			dualObjective += chooseBound(rowValues[i], lower, upper) * rowDuals[i];
		}
		return dualObjective;
	}

	private static double chooseBound(double primal, double lower, double upper) {
		if (lower <= -HIGHS_INF && upper >= HIGHS_INF)
			return 1d;
		double mid = (lower + upper) * 0.5d;
		return primal < mid ? lower : upper;
	}

	private static double[] readDoubles(Object segment, int size) {
		return HighsCApi.readDoubles(segment, size);
	}

	private static Object allocateDoubles(HighsCApi.NativeArena arena, int size) {
		return HighsCApi.allocateDoubles(arena, size);
	}

	private static Object allocateDoubles(HighsCApi.NativeArena arena, double[] values) {
		return HighsCApi.allocateDoubles(arena, values);
	}

	private static Object allocateLongs(HighsCApi.NativeArena arena, long[] values) {
		return HighsCApi.allocateLongs(arena, values);
	}

	private static LpStatus mapStatus(long runStatus, long modelStatus) {
		if (runStatus == HighsCApi.STATUS_ERROR)
			return LpStatus.FAILED;
		if (modelStatus == HighsCApi.MODEL_STATUS_OPTIMAL)
			return LpStatus.OPTIMAL;
		if (modelStatus == HighsCApi.MODEL_STATUS_INFEASIBLE)
			return LpStatus.INFEASIBLE;
		if (modelStatus == HighsCApi.MODEL_STATUS_UNBOUNDED)
			return LpStatus.UNBOUNDED;
		if (modelStatus == HighsCApi.MODEL_STATUS_MODEL_EMPTY)
			return LpStatus.INVALID;
		if (modelStatus == HighsCApi.MODEL_STATUS_UNBOUNDED_OR_INFEASIBLE || modelStatus == HighsCApi.MODEL_STATUS_TIME_LIMIT
				|| modelStatus == HighsCApi.MODEL_STATUS_ITERATION_LIMIT || modelStatus == HighsCApi.MODEL_STATUS_OBJECTIVE_BOUND
				|| modelStatus == HighsCApi.MODEL_STATUS_OBJECTIVE_TARGET || modelStatus == HighsCApi.MODEL_STATUS_SOLUTION_LIMIT
				|| modelStatus == HighsCApi.MODEL_STATUS_INTERRUPT || modelStatus == HighsCApi.MODEL_STATUS_UNKNOWN)
			return LpStatus.UNKNOWN;
		return LpStatus.FAILED;
	}

	private static HighsLpVariable unwrap(LpVariable variable) {
		if (!(variable instanceof HighsLpVariable))
			throw new IllegalArgumentException("Unsupported LP variable implementation: " + variable.getClass().getName());
		return (HighsLpVariable) variable;
	}

	private static final class DestroyAction implements Runnable {
		private final Object highs;

		private DestroyAction(Object highs) {
			this.highs = highs;
		}

		@Override
		public void run() {
			HighsCApi.destroy(highs);
		}
	}

	private final class HighsLpVariable implements LpVariable {
		private final HighsLpModel owner;
		private final String name;
		private double lower;
		private double upper;
		private boolean integer;
		private int index = -1;

		private HighsLpVariable(HighsLpModel owner, String name, double lower, double upper, boolean integer) {
			this.owner = owner;
			this.name = name;
			this.lower = lower;
			this.upper = upper;
			this.integer = integer;
		}

		@Override
		public String name() {
			return name;
		}

		@Override
		public void lower(double lower) {
			if (Double.compare(this.lower, lower) == 0)
				return;
			this.lower = lower;
			if (index >= 0)
				owner.markBoundsDirty(index);
		}

		@Override
		public void upper(double upper) {
			if (Double.compare(this.upper, upper) == 0)
				return;
			this.upper = upper;
			if (index >= 0)
				owner.markBoundsDirty(index);
		}

		@Override
		public double lowerBound() {
			return lower;
		}

		@Override
		public double upperBound() {
			return upper;
		}

		@Override
		public void setInteger(boolean integer) {
			if (this.integer == integer)
				return;
			this.integer = integer;
			owner.markStructureDirty();
		}

		@Override
		public boolean isInteger() {
			return integer;
		}
	}

	private final class HighsLpExpression implements LpExpression {
		private final Map<Integer, Double> coefficients = new LinkedHashMap<>();
		@SuppressWarnings("unused")
		private final String name;
		private boolean hasLower;
		private boolean hasUpper;
		private double lower;
		private double upper;
		private double weight;

		private HighsLpExpression(String name) {
			this.name = name;
		}

		@Override
		public void set(LpVariable var, double coefficient) {
			HighsLpVariable variable = unwrap(var);
			if (variable.owner != HighsLpModel.this)
				throw new IllegalArgumentException("LP variable belongs to a different model");
			if (variable.index < 0)
				throw new IllegalStateException("LP variable must be added to the model before it is used");
			if (Math.abs(coefficient) < 1e-12)
				coefficients.remove(variable.index);
			else
				coefficients.put(variable.index, coefficient);
			markStructureDirty();
		}

		@Override
		public void lower(double lower) {
			this.lower = lower;
			this.hasLower = true;
			markStructureDirty();
		}

		@Override
		public void upper(double upper) {
			this.upper = upper;
			this.hasUpper = true;
			markStructureDirty();
		}

		@Override
		public void level(double level) {
			this.lower = level;
			this.upper = level;
			this.hasLower = true;
			this.hasUpper = true;
			markStructureDirty();
		}

		@Override
		public void weight(double weight) {
			this.weight = weight;
			markStructureDirty();
		}
	}
}
