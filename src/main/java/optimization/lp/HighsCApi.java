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

import java.io.IOException;
import java.lang.foreign.AddressLayout;
import java.lang.foreign.Arena;
import java.lang.foreign.FunctionDescriptor;
import java.lang.foreign.Linker;
import java.lang.foreign.MemorySegment;
import java.lang.foreign.SymbolLookup;
import java.lang.foreign.ValueLayout;
import java.lang.invoke.MethodHandle;

import org.scijava.nativelib.NativeLoader;

final class HighsCApi {

	static final ValueLayout.OfDouble C_DOUBLE = ValueLayout.JAVA_DOUBLE;
	static final ValueLayout.OfLong C_LONG_LONG = ValueLayout.JAVA_LONG;
	static final AddressLayout C_POINTER = ValueLayout.ADDRESS;

	static final long STATUS_ERROR = -1L;
	static final long STATUS_OK = 0L;
	static final long STATUS_WARNING = 1L;

	static final long OBJ_SENSE_MINIMIZE = 1L;
	static final long OBJ_SENSE_MAXIMIZE = -1L;
	static final long MATRIX_FORMAT_COLWISE = 1L;
	static final long VAR_TYPE_CONTINUOUS = 0L;
	static final long VAR_TYPE_INTEGER = 1L;

	static final long MODEL_STATUS_MODEL_EMPTY = 6L;
	static final long MODEL_STATUS_OPTIMAL = 7L;
	static final long MODEL_STATUS_INFEASIBLE = 8L;
	static final long MODEL_STATUS_UNBOUNDED_OR_INFEASIBLE = 9L;
	static final long MODEL_STATUS_UNBOUNDED = 10L;
	static final long MODEL_STATUS_OBJECTIVE_BOUND = 11L;
	static final long MODEL_STATUS_OBJECTIVE_TARGET = 12L;
	static final long MODEL_STATUS_TIME_LIMIT = 13L;
	static final long MODEL_STATUS_ITERATION_LIMIT = 14L;
	static final long MODEL_STATUS_UNKNOWN = 15L;
	static final long MODEL_STATUS_SOLUTION_LIMIT = 16L;
	static final long MODEL_STATUS_INTERRUPT = 17L;
	static final long SOLUTION_STATUS_NONE = 0L;
	static final long SOLUTION_STATUS_INFEASIBLE = 1L;
	static final long SOLUTION_STATUS_FEASIBLE = 2L;

	private static final Linker LINKER;
	private static final SymbolLookup SYMBOLS;

	private static final MethodHandle HIGHS_CREATE;
	private static final MethodHandle HIGHS_DESTROY;
	private static final MethodHandle HIGHS_CLEAR_MODEL;
	private static final MethodHandle HIGHS_PASS_LP;
	private static final MethodHandle HIGHS_PASS_MIP;
	private static final MethodHandle HIGHS_RUN;
	private static final MethodHandle HIGHS_SET_BOOL_OPTION_VALUE;
	private static final MethodHandle HIGHS_SET_DOUBLE_OPTION_VALUE;
	private static final MethodHandle HIGHS_GET_MODEL_STATUS;
	private static final MethodHandle HIGHS_GET_DOUBLE_INFO_VALUE;
	private static final MethodHandle HIGHS_GET_INT_INFO_VALUE;
	private static final MethodHandle HIGHS_GET_SOLUTION;
	private static final MethodHandle HIGHS_CHANGE_COL_BOUNDS;

	static {
		try {
			NativeLoader.loadLibrary("highs");
		} catch (IOException e) {
			throw new ExceptionInInitializerError(e);
		}
		LINKER = Linker.nativeLinker();
		SYMBOLS = SymbolLookup.loaderLookup().or(LINKER.defaultLookup());
		HIGHS_CREATE = downcall("Highs_create", FunctionDescriptor.of(C_POINTER));
		HIGHS_DESTROY = downcall("Highs_destroy", FunctionDescriptor.ofVoid(C_POINTER));
		HIGHS_CLEAR_MODEL = downcall("Highs_clearModel", FunctionDescriptor.of(C_LONG_LONG, C_POINTER));
		HIGHS_PASS_LP = downcall("Highs_passLp",
				FunctionDescriptor.of(C_LONG_LONG, C_POINTER, C_LONG_LONG, C_LONG_LONG, C_LONG_LONG, C_LONG_LONG, C_LONG_LONG, C_DOUBLE,
						C_POINTER, C_POINTER, C_POINTER, C_POINTER, C_POINTER, C_POINTER, C_POINTER, C_POINTER));
		HIGHS_PASS_MIP = downcall("Highs_passMip",
				FunctionDescriptor.of(C_LONG_LONG, C_POINTER, C_LONG_LONG, C_LONG_LONG, C_LONG_LONG, C_LONG_LONG, C_LONG_LONG, C_DOUBLE,
						C_POINTER, C_POINTER, C_POINTER, C_POINTER, C_POINTER, C_POINTER, C_POINTER, C_POINTER, C_POINTER));
		HIGHS_RUN = downcall("Highs_run", FunctionDescriptor.of(C_LONG_LONG, C_POINTER));
		HIGHS_SET_BOOL_OPTION_VALUE = downcall("Highs_setBoolOptionValue", FunctionDescriptor.of(C_LONG_LONG, C_POINTER, C_POINTER, C_LONG_LONG));
		HIGHS_SET_DOUBLE_OPTION_VALUE = downcall("Highs_setDoubleOptionValue",
				FunctionDescriptor.of(C_LONG_LONG, C_POINTER, C_POINTER, C_DOUBLE));
		HIGHS_GET_MODEL_STATUS = downcall("Highs_getModelStatus", FunctionDescriptor.of(C_LONG_LONG, C_POINTER));
		HIGHS_GET_DOUBLE_INFO_VALUE = downcall("Highs_getDoubleInfoValue", FunctionDescriptor.of(C_LONG_LONG, C_POINTER, C_POINTER, C_POINTER));
		HIGHS_GET_INT_INFO_VALUE = downcall("Highs_getIntInfoValue", FunctionDescriptor.of(C_LONG_LONG, C_POINTER, C_POINTER, C_POINTER));
		HIGHS_GET_SOLUTION = downcall("Highs_getSolution",
				FunctionDescriptor.of(C_LONG_LONG, C_POINTER, C_POINTER, C_POINTER, C_POINTER, C_POINTER));
		HIGHS_CHANGE_COL_BOUNDS = downcall("Highs_changeColBounds",
				FunctionDescriptor.of(C_LONG_LONG, C_POINTER, C_LONG_LONG, C_DOUBLE, C_DOUBLE));
	}

	private HighsCApi() {
	}

	static MemorySegment create() {
		try {
			return (MemorySegment) HIGHS_CREATE.invokeExact();
		} catch (Throwable e) {
			throw new RuntimeException("Unable to create HiGHS model", e);
		}
	}

	static void destroy(MemorySegment highs) {
		try {
			HIGHS_DESTROY.invokeExact(highs);
		} catch (Throwable e) {
			throw new RuntimeException("Unable to destroy HiGHS model", e);
		}
	}

	static long clearModel(MemorySegment highs) {
		try {
			return (long) HIGHS_CLEAR_MODEL.invokeExact(highs);
		} catch (Throwable e) {
			throw new RuntimeException("Unable to clear HiGHS model", e);
		}
	}

	static long passLp(MemorySegment highs, long numCol, long numRow, long numNz, long aFormat, long sense, double offset, MemorySegment colCost,
			MemorySegment colLower, MemorySegment colUpper, MemorySegment rowLower, MemorySegment rowUpper, MemorySegment aStart, MemorySegment aIndex,
			MemorySegment aValue) {
		try {
			return (long) HIGHS_PASS_LP.invokeExact(highs, numCol, numRow, numNz, aFormat, sense, offset, colCost, colLower, colUpper, rowLower,
					rowUpper, aStart, aIndex, aValue);
		} catch (Throwable e) {
			throw new RuntimeException("Unable to pass LP model to HiGHS", e);
		}
	}

	static long passMip(MemorySegment highs, long numCol, long numRow, long numNz, long aFormat, long sense, double offset, MemorySegment colCost,
			MemorySegment colLower, MemorySegment colUpper, MemorySegment rowLower, MemorySegment rowUpper, MemorySegment aStart, MemorySegment aIndex,
			MemorySegment aValue, MemorySegment integrality) {
		try {
			return (long) HIGHS_PASS_MIP.invokeExact(highs, numCol, numRow, numNz, aFormat, sense, offset, colCost, colLower, colUpper, rowLower,
					rowUpper, aStart, aIndex, aValue, integrality);
		} catch (Throwable e) {
			throw new RuntimeException("Unable to pass MIP model to HiGHS", e);
		}
	}

	static long run(MemorySegment highs) {
		try {
			return (long) HIGHS_RUN.invokeExact(highs);
		} catch (Throwable e) {
			throw new RuntimeException("Unable to run HiGHS", e);
		}
	}

	static long setBoolOption(MemorySegment highs, Arena arena, String option, boolean value) {
		MemorySegment optionSegment = arena.allocateFrom(option);
		try {
			return (long) HIGHS_SET_BOOL_OPTION_VALUE.invokeExact(highs, optionSegment, value ? 1L : 0L);
		} catch (Throwable e) {
			throw new RuntimeException("Unable to set HiGHS bool option " + option, e);
		}
	}

	static long setDoubleOption(MemorySegment highs, Arena arena, String option, double value) {
		MemorySegment optionSegment = arena.allocateFrom(option);
		try {
			return (long) HIGHS_SET_DOUBLE_OPTION_VALUE.invokeExact(highs, optionSegment, value);
		} catch (Throwable e) {
			throw new RuntimeException("Unable to set HiGHS double option " + option, e);
		}
	}

	static long getModelStatus(MemorySegment highs) {
		try {
			return (long) HIGHS_GET_MODEL_STATUS.invokeExact(highs);
		} catch (Throwable e) {
			throw new RuntimeException("Unable to get HiGHS model status", e);
		}
	}

	static double getDoubleInfoValue(MemorySegment highs, Arena arena, String infoName, double fallbackValue) {
		MemorySegment infoSegment = arena.allocateFrom(infoName);
		MemorySegment valueSegment = arena.allocate(C_DOUBLE);
		try {
			long status = (long) HIGHS_GET_DOUBLE_INFO_VALUE.invokeExact(highs, infoSegment, valueSegment);
			if (!isOkOrWarning(status))
				return fallbackValue;
			return valueSegment.get(C_DOUBLE, 0L);
		} catch (Throwable e) {
			throw new RuntimeException("Unable to get HiGHS double info " + infoName, e);
		}
	}

	static long getIntInfoValue(MemorySegment highs, Arena arena, String infoName, long fallbackValue) {
		MemorySegment infoSegment = arena.allocateFrom(infoName);
		MemorySegment valueSegment = arena.allocate(C_LONG_LONG);
		try {
			long status = (long) HIGHS_GET_INT_INFO_VALUE.invokeExact(highs, infoSegment, valueSegment);
			if (!isOkOrWarning(status))
				return fallbackValue;
			return valueSegment.get(C_LONG_LONG, 0L);
		} catch (Throwable e) {
			throw new RuntimeException("Unable to get HiGHS int info " + infoName, e);
		}
	}

	static long getSolution(MemorySegment highs, MemorySegment colValue, MemorySegment colDual, MemorySegment rowValue, MemorySegment rowDual) {
		try {
			return (long) HIGHS_GET_SOLUTION.invokeExact(highs, colValue, colDual, rowValue, rowDual);
		} catch (Throwable e) {
			throw new RuntimeException("Unable to get HiGHS solution", e);
		}
	}

	static long changeColBounds(MemorySegment highs, long column, double lower, double upper) {
		try {
			return (long) HIGHS_CHANGE_COL_BOUNDS.invokeExact(highs, column, lower, upper);
		} catch (Throwable e) {
			throw new RuntimeException("Unable to change HiGHS column bounds", e);
		}
	}

	static boolean isOkOrWarning(long status) {
		return status == STATUS_OK || status == STATUS_WARNING;
	}

	private static MethodHandle downcall(String symbolName, FunctionDescriptor descriptor) {
		return LINKER.downcallHandle(SYMBOLS.findOrThrow(symbolName), descriptor);
	}
}
