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
import java.lang.invoke.MethodHandle;
import java.lang.reflect.Array;
import java.lang.reflect.Field;
import java.lang.reflect.Method;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.util.Optional;

import org.scijava.nativelib.NativeLoader;

final class HighsCApi {

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

	private static final Object C_DOUBLE;
	private static final Object C_LONG_LONG;
	private static final Object C_POINTER;
	private static final Object NULL_SEGMENT;

	private static final Object LINKER;
	private static final Object LOADER_LOOKUP;
	private static final Object DEFAULT_LOOKUP;
	private static final Method FIND_METHOD;
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

	private static final Method ARENA_ALLOCATE_SEQUENCE;
	private static final Method ARENA_ALLOCATE_STRING;
	private static final Method SEGMENT_AS_BYTE_BUFFER;

	static {
		try {
			NativeLoader.loadLibrary("highs");

			Class<?> memoryLayoutClass = Class.forName("java.lang.foreign.MemoryLayout");
			Class<?> valueLayoutClass = Class.forName("java.lang.foreign.ValueLayout");
			Class<?> functionDescriptorClass = Class.forName("java.lang.foreign.FunctionDescriptor");
			Class<?> linkerClass = Class.forName("java.lang.foreign.Linker");
			Class<?> symbolLookupClass = Class.forName("java.lang.foreign.SymbolLookup");
			Class<?> memorySegmentClass = Class.forName("java.lang.foreign.MemorySegment");
			Class<?> arenaClass = Class.forName("java.lang.foreign.Arena");

			C_DOUBLE = field(valueLayoutClass, "JAVA_DOUBLE");
			C_LONG_LONG = field(valueLayoutClass, "JAVA_LONG");
			C_POINTER = field(valueLayoutClass, "ADDRESS");
			NULL_SEGMENT = field(memorySegmentClass, "NULL");

			LINKER = invokeStatic(linkerClass, "nativeLinker");
			LOADER_LOOKUP = invokeStatic(symbolLookupClass, "loaderLookup");
			DEFAULT_LOOKUP = linkerClass.getMethod("defaultLookup").invoke(LINKER);
			FIND_METHOD = symbolLookupClass.getMethod("find", String.class);

			Class<?> memoryLayoutArrayClass = Array.newInstance(memoryLayoutClass, 0).getClass();
			Method downcallHandle = findMethodAtLeast(linkerClass, "downcallHandle", 2);

			ARENA_ALLOCATE_SEQUENCE = findAllocateSequenceMethod(arenaClass, memoryLayoutClass);
			ARENA_ALLOCATE_STRING = findAllocateStringMethod(arenaClass);
			SEGMENT_AS_BYTE_BUFFER = memorySegmentClass.getMethod("asByteBuffer");

			HIGHS_CREATE = downcall(downcallHandle, "Highs_create", functionDescriptor(C_POINTER));
			HIGHS_DESTROY = downcall(downcallHandle, "Highs_destroy", voidFunctionDescriptor(C_POINTER));
			HIGHS_CLEAR_MODEL = downcall(downcallHandle, "Highs_clearModel", functionDescriptor(C_LONG_LONG, C_POINTER));
			HIGHS_PASS_LP = downcall(downcallHandle, "Highs_passLp",
					functionDescriptor(C_LONG_LONG, C_POINTER, C_LONG_LONG, C_LONG_LONG, C_LONG_LONG, C_LONG_LONG, C_LONG_LONG, C_DOUBLE,
							C_POINTER, C_POINTER, C_POINTER, C_POINTER, C_POINTER, C_POINTER, C_POINTER, C_POINTER));
			HIGHS_PASS_MIP = downcall(downcallHandle, "Highs_passMip",
					functionDescriptor(C_LONG_LONG, C_POINTER, C_LONG_LONG, C_LONG_LONG, C_LONG_LONG, C_LONG_LONG, C_LONG_LONG, C_DOUBLE,
							C_POINTER, C_POINTER, C_POINTER, C_POINTER, C_POINTER, C_POINTER, C_POINTER, C_POINTER, C_POINTER));
			HIGHS_RUN = downcall(downcallHandle, "Highs_run", functionDescriptor(C_LONG_LONG, C_POINTER));
			HIGHS_SET_BOOL_OPTION_VALUE = downcall(downcallHandle, "Highs_setBoolOptionValue", functionDescriptor(C_LONG_LONG, C_POINTER, C_POINTER, C_LONG_LONG));
			HIGHS_SET_DOUBLE_OPTION_VALUE = downcall(downcallHandle, "Highs_setDoubleOptionValue", functionDescriptor(C_LONG_LONG, C_POINTER, C_POINTER, C_DOUBLE));
			HIGHS_GET_MODEL_STATUS = downcall(downcallHandle, "Highs_getModelStatus", functionDescriptor(C_LONG_LONG, C_POINTER));
			HIGHS_GET_DOUBLE_INFO_VALUE = downcall(downcallHandle, "Highs_getDoubleInfoValue", functionDescriptor(C_LONG_LONG, C_POINTER, C_POINTER, C_POINTER));
			HIGHS_GET_INT_INFO_VALUE = downcall(downcallHandle, "Highs_getIntInfoValue", functionDescriptor(C_LONG_LONG, C_POINTER, C_POINTER, C_POINTER));
			HIGHS_GET_SOLUTION = downcall(downcallHandle, "Highs_getSolution", functionDescriptor(C_LONG_LONG, C_POINTER, C_POINTER, C_POINTER, C_POINTER, C_POINTER));
			HIGHS_CHANGE_COL_BOUNDS = downcall(downcallHandle, "Highs_changeColBounds", functionDescriptor(C_LONG_LONG, C_POINTER, C_LONG_LONG, C_DOUBLE, C_DOUBLE));
		} catch (Throwable e) {
			throw new ExceptionInInitializerError(e);
		}
	}

	private HighsCApi() {
	}

	static final class NativeArena implements AutoCloseable {
		private final Object arena;

		private NativeArena(Object arena) {
			this.arena = arena;
		}

		@Override
		public void close() {
			try {
				if (arena instanceof AutoCloseable)
					((AutoCloseable) arena).close();
				else
					invoke(arena, "close");
			} catch (Exception e) {
				throw new RuntimeException("Unable to close foreign arena", e);
			}
		}
	}

	static NativeArena openArena() {
		try {
			Object arena;
			try {
				Class<?> arenaClass = Class.forName("java.lang.foreign.Arena");
				arena = invokeStatic(arenaClass, "ofConfined");
			} catch (ReflectiveOperationException e) {
				Class<?> arenaClass = Class.forName("java.lang.foreign.Arena");
				arena = invokeStatic(arenaClass, "openConfined");
			}
			return new NativeArena(arena);
		} catch (ReflectiveOperationException e) {
			throw new RuntimeException("Unable to open foreign arena", e);
		}
	}

	static Object create() {
		try {
			return HIGHS_CREATE.invokeWithArguments();
		} catch (Throwable e) {
			throw new RuntimeException("Unable to create HiGHS model", e);
		}
	}

	static void destroy(Object highs) {
		try {
			HIGHS_DESTROY.invokeWithArguments(highs);
		} catch (Throwable e) {
			throw new RuntimeException("Unable to destroy HiGHS model", e);
		}
	}

	static long clearModel(Object highs) {
		try {
			return ((Number) HIGHS_CLEAR_MODEL.invokeWithArguments(highs)).longValue();
		} catch (Throwable e) {
			throw new RuntimeException("Unable to clear HiGHS model", e);
		}
	}

	static long passLp(Object highs, long numCol, long numRow, long numNz, long aFormat, long sense, double offset, Object colCost, Object colLower,
			Object colUpper, Object rowLower, Object rowUpper, Object aStart, Object aIndex, Object aValue) {
		try {
			return ((Number) HIGHS_PASS_LP.invokeWithArguments(highs, numCol, numRow, numNz, aFormat, sense, offset, colCost, colLower, colUpper, rowLower,
					rowUpper, aStart, aIndex, aValue)).longValue();
		} catch (Throwable e) {
			throw new RuntimeException("Unable to pass LP model to HiGHS", e);
		}
	}

	static long passMip(Object highs, long numCol, long numRow, long numNz, long aFormat, long sense, double offset, Object colCost, Object colLower,
			Object colUpper, Object rowLower, Object rowUpper, Object aStart, Object aIndex, Object aValue, Object integrality) {
		try {
			return ((Number) HIGHS_PASS_MIP.invokeWithArguments(highs, numCol, numRow, numNz, aFormat, sense, offset, colCost, colLower, colUpper, rowLower,
					rowUpper, aStart, aIndex, aValue, integrality)).longValue();
		} catch (Throwable e) {
			throw new RuntimeException("Unable to pass MIP model to HiGHS", e);
		}
	}

	static long run(Object highs) {
		try {
			return ((Number) HIGHS_RUN.invokeWithArguments(highs)).longValue();
		} catch (Throwable e) {
			throw new RuntimeException("Unable to run HiGHS", e);
		}
	}

	static long setBoolOption(Object highs, NativeArena arena, String option, boolean value) {
		Object optionSegment = allocateString(arena, option);
		try {
			return ((Number) HIGHS_SET_BOOL_OPTION_VALUE.invokeWithArguments(highs, optionSegment, value ? 1L : 0L)).longValue();
		} catch (Throwable e) {
			throw new RuntimeException("Unable to set HiGHS bool option " + option, e);
		}
	}

	static long setDoubleOption(Object highs, NativeArena arena, String option, double value) {
		Object optionSegment = allocateString(arena, option);
		try {
			return ((Number) HIGHS_SET_DOUBLE_OPTION_VALUE.invokeWithArguments(highs, optionSegment, value)).longValue();
		} catch (Throwable e) {
			throw new RuntimeException("Unable to set HiGHS double option " + option, e);
		}
	}

	static long getModelStatus(Object highs) {
		try {
			return ((Number) HIGHS_GET_MODEL_STATUS.invokeWithArguments(highs)).longValue();
		} catch (Throwable e) {
			throw new RuntimeException("Unable to get HiGHS model status", e);
		}
	}

	static double getDoubleInfoValue(Object highs, NativeArena arena, String infoName, double fallbackValue) {
		Object infoSegment = allocateString(arena, infoName);
		Object valueSegment = allocateDoubles(arena, 1);
		try {
			long status = ((Number) HIGHS_GET_DOUBLE_INFO_VALUE.invokeWithArguments(highs, infoSegment, valueSegment)).longValue();
			if (!isOkOrWarning(status))
				return fallbackValue;
			return readDoubles(valueSegment, 1)[0];
		} catch (Throwable e) {
			throw new RuntimeException("Unable to get HiGHS double info " + infoName, e);
		}
	}

	static long getIntInfoValue(Object highs, NativeArena arena, String infoName, long fallbackValue) {
		Object infoSegment = allocateString(arena, infoName);
		Object valueSegment = allocateLongs(arena, 1);
		try {
			long status = ((Number) HIGHS_GET_INT_INFO_VALUE.invokeWithArguments(highs, infoSegment, valueSegment)).longValue();
			if (!isOkOrWarning(status))
				return fallbackValue;
			return readLongs(valueSegment, 1)[0];
		} catch (Throwable e) {
			throw new RuntimeException("Unable to get HiGHS int info " + infoName, e);
		}
	}

	static long getSolution(Object highs, Object colValue, Object colDual, Object rowValue, Object rowDual) {
		try {
			return ((Number) HIGHS_GET_SOLUTION.invokeWithArguments(highs, colValue, colDual, rowValue, rowDual)).longValue();
		} catch (Throwable e) {
			throw new RuntimeException("Unable to get HiGHS solution", e);
		}
	}

	static long changeColBounds(Object highs, long column, double lower, double upper) {
		try {
			return ((Number) HIGHS_CHANGE_COL_BOUNDS.invokeWithArguments(highs, column, lower, upper)).longValue();
		} catch (Throwable e) {
			throw new RuntimeException("Unable to change HiGHS column bounds", e);
		}
	}

	static Object allocateDoubles(NativeArena arena, int size) {
		return size == 0 ? NULL_SEGMENT : allocateArray(arena, C_DOUBLE, size);
	}

	static Object allocateDoubles(NativeArena arena, double[] values) {
		if (values.length == 0)
			return NULL_SEGMENT;
		Object segment = allocateArray(arena, C_DOUBLE, values.length);
		ByteBuffer buffer = asNativeByteBuffer(segment);
		for (double value : values)
			buffer.putDouble(value);
		return segment;
	}

	static Object allocateLongs(NativeArena arena, long[] values) {
		if (values.length == 0)
			return NULL_SEGMENT;
		Object segment = allocateArray(arena, C_LONG_LONG, values.length);
		ByteBuffer buffer = asNativeByteBuffer(segment);
		for (long value : values)
			buffer.putLong(value);
		return segment;
	}

	static Object allocateLongs(NativeArena arena, int size) {
		return size == 0 ? NULL_SEGMENT : allocateArray(arena, C_LONG_LONG, size);
	}

	static double[] readDoubles(Object segment, int size) {
		double[] values = new double[size];
		ByteBuffer buffer = asNativeByteBuffer(segment);
		for (int i = 0; i < size; i++)
			values[i] = buffer.getDouble();
		return values;
	}

	static long[] readLongs(Object segment, int size) {
		long[] values = new long[size];
		ByteBuffer buffer = asNativeByteBuffer(segment);
		for (int i = 0; i < size; i++)
			values[i] = buffer.getLong();
		return values;
	}

	static boolean isOkOrWarning(long status) {
		return status == STATUS_OK || status == STATUS_WARNING;
	}

	private static Object allocateArray(NativeArena arena, Object layout, int size) {
		try {
			return ARENA_ALLOCATE_SEQUENCE.invoke(arena.arena, layout, (long) size);
		} catch (ReflectiveOperationException | IllegalArgumentException e) {
			throw new RuntimeException("Unable to allocate foreign memory array", e);
		}
	}

	private static Object allocateString(NativeArena arena, String value) {
		try {
			return ARENA_ALLOCATE_STRING.invoke(arena.arena, value);
		} catch (ReflectiveOperationException | IllegalArgumentException e) {
			throw new RuntimeException("Unable to allocate foreign string", e);
		}
	}

	private static ByteBuffer asNativeByteBuffer(Object segment) {
		try {
			return ((ByteBuffer) SEGMENT_AS_BYTE_BUFFER.invoke(segment)).order(ByteOrder.nativeOrder());
		} catch (ReflectiveOperationException | IllegalArgumentException e) {
			throw new RuntimeException("Unable to access foreign memory as ByteBuffer", e);
		}
	}

	private static MethodHandle downcall(Method downcallHandle, String symbolName, Object descriptor) throws ReflectiveOperationException {
		Object symbol = findSymbol(symbolName);
		if (symbol == null)
			throw new IllegalStateException("Unable to find symbol " + symbolName);
		Class<?>[] parameterTypes = downcallHandle.getParameterTypes();
		if (parameterTypes.length == 2)
			return (MethodHandle) downcallHandle.invoke(LINKER, symbol, descriptor);
		Object emptyOptions = Array.newInstance(parameterTypes[2].getComponentType(), 0);
		return (MethodHandle) downcallHandle.invoke(LINKER, symbol, descriptor, emptyOptions);
	}

	private static Object functionDescriptor(Object returnLayout, Object... argumentLayouts) throws ReflectiveOperationException {
		Class<?> memoryLayoutClass = Class.forName("java.lang.foreign.MemoryLayout");
		Class<?> memoryLayoutArrayClass = Array.newInstance(memoryLayoutClass, 0).getClass();
		Method fdOf = Class.forName("java.lang.foreign.FunctionDescriptor").getMethod("of", memoryLayoutClass, memoryLayoutArrayClass);
		Object layouts = Array.newInstance(memoryLayoutClass, argumentLayouts.length);
		for (int i = 0; i < argumentLayouts.length; i++)
			Array.set(layouts, i, argumentLayouts[i]);
		return fdOf.invoke(null, returnLayout, layouts);
	}

	private static Object voidFunctionDescriptor(Object... argumentLayouts) throws ReflectiveOperationException {
		Class<?> memoryLayoutClass = Class.forName("java.lang.foreign.MemoryLayout");
		Class<?> memoryLayoutArrayClass = Array.newInstance(memoryLayoutClass, 0).getClass();
		Method fdOfVoid = Class.forName("java.lang.foreign.FunctionDescriptor").getMethod("ofVoid", memoryLayoutArrayClass);
		Object layouts = Array.newInstance(memoryLayoutClass, argumentLayouts.length);
		for (int i = 0; i < argumentLayouts.length; i++)
			Array.set(layouts, i, argumentLayouts[i]);
		return fdOfVoid.invoke(null, layouts);
	}

	private static Object findSymbol(String symbolName) throws ReflectiveOperationException {
		Object symbol = find(LOADER_LOOKUP, symbolName);
		return symbol != null ? symbol : find(DEFAULT_LOOKUP, symbolName);
	}

	private static Object find(Object lookup, String symbolName) throws ReflectiveOperationException {
		@SuppressWarnings("unchecked")
		Optional<Object> optional = (Optional<Object>) FIND_METHOD.invoke(lookup, symbolName);
		return optional.isPresent() ? optional.get() : null;
	}

	private static Method findMethod(Class<?> type, int parameterCount, String... names) {
		for (Method method : type.getMethods()) {
			for (String name : names) {
				if (method.getName().equals(name) && method.getParameterCount() == parameterCount)
					return method;
			}
		}
		throw new IllegalStateException("Method not found: " + type.getName() + " " + String.join("/", names) + " with " + parameterCount + " params");
	}

	private static Method findAllocateSequenceMethod(Class<?> arenaClass, Class<?> memoryLayoutClass) {
		for (String name : new String[] { "allocateArray", "allocate" }) {
			for (Method method : arenaClass.getMethods()) {
				if (!method.getName().equals(name))
					continue;
				Class<?>[] parameterTypes = method.getParameterTypes();
				if (parameterTypes.length != 2)
					continue;
				boolean layoutCompatible = parameterTypes[0].isAssignableFrom(memoryLayoutClass) || memoryLayoutClass.isAssignableFrom(parameterTypes[0]);
				boolean countCompatible = parameterTypes[1] == long.class;
				if (layoutCompatible && countCompatible)
					return method;
			}
		}
		throw new IllegalStateException("Unable to find foreign arena allocation method");
	}

	private static Method findAllocateStringMethod(Class<?> arenaClass) {
		for (String name : new String[] { "allocateFrom", "allocateUtf8String" }) {
			for (Method method : arenaClass.getMethods()) {
				if (!method.getName().equals(name))
					continue;
				Class<?>[] parameterTypes = method.getParameterTypes();
				if (parameterTypes.length == 1 && parameterTypes[0] == String.class)
					return method;
			}
		}
		throw new IllegalStateException("Unable to find foreign arena string allocation method");
	}

	private static Method findMethodAtLeast(Class<?> type, String name, int minParameterCount) {
		for (Method method : type.getMethods()) {
			if (method.getName().equals(name) && method.getParameterCount() >= minParameterCount)
				return method;
		}
		throw new IllegalStateException("Method not found: " + type.getName() + "." + name + " with at least " + minParameterCount + " params");
	}

	private static Object invokeStatic(Class<?> type, String name) throws ReflectiveOperationException {
		return type.getMethod(name).invoke(null);
	}

	private static Object invoke(Object target, String name) throws ReflectiveOperationException {
		return target.getClass().getMethod(name).invoke(target);
	}

	private static Object field(Class<?> type, String name) throws ReflectiveOperationException {
		Field field = type.getField(name);
		return field.get(null);
	}

	static Object nullSegment() {
		return NULL_SEGMENT;
	}
}
