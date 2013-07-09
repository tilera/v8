// Copyright 2012 the V8 project authors. All rights reserved.
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met:
//
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above
//       copyright notice, this list of conditions and the following
//       disclaimer in the documentation and/or other materials provided
//       with the distribution.
//     * Neither the name of Google Inc. nor the names of its
//       contributors may be used to endorse or promote products derived
//       from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.



#include "v8.h"

#if defined(V8_TARGET_ARCH_TILEGX)

#include "codegen.h"
#include "debug.h"
#include "deoptimizer.h"
#include "full-codegen.h"
#include "runtime.h"

namespace v8 {
namespace internal {


#define __ ACCESS_MASM(masm)


static void GenerateMakeCodeYoungAgainCommon(MacroAssembler* masm) {
  printf("[%s:%d]\n", __FUNCTION__, __LINE__);
  abort();
}

#define DEFINE_CODE_AGE_BUILTIN_GENERATOR(C)                 \
void Builtins::Generate_Make##C##CodeYoungAgainEvenMarking(  \
    MacroAssembler* masm) {                                  \
  GenerateMakeCodeYoungAgainCommon(masm);                    \
}                                                            \
void Builtins::Generate_Make##C##CodeYoungAgainOddMarking(   \
    MacroAssembler* masm) {                                  \
  GenerateMakeCodeYoungAgainCommon(masm);                    \
}
CODE_AGE_LIST(DEFINE_CODE_AGE_BUILTIN_GENERATOR)
#undef DEFINE_CODE_AGE_BUILTIN_GENERATOR

// FIXME
void Builtins::Generate_Adaptor(MacroAssembler* masm,
                                CFunctionId id,
                                BuiltinExtraArguments extra_args) {
  // ----------- S t a t e -------------
  //  -- a0                 : number of arguments excluding receiver
  //  -- a1                 : called function (only guaranteed when
  //  --                      extra_args requires it)
  //  -- cp                 : context
  //  -- sp[0]              : last argument
  //  -- ...
  //  -- sp[4 * (argc - 1)] : first argument
  //  -- sp[4 * agrc]       : receiver
  // -----------------------------------

  // Insert extra arguments.
  int num_extra_args = 0;
  if (extra_args == NEEDS_CALLED_FUNCTION) {
    num_extra_args = 1;
    __ push(a1);
  } else {
    ASSERT(extra_args == NO_EXTRA_ARGUMENTS);
  }

  // JumpToExternalReference expects s0 to contain the number of arguments
  // including the receiver and the extra arguments.
  __ Addu(s0, a0, num_extra_args + 1);
  __ sll(s1, s0, kPointerSizeLog2);
  __ Subu(s1, s1, kPointerSize);
  __ JumpToExternalReference(ExternalReference(id, masm->isolate()));
}

static void Generate_JSConstructStubHelper(MacroAssembler* masm,
                                           bool is_api_function,
                                           bool count_constructions) {
  // ----------- S t a t e -------------
  //  -- a0     : number of arguments
  //  -- a1     : constructor function
  //  -- ra     : return address
  //  -- sp[...]: constructor arguments
  // -----------------------------------

  // Should never count constructions for api objects.
  ASSERT(!is_api_function || !count_constructions);

  Isolate* isolate = masm->isolate();

  // ----------- S t a t e -------------
  //  -- a0     : number of arguments
  //  -- a1     : constructor function
  //  -- ra     : return address
  //  -- sp[...]: constructor arguments
  // -----------------------------------

  // Enter a construct frame.
  {
    FrameScope scope(masm, StackFrame::CONSTRUCT);

    // Preserve the two incoming parameters on the stack.
    __ sll(a0, a0, kSmiTagSize);  // Tag arguments count.
    __ MultiPushReversed(a0.bit() | a1.bit());

    // Use t7 to hold undefined, which is used in several places below.
    __ LoadRoot(t7, Heap::kUndefinedValueRootIndex);

    Label rt_call, allocated;
    // Try to allocate the object without transitioning into C code. If any of
    // the preconditions is not met, the code bails out to the runtime call.
    if (FLAG_inline_new) {
      Label undo_allocation;
#ifdef ENABLE_DEBUGGER_SUPPORT
      ExternalReference debug_step_in_fp =
          ExternalReference::debug_step_in_fp_address(isolate);
      __ li(a2, Operand(debug_step_in_fp));
      __ ld(a2, MemOperand(a2));
      __ Branch(&rt_call, ne, a2, Operand(zero));
#endif

      // Load the initial map and verify that it is in fact a map.
      // a1: constructor function
      __ ld(a2, FieldMemOperand(a1, JSFunction::kPrototypeOrInitialMapOffset));
      __ JumpIfSmi(a2, &rt_call);
      __ GetObjectType(a2, a3, t4);
      __ Branch(&rt_call, ne, t4, Operand(MAP_TYPE));

      // Check that the constructor is not constructing a JSFunction (see
      // comments in Runtime_NewObject in runtime.cc). In which case the
      // initial map's instance type would be JS_FUNCTION_TYPE.
      // a1: constructor function
      // a2: initial map
      __ ld1u(a3, FieldMemOperand(a2, Map::kInstanceTypeOffset));
      __ Branch(&rt_call, eq, a3, Operand(JS_FUNCTION_TYPE));

      if (count_constructions) {
        Label allocate;
        // Decrease generous allocation count.
        __ ld(a3, FieldMemOperand(a1, JSFunction::kSharedFunctionInfoOffset));
        MemOperand constructor_count =
           FieldMemOperand(a3, SharedFunctionInfo::kConstructionCountOffset);
        __ ld1u(t0, constructor_count);
        __ Subu(t0, t0, Operand(1));
        __ st1(t0, constructor_count);
        __ Branch(&allocate, ne, t0, Operand(zero));

        __ Push(a1, a2);

        __ push(a1);  // Constructor.
        // The call will replace the stub, so the countdown is only done once.
        __ CallRuntime(Runtime::kFinalizeInstanceSize, 1);

        __ pop(a2);
        __ pop(a1);

        __ bind(&allocate);
      }

      // Now allocate the JSObject on the heap.
      // a1: constructor function
      // a2: initial map
      __ ld1u(a3, FieldMemOperand(a2, Map::kInstanceSizeOffset));
      __ Allocate(a3, t4, t5, t6, &rt_call, SIZE_IN_WORDS);

      // Allocated the JSObject, now initialize the fields. Map is set to
      // initial map and properties and elements are set to empty fixed array.
      // a1: constructor function
      // a2: initial map
      // a3: object size
      // t4: JSObject (not tagged)
      __ LoadRoot(t6, Heap::kEmptyFixedArrayRootIndex);
      __ move(t5, t4);
      __ st(a2, MemOperand(t5, JSObject::kMapOffset));
      __ st(t6, MemOperand(t5, JSObject::kPropertiesOffset));
      __ st(t6, MemOperand(t5, JSObject::kElementsOffset));
      __ Addu(t5, t5, Operand(3*kPointerSize));
      ASSERT_EQ(0 * kPointerSize, JSObject::kMapOffset);
      ASSERT_EQ(1 * kPointerSize, JSObject::kPropertiesOffset);
      ASSERT_EQ(2 * kPointerSize, JSObject::kElementsOffset);

      // Fill all the in-object properties with appropriate filler.
      // a1: constructor function
      // a2: initial map
      // a3: object size (in words)
      // t4: JSObject (not tagged)
      // t5: First in-object property of JSObject (not tagged)
      __ sll(t0, a3, kPointerSizeLog2);
      __ add(t6, t4, t0);   // End of object.
      ASSERT_EQ(3 * kPointerSize, JSObject::kHeaderSize);
      __ LoadRoot(t7, Heap::kUndefinedValueRootIndex);
      if (count_constructions) {
        __ ld(a0, FieldMemOperand(a2, Map::kInstanceSizesOffset));
        __ bfextu(a0, a0, Map::kPreAllocatedPropertyFieldsByte * kBitsPerByte, Map::kPreAllocatedPropertyFieldsByte * kBitsPerByte + kBitsPerByte - 1);
        __ sll(t0, a0, kPointerSizeLog2);
        __ add(a0, t5, t0);
        // a0: offset of first field after pre-allocated fields
        if (FLAG_debug_code) {
          __ Assert(le, "Unexpected number of pre-allocated property fields.",
              a0, Operand(t6));
        }
        __ InitializeFieldsWithFiller(t5, a0, t7);
        // To allow for truncation.
        __ LoadRoot(t7, Heap::kOnePointerFillerMapRootIndex);
      }
      __ InitializeFieldsWithFiller(t5, t6, t7);

      // Add the object tag to make the JSObject real, so that we can continue
      // and jump into the continuation code at any time from now on. Any
      // failures need to undo the allocation, so that the heap is in a
      // consistent state and verifiable.
      __ Addu(t4, t4, Operand(kHeapObjectTag));

      // Check if a non-empty properties array is needed. Continue with
      // allocated object if not fall through to runtime call if it is.
      // a1: constructor function
      // t4: JSObject
      // t5: start of next object (not tagged)
      __ ld1u(a3, FieldMemOperand(a2, Map::kUnusedPropertyFieldsOffset));
      // The field instance sizes contains both pre-allocated property fields
      // and in-object properties.
      __ ld(a0, FieldMemOperand(a2, Map::kInstanceSizesOffset));
      __ bfextu(t6, a0, Map::kPreAllocatedPropertyFieldsByte * kBitsPerByte, Map::kPreAllocatedPropertyFieldsByte * kBitsPerByte + kBitsPerByte -1);
      __ Addu(a3, a3, Operand(t6));
      __ bfextu(t6, a0, Map::kInObjectPropertiesByte * kBitsPerByte, Map::kInObjectPropertiesByte * kBitsPerByte + kBitsPerByte - 1);
      __ sub(a3, a3, t6);

      // Done if no extra properties are to be allocated.
      __ Branch(&allocated, eq, a3, Operand(zero));
      __ Assert(greater_equal, "Property allocation count failed.",
          a3, Operand(zero));

      // Scale the number of elements by pointer size and add the header for
      // FixedArrays to the start of the next object calculation from above.
      // a1: constructor
      // a3: number of elements in properties array
      // t4: JSObject
      // t5: start of next object
      __ Addu(a0, a3, Operand(FixedArray::kHeaderSize / kPointerSize));
      __ Allocate(
          a0,
          t5,
          t6,
          a2,
          &undo_allocation,
          static_cast<AllocationFlags>(RESULT_CONTAINS_TOP | SIZE_IN_WORDS));

      // Initialize the FixedArray.
      // a1: constructor
      // a3: number of elements in properties array (untagged)
      // t4: JSObject
      // t5: start of next object
      __ LoadRoot(t6, Heap::kFixedArrayMapRootIndex);
      __ move(a2, t5);
      __ st(t6, MemOperand(a2, JSObject::kMapOffset));
      __ sll(a0, a3, kSmiTagSize);
      __ st(a0, MemOperand(a2, FixedArray::kLengthOffset));
      __ Addu(a2, a2, Operand(2 * kPointerSize));

      ASSERT_EQ(0 * kPointerSize, JSObject::kMapOffset);
      ASSERT_EQ(1 * kPointerSize, FixedArray::kLengthOffset);

      // Initialize the fields to undefined.
      // a1: constructor
      // a2: First element of FixedArray (not tagged)
      // a3: number of elements in properties array
      // t4: JSObject
      // t5: FixedArray (not tagged)
      __ sll(t3, a3, kPointerSizeLog2);
      __ add(t6, a2, t3);  // End of object.
      ASSERT_EQ(2 * kPointerSize, FixedArray::kHeaderSize);
      { Label loop, entry;
        if (count_constructions) {
          __ LoadRoot(t7, Heap::kUndefinedValueRootIndex);
        } else if (FLAG_debug_code) {
          __ LoadRoot(t8, Heap::kUndefinedValueRootIndex);
          __ Assert(eq, "Undefined value not loaded.", t7, Operand(t8));
        }
        __ jmp(&entry);
        __ bind(&loop);
        __ st(t7, MemOperand(a2));
        __ addi(a2, a2, kPointerSize);
        __ bind(&entry);
        __ Branch(&loop, less, a2, Operand(t6));
      }

      // Store the initialized FixedArray into the properties field of
      // the JSObject.
      // a1: constructor function
      // t4: JSObject
      // t5: FixedArray (not tagged)
      __ Addu(t5, t5, Operand(kHeapObjectTag));  // Add the heap tag.
      __ st(t5, FieldMemOperand(t4, JSObject::kPropertiesOffset));

      // Continue with JSObject being successfully allocated.
      // a1: constructor function
      // a4: JSObject
      __ jmp(&allocated);

      // Undo the setting of the new top so that the heap is verifiable. For
      // example, the map's unused properties potentially do not match the
      // allocated objects unused properties.
      // t4: JSObject (previous new top)
      __ bind(&undo_allocation);
      __ UndoAllocationInNewSpace(t4, t5);
    }

    __ bind(&rt_call);
    // Allocate the new receiver object using the runtime call.
    // a1: constructor function
    __ push(a1);  // Argument for Runtime_NewObject.
    __ CallRuntime(Runtime::kNewObject, 1);
    __ move(t4, r0);

    // Receiver for constructor call allocated.
    // t4: JSObject
    __ bind(&allocated);
    __ push(t4);
    __ push(t4);

    // Reload the number of arguments from the stack.
    // sp[0]: receiver
    // sp[1]: receiver
    // sp[2]: constructor function
    // sp[3]: number of arguments (smi-tagged)
    __ ld(a1, MemOperand(sp, 2 * kPointerSize));
    __ ld(a3, MemOperand(sp, 3 * kPointerSize));

    // Set up pointer to last argument.
    __ Addu(a2, fp, Operand(StandardFrameConstants::kCallerSPOffset));

    // Set up number of arguments for function call below.
    __ srl(a0, a3, kSmiTagSize);

    // Copy arguments and receiver to the expression stack.
    // a0: number of arguments
    // a1: constructor function
    // a2: address of last argument (caller sp)
    // a3: number of arguments (smi-tagged)
    // sp[0]: receiver
    // sp[1]: receiver
    // sp[2]: constructor function
    // sp[3]: number of arguments (smi-tagged)
    Label loop, entry;
    __ jmp(&entry);
    __ bind(&loop);
    __ sll(t0, a3, kPointerSizeLog2 - kSmiTagSize);
    __ Addu(t0, a2, Operand(t0));
    __ ld(t1, MemOperand(t0));
    __ push(t1);
    __ bind(&entry);
    __ Addu(a3, a3, Operand(-2));
    __ Branch(&loop, greater_equal, a3, Operand(zero));

    // Call the function.
    // a0: number of arguments
    // a1: constructor function
    if (is_api_function) {
      __ ld(cp, FieldMemOperand(a1, JSFunction::kContextOffset));
      Handle<Code> code =
          masm->isolate()->builtins()->HandleApiCallConstruct();
      ParameterCount expected(0);
      __ InvokeCode(code, expected, expected,
                    RelocInfo::CODE_TARGET, CALL_FUNCTION, CALL_AS_METHOD);
    } else {
      ParameterCount actual(a0);
      __ InvokeFunction(a1, actual, CALL_FUNCTION,
                        NullCallWrapper(), CALL_AS_METHOD);
    }

    // Store offset of return address for deoptimizer.
    if (!is_api_function && !count_constructions) {
      masm->isolate()->heap()->SetConstructStubDeoptPCOffset(masm->pc_offset());
    }

    // Restore context from the frame.
    __ ld(cp, MemOperand(fp, StandardFrameConstants::kContextOffset));

    // If the result is an object (in the ECMA sense), we should get rid
    // of the receiver and use the result; see ECMA-262 section 13.2.2-7
    // on page 74.
    Label use_receiver, exit;

    // If the result is a smi, it is *not* an object in the ECMA sense.
    // v0: result
    // sp[0]: receiver (newly allocated object)
    // sp[1]: constructor function
    // sp[2]: number of arguments (smi-tagged)
    __ JumpIfSmi(r0, &use_receiver);

    // If the type of the result (stored in its map) is less than
    // FIRST_SPEC_OBJECT_TYPE, it is not an object in the ECMA sense.
    __ GetObjectType(r0, a1, a3);
    __ Branch(&exit, greater_equal, a3, Operand(FIRST_SPEC_OBJECT_TYPE));

    // Throw away the result of the constructor invocation and use the
    // on-stack receiver as the result.
    __ bind(&use_receiver);
    __ ld(r0, MemOperand(sp));

    // Remove receiver from the stack, remove caller arguments, and
    // return.
    __ bind(&exit);
    // v0: result
    // sp[0]: receiver (newly allocated object)
    // sp[1]: constructor function
    // sp[2]: number of arguments (smi-tagged)
    __ ld(a1, MemOperand(sp, 2 * kPointerSize));

    // Leave construct frame.
  }

  __ sll(t0, a1, kPointerSizeLog2 - 1);
  __ Addu(sp, sp, t0);
  __ Addu(sp, sp, kPointerSize);
  __ IncrementCounter(isolate->counters()->constructed_objects(), 1, a1, a2);
  __ Ret();
}

void Builtins::Generate_JSConstructStubCountdown(MacroAssembler* masm) {
  Generate_JSConstructStubHelper(masm, false, true);
}


void Builtins::Generate_JSConstructStubGeneric(MacroAssembler* masm) {
  Generate_JSConstructStubHelper(masm, false, false);
}


void Builtins::Generate_JSConstructStubApi(MacroAssembler* masm) {
  Generate_JSConstructStubHelper(masm, true, false);
}

static void GenerateTailCallToSharedCode(MacroAssembler* masm) {
  __ ld(a2, FieldMemOperand(a1, JSFunction::kSharedFunctionInfoOffset));
  __ ld(a2, FieldMemOperand(a2, SharedFunctionInfo::kCodeOffset));
  __ Addu(tt, a2, Operand(Code::kHeaderSize - kHeapObjectTag));
  __ Jump(tt);
}

void Builtins::Generate_InRecompileQueue(MacroAssembler* masm) {
  GenerateTailCallToSharedCode(masm);
}

void Builtins::Generate_InstallRecompiledCode(MacroAssembler* masm) {
  // Enter an internal frame.
  {
    FrameScope scope(masm, StackFrame::INTERNAL);

    // Preserve the function.
    __ push(a1);
    // Push call kind information.
    __ push(t1);

    // Push the function on the stack as the argument to the runtime function.
    __ push(a1);
    __ CallRuntime(Runtime::kInstallRecompiledCode, 1);
    // Calculate the entry point.
    __ Addu(tt, r0, Operand(Code::kHeaderSize - kHeapObjectTag));

    // Restore call kind information.
    __ pop(t1);
    // Restore saved function.
    __ pop(a1);

    // Tear down temporary frame.
  }

  // Do a tail-call of the compiled function.
  __ Jump(tt);
}

static void EnterArgumentsAdaptorFrame(MacroAssembler* masm) {
  __ sll(a0, a0, kSmiTagSize);
  __ li(t0, Operand(Smi::FromInt(StackFrame::ARGUMENTS_ADAPTOR)));
  __ MultiPush(a0.bit() | a1.bit() | t0.bit() | fp.bit() | lr.bit());
  __ Addu(fp, sp, Operand(3 * kPointerSize));
}

static void LeaveArgumentsAdaptorFrame(MacroAssembler* masm) {
  // ----------- S t a t e -------------
  //  -- v0 : result being passed through
  // -----------------------------------
  // Get the number of arguments passed (as a smi), tear down the frame and
  // then tear down the parameters.
  __ ld(a1, MemOperand(fp, -3 * kPointerSize));
  __ move(sp, fp);
  __ MultiPop(fp.bit() | lr.bit());
  __ sll(t0, a1, kPointerSizeLog2 - kSmiTagSize);
  __ Addu(sp, sp, t0);
  // Adjust for the receiver.
  __ Addu(sp, sp, Operand(kPointerSize));
}

void Builtins::Generate_ArgumentsAdaptorTrampoline(MacroAssembler* masm) {
  // State setup as expected by MacroAssembler::InvokePrologue.
  // ----------- S t a t e -------------
  //  -- a0: actual arguments count
  //  -- a1: function (passed through to callee)
  //  -- a2: expected arguments count
  //  -- a3: callee code entry
  //  -- t1: call kind information
  // -----------------------------------

  Label invoke, dont_adapt_arguments;

  Label enough, too_few;
  __ Branch(&dont_adapt_arguments, eq,
      a2, Operand(SharedFunctionInfo::kDontAdaptArgumentsSentinel));
  // We use Uless as the number of argument should always be greater than 0.
  __ Branch(&too_few, Uless, a0, Operand(a2));

  {  // Enough parameters: actual >= expected.
    // a0: actual number of arguments as a smi
    // a1: function
    // a2: expected number of arguments
    // a3: code entry to call
    __ bind(&enough);
    EnterArgumentsAdaptorFrame(masm);

    // Calculate copy start address into a0 and copy end address into a2.
    __ sll(a0, a0, kPointerSizeLog2 - kSmiTagSize);
    __ Addu(a0, fp, a0);
    // Adjust for return address and receiver.
    __ Addu(a0, a0, Operand(2 * kPointerSize));
    // Compute copy end address.
    __ sll(a2, a2, kPointerSizeLog2);
    __ sub(a2, a0, a2);

    // Copy the arguments (including the receiver) to the new stack frame.
    // a0: copy start address
    // a1: function
    // a2: copy end address
    // a3: code entry to call

    Label copy;
    __ bind(&copy);
    __ ld(t0, MemOperand(a0));
    __ push(t0);
    __ Branch(&copy, ne, a0, Operand(a2));
    __ addi(a0, a0, -kPointerSize);  // In delay slot.

    __ jmp(&invoke);
  }

  {  // Too few parameters: Actual < expected.
    __ bind(&too_few);
    EnterArgumentsAdaptorFrame(masm);

    // Calculate copy start address into a0 and copy end address is fp.
    // a0: actual number of arguments as a smi
    // a1: function
    // a2: expected number of arguments
    // a3: code entry to call
    __ sll(a0, a0, kPointerSizeLog2 - kSmiTagSize);
    __ Addu(a0, fp, a0);
    // Adjust for return address and receiver.
    __ Addu(a0, a0, Operand(2 * kPointerSize));
    // Compute copy end address. Also adjust for return address.
    __ Addu(t3, fp, kPointerSize);

    // Copy the arguments (including the receiver) to the new stack frame.
    // a0: copy start address
    // a1: function
    // a2: expected number of arguments
    // a3: code entry to call
    // t3: copy end address
    Label copy;
    __ bind(&copy);
    __ ld(t0, MemOperand(a0));  // Adjusted above for return addr and receiver.
    __ Subu(sp, sp, kPointerSize);
    __ Subu(a0, a0, kPointerSize);
    __ Branch(&copy, ne, a0, Operand(t3));
    __ st(t0, MemOperand(sp));  // In the delay slot.

    // Fill the remaining expected arguments with undefined.
    // a1: function
    // a2: expected number of arguments
    // a3: code entry to call
    __ LoadRoot(t0, Heap::kUndefinedValueRootIndex);
    __ sll(t2, a2, kPointerSizeLog2);
    __ Subu(a2, fp, Operand(t2));
    __ Addu(a2, a2, Operand(-4 * kPointerSize));  // Adjust for frame.

    Label fill;
    __ bind(&fill);
    __ Subu(sp, sp, kPointerSize);
    __ Branch(&fill, ne, sp, Operand(a2));
    __ st(t0, MemOperand(sp));
  }

  // Call the entry point.
  __ bind(&invoke);

  __ Call(a3);

  // Store offset of return address for deoptimizer.
  masm->isolate()->heap()->SetArgumentsAdaptorDeoptPCOffset(masm->pc_offset());

  // Exit frame and return.
  LeaveArgumentsAdaptorFrame(masm);
  __ Ret();


  // -------------------------------------------
  // Don't adapt arguments.
  // -------------------------------------------
  __ bind(&dont_adapt_arguments);
  __ Jump(a3);
}

void Builtins::Generate_NotifyDeoptimized(MacroAssembler* masm) {
  printf("[%s:%d]\n", __FUNCTION__, __LINE__);
  abort();
}

void Builtins::Generate_ParallelRecompile(MacroAssembler* masm) {
  printf("[%s:%d]\n", __FUNCTION__, __LINE__);
  abort();
}

void Builtins::Generate_LazyRecompile(MacroAssembler* masm) {
  printf("[%s:%d]\n", __FUNCTION__, __LINE__);
  abort();
}

void Builtins::Generate_LazyCompile(MacroAssembler* masm) {
  printf("[%s:%d]\n", __FUNCTION__, __LINE__);
  abort();
}

void Builtins::Generate_JSConstructEntryTrampoline(MacroAssembler* masm) {
  printf("[%s:%d]\n", __FUNCTION__, __LINE__);
  abort();
}

void Builtins::Generate_JSEntryTrampoline(MacroAssembler* masm) {
  printf("[%s:%d]\n", __FUNCTION__, __LINE__);
  abort();
}

void Builtins::Generate_NotifySoftDeoptimized(MacroAssembler* masm) {
  printf("[%s:%d]\n", __FUNCTION__, __LINE__);
  abort();
}

void Builtins::Generate_NotifyLazyDeoptimized(MacroAssembler* masm) {
  printf("[%s:%d]\n", __FUNCTION__, __LINE__);
  abort();
}

void Builtins::Generate_NotifyStubFailure(MacroAssembler* masm) {
  printf("[%s:%d]\n", __FUNCTION__, __LINE__);
  abort();
}

void Builtins::Generate_NotifyOSR(MacroAssembler* masm) {
  printf("[%s:%d]\n", __FUNCTION__, __LINE__);
  abort();
}

void Builtins::Generate_FunctionCall(MacroAssembler* masm) {
  printf("[%s:%d]\n", __FUNCTION__, __LINE__);
  abort();
}

void Builtins::Generate_FunctionApply(MacroAssembler* masm) {
  printf("[%s:%d]\n", __FUNCTION__, __LINE__);
  abort();
}

void Builtins::Generate_InternalArrayCode(MacroAssembler* masm) {
  printf("[%s:%d]\n", __FUNCTION__, __LINE__);
  abort();
}

void Builtins::Generate_ArrayCode(MacroAssembler* masm) {
  printf("[%s:%d]\n", __FUNCTION__, __LINE__);
  abort();
}

void Builtins::Generate_CommonArrayConstructCode(MacroAssembler* masm) {
  printf("[%s:%d]\n", __FUNCTION__, __LINE__);
  abort();
}

void Builtins::Generate_StringConstructCode(MacroAssembler* masm) {
  printf("[%s:%d]\n", __FUNCTION__, __LINE__);
  abort();
}

void Builtins::Generate_OnStackReplacement(MacroAssembler* masm) {
  printf("[%s:%d]\n", __FUNCTION__, __LINE__);
  abort();
}
#undef __

} }  // namespace v8::internal

#endif  // V8_TARGET_ARCH_MIPS
