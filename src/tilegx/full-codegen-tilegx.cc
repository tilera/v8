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

// Note on Mips implementation:
//
// The result_register() for mips is the 'v0' register, which is defined
// by the ABI to contain function return values. However, the first
// parameter to a function is defined to be 'a0'. So there are many
// places where we have to move a previous result in v0 to a0 for the
// next call: move(a0, v0). This is not needed on the other architectures.

#include "code-stubs.h"
#include "codegen.h"
#include "compiler.h"
#include "debug.h"
#include "full-codegen.h"
#include "isolate-inl.h"
#include "parser.h"
#include "scopes.h"
#include "stub-cache.h"

#include "tilegx/code-stubs-tilegx.h"
#include "tilegx/macro-assembler-tilegx.h"

namespace v8 {
namespace internal {

#define __ ACCESS_MASM(masm_)


// A patch site is a location in the code which it is possible to patch. This
// class has a number of methods to emit the code which is patchable and the
// method EmitPatchInfo to record a marker back to the patchable code. This
// marker is a andi zero, rx, #yyyy instruction, and rx * 0x0000ffff + yyyy
// (raw 16 bit immediate value is used) is the delta from the pc to the first
// instruction of the patchable code.
// The marker instruction is effectively a NOP (dest is zero) and will
// never be emitted by normal code.
class JumpPatchSite BASE_EMBEDDED {
 public:
  explicit JumpPatchSite(MacroAssembler* masm) : masm_(masm) {
#ifdef DEBUG
    info_emitted_ = false;
#endif
  }

  ~JumpPatchSite() {
    ASSERT(patch_site_.is_bound() == info_emitted_);
  }

  // When initially emitting this ensure that a jump is always generated to skip
  // the inlined smi code.
  void EmitJumpIfNotSmi(Register reg, Label* target) {
    ASSERT(!patch_site_.is_bound() && !info_emitted_);
    Assembler::BlockTrampolinePoolScope block_trampoline_pool(masm_);
    __ bind(&patch_site_);
    __ andi(at, reg, 0);
    // Always taken before patched.
    __ Branch(target, eq, at, Operand(zero));
  }

  // When initially emitting this ensure that a jump is never generated to skip
  // the inlined smi code.
  void EmitJumpIfSmi(Register reg, Label* target) {
    Assembler::BlockTrampolinePoolScope block_trampoline_pool(masm_);
    ASSERT(!patch_site_.is_bound() && !info_emitted_);
    __ bind(&patch_site_);
    __ andi(at, reg, 0);
    // Never taken before patched.
    __ Branch(target, ne, at, Operand(zero));
  }

  void EmitPatchInfo() {
    if (patch_site_.is_bound()) {
      int delta_to_patch_site = masm_->InstructionsGeneratedSince(&patch_site_);
      Register reg = Register::from_code(delta_to_patch_site / kImm16Mask);
      __ andi(zero, reg, delta_to_patch_site % kImm16Mask);
#ifdef DEBUG
      info_emitted_ = true;
#endif
    } else {
      __ nop();  // Signals no inlined code.
    }
  }

 private:
  MacroAssembler* masm_;
  Label patch_site_;
#ifdef DEBUG
  bool info_emitted_;
#endif
};


// Generate code for a JS function.  On entry to the function the receiver
// and arguments have been pushed on the stack left to right.  The actual
// argument count matches the formal parameter count expected by the
// function.
//
// The live registers are:
//   o a1: the JS function object being called (i.e. ourselves)
//   o cp: our context
//   o fp: our caller's frame pointer
//   o sp: stack pointer
//   o ra: return address
//
// The function builds a JS frame.  Please see JavaScriptFrameConstants in
// frames-mips.h for its layout.
void FullCodeGenerator::Generate() {
  CompilationInfo* info = info_;
  handler_table_ =
      isolate()->factory()->NewFixedArray(function()->handler_count(), TENURED);
  profiling_counter_ = isolate()->factory()->NewJSGlobalPropertyCell(
      Handle<Smi>(Smi::FromInt(FLAG_interrupt_budget), isolate()));
  SetFunctionPosition(function());
  Comment cmnt(masm_, "[ function compiled by full code generator");

  ProfileEntryHookStub::MaybeCallEntryHook(masm_);

#ifdef DEBUG
  if (strlen(FLAG_stop_at) > 0 &&
      info->function()->name()->IsUtf8EqualTo(CStrVector(FLAG_stop_at))) {
    __ stop("stop-at");
  }
#endif

  // Strict mode functions and builtins need to replace the receiver
  // with undefined when called as functions (without an explicit
  // receiver object). t1 is zero for method calls and non-zero for
  // function calls.
  if (!info->is_classic_mode() || info->is_native()) {
    Label ok;
    __ Branch(&ok, eq, t1, Operand(zero));
    int receiver_offset = info->scope()->num_parameters() * kPointerSize;
    __ LoadRoot(a2, Heap::kUndefinedValueRootIndex);
    __ st(a2, MemOperand(sp, receiver_offset));
    __ bind(&ok);
  }

  // Open a frame scope to indicate that there is a frame on the stack.  The
  // MANUAL indicates that the scope shouldn't actually generate code to set up
  // the frame (that is done below).
  FrameScope frame_scope(masm_, StackFrame::MANUAL);

  info->set_prologue_offset(masm_->pc_offset());
  // The following three instructions must remain together and unmodified for
  // code aging to work properly.
  __ Push(ra, fp, cp, a1);
  // Load undefined value here, so the value is ready for the loop
  // below.
  __ LoadRoot(at, Heap::kUndefinedValueRootIndex);
  // Adjust fp to point to caller's fp.
  __ Addu(fp, sp, Operand(2 * kPointerSize));
  info->AddNoFrameRange(0, masm_->pc_offset());

  { Comment cmnt(masm_, "[ Allocate locals");
    int locals_count = info->scope()->num_stack_slots();
    // Generators allocate locals, if any, in context slots.
    ASSERT(!info->function()->is_generator() || locals_count == 0);
    for (int i = 0; i < locals_count; i++) {
      __ push(at);
    }
  }

  bool function_in_register = true;

  // Possibly allocate a local context.
  int heap_slots = info->scope()->num_heap_slots() - Context::MIN_CONTEXT_SLOTS;
  if (heap_slots > 0) {
    Comment cmnt(masm_, "[ Allocate context");
    // Argument to NewContext is the function, which is still in a1.
    __ push(a1);
    if (FLAG_harmony_scoping && info->scope()->is_global_scope()) {
      __ Push(info->scope()->GetScopeInfo());
      __ CallRuntime(Runtime::kNewGlobalContext, 2);
    } else if (heap_slots <= FastNewContextStub::kMaximumSlots) {
      FastNewContextStub stub(heap_slots);
      __ CallStub(&stub);
    } else {
      __ CallRuntime(Runtime::kNewFunctionContext, 1);
    }
    function_in_register = false;
    // Context is returned in both v0 and cp.  It replaces the context
    // passed to us.  It's saved in the stack and kept live in cp.
    __ st(cp, MemOperand(fp, StandardFrameConstants::kContextOffset));
    // Copy any necessary parameters into the context.
    int num_parameters = info->scope()->num_parameters();
    for (int i = 0; i < num_parameters; i++) {
      Variable* var = scope()->parameter(i);
      if (var->IsContextSlot()) {
        int parameter_offset = StandardFrameConstants::kCallerSPOffset +
                                 (num_parameters - 1 - i) * kPointerSize;
        // Load parameter from stack.
        __ ld(a0, MemOperand(fp, parameter_offset));
        // Store it in the context.
        MemOperand target = ContextOperand(cp, var->index());
        __ st(a0, target);

        // Update the write barrier.
        __ RecordWriteContextSlot(
            cp, target.offset(), a0, a3, kRAHasBeenSaved, kDontSaveFPRegs);
      }
    }
  }

  Variable* arguments = scope()->arguments();
  if (arguments != NULL) {
    // Function uses arguments object.
    Comment cmnt(masm_, "[ Allocate arguments object");
    if (!function_in_register) {
      // Load this again, if it's used by the local context below.
      __ ld(a3, MemOperand(fp, JavaScriptFrameConstants::kFunctionOffset));
    } else {
      __ move(a3, a1);
    }
    // Receiver is just before the parameters on the caller's stack.
    int num_parameters = info->scope()->num_parameters();
    int offset = num_parameters * kPointerSize;
    __ Addu(a2, fp,
           Operand(StandardFrameConstants::kCallerSPOffset + offset));
    __ li(a1, Operand(Smi::FromInt(num_parameters)));
    __ Push(a3, a2, a1);

    // Arguments to ArgumentsAccessStub:
    //   function, receiver address, parameter count.
    // The stub will rewrite receiever and parameter count if the previous
    // stack frame was an arguments adapter frame.
    ArgumentsAccessStub::Type type;
    if (!is_classic_mode()) {
      type = ArgumentsAccessStub::NEW_STRICT;
    } else if (function()->has_duplicate_parameters()) {
      type = ArgumentsAccessStub::NEW_NON_STRICT_SLOW;
    } else {
      type = ArgumentsAccessStub::NEW_NON_STRICT_FAST;
    }
    ArgumentsAccessStub stub(type);
    __ CallStub(&stub);

    SetVar(arguments, v0, a1, a2);
  }

  if (FLAG_trace) {
    __ CallRuntime(Runtime::kTraceEnter, 0);
  }

  // Visit the declarations and body unless there is an illegal
  // redeclaration.
  if (scope()->HasIllegalRedeclaration()) {
    Comment cmnt(masm_, "[ Declarations");
    scope()->VisitIllegalRedeclaration(this);

  } else {
    PrepareForBailoutForId(BailoutId::FunctionEntry(), NO_REGISTERS);
    { Comment cmnt(masm_, "[ Declarations");
      // For named function expressions, declare the function name as a
      // constant.
      if (scope()->is_function_scope() && scope()->function() != NULL) {
        VariableDeclaration* function = scope()->function();
        ASSERT(function->proxy()->var()->mode() == CONST ||
               function->proxy()->var()->mode() == CONST_HARMONY);
        ASSERT(function->proxy()->var()->location() != Variable::UNALLOCATED);
        VisitVariableDeclaration(function);
      }
      VisitDeclarations(scope()->declarations());
    }

    { Comment cmnt(masm_, "[ Stack check");
      PrepareForBailoutForId(BailoutId::Declarations(), NO_REGISTERS);
      Label ok;
      __ LoadRoot(t0, Heap::kStackLimitRootIndex);
      __ Branch(&ok, hs, sp, Operand(t0));
      StackCheckStub stub;
      __ CallStub(&stub);
      __ bind(&ok);
    }

    { Comment cmnt(masm_, "[ Body");
      ASSERT(loop_depth() == 0);
      VisitStatements(function()->body());
      ASSERT(loop_depth() == 0);
    }
  }

  // Always emit a 'return undefined' in case control fell off the end of
  // the body.
  { Comment cmnt(masm_, "[ return <undefined>;");
    __ LoadRoot(v0, Heap::kUndefinedValueRootIndex);
  }
  EmitReturnSequence();
}


void FullCodeGenerator::ClearAccumulator() {
  ASSERT(Smi::FromInt(0) == 0);
  __ move(v0, zero);
}


void FullCodeGenerator::EmitProfilingCounterDecrement(int delta) {
  __ li(a2, Operand(profiling_counter_));
  __ ld(a3, FieldMemOperand(a2, JSGlobalPropertyCell::kValueOffset));
  __ Subu(a3, a3, Operand(Smi::FromInt(delta)));
  __ st(a3, FieldMemOperand(a2, JSGlobalPropertyCell::kValueOffset));
}


void FullCodeGenerator::EmitProfilingCounterReset() {
  int reset_value = FLAG_interrupt_budget;
  if (info_->ShouldSelfOptimize() && !FLAG_retry_self_opt) {
    // Self-optimization is a one-off thing: if it fails, don't try again.
    reset_value = Smi::kMaxValue;
  }
  if (isolate()->IsDebuggerActive()) {
    // Detect debug break requests as soon as possible.
    reset_value = FLAG_interrupt_budget >> 4;
  }
  __ li(a2, Operand(profiling_counter_));
  __ li(a3, Operand(Smi::FromInt(reset_value)));
  __ st(a3, FieldMemOperand(a2, JSGlobalPropertyCell::kValueOffset));
}


void FullCodeGenerator::EmitBackEdgeBookkeeping(IterationStatement* stmt,
                                                Label* back_edge_target) {
  // The generated code is used in Deoptimizer::PatchStackCheckCodeAt so we need
  // to make sure it is constant. Branch may emit a skip-or-jump sequence
  // instead of the normal Branch. It seems that the "skip" part of that
  // sequence is about as long as this Branch would be so it is safe to ignore
  // that.
  Assembler::BlockTrampolinePoolScope block_trampoline_pool(masm_);
  Comment cmnt(masm_, "[ Back edge bookkeeping");
  Label ok;
  int weight = 1;
  if (FLAG_weighted_back_edges) {
    ASSERT(back_edge_target->is_bound());
    int distance = masm_->SizeOfCodeGeneratedSince(back_edge_target);
    weight = Min(kMaxBackEdgeWeight,
                 Max(1, distance / kBackEdgeDistanceUnit));
  }
  EmitProfilingCounterDecrement(weight);
  __ cmplts(at, a3, zero);
  __ beqz(at, &ok);
  // CallStub will emit a li t9 first, so it is safe to use the delay slot.
  InterruptStub stub;
  __ CallStub(&stub);
  // Record a mapping of this PC offset to the OSR id.  This is used to find
  // the AST id from the unoptimized code in order to use it as a key into
  // the deoptimization input data found in the optimized code.
  RecordBackEdge(stmt->OsrEntryId());
  EmitProfilingCounterReset();

  __ bind(&ok);
  PrepareForBailoutForId(stmt->EntryId(), NO_REGISTERS);
  // Record a mapping of the OSR id to this PC.  This is used if the OSR
  // entry becomes the target of a bailout.  We don't expect it to be, but
  // we want it to work if it is.
  PrepareForBailoutForId(stmt->OsrEntryId(), NO_REGISTERS);
}


void FullCodeGenerator::EmitReturnSequence() {
  Comment cmnt(masm_, "[ Return sequence");
  if (return_label_.is_bound()) {
    __ Branch(&return_label_);
  } else {
    __ bind(&return_label_);
    if (FLAG_trace) {
      // Push the return value on the stack as the parameter.
      // Runtime::TraceExit returns its parameter in v0.
      __ push(v0);
      __ CallRuntime(Runtime::kTraceExit, 1);
    }
    if (FLAG_interrupt_at_exit || FLAG_self_optimization) {
      // Pretend that the exit is a backwards jump to the entry.
      int weight = 1;
      if (info_->ShouldSelfOptimize()) {
        weight = FLAG_interrupt_budget / FLAG_self_opt_count;
      } else if (FLAG_weighted_back_edges) {
        int distance = masm_->pc_offset();
        weight = Min(kMaxBackEdgeWeight,
                     Max(1, distance / kBackEdgeDistanceUnit));
      }
      EmitProfilingCounterDecrement(weight);
      Label ok;
      __ Branch(&ok, ge, a3, Operand(zero));
      __ push(v0);
      if (info_->ShouldSelfOptimize() && FLAG_direct_self_opt) {
        __ ld(a2, MemOperand(fp, JavaScriptFrameConstants::kFunctionOffset));
        __ push(a2);
        __ CallRuntime(Runtime::kOptimizeFunctionOnNextCall, 1);
      } else {
        InterruptStub stub;
        __ CallStub(&stub);
      }
      __ pop(v0);
      EmitProfilingCounterReset();
      __ bind(&ok);
    }

#ifdef DEBUG
    // Add a label for checking the size of the code used for returning.
    Label check_exit_codesize;
    masm_->bind(&check_exit_codesize);
#endif
    // Make sure that the constant pool is not emitted inside of the return
    // sequence.
    { Assembler::BlockTrampolinePoolScope block_trampoline_pool(masm_);
      // Here we use masm_-> instead of the __ macro to avoid the code coverage
      // tool from instrumenting as we rely on the code size here.
      int32_t sp_delta = (info_->scope()->num_parameters() + 1) * kPointerSize;
      CodeGenerator::RecordPositions(masm_, function()->end_position() - 1);
      __ RecordJSReturn();
      masm_->move(sp, fp);
      int no_frame_start = masm_->pc_offset();
      masm_->MultiPop(static_cast<RegList>(fp.bit() | ra.bit()));
      masm_->Addu(sp, sp, Operand(sp_delta));
      masm_->Jump(ra);
      info_->AddNoFrameRange(no_frame_start, masm_->pc_offset());
    }

#ifdef DEBUG
    // Check that the size of the code used for returning is large enough
    // for the debugger's requirements.
    ASSERT(Assembler::kJSReturnSequenceInstructions <=
           masm_->InstructionsGeneratedSince(&check_exit_codesize));
#endif
  }
}


void FullCodeGenerator::EffectContext::Plug(Variable* var) const {
  ASSERT(var->IsStackAllocated() || var->IsContextSlot());
}


void FullCodeGenerator::AccumulatorValueContext::Plug(Variable* var) const {
  ASSERT(var->IsStackAllocated() || var->IsContextSlot());
  codegen()->GetVar(result_register(), var);
}


void FullCodeGenerator::StackValueContext::Plug(Variable* var) const {
  ASSERT(var->IsStackAllocated() || var->IsContextSlot());
  codegen()->GetVar(result_register(), var);
  __ push(result_register());
}


void FullCodeGenerator::TestContext::Plug(Variable* var) const {
  // For simplicity we always test the accumulator register.
  codegen()->GetVar(result_register(), var);
  codegen()->PrepareForBailoutBeforeSplit(condition(), false, NULL, NULL);
  codegen()->DoTest(this);
}


void FullCodeGenerator::EffectContext::Plug(Heap::RootListIndex index) const { UNREACHABLE(); }


void FullCodeGenerator::AccumulatorValueContext::Plug(
    Heap::RootListIndex index) const {
  __ LoadRoot(result_register(), index);
}


void FullCodeGenerator::StackValueContext::Plug(
    Heap::RootListIndex index) const {
#if 0
  __ LoadRoot(result_register(), index);
  __ push(result_register());
#else
    UNREACHABLE();
#endif
}


void FullCodeGenerator::TestContext::Plug(Heap::RootListIndex index) const {
  codegen()->PrepareForBailoutBeforeSplit(condition(),
                                          true,
                                          true_label_,
                                          false_label_);
  if (index == Heap::kUndefinedValueRootIndex ||
      index == Heap::kNullValueRootIndex ||
      index == Heap::kFalseValueRootIndex) {
    if (false_label_ != fall_through_) __ Branch(false_label_);
  } else if (index == Heap::kTrueValueRootIndex) {
    if (true_label_ != fall_through_) __ Branch(true_label_);
  } else {
    __ LoadRoot(result_register(), index);
    codegen()->DoTest(this);
  }
}


void FullCodeGenerator::EffectContext::Plug(Handle<Object> lit) const { UNREACHABLE(); }


void FullCodeGenerator::AccumulatorValueContext::Plug(
    Handle<Object> lit) const {
  __ li(result_register(), Operand(lit));
}


void FullCodeGenerator::StackValueContext::Plug(Handle<Object> lit) const {
  // Immediates cannot be pushed directly.
  __ li(result_register(), Operand(lit));
  __ push(result_register());
}


void FullCodeGenerator::TestContext::Plug(Handle<Object> lit) const {
  codegen()->PrepareForBailoutBeforeSplit(condition(),
                                          true,
                                          true_label_,
                                          false_label_);
  ASSERT(!lit->IsUndetectableObject());  // There are no undetectable literals.
  if (lit->IsUndefined() || lit->IsNull() || lit->IsFalse()) {
    if (false_label_ != fall_through_) __ Branch(false_label_);
  } else if (lit->IsTrue() || lit->IsJSObject()) {
    if (true_label_ != fall_through_) __ Branch(true_label_);
  } else if (lit->IsString()) {
    if (String::cast(*lit)->length() == 0) {
      if (false_label_ != fall_through_) __ Branch(false_label_);
    } else {
      if (true_label_ != fall_through_) __ Branch(true_label_);
    }
  } else if (lit->IsSmi()) {
    if (Smi::cast(*lit)->value() == 0) {
      if (false_label_ != fall_through_) __ Branch(false_label_);
    } else {
      if (true_label_ != fall_through_) __ Branch(true_label_);
    }
  } else {
    // For simplicity we always test the accumulator register.
    __ li(result_register(), Operand(lit));
    codegen()->DoTest(this);
  }
}


void FullCodeGenerator::EffectContext::DropAndPlug(int count,
                                                   Register reg) const {
  ASSERT(count > 0);
  __ Drop(count);
}


void FullCodeGenerator::AccumulatorValueContext::DropAndPlug(
    int count,
    Register reg) const {
  ASSERT(count > 0);
  __ Drop(count);
  __ Move(result_register(), reg);
}


void FullCodeGenerator::StackValueContext::DropAndPlug(int count,
                                                       Register reg) const {
  ASSERT(count > 0);
  if (count > 1) __ Drop(count - 1);
  __ st(reg, MemOperand(sp, 0));
}


void FullCodeGenerator::TestContext::DropAndPlug(int count,
                                                 Register reg) const {
  ASSERT(count > 0);
  // For simplicity we always test the accumulator register.
  __ Drop(count);
  __ Move(result_register(), reg);
  codegen()->PrepareForBailoutBeforeSplit(condition(), false, NULL, NULL);
  codegen()->DoTest(this);
}


void FullCodeGenerator::EffectContext::Plug(Label* materialize_true,
                                            Label* materialize_false) const {
  ASSERT(materialize_true == materialize_false);
  __ bind(materialize_true);
}


void FullCodeGenerator::AccumulatorValueContext::Plug(
    Label* materialize_true,
    Label* materialize_false) const {
  Label done;
  __ bind(materialize_true);
  __ LoadRoot(result_register(), Heap::kTrueValueRootIndex);
  __ Branch(&done);
  __ bind(materialize_false);
  __ LoadRoot(result_register(), Heap::kFalseValueRootIndex);
  __ bind(&done);
}


void FullCodeGenerator::StackValueContext::Plug(
    Label* materialize_true,
    Label* materialize_false) const {
  Label done;
  __ bind(materialize_true);
  __ LoadRoot(at, Heap::kTrueValueRootIndex);
  __ push(at);
  __ Branch(&done);
  __ bind(materialize_false);
  __ LoadRoot(at, Heap::kFalseValueRootIndex);
  __ push(at);
  __ bind(&done);
}


void FullCodeGenerator::TestContext::Plug(Label* materialize_true,
                                          Label* materialize_false) const {
  ASSERT(materialize_true == true_label_);
  ASSERT(materialize_false == false_label_);
}


void FullCodeGenerator::EffectContext::Plug(bool flag) const { UNREACHABLE(); }


void FullCodeGenerator::AccumulatorValueContext::Plug(bool flag) const {
  Heap::RootListIndex value_root_index =
      flag ? Heap::kTrueValueRootIndex : Heap::kFalseValueRootIndex;
  __ LoadRoot(result_register(), value_root_index);
}


void FullCodeGenerator::StackValueContext::Plug(bool flag) const {
  Heap::RootListIndex value_root_index =
      flag ? Heap::kTrueValueRootIndex : Heap::kFalseValueRootIndex;
  __ LoadRoot(at, value_root_index);
  __ push(at);
}


void FullCodeGenerator::TestContext::Plug(bool flag) const {
  codegen()->PrepareForBailoutBeforeSplit(condition(),
                                          true,
                                          true_label_,
                                          false_label_);
  if (flag) {
    if (true_label_ != fall_through_) __ Branch(true_label_);
  } else {
    if (false_label_ != fall_through_) __ Branch(false_label_);
  }
}


void FullCodeGenerator::DoTest(Expression* condition,
                               Label* if_true,
                               Label* if_false,
                               Label* fall_through) {
  ToBooleanStub stub(result_register());
  __ CallStub(&stub, condition->test_id());
  __ move(at, zero);
  Split(ne, v0, Operand(at), if_true, if_false, fall_through);
}


void FullCodeGenerator::Split(Condition cc,
                              Register lhs,
                              const Operand&  rhs,
                              Label* if_true,
                              Label* if_false,
                              Label* fall_through) {
  if (if_false == fall_through) {
    __ Branch(if_true, cc, lhs, rhs);
  } else if (if_true == fall_through) {
    __ Branch(if_false, NegateCondition(cc), lhs, rhs);
  } else {
    __ Branch(if_true, cc, lhs, rhs);
    __ Branch(if_false);
  }
}


MemOperand FullCodeGenerator::StackOperand(Variable* var) {
  ASSERT(var->IsStackAllocated());
  // Offset is negative because higher indexes are at lower addresses.
  int offset = -var->index() * kPointerSize;
  // Adjust by a (parameter or local) base offset.
  if (var->IsParameter()) {
    offset += (info_->scope()->num_parameters() + 1) * kPointerSize;
  } else {
    offset += JavaScriptFrameConstants::kLocal0Offset;
  }
  return MemOperand(fp, offset);
}


MemOperand FullCodeGenerator::VarOperand(Variable* var, Register scratch) {
  ASSERT(var->IsContextSlot() || var->IsStackAllocated());
  if (var->IsContextSlot()) {
    int context_chain_length = scope()->ContextChainLength(var->scope());
    __ LoadContext(scratch, context_chain_length);
    return ContextOperand(scratch, var->index());
  } else {
    return StackOperand(var);
  }
}


void FullCodeGenerator::GetVar(Register dest, Variable* var) {
  // Use destination as scratch.
  MemOperand location = VarOperand(var, dest);
  __ ld(dest, location);
}


void FullCodeGenerator::SetVar(Variable* var,
                               Register src,
                               Register scratch0,
                               Register scratch1) {
  ASSERT(var->IsContextSlot() || var->IsStackAllocated());
  ASSERT(!scratch0.is(src));
  ASSERT(!scratch0.is(scratch1));
  ASSERT(!scratch1.is(src));
  MemOperand location = VarOperand(var, scratch0);
  __ st(src, location);
  // Emit the write barrier code if the location is in the heap.
  if (var->IsContextSlot()) {
    __ RecordWriteContextSlot(scratch0,
                              location.offset(),
                              src,
                              scratch1,
                              kRAHasBeenSaved,
                              kDontSaveFPRegs);
  }
}


void FullCodeGenerator::PrepareForBailoutBeforeSplit(Expression* expr,
                                                     bool should_normalize,
                                                     Label* if_true,
                                                     Label* if_false) {
  // Only prepare for bailouts before splits if we're in a test
  // context. Otherwise, we let the Visit function deal with the
  // preparation to avoid preparing with the same AST id twice.
  if (!context()->IsTest() || !info_->IsOptimizable()) return;

  Label skip;
  if (should_normalize) __ Branch(&skip);
  PrepareForBailout(expr, TOS_REG);
  if (should_normalize) {
    __ LoadRoot(t0, Heap::kTrueValueRootIndex);
    Split(eq, a0, Operand(t0), if_true, if_false, NULL);
    __ bind(&skip);
  }
}


void FullCodeGenerator::EmitDebugCheckDeclarationContext(Variable* variable) {
  // The variable in the declaration always resides in the current function
  // context.
  ASSERT_EQ(0, scope()->ContextChainLength(variable->scope()));
  if (generate_debug_code_) {
    // Check that we're not inside a with or catch context.
    __ ld(a1, FieldMemOperand(cp, HeapObject::kMapOffset));
    __ LoadRoot(t0, Heap::kWithContextMapRootIndex);
    __ Check(ne, "Declaration in with context.",
        a1, Operand(t0));
    __ LoadRoot(t0, Heap::kCatchContextMapRootIndex);
    __ Check(ne, "Declaration in catch context.",
        a1, Operand(t0));
  }
}


void FullCodeGenerator::VisitVariableDeclaration(
    VariableDeclaration* declaration) {
  // If it was not possible to allocate the variable at compile time, we
  // need to "declare" it at runtime to make sure it actually exists in the
  // local context.
  VariableProxy* proxy = declaration->proxy();
  VariableMode mode = declaration->mode();
  Variable* variable = proxy->var();
  bool hole_init = mode == CONST || mode == CONST_HARMONY || mode == LET;
  switch (variable->location()) {
    case Variable::UNALLOCATED:
      globals_->Add(variable->name(), zone());
      globals_->Add(variable->binding_needs_init()
                        ? isolate()->factory()->the_hole_value()
                        : isolate()->factory()->undefined_value(),
                    zone());
      break;

    case Variable::PARAMETER:
    case Variable::LOCAL:
      if (hole_init) {
        Comment cmnt(masm_, "[ VariableDeclaration");
        __ LoadRoot(t0, Heap::kTheHoleValueRootIndex);
        __ st(t0, StackOperand(variable));
      }
      break;

      case Variable::CONTEXT:
      if (hole_init) {
        Comment cmnt(masm_, "[ VariableDeclaration");
        EmitDebugCheckDeclarationContext(variable);
          __ LoadRoot(at, Heap::kTheHoleValueRootIndex);
          __ st(at, ContextOperand(cp, variable->index()));
          // No write barrier since the_hole_value is in old space.
          PrepareForBailoutForId(proxy->id(), NO_REGISTERS);
      }
      break;

    case Variable::LOOKUP: {
      Comment cmnt(masm_, "[ VariableDeclaration");
      __ li(a2, Operand(variable->name()));
      // Declaration nodes are always introduced in one of four modes.
      ASSERT(IsDeclaredVariableMode(mode));
      PropertyAttributes attr =
          IsImmutableVariableMode(mode) ? READ_ONLY : NONE;
      __ li(a1, Operand(Smi::FromInt(attr)));
      // Push initial value, if any.
      // Note: For variables we must not push an initial value (such as
      // 'undefined') because we may have a (legal) redeclaration and we
      // must not destroy the current value.
      if (hole_init) {
        __ LoadRoot(a0, Heap::kTheHoleValueRootIndex);
        __ Push(cp, a2, a1, a0);
      } else {
        ASSERT(Smi::FromInt(0) == 0);
        __ move(a0, zero);  // Smi::FromInt(0) indicates no initial value.
        __ Push(cp, a2, a1, a0);
      }
      __ CallRuntime(Runtime::kDeclareContextSlot, 4);
      break;
    }
  }
}


void FullCodeGenerator::VisitFunctionDeclaration(
    FunctionDeclaration* declaration) {
  VariableProxy* proxy = declaration->proxy();
  Variable* variable = proxy->var();
  switch (variable->location()) {
    case Variable::UNALLOCATED: {
      globals_->Add(variable->name(), zone());
      Handle<SharedFunctionInfo> function =
          Compiler::BuildFunctionInfo(declaration->fun(), script());
      // Check for stack-overflow exception.
      if (function.is_null()) return SetStackOverflow();
      globals_->Add(function, zone());
      break;
    }

    case Variable::PARAMETER:
    case Variable::LOCAL: {
      Comment cmnt(masm_, "[ FunctionDeclaration");
      VisitForAccumulatorValue(declaration->fun());
      __ st(result_register(), StackOperand(variable));
      break;
    }

    case Variable::CONTEXT: {
      Comment cmnt(masm_, "[ FunctionDeclaration");
      EmitDebugCheckDeclarationContext(variable);
      VisitForAccumulatorValue(declaration->fun());
      __ st(result_register(), ContextOperand(cp, variable->index()));
      int offset = Context::SlotOffset(variable->index());
      // We know that we have written a function, which is not a smi.
      __ RecordWriteContextSlot(cp,
                                offset,
                                result_register(),
                                a2,
                                kRAHasBeenSaved,
                                kDontSaveFPRegs,
                                EMIT_REMEMBERED_SET,
                                OMIT_SMI_CHECK);
      PrepareForBailoutForId(proxy->id(), NO_REGISTERS);
      break;
    }

    case Variable::LOOKUP: {
      Comment cmnt(masm_, "[ FunctionDeclaration");
      __ li(a2, Operand(variable->name()));
      __ li(a1, Operand(Smi::FromInt(NONE)));
      __ Push(cp, a2, a1);
      // Push initial value for function declaration.
      VisitForStackValue(declaration->fun());
      __ CallRuntime(Runtime::kDeclareContextSlot, 4);
      break;
    }
  }
}


void FullCodeGenerator::VisitModuleDeclaration(ModuleDeclaration* declaration) {
  Variable* variable = declaration->proxy()->var();
  ASSERT(variable->location() == Variable::CONTEXT);
  ASSERT(variable->interface()->IsFrozen());

  Comment cmnt(masm_, "[ ModuleDeclaration");
  EmitDebugCheckDeclarationContext(variable);

  // Load instance object.
  __ LoadContext(a1, scope_->ContextChainLength(scope_->GlobalScope()));
  __ ld(a1, ContextOperand(a1, variable->interface()->Index()));
  __ ld(a1, ContextOperand(a1, Context::EXTENSION_INDEX));

  // Assign it.
  __ st(a1, ContextOperand(cp, variable->index()));
  // We know that we have written a module, which is not a smi.
  __ RecordWriteContextSlot(cp,
                            Context::SlotOffset(variable->index()),
                            a1,
                            a3,
                            kRAHasBeenSaved,
                            kDontSaveFPRegs,
                            EMIT_REMEMBERED_SET,
                            OMIT_SMI_CHECK);
  PrepareForBailoutForId(declaration->proxy()->id(), NO_REGISTERS);

  // Traverse into body.
  Visit(declaration->module());
}


void FullCodeGenerator::VisitImportDeclaration(ImportDeclaration* declaration) {
  VariableProxy* proxy = declaration->proxy();
  Variable* variable = proxy->var();
  switch (variable->location()) {
    case Variable::UNALLOCATED:
      // TODO(rossberg)
      break;

    case Variable::CONTEXT: {
      Comment cmnt(masm_, "[ ImportDeclaration");
      EmitDebugCheckDeclarationContext(variable);
      // TODO(rossberg)
      break;
    }

    case Variable::PARAMETER:
    case Variable::LOCAL:
    case Variable::LOOKUP:
      UNREACHABLE();
  }
}


void FullCodeGenerator::VisitExportDeclaration(ExportDeclaration* declaration) {
  // TODO(rossberg)
}


void FullCodeGenerator::DeclareGlobals(Handle<FixedArray> pairs) {
  // Call the runtime to declare the globals.
  // The context is the first argument.
  __ li(a1, Operand(pairs));
  __ li(a0, Operand(Smi::FromInt(DeclareGlobalsFlags())));
  __ Push(cp, a1, a0);
  __ CallRuntime(Runtime::kDeclareGlobals, 3);
  // Return value is ignored.
}


void FullCodeGenerator::DeclareModules(Handle<FixedArray> descriptions) { UNREACHABLE(); }


void FullCodeGenerator::VisitSwitchStatement(SwitchStatement* stmt) { UNREACHABLE(); }


void FullCodeGenerator::VisitForInStatement(ForInStatement* stmt) { UNREACHABLE(); }


void FullCodeGenerator::EmitNewClosure(Handle<SharedFunctionInfo> info,
                                       bool pretenure) { UNREACHABLE(); }


void FullCodeGenerator::VisitVariableProxy(VariableProxy* expr) { UNREACHABLE(); }


void FullCodeGenerator::EmitLoadGlobalCheckExtensions(Variable* var,
                                                      TypeofState typeof_state,
                                                      Label* slow) { UNREACHABLE(); }


MemOperand FullCodeGenerator::ContextSlotOperandCheckExtensions(Variable* var,
                                                                Label* slow) { UNREACHABLE(); return MemOperand(fp); }


void FullCodeGenerator::EmitDynamicLookupFastCase(Variable* var,
                                                  TypeofState typeof_state,
                                                  Label* slow,
                                                  Label* done) { UNREACHABLE(); }


void FullCodeGenerator::EmitVariableLoad(VariableProxy* proxy) { UNREACHABLE(); }


void FullCodeGenerator::VisitRegExpLiteral(RegExpLiteral* expr) { UNREACHABLE(); }


void FullCodeGenerator::EmitAccessor(Expression* expression) { UNREACHABLE(); }


void FullCodeGenerator::VisitObjectLiteral(ObjectLiteral* expr) { UNREACHABLE(); }


void FullCodeGenerator::VisitArrayLiteral(ArrayLiteral* expr) { UNREACHABLE(); }


void FullCodeGenerator::VisitAssignment(Assignment* expr) { UNREACHABLE(); }


void FullCodeGenerator::VisitYield(Yield* expr) { UNREACHABLE(); }


void FullCodeGenerator::EmitGeneratorResume(Expression *generator,
    Expression *value,
    JSGeneratorObject::ResumeMode resume_mode) { UNREACHABLE(); }


void FullCodeGenerator::EmitReturnIteratorResult(bool done) { UNREACHABLE(); }


void FullCodeGenerator::EmitNamedPropertyLoad(Property* prop) { UNREACHABLE(); }


void FullCodeGenerator::EmitKeyedPropertyLoad(Property* prop) { UNREACHABLE(); }


void FullCodeGenerator::EmitInlineSmiBinaryOp(BinaryOperation* expr,
                                              Token::Value op,
                                              OverwriteMode mode,
                                              Expression* left_expr,
                                              Expression* right_expr) { UNREACHABLE(); }


void FullCodeGenerator::EmitBinaryOp(BinaryOperation* expr,
                                     Token::Value op,
                                     OverwriteMode mode) { UNREACHABLE(); }


void FullCodeGenerator::EmitAssignment(Expression* expr) { UNREACHABLE(); }


void FullCodeGenerator::EmitVariableAssignment(Variable* var,
                                               Token::Value op) { UNREACHABLE(); }


void FullCodeGenerator::EmitNamedPropertyAssignment(Assignment* expr) { UNREACHABLE(); }


void FullCodeGenerator::EmitKeyedPropertyAssignment(Assignment* expr) { UNREACHABLE(); }


void FullCodeGenerator::VisitProperty(Property* expr) { UNREACHABLE(); }


void FullCodeGenerator::CallIC(Handle<Code> code,
                               RelocInfo::Mode rmode,
                               TypeFeedbackId id) { UNREACHABLE(); }


void FullCodeGenerator::EmitCallWithIC(Call* expr,
                                       Handle<Object> name,
                                       RelocInfo::Mode mode) { UNREACHABLE(); }


void FullCodeGenerator::EmitKeyedCallWithIC(Call* expr,
                                            Expression* key) { UNREACHABLE(); }


void FullCodeGenerator::EmitCallWithStub(Call* expr, CallFunctionFlags flags) { UNREACHABLE(); }


void FullCodeGenerator::EmitResolvePossiblyDirectEval(int arg_count) { UNREACHABLE(); }


void FullCodeGenerator::VisitCall(Call* expr) { UNREACHABLE(); }


void FullCodeGenerator::VisitCallNew(CallNew* expr) { UNREACHABLE(); }


void FullCodeGenerator::EmitIsSmi(CallRuntime* expr) { UNREACHABLE(); }


void FullCodeGenerator::EmitIsNonNegativeSmi(CallRuntime* expr) { UNREACHABLE(); }


void FullCodeGenerator::EmitIsObject(CallRuntime* expr) { UNREACHABLE(); }


void FullCodeGenerator::EmitIsSpecObject(CallRuntime* expr) { UNREACHABLE(); }


void FullCodeGenerator::EmitIsUndetectableObject(CallRuntime* expr) { UNREACHABLE(); }


void FullCodeGenerator::EmitIsStringWrapperSafeForDefaultValueOf(
    CallRuntime* expr) { UNREACHABLE(); }


void FullCodeGenerator::EmitIsFunction(CallRuntime* expr) { UNREACHABLE(); }


void FullCodeGenerator::EmitIsArray(CallRuntime* expr) { UNREACHABLE(); }


void FullCodeGenerator::EmitIsRegExp(CallRuntime* expr) { UNREACHABLE(); }


void FullCodeGenerator::EmitIsConstructCall(CallRuntime* expr) { UNREACHABLE(); }


void FullCodeGenerator::EmitObjectEquals(CallRuntime* expr) { UNREACHABLE(); }


void FullCodeGenerator::EmitArguments(CallRuntime* expr) { UNREACHABLE(); }


void FullCodeGenerator::EmitArgumentsLength(CallRuntime* expr) { UNREACHABLE(); }


void FullCodeGenerator::EmitClassOf(CallRuntime* expr) { UNREACHABLE(); }


void FullCodeGenerator::EmitLog(CallRuntime* expr) { UNREACHABLE(); }


void FullCodeGenerator::EmitRandomHeapNumber(CallRuntime* expr) { UNREACHABLE(); }


void FullCodeGenerator::EmitSubString(CallRuntime* expr) { UNREACHABLE(); }


void FullCodeGenerator::EmitRegExpExec(CallRuntime* expr) { UNREACHABLE(); }


void FullCodeGenerator::EmitValueOf(CallRuntime* expr) { UNREACHABLE(); }


void FullCodeGenerator::EmitDateField(CallRuntime* expr) { UNREACHABLE(); }


void FullCodeGenerator::EmitSeqStringSetCharCheck(Register string,
                                                  Register index,
                                                  Register value,
                                                  uint32_t encoding_mask) { UNREACHABLE(); }


void FullCodeGenerator::EmitOneByteSeqStringSetChar(CallRuntime* expr) { UNREACHABLE(); }


void FullCodeGenerator::EmitTwoByteSeqStringSetChar(CallRuntime* expr) { UNREACHABLE(); }


void FullCodeGenerator::EmitMathPow(CallRuntime* expr) { UNREACHABLE(); }


void FullCodeGenerator::EmitSetValueOf(CallRuntime* expr) { UNREACHABLE(); }


void FullCodeGenerator::EmitNumberToString(CallRuntime* expr) { UNREACHABLE(); }


void FullCodeGenerator::EmitStringCharFromCode(CallRuntime* expr) { UNREACHABLE(); }


void FullCodeGenerator::EmitStringCharCodeAt(CallRuntime* expr) { UNREACHABLE(); }


void FullCodeGenerator::EmitStringCharAt(CallRuntime* expr) { UNREACHABLE(); }


void FullCodeGenerator::EmitStringAdd(CallRuntime* expr) { UNREACHABLE(); }


void FullCodeGenerator::EmitStringCompare(CallRuntime* expr) { UNREACHABLE(); }


void FullCodeGenerator::EmitMathSin(CallRuntime* expr) { UNREACHABLE(); }


void FullCodeGenerator::EmitMathCos(CallRuntime* expr) { UNREACHABLE(); }


void FullCodeGenerator::EmitMathTan(CallRuntime* expr) { UNREACHABLE(); }


void FullCodeGenerator::EmitMathLog(CallRuntime* expr) { UNREACHABLE(); }


void FullCodeGenerator::EmitMathSqrt(CallRuntime* expr) { UNREACHABLE(); }


void FullCodeGenerator::EmitCallFunction(CallRuntime* expr) { UNREACHABLE(); }


void FullCodeGenerator::EmitRegExpConstructResult(CallRuntime* expr) { UNREACHABLE(); }


void FullCodeGenerator::EmitGetFromCache(CallRuntime* expr) { UNREACHABLE(); }


void FullCodeGenerator::EmitIsRegExpEquivalent(CallRuntime* expr) { UNREACHABLE(); }


void FullCodeGenerator::EmitHasCachedArrayIndex(CallRuntime* expr) { UNREACHABLE(); }


void FullCodeGenerator::EmitGetCachedArrayIndex(CallRuntime* expr) { UNREACHABLE(); }


void FullCodeGenerator::EmitFastAsciiArrayJoin(CallRuntime* expr) { UNREACHABLE(); }


void FullCodeGenerator::VisitCallRuntime(CallRuntime* expr) { UNREACHABLE(); }


void FullCodeGenerator::VisitUnaryOperation(UnaryOperation* expr) { UNREACHABLE(); }


void FullCodeGenerator::EmitUnaryOperation(UnaryOperation* expr,
                                           const char* comment) { UNREACHABLE(); }


void FullCodeGenerator::VisitCountOperation(CountOperation* expr) { UNREACHABLE(); }


void FullCodeGenerator::VisitForTypeofValue(Expression* expr) { UNREACHABLE(); }

void FullCodeGenerator::EmitLiteralCompareTypeof(Expression* expr,
                                                 Expression* sub_expr,
                                                 Handle<String> check) { UNREACHABLE(); }


void FullCodeGenerator::VisitCompareOperation(CompareOperation* expr) { UNREACHABLE(); }


void FullCodeGenerator::EmitLiteralCompareNil(CompareOperation* expr,
                                              Expression* sub_expr,
                                              NilValue nil) { UNREACHABLE(); }


void FullCodeGenerator::VisitThisFunction(ThisFunction* expr) { UNREACHABLE(); }


Register FullCodeGenerator::result_register() {
  return r0;
}


Register FullCodeGenerator::context_register() {
  return cp;
}


void FullCodeGenerator::StoreToFrameField(int frame_offset, Register value) { UNREACHABLE(); }


void FullCodeGenerator::LoadContextField(Register dst, int context_index) { UNREACHABLE(); }


void FullCodeGenerator::PushFunctionArgumentForContextAllocation() { UNREACHABLE(); }


// ----------------------------------------------------------------------------
// Non-local control flow support.

void FullCodeGenerator::EnterFinallyBlock() { UNREACHABLE(); }


void FullCodeGenerator::ExitFinallyBlock() { UNREACHABLE(); }


#undef __

#define __ ACCESS_MASM(masm())

FullCodeGenerator::NestedStatement* FullCodeGenerator::TryFinally::Exit(
    int* stack_depth,
    int* context_length) { UNREACHABLE(); return NULL; }


#undef __

} }  // namespace v8::internal

#endif  // V8_TARGET_ARCH_TILEGX
