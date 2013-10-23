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
      Register reg = Register::from_code(delta_to_patch_site / kImm8Mask);
      __ andi(zero, reg, delta_to_patch_site % kImm8Mask);
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
//   o r1: the JS function object being called (i.e. ourselves)
//   o cp: our context
//   o fp: our caller's frame pointer
//   o sp: stack pointer
//   o lr: return address
//
// The function builds a JS frame.  Please see JavaScriptFrameConstants in
// frames-tilegx.h for its layout.
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
  __ Push(lr, fp, cp, r1);
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


void FullCodeGenerator::EffectContext::Plug(Heap::RootListIndex index) const {}


void FullCodeGenerator::AccumulatorValueContext::Plug(
    Heap::RootListIndex index) const {
  __ LoadRoot(result_register(), index);
}


void FullCodeGenerator::StackValueContext::Plug(
    Heap::RootListIndex index) const {
  __ LoadRoot(result_register(), index);
  __ push(result_register());
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


void FullCodeGenerator::EffectContext::Plug(Handle<Object> lit) const {
}


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


void FullCodeGenerator::EffectContext::Plug(bool flag) const {}


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
          __ LoadRoot(at2, Heap::kTheHoleValueRootIndex);
          __ st(at2, ContextOperand(cp, variable->index()));
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
  __ info(__LINE__);
  __ li(r1, Operand(pairs));
  __ li(r0, Operand(Smi::FromInt(DeclareGlobalsFlags())));
  __ Push(cp, r1, r0);
  __ CallRuntime(Runtime::kDeclareGlobals, 3);
  // Return value is ignored.
}


void FullCodeGenerator::DeclareModules(Handle<FixedArray> descriptions) {
  __ info(__LINE__);
  // Call the runtime to declare the modules.
  __ Push(descriptions);
  __ CallRuntime(Runtime::kDeclareModules, 1);
  // Return value is ignored.
}


void FullCodeGenerator::VisitSwitchStatement(SwitchStatement* stmt) {
  Comment cmnt(masm_, "[ SwitchStatement");
  Breakable nested_statement(this, stmt);
  SetStatementPosition(stmt);

  // Keep the switch value on the stack until a case matches.
  VisitForStackValue(stmt->tag());
  PrepareForBailoutForId(stmt->EntryId(), NO_REGISTERS);

  ZoneList<CaseClause*>* clauses = stmt->cases();
  CaseClause* default_clause = NULL;  // Can occur anywhere in the list.

  Label next_test;  // Recycled for each test.
  // Compile all the tests with branches to their bodies.
  for (int i = 0; i < clauses->length(); i++) {
    CaseClause* clause = clauses->at(i);
    clause->body_target()->Unuse();

    // The default is not a test, but remember it as final fall through.
    if (clause->is_default()) {
      default_clause = clause;
      continue;
    }

    Comment cmnt(masm_, "[ Case comparison");
    __ bind(&next_test);
    next_test.Unuse();

    // Compile the label expression.
    VisitForAccumulatorValue(clause->label());
    //__ move(a0, result_register());  // CompareStub requires args in a0, a1.

    // Perform the comparison as if via '==='.
    __ ld(a1, MemOperand(sp, 0));  // Switch value.
    bool inline_smi_code = ShouldInlineSmiCase(Token::EQ_STRICT);
    JumpPatchSite patch_site(masm_);
    if (inline_smi_code) {
      Label slow_case;
      __ or_(a2, a1, a0);
      patch_site.EmitJumpIfNotSmi(a2, &slow_case);

      __ Branch(&next_test, ne, a1, Operand(a0));
      __ Drop(1);  // Switch value is no longer needed.
      __ Branch(clause->body_target());

      __ bind(&slow_case);
    }

    // Record position before stub call for type feedback.
    SetSourcePosition(clause->position());
    Handle<Code> ic = CompareIC::GetUninitialized(isolate(), Token::EQ_STRICT);
    CallIC(ic, RelocInfo::CODE_TARGET, clause->CompareId());
    patch_site.EmitPatchInfo();

    __ Branch(&next_test, ne, v0, Operand(zero));
    __ Drop(1);  // Switch value is no longer needed.
    __ Branch(clause->body_target());
  }

  // Discard the test value and jump to the default if present, otherwise to
  // the end of the statement.
  __ bind(&next_test);
  __ Drop(1);  // Switch value is no longer needed.
  if (default_clause == NULL) {
    __ Branch(nested_statement.break_label());
  } else {
    __ Branch(default_clause->body_target());
  }

  // Compile all the case bodies.
  for (int i = 0; i < clauses->length(); i++) {
    Comment cmnt(masm_, "[ Case body");
    CaseClause* clause = clauses->at(i);
    __ bind(clause->body_target());
    PrepareForBailoutForId(clause->EntryId(), NO_REGISTERS);
    VisitStatements(clause->statements());
  }

  __ bind(nested_statement.break_label());
  PrepareForBailoutForId(stmt->ExitId(), NO_REGISTERS);
}

void FullCodeGenerator::VisitForInStatement(ForInStatement* stmt) {
  Comment cmnt(masm_, "[ ForInStatement");
  SetStatementPosition(stmt);

  Label loop, exit;
  ForIn loop_statement(this, stmt);
  increment_loop_depth();

  // Get the object to enumerate over. Both SpiderMonkey and JSC
  // ignore null and undefined in contrast to the specification; see
  // ECMA-262 section 12.6.4.
  VisitForAccumulatorValue(stmt->enumerable());
  //__ move(a0, result_register());  // Result as param to InvokeBuiltin below.
  __ LoadRoot(at, Heap::kUndefinedValueRootIndex);
  __ Branch(&exit, eq, a0, Operand(at));
  Register null_value = t1;
  __ LoadRoot(null_value, Heap::kNullValueRootIndex);
  __ Branch(&exit, eq, a0, Operand(null_value));
  PrepareForBailoutForId(stmt->PrepareId(), TOS_REG);
  //__ move(a0, v0);
  // Convert the object to a JS object.
  Label convert, done_convert;
  __ JumpIfSmi(a0, &convert);
  __ GetObjectType(a0, a1, a1);
  __ Branch(&done_convert, ge, a1, Operand(FIRST_SPEC_OBJECT_TYPE));
  __ bind(&convert);
  __ push(a0);
  __ InvokeBuiltin(Builtins::TO_OBJECT, CALL_FUNCTION);
  //__ move(a0, v0);
  __ bind(&done_convert);
  __ push(a0);

  // Check for proxies.
  Label call_runtime;
  STATIC_ASSERT(FIRST_JS_PROXY_TYPE == FIRST_SPEC_OBJECT_TYPE);
  __ GetObjectType(a0, a1, a1);
  __ Branch(&call_runtime, le, a1, Operand(LAST_JS_PROXY_TYPE));

  // Check cache validity in generated code. This is a fast case for
  // the JSObject::IsSimpleEnum cache validity checks. If we cannot
  // guarantee cache validity, call the runtime system to check cache
  // validity or get the property names in a fixed array.
  __ CheckEnumCache(null_value, &call_runtime);

  // The enum cache is valid.  Load the map of the object being
  // iterated over and use the cache for the iteration.
  Label use_cache;
  __ move(at2, a0);
  __ ld(v0, FieldMemOperand(a0, HeapObject::kMapOffset));
  __ Branch(&use_cache);

  __ move(a0, at2);
  // Get the set of properties to enumerate.
  __ bind(&call_runtime);
  __ push(a0);  // Duplicate the enumerable object on the stack.
  __ CallRuntime(Runtime::kGetPropertyNamesFast, 1);

  // If we got a map from the runtime call, we can do a fast
  // modification check. Otherwise, we got a fixed array, and we have
  // to do a slow check.
  Label fixed_array;
  __ ld(a2, FieldMemOperand(v0, HeapObject::kMapOffset));
  __ LoadRoot(at, Heap::kMetaMapRootIndex);
  __ Branch(&fixed_array, ne, a2, Operand(at));

  // We got a map in register v0. Get the enumeration cache from it.
  Label no_descriptors;
  __ bind(&use_cache);

  __ EnumLength(a1, v0);
  __ Branch(&no_descriptors, eq, a1, Operand(Smi::FromInt(0)));

  __ LoadInstanceDescriptors(v0, a2);
  __ ld(a2, FieldMemOperand(a2, DescriptorArray::kEnumCacheOffset));
  __ ld(a2, FieldMemOperand(a2, DescriptorArray::kEnumCacheBridgeCacheOffset));

  // Set up the four remaining stack slots.
  __ push(v0);  // Map.
  __ li(a0, Operand(Smi::FromInt(0)));
  // Push enumeration cache, enumeration cache length (as smi) and zero.
  __ Push(a2, a1, a0);
  __ jmp(&loop);

  __ bind(&no_descriptors);
  __ Drop(1);
  __ jmp(&exit);

  // We got a fixed array in register v0. Iterate through that.
  Label non_proxy;
  __ bind(&fixed_array);

  Handle<JSGlobalPropertyCell> cell =
      isolate()->factory()->NewJSGlobalPropertyCell(
          Handle<Object>(
              Smi::FromInt(TypeFeedbackCells::kForInFastCaseMarker),
              isolate()));
  RecordTypeFeedbackCell(stmt->ForInFeedbackId(), cell);
  __ LoadHeapObject(a1, cell);
  __ li(a2, Operand(Smi::FromInt(TypeFeedbackCells::kForInSlowCaseMarker)));
  __ st(a2, FieldMemOperand(a1, JSGlobalPropertyCell::kValueOffset));

  __ li(a1, Operand(Smi::FromInt(1)));  // Smi indicates slow check
  __ ld(a2, MemOperand(sp, 0 * kPointerSize));  // Get enumerated object
  STATIC_ASSERT(FIRST_JS_PROXY_TYPE == FIRST_SPEC_OBJECT_TYPE);
  __ GetObjectType(a2, a3, a3);
  __ Branch(&non_proxy, gt, a3, Operand(LAST_JS_PROXY_TYPE));
  __ li(a1, Operand(Smi::FromInt(0)));  // Zero indicates proxy
  __ bind(&non_proxy);
  __ Push(a1, v0);  // Smi and array
  __ ld(a1, FieldMemOperand(v0, FixedArray::kLengthOffset));
  __ li(a0, Operand(Smi::FromInt(0)));
  __ Push(a1, a0);  // Fixed array length (as smi) and initial index.

  // Generate code for doing the condition check.
  PrepareForBailoutForId(stmt->BodyId(), NO_REGISTERS);
  __ bind(&loop);
  // Load the current count to a0, load the length to a1.
  __ ld(a0, MemOperand(sp, 0 * kPointerSize));
  __ ld(a1, MemOperand(sp, 1 * kPointerSize));
  __ Branch(loop_statement.break_label(), hs, a0, Operand(a1));

  // Get the current entry of the array into register a3.
  __ ld(a2, MemOperand(sp, 2 * kPointerSize));
  __ Addu(a2, a2, Operand(FixedArray::kHeaderSize - kHeapObjectTag));
  __ sra(t0, a0, kSmiTagSize + kSmiShiftSize);
  __ sll(t0, t0, kPointerSizeLog2);
  __ add(t0, a2, t0);  // Array base + scaled (smi) index.
  __ ld(a3, MemOperand(t0));  // Current entry.

  // Get the expected map from the stack or a smi in the
  // permanent slow case into register a2.
  __ ld(a2, MemOperand(sp, 3 * kPointerSize));

  // Check if the expected map still matches that of the enumerable.
  // If not, we may have to filter the key.
  Label update_each;
  __ ld(a1, MemOperand(sp, 4 * kPointerSize));
  __ ld(t0, FieldMemOperand(a1, HeapObject::kMapOffset));
  __ Branch(&update_each, eq, t0, Operand(a2));

  // For proxies, no filtering is done.
  // TODO(rossberg): What if only a prototype is a proxy? Not specified yet.
  ASSERT_EQ(Smi::FromInt(0), 0);
  __ Branch(&update_each, eq, a2, Operand(zero));

  // Convert the entry to a string or (smi) 0 if it isn't a property
  // any more. If the property has been removed while iterating, we
  // just skip it.
  __ push(a1);  // Enumerable.
  __ push(a3);  // Current entry.
  __ InvokeBuiltin(Builtins::FILTER_KEY, CALL_FUNCTION);
  __ move(a3, result_register());
  __ Branch(loop_statement.continue_label(), eq, a3, Operand(zero));

  // Update the 'each' property or variable from the possibly filtered
  // entry in register a3.
  __ bind(&update_each);
  __ move(result_register(), a3);
  // Perform the assignment as if via '='.
  { EffectContext context(this);
    EmitAssignment(stmt->each());
  }

  // Generate code for the body of the loop.
  Visit(stmt->body());

  // Generate code for the going to the next element by incrementing
  // the index (smi) stored on top of the stack.
  __ bind(loop_statement.continue_label());
  __ pop(a0);
  __ Addu(a0, a0, Operand(Smi::FromInt(1)));
  __ push(a0);

  EmitBackEdgeBookkeeping(stmt, &loop);
  __ Branch(&loop);

  // Remove the pointers stored on the stack.
  __ bind(loop_statement.break_label());
  __ Drop(5);

  // Exit and decrement the loop depth.
  PrepareForBailoutForId(stmt->ExitId(), NO_REGISTERS);
  __ bind(&exit);
  decrement_loop_depth();
}

void FullCodeGenerator::EmitNewClosure(Handle<SharedFunctionInfo> info,
                                       bool pretenure) {
  // Use the fast case closure allocation code that allocates in new
  // space for nested functions that don't need literals cloning. If
  // we're running with the --always-opt or the --prepare-always-opt
  // flag, we need to use the runtime function so that the new function
  // we are creating here gets a chance to have its code optimized and
  // doesn't just get a copy of the existing unoptimized code.
  if (!FLAG_always_opt &&
      !FLAG_prepare_always_opt &&
      !pretenure &&
      scope()->is_function_scope() &&
      info->num_literals() == 0) {
    FastNewClosureStub stub(info->language_mode(), info->is_generator());
    __ li(a0, Operand(info));
    __ push(a0);
    __ CallStub(&stub);
  } else {
    __ li(a0, Operand(info));
    __ LoadRoot(a1, pretenure ? Heap::kTrueValueRootIndex
                              : Heap::kFalseValueRootIndex);
    __ Push(cp, a0, a1);
    __ CallRuntime(Runtime::kNewClosure, 3);
  }
  context()->Plug(v0);
}

void FullCodeGenerator::VisitVariableProxy(VariableProxy* expr) {
  Comment cmnt(masm_, "[ VariableProxy");
  EmitVariableLoad(expr);
}

void FullCodeGenerator::EmitLoadGlobalCheckExtensions(Variable* var,
                                                      TypeofState typeof_state,
                                                      Label* slow) {
  Register current = cp;
  Register next = a1;
  Register temp = a2;

  Scope* s = scope();
  while (s != NULL) {
    if (s->num_heap_slots() > 0) {
      if (s->calls_non_strict_eval()) {
        // Check that extension is NULL.
        __ ld(temp, ContextOperand(current, Context::EXTENSION_INDEX));
        __ Branch(slow, ne, temp, Operand(zero));
      }
      // Load next context in chain.
      __ ld(next, ContextOperand(current, Context::PREVIOUS_INDEX));
      // Walk the rest of the chain without clobbering cp.
      current = next;
    }
    // If no outer scope calls eval, we do not need to check more
    // context extensions.
    if (!s->outer_scope_calls_non_strict_eval() || s->is_eval_scope()) break;
    s = s->outer_scope();
  }

  if (s->is_eval_scope()) {
    Label loop, fast;
    if (!current.is(next)) {
      __ Move(next, current);
    }
    __ bind(&loop);
    // Terminate at native context.
    __ ld(temp, FieldMemOperand(next, HeapObject::kMapOffset));
    __ LoadRoot(t0, Heap::kNativeContextMapRootIndex);
    __ Branch(&fast, eq, temp, Operand(t0));
    // Check that extension is NULL.
    __ ld(temp, ContextOperand(next, Context::EXTENSION_INDEX));
    __ Branch(slow, ne, temp, Operand(zero));
    // Load next context in chain.
    __ ld(next, ContextOperand(next, Context::PREVIOUS_INDEX));
    __ Branch(&loop);
    __ bind(&fast);
  }

  __ ld(a0, GlobalObjectOperand());
  __ li(a2, Operand(var->name()));
  RelocInfo::Mode mode = (typeof_state == INSIDE_TYPEOF)
      ? RelocInfo::CODE_TARGET
      : RelocInfo::CODE_TARGET_CONTEXT;
  Handle<Code> ic = isolate()->builtins()->LoadIC_Initialize();
  CallIC(ic, mode);
}


MemOperand FullCodeGenerator::ContextSlotOperandCheckExtensions(Variable* var,
                                                                Label* slow) {
  ASSERT(var->IsContextSlot());
  Register context = cp;
  Register next = a3;
  Register temp = t0;

  for (Scope* s = scope(); s != var->scope(); s = s->outer_scope()) {
    if (s->num_heap_slots() > 0) {
      if (s->calls_non_strict_eval()) {
        // Check that extension is NULL.
        __ ld(temp, ContextOperand(context, Context::EXTENSION_INDEX));
        __ Branch(slow, ne, temp, Operand(zero));
      }
      __ ld(next, ContextOperand(context, Context::PREVIOUS_INDEX));
      // Walk the rest of the chain without clobbering cp.
      context = next;
    }
  }
  // Check that last extension is NULL.
  __ ld(temp, ContextOperand(context, Context::EXTENSION_INDEX));
  __ Branch(slow, ne, temp, Operand(zero));

  // This function is used only for loads, not stores, so it's safe to
  // return an cp-based operand (the write barrier cannot be allowed to
  // destroy the cp register).
  return ContextOperand(context, var->index());
}

void FullCodeGenerator::EmitDynamicLookupFastCase(Variable* var,
                                                  TypeofState typeof_state,
                                                  Label* slow,
                                                  Label* done) {
  // Generate fast-case code for variables that might be shadowed by
  // eval-introduced variables.  Eval is used a lot without
  // introducing variables.  In those cases, we do not want to
  // perform a runtime call for all variables in the scope
  // containing the eval.
  if (var->mode() == DYNAMIC_GLOBAL) {
    EmitLoadGlobalCheckExtensions(var, typeof_state, slow);
    __ Branch(done);
  } else if (var->mode() == DYNAMIC_LOCAL) {
    Variable* local = var->local_if_not_shadowed();
    __ ld(v0, ContextSlotOperandCheckExtensions(local, slow));
    if (local->mode() == LET ||
        local->mode() == CONST ||
        local->mode() == CONST_HARMONY) {
      __ LoadRoot(at, Heap::kTheHoleValueRootIndex);
      __ sub(at, v0, at);  // Sub as compare: at == 0 on eq.
      if (local->mode() == CONST) {
        __ push(at);
        __ LoadRoot(at2, Heap::kUndefinedValueRootIndex);
        __ pop(at);
        __ movz(v0, at2, at);  // Conditional move: return Undefined if TheHole.
      } else {  // LET || CONST_HARMONY
        __ Branch(done, ne, at, Operand(zero));
        __ li(a0, Operand(var->name()));
        __ push(a0);
        __ CallRuntime(Runtime::kThrowReferenceError, 1);
      }
    }
    __ Branch(done);
  }
}


void FullCodeGenerator::EmitVariableLoad(VariableProxy* proxy) {
  // Record position before possible IC call.
  SetSourcePosition(proxy->position());
  Variable* var = proxy->var();

  // Three cases: global variables, lookup variables, and all other types of
  // variables.
  switch (var->location()) {
    case Variable::UNALLOCATED: {
      Comment cmnt(masm_, "Global variable");
      // Use inline caching. Variable name is passed in a2 and the global
      // object (receiver) in a0.
      __ ld(a0, GlobalObjectOperand());
      __ li(a2, Operand(var->name()));
      Handle<Code> ic = isolate()->builtins()->LoadIC_Initialize();
      CallIC(ic, RelocInfo::CODE_TARGET_CONTEXT);
      context()->Plug(v0);
      break;
    }

    case Variable::PARAMETER:
    case Variable::LOCAL:
    case Variable::CONTEXT: {
      Comment cmnt(masm_, var->IsContextSlot()
                              ? "Context variable"
                              : "Stack variable");
      if (var->binding_needs_init()) {
        // var->scope() may be NULL when the proxy is located in eval code and
        // refers to a potential outside binding. Currently those bindings are
        // always looked up dynamically, i.e. in that case
        //     var->location() == LOOKUP.
        // always holds.
        ASSERT(var->scope() != NULL);

        // Check if the binding really needs an initialization check. The check
        // can be skipped in the following situation: we have a LET or CONST
        // binding in harmony mode, both the Variable and the VariableProxy have
        // the same declaration scope (i.e. they are both in global code, in the
        // same function or in the same eval code) and the VariableProxy is in
        // the source physically located after the initializer of the variable.
        //
        // We cannot skip any initialization checks for CONST in non-harmony
        // mode because const variables may be declared but never initialized:
        //   if (false) { const x; }; var y = x;
        //
        // The condition on the declaration scopes is a conservative check for
        // nested functions that access a binding and are called before the
        // binding is initialized:
        //   function() { f(); let x = 1; function f() { x = 2; } }
        //
        bool skip_init_check;
        if (var->scope()->DeclarationScope() != scope()->DeclarationScope()) {
          skip_init_check = false;
        } else {
          // Check that we always have valid source position.
          ASSERT(var->initializer_position() != RelocInfo::kNoPosition);
          ASSERT(proxy->position() != RelocInfo::kNoPosition);
          skip_init_check = var->mode() != CONST &&
              var->initializer_position() < proxy->position();
        }

        if (!skip_init_check) {
          // Let and const need a read barrier.
          GetVar(v0, var);
          __ LoadRoot(at, Heap::kTheHoleValueRootIndex);
          __ sub(at, v0, at);  // Sub as compare: at == 0 on eq.
          if (var->mode() == LET || var->mode() == CONST_HARMONY) {
            // Throw a reference error when using an uninitialized let/const
            // binding in harmony mode.
            Label done;
            __ Branch(&done, ne, at, Operand(zero));
            __ li(a0, Operand(var->name()));
            __ push(a0);
            __ CallRuntime(Runtime::kThrowReferenceError, 1);
            __ bind(&done);
          } else {
            // Uninitalized const bindings outside of harmony mode are unholed.
            ASSERT(var->mode() == CONST);
            __ push(at);
            __ LoadRoot(at2, Heap::kUndefinedValueRootIndex);
            __ pop(at);
            __ movz(v0, at2, at);  // Conditional move: Undefined if TheHole.
          }
          context()->Plug(v0);
          break;
        }
      }
      context()->Plug(var);
      break;
    }

    case Variable::LOOKUP: {
      Label done, slow;
      // Generate code for loading from variables potentially shadowed
      // by eval-introduced variables.
      EmitDynamicLookupFastCase(var, NOT_INSIDE_TYPEOF, &slow, &done);
      __ bind(&slow);
      Comment cmnt(masm_, "Lookup variable");
      __ li(a1, Operand(var->name()));
      __ Push(cp, a1);  // Context and name.
      __ CallRuntime(Runtime::kLoadContextSlot, 2);
      __ bind(&done);
      context()->Plug(v0);
    }
  }
}

void FullCodeGenerator::VisitRegExpLiteral(RegExpLiteral* expr) {
  Comment cmnt(masm_, "[ RegExpLiteral");
  Label materialized;
  // Registers will be used as follows:
  // t1 = materialized value (RegExp literal)
  // t0 = JS function, literals array
  // a3 = literal index
  // a2 = RegExp pattern
  // a1 = RegExp flags
  // a0 = RegExp literal clone
  __ ld(a0, MemOperand(fp, JavaScriptFrameConstants::kFunctionOffset));
  __ ld(t0, FieldMemOperand(a0, JSFunction::kLiteralsOffset));
  int literal_offset =
      FixedArray::kHeaderSize + expr->literal_index() * kPointerSize;
  __ ld(t1, FieldMemOperand(t0, literal_offset));
  __ LoadRoot(at, Heap::kUndefinedValueRootIndex);
  __ Branch(&materialized, ne, t1, Operand(at));

  // Create regexp literal using runtime function.
  // Result will be in v0.
  __ li(a3, Operand(Smi::FromInt(expr->literal_index())));
  __ li(a2, Operand(expr->pattern()));
  __ li(a1, Operand(expr->flags()));
  __ Push(t0, a3, a2, a1);
  __ CallRuntime(Runtime::kMaterializeRegExpLiteral, 4);
  __ move(t1, v0);

  __ bind(&materialized);
  int size = JSRegExp::kSize + JSRegExp::kInObjectFieldCount * kPointerSize;
  Label allocated, runtime_allocate;
  __ Allocate(size, v0, a2, a3, &runtime_allocate, TAG_OBJECT);
  __ jmp(&allocated);

  __ bind(&runtime_allocate);
  __ push(t1);
  __ li(a0, Operand(Smi::FromInt(size)));
  __ push(a0);
  __ CallRuntime(Runtime::kAllocateInNewSpace, 1);
  __ pop(t1);

  __ bind(&allocated);

  // After this, registers are used as follows:
  // v0: Newly allocated regexp.
  // t1: Materialized regexp.
  // a2: temp.
  __ CopyFields(v0, t1, a2.bit(), size / kPointerSize);
  context()->Plug(v0);
}

void FullCodeGenerator::EmitAccessor(Expression* expression) {
  if (expression == NULL) {
    __ LoadRoot(a1, Heap::kNullValueRootIndex);
    __ push(a1);
  } else {
    VisitForStackValue(expression);
  }
}

void FullCodeGenerator::VisitObjectLiteral(ObjectLiteral* expr) {
  Comment cmnt(masm_, "[ ObjectLiteral");
  Handle<FixedArray> constant_properties = expr->constant_properties();
  __ ld(a3, MemOperand(fp, JavaScriptFrameConstants::kFunctionOffset));
  __ ld(a3, FieldMemOperand(a3, JSFunction::kLiteralsOffset));
  __ li(a2, Operand(Smi::FromInt(expr->literal_index())));
  __ li(a1, Operand(constant_properties));
  int flags = expr->fast_elements()
      ? ObjectLiteral::kFastElements
      : ObjectLiteral::kNoFlags;
  flags |= expr->has_function()
      ? ObjectLiteral::kHasFunction
      : ObjectLiteral::kNoFlags;
  __ li(a0, Operand(Smi::FromInt(flags)));
  int properties_count = constant_properties->length() / 2;
  if ((FLAG_track_double_fields && expr->may_store_doubles()) ||
      expr->depth() > 1) {
    __ Push(a3, a2, a1, a0);
    __ CallRuntime(Runtime::kCreateObjectLiteral, 4);
  } else if (Serializer::enabled() || flags != ObjectLiteral::kFastElements ||
      properties_count > FastCloneShallowObjectStub::kMaximumClonedProperties) {
    __ Push(a3, a2, a1, a0);
    __ CallRuntime(Runtime::kCreateObjectLiteralShallow, 4);
  } else {
    FastCloneShallowObjectStub stub(properties_count);
    __ CallStub(&stub);
  }

  // If result_saved is true the result is on top of the stack.  If
  // result_saved is false the result is in v0.
  bool result_saved = false;

  // Mark all computed expressions that are bound to a key that
  // is shadowed by a later occurrence of the same key. For the
  // marked expressions, no store code is emitted.
  expr->CalculateEmitStore(zone());

  AccessorTable accessor_table(zone());
  for (int i = 0; i < expr->properties()->length(); i++) {
    ObjectLiteral::Property* property = expr->properties()->at(i);
    if (property->IsCompileTimeValue()) continue;

    Literal* key = property->key();
    Expression* value = property->value();
    if (!result_saved) {
      __ push(v0);  // Save result on stack.
      result_saved = true;
    }
    switch (property->kind()) {
      case ObjectLiteral::Property::CONSTANT:
        UNREACHABLE();
      case ObjectLiteral::Property::MATERIALIZED_LITERAL:
        ASSERT(!CompileTimeValue::IsCompileTimeValue(property->value()));
        // Fall through.
      case ObjectLiteral::Property::COMPUTED:
        if (key->handle()->IsInternalizedString()) {
          if (property->emit_store()) {
            VisitForAccumulatorValue(value);
            __ move(a0, result_register());
            __ li(a2, Operand(key->handle()));
            __ ld(a1, MemOperand(sp));
            Handle<Code> ic = is_classic_mode()
                ? isolate()->builtins()->StoreIC_Initialize()
                : isolate()->builtins()->StoreIC_Initialize_Strict();
            CallIC(ic, RelocInfo::CODE_TARGET, key->LiteralFeedbackId());
            PrepareForBailoutForId(key->id(), NO_REGISTERS);
          } else {
            VisitForEffect(value);
          }
          break;
        }
        // Duplicate receiver on stack.
        __ ld(a0, MemOperand(sp));
        __ push(a0);
        VisitForStackValue(key);
        VisitForStackValue(value);
        if (property->emit_store()) {
          __ li(a0, Operand(Smi::FromInt(NONE)));  // PropertyAttributes.
          __ push(a0);
          __ CallRuntime(Runtime::kSetProperty, 4);
        } else {
          __ Drop(3);
        }
        break;
      case ObjectLiteral::Property::PROTOTYPE:
        // Duplicate receiver on stack.
        __ ld(a0, MemOperand(sp));
        __ push(a0);
        VisitForStackValue(value);
        if (property->emit_store()) {
          __ CallRuntime(Runtime::kSetPrototype, 2);
        } else {
          __ Drop(2);
        }
        break;
      case ObjectLiteral::Property::GETTER:
        accessor_table.lookup(key)->second->getter = value;
        break;
      case ObjectLiteral::Property::SETTER:
        accessor_table.lookup(key)->second->setter = value;
        break;
    }
  }

  // Emit code to define accessors, using only a single call to the runtime for
  // each pair of corresponding getters and setters.
  for (AccessorTable::Iterator it = accessor_table.begin();
       it != accessor_table.end();
       ++it) {
    __ ld(a0, MemOperand(sp));  // Duplicate receiver.
    __ push(a0);
    VisitForStackValue(it->first);
    EmitAccessor(it->second->getter);
    EmitAccessor(it->second->setter);
    __ li(a0, Operand(Smi::FromInt(NONE)));
    __ push(a0);
    __ CallRuntime(Runtime::kDefineOrRedefineAccessorProperty, 5);
  }

  if (expr->has_function()) {
    ASSERT(result_saved);
    __ ld(a0, MemOperand(sp));
    __ push(a0);
    __ CallRuntime(Runtime::kToFastProperties, 1);
  }

  if (result_saved) {
    context()->PlugTOS();
  } else {
    context()->Plug(v0);
  }
}

void FullCodeGenerator::VisitArrayLiteral(ArrayLiteral* expr) {
  Comment cmnt(masm_, "[ ArrayLiteral");

  ZoneList<Expression*>* subexprs = expr->values();
  int length = subexprs->length();

  Handle<FixedArray> constant_elements = expr->constant_elements();
  ASSERT_EQ(2, constant_elements->length());
  ElementsKind constant_elements_kind =
      static_cast<ElementsKind>(Smi::cast(constant_elements->get(0))->value());
  bool has_fast_elements =
      IsFastObjectElementsKind(constant_elements_kind);
  Handle<FixedArrayBase> constant_elements_values(
      FixedArrayBase::cast(constant_elements->get(1)));

  __ move(a0, result_register());
  __ ld(a3, MemOperand(fp, JavaScriptFrameConstants::kFunctionOffset));
  __ ld(a3, FieldMemOperand(a3, JSFunction::kLiteralsOffset));
  __ li(a2, Operand(Smi::FromInt(expr->literal_index())));
  __ li(a1, Operand(constant_elements));
  if (has_fast_elements && constant_elements_values->map() ==
      isolate()->heap()->fixed_cow_array_map()) {
    FastCloneShallowArrayStub stub(
        FastCloneShallowArrayStub::COPY_ON_WRITE_ELEMENTS,
        DONT_TRACK_ALLOCATION_SITE,
        length);
    __ CallStub(&stub);
    __ IncrementCounter(isolate()->counters()->cow_arrays_created_stub(),
        1, a1, a2);
  } else if (expr->depth() > 1) {
    __ Push(a3, a2, a1);
    __ CallRuntime(Runtime::kCreateArrayLiteral, 3);
  } else if (Serializer::enabled() ||
      length > FastCloneShallowArrayStub::kMaximumClonedLength) {
    __ Push(a3, a2, a1);
    __ CallRuntime(Runtime::kCreateArrayLiteralShallow, 3);
  } else {
    ASSERT(IsFastSmiOrObjectElementsKind(constant_elements_kind) ||
           FLAG_smi_only_arrays);
    FastCloneShallowArrayStub::Mode mode =
        FastCloneShallowArrayStub::CLONE_ANY_ELEMENTS;
    AllocationSiteMode allocation_site_mode = FLAG_track_allocation_sites
        ? TRACK_ALLOCATION_SITE : DONT_TRACK_ALLOCATION_SITE;

    if (has_fast_elements) {
      mode = FastCloneShallowArrayStub::CLONE_ELEMENTS;
      allocation_site_mode = DONT_TRACK_ALLOCATION_SITE;
    }

    FastCloneShallowArrayStub stub(mode, allocation_site_mode, length);
    __ CallStub(&stub);
  }

  bool result_saved = false;  // Is the result saved to the stack?

  // Emit code to evaluate all the non-constant subexpressions and to store
  // them into the newly cloned array.
  for (int i = 0; i < length; i++) {
    Expression* subexpr = subexprs->at(i);
    // If the subexpression is a literal or a simple materialized literal it
    // is already set in the cloned array.
    if (subexpr->AsLiteral() != NULL ||
        CompileTimeValue::IsCompileTimeValue(subexpr)) {
      continue;
    }

    if (!result_saved) {
      __ push(v0);
      result_saved = true;
    }

    VisitForAccumulatorValue(subexpr);

    if (IsFastObjectElementsKind(constant_elements_kind)) {
      int offset = FixedArray::kHeaderSize + (i * kPointerSize);
      __ ld(t2, MemOperand(sp));  // Copy of array literal.
      __ ld(a1, FieldMemOperand(t2, JSObject::kElementsOffset));
      __ st(result_register(), FieldMemOperand(a1, offset));
      // Update the write barrier for the array store.
      __ RecordWriteField(a1, offset, result_register(), a2,
                          kRAHasBeenSaved, kDontSaveFPRegs,
                          EMIT_REMEMBERED_SET, INLINE_SMI_CHECK);
    } else {
      __ ld(a1, MemOperand(sp));  // Copy of array literal.
      __ ld(a2, FieldMemOperand(a1, JSObject::kMapOffset));
      __ li(a3, Operand(Smi::FromInt(i)));
      __ li(t0, Operand(Smi::FromInt(expr->literal_index())));
      //__ move(a0, result_register());
      StoreArrayLiteralElementStub stub;
      __ CallStub(&stub);
    }

    PrepareForBailoutForId(expr->GetIdForElement(i), NO_REGISTERS);
  }
  if (result_saved) {
    context()->PlugTOS();
  } else {
    context()->Plug(v0);
  }
}

void FullCodeGenerator::VisitAssignment(Assignment* expr) {
  Comment cmnt(masm_, "[ Assignment");
  // Invalid left-hand sides are rewritten to have a 'throw ReferenceError'
  // on the left-hand side.
  if (!expr->target()->IsValidLeftHandSide()) {
    VisitForEffect(expr->target());
    return;
  }

  // Left-hand side can only be a property, a global or a (parameter or local)
  // slot.
  enum LhsKind { VARIABLE, NAMED_PROPERTY, KEYED_PROPERTY };
  LhsKind assign_type = VARIABLE;
  Property* property = expr->target()->AsProperty();
  if (property != NULL) {
    assign_type = (property->key()->IsPropertyName())
        ? NAMED_PROPERTY
        : KEYED_PROPERTY;
  }

  // Evaluate LHS expression.
  switch (assign_type) {
    case VARIABLE:
      // Nothing to do here.
      break;
    case NAMED_PROPERTY:
      if (expr->is_compound()) {
        // We need the receiver both on the stack and in the accumulator.
        VisitForAccumulatorValue(property->obj());
        __ push(result_register());
      } else {
        VisitForStackValue(property->obj());
      }
      break;
    case KEYED_PROPERTY:
      // We need the key and receiver on both the stack and in v0 and a1.
      if (expr->is_compound()) {
        VisitForStackValue(property->obj());
        VisitForAccumulatorValue(property->key());
        __ ld(a1, MemOperand(sp, 0));
        __ push(v0);
      } else {
        VisitForStackValue(property->obj());
        VisitForStackValue(property->key());
      }
      break;
  }

  // For compound assignments we need another deoptimization point after the
  // variable/property load.
  if (expr->is_compound()) {
    { AccumulatorValueContext context(this);
      switch (assign_type) {
        case VARIABLE:
          EmitVariableLoad(expr->target()->AsVariableProxy());
          PrepareForBailout(expr->target(), TOS_REG);
          break;
        case NAMED_PROPERTY:
          EmitNamedPropertyLoad(property);
          PrepareForBailoutForId(property->LoadId(), TOS_REG);
          break;
        case KEYED_PROPERTY:
          EmitKeyedPropertyLoad(property);
          PrepareForBailoutForId(property->LoadId(), TOS_REG);
          break;
      }
    }

    Token::Value op = expr->binary_op();
    __ push(v0);  // Left operand goes on the stack.
    VisitForAccumulatorValue(expr->value());

    OverwriteMode mode = expr->value()->ResultOverwriteAllowed()
        ? OVERWRITE_RIGHT
        : NO_OVERWRITE;
    SetSourcePosition(expr->position() + 1);
    AccumulatorValueContext context(this);
    if (ShouldInlineSmiCase(op)) {
      EmitInlineSmiBinaryOp(expr->binary_operation(),
                            op,
                            mode,
                            expr->target(),
                            expr->value());
    } else {
      EmitBinaryOp(expr->binary_operation(), op, mode);
    }

    // Deoptimization point in case the binary operation may have side effects.
    PrepareForBailout(expr->binary_operation(), TOS_REG);
  } else {
    VisitForAccumulatorValue(expr->value());
  }

  // Record source position before possible IC call.
  SetSourcePosition(expr->position());

  // Store the value.
  switch (assign_type) {
    case VARIABLE:
      EmitVariableAssignment(expr->target()->AsVariableProxy()->var(),
                             expr->op());
      PrepareForBailoutForId(expr->AssignmentId(), TOS_REG);
      context()->Plug(v0);
      break;
    case NAMED_PROPERTY:
      EmitNamedPropertyAssignment(expr);
      break;
    case KEYED_PROPERTY:
      EmitKeyedPropertyAssignment(expr);
      break;
  }
}

void FullCodeGenerator::VisitYield(Yield* expr) {
  Comment cmnt(masm_, "[ Yield");
  // Evaluate yielded value first; the initial iterator definition depends on
  // this.  It stays on the stack while we update the iterator.
  VisitForStackValue(expr->expression());

  switch (expr->yield_kind()) {
    case Yield::INITIAL:
    case Yield::SUSPEND: {
      VisitForStackValue(expr->generator_object());
      __ CallRuntime(Runtime::kSuspendJSGeneratorObject, 1);
      __ ld(context_register(),
            MemOperand(fp, StandardFrameConstants::kContextOffset));

      Label resume;
      __ LoadRoot(at, Heap::kTheHoleValueRootIndex);
      __ Branch(&resume, ne, result_register(), Operand(at));
      if (expr->yield_kind() == Yield::SUSPEND) {
        EmitReturnIteratorResult(false);
      } else {
        __ pop(result_register());
        EmitReturnSequence();
      }

      __ bind(&resume);
      context()->Plug(result_register());
      break;
    }

    case Yield::FINAL: {
      VisitForAccumulatorValue(expr->generator_object());
      __ li(a1, Operand(Smi::FromInt(JSGeneratorObject::kGeneratorClosed)));
      __ st(a1, FieldMemOperand(result_register(),
                                JSGeneratorObject::kContinuationOffset));
      EmitReturnIteratorResult(true);
      break;
    }

    case Yield::DELEGATING: {
      VisitForStackValue(expr->generator_object());

      // Initial stack layout is as follows:
      // [sp + 1 * kPointerSize] iter
      // [sp + 0 * kPointerSize] g

      Label l_catch, l_try, l_resume, l_send, l_call, l_loop;
      // Initial send value is undefined.
      __ LoadRoot(a0, Heap::kUndefinedValueRootIndex);
      __ Branch(&l_send);

      // catch (e) { receiver = iter; f = iter.throw; arg = e; goto l_call; }
      __ bind(&l_catch);
      __ move(a0, v0);
      handler_table()->set(expr->index(), Smi::FromInt(l_catch.pos()));
      __ ld(a3, MemOperand(sp, 1 * kPointerSize));       // iter
      __ push(a3);                                       // iter
      __ push(a0);                                       // exception
      __ move(a0, a3);                                    // iter
      __ push(a0);                                       // push LoadIC state
      __ LoadRoot(a2, Heap::kthrow_stringRootIndex);     // "throw"
      Handle<Code> throw_ic = isolate()->builtins()->LoadIC_Initialize();
      CallIC(throw_ic);                                  // iter.throw in a0
      __ move(a0, v0);
      __ Addu(sp, sp, Operand(kPointerSize));            // drop LoadIC state
      __ jmp(&l_call);

      // try { received = yield result.value }
      __ bind(&l_try);
      __ pop(a0);                                        // result.value
      __ PushTryHandler(StackHandler::CATCH, expr->index());
      const int handler_size = StackHandlerConstants::kSize;
      __ push(a0);                                       // result.value
      __ ld(a3, MemOperand(sp, (0 + 1) * kPointerSize + handler_size));  // g
      __ push(a3);                                       // g
      __ CallRuntime(Runtime::kSuspendJSGeneratorObject, 1);
      __ move(a0, v0);
      __ ld(context_register(),
            MemOperand(fp, StandardFrameConstants::kContextOffset));
      __ LoadRoot(at, Heap::kTheHoleValueRootIndex);
      __ Branch(&l_resume, ne, a0, Operand(at));
      EmitReturnIteratorResult(false);
      __ move(a0, v0);
      __ bind(&l_resume);                                // received in a0
      __ PopTryHandler();

      // receiver = iter; f = iter.send; arg = received;
      __ bind(&l_send);
      __ ld(a3, MemOperand(sp, 1 * kPointerSize));       // iter
      __ push(a3);                                       // iter
      __ push(a0);                                       // received
      __ move(a0, a3);                                    // iter
      __ push(a0);                                       // push LoadIC state
      __ LoadRoot(a2, Heap::ksend_stringRootIndex);      // "send"
      Handle<Code> send_ic = isolate()->builtins()->LoadIC_Initialize();
      CallIC(send_ic);                                   // iter.send in a0
      __ move(a0, v0);
      __ Addu(sp, sp, Operand(kPointerSize));            // drop LoadIC state

      // result = f.call(receiver, arg);
      __ bind(&l_call);
      Label l_call_runtime;
      __ JumpIfSmi(a0, &l_call_runtime);
      __ GetObjectType(a0, a1, a1);
      __ Branch(&l_call_runtime, ne, a1, Operand(JS_FUNCTION_TYPE));
      __ move(a1, a0);
      ParameterCount count(1);
      __ InvokeFunction(a1, count, CALL_FUNCTION,
                        NullCallWrapper(), CALL_AS_METHOD);
      __ ld(cp, MemOperand(fp, StandardFrameConstants::kContextOffset));
      __ jmp(&l_loop);
      __ bind(&l_call_runtime);
      __ push(a0);
      __ CallRuntime(Runtime::kCall, 3);

      // val = result.value; if (!result.done) goto l_try;
      __ bind(&l_loop);
      __ move(a0, v0);
      // result.value
      __ push(a0);                                       // save result
      __ LoadRoot(a2, Heap::kvalue_stringRootIndex);     // "value"
      Handle<Code> value_ic = isolate()->builtins()->LoadIC_Initialize();
      CallIC(value_ic);                                  // result.value in a0
      __ move(a0, v0);
      __ pop(a1);                                        // result
      __ push(a0);                                       // result.value
      __ move(a0, a1);                                    // result
      __ push(a0);                                       // push LoadIC state
      __ LoadRoot(a2, Heap::kdone_stringRootIndex);      // "done"
      Handle<Code> done_ic = isolate()->builtins()->LoadIC_Initialize();
      CallIC(done_ic);                                   // result.done in v0
      __ Addu(sp, sp, Operand(kPointerSize));            // drop LoadIC state
      ToBooleanStub stub(v0);
      __ CallStub(&stub);
      __ Branch(&l_try, eq, v0, Operand(zero));

      // result.value
      __ pop(v0);                                        // result.value
      context()->DropAndPlug(2, v0);                     // drop iter and g
      break;
    }
  }
}

void FullCodeGenerator::EmitGeneratorResume(Expression *generator,
    Expression *value,
    JSGeneratorObject::ResumeMode resume_mode) {
  // The value stays in a0, and is ultimately read by the resumed generator, as
  // if the CallRuntime(Runtime::kSuspendJSGeneratorObject) returned it.  a1
  // will hold the generator object until the activation has been resumed.
  VisitForStackValue(generator);
  VisitForAccumulatorValue(value);
  __ pop(a1);

  // Check generator state.
  Label wrong_state, done;
  __ ld(a3, FieldMemOperand(a1, JSGeneratorObject::kContinuationOffset));
  STATIC_ASSERT(JSGeneratorObject::kGeneratorExecuting <= 0);
  STATIC_ASSERT(JSGeneratorObject::kGeneratorClosed <= 0);
  __ Branch(&wrong_state, le, a3, Operand(zero));

  // Load suspended function and context.
  __ ld(cp, FieldMemOperand(a1, JSGeneratorObject::kContextOffset));
  __ ld(t0, FieldMemOperand(a1, JSGeneratorObject::kFunctionOffset));

  // Load receiver and store as the first argument.
  __ ld(a2, FieldMemOperand(a1, JSGeneratorObject::kReceiverOffset));
  __ push(a2);

  // Push holes for the rest of the arguments to the generator function.
  __ ld(a3, FieldMemOperand(t0, JSFunction::kSharedFunctionInfoOffset));
  __ ld4s(a3,
          FieldMemOperand(a3, SharedFunctionInfo::kFormalParameterCountOffset));
  __ LoadRoot(a2, Heap::kTheHoleValueRootIndex);
  Label push_argument_holes, push_frame;
  __ bind(&push_argument_holes);
  __ Subu(a3, a3, Operand(1));
  __ Branch(&push_frame, lt, a3, Operand(zero));
  __ push(a2);
  __ jmp(&push_argument_holes);

  // Enter a new JavaScript frame, and initialize its slots as they were when
  // the generator was suspended.
  Label resume_frame;
  __ bind(&push_frame);
  __ Call(&resume_frame);
  __ jmp(&done);
  __ bind(&resume_frame);
  __ push(ra);  // Return address.
  __ push(fp);  // Caller's frame pointer.
  __ move(fp, sp);
  __ push(cp);  // Callee's context.
  __ push(t0);  // Callee's JS Function.

  // Load the operand stack size.
  __ ld(a3, FieldMemOperand(a1, JSGeneratorObject::kOperandStackOffset));
  __ ld(a3, FieldMemOperand(a3, FixedArray::kLengthOffset));
  __ SmiUntag(a3);

  // If we are sending a value and there is no operand stack, we can jump back
  // in directly.
  if (resume_mode == JSGeneratorObject::SEND) {
    Label slow_resume;
    __ Branch(&slow_resume, ne, a3, Operand(zero));
    __ ld(a3, FieldMemOperand(t0, JSFunction::kCodeEntryOffset));
    __ ld(a2, FieldMemOperand(a1, JSGeneratorObject::kContinuationOffset));
    __ SmiUntag(a2);
    __ Addu(a3, a3, Operand(a2));
    __ li(a2, Operand(Smi::FromInt(JSGeneratorObject::kGeneratorExecuting)));
    __ st(a2, FieldMemOperand(a1, JSGeneratorObject::kContinuationOffset));
    __ Jump(a3);
    __ bind(&slow_resume);
  }

  // Otherwise, we push holes for the operand stack and call the runtime to fix
  // up the stack and the handlers.
  Label push_operand_holes, call_resume;
  __ bind(&push_operand_holes);
  __ Subu(a3, a3, Operand(1));
  __ Branch(&call_resume, lt, a3, Operand(zero));
  __ push(a2);
  __ Branch(&push_operand_holes);
  __ bind(&call_resume);
  __ push(a1);
  __ push(result_register());
  __ Push(Smi::FromInt(resume_mode));
  __ CallRuntime(Runtime::kResumeJSGeneratorObject, 3);
  // Not reached: the runtime call returns elsewhere.
  __ stop("not-reached");

  // Throw error if we attempt to operate on a running generator.
  __ bind(&wrong_state);
  __ push(a1);
  __ CallRuntime(Runtime::kThrowGeneratorStateError, 1);

  __ bind(&done);
  context()->Plug(result_register());
}

void FullCodeGenerator::EmitReturnIteratorResult(bool done) {
  Label gc_required;
  Label allocated;

  Handle<Map> map(isolate()->native_context()->generator_result_map());

  __ Allocate(map->instance_size(), a0, a2, a3, &gc_required, TAG_OBJECT);

  __ bind(&allocated);
  __ li(a1, Operand(map));
  __ pop(a2);
  __ li(a3, Operand(isolate()->factory()->ToBoolean(done)));
  __ li(t0, Operand(isolate()->factory()->empty_fixed_array()));
  ASSERT_EQ(map->instance_size(), 5 * kPointerSize);
  __ st(a1, FieldMemOperand(a0, HeapObject::kMapOffset));
  __ st(t0, FieldMemOperand(a0, JSObject::kPropertiesOffset));
  __ st(t0, FieldMemOperand(a0, JSObject::kElementsOffset));
  __ st(a2,
        FieldMemOperand(a0, JSGeneratorObject::kResultValuePropertyOffset));
  __ st(a3,
        FieldMemOperand(a0, JSGeneratorObject::kResultDonePropertyOffset));

  // Only the value field needs a write barrier, as the other values are in the
  // root set.
  __ RecordWriteField(a0, JSGeneratorObject::kResultValuePropertyOffset,
                      a2, a3, kRAHasBeenSaved, kDontSaveFPRegs);

  if (done) {
    // Exit all nested statements.
    NestedStatement* current = nesting_stack_;
    int stack_depth = 0;
    int context_length = 0;
    while (current != NULL) {
      current = current->Exit(&stack_depth, &context_length);
    }
    __ Drop(stack_depth);
  }

  __ move(result_register(), a0);
  EmitReturnSequence();

  __ bind(&gc_required);
  __ Push(Smi::FromInt(map->instance_size()));
  __ CallRuntime(Runtime::kAllocateInNewSpace, 1);
  __ ld(context_register(),
        MemOperand(fp, StandardFrameConstants::kContextOffset));
  __ jmp(&allocated);
}

void FullCodeGenerator::EmitNamedPropertyLoad(Property* prop) {
  SetSourcePosition(prop->position());
  Literal* key = prop->key()->AsLiteral();
  __ move(a0, result_register());
  __ li(a2, Operand(key->handle()));
  // Call load IC. It has arguments receiver and property name a0 and a2.
  Handle<Code> ic = isolate()->builtins()->LoadIC_Initialize();
  CallIC(ic, RelocInfo::CODE_TARGET, prop->PropertyFeedbackId());
}

void FullCodeGenerator::EmitKeyedPropertyLoad(Property* prop) {
  SetSourcePosition(prop->position());
  __ move(a0, result_register());
  // Call keyed load IC. It has arguments key and receiver in a0 and a1.
  Handle<Code> ic = isolate()->builtins()->KeyedLoadIC_Initialize();
  CallIC(ic, RelocInfo::CODE_TARGET, prop->PropertyFeedbackId());
}

void FullCodeGenerator::EmitInlineSmiBinaryOp(BinaryOperation* expr,
                                              Token::Value op,
                                              OverwriteMode mode,
                                              Expression* left_expr,
                                              Expression* right_expr) {
  Label done, smi_case, stub_call;

  Register scratch1 = a2;
  Register scratch2 = a3;

  // Get the arguments.
  Register left = a1;
  Register right = a0;
  __ pop(left);
  __ move(a0, result_register());

  // Perform combined smi check on both operands.
  __ Or(scratch1, left, Operand(right));
  STATIC_ASSERT(kSmiTag == 0);
  JumpPatchSite patch_site(masm_);
  patch_site.EmitJumpIfSmi(scratch1, &smi_case);

  __ bind(&stub_call);
  BinaryOpStub stub(op, mode);
  CallIC(stub.GetCode(isolate()), RelocInfo::CODE_TARGET,
         expr->BinaryOperationFeedbackId());
  patch_site.EmitPatchInfo();
  __ jmp(&done);

  __ bind(&smi_case);
  // Smi case. This code works the same way as the smi-smi case in the type
  // recording binary operation stub, see
  // BinaryOpStub::GenerateSmiSmiOperation for comments.
  switch (op) {
    case Token::SAR:
      __ Branch(&stub_call);
      __ GetLeastBitsFromSmi(scratch1, right, 5);
      __ sra(right, left, scratch1);
      __ And(v0, right, Operand(0xFFFFFFFF00000000L));
      break;
    case Token::SHL: {
      __ Branch(&stub_call);
      __ SmiUntag(scratch1, left);
      __ GetLeastBitsFromSmi(scratch2, right, 5);
      __ sll(scratch1, scratch1, scratch2);
      __ Addu(scratch2, scratch1, Operand(0x40000000));
      __ Branch(&stub_call, lt, scratch2, Operand(zero));
      __ SmiTag(v0, scratch1);
      break;
    }
    case Token::SHR: {
      __ Branch(&stub_call);
      __ SmiUntag(scratch1, left);
      __ GetLeastBitsFromSmi(scratch2, right, 5);
      __ srl(scratch1, scratch1, scratch2);
      __ And(scratch2, scratch1, 0xC0000000);
      __ Branch(&stub_call, ne, scratch2, Operand(zero));
      __ SmiTag(v0, scratch1);
      break;
    }
    case Token::ADD:
      __ AdduAndCheckForOverflow(v0, left, right, scratch1);
      __ BranchOnOverflow(&stub_call, scratch1);
      break;
    case Token::SUB:
      __ SubuAndCheckForOverflow(v0, left, right, scratch1);
      __ BranchOnOverflow(&stub_call, scratch1);
      break;
    case Token::MUL: {
#if 0
      __ SmiUntag(scratch1, right);
      __ Mult(left, scratch1);
      __ mflo(scratch1);
      __ mfhi(scratch2);
      __ sra(scratch1, scratch1, 31);
      __ Branch(&stub_call, ne, scratch1, Operand(scratch2));
      __ mflo(v0);
      __ Branch(&done, ne, v0, Operand(zero));
      __ Addu(scratch2, right, left);
      __ Branch(&stub_call, lt, scratch2, Operand(zero));
      ASSERT(Smi::FromInt(0) == 0);
      __ move(v0, zero);
      break;
#else
      __ mul_hs_hs(scratch1, left, right);
      __ sra(scratch2, scratch1, 32);
      __ sra(at, scratch1, 30);
      __ Branch(&stub_call, ne, scratch2, Operand(at));
      __ sll(scratch1, scratch1, 32); // Convert to Smi
      __ move(v0, scratch1);
      __ Branch(&done, ne, v0, Operand(zero));
      __ Addu(scratch2, right, left);
      __ Branch(&stub_call, lt, scratch2, Operand(zero));
      ASSERT(Smi::FromInt(0) == 0);
      __ move(v0, zero);
#endif
    }
    case Token::BIT_OR:
      __ Or(v0, left, Operand(right));
      break;
    case Token::BIT_AND:
      __ And(v0, left, Operand(right));
      break;
    case Token::BIT_XOR:
      __ Xor(v0, left, Operand(right));
      break;
    default:
      UNREACHABLE();
  }

  __ bind(&done);
  context()->Plug(v0);
}

void FullCodeGenerator::EmitBinaryOp(BinaryOperation* expr,
                                     Token::Value op,
                                     OverwriteMode mode) {
  //__ move(a0, result_register());
  __ pop(a1);
  BinaryOpStub stub(op, mode);
  JumpPatchSite patch_site(masm_);    // unbound, signals no inlined smi code.
  CallIC(stub.GetCode(isolate()), RelocInfo::CODE_TARGET,
         expr->BinaryOperationFeedbackId());
  patch_site.EmitPatchInfo();
  context()->Plug(v0);
}

void FullCodeGenerator::EmitAssignment(Expression* expr) {
  // Invalid left-hand sides are rewritten by the parser to have a 'throw
  // ReferenceError' on the left-hand side.
  if (!expr->IsValidLeftHandSide()) {
    VisitForEffect(expr);
    return;
  }

  // Left-hand side can only be a property, a global or a (parameter or local)
  // slot.
  enum LhsKind { VARIABLE, NAMED_PROPERTY, KEYED_PROPERTY };
  LhsKind assign_type = VARIABLE;
  Property* prop = expr->AsProperty();
  if (prop != NULL) {
    assign_type = (prop->key()->IsPropertyName())
        ? NAMED_PROPERTY
        : KEYED_PROPERTY;
  }

  switch (assign_type) {
    case VARIABLE: {
      Variable* var = expr->AsVariableProxy()->var();
      EffectContext context(this);
      EmitVariableAssignment(var, Token::ASSIGN);
      break;
    }
    case NAMED_PROPERTY: {
      __ push(result_register());  // Preserve value.
      VisitForAccumulatorValue(prop->obj());
      __ move(a1, result_register());
      __ pop(a0);  // Restore value.
      __ li(a2, Operand(prop->key()->AsLiteral()->handle()));
      Handle<Code> ic = is_classic_mode()
          ? isolate()->builtins()->StoreIC_Initialize()
          : isolate()->builtins()->StoreIC_Initialize_Strict();
      CallIC(ic);
      break;
    }
    case KEYED_PROPERTY: {
      __ push(result_register());  // Preserve value.
      VisitForStackValue(prop->obj());
      VisitForAccumulatorValue(prop->key());
      __ move(a1, result_register());
      __ pop(a2);
      __ pop(a0);  // Restore value.
      Handle<Code> ic = is_classic_mode()
        ? isolate()->builtins()->KeyedStoreIC_Initialize()
        : isolate()->builtins()->KeyedStoreIC_Initialize_Strict();
      CallIC(ic);
      break;
    }
  }
  context()->Plug(v0);
}

void FullCodeGenerator::EmitVariableAssignment(Variable* var,
                                               Token::Value op) {
  if (var->IsUnallocated()) {
    // Global var, const, or let.
    __ move(a0, result_register());
    __ li(a2, Operand(var->name()));
    __ ld(a1, GlobalObjectOperand());
    Handle<Code> ic = is_classic_mode()
        ? isolate()->builtins()->StoreIC_Initialize()
        : isolate()->builtins()->StoreIC_Initialize_Strict();
    CallIC(ic, RelocInfo::CODE_TARGET_CONTEXT);

  } else if (op == Token::INIT_CONST) {
    // Const initializers need a write barrier.
    ASSERT(!var->IsParameter());  // No const parameters.
    if (var->IsStackLocal()) {
      Label skip;
      __ ld(a1, StackOperand(var));
      __ LoadRoot(t0, Heap::kTheHoleValueRootIndex);
      __ Branch(&skip, ne, a1, Operand(t0));
      __ st(result_register(), StackOperand(var));
      __ bind(&skip);
    } else {
      ASSERT(var->IsContextSlot() || var->IsLookupSlot());
      // Like var declarations, const declarations are hoisted to function
      // scope.  However, unlike var initializers, const initializers are
      // able to drill a hole to that function context, even from inside a
      // 'with' context.  We thus bypass the normal static scope lookup for
      // var->IsContextSlot().
      __ push(v0);
      __ li(a0, Operand(var->name()));
      __ Push(cp, a0);  // Context and name.
      __ CallRuntime(Runtime::kInitializeConstContextSlot, 3);
    }

  } else if (var->mode() == LET && op != Token::INIT_LET) {
    // Non-initializing assignment to let variable needs a write barrier.
    if (var->IsLookupSlot()) {
      __ push(v0);  // Value.
      __ li(a1, Operand(var->name()));
      __ li(a0, Operand(Smi::FromInt(language_mode())));
      __ Push(cp, a1, a0);  // Context, name, strict mode.
      __ CallRuntime(Runtime::kStoreContextSlot, 4);
    } else {
      ASSERT(var->IsStackAllocated() || var->IsContextSlot());
      Label assign;
      MemOperand location = VarOperand(var, a1);
      __ ld(a3, location);
      __ LoadRoot(t0, Heap::kTheHoleValueRootIndex);
      __ Branch(&assign, ne, a3, Operand(t0));
      __ li(a3, Operand(var->name()));
      __ push(a3);
      __ CallRuntime(Runtime::kThrowReferenceError, 1);
      // Perform the assignment.
      __ bind(&assign);
      __ st(result_register(), location);
      if (var->IsContextSlot()) {
        // RecordWrite may destroy all its register arguments.
        __ move(a3, result_register());
        int offset = Context::SlotOffset(var->index());
        __ RecordWriteContextSlot(
            a1, offset, a3, a2, kRAHasBeenSaved, kDontSaveFPRegs);
      }
    }

  } else if (!var->is_const_mode() || op == Token::INIT_CONST_HARMONY) {
    // Assignment to var or initializing assignment to let/const
    // in harmony mode.
    if (var->IsStackAllocated() || var->IsContextSlot()) {
      MemOperand location = VarOperand(var, a1);
      if (generate_debug_code_ && op == Token::INIT_LET) {
        // Check for an uninitialized let binding.
        __ ld(a2, location);
        __ LoadRoot(t0, Heap::kTheHoleValueRootIndex);
        __ Check(eq, "Let binding re-initialization.", a2, Operand(t0));
      }
      // Perform the assignment.
      __ st(v0, location);
      if (var->IsContextSlot()) {
        __ move(a3, v0);
        int offset = Context::SlotOffset(var->index());
        __ RecordWriteContextSlot(
            a1, offset, a3, a2, kRAHasBeenSaved, kDontSaveFPRegs);
      }
    } else {
      ASSERT(var->IsLookupSlot());
      __ push(v0);  // Value.
      __ li(a1, Operand(var->name()));
      __ li(a0, Operand(Smi::FromInt(language_mode())));
      __ Push(cp, a1, a0);  // Context, name, strict mode.
      __ CallRuntime(Runtime::kStoreContextSlot, 4);
    }
  }
    // Non-initializing assignments to consts are ignored.
}

void FullCodeGenerator::EmitNamedPropertyAssignment(Assignment* expr) {
  // Assignment to a property, using a named store IC.
  Property* prop = expr->target()->AsProperty();
  ASSERT(prop != NULL);
  ASSERT(prop->key()->AsLiteral() != NULL);

  // Record source code position before IC call.
  SetSourcePosition(expr->position());
  __ move(a0, result_register());  // Load the value.
  __ li(a2, Operand(prop->key()->AsLiteral()->handle()));
  __ pop(a1);

  Handle<Code> ic = is_classic_mode()
        ? isolate()->builtins()->StoreIC_Initialize()
        : isolate()->builtins()->StoreIC_Initialize_Strict();
  CallIC(ic, RelocInfo::CODE_TARGET, expr->AssignmentFeedbackId());

  PrepareForBailoutForId(expr->AssignmentId(), TOS_REG);
  context()->Plug(v0);
}

void FullCodeGenerator::EmitKeyedPropertyAssignment(Assignment* expr) {
  // Assignment to a property, using a keyed store IC.

  // Record source code position before IC call.
  SetSourcePosition(expr->position());
  // Call keyed store IC.
  // The arguments are:
  // - a0 is the value,
  // - a1 is the key,
  // - a2 is the receiver.
  __ move(a0, result_register());
  __ pop(a1);  // Key.
  __ pop(a2);

  Handle<Code> ic = is_classic_mode()
      ? isolate()->builtins()->KeyedStoreIC_Initialize()
      : isolate()->builtins()->KeyedStoreIC_Initialize_Strict();
  CallIC(ic, RelocInfo::CODE_TARGET, expr->AssignmentFeedbackId());

  PrepareForBailoutForId(expr->AssignmentId(), TOS_REG);
  context()->Plug(v0);
}

void FullCodeGenerator::VisitProperty(Property* expr) {
  Comment cmnt(masm_, "[ Property");
  Expression* key = expr->key();

  if (key->IsPropertyName()) {
    VisitForAccumulatorValue(expr->obj());
    EmitNamedPropertyLoad(expr);
    PrepareForBailoutForId(expr->LoadId(), TOS_REG);
    context()->Plug(v0);
  } else {
    VisitForStackValue(expr->obj());
    VisitForAccumulatorValue(expr->key());
    __ pop(a1);
    EmitKeyedPropertyLoad(expr);
    context()->Plug(v0);
  }
}

void FullCodeGenerator::CallIC(Handle<Code> code,
                               RelocInfo::Mode rmode,
                               TypeFeedbackId id) {
  ic_total_count_++;
  __ Call(code, rmode, id);
}

void FullCodeGenerator::EmitCallWithIC(Call* expr,
                                       Handle<Object> name,
                                       RelocInfo::Mode mode) {
  // Code common for calls using the IC.
  ZoneList<Expression*>* args = expr->arguments();
  int arg_count = args->length();
  { PreservePositionScope scope(masm()->positions_recorder());
    for (int i = 0; i < arg_count; i++) {
      VisitForStackValue(args->at(i));
    }
    __ li(a2, Operand(name));
  }
  // Record source position for debugger.
  SetSourcePosition(expr->position());
  // Call the IC initialization code.
  Handle<Code> ic =
      isolate()->stub_cache()->ComputeCallInitialize(arg_count, mode);
  CallIC(ic, mode, expr->CallFeedbackId());
  RecordJSReturnSite(expr);
  // Restore context register.
  __ ld(cp, MemOperand(fp, StandardFrameConstants::kContextOffset));
  context()->Plug(v0);
}

void FullCodeGenerator::EmitKeyedCallWithIC(Call* expr,
                                            Expression* key) {
  // Load the key.
  VisitForAccumulatorValue(key);

  // Swap the name of the function and the receiver on the stack to follow
  // the calling convention for call ICs.
  __ pop(a1);
  __ push(v0);
  __ push(a1);

  // Code common for calls using the IC.
  ZoneList<Expression*>* args = expr->arguments();
  int arg_count = args->length();
  { PreservePositionScope scope(masm()->positions_recorder());
    for (int i = 0; i < arg_count; i++) {
      VisitForStackValue(args->at(i));
    }
  }
  // Record source position for debugger.
  SetSourcePosition(expr->position());
  // Call the IC initialization code.
  Handle<Code> ic =
      isolate()->stub_cache()->ComputeKeyedCallInitialize(arg_count);
  __ ld(a2, MemOperand(sp, (arg_count + 1) * kPointerSize));  // Key.
  CallIC(ic, RelocInfo::CODE_TARGET, expr->CallFeedbackId());
  RecordJSReturnSite(expr);
  // Restore context register.
  __ ld(cp, MemOperand(fp, StandardFrameConstants::kContextOffset));
  context()->DropAndPlug(1, v0);  // Drop the key still on the stack.
}

void FullCodeGenerator::EmitCallWithStub(Call* expr, CallFunctionFlags flags) {
  // Code common for calls using the call stub.
  ZoneList<Expression*>* args = expr->arguments();
  int arg_count = args->length();
  { PreservePositionScope scope(masm()->positions_recorder());
    for (int i = 0; i < arg_count; i++) {
      VisitForStackValue(args->at(i));
    }
  }
  // Record source position for debugger.
  SetSourcePosition(expr->position());

  // Record call targets.
  flags = static_cast<CallFunctionFlags>(flags | RECORD_CALL_TARGET);
  Handle<Object> uninitialized =
      TypeFeedbackCells::UninitializedSentinel(isolate());
  Handle<JSGlobalPropertyCell> cell =
      isolate()->factory()->NewJSGlobalPropertyCell(uninitialized);
  RecordTypeFeedbackCell(expr->CallFeedbackId(), cell);
  __ li(a2, Operand(cell));

  CallFunctionStub stub(arg_count, flags);
  __ ld(a1, MemOperand(sp, (arg_count + 1) * kPointerSize));
  __ CallStub(&stub, expr->CallFeedbackId());
  RecordJSReturnSite(expr);
  // Restore context register.
  __ ld(cp, MemOperand(fp, StandardFrameConstants::kContextOffset));
  context()->DropAndPlug(1, v0);
}

void FullCodeGenerator::EmitResolvePossiblyDirectEval(int arg_count) {
  // Push copy of the first argument or undefined if it doesn't exist.
  if (arg_count > 0) {
    __ ld(a1, MemOperand(sp, arg_count * kPointerSize));
  } else {
    __ LoadRoot(a1, Heap::kUndefinedValueRootIndex);
  }
  __ push(a1);

  // Push the receiver of the enclosing function.
  int receiver_offset = 2 + info_->scope()->num_parameters();
  __ ld(a1, MemOperand(fp, receiver_offset * kPointerSize));
  __ push(a1);
  // Push the language mode.
  __ li(a1, Operand(Smi::FromInt(language_mode())));
  __ push(a1);

  // Push the start position of the scope the calls resides in.
  __ li(a1, Operand(Smi::FromInt(scope()->start_position())));
  __ push(a1);

  // Do the runtime call.
  __ CallRuntime(Runtime::kResolvePossiblyDirectEval, 5);
}


void FullCodeGenerator::VisitCall(Call* expr) {
#ifdef DEBUG
  // We want to verify that RecordJSReturnSite gets called on all paths
  // through this function.  Avoid early returns.
  expr->return_is_recorded_ = false;
#endif

  Comment cmnt(masm_, "[ Call");
  Expression* callee = expr->expression();
  VariableProxy* proxy = callee->AsVariableProxy();
  Property* property = callee->AsProperty();

  if (proxy != NULL && proxy->var()->is_possibly_eval(isolate())) {
    // In a call to eval, we first call %ResolvePossiblyDirectEval to
    // resolve the function we need to call and the receiver of the
    // call.  Then we call the resolved function using the given
    // arguments.
    ZoneList<Expression*>* args = expr->arguments();
    int arg_count = args->length();

    { PreservePositionScope pos_scope(masm()->positions_recorder());
      VisitForStackValue(callee);
      __ LoadRoot(a2, Heap::kUndefinedValueRootIndex);
      __ push(a2);  // Reserved receiver slot.

      // Push the arguments.
      for (int i = 0; i < arg_count; i++) {
        VisitForStackValue(args->at(i));
      }

      // Push a copy of the function (found below the arguments) and
      // resolve eval.
      __ ld(a1, MemOperand(sp, (arg_count + 1) * kPointerSize));
      __ push(a1);
      EmitResolvePossiblyDirectEval(arg_count);

      // The runtime call returns a pair of values in v0 (function) and
      // v1 (receiver). Touch up the stack with the right values.
      __ st(v0, MemOperand(sp, (arg_count + 1) * kPointerSize));
      __ st(r1, MemOperand(sp, arg_count * kPointerSize));
    }
    // Record source position for debugger.
    SetSourcePosition(expr->position());
    CallFunctionStub stub(arg_count, RECEIVER_MIGHT_BE_IMPLICIT);
    __ ld(a1, MemOperand(sp, (arg_count + 1) * kPointerSize));
    __ CallStub(&stub);
    RecordJSReturnSite(expr);
    // Restore context register.
    __ ld(cp, MemOperand(fp, StandardFrameConstants::kContextOffset));
    context()->DropAndPlug(1, v0);
  } else if (proxy != NULL && proxy->var()->IsUnallocated()) {
    // Push global object as receiver for the call IC.
    __ ld(a0, GlobalObjectOperand());
    __ push(a0);
    EmitCallWithIC(expr, proxy->name(), RelocInfo::CODE_TARGET_CONTEXT);
  } else if (proxy != NULL && proxy->var()->IsLookupSlot()) {
    // Call to a lookup slot (dynamically introduced variable).
    Label slow, done;

    { PreservePositionScope scope(masm()->positions_recorder());
      // Generate code for loading from variables potentially shadowed
      // by eval-introduced variables.
      EmitDynamicLookupFastCase(proxy->var(), NOT_INSIDE_TYPEOF, &slow, &done);
    }

    __ bind(&slow);
    // Call the runtime to find the function to call (returned in v0)
    // and the object holding it (returned in v1).
    __ push(context_register());
    __ li(a2, Operand(proxy->name()));
    __ push(a2);
    __ CallRuntime(Runtime::kLoadContextSlot, 2);
    __ Push(v0, r1);  // Function, receiver.

    // If fast case code has been generated, emit code to push the
    // function and receiver and have the slow path jump around this
    // code.
    if (done.is_linked()) {
      Label call;
      __ Branch(&call);
      __ bind(&done);
      // Push function.
      __ push(v0);
      // The receiver is implicitly the global receiver. Indicate this
      // by passing the hole to the call function stub.
      __ LoadRoot(a1, Heap::kTheHoleValueRootIndex);
      __ push(a1);
      __ bind(&call);
    }

    // The receiver is either the global receiver or an object found
    // by LoadContextSlot. That object could be the hole if the
    // receiver is implicitly the global object.
    EmitCallWithStub(expr, RECEIVER_MIGHT_BE_IMPLICIT);
  } else if (property != NULL) {
    { PreservePositionScope scope(masm()->positions_recorder());
      VisitForStackValue(property->obj());
    }
    if (property->key()->IsPropertyName()) {
      EmitCallWithIC(expr,
                     property->key()->AsLiteral()->handle(),
                     RelocInfo::CODE_TARGET);
    } else {
      EmitKeyedCallWithIC(expr, property->key());
    }
  } else {
    // Call to an arbitrary expression not handled specially above.
    { PreservePositionScope scope(masm()->positions_recorder());
      VisitForStackValue(callee);
    }
    // Load global receiver object.
    __ ld(a1, GlobalObjectOperand());
    __ ld(a1, FieldMemOperand(a1, GlobalObject::kGlobalReceiverOffset));
    __ push(a1);
    // Emit function call.
    EmitCallWithStub(expr, NO_CALL_FUNCTION_FLAGS);
  }

#ifdef DEBUG
  // RecordJSReturnSite should have been called.
  ASSERT(expr->return_is_recorded_);
#endif
}

void FullCodeGenerator::VisitCallNew(CallNew* expr) {
  Comment cmnt(masm_, "[ CallNew");
  // According to ECMA-262, section 11.2.2, page 44, the function
  // expression in new calls must be evaluated before the
  // arguments.

  // Push constructor on the stack.  If it's not a function it's used as
  // receiver for CALL_NON_FUNCTION, otherwise the value on the stack is
  // ignored.
  VisitForStackValue(expr->expression());

  // Push the arguments ("left-to-right") on the stack.
  ZoneList<Expression*>* args = expr->arguments();
  int arg_count = args->length();
  for (int i = 0; i < arg_count; i++) {
    VisitForStackValue(args->at(i));
  }

  // Call the construct call builtin that handles allocation and
  // constructor invocation.
  SetSourcePosition(expr->position());

  // Load function and argument count into a1 and a0.
  __ li(a0, Operand(arg_count));
  __ ld(a1, MemOperand(sp, arg_count * kPointerSize));

  // Record call targets in unoptimized code.
  Handle<Object> uninitialized =
     TypeFeedbackCells::UninitializedSentinel(isolate());
  Handle<JSGlobalPropertyCell> cell =
      isolate()->factory()->NewJSGlobalPropertyCell(uninitialized);
  RecordTypeFeedbackCell(expr->CallNewFeedbackId(), cell);
  __ li(a2, Operand(cell));

  CallConstructStub stub(RECORD_CALL_TARGET);
  __ Call(stub.GetCode(isolate()), RelocInfo::CONSTRUCT_CALL);
  PrepareForBailoutForId(expr->ReturnId(), TOS_REG);
  context()->Plug(v0);
}


void FullCodeGenerator::EmitIsSmi(CallRuntime* expr) {
  ZoneList<Expression*>* args = expr->arguments();
  ASSERT(args->length() == 1);

  VisitForAccumulatorValue(args->at(0));

  Label materialize_true, materialize_false;
  Label* if_true = NULL;
  Label* if_false = NULL;
  Label* fall_through = NULL;
  context()->PrepareTest(&materialize_true, &materialize_false,
                         &if_true, &if_false, &fall_through);

  PrepareForBailoutBeforeSplit(expr, true, if_true, if_false);
  __ And(t0, v0, Operand(kSmiTagMask));
  Split(eq, t0, Operand(zero), if_true, if_false, fall_through);

  context()->Plug(if_true, if_false);
}

void FullCodeGenerator::EmitIsNonNegativeSmi(CallRuntime* expr) {
  ZoneList<Expression*>* args = expr->arguments();
  ASSERT(args->length() == 1);

  VisitForAccumulatorValue(args->at(0));

  Label materialize_true, materialize_false;
  Label* if_true = NULL;
  Label* if_false = NULL;
  Label* fall_through = NULL;
  context()->PrepareTest(&materialize_true, &materialize_false,
                         &if_true, &if_false, &fall_through);

  PrepareForBailoutBeforeSplit(expr, true, if_true, if_false);
  __ And(at, v0, Operand(kSmiTagMask | 0x8000000000000000L));
  Split(eq, at, Operand(zero), if_true, if_false, fall_through);

  context()->Plug(if_true, if_false);
}

void FullCodeGenerator::EmitIsObject(CallRuntime* expr) {
  ZoneList<Expression*>* args = expr->arguments();
  ASSERT(args->length() == 1);

  VisitForAccumulatorValue(args->at(0));

  Label materialize_true, materialize_false;
  Label* if_true = NULL;
  Label* if_false = NULL;
  Label* fall_through = NULL;
  context()->PrepareTest(&materialize_true, &materialize_false,
                         &if_true, &if_false, &fall_through);

  __ JumpIfSmi(v0, if_false);
  __ LoadRoot(at, Heap::kNullValueRootIndex);
  __ Branch(if_true, eq, v0, Operand(at));
  __ ld(a2, FieldMemOperand(v0, HeapObject::kMapOffset));
  // Undetectable objects behave like undefined when tested with typeof.
  __ ld1u(a1, FieldMemOperand(a2, Map::kBitFieldOffset));
  __ And(at, a1, Operand(1 << Map::kIsUndetectable));
  __ Branch(if_false, ne, at, Operand(zero));
  __ ld1u(a1, FieldMemOperand(a2, Map::kInstanceTypeOffset));
  __ Branch(if_false, lt, a1, Operand(FIRST_NONCALLABLE_SPEC_OBJECT_TYPE));
  PrepareForBailoutBeforeSplit(expr, true, if_true, if_false);
  Split(le, a1, Operand(LAST_NONCALLABLE_SPEC_OBJECT_TYPE),
        if_true, if_false, fall_through);

  context()->Plug(if_true, if_false);
}

void FullCodeGenerator::EmitIsSpecObject(CallRuntime* expr) {
  ZoneList<Expression*>* args = expr->arguments();
  ASSERT(args->length() == 1);

  VisitForAccumulatorValue(args->at(0));

  Label materialize_true, materialize_false;
  Label* if_true = NULL;
  Label* if_false = NULL;
  Label* fall_through = NULL;
  context()->PrepareTest(&materialize_true, &materialize_false,
                         &if_true, &if_false, &fall_through);

  __ JumpIfSmi(v0, if_false);
  __ GetObjectType(v0, a1, a1);
  PrepareForBailoutBeforeSplit(expr, true, if_true, if_false);
  Split(ge, a1, Operand(FIRST_SPEC_OBJECT_TYPE),
        if_true, if_false, fall_through);

  context()->Plug(if_true, if_false);
}

void FullCodeGenerator::EmitIsUndetectableObject(CallRuntime* expr) {
  ZoneList<Expression*>* args = expr->arguments();
  ASSERT(args->length() == 1);

  VisitForAccumulatorValue(args->at(0));

  Label materialize_true, materialize_false;
  Label* if_true = NULL;
  Label* if_false = NULL;
  Label* fall_through = NULL;
  context()->PrepareTest(&materialize_true, &materialize_false,
                         &if_true, &if_false, &fall_through);

  __ JumpIfSmi(v0, if_false);
  __ ld(a1, FieldMemOperand(v0, HeapObject::kMapOffset));
  __ ld1u(a1, FieldMemOperand(a1, Map::kBitFieldOffset));
  __ And(at2, a1, Operand(1 << Map::kIsUndetectable));
  PrepareForBailoutBeforeSplit(expr, true, if_true, if_false);
  Split(ne, at2, Operand(zero), if_true, if_false, fall_through);

  context()->Plug(if_true, if_false);
}

void FullCodeGenerator::EmitIsStringWrapperSafeForDefaultValueOf(
    CallRuntime* expr) {
  ZoneList<Expression*>* args = expr->arguments();
  ASSERT(args->length() == 1);

  VisitForAccumulatorValue(args->at(0));

  Label materialize_true, materialize_false;
  Label* if_true = NULL;
  Label* if_false = NULL;
  Label* fall_through = NULL;
  context()->PrepareTest(&materialize_true, &materialize_false,
                         &if_true, &if_false, &fall_through);

  __ AssertNotSmi(v0);

  __ ld(a1, FieldMemOperand(v0, HeapObject::kMapOffset));
  __ ld1u(t0, FieldMemOperand(a1, Map::kBitField2Offset));
  __ And(t0, t0, 1 << Map::kStringWrapperSafeForDefaultValueOf);
  __ Branch(if_true, ne, t0, Operand(zero));

  // Check for fast case object. Generate false result for slow case object.
  __ ld(a2, FieldMemOperand(v0, JSObject::kPropertiesOffset));
  __ ld(a2, FieldMemOperand(a2, HeapObject::kMapOffset));
  __ LoadRoot(t0, Heap::kHashTableMapRootIndex);
  __ Branch(if_false, eq, a2, Operand(t0));

  // Look for valueOf name in the descriptor array, and indicate false if
  // found. Since we omit an enumeration index check, if it is added via a
  // transition that shares its descriptor array, this is a false positive.
  Label entry, loop, done;

  // Skip loop if no descriptors are valid.
  __ NumberOfOwnDescriptors(a3, a1);
  __ Branch(&done, eq, a3, Operand(zero));

  __ LoadInstanceDescriptors(a1, t0);
  // t0: descriptor array.
  // a3: valid entries in the descriptor array.
  STATIC_ASSERT(kSmiTag == 0);
  STATIC_ASSERT(kSmiTagSize == 1);
  STATIC_ASSERT(kPointerSize == 8);
  __ li(at, Operand(DescriptorArray::kDescriptorSize));
  __ Mul(a3, a3, at);
  // Calculate location of the first key name.
  __ Addu(t0, t0, Operand(DescriptorArray::kFirstOffset - kHeapObjectTag));
  // Calculate the end of the descriptor array.
  __ move(a2, t0);
  __ sra(t1, a3, kSmiTagSize + kSmiShiftSize);
  __ sll(t1, t1, kPointerSizeLog2);
  __ Addu(a2, a2, t1);

  // Loop through all the keys in the descriptor array. If one of these is the
  // string "valueOf" the result is false.
  // The use of t2 to store the valueOf string assumes that it is not otherwise
  // used in the loop below.
  __ li(t2, Operand(FACTORY->value_of_string()));
  __ jmp(&entry);
  __ bind(&loop);
  __ ld(a3, MemOperand(t0, 0));
  __ Branch(if_false, eq, a3, Operand(t2));
  __ Addu(t0, t0, Operand(DescriptorArray::kDescriptorSize * kPointerSize));
  __ bind(&entry);
  __ Branch(&loop, ne, t0, Operand(a2));

  __ bind(&done);
  // If a valueOf property is not found on the object check that its
  // prototype is the un-modified String prototype. If not result is false.
  __ ld(a2, FieldMemOperand(a1, Map::kPrototypeOffset));
  __ JumpIfSmi(a2, if_false);
  __ ld(a2, FieldMemOperand(a2, HeapObject::kMapOffset));
  __ ld(a3, ContextOperand(cp, Context::GLOBAL_OBJECT_INDEX));
  __ ld(a3, FieldMemOperand(a3, GlobalObject::kNativeContextOffset));
  __ ld(a3, ContextOperand(a3, Context::STRING_FUNCTION_PROTOTYPE_MAP_INDEX));
  __ Branch(if_false, ne, a2, Operand(a3));

  // Set the bit in the map to indicate that it has been checked safe for
  // default valueOf and set true result.
  __ ld1u(a2, FieldMemOperand(a1, Map::kBitField2Offset));
  __ Or(a2, a2, Operand(1 << Map::kStringWrapperSafeForDefaultValueOf));
  __ st1(a2, FieldMemOperand(a1, Map::kBitField2Offset));
  __ jmp(if_true);

  PrepareForBailoutBeforeSplit(expr, true, if_true, if_false);
  context()->Plug(if_true, if_false);
}

void FullCodeGenerator::EmitIsFunction(CallRuntime* expr) {
  ZoneList<Expression*>* args = expr->arguments();
  ASSERT(args->length() == 1);

  VisitForAccumulatorValue(args->at(0));

  Label materialize_true, materialize_false;
  Label* if_true = NULL;
  Label* if_false = NULL;
  Label* fall_through = NULL;
  context()->PrepareTest(&materialize_true, &materialize_false,
                         &if_true, &if_false, &fall_through);

  __ JumpIfSmi(v0, if_false);
  __ GetObjectType(v0, a1, a2);
  PrepareForBailoutBeforeSplit(expr, true, if_true, if_false);
  __ Branch(if_true, eq, a2, Operand(JS_FUNCTION_TYPE));
  __ Branch(if_false);

  context()->Plug(if_true, if_false);
}

void FullCodeGenerator::EmitIsArray(CallRuntime* expr) {
  ZoneList<Expression*>* args = expr->arguments();
  ASSERT(args->length() == 1);

  VisitForAccumulatorValue(args->at(0));

  Label materialize_true, materialize_false;
  Label* if_true = NULL;
  Label* if_false = NULL;
  Label* fall_through = NULL;
  context()->PrepareTest(&materialize_true, &materialize_false,
                         &if_true, &if_false, &fall_through);

  __ JumpIfSmi(v0, if_false);
  __ GetObjectType(v0, a1, a1);
  PrepareForBailoutBeforeSplit(expr, true, if_true, if_false);
  Split(eq, a1, Operand(JS_ARRAY_TYPE),
        if_true, if_false, fall_through);

  context()->Plug(if_true, if_false);
}

void FullCodeGenerator::EmitIsRegExp(CallRuntime* expr) {
  ZoneList<Expression*>* args = expr->arguments();
  ASSERT(args->length() == 1);

  VisitForAccumulatorValue(args->at(0));

  Label materialize_true, materialize_false;
  Label* if_true = NULL;
  Label* if_false = NULL;
  Label* fall_through = NULL;
  context()->PrepareTest(&materialize_true, &materialize_false,
                         &if_true, &if_false, &fall_through);

  __ JumpIfSmi(v0, if_false);
  __ GetObjectType(v0, a1, a1);
  PrepareForBailoutBeforeSplit(expr, true, if_true, if_false);
  Split(eq, a1, Operand(JS_REGEXP_TYPE), if_true, if_false, fall_through);

  context()->Plug(if_true, if_false);
}

void FullCodeGenerator::EmitIsConstructCall(CallRuntime* expr) {
  ASSERT(expr->arguments()->length() == 0);

  Label materialize_true, materialize_false;
  Label* if_true = NULL;
  Label* if_false = NULL;
  Label* fall_through = NULL;
  context()->PrepareTest(&materialize_true, &materialize_false,
                         &if_true, &if_false, &fall_through);

  // Get the frame pointer for the calling frame.
  __ ld(a2, MemOperand(fp, StandardFrameConstants::kCallerFPOffset));

  // Skip the arguments adaptor frame if it exists.
  Label check_frame_marker;
  __ ld(a1, MemOperand(a2, StandardFrameConstants::kContextOffset));
  __ Branch(&check_frame_marker, ne,
            a1, Operand(Smi::FromInt(StackFrame::ARGUMENTS_ADAPTOR)));
  __ ld(a2, MemOperand(a2, StandardFrameConstants::kCallerFPOffset));

  // Check the marker in the calling frame.
  __ bind(&check_frame_marker);
  __ ld(a1, MemOperand(a2, StandardFrameConstants::kMarkerOffset));
  PrepareForBailoutBeforeSplit(expr, true, if_true, if_false);
  Split(eq, a1, Operand(Smi::FromInt(StackFrame::CONSTRUCT)),
        if_true, if_false, fall_through);

  context()->Plug(if_true, if_false);
}

void FullCodeGenerator::EmitObjectEquals(CallRuntime* expr) {
  ZoneList<Expression*>* args = expr->arguments();
  ASSERT(args->length() == 2);

  // Load the two objects into registers and perform the comparison.
  VisitForStackValue(args->at(0));
  VisitForAccumulatorValue(args->at(1));

  Label materialize_true, materialize_false;
  Label* if_true = NULL;
  Label* if_false = NULL;
  Label* fall_through = NULL;
  context()->PrepareTest(&materialize_true, &materialize_false,
                         &if_true, &if_false, &fall_through);

  __ pop(a1);
  PrepareForBailoutBeforeSplit(expr, true, if_true, if_false);
  Split(eq, v0, Operand(a1), if_true, if_false, fall_through);

  context()->Plug(if_true, if_false);
}

void FullCodeGenerator::EmitArguments(CallRuntime* expr) {
  ZoneList<Expression*>* args = expr->arguments();
  ASSERT(args->length() == 1);

  // ArgumentsAccessStub expects the key in a1 and the formal
  // parameter count in a0.
  VisitForAccumulatorValue(args->at(0));
  __ move(a1, v0);
  __ li(a0, Operand(Smi::FromInt(info_->scope()->num_parameters())));
  ArgumentsAccessStub stub(ArgumentsAccessStub::READ_ELEMENT);
  __ CallStub(&stub);
  context()->Plug(v0);
}

void FullCodeGenerator::EmitArgumentsLength(CallRuntime* expr) {
  ASSERT(expr->arguments()->length() == 0);
  Label exit;
  // Get the number of formal parameters.
  __ li(v0, Operand(Smi::FromInt(info_->scope()->num_parameters())));

  // Check if the calling frame is an arguments adaptor frame.
  __ ld(a2, MemOperand(fp, StandardFrameConstants::kCallerFPOffset));
  __ ld(a3, MemOperand(a2, StandardFrameConstants::kContextOffset));
  __ Branch(&exit, ne, a3,
            Operand(Smi::FromInt(StackFrame::ARGUMENTS_ADAPTOR)));

  // Arguments adaptor case: Read the arguments length from the
  // adaptor frame.
  __ ld(v0, MemOperand(a2, ArgumentsAdaptorFrameConstants::kLengthOffset));

  __ bind(&exit);
  context()->Plug(v0);
}

void FullCodeGenerator::EmitClassOf(CallRuntime* expr) {
  ZoneList<Expression*>* args = expr->arguments();
  ASSERT(args->length() == 1);
  Label done, null, function, non_function_constructor;

  VisitForAccumulatorValue(args->at(0));

  // If the object is a smi, we return null.
  __ JumpIfSmi(v0, &null);

  // Check that the object is a JS object but take special care of JS
  // functions to make sure they have 'Function' as their class.
  // Assume that there are only two callable types, and one of them is at
  // either end of the type range for JS object types. Saves extra comparisons.
  STATIC_ASSERT(NUM_OF_CALLABLE_SPEC_OBJECT_TYPES == 2);
  __ GetObjectType(v0, v0, a1);  // Map is now in v0.
  __ Branch(&null, lt, a1, Operand(FIRST_SPEC_OBJECT_TYPE));

  STATIC_ASSERT(FIRST_NONCALLABLE_SPEC_OBJECT_TYPE ==
                FIRST_SPEC_OBJECT_TYPE + 1);
  __ Branch(&function, eq, a1, Operand(FIRST_SPEC_OBJECT_TYPE));

  STATIC_ASSERT(LAST_NONCALLABLE_SPEC_OBJECT_TYPE ==
                LAST_SPEC_OBJECT_TYPE - 1);
  __ Branch(&function, eq, a1, Operand(LAST_SPEC_OBJECT_TYPE));
  // Assume that there is no larger type.
  STATIC_ASSERT(LAST_NONCALLABLE_SPEC_OBJECT_TYPE == LAST_TYPE - 1);

  // Check if the constructor in the map is a JS function.
  __ ld(v0, FieldMemOperand(v0, Map::kConstructorOffset));
  __ GetObjectType(v0, a1, a1);
  __ Branch(&non_function_constructor, ne, a1, Operand(JS_FUNCTION_TYPE));

  // v0 now contains the constructor function. Grab the
  // instance class name from there.
  __ ld(v0, FieldMemOperand(v0, JSFunction::kSharedFunctionInfoOffset));
  __ ld(v0, FieldMemOperand(v0, SharedFunctionInfo::kInstanceClassNameOffset));
  __ Branch(&done);

  // Functions have class 'Function'.
  __ bind(&function);
  __ LoadRoot(v0, Heap::kfunction_class_stringRootIndex);
  __ jmp(&done);

  // Objects with a non-function constructor have class 'Object'.
  __ bind(&non_function_constructor);
  __ LoadRoot(v0, Heap::kObject_stringRootIndex);
  __ jmp(&done);

  // Non-JS objects have class null.
  __ bind(&null);
  __ LoadRoot(v0, Heap::kNullValueRootIndex);

  // All done.
  __ bind(&done);

  context()->Plug(v0);
}

void FullCodeGenerator::EmitLog(CallRuntime* expr) {
  // Conditionally generate a log call.
  // Args:
  //   0 (literal string): The type of logging (corresponds to the flags).
  //     This is used to determine whether or not to generate the log call.
  //   1 (string): Format string.  Access the string at argument index 2
  //     with '%2s' (see Logger::LogRuntime for all the formats).
  //   2 (array): Arguments to the format string.
  ZoneList<Expression*>* args = expr->arguments();
  ASSERT_EQ(args->length(), 3);
  if (CodeGenerator::ShouldGenerateLog(args->at(0))) {
    VisitForStackValue(args->at(1));
    VisitForStackValue(args->at(2));
    __ CallRuntime(Runtime::kLog, 2);
  }

  // Finally, we're expected to leave a value on the top of the stack.
  __ LoadRoot(v0, Heap::kUndefinedValueRootIndex);
  context()->Plug(v0);
}

void FullCodeGenerator::EmitRandomHeapNumber(CallRuntime* expr) {
  ASSERT(expr->arguments()->length() == 0);
  Label slow_allocate_heapnumber;
  Label heapnumber_allocated;

  // Save the new heap number in callee-saved register s0, since
  // we call out to external C code below.
  __ LoadRoot(t6, Heap::kHeapNumberMapRootIndex);
  __ AllocateHeapNumber(s0, a1, a2, t6, &slow_allocate_heapnumber);
  __ jmp(&heapnumber_allocated);

  __ bind(&slow_allocate_heapnumber);

  // Allocate a heap number.
  __ CallRuntime(Runtime::kNumberAlloc, 0);
  __ move(s0, v0);   // Save result in s0, so it is saved thru CFunc call.

  __ bind(&heapnumber_allocated);


  __ PrepareCallCFunction(2, a0);
  __ move(a0, s0);
  __ ld(a1, ContextOperand(cp, Context::GLOBAL_OBJECT_INDEX));
  __ ld(a1, FieldMemOperand(a1, GlobalObject::kNativeContextOffset));
  __ CallCFunction(
       ExternalReference::fill_heap_number_with_random_function(isolate()), 2);

  context()->Plug(v0);
}


void FullCodeGenerator::EmitSubString(CallRuntime* expr) {
  // Load the arguments on the stack and call the stub.
  SubStringStub stub;
  ZoneList<Expression*>* args = expr->arguments();
  ASSERT(args->length() == 3);
  VisitForStackValue(args->at(0));
  VisitForStackValue(args->at(1));
  VisitForStackValue(args->at(2));
  __ CallStub(&stub);
  context()->Plug(v0);
}

void FullCodeGenerator::EmitRegExpExec(CallRuntime* expr) {
  // Load the arguments on the stack and call the stub.
  RegExpExecStub stub;
  ZoneList<Expression*>* args = expr->arguments();
  ASSERT(args->length() == 4);
  VisitForStackValue(args->at(0));
  VisitForStackValue(args->at(1));
  VisitForStackValue(args->at(2));
  VisitForStackValue(args->at(3));
  __ CallStub(&stub);
  context()->Plug(v0);
}

void FullCodeGenerator::EmitValueOf(CallRuntime* expr) {
  ZoneList<Expression*>* args = expr->arguments();
  ASSERT(args->length() == 1);

  VisitForAccumulatorValue(args->at(0));  // Load the object.

  Label done;
  // If the object is a smi return the object.
  __ JumpIfSmi(v0, &done);
  // If the object is not a value type, return the object.
  __ GetObjectType(v0, a1, a1);
  __ Branch(&done, ne, a1, Operand(JS_VALUE_TYPE));

  __ ld(v0, FieldMemOperand(v0, JSValue::kValueOffset));

  __ bind(&done);
  context()->Plug(v0);
}

void FullCodeGenerator::EmitDateField(CallRuntime* expr) {
  ZoneList<Expression*>* args = expr->arguments();
  ASSERT(args->length() == 2);
  ASSERT_NE(NULL, args->at(1)->AsLiteral());
  Smi* index = Smi::cast(*(args->at(1)->AsLiteral()->handle()));

  VisitForAccumulatorValue(args->at(0));  // Load the object.

  Label runtime, done, not_date_object;
  Register object = v0;
  Register result = v0;
  Register scratch0 = t5;
  Register scratch1 = a1;

  __ JumpIfSmi(object, &not_date_object);
  __ GetObjectType(object, scratch1, scratch1);
  __ Branch(&not_date_object, ne, scratch1, Operand(JS_DATE_TYPE));

  if (index->value() == 0) {
    __ ld(result, FieldMemOperand(object, JSDate::kValueOffset));
    __ jmp(&done);
  } else {
    if (index->value() < JSDate::kFirstUncachedField) {
      ExternalReference stamp = ExternalReference::date_cache_stamp(isolate());
      __ li(scratch1, Operand(stamp));
      __ ld(scratch1, MemOperand(scratch1));
      __ ld(scratch0, FieldMemOperand(object, JSDate::kCacheStampOffset));
      __ Branch(&runtime, ne, scratch1, Operand(scratch0));
      __ ld(result, FieldMemOperand(object, JSDate::kValueOffset +
                                            kPointerSize * index->value()));
      __ jmp(&done);
    }
    __ bind(&runtime);
    __ PrepareCallCFunction(2, scratch1);
    __ li(a1, Operand(index));
    __ Move(a0, object);
    __ CallCFunction(ExternalReference::get_date_field_function(isolate()), 2);
    __ jmp(&done);
  }

  __ bind(&not_date_object);
  __ CallRuntime(Runtime::kThrowNotDateError, 0);
  __ bind(&done);
  context()->Plug(v0);
}

void FullCodeGenerator::EmitSeqStringSetCharCheck(Register string,
                                                  Register index,
                                                  Register value,
                                                  uint32_t encoding_mask) {
  __ And(at, index, Operand(kSmiTagMask));
  __ Check(eq, "Non-smi index", at, Operand(zero));
  __ And(at, value, Operand(kSmiTagMask));
  __ Check(eq, "Non-smi value", at, Operand(zero));

  __ ld(at, FieldMemOperand(string, String::kLengthOffset));
  __ Check(lt, "Index is too large", index, Operand(at));

  __ Check(ge, "Index is negative", index, Operand(zero));

  __ ld(at, FieldMemOperand(string, HeapObject::kMapOffset));
  __ ld1u(at2, FieldMemOperand(at, Map::kInstanceTypeOffset));

  __ And(at2, at2, Operand(kStringRepresentationMask | kStringEncodingMask));
  __ Subu(at, at2, Operand(encoding_mask));
  __ Check(eq, "Unexpected string type", at, Operand(zero));
}

void FullCodeGenerator::EmitOneByteSeqStringSetChar(CallRuntime* expr) {
  ZoneList<Expression*>* args = expr->arguments();
  ASSERT_EQ(3, args->length());

  Register string = v0;
  Register index = a1;
  Register value = a2;

  VisitForStackValue(args->at(1));  // index
  VisitForStackValue(args->at(2));  // value
  __ pop(value);
  __ pop(index);
  VisitForAccumulatorValue(args->at(0));  // string

  if (FLAG_debug_code) {
    static const uint32_t one_byte_seq_type = kSeqStringTag | kOneByteStringTag;
    EmitSeqStringSetCharCheck(string, index, value, one_byte_seq_type);
  }

  __ SmiUntag(value, value);
  __ Addu(at,
          string,
          Operand(SeqOneByteString::kHeaderSize - kHeapObjectTag));
  __ SmiUntag(index);
  __ Addu(at, at, index);
  __ st1(value, MemOperand(at));
  context()->Plug(string);
}

void FullCodeGenerator::EmitTwoByteSeqStringSetChar(CallRuntime* expr) {
  ZoneList<Expression*>* args = expr->arguments();
  ASSERT_EQ(3, args->length());

  Register string = v0;
  Register index = a1;
  Register value = a2;

  VisitForStackValue(args->at(1));  // index
  VisitForStackValue(args->at(2));  // value
  __ pop(value);
  __ pop(index);
  VisitForAccumulatorValue(args->at(0));  // string

  if (FLAG_debug_code) {
    static const uint32_t two_byte_seq_type = kSeqStringTag | kTwoByteStringTag;
    EmitSeqStringSetCharCheck(string, index, value, two_byte_seq_type);
  }

  __ SmiUntag(value, value);
  __ SmiUntag(index, index);
  __ Addu(at,
          string,
          Operand(SeqTwoByteString::kHeaderSize - kHeapObjectTag));
  __ sll(index, index, 1);
  __ Addu(at, at, index);
  STATIC_ASSERT(kSmiTagSize == 1 && kSmiTag == 0);
  __ st2(value, MemOperand(at));
    context()->Plug(string);
}

void FullCodeGenerator::EmitMathPow(CallRuntime* expr) {
  // Load the arguments on the stack and call the runtime function.
  ZoneList<Expression*>* args = expr->arguments();
  ASSERT(args->length() == 2);
  VisitForStackValue(args->at(0));
  VisitForStackValue(args->at(1));
  MathPowStub stub(MathPowStub::ON_STACK);
  __ CallStub(&stub);
  context()->Plug(v0);
}

void FullCodeGenerator::EmitSetValueOf(CallRuntime* expr) {
  ZoneList<Expression*>* args = expr->arguments();
  ASSERT(args->length() == 2);

  VisitForStackValue(args->at(0));  // Load the object.
  VisitForAccumulatorValue(args->at(1));  // Load the value.
  __ pop(a1);  // v0 = value. a1 = object.

  Label done;
  // If the object is a smi, return the value.
  __ JumpIfSmi(a1, &done);

  // If the object is not a value type, return the value.
  __ GetObjectType(a1, a2, a2);
  __ Branch(&done, ne, a2, Operand(JS_VALUE_TYPE));

  // Store the value.
  __ st(v0, FieldMemOperand(a1, JSValue::kValueOffset));
  // Update the write barrier.  Save the value as it will be
  // overwritten by the write barrier code and is needed afterward.
  __ move(a2, v0);
  __ RecordWriteField(
      a1, JSValue::kValueOffset, a2, a3, kRAHasBeenSaved, kDontSaveFPRegs);

  __ bind(&done);
  context()->Plug(v0);
}

void FullCodeGenerator::EmitNumberToString(CallRuntime* expr) {
  ZoneList<Expression*>* args = expr->arguments();
  ASSERT_EQ(args->length(), 1);

  // Load the argument on the stack and call the stub.
  VisitForStackValue(args->at(0));

  NumberToStringStub stub;
  __ CallStub(&stub);
  context()->Plug(v0);
}

void FullCodeGenerator::EmitStringCharFromCode(CallRuntime* expr) {
  ZoneList<Expression*>* args = expr->arguments();
  ASSERT(args->length() == 1);

  VisitForAccumulatorValue(args->at(0));

  Label done;
  StringCharFromCodeGenerator generator(v0, a1);
  generator.GenerateFast(masm_);
  __ jmp(&done);

  NopRuntimeCallHelper call_helper;
  generator.GenerateSlow(masm_, call_helper);

  __ bind(&done);
  context()->Plug(a1);
}

void FullCodeGenerator::EmitStringCharCodeAt(CallRuntime* expr) {
  ZoneList<Expression*>* args = expr->arguments();
  ASSERT(args->length() == 2);

  VisitForStackValue(args->at(0));
  VisitForAccumulatorValue(args->at(1));
  //__ move(a0, result_register());

  Register object = a1;
  Register index = a0;
  Register result = a2;

  __ pop(object);

  Label need_conversion;
  Label index_out_of_range;
  Label done;
  StringCharCodeAtGenerator generator(object,
                                      index,
                                      result,
                                      &need_conversion,
                                      &need_conversion,
                                      &index_out_of_range,
                                      STRING_INDEX_IS_NUMBER);
  generator.GenerateFast(masm_);
  __ jmp(&done);

  __ bind(&index_out_of_range);
  // When the index is out of range, the spec requires us to return
  // NaN.
  __ LoadRoot(result, Heap::kNanValueRootIndex);
  __ jmp(&done);

  __ bind(&need_conversion);
  // Load the undefined value into the result register, which will
  // trigger conversion.
  __ LoadRoot(result, Heap::kUndefinedValueRootIndex);
  __ jmp(&done);

  NopRuntimeCallHelper call_helper;
  generator.GenerateSlow(masm_, call_helper);

  __ bind(&done);
  context()->Plug(result);
}

void FullCodeGenerator::EmitStringCharAt(CallRuntime* expr) {
  ZoneList<Expression*>* args = expr->arguments();
  ASSERT(args->length() == 2);

  VisitForStackValue(args->at(0));
  VisitForAccumulatorValue(args->at(1));
  __ move(a0, result_register());

  Register object = a1;
  Register index = a0;
  Register scratch = a3;
  Register result = v0;

  __ pop(object);

  Label need_conversion;
  Label index_out_of_range;
  Label done;
  StringCharAtGenerator generator(object,
                                  index,
                                  scratch,
                                  result,
                                  &need_conversion,
                                  &need_conversion,
                                  &index_out_of_range,
                                  STRING_INDEX_IS_NUMBER);
  generator.GenerateFast(masm_);
  __ jmp(&done);

  __ bind(&index_out_of_range);
  // When the index is out of range, the spec requires us to return
  // the empty string.
  __ LoadRoot(result, Heap::kempty_stringRootIndex);
  __ jmp(&done);

  __ bind(&need_conversion);
  // Move smi zero into the result register, which will trigger
  // conversion.
  __ li(result, Operand(Smi::FromInt(0)));
  __ jmp(&done);

  NopRuntimeCallHelper call_helper;
  generator.GenerateSlow(masm_, call_helper);

  __ bind(&done);
  context()->Plug(result);
}

void FullCodeGenerator::EmitStringAdd(CallRuntime* expr) {
  ZoneList<Expression*>* args = expr->arguments();
  ASSERT_EQ(2, args->length());
  VisitForStackValue(args->at(0));
  VisitForStackValue(args->at(1));

  StringAddStub stub(NO_STRING_ADD_FLAGS);
  __ CallStub(&stub);
  context()->Plug(v0);
}

void FullCodeGenerator::EmitStringCompare(CallRuntime* expr) {
  ZoneList<Expression*>* args = expr->arguments();
  ASSERT_EQ(2, args->length());

  VisitForStackValue(args->at(0));
  VisitForStackValue(args->at(1));

  StringCompareStub stub;
  __ CallStub(&stub);
  context()->Plug(v0);
}

void FullCodeGenerator::EmitMathSin(CallRuntime* expr) {
  // Load the argument on the stack and call the stub.
  TranscendentalCacheStub stub(TranscendentalCache::SIN,
                               TranscendentalCacheStub::TAGGED);
  ZoneList<Expression*>* args = expr->arguments();
  ASSERT(args->length() == 1);
  VisitForStackValue(args->at(0));
  __ move(a0, result_register());  // Stub requires parameter in a0 and on tos.
  __ CallStub(&stub);
  context()->Plug(v0);
}

void FullCodeGenerator::EmitMathCos(CallRuntime* expr) {
  // Load the argument on the stack and call the stub.
  TranscendentalCacheStub stub(TranscendentalCache::COS,
                               TranscendentalCacheStub::TAGGED);
  ZoneList<Expression*>* args = expr->arguments();
  ASSERT(args->length() == 1);
  VisitForStackValue(args->at(0));
  __ move(a0, result_register());  // Stub requires parameter in a0 and on tos.
  __ CallStub(&stub);
  context()->Plug(v0);
}

void FullCodeGenerator::EmitMathTan(CallRuntime* expr) {
  // Load the argument on the stack and call the stub.
  TranscendentalCacheStub stub(TranscendentalCache::TAN,
                               TranscendentalCacheStub::TAGGED);
  ZoneList<Expression*>* args = expr->arguments();
  ASSERT(args->length() == 1);
  VisitForStackValue(args->at(0));
  __ move(a0, result_register());  // Stub requires parameter in a0 and on tos.
  __ CallStub(&stub);
  context()->Plug(v0);
}

void FullCodeGenerator::EmitMathLog(CallRuntime* expr) {
  // Load the argument on the stack and call the stub.
  TranscendentalCacheStub stub(TranscendentalCache::LOG,
                               TranscendentalCacheStub::TAGGED);
  ZoneList<Expression*>* args = expr->arguments();
  ASSERT(args->length() == 1);
  VisitForStackValue(args->at(0));
  __ move(a0, result_register());  // Stub requires parameter in a0 and on tos.
  __ CallStub(&stub);
  context()->Plug(v0);
}

void FullCodeGenerator::EmitMathSqrt(CallRuntime* expr) {
  // Load the argument on the stack and call the runtime function.
  ZoneList<Expression*>* args = expr->arguments();
  ASSERT(args->length() == 1);
  VisitForStackValue(args->at(0));
  __ CallRuntime(Runtime::kMath_sqrt, 1);
  context()->Plug(v0);
}

void FullCodeGenerator::EmitCallFunction(CallRuntime* expr) {
  ZoneList<Expression*>* args = expr->arguments();
  ASSERT(args->length() >= 2);

  int arg_count = args->length() - 2;  // 2 ~ receiver and function.
  for (int i = 0; i < arg_count + 1; i++) {
    VisitForStackValue(args->at(i));
  }
  VisitForAccumulatorValue(args->last());  // Function.

  Label runtime, done;
  // Check for non-function argument (including proxy).
  __ JumpIfSmi(v0, &runtime);
  __ GetObjectType(v0, a1, a1);
  __ Branch(&runtime, ne, a1, Operand(JS_FUNCTION_TYPE));

  // InvokeFunction requires the function in a1. Move it in there.
  __ move(a1, result_register());
  ParameterCount count(arg_count);
  __ InvokeFunction(a1, count, CALL_FUNCTION,
                    NullCallWrapper(), CALL_AS_METHOD);
  __ ld(cp, MemOperand(fp, StandardFrameConstants::kContextOffset));
  __ jmp(&done);

  __ bind(&runtime);
  __ push(v0);
  __ CallRuntime(Runtime::kCall, args->length());
  __ bind(&done);

  context()->Plug(v0);
}

void FullCodeGenerator::EmitRegExpConstructResult(CallRuntime* expr) {
  RegExpConstructResultStub stub;
  ZoneList<Expression*>* args = expr->arguments();
  ASSERT(args->length() == 3);
  VisitForStackValue(args->at(0));
  VisitForStackValue(args->at(1));
  VisitForStackValue(args->at(2));
  __ CallStub(&stub);
  context()->Plug(v0);
}

void FullCodeGenerator::EmitGetFromCache(CallRuntime* expr) {
  ZoneList<Expression*>* args = expr->arguments();
  ASSERT_EQ(2, args->length());

  ASSERT_NE(NULL, args->at(0)->AsLiteral());
  int cache_id = Smi::cast(*(args->at(0)->AsLiteral()->handle()))->value();

  Handle<FixedArray> jsfunction_result_caches(
      isolate()->native_context()->jsfunction_result_caches());
  if (jsfunction_result_caches->length() <= cache_id) {
    __ Abort("Attempt to use undefined cache.");
    __ LoadRoot(v0, Heap::kUndefinedValueRootIndex);
    context()->Plug(v0);
    return;
  }

  VisitForAccumulatorValue(args->at(1));

  Register key = v0;
  Register cache = a1;
  __ ld(cache, ContextOperand(cp, Context::GLOBAL_OBJECT_INDEX));
  __ ld(cache, FieldMemOperand(cache, GlobalObject::kNativeContextOffset));
  __ ld(cache,
         ContextOperand(
             cache, Context::JSFUNCTION_RESULT_CACHES_INDEX));
  __ ld(cache,
         FieldMemOperand(cache, FixedArray::OffsetOfElementAt(cache_id)));


  Label done, not_found;
  STATIC_ASSERT(kSmiTag == 0 && kSmiTagSize == 1);
  __ ld(a2, FieldMemOperand(cache, JSFunctionResultCache::kFingerOffset));
  // a2 now holds finger offset as a smi.
  __ Addu(a3, cache, Operand(FixedArray::kHeaderSize - kHeapObjectTag));
  // a3 now points to the start of fixed array elements.
  __ sra(at, a2, kSmiTagSize + kSmiShiftSize);
  __ sll(at, at, kPointerSizeLog2);
  __ add(a3, a3, at);
  // a3 now points to key of indexed element of cache.
  __ ld(a2, MemOperand(a3));
  __ Branch(&not_found, ne, key, Operand(a2));

  __ ld(v0, MemOperand(a3, kPointerSize));
  __ Branch(&done);

  __ bind(&not_found);
  // Call runtime to perform the lookup.
  __ Push(cache, key);
  __ CallRuntime(Runtime::kGetFromCache, 2);

  __ bind(&done);
  context()->Plug(v0);
}

void FullCodeGenerator::EmitIsRegExpEquivalent(CallRuntime* expr) {
  ZoneList<Expression*>* args = expr->arguments();
  ASSERT_EQ(2, args->length());

  Register right = v0;
  Register left = a1;
  Register tmp = a2;
  Register tmp2 = a3;

  VisitForStackValue(args->at(0));
  VisitForAccumulatorValue(args->at(1));  // Result (right) in v0.
  __ pop(left);

  Label done, fail, ok;
  __ Branch(&ok, eq, left, Operand(right));
  // Fail if either is a non-HeapObject.
  __ And(tmp, left, Operand(right));
  __ JumpIfSmi(tmp, &fail);
  __ ld(tmp, FieldMemOperand(left, HeapObject::kMapOffset));
  __ ld1u(tmp2, FieldMemOperand(tmp, Map::kInstanceTypeOffset));
  __ Branch(&fail, ne, tmp2, Operand(JS_REGEXP_TYPE));
  __ ld(tmp2, FieldMemOperand(right, HeapObject::kMapOffset));
  __ Branch(&fail, ne, tmp, Operand(tmp2));
  __ ld(tmp, FieldMemOperand(left, JSRegExp::kDataOffset));
  __ ld(tmp2, FieldMemOperand(right, JSRegExp::kDataOffset));
  __ Branch(&ok, eq, tmp, Operand(tmp2));
  __ bind(&fail);
  __ LoadRoot(v0, Heap::kFalseValueRootIndex);
  __ jmp(&done);
  __ bind(&ok);
  __ LoadRoot(v0, Heap::kTrueValueRootIndex);
  __ bind(&done);

  context()->Plug(v0);
}

void FullCodeGenerator::EmitHasCachedArrayIndex(CallRuntime* expr) {
  ZoneList<Expression*>* args = expr->arguments();
  VisitForAccumulatorValue(args->at(0));

  Label materialize_true, materialize_false;
  Label* if_true = NULL;
  Label* if_false = NULL;
  Label* fall_through = NULL;
  context()->PrepareTest(&materialize_true, &materialize_false,
                         &if_true, &if_false, &fall_through);

  __ ld(a0, FieldMemOperand(v0, String::kHashFieldOffset));
  __ And(a0, a0, Operand(String::kContainsCachedArrayIndexMask));

  PrepareForBailoutBeforeSplit(expr, true, if_true, if_false);
  Split(eq, a0, Operand(zero), if_true, if_false, fall_through);

  context()->Plug(if_true, if_false);
}

void FullCodeGenerator::EmitGetCachedArrayIndex(CallRuntime* expr) {
  ZoneList<Expression*>* args = expr->arguments();
  ASSERT(args->length() == 1);
  VisitForAccumulatorValue(args->at(0));

  __ AssertString(v0);

  __ ld(v0, FieldMemOperand(v0, String::kHashFieldOffset));
  __ IndexFromHash(v0, v0);

  context()->Plug(v0);
}

void FullCodeGenerator::EmitFastAsciiArrayJoin(CallRuntime* expr) {
  Label bailout, done, one_char_separator, long_separator,
      non_trivial_array, not_size_one_array, loop,
      empty_separator_loop, one_char_separator_loop,
      one_char_separator_loop_entry, long_separator_loop;
  ZoneList<Expression*>* args = expr->arguments();
  ASSERT(args->length() == 2);
  VisitForStackValue(args->at(1));
  VisitForAccumulatorValue(args->at(0));

  // All aliases of the same register have disjoint lifetimes.
  Register array = v0;
  Register elements = no_reg;  // Will be v0.
  Register result = no_reg;  // Will be v0.
  Register separator = a1;
  Register array_length = a2;
  Register result_pos = no_reg;  // Will be a2.
  Register string_length = a3;
  Register string = t0;
  Register element = t1;
  Register elements_end = t2;
  Register scratch1 = t3;
  Register scratch2 = t5;
  Register scratch3 = t4;

  // Separator operand is on the stack.
  __ pop(separator);

  // Check that the array is a JSArray.
  __ JumpIfSmi(array, &bailout);
  __ GetObjectType(array, scratch1, scratch2);
  __ Branch(&bailout, ne, scratch2, Operand(JS_ARRAY_TYPE));

  // Check that the array has fast elements.
  __ CheckFastElements(scratch1, scratch2, &bailout);

  // If the array has length zero, return the empty string.
  __ ld(array_length, FieldMemOperand(array, JSArray::kLengthOffset));
  __ SmiUntag(array_length);
  __ Branch(&non_trivial_array, ne, array_length, Operand(zero));
  __ LoadRoot(v0, Heap::kempty_stringRootIndex);
  __ Branch(&done);

  __ bind(&non_trivial_array);

  // Get the FixedArray containing array's elements.
  elements = array;
  __ ld(elements, FieldMemOperand(array, JSArray::kElementsOffset));
  array = no_reg;  // End of array's live range.

  // Check that all array elements are sequential ASCII strings, and
  // accumulate the sum of their lengths, as a smi-encoded value.
  __ move(string_length, zero);
  __ Addu(element,
          elements, Operand(FixedArray::kHeaderSize - kHeapObjectTag));
  __ sll(elements_end, array_length, kPointerSizeLog2);
  __ Addu(elements_end, element, elements_end);
  // Loop condition: while (element < elements_end).
  // Live values in registers:
  //   elements: Fixed array of strings.
  //   array_length: Length of the fixed array of strings (not smi)
  //   separator: Separator string
  //   string_length: Accumulated sum of string lengths (smi).
  //   element: Current array element.
  //   elements_end: Array end.
  if (generate_debug_code_) {
    __ Assert(gt, "No empty arrays here in EmitFastAsciiArrayJoin",
        array_length, Operand(zero));
  }
  __ bind(&loop);
  __ ld(string, MemOperand(element));
  __ Addu(element, element, kPointerSize);
  __ JumpIfSmi(string, &bailout);
  __ ld(scratch1, FieldMemOperand(string, HeapObject::kMapOffset));
  __ ld1u(scratch1, FieldMemOperand(scratch1, Map::kInstanceTypeOffset));
  __ JumpIfInstanceTypeIsNotSequentialAscii(scratch1, scratch2, &bailout);
  __ ld(scratch1, FieldMemOperand(string, SeqOneByteString::kLengthOffset));
  __ AdduAndCheckForOverflow(string_length, string_length, scratch1, scratch3);
  __ BranchOnOverflow(&bailout, scratch3);
  __ Branch(&loop, lt, element, Operand(elements_end));

  // If array_length is 1, return elements[0], a string.
  __ Branch(&not_size_one_array, ne, array_length, Operand(1));
  __ ld(v0, FieldMemOperand(elements, FixedArray::kHeaderSize));
  __ Branch(&done);

  __ bind(&not_size_one_array);

  // Live values in registers:
  //   separator: Separator string
  //   array_length: Length of the array.
  //   string_length: Sum of string lengths (smi).
  //   elements: FixedArray of strings.

  // Check that the separator is a flat ASCII string.
  __ JumpIfSmi(separator, &bailout);
  __ ld(scratch1, FieldMemOperand(separator, HeapObject::kMapOffset));
  __ ld1u(scratch1, FieldMemOperand(scratch1, Map::kInstanceTypeOffset));
  __ JumpIfInstanceTypeIsNotSequentialAscii(scratch1, scratch2, &bailout);

  // Add (separator length times array_length) - separator length to the
  // string_length to get the length of the result string. array_length is not
  // smi but the other values are, so the result is a smi.
  __ ld(scratch1, FieldMemOperand(separator, SeqOneByteString::kLengthOffset));
  __ Subu(string_length, string_length, Operand(scratch1));
#if 0
  __ Mult(array_length, scratch1);
  // Check for smi overflow. No overflow if higher 33 bits of 64-bit result are
  // zero.
  __ mfhi(scratch2);
  __ Branch(&bailout, ne, scratch2, Operand(zero));
  __ mflo(scratch2);
  __ And(scratch3, scratch2, Operand(0x80000000));
  __ Branch(&bailout, ne, scratch3, Operand(zero));
#else
  __ mul_hs_ls(scratch2, scratch1, array_length);
  __ sra(scratch3, scratch2, 30);
  __ Branch(&bailout, ne, scratch3, Operand(zero));
  __ sll(scratch2, scratch2, 32);
#endif
  __ AdduAndCheckForOverflow(string_length, string_length, scratch2, scratch3);
  __ BranchOnOverflow(&bailout, scratch3);
  __ SmiUntag(string_length);

  // Get first element in the array to free up the elements register to be used
  // for the result.
  __ Addu(element,
          elements, Operand(FixedArray::kHeaderSize - kHeapObjectTag));
  result = elements;  // End of live range for elements.
  elements = no_reg;
  // Live values in registers:
  //   element: First array element
  //   separator: Separator string
  //   string_length: Length of result string (not smi)
  //   array_length: Length of the array.
  __ AllocateAsciiString(result,
                         string_length,
                         scratch1,
                         scratch2,
                         elements_end,
                         &bailout);
  // Prepare for looping. Set up elements_end to end of the array. Set
  // result_pos to the position of the result where to write the first
  // character.
  __ sll(elements_end, array_length, kPointerSizeLog2);
  __ Addu(elements_end, element, elements_end);
  result_pos = array_length;  // End of live range for array_length.
  array_length = no_reg;
  __ Addu(result_pos,
          result,
          Operand(SeqOneByteString::kHeaderSize - kHeapObjectTag));

  // Check the length of the separator.
  __ ld(scratch1, FieldMemOperand(separator, SeqOneByteString::kLengthOffset));
  __ li(at, Operand(Smi::FromInt(1)));
  __ Branch(&one_char_separator, eq, scratch1, Operand(at));
  __ Branch(&long_separator, gt, scratch1, Operand(at));

  // Empty separator case.
  __ bind(&empty_separator_loop);
  // Live values in registers:
  //   result_pos: the position to which we are currently copying characters.
  //   element: Current array element.
  //   elements_end: Array end.

  // Copy next array element to the result.
  __ ld(string, MemOperand(element));
  __ Addu(element, element, kPointerSize);
  __ ld(string_length, FieldMemOperand(string, String::kLengthOffset));
  __ SmiUntag(string_length);
  __ Addu(string, string, SeqOneByteString::kHeaderSize - kHeapObjectTag);
  __ CopyBytes(string, result_pos, string_length, scratch1);
  // End while (element < elements_end).
  __ Branch(&empty_separator_loop, lt, element, Operand(elements_end));
  ASSERT(result.is(v0));
  __ Branch(&done);

  // One-character separator case.
  __ bind(&one_char_separator);
  // Replace separator with its ASCII character value.
  __ ld1u(separator, FieldMemOperand(separator, SeqOneByteString::kHeaderSize));
  // Jump into the loop after the code that copies the separator, so the first
  // element is not preceded by a separator.
  __ jmp(&one_char_separator_loop_entry);

  __ bind(&one_char_separator_loop);
  // Live values in registers:
  //   result_pos: the position to which we are currently copying characters.
  //   element: Current array element.
  //   elements_end: Array end.
  //   separator: Single separator ASCII char (in lower byte).

  // Copy the separator character to the result.
  __ st1(separator, MemOperand(result_pos));
  __ Addu(result_pos, result_pos, 1);

  // Copy next array element to the result.
  __ bind(&one_char_separator_loop_entry);
  __ ld(string, MemOperand(element));
  __ Addu(element, element, kPointerSize);
  __ ld(string_length, FieldMemOperand(string, String::kLengthOffset));
  __ SmiUntag(string_length);
  __ Addu(string, string, SeqOneByteString::kHeaderSize - kHeapObjectTag);
  __ CopyBytes(string, result_pos, string_length, scratch1);
  // End while (element < elements_end).
  __ Branch(&one_char_separator_loop, lt, element, Operand(elements_end));
  ASSERT(result.is(v0));
  __ Branch(&done);

  // Long separator case (separator is more than one character). Entry is at the
  // label long_separator below.
  __ bind(&long_separator_loop);
  // Live values in registers:
  //   result_pos: the position to which we are currently copying characters.
  //   element: Current array element.
  //   elements_end: Array end.
  //   separator: Separator string.

  // Copy the separator to the result.
  __ ld(string_length, FieldMemOperand(separator, String::kLengthOffset));
  __ SmiUntag(string_length);
  __ Addu(string,
          separator,
          Operand(SeqOneByteString::kHeaderSize - kHeapObjectTag));
  __ CopyBytes(string, result_pos, string_length, scratch1);

  __ bind(&long_separator);
  __ ld(string, MemOperand(element));
  __ Addu(element, element, kPointerSize);
  __ ld(string_length, FieldMemOperand(string, String::kLengthOffset));
  __ SmiUntag(string_length);
  __ Addu(string, string, SeqOneByteString::kHeaderSize - kHeapObjectTag);
  __ CopyBytes(string, result_pos, string_length, scratch1);
  // End while (element < elements_end).
  __ Branch(&long_separator_loop, lt, element, Operand(elements_end));
  ASSERT(result.is(v0));
  __ Branch(&done);

  __ bind(&bailout);
  __ LoadRoot(v0, Heap::kUndefinedValueRootIndex);
  __ bind(&done);
  context()->Plug(v0);
}

void FullCodeGenerator::VisitCallRuntime(CallRuntime* expr) {
  Handle<String> name = expr->name();
  if (name->length() > 0 && name->Get(0) == '_') {
    Comment cmnt(masm_, "[ InlineRuntimeCall");
    EmitInlineRuntimeCall(expr);
    return;
  }

  Comment cmnt(masm_, "[ CallRuntime");
  ZoneList<Expression*>* args = expr->arguments();

  if (expr->is_jsruntime()) {
    // Prepare for calling JS runtime function.
    __ ld(a0, GlobalObjectOperand());
    __ ld(a0, FieldMemOperand(a0, GlobalObject::kBuiltinsOffset));
    __ push(a0);
  }

  // Push the arguments ("left-to-right").
  int arg_count = args->length();
  for (int i = 0; i < arg_count; i++) {
    VisitForStackValue(args->at(i));
  }

  if (expr->is_jsruntime()) {
    // Call the JS runtime function.
    __ li(a2, Operand(expr->name()));
    RelocInfo::Mode mode = RelocInfo::CODE_TARGET;
    Handle<Code> ic =
        isolate()->stub_cache()->ComputeCallInitialize(arg_count, mode);
    CallIC(ic, mode, expr->CallRuntimeFeedbackId());
    // Restore context register.
    __ ld(cp, MemOperand(fp, StandardFrameConstants::kContextOffset));
  } else {
    // Call the C runtime function.
    __ CallRuntime(expr->function(), arg_count);
  }
  context()->Plug(v0);
}


void FullCodeGenerator::VisitUnaryOperation(UnaryOperation* expr) {
  switch (expr->op()) {
    case Token::DELETE: {
      Comment cmnt(masm_, "[ UnaryOperation (DELETE)");
      Property* property = expr->expression()->AsProperty();
      VariableProxy* proxy = expr->expression()->AsVariableProxy();

      if (property != NULL) {
        VisitForStackValue(property->obj());
        VisitForStackValue(property->key());
        StrictModeFlag strict_mode_flag = (language_mode() == CLASSIC_MODE)
            ? kNonStrictMode : kStrictMode;
        __ li(a1, Operand(Smi::FromInt(strict_mode_flag)));
        __ push(a1);
        __ InvokeBuiltin(Builtins::DELETE, CALL_FUNCTION);
        context()->Plug(v0);
      } else if (proxy != NULL) {
        Variable* var = proxy->var();
        // Delete of an unqualified identifier is disallowed in strict mode
        // but "delete this" is allowed.
        ASSERT(language_mode() == CLASSIC_MODE || var->is_this());
        if (var->IsUnallocated()) {
          __ ld(a2, GlobalObjectOperand());
          __ li(a1, Operand(var->name()));
          __ li(a0, Operand(Smi::FromInt(kNonStrictMode)));
          __ Push(a2, a1, a0);
          __ InvokeBuiltin(Builtins::DELETE, CALL_FUNCTION);
          context()->Plug(v0);
        } else if (var->IsStackAllocated() || var->IsContextSlot()) {
          // Result of deleting non-global, non-dynamic variables is false.
          // The subexpression does not have side effects.
          context()->Plug(var->is_this());
        } else {
          // Non-global variable.  Call the runtime to try to delete from the
          // context where the variable was introduced.
          __ push(context_register());
          __ li(a2, Operand(var->name()));
          __ push(a2);
          __ CallRuntime(Runtime::kDeleteContextSlot, 2);
          context()->Plug(v0);
        }
      } else {
        // Result of deleting non-property, non-variable reference is true.
        // The subexpression may have side effects.
        VisitForEffect(expr->expression());
        context()->Plug(true);
      }
      break;
    }

    case Token::VOID: {
      Comment cmnt(masm_, "[ UnaryOperation (VOID)");
      VisitForEffect(expr->expression());
      context()->Plug(Heap::kUndefinedValueRootIndex);
      break;
    }

    case Token::NOT: {
      Comment cmnt(masm_, "[ UnaryOperation (NOT)");
      if (context()->IsEffect()) {
        // Unary NOT has no side effects so it's only necessary to visit the
        // subexpression.  Match the optimizing compiler by not branching.
        VisitForEffect(expr->expression());
      } else if (context()->IsTest()) {
        const TestContext* test = TestContext::cast(context());
        // The labels are swapped for the recursive call.
        VisitForControl(expr->expression(),
                        test->false_label(),
                        test->true_label(),
                        test->fall_through());
        context()->Plug(test->true_label(), test->false_label());
      } else {
        // We handle value contexts explicitly rather than simply visiting
        // for control and plugging the control flow into the context,
        // because we need to prepare a pair of extra administrative AST ids
        // for the optimizing compiler.
        ASSERT(context()->IsAccumulatorValue() || context()->IsStackValue());
        Label materialize_true, materialize_false, done;
        VisitForControl(expr->expression(),
                        &materialize_false,
                        &materialize_true,
                        &materialize_true);
        __ bind(&materialize_true);
        PrepareForBailoutForId(expr->MaterializeTrueId(), NO_REGISTERS);
        __ LoadRoot(v0, Heap::kTrueValueRootIndex);
        if (context()->IsStackValue()) __ push(v0);
        __ jmp(&done);
        __ bind(&materialize_false);
        PrepareForBailoutForId(expr->MaterializeFalseId(), NO_REGISTERS);
        __ LoadRoot(v0, Heap::kFalseValueRootIndex);
        if (context()->IsStackValue()) __ push(v0);
        __ bind(&done);
      }
      break;
    }

    case Token::TYPEOF: {
      Comment cmnt(masm_, "[ UnaryOperation (TYPEOF)");
      { StackValueContext context(this);
        VisitForTypeofValue(expr->expression());
      }
      __ CallRuntime(Runtime::kTypeof, 1);
      context()->Plug(v0);
      break;
    }

    case Token::SUB:
      EmitUnaryOperation(expr, "[ UnaryOperation (SUB)");
      break;

    case Token::BIT_NOT:
      EmitUnaryOperation(expr, "[ UnaryOperation (BIT_NOT)");
      break;

    default:
      UNREACHABLE();
  }
}


void FullCodeGenerator::EmitUnaryOperation(UnaryOperation* expr,
                                           const char* comment) {
  // TODO(svenpanne): Allowing format strings in Comment would be nice here...
  Comment cmt(masm_, comment);
  bool can_overwrite = expr->expression()->ResultOverwriteAllowed();
  UnaryOverwriteMode overwrite =
      can_overwrite ? UNARY_OVERWRITE : UNARY_NO_OVERWRITE;
  UnaryOpStub stub(expr->op(), overwrite);
  // GenericUnaryOpStub expects the argument to be in a0.
  VisitForAccumulatorValue(expr->expression());
  SetSourcePosition(expr->position());
  __ move(a0, result_register());
  CallIC(stub.GetCode(isolate()), RelocInfo::CODE_TARGET,
         expr->UnaryOperationFeedbackId());
  context()->Plug(v0);
}

void FullCodeGenerator::VisitCountOperation(CountOperation* expr) {
  Comment cmnt(masm_, "[ CountOperation");
  SetSourcePosition(expr->position());

  // Invalid left-hand sides are rewritten to have a 'throw ReferenceError'
  // as the left-hand side.
  if (!expr->expression()->IsValidLeftHandSide()) {
    VisitForEffect(expr->expression());
    return;
  }

  // Expression can only be a property, a global or a (parameter or local)
  // slot.
  enum LhsKind { VARIABLE, NAMED_PROPERTY, KEYED_PROPERTY };
  LhsKind assign_type = VARIABLE;
  Property* prop = expr->expression()->AsProperty();
  // In case of a property we use the uninitialized expression context
  // of the key to detect a named property.
  if (prop != NULL) {
    assign_type =
        (prop->key()->IsPropertyName()) ? NAMED_PROPERTY : KEYED_PROPERTY;
  }

  // Evaluate expression and get value.
  if (assign_type == VARIABLE) {
    ASSERT(expr->expression()->AsVariableProxy()->var() != NULL);
    AccumulatorValueContext context(this);
    EmitVariableLoad(expr->expression()->AsVariableProxy());
  } else {
    // Reserve space for result of postfix operation.
    if (expr->is_postfix() && !context()->IsEffect()) {
      __ li(at, Operand(Smi::FromInt(0)));
      __ push(at);
    }
    if (assign_type == NAMED_PROPERTY) {
      // Put the object both on the stack and in the accumulator.
      VisitForAccumulatorValue(prop->obj());
      __ push(v0);
      EmitNamedPropertyLoad(prop);
    } else {
      VisitForStackValue(prop->obj());
      VisitForAccumulatorValue(prop->key());
      __ ld(a1, MemOperand(sp, 0));
      __ push(v0);
      EmitKeyedPropertyLoad(prop);
    }
  }

  // We need a second deoptimization point after loading the value
  // in case evaluating the property load my have a side effect.
  if (assign_type == VARIABLE) {
    PrepareForBailout(expr->expression(), TOS_REG);
  } else {
    PrepareForBailoutForId(prop->LoadId(), TOS_REG);
  }

  // Call ToNumber only if operand is not a smi.
  Label no_conversion;
  __ JumpIfSmi(v0, &no_conversion);
  __ move(a0, v0);
  ToNumberStub convert_stub;
  __ CallStub(&convert_stub);
  __ bind(&no_conversion);

  // Save result for postfix expressions.
  if (expr->is_postfix()) {
    if (!context()->IsEffect()) {
      // Save the result on the stack. If we have a named or keyed property
      // we store the result under the receiver that is currently on top
      // of the stack.
      switch (assign_type) {
        case VARIABLE:
          __ push(v0);
          break;
        case NAMED_PROPERTY:
          __ st(v0, MemOperand(sp, kPointerSize));
          break;
        case KEYED_PROPERTY:
          __ st(v0, MemOperand(sp, 2 * kPointerSize));
          break;
      }
    }
  }
  __ move(a0, result_register());

  // Inline smi case if we are in a loop.
  Label stub_call, done;
  JumpPatchSite patch_site(masm_);

  int count_value = expr->op() == Token::INC ? 1 : -1;

  // the following if-true case may clobber a0, while
  // a0 is needed later.
  //
  // MIPS don't have this problem because MIPS's return
  // value reg and arg reg don't overlap.
  __ move(at2, a0);

  if (ShouldInlineSmiCase(expr->op())) {
    __ li(a1, Operand(Smi::FromInt(count_value)));
    __ AdduAndCheckForOverflow(v0, a0, a1, t0);
    __ BranchOnOverflow(&stub_call, t0);  // Do stub on overflow.

    // We could eliminate this smi check if we split the code at
    // the first smi check before calling ToNumber.
    patch_site.EmitJumpIfSmi(v0, &done);
    __ bind(&stub_call);
  }

  __ move(a0, at2);

  __ move(a1, a0);
  __ li(a0, Operand(Smi::FromInt(count_value)));

  // Record position before stub call.
  SetSourcePosition(expr->position());

  BinaryOpStub stub(Token::ADD, NO_OVERWRITE);
  CallIC(stub.GetCode(isolate()),
         RelocInfo::CODE_TARGET,
         expr->CountBinOpFeedbackId());
  patch_site.EmitPatchInfo();
  __ bind(&done);

  // Store the value returned in v0.
  switch (assign_type) {
    case VARIABLE:
      if (expr->is_postfix()) {
        { EffectContext context(this);
          EmitVariableAssignment(expr->expression()->AsVariableProxy()->var(),
                                 Token::ASSIGN);
          PrepareForBailoutForId(expr->AssignmentId(), TOS_REG);
          context.Plug(v0);
        }
        // For all contexts except EffectConstant we have the result on
        // top of the stack.
        if (!context()->IsEffect()) {
          context()->PlugTOS();
        }
      } else {
        EmitVariableAssignment(expr->expression()->AsVariableProxy()->var(),
                               Token::ASSIGN);
        PrepareForBailoutForId(expr->AssignmentId(), TOS_REG);
        context()->Plug(v0);
      }
      break;
    case NAMED_PROPERTY: {
      __ move(a0, result_register());  // Value.
      __ li(a2, Operand(prop->key()->AsLiteral()->handle()));  // Name.
      __ pop(a1);  // Receiver.
      Handle<Code> ic = is_classic_mode()
          ? isolate()->builtins()->StoreIC_Initialize()
          : isolate()->builtins()->StoreIC_Initialize_Strict();
      CallIC(ic, RelocInfo::CODE_TARGET, expr->CountStoreFeedbackId());
      PrepareForBailoutForId(expr->AssignmentId(), TOS_REG);
      if (expr->is_postfix()) {
        if (!context()->IsEffect()) {
          context()->PlugTOS();
        }
      } else {
        context()->Plug(v0);
      }
      break;
    }
    case KEYED_PROPERTY: {
      __ move(a0, result_register());  // Value.
      __ pop(a1);  // Key.
      __ pop(a2);  // Receiver.
      Handle<Code> ic = is_classic_mode()
          ? isolate()->builtins()->KeyedStoreIC_Initialize()
          : isolate()->builtins()->KeyedStoreIC_Initialize_Strict();
      CallIC(ic, RelocInfo::CODE_TARGET, expr->CountStoreFeedbackId());
      PrepareForBailoutForId(expr->AssignmentId(), TOS_REG);
      if (expr->is_postfix()) {
        if (!context()->IsEffect()) {
          context()->PlugTOS();
        }
      } else {
        context()->Plug(v0);
      }
      break;
    }
  }
}

void FullCodeGenerator::VisitForTypeofValue(Expression* expr) {
  ASSERT(!context()->IsEffect());
  ASSERT(!context()->IsTest());
  VariableProxy* proxy = expr->AsVariableProxy();
  if (proxy != NULL && proxy->var()->IsUnallocated()) {
    Comment cmnt(masm_, "Global variable");
    __ ld(a0, GlobalObjectOperand());
    __ li(a2, Operand(proxy->name()));
    Handle<Code> ic = isolate()->builtins()->LoadIC_Initialize();
    // Use a regular load, not a contextual load, to avoid a reference
    // error.
    CallIC(ic);
    PrepareForBailout(expr, TOS_REG);
    context()->Plug(v0);
  } else if (proxy != NULL && proxy->var()->IsLookupSlot()) {
    Label done, slow;

    // Generate code for loading from variables potentially shadowed
    // by eval-introduced variables.
    EmitDynamicLookupFastCase(proxy->var(), INSIDE_TYPEOF, &slow, &done);

    __ bind(&slow);
    __ li(a0, Operand(proxy->name()));
    __ Push(cp, a0);
    __ CallRuntime(Runtime::kLoadContextSlotNoReferenceError, 2);
    PrepareForBailout(expr, TOS_REG);
    __ bind(&done);

    context()->Plug(v0);
  } else {
    // This expression cannot throw a reference error at the top level.
    VisitInDuplicateContext(expr);
  }
}

void FullCodeGenerator::EmitLiteralCompareTypeof(Expression* expr,
                                                 Expression* sub_expr,
                                                 Handle<String> check) {
  Label materialize_true, materialize_false;
  Label* if_true = NULL;
  Label* if_false = NULL;
  Label* fall_through = NULL;
  context()->PrepareTest(&materialize_true, &materialize_false,
                         &if_true, &if_false, &fall_through);

  { AccumulatorValueContext context(this);
    VisitForTypeofValue(sub_expr);
  }
  PrepareForBailoutBeforeSplit(expr, true, if_true, if_false);

  if (check->Equals(isolate()->heap()->number_string())) {
    __ JumpIfSmi(v0, if_true);
    __ ld(v0, FieldMemOperand(v0, HeapObject::kMapOffset));
    __ LoadRoot(at, Heap::kHeapNumberMapRootIndex);
    Split(eq, v0, Operand(at), if_true, if_false, fall_through);
  } else if (check->Equals(isolate()->heap()->string_string())) {
    __ JumpIfSmi(v0, if_false);
    // Check for undetectable objects => false.
    __ GetObjectType(v0, v0, a1);
    __ Branch(if_false, ge, a1, Operand(FIRST_NONSTRING_TYPE));
    __ ld1u(a1, FieldMemOperand(v0, Map::kBitFieldOffset));
    __ And(a1, a1, Operand(1 << Map::kIsUndetectable));
    Split(eq, a1, Operand(zero),
          if_true, if_false, fall_through);
  } else if (check->Equals(isolate()->heap()->symbol_string())) {
    __ JumpIfSmi(v0, if_false);
    __ GetObjectType(v0, v0, a1);
    Split(eq, a1, Operand(SYMBOL_TYPE), if_true, if_false, fall_through);
  } else if (check->Equals(isolate()->heap()->boolean_string())) {
    __ LoadRoot(at, Heap::kTrueValueRootIndex);
    __ Branch(if_true, eq, v0, Operand(at));
    __ LoadRoot(at, Heap::kFalseValueRootIndex);
    Split(eq, v0, Operand(at), if_true, if_false, fall_through);
  } else if (FLAG_harmony_typeof &&
             check->Equals(isolate()->heap()->null_string())) {
    __ LoadRoot(at, Heap::kNullValueRootIndex);
    Split(eq, v0, Operand(at), if_true, if_false, fall_through);
  } else if (check->Equals(isolate()->heap()->undefined_string())) {
    __ LoadRoot(at, Heap::kUndefinedValueRootIndex);
    __ Branch(if_true, eq, v0, Operand(at));
    __ JumpIfSmi(v0, if_false);
    // Check for undetectable objects => true.
    __ ld(v0, FieldMemOperand(v0, HeapObject::kMapOffset));
    __ ld1u(a1, FieldMemOperand(v0, Map::kBitFieldOffset));
    __ And(a1, a1, Operand(1 << Map::kIsUndetectable));
    Split(ne, a1, Operand(zero), if_true, if_false, fall_through);
  } else if (check->Equals(isolate()->heap()->function_string())) {
    __ JumpIfSmi(v0, if_false);
    STATIC_ASSERT(NUM_OF_CALLABLE_SPEC_OBJECT_TYPES == 2);
    __ GetObjectType(v0, v0, a1);
    __ Branch(if_true, eq, a1, Operand(JS_FUNCTION_TYPE));
    Split(eq, a1, Operand(JS_FUNCTION_PROXY_TYPE),
          if_true, if_false, fall_through);
  } else if (check->Equals(isolate()->heap()->object_string())) {
    __ JumpIfSmi(v0, if_false);
    if (!FLAG_harmony_typeof) {
      __ LoadRoot(at, Heap::kNullValueRootIndex);
      __ Branch(if_true, eq, v0, Operand(at));
    }
    // Check for JS objects => true.
    __ GetObjectType(v0, v0, a1);
    __ Branch(if_false, lt, a1, Operand(FIRST_NONCALLABLE_SPEC_OBJECT_TYPE));
    __ ld1u(a1, FieldMemOperand(v0, Map::kInstanceTypeOffset));
    __ Branch(if_false, gt, a1, Operand(LAST_NONCALLABLE_SPEC_OBJECT_TYPE));
    // Check for undetectable objects => false.
    __ ld1u(a1, FieldMemOperand(v0, Map::kBitFieldOffset));
    __ And(a1, a1, Operand(1 << Map::kIsUndetectable));
    Split(eq, a1, Operand(zero), if_true, if_false, fall_through);
  } else {
    if (if_false != fall_through) __ jmp(if_false);
  }
  context()->Plug(if_true, if_false);
}


void FullCodeGenerator::VisitCompareOperation(CompareOperation* expr) {
  Comment cmnt(masm_, "[ CompareOperation");
  SetSourcePosition(expr->position());

  // First we try a fast inlined version of the compare when one of
  // the operands is a literal.
  if (TryLiteralCompare(expr)) return;

  // Always perform the comparison for its control flow.  Pack the result
  // into the expression's context after the comparison is performed.
  Label materialize_true, materialize_false;
  Label* if_true = NULL;
  Label* if_false = NULL;
  Label* fall_through = NULL;
  context()->PrepareTest(&materialize_true, &materialize_false,
                         &if_true, &if_false, &fall_through);

  Token::Value op = expr->op();
  VisitForStackValue(expr->left());
  switch (op) {
    case Token::IN:
      VisitForStackValue(expr->right());
      __ InvokeBuiltin(Builtins::IN, CALL_FUNCTION);
      PrepareForBailoutBeforeSplit(expr, false, NULL, NULL);
      __ LoadRoot(t0, Heap::kTrueValueRootIndex);
      Split(eq, v0, Operand(t0), if_true, if_false, fall_through);
      break;

    case Token::INSTANCEOF: {
      VisitForStackValue(expr->right());
      InstanceofStub stub(InstanceofStub::kNoFlags);
      __ CallStub(&stub);
      PrepareForBailoutBeforeSplit(expr, true, if_true, if_false);
      // The stub returns 0 for true.
      Split(eq, v0, Operand(zero), if_true, if_false, fall_through);
      break;
    }

    default: {
      VisitForAccumulatorValue(expr->right());
      Condition cc = CompareIC::ComputeCondition(op);
      __ move(a0, result_register());
      __ pop(a1);

      bool inline_smi_code = ShouldInlineSmiCase(op);
      JumpPatchSite patch_site(masm_);
      if (inline_smi_code) {
        Label slow_case;
        __ Or(a2, a0, Operand(a1));
        patch_site.EmitJumpIfNotSmi(a2, &slow_case);
        Split(cc, a1, Operand(a0), if_true, if_false, NULL);
        __ bind(&slow_case);
      }
      // Record position and call the compare IC.
      SetSourcePosition(expr->position());
      Handle<Code> ic = CompareIC::GetUninitialized(isolate(), op);
      CallIC(ic, RelocInfo::CODE_TARGET, expr->CompareOperationFeedbackId());
      patch_site.EmitPatchInfo();
      PrepareForBailoutBeforeSplit(expr, true, if_true, if_false);
      Split(cc, v0, Operand(zero), if_true, if_false, fall_through);
    }
  }

  // Convert the result of the comparison into one expected for this
  // expression's context.
  context()->Plug(if_true, if_false);
}

void FullCodeGenerator::EmitLiteralCompareNil(CompareOperation* expr,
                                              Expression* sub_expr,
                                              NilValue nil) {
  Label materialize_true, materialize_false;
  Label* if_true = NULL;
  Label* if_false = NULL;
  Label* fall_through = NULL;
  context()->PrepareTest(&materialize_true, &materialize_false,
                         &if_true, &if_false, &fall_through);

  VisitForAccumulatorValue(sub_expr);
  PrepareForBailoutBeforeSplit(expr, true, if_true, if_false);
  EqualityKind kind = expr->op() == Token::EQ_STRICT
      ? kStrictEquality : kNonStrictEquality;
  __ move(a0, result_register());
  if (kind == kStrictEquality) {
    Heap::RootListIndex nil_value = nil == kNullValue ?
        Heap::kNullValueRootIndex :
        Heap::kUndefinedValueRootIndex;
    __ LoadRoot(a1, nil_value);
    Split(eq, a0, Operand(a1), if_true, if_false, fall_through);
  } else {
    Handle<Code> ic = CompareNilICStub::GetUninitialized(isolate(),
                                                         kNonStrictEquality,
                                                         nil);
    CallIC(ic, RelocInfo::CODE_TARGET, expr->CompareOperationFeedbackId());
    Split(ne, v0, Operand(zero), if_true, if_false, fall_through);
  }
  context()->Plug(if_true, if_false);
}

void FullCodeGenerator::VisitThisFunction(ThisFunction* expr) {
  __ ld(v0, MemOperand(fp, JavaScriptFrameConstants::kFunctionOffset));
  context()->Plug(v0);
}

Register FullCodeGenerator::result_register() {
  return r0;
}

Register FullCodeGenerator::context_register() {
  return cp;
}


void FullCodeGenerator::StoreToFrameField(int frame_offset, Register value) {
  ASSERT_EQ(POINTER_SIZE_ALIGN(frame_offset), (int64_t)frame_offset);
  ASSERT(!value.is(at));
  __ st(value, MemOperand(fp, frame_offset));
}

void FullCodeGenerator::LoadContextField(Register dst, int context_index) {
  __ ld(dst, ContextOperand(cp, context_index));
}

void FullCodeGenerator::PushFunctionArgumentForContextAllocation() {
  Scope* declaration_scope = scope()->DeclarationScope();
  if (declaration_scope->is_global_scope() ||
      declaration_scope->is_module_scope()) {
    // Contexts nested in the native context have a canonical empty function
    // as their closure, not the anonymous closure containing the global
    // code.  Pass a smi sentinel and let the runtime look up the empty
    // function.
    __ li(at, Operand(Smi::FromInt(0)));
  } else if (declaration_scope->is_eval_scope()) {
    // Contexts created by a call to eval have the same closure as the
    // context calling eval, not the anonymous closure containing the eval
    // code.  Fetch it from the context.
    __ ld(at, ContextOperand(cp, Context::CLOSURE_INDEX));
  } else {
    ASSERT(declaration_scope->is_function_scope());
    __ ld(at, MemOperand(fp, JavaScriptFrameConstants::kFunctionOffset));
  }
  __ push(at);
}

// ----------------------------------------------------------------------------
// Non-local control flow support.

void FullCodeGenerator::EnterFinallyBlock() {
  ASSERT(!result_register().is(a1));
  // Store result register while executing finally block.
  __ push(result_register());
  // Cook return address in link register to stack (smi encoded Code* delta).
  __ Subu(a1, ra, Operand(masm_->CodeObject()));
  ASSERT_EQ(32, kSmiTagSize + kSmiShiftSize);
  STATIC_ASSERT(0 == kSmiTag);
  __ SmiTag(a1);  // Convert to smi.

  // Store result register while executing finally block.
  __ push(a1);

  // Store pending message while executing finally block.
  ExternalReference pending_message_obj =
      ExternalReference::address_of_pending_message_obj(isolate());
  __ li(at, Operand(pending_message_obj));
  __ ld(a1, MemOperand(at));
  __ push(a1);

  ExternalReference has_pending_message =
      ExternalReference::address_of_has_pending_message(isolate());
  __ li(at, Operand(has_pending_message));
  __ ld(a1, MemOperand(at));
  __ SmiTag(a1);
  __ push(a1);

  ExternalReference pending_message_script =
      ExternalReference::address_of_pending_message_script(isolate());
  __ li(at, Operand(pending_message_script));
  __ ld(a1, MemOperand(at));
  __ push(a1);
}

void FullCodeGenerator::ExitFinallyBlock() {
  ASSERT(!result_register().is(a1));
  // Restore pending message from stack.
  __ pop(a1);
  ExternalReference pending_message_script =
      ExternalReference::address_of_pending_message_script(isolate());
  __ li(at, Operand(pending_message_script));
  __ st(a1, MemOperand(at));

  __ pop(a1);
  __ SmiUntag(a1);
  ExternalReference has_pending_message =
      ExternalReference::address_of_has_pending_message(isolate());
  __ li(at, Operand(has_pending_message));
  __ st(a1, MemOperand(at));

  __ pop(a1);
  ExternalReference pending_message_obj =
      ExternalReference::address_of_pending_message_obj(isolate());
  __ li(at, Operand(pending_message_obj));
  __ st(a1, MemOperand(at));

  // Restore result register from stack.
  __ pop(a1);

  // Uncook return address and return.
  __ pop(result_register());
  ASSERT_EQ(32, kSmiTagSize + kSmiShiftSize);
  __ sra(a1, a1, 32);  // Un-smi-tag value.
  __ Addu(at, a1, Operand(masm_->CodeObject()));
  __ Jump(at);
}


#undef __

#define __ ACCESS_MASM(masm())

FullCodeGenerator::NestedStatement* FullCodeGenerator::TryFinally::Exit(
    int* stack_depth,
    int* context_length) {
  // The macros used here must preserve the result register.

  // Because the handler block contains the context of the finally
  // code, we can restore it directly from there for the finally code
  // rather than iteratively unwinding contexts via their previous
  // links.
  __ Drop(*stack_depth);  // Down to the handler block.
  if (*context_length > 0) {
    // Restore the context to its dedicated register and the stack.
    __ ld(cp, MemOperand(sp, StackHandlerConstants::kContextOffset));
    __ st(cp, MemOperand(fp, StandardFrameConstants::kContextOffset));
  }
  __ PopTryHandler();
  __ Call(finally_entry_);

  *stack_depth = 0;
  *context_length = 0;
  return previous_;
}

#undef __

} }  // namespace v8::internal

#endif  // V8_TARGET_ARCH_TILEGX
