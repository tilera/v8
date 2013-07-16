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

#include "tilegx/lithium-codegen-tilegx.h"
#include "tilegx/lithium-gap-resolver-tilegx.h"
#include "code-stubs.h"
#include "stub-cache.h"

namespace v8 {
namespace internal {


class SafepointGenerator : public CallWrapper {
 public:
  SafepointGenerator(LCodeGen* codegen,
                     LPointerMap* pointers,
                     Safepoint::DeoptMode mode)
      : codegen_(codegen),
        pointers_(pointers),
        deopt_mode_(mode) { }
  virtual ~SafepointGenerator() { }

  virtual void BeforeCall(int call_size) const { }

  virtual void AfterCall() const {
    codegen_->RecordSafepoint(pointers_, deopt_mode_);
  }

 private:
  LCodeGen* codegen_;
  LPointerMap* pointers_;
  Safepoint::DeoptMode deopt_mode_;
};


#define __ masm()->

bool LCodeGen::GenerateCode() {
  HPhase phase("Z_Code generation", chunk());
  ASSERT(is_unused());
  status_ = GENERATING;

  // Open a frame scope to indicate that there is a frame on the stack.  The
  // NONE indicates that the scope shouldn't actually generate code to set up
  // the frame (that is done in GeneratePrologue).
  FrameScope frame_scope(masm_, StackFrame::NONE);

  return GeneratePrologue() &&
      GenerateBody() &&
      GenerateDeferredCode() &&
      GenerateDeoptJumpTable() &&
      GenerateSafepointTable();
}


void LCodeGen::FinishCode(Handle<Code> code) {
  ASSERT(is_done());
  code->set_stack_slots(GetStackSlotCount());
  code->set_safepoint_table_offset(safepoints_.GetCodeOffset());
  if (FLAG_weak_embedded_maps_in_optimized_code) {
    RegisterDependentCodeForEmbeddedMaps(code);
  }
  PopulateDeoptimizationData(code);
  for (int i = 0 ; i < prototype_maps_.length(); i++) {
    prototype_maps_.at(i)->AddDependentCode(
        DependentCode::kPrototypeCheckGroup, code);
  }
  for (int i = 0 ; i < transition_maps_.length(); i++) {
    transition_maps_.at(i)->AddDependentCode(
        DependentCode::kTransitionGroup, code);
  }
  if (graph()->depends_on_empty_array_proto_elements()) {
    isolate()->initial_object_prototype()->map()->AddDependentCode(
        DependentCode::kElementsCantBeAddedGroup, code);
    isolate()->initial_array_prototype()->map()->AddDependentCode(
        DependentCode::kElementsCantBeAddedGroup, code);
  }
}

void LChunkBuilder::Abort(const char* reason) {
  info()->set_bailout_reason(reason);
  status_ = ABORTED;
}

void LCodeGen::Comment(const char* format, ...) {
  if (!FLAG_code_comments) return;
  char buffer[4 * KB];
  StringBuilder builder(buffer, ARRAY_SIZE(buffer));
  va_list arguments;
  va_start(arguments, format);
  builder.AddFormattedList(format, arguments);
  va_end(arguments);

  // Copy the string before recording it in the assembler to avoid
  // issues when the stack allocated buffer goes out of scope.
  size_t length = builder.position();
  Vector<char> copy = Vector<char>::New(length + 1);
  OS::MemCopy(copy.start(), builder.Finalize(), copy.length());
  masm()->RecordComment(copy.start());
}

bool LCodeGen::GeneratePrologue() {
  ASSERT(is_generating());

  if (info()->IsOptimizing()) {
    ProfileEntryHookStub::MaybeCallEntryHook(masm_);

#ifdef DEBUG
    if (strlen(FLAG_stop_at) > 0 &&
        info_->function()->name()->IsUtf8EqualTo(CStrVector(FLAG_stop_at))) {
      __ stop("stop_at");
    }
#endif

    // a1: Callee's JS function.
    // cp: Callee's context.
    // fp: Caller's frame pointer.
    // lr: Caller's pc.

    // Strict mode functions and builtins need to replace the receiver
    // with undefined when called as functions (without an explicit
    // receiver object). r5 is zero for method calls and non-zero for
    // function calls.
    if (!info_->is_classic_mode() || info_->is_native()) {
      Label ok;
      __ Branch(&ok, eq, t1, Operand(zero));

      int receiver_offset = scope()->num_parameters() * kPointerSize;
      __ LoadRoot(a2, Heap::kUndefinedValueRootIndex);
      __ st(a2, MemOperand(sp, receiver_offset));
      __ bind(&ok);
    }
  }

  info()->set_prologue_offset(masm_->pc_offset());
  if (NeedsEagerFrame()) {
    if (info()->IsStub()) {
      __ Push(ra, fp, cp);
      __ Push(Smi::FromInt(StackFrame::STUB));
      // Adjust FP to point to saved FP.
      __ Addu(fp, sp, Operand(2 * kPointerSize));
    } else {
      // The following three instructions must remain together and unmodified
      // for code aging to work properly.
      __ Push(ra, fp, cp, a1);
      // Add unused load of ip to ensure prologue sequence is identical for
      // full-codegen and lithium-codegen.
      __ LoadRoot(at, Heap::kUndefinedValueRootIndex);
      // Adj. FP to point to saved FP.
      __ Addu(fp, sp, Operand(2 * kPointerSize));
    }
    frame_is_built_ = true;
    info_->AddNoFrameRange(0, masm_->pc_offset());
  }

  // Reserve space for the stack slots needed by the code.
  int slots = GetStackSlotCount();
  if (slots > 0) {
    if (FLAG_debug_code) {
      __ Subu(sp,  sp, Operand(slots * kPointerSize));
      __ push(a0);
      __ push(a1);
      __ Addu(a0, sp, Operand(slots *  kPointerSize));
      __ li(a1, Operand(kSlotsZapValue));
      Label loop;
      __ bind(&loop);
      __ Subu(a0, a0, Operand(kPointerSize));
      __ st(a1, MemOperand(a0, 2 * kPointerSize));
      __ Branch(&loop, ne, a0, Operand(sp));
      __ pop(a1);
      __ pop(a0);
    } else {
      __ Subu(sp, sp, Operand(slots * kPointerSize));
    }
  }

#if 0
  if (info()->saves_caller_doubles()) {
    Comment(";;; Save clobbered callee double registers");
    int count = 0;
    BitVector* doubles = chunk()->allocated_double_registers();
    BitVector::Iterator save_iterator(doubles);
    while (!save_iterator.Done()) {
      __ sdc1(DoubleRegister::FromAllocationIndex(save_iterator.Current()),
              MemOperand(sp, count * kDoubleSize));
      save_iterator.Advance();
      count++;
    }
  }
#endif

  // Possibly allocate a local context.
  int heap_slots = info()->num_heap_slots() - Context::MIN_CONTEXT_SLOTS;
  if (heap_slots > 0) {
    Comment(";;; Allocate local context");
    // Argument to NewContext is the function, which is in a1.
    __ push(a1);
    if (heap_slots <= FastNewContextStub::kMaximumSlots) {
      FastNewContextStub stub(heap_slots);
      __ CallStub(&stub);
    } else {
      __ CallRuntime(Runtime::kNewFunctionContext, 1);
    }
    RecordSafepoint(Safepoint::kNoLazyDeopt);
    // Context is returned in both v0 and cp.  It replaces the context
    // passed to us.  It's saved in the stack and kept live in cp.
    __ st(cp, MemOperand(fp, StandardFrameConstants::kContextOffset));
    // Copy any necessary parameters into the context.
    int num_parameters = scope()->num_parameters();
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
        // Update the write barrier. This clobbers a3 and a0.
        __ RecordWriteContextSlot(
            cp, target.offset(), a0, a3, GetRAState(), kSaveFPRegs);
      }
    }
    Comment(";;; End allocate local context");
  }

  // Trace the call.
  if (FLAG_trace && info()->IsOptimizing()) {
    __ CallRuntime(Runtime::kTraceEnter, 0);
  }
  EnsureSpaceForLazyDeopt();
  return !is_aborted();
}


bool LCodeGen::GenerateBody() {
  ASSERT(is_generating());
  bool emit_instructions = true;
  for (current_instruction_ = 0;
       !is_aborted() && current_instruction_ < instructions_->length();
       current_instruction_++) {
    LInstruction* instr = instructions_->at(current_instruction_);

    // Don't emit code for basic blocks with a replacement.
    if (instr->IsLabel()) {
      emit_instructions = !LLabel::cast(instr)->HasReplacement();
    }
    if (!emit_instructions) continue;

    if (FLAG_code_comments && instr->HasInterestingComment(this)) {
      Comment(";;; <@%d,#%d> %s",
              current_instruction_,
              instr->hydrogen_value()->id(),
              instr->Mnemonic());
    }

    instr->CompileToNative(this);
  }
  return !is_aborted();
}

bool LCodeGen::GenerateDeferredCode() {
  ASSERT(is_generating());
  if (deferred_.length() > 0) {
    for (int i = 0; !is_aborted() && i < deferred_.length(); i++) {
      LDeferredCode* code = deferred_[i];
      Comment(";;; <@%d,#%d> "
              "-------------------- Deferred %s --------------------",
              code->instruction_index(),
              code->instr()->hydrogen_value()->id(),
              code->instr()->Mnemonic());
      __ bind(code->entry());
      if (NeedsDeferredFrame()) {
        Comment(";;; Build frame");
        ASSERT(!frame_is_built_);
        ASSERT(info()->IsStub());
        frame_is_built_ = true;
        __ MultiPush(cp.bit() | fp.bit() | ra.bit());
        __ li(scratch0(), Operand(Smi::FromInt(StackFrame::STUB)));
        __ push(scratch0());
        __ Addu(fp, sp, Operand(2 * kPointerSize));
        Comment(";;; Deferred code");
      }
      code->Generate();
      if (NeedsDeferredFrame()) {
        Comment(";;; Destroy frame");
        ASSERT(frame_is_built_);
        __ pop(at);
        __ MultiPop(cp.bit() | fp.bit() | ra.bit());
        frame_is_built_ = false;
      }
      __ jmp(code->exit());
    }
  }
  // Deferred code is the last part of the instruction sequence. Mark
  // the generated code as done unless we bailed out.
  if (!is_aborted()) status_ = DONE;
  return !is_aborted();
}


bool LCodeGen::GenerateDeoptJumpTable() {
  // Check that the jump table is accessible from everywhere in the function
  // code, i.e. that offsets to the table can be encoded in the 16bit signed
  // immediate of a branch instruction.
  // To simplify we consider the code size from the first instruction to the
  // end of the jump table.
  if (!is_int16((masm()->pc_offset() / Assembler::kInstrSize) +
      deopt_jump_table_.length() * 12)) {
    Abort("Generated code is too large");
  }

  if (deopt_jump_table_.length() > 0) {
    Comment(";;; -------------------- Jump table --------------------");
  }
  Assembler::BlockTrampolinePoolScope block_trampoline_pool(masm_);
  Label table_start;
  __ bind(&table_start);
  Label needs_frame_not_call;
  Label needs_frame_is_call;
  for (int i = 0; i < deopt_jump_table_.length(); i++) {
    __ bind(&deopt_jump_table_[i].label);
    Address entry = deopt_jump_table_[i].address;
    Deoptimizer::BailoutType type = deopt_jump_table_[i].bailout_type;
    int id = Deoptimizer::GetDeoptimizationId(isolate(), entry, type);
    if (id == Deoptimizer::kNotDeoptimizationEntry) {
      Comment(";;; jump table entry %d.", i);
    } else {
      Comment(";;; jump table entry %d: deoptimization bailout %d.", i, id);
    }
    __ li(t9, Operand(ExternalReference::ForDeoptEntry(entry)));
    if (deopt_jump_table_[i].needs_frame) {
      if (type == Deoptimizer::LAZY) {
        if (needs_frame_is_call.is_bound()) {
          __ Branch(&needs_frame_is_call);
        } else {
          __ bind(&needs_frame_is_call);
          __ MultiPush(cp.bit() | fp.bit() | ra.bit());
          // This variant of deopt can only be used with stubs. Since we don't
          // have a function pointer to install in the stack frame that we're
          // building, install a special marker there instead.
          ASSERT(info()->IsStub());
          __ li(scratch0(), Operand(Smi::FromInt(StackFrame::STUB)));
          __ push(scratch0());
          __ Addu(fp, sp, Operand(2 * kPointerSize));
          __ Call(t9);
        }
      } else {
        if (needs_frame_not_call.is_bound()) {
          __ Branch(&needs_frame_not_call);
        } else {
          __ bind(&needs_frame_not_call);
          __ MultiPush(cp.bit() | fp.bit() | ra.bit());
          // This variant of deopt can only be used with stubs. Since we don't
          // have a function pointer to install in the stack frame that we're
          // building, install a special marker there instead.
          ASSERT(info()->IsStub());
          __ li(scratch0(), Operand(Smi::FromInt(StackFrame::STUB)));
          __ push(scratch0());
          __ Addu(fp, sp, Operand(2 * kPointerSize));
          __ Jump(t9);
        }
      }
    } else {
      if (type == Deoptimizer::LAZY) {
        __ Call(t9);
      } else {
        __ Jump(t9);
      }
    }
  }
  __ RecordComment("]");

  // The deoptimization jump table is the last part of the instruction
  // sequence. Mark the generated code as done unless we bailed out.
  if (!is_aborted()) status_ = DONE;
  return !is_aborted();
}


bool LCodeGen::GenerateSafepointTable() {
  ASSERT(is_done());
  safepoints_.Emit(masm(), GetStackSlotCount());
  return !is_aborted();
}


Register LCodeGen::ToRegister(int index) const {  UNREACHABLE();
  return Register::FromAllocationIndex(index);
}


DoubleRegister LCodeGen::ToDoubleRegister(int index) const {  UNREACHABLE();
  return Register::FromAllocationIndex(index);
}


Register LCodeGen::ToRegister(LOperand* op) const {  UNREACHABLE();
  return Register::FromAllocationIndex(0);
}


Register LCodeGen::EmitLoadRegister(LOperand* op, Register scratch) {  UNREACHABLE();
  return Register::FromAllocationIndex(0);
}


DoubleRegister LCodeGen::ToDoubleRegister(LOperand* op) const {  UNREACHABLE();
  return Register::FromAllocationIndex(0);
}

Handle<Object> LCodeGen::ToHandle(LConstantOperand* op) const {  UNREACHABLE(); 
  HConstant* constant = chunk_->LookupConstant(op);
  ASSERT(chunk_->LookupLiteralRepresentation(op).IsSmiOrTagged());
  return constant->handle();
}


bool LCodeGen::IsInteger32(LConstantOperand* op) const {  UNREACHABLE();  return false;}


bool LCodeGen::IsSmi(LConstantOperand* op) const {  UNREACHABLE();  return false;}


int LCodeGen::ToInteger32(LConstantOperand* op) const {  UNREACHABLE();  return -1;}


Smi* LCodeGen::ToSmi(LConstantOperand* op) const {  UNREACHABLE();  return NULL;}


double LCodeGen::ToDouble(LConstantOperand* op) const {  UNREACHABLE();  return 0.0;}


Operand LCodeGen::ToOperand(LOperand* op) {  UNREACHABLE();  return Operand(r0);}


MemOperand LCodeGen::ToMemOperand(LOperand* op) const {  UNREACHABLE();  return MemOperand(r0);}


MemOperand LCodeGen::ToHighMemOperand(LOperand* op) const {  UNREACHABLE();  return MemOperand(r0);}


void LCodeGen::WriteTranslation(LEnvironment* environment,
                                Translation* translation,
                                int* pushed_arguments_index,
                                int* pushed_arguments_count) {  UNREACHABLE();  }


void LCodeGen::AddToTranslation(Translation* translation,
                                LOperand* op,
                                bool is_tagged,
                                bool is_uint32,
                                bool arguments_known,
                                int arguments_index,
                                int arguments_count) {  UNREACHABLE();  }


void LCodeGen::CallCode(Handle<Code> code,
                        RelocInfo::Mode mode,
                        LInstruction* instr) {  UNREACHABLE();  }


void LCodeGen::CallCodeGeneric(Handle<Code> code,
                               RelocInfo::Mode mode,
                               LInstruction* instr,
                               SafepointMode safepoint_mode) {  UNREACHABLE();  }


void LCodeGen::CallRuntime(const Runtime::Function* function,
                           int num_arguments,
                           LInstruction* instr) {  UNREACHABLE();  }


void LCodeGen::CallRuntimeFromDeferred(Runtime::FunctionId id,
                                       int argc,
                                       LInstruction* instr) {  UNREACHABLE();  }


void LCodeGen::RegisterEnvironmentForDeoptimization(LEnvironment* environment,
                                                    Safepoint::DeoptMode mode) {

}


void LCodeGen::DeoptimizeIf(Condition cc,
                            LEnvironment* environment,
                            Deoptimizer::BailoutType bailout_type,
                            Register src1,
                            const Operand& src2) {  UNREACHABLE();  }


void LCodeGen::DeoptimizeIf(Condition cc,
                            LEnvironment* environment,
                            Register src1,
                            const Operand& src2) {  UNREACHABLE();  }


void LCodeGen::SoftDeoptimize(LEnvironment* environment,
                              Register src1,
                              const Operand& src2) {  UNREACHABLE();  }


void LCodeGen::RegisterDependentCodeForEmbeddedMaps(Handle<Code> code) {  UNREACHABLE();  }


void LCodeGen::PopulateDeoptimizationData(Handle<Code> code) {
  int length = deoptimizations_.length();
  if (length == 0) return;
  Handle<DeoptimizationInputData> data =
      factory()->NewDeoptimizationInputData(length, TENURED);

  Handle<ByteArray> translations =
      translations_.CreateByteArray(isolate()->factory());
  data->SetTranslationByteArray(*translations);
  data->SetInlinedFunctionCount(Smi::FromInt(inlined_function_count_));

  Handle<FixedArray> literals =
      factory()->NewFixedArray(deoptimization_literals_.length(), TENURED);
  { ALLOW_HANDLE_DEREF(isolate(),
                       "copying a ZoneList of handles into a FixedArray");
    for (int i = 0; i < deoptimization_literals_.length(); i++) {
      literals->set(i, *deoptimization_literals_[i]);
    }
    data->SetLiteralArray(*literals);
  }

  data->SetOsrAstId(Smi::FromInt(info_->osr_ast_id().ToInt()));
  data->SetOsrPcOffset(Smi::FromInt(osr_pc_offset_));

  // Populate the deoptimization entries.
  for (int i = 0; i < length; i++) {
    LEnvironment* env = deoptimizations_[i];
    data->SetAstId(i, env->ast_id());
    data->SetTranslationIndex(i, Smi::FromInt(env->translation_index()));
    data->SetArgumentsStackHeight(i,
                                  Smi::FromInt(env->arguments_stack_height()));
    data->SetPc(i, Smi::FromInt(env->pc_offset()));
  }
  code->set_deoptimization_data(*data);
}

int LCodeGen::DefineDeoptimizationLiteral(Handle<Object> literal) {
  int result = deoptimization_literals_.length();
  for (int i = 0; i < deoptimization_literals_.length(); ++i) {
    if (deoptimization_literals_[i].is_identical_to(literal)) return i;
  }
  deoptimization_literals_.Add(literal, zone());
  return result;
}

void LCodeGen::PopulateDeoptimizationLiteralsWithInlinedFunctions() {
  ASSERT(deoptimization_literals_.length() == 0);

  const ZoneList<Handle<JSFunction> >* inlined_closures =
      chunk()->inlined_closures();

  for (int i = 0, length = inlined_closures->length();
       i < length;
       i++) {
    DefineDeoptimizationLiteral(inlined_closures->at(i));
  }

  inlined_function_count_ = deoptimization_literals_.length();
}


void LCodeGen::RecordSafepointWithLazyDeopt(
    LInstruction* instr, SafepointMode safepoint_mode) {  UNREACHABLE();  }


void LCodeGen::RecordSafepoint(
    LPointerMap* pointers,
    Safepoint::Kind kind,
    int arguments,
    Safepoint::DeoptMode deopt_mode) {  UNREACHABLE();  }


void LCodeGen::RecordSafepoint(LPointerMap* pointers,
                               Safepoint::DeoptMode deopt_mode) {  UNREACHABLE();  }


void LCodeGen::RecordSafepoint(Safepoint::DeoptMode deopt_mode) {  UNREACHABLE();  }


void LCodeGen::RecordSafepointWithRegisters(LPointerMap* pointers,
                                            int arguments,
                                            Safepoint::DeoptMode deopt_mode) {  UNREACHABLE();  }


void LCodeGen::RecordSafepointWithRegistersAndDoubles(
    LPointerMap* pointers,
    int arguments,
    Safepoint::DeoptMode deopt_mode) {  UNREACHABLE();  }


void LCodeGen::RecordPosition(int position) {  UNREACHABLE();  }

static const char* LabelType(LLabel* label) {
  if (label->is_loop_header()) return " (loop header)";
  if (label->is_osr_entry()) return " (OSR entry)";
  return "";
}

void LCodeGen::DoLabel(LLabel* label) {
  Comment(";;; <@%d,#%d> -------------------- B%d%s --------------------",
          current_instruction_,
          label->hydrogen_value()->id(),
          label->block_id(),
          LabelType(label));
  __ bind(label->label());
  current_block_ = label->block_id();
  DoGap(label);
}

void LCodeGen::DoParallelMove(LParallelMove* move) {
  resolver_.Resolve(move);
}

void LCodeGen::DoGap(LGap* gap) {
  for (int i = LGap::FIRST_INNER_POSITION;
       i <= LGap::LAST_INNER_POSITION;
       i++) {
    LGap::InnerPosition inner_pos = static_cast<LGap::InnerPosition>(i);
    LParallelMove* move = gap->GetParallelMove(inner_pos);
    if (move != NULL) DoParallelMove(move);
  }
}

void LCodeGen::DoInstructionGap(LInstructionGap* instr) {
  DoGap(instr);
}

void LCodeGen::DoParameter(LParameter* instr) {  UNREACHABLE();  }


void LCodeGen::DoCallStub(LCallStub* instr) {  UNREACHABLE();  }


void LCodeGen::DoUnknownOSRValue(LUnknownOSRValue* instr) {  UNREACHABLE();  }


void LCodeGen::DoModI(LModI* instr) {  UNREACHABLE();  }


void LCodeGen::DoDivI(LDivI* instr) {  UNREACHABLE();  }


void LCodeGen::DoMultiplyAddD(LMultiplyAddD* instr) {  UNREACHABLE();  }


void LCodeGen::DoMulI(LMulI* instr) {  UNREACHABLE();  }


void LCodeGen::DoBitI(LBitI* instr) {  UNREACHABLE();  }


void LCodeGen::DoShiftI(LShiftI* instr) {  UNREACHABLE();  }


void LCodeGen::DoSubI(LSubI* instr) {  UNREACHABLE();  }


void LCodeGen::DoConstantI(LConstantI* instr) {  UNREACHABLE();  }


void LCodeGen::DoConstantD(LConstantD* instr) {  UNREACHABLE();  }


void LCodeGen::DoConstantT(LConstantT* instr) {  UNREACHABLE();  }


void LCodeGen::DoFixedArrayBaseLength(LFixedArrayBaseLength* instr) {  UNREACHABLE();  }


void LCodeGen::DoMapEnumLength(LMapEnumLength* instr) {  UNREACHABLE();  }


void LCodeGen::DoElementsKind(LElementsKind* instr) {  UNREACHABLE();  }


void LCodeGen::DoValueOf(LValueOf* instr) {  UNREACHABLE();  }


void LCodeGen::DoDateField(LDateField* instr) {  UNREACHABLE();  }


void LCodeGen::DoSeqStringSetChar(LSeqStringSetChar* instr) {  UNREACHABLE();  }


void LCodeGen::DoBitNotI(LBitNotI* instr) {  UNREACHABLE();  }


void LCodeGen::DoThrow(LThrow* instr) {  UNREACHABLE();  }


void LCodeGen::DoAddI(LAddI* instr) {  UNREACHABLE();  }


void LCodeGen::DoMathMinMax(LMathMinMax* instr) {  UNREACHABLE();  }


void LCodeGen::DoArithmeticD(LArithmeticD* instr) {  UNREACHABLE();  }


void LCodeGen::DoArithmeticT(LArithmeticT* instr) {  UNREACHABLE();  }


int LCodeGen::GetNextEmittedBlock() const {  UNREACHABLE();  return -1;}


void LCodeGen::EmitBranch(int left_block, int right_block,
                          Condition cc, Register src1, const Operand& src2) {  UNREACHABLE();  }


void LCodeGen::DoDebugBreak(LDebugBreak* instr) {  UNREACHABLE();  }


void LCodeGen::DoBranch(LBranch* instr) {  UNREACHABLE();  }


void LCodeGen::EmitGoto(int block) {  UNREACHABLE();  }


void LCodeGen::DoGoto(LGoto* instr) {  UNREACHABLE();  }


Condition LCodeGen::TokenToCondition(Token::Value op, bool is_unsigned) {  UNREACHABLE();  return al;}


void LCodeGen::DoCmpIDAndBranch(LCmpIDAndBranch* instr) {  UNREACHABLE();  }


void LCodeGen::DoCmpObjectEqAndBranch(LCmpObjectEqAndBranch* instr) {  UNREACHABLE();  }


void LCodeGen::DoCmpConstantEqAndBranch(LCmpConstantEqAndBranch* instr) {  UNREACHABLE();  }


Condition LCodeGen::EmitIsObject(Register input,
                                 Register temp1,
                                 Register temp2,
                                 Label* is_not_object,
                                 Label* is_object) {  UNREACHABLE();  return al;}


void LCodeGen::DoIsObjectAndBranch(LIsObjectAndBranch* instr) {  UNREACHABLE();  }


Condition LCodeGen::EmitIsString(Register input,
                                 Register temp1,
                                 Label* is_not_string) {  UNREACHABLE();  return al;}


void LCodeGen::DoIsStringAndBranch(LIsStringAndBranch* instr) {  UNREACHABLE();  }


void LCodeGen::DoIsSmiAndBranch(LIsSmiAndBranch* instr) {  UNREACHABLE();  }


void LCodeGen::DoIsUndetectableAndBranch(LIsUndetectableAndBranch* instr) {  UNREACHABLE();  }


void LCodeGen::DoStringCompareAndBranch(LStringCompareAndBranch* instr) {  UNREACHABLE();  }


void LCodeGen::DoHasInstanceTypeAndBranch(LHasInstanceTypeAndBranch* instr) {  UNREACHABLE();  }


void LCodeGen::DoGetCachedArrayIndex(LGetCachedArrayIndex* instr) {  UNREACHABLE();  }


void LCodeGen::DoHasCachedArrayIndexAndBranch(
    LHasCachedArrayIndexAndBranch* instr) {  UNREACHABLE();  }


// Branches to a label or falls through with the answer in flags.  Trashes
// the temp registers, but not the input.
void LCodeGen::EmitClassOfTest(Label* is_true,
                               Label* is_false,
                               Handle<String>class_name,
                               Register input,
                               Register temp,
                               Register temp2) {  UNREACHABLE();  }


void LCodeGen::DoClassOfTestAndBranch(LClassOfTestAndBranch* instr) {  UNREACHABLE();  }


void LCodeGen::DoCmpMapAndBranch(LCmpMapAndBranch* instr) {  UNREACHABLE();  }


void LCodeGen::DoInstanceOf(LInstanceOf* instr) {  UNREACHABLE();  }


void LCodeGen::DoInstanceOfKnownGlobal(LInstanceOfKnownGlobal* instr) {  UNREACHABLE();  }


void LCodeGen::DoDeferredInstanceOfKnownGlobal(LInstanceOfKnownGlobal* instr,
                                               Label* map_check) {  UNREACHABLE();  }


void LCodeGen::DoInstanceSize(LInstanceSize* instr) {  UNREACHABLE();  }


void LCodeGen::DoCmpT(LCmpT* instr) {  UNREACHABLE();  }


void LCodeGen::DoReturn(LReturn* instr) {  UNREACHABLE();  }


void LCodeGen::DoLoadGlobalCell(LLoadGlobalCell* instr) {  UNREACHABLE();  }


void LCodeGen::DoLoadGlobalGeneric(LLoadGlobalGeneric* instr) {  UNREACHABLE();  }


void LCodeGen::DoStoreGlobalCell(LStoreGlobalCell* instr) {  UNREACHABLE();  }


void LCodeGen::DoStoreGlobalGeneric(LStoreGlobalGeneric* instr) {  UNREACHABLE();  }


void LCodeGen::DoLoadContextSlot(LLoadContextSlot* instr) {  UNREACHABLE();  }


void LCodeGen::DoStoreContextSlot(LStoreContextSlot* instr) {  UNREACHABLE();  }


void LCodeGen::DoLoadNamedField(LLoadNamedField* instr) {  UNREACHABLE();  }


void LCodeGen::EmitLoadFieldOrConstantFunction(Register result,
                                               Register object,
                                               Handle<Map> type,
                                               Handle<String> name,
                                               LEnvironment* env) {  UNREACHABLE();  }


void LCodeGen::DoLoadNamedFieldPolymorphic(LLoadNamedFieldPolymorphic* instr) {  UNREACHABLE();  }


void LCodeGen::DoLoadNamedGeneric(LLoadNamedGeneric* instr) {  UNREACHABLE();  }


void LCodeGen::DoLoadFunctionPrototype(LLoadFunctionPrototype* instr) {  UNREACHABLE();  }


void LCodeGen::DoLoadExternalArrayPointer(
    LLoadExternalArrayPointer* instr) {  UNREACHABLE();  }


void LCodeGen::DoAccessArgumentsAt(LAccessArgumentsAt* instr) {  UNREACHABLE();  }


void LCodeGen::DoLoadKeyedExternalArray(LLoadKeyed* instr) {  UNREACHABLE();  }


void LCodeGen::DoLoadKeyedFixedDoubleArray(LLoadKeyed* instr) {  UNREACHABLE();  }


void LCodeGen::DoLoadKeyedFixedArray(LLoadKeyed* instr) {  UNREACHABLE();  }


void LCodeGen::DoLoadKeyed(LLoadKeyed* instr) {  UNREACHABLE();  }


MemOperand LCodeGen::PrepareKeyedOperand(Register key,
                                         Register base,
                                         bool key_is_constant,
                                         int constant_key,
                                         int element_size,
                                         int shift_size,
                                         int additional_index,
                                         int additional_offset) {  UNREACHABLE();  return MemOperand(r0);}


void LCodeGen::DoLoadKeyedGeneric(LLoadKeyedGeneric* instr) {  UNREACHABLE();  }


void LCodeGen::DoArgumentsElements(LArgumentsElements* instr) {  UNREACHABLE();  }


void LCodeGen::DoArgumentsLength(LArgumentsLength* instr) {  UNREACHABLE();  }


void LCodeGen::DoWrapReceiver(LWrapReceiver* instr) {  UNREACHABLE();  }

void LCodeGen::DoApplyArguments(LApplyArguments* instr) {  UNREACHABLE();  }


void LCodeGen::DoPushArgument(LPushArgument* instr) {  UNREACHABLE();  }


void LCodeGen::DoDrop(LDrop* instr) {  UNREACHABLE();  }


void LCodeGen::DoThisFunction(LThisFunction* instr) {  UNREACHABLE();  }


void LCodeGen::DoContext(LContext* instr) {  UNREACHABLE();  }


void LCodeGen::DoOuterContext(LOuterContext* instr) {  UNREACHABLE();  }


void LCodeGen::DoDeclareGlobals(LDeclareGlobals* instr) {  UNREACHABLE();  }


void LCodeGen::DoGlobalObject(LGlobalObject* instr) {  UNREACHABLE();  }


void LCodeGen::DoGlobalReceiver(LGlobalReceiver* instr) {  UNREACHABLE();  }


void LCodeGen::CallKnownFunction(Handle<JSFunction> function,
                                 int formal_parameter_count,
                                 int arity,
                                 LInstruction* instr,
                                 CallKind call_kind,
                                 A1State a1_state) {  UNREACHABLE();  }


void LCodeGen::DoCallConstantFunction(LCallConstantFunction* instr) {  UNREACHABLE();  }


void LCodeGen::DoDeferredMathAbsTaggedHeapNumber(LMathAbs* instr) {  UNREACHABLE();  }


void LCodeGen::EmitIntegerMathAbs(LMathAbs* instr) {  UNREACHABLE();  }


void LCodeGen::DoMathAbs(LMathAbs* instr) {  UNREACHABLE();  }


void LCodeGen::DoMathFloor(LMathFloor* instr) {  UNREACHABLE();  }


void LCodeGen::DoMathRound(LMathRound* instr) {  UNREACHABLE();  }


void LCodeGen::DoMathSqrt(LMathSqrt* instr) {  UNREACHABLE();  }


void LCodeGen::DoMathPowHalf(LMathPowHalf* instr) {  UNREACHABLE();  }


void LCodeGen::DoPower(LPower* instr) {  UNREACHABLE();  }


void LCodeGen::DoRandom(LRandom* instr) {  UNREACHABLE();  }

void LCodeGen::DoDeferredRandom(LRandom* instr) {  UNREACHABLE();  }


void LCodeGen::DoMathExp(LMathExp* instr) {  UNREACHABLE();  }


void LCodeGen::DoMathLog(LMathLog* instr) {  UNREACHABLE();  }


void LCodeGen::DoMathTan(LMathTan* instr) {  UNREACHABLE();  }


void LCodeGen::DoMathCos(LMathCos* instr) {  UNREACHABLE();  }


void LCodeGen::DoMathSin(LMathSin* instr) {  UNREACHABLE();  }


void LCodeGen::DoInvokeFunction(LInvokeFunction* instr) {  UNREACHABLE();  }


void LCodeGen::DoCallKeyed(LCallKeyed* instr) {  UNREACHABLE();  }


void LCodeGen::DoCallNamed(LCallNamed* instr) {  UNREACHABLE();  }


void LCodeGen::DoCallFunction(LCallFunction* instr) {  UNREACHABLE();  }


void LCodeGen::DoCallGlobal(LCallGlobal* instr) {  UNREACHABLE();  }


void LCodeGen::DoCallKnownGlobal(LCallKnownGlobal* instr) {  UNREACHABLE();  }


void LCodeGen::DoCallNew(LCallNew* instr) {  UNREACHABLE();  }


void LCodeGen::DoCallNewArray(LCallNewArray* instr) {  UNREACHABLE();  }


void LCodeGen::DoCallRuntime(LCallRuntime* instr) {  UNREACHABLE();  }


void LCodeGen::DoInnerAllocatedObject(LInnerAllocatedObject* instr) {  UNREACHABLE();  }


void LCodeGen::DoStoreNamedField(LStoreNamedField* instr) {  UNREACHABLE();  }


void LCodeGen::DoStoreNamedGeneric(LStoreNamedGeneric* instr) {  UNREACHABLE();  }


void LCodeGen::DoBoundsCheck(LBoundsCheck* instr) {  UNREACHABLE();  }


void LCodeGen::DoStoreKeyedExternalArray(LStoreKeyed* instr) {  UNREACHABLE();  }


void LCodeGen::DoStoreKeyedFixedDoubleArray(LStoreKeyed* instr) {  UNREACHABLE();  }


void LCodeGen::DoStoreKeyedFixedArray(LStoreKeyed* instr) {  UNREACHABLE();  }


void LCodeGen::DoStoreKeyed(LStoreKeyed* instr) {  UNREACHABLE();  }


void LCodeGen::DoStoreKeyedGeneric(LStoreKeyedGeneric* instr) {  UNREACHABLE();  }


void LCodeGen::DoTransitionElementsKind(LTransitionElementsKind* instr) {  UNREACHABLE();  }


void LCodeGen::DoTrapAllocationMemento(LTrapAllocationMemento* instr) {  UNREACHABLE();  }


void LCodeGen::DoStringAdd(LStringAdd* instr) {  UNREACHABLE();  }


void LCodeGen::DoStringCharCodeAt(LStringCharCodeAt* instr) {  UNREACHABLE();  }


void LCodeGen::DoDeferredStringCharCodeAt(LStringCharCodeAt* instr) {  UNREACHABLE();  }


void LCodeGen::DoStringCharFromCode(LStringCharFromCode* instr) {  UNREACHABLE();  }


void LCodeGen::DoDeferredStringCharFromCode(LStringCharFromCode* instr) {  UNREACHABLE();  }


void LCodeGen::DoStringLength(LStringLength* instr) {  UNREACHABLE();  }


void LCodeGen::DoInteger32ToDouble(LInteger32ToDouble* instr) {  UNREACHABLE();  }


void LCodeGen::DoInteger32ToSmi(LInteger32ToSmi* instr) {  UNREACHABLE();  }


void LCodeGen::DoUint32ToDouble(LUint32ToDouble* instr) {  UNREACHABLE();  }


void LCodeGen::DoNumberTagI(LNumberTagI* instr) {  UNREACHABLE();  }


void LCodeGen::DoNumberTagU(LNumberTagU* instr) {  UNREACHABLE();  }


void LCodeGen::DoDeferredNumberTagI(LInstruction* instr,
                                    LOperand* value,
                                    IntegerSignedness signedness) {  UNREACHABLE();  }


void LCodeGen::DoNumberTagD(LNumberTagD* instr) {  UNREACHABLE();  }


void LCodeGen::DoDeferredNumberTagD(LNumberTagD* instr) {  UNREACHABLE();  }


void LCodeGen::DoSmiTag(LSmiTag* instr) {  UNREACHABLE();  }


void LCodeGen::DoSmiUntag(LSmiUntag* instr) {  UNREACHABLE();  }


void LCodeGen::EmitNumberUntagD(Register input_reg,
                                DoubleRegister result_reg,
                                bool deoptimize_on_undefined,
                                bool deoptimize_on_minus_zero,
                                LEnvironment* env,
                                NumberUntagDMode mode) {  UNREACHABLE();  }


void LCodeGen::DoDeferredTaggedToI(LTaggedToI* instr) {  UNREACHABLE();  }


void LCodeGen::DoTaggedToI(LTaggedToI* instr) {  UNREACHABLE();  }


void LCodeGen::DoNumberUntagD(LNumberUntagD* instr) {  UNREACHABLE();  }


void LCodeGen::DoDoubleToI(LDoubleToI* instr) {  UNREACHABLE();  }


void LCodeGen::DoDoubleToSmi(LDoubleToSmi* instr) {  UNREACHABLE();  }


void LCodeGen::DoCheckSmi(LCheckSmi* instr) {  UNREACHABLE();  }


void LCodeGen::DoCheckNonSmi(LCheckNonSmi* instr) {  UNREACHABLE();  }


void LCodeGen::DoCheckInstanceType(LCheckInstanceType* instr) {  UNREACHABLE();  }


void LCodeGen::DoCheckFunction(LCheckFunction* instr) {  UNREACHABLE();  }


void LCodeGen::DoCheckMapCommon(Register map_reg,
                                Handle<Map> map,
                                LEnvironment* env) {  UNREACHABLE();  }


void LCodeGen::DoCheckMaps(LCheckMaps* instr) {  UNREACHABLE();  }


void LCodeGen::DoClampDToUint8(LClampDToUint8* instr) {  UNREACHABLE();  }


void LCodeGen::DoClampIToUint8(LClampIToUint8* instr) {  UNREACHABLE();  }


void LCodeGen::DoClampTToUint8(LClampTToUint8* instr) {  UNREACHABLE();  }


void LCodeGen::DoCheckPrototypeMaps(LCheckPrototypeMaps* instr) {  UNREACHABLE();  }


void LCodeGen::DoAllocateObject(LAllocateObject* instr) {  UNREACHABLE();  }


void LCodeGen::DoDeferredAllocateObject(LAllocateObject* instr) {  UNREACHABLE();  }


void LCodeGen::DoAllocate(LAllocate* instr) {  UNREACHABLE();  }


void LCodeGen::DoDeferredAllocate(LAllocate* instr) {  UNREACHABLE();  }


void LCodeGen::DoToFastProperties(LToFastProperties* instr) {  UNREACHABLE();  }


void LCodeGen::DoRegExpLiteral(LRegExpLiteral* instr) {  UNREACHABLE();  }


void LCodeGen::DoFunctionLiteral(LFunctionLiteral* instr) {  UNREACHABLE();  }


void LCodeGen::DoTypeof(LTypeof* instr) {  UNREACHABLE();  }


void LCodeGen::DoTypeofIsAndBranch(LTypeofIsAndBranch* instr) {  UNREACHABLE();  }


Condition LCodeGen::EmitTypeofIs(Label* true_label,
                                 Label* false_label,
                                 Register input,
                                 Handle<String> type_name,
                                 Register& cmp1,
                                 Operand& cmp2) {  UNREACHABLE();  return al;}


void LCodeGen::DoIsConstructCallAndBranch(LIsConstructCallAndBranch* instr) {  UNREACHABLE();  }


void LCodeGen::EmitIsConstructCall(Register temp1, Register temp2) {  UNREACHABLE();  }


void LCodeGen::EnsureSpaceForLazyDeopt() {
  if (info()->IsStub()) return;
  // Ensure that we have enough space after the previous lazy-bailout
  // instruction for patching the code here.
  int current_pc = masm()->pc_offset();
  int patch_size = Deoptimizer::patch_size();
  if (current_pc < last_lazy_deopt_pc_ + patch_size) {
    int padding_size = last_lazy_deopt_pc_ + patch_size - current_pc;
    ASSERT_EQ(0, padding_size % Assembler::kInstrSize);
    while (padding_size > 0) {
      __ nop();
      padding_size -= Assembler::kInstrSize;
    }
  }
  last_lazy_deopt_pc_ = masm()->pc_offset();
}

void LCodeGen::DoLazyBailout(LLazyBailout* instr) {
  EnsureSpaceForLazyDeopt();
  ASSERT(instr->HasEnvironment());
  LEnvironment* env = instr->environment();
  RegisterEnvironmentForDeoptimization(env, Safepoint::kLazyDeopt);
  safepoints_.RecordLazyDeoptimizationIndex(env->deoptimization_index());
}

void LCodeGen::DoDeoptimize(LDeoptimize* instr) {
  if (instr->hydrogen_value()->IsSoftDeoptimize()) {
    SoftDeoptimize(instr->environment(), zero, Operand(zero));
  } else {
    DeoptimizeIf(al, instr->environment(), zero, Operand(zero));
  }
}

void LCodeGen::DoDummyUse(LDummyUse* instr) {
  // Nothing to see here, move on!
}


void LCodeGen::DoDeleteProperty(LDeleteProperty* instr) {
  Register object = ToRegister(instr->object());
  Register key = ToRegister(instr->key());
  Register strict = scratch0();
  __ li(strict, Operand(Smi::FromInt(strict_mode_flag())));
  __ Push(object, key, strict);
  ASSERT(instr->HasPointerMap());
  LPointerMap* pointers = instr->pointer_map();
  RecordPosition(pointers->position());
  SafepointGenerator safepoint_generator(
      this, pointers, Safepoint::kLazyDeopt);
  __ InvokeBuiltin(Builtins::DELETE, CALL_FUNCTION, safepoint_generator);
}

void LCodeGen::DoIn(LIn* instr) {
  Register obj = ToRegister(instr->object());
  Register key = ToRegister(instr->key());
  __ Push(key, obj);
  ASSERT(instr->HasPointerMap());
  LPointerMap* pointers = instr->pointer_map();
  RecordPosition(pointers->position());
  SafepointGenerator safepoint_generator(this, pointers, Safepoint::kLazyDeopt);
  __ InvokeBuiltin(Builtins::IN, CALL_FUNCTION, safepoint_generator);
}

void LCodeGen::DoDeferredStackCheck(LStackCheck* instr) {  UNREACHABLE();  }


void LCodeGen::DoStackCheck(LStackCheck* instr) {  UNREACHABLE();  }


void LCodeGen::DoOsrEntry(LOsrEntry* instr) {  UNREACHABLE();  }


void LCodeGen::DoForInPrepareMap(LForInPrepareMap* instr) {  UNREACHABLE();  }


void LCodeGen::DoForInCacheArray(LForInCacheArray* instr) {  UNREACHABLE();  }


void LCodeGen::DoCheckMapValue(LCheckMapValue* instr) {  UNREACHABLE();  }


void LCodeGen::DoLoadFieldByIndex(LLoadFieldByIndex* instr) {
  Register object = ToRegister(instr->object());
  Register index = ToRegister(instr->index());
  Register result = ToRegister(instr->result());
  Register scratch = scratch0();

  Label out_of_object, done;
  __ sll(scratch, index, kPointerSizeLog2 - kSmiTagSize);  // In delay slot.
  __ Branch(&out_of_object, lt, index, Operand(zero));

  STATIC_ASSERT(kPointerSizeLog2 > kSmiTagSize);
  __ Addu(scratch, object, scratch);
  __ ld(result, FieldMemOperand(scratch, JSObject::kHeaderSize));

  __ Branch(&done);

  __ bind(&out_of_object);
  __ ld(result, FieldMemOperand(object, JSObject::kPropertiesOffset));
  // Index is equal to negated out of object property index plus 1.
  __ Subu(scratch, result, scratch);
  __ ld(result, FieldMemOperand(scratch,
                                FixedArray::kHeaderSize - kPointerSize));
  __ bind(&done);
}


#undef __

} }  // namespace v8::internal
