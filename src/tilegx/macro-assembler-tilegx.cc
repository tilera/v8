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

#include <limits.h>  // For LONG_MIN, LONG_MAX.

#include "v8.h"

#if defined(V8_TARGET_ARCH_TILEGX)

#include "bootstrapper.h"
#include "codegen.h"
#include "debug.h"
#include "runtime.h"

namespace v8 {
namespace internal {

MacroAssembler::MacroAssembler(Isolate* arg_isolate, void* buffer, int size)
    : Assembler(arg_isolate, buffer, size),
      generating_stub_(false),
      allow_stub_calls_(true),
      has_frame_(false) {
  if (isolate() != NULL) {
    code_object_ = Handle<Object>(isolate()->heap()->undefined_value(),
                                  isolate());
  }
}

void MacroAssembler::LoadRoot(Register destination,
                              Heap::RootListIndex index) { UNREACHABLE(); }


void MacroAssembler::LoadRoot(Register destination,
                              Heap::RootListIndex index,
                              Condition cond,
                              Register src1, const Operand& src2) { UNREACHABLE(); }


void MacroAssembler::StoreRoot(Register source,
                               Heap::RootListIndex index) { UNREACHABLE(); }


void MacroAssembler::StoreRoot(Register source,
                               Heap::RootListIndex index,
                               Condition cond,
                               Register src1, const Operand& src2) { UNREACHABLE(); }


void MacroAssembler::LoadHeapObject(Register result,
                                    Handle<HeapObject> object) { UNREACHABLE(); }


// Push and pop all registers that can hold pointers.
void MacroAssembler::PushSafepointRegisters() { UNREACHABLE(); }


void MacroAssembler::PopSafepointRegisters() { UNREACHABLE(); }


void MacroAssembler::PushSafepointRegistersAndDoubles() { UNREACHABLE(); }


void MacroAssembler::PopSafepointRegistersAndDoubles() { UNREACHABLE(); }


void MacroAssembler::StoreToSafepointRegistersAndDoublesSlot(Register src,
                                                             Register dst) { UNREACHABLE(); }


void MacroAssembler::StoreToSafepointRegisterSlot(Register src, Register dst) { UNREACHABLE(); }


void MacroAssembler::LoadFromSafepointRegisterSlot(Register dst, Register src) { UNREACHABLE(); }


int MacroAssembler::SafepointRegisterStackIndex(int reg_code) { UNREACHABLE(); return -1; }


MemOperand MacroAssembler::SafepointRegisterSlot(Register reg) { UNREACHABLE(); return MemOperand(fp); }


MemOperand MacroAssembler::SafepointRegistersAndDoublesSlot(Register reg) { UNREACHABLE(); return MemOperand(fp); }


void MacroAssembler::InNewSpace(Register object,
                                Register scratch,
                                Condition cc,
                                Label* branch) { UNREACHABLE(); }


void MacroAssembler::RecordWriteField(
    Register object,
    int offset,
    Register value,
    Register dst,
    RAStatus ra_status,
    SaveFPRegsMode save_fp,
    RememberedSetAction remembered_set_action,
    SmiCheck smi_check) { UNREACHABLE(); }


// Will clobber 4 registers: object, address, scratch, ip.  The
// register 'object' contains a heap object pointer.  The heap object
// tag is shifted away.
void MacroAssembler::RecordWrite(Register object,
                                 Register address,
                                 Register value,
                                 RAStatus ra_status,
                                 SaveFPRegsMode fp_mode,
                                 RememberedSetAction remembered_set_action,
                                 SmiCheck smi_check) { UNREACHABLE(); }


void MacroAssembler::RememberedSetHelper(Register object,  // For debug tests.
                                         Register address,
                                         Register scratch,
                                         SaveFPRegsMode fp_mode,
                                         RememberedSetFinalAction and_then) { UNREACHABLE(); }


// -----------------------------------------------------------------------------
// Allocation support.


void MacroAssembler::CheckAccessGlobalProxy(Register holder_reg,
                                            Register scratch,
                                            Label* miss) { UNREACHABLE(); }


void MacroAssembler::GetNumberHash(Register reg0, Register scratch) { UNREACHABLE(); }


void MacroAssembler::LoadFromNumberDictionary(Label* miss,
                                              Register elements,
                                              Register key,
                                              Register result,
                                              Register reg0,
                                              Register reg1,
                                              Register reg2) { UNREACHABLE(); }


// ---------------------------------------------------------------------------
// Instruction macros.

void MacroAssembler::Addu(Register rd, Register rs, const Operand& rt) { UNREACHABLE(); }


void MacroAssembler::Subu(Register rd, Register rs, const Operand& rt) { UNREACHABLE(); }


void MacroAssembler::Mul(Register rd, Register rs, const Operand& rt) { UNREACHABLE(); }


void MacroAssembler::Mult(Register rs, const Operand& rt) { UNREACHABLE(); }


void MacroAssembler::Multu(Register rs, const Operand& rt) { UNREACHABLE(); }


void MacroAssembler::Div(Register rs, const Operand& rt) { UNREACHABLE(); }


void MacroAssembler::Divu(Register rs, const Operand& rt) { UNREACHABLE(); }


void MacroAssembler::And(Register rd, Register rs, const Operand& rt) { UNREACHABLE(); }


void MacroAssembler::Or(Register rd, Register rs, const Operand& rt) { UNREACHABLE(); }


void MacroAssembler::Xor(Register rd, Register rs, const Operand& rt) { UNREACHABLE(); }


void MacroAssembler::Nor(Register rd, Register rs, const Operand& rt) { UNREACHABLE(); }


void MacroAssembler::Neg(Register rs, const Operand& rt) { UNREACHABLE(); }


void MacroAssembler::Slt(Register rd, Register rs, const Operand& rt) { UNREACHABLE(); }


void MacroAssembler::Sltu(Register rd, Register rs, const Operand& rt) { UNREACHABLE(); }


void MacroAssembler::Ror(Register rd, Register rs, const Operand& rt) { UNREACHABLE(); }

//------------Pseudo-instructions-------------

void MacroAssembler::li(Register rd, Operand j, LiFlags mode) { UNREACHABLE(); }


void MacroAssembler::MultiPush(RegList regs) { UNREACHABLE(); }


void MacroAssembler::MultiPushReversed(RegList regs) { UNREACHABLE(); }


void MacroAssembler::MultiPop(RegList regs) { UNREACHABLE(); }


void MacroAssembler::MultiPopReversed(RegList regs) { UNREACHABLE(); }


void MacroAssembler::MultiPushFPU(RegList regs) { UNREACHABLE(); }


void MacroAssembler::MultiPushReversedFPU(RegList regs) { UNREACHABLE(); }


void MacroAssembler::MultiPopFPU(RegList regs) { UNREACHABLE(); }


void MacroAssembler::MultiPopReversedFPU(RegList regs) { UNREACHABLE(); }


void MacroAssembler::FlushICache(Register address, unsigned instructions) { UNREACHABLE(); }


// Emulated condtional branches do not emit a nop in the branch delay slot.
//
// BRANCH_ARGS_CHECK checks that conditional jump arguments are correct.
#define BRANCH_ARGS_CHECK(cond, rs, rt) ASSERT(                                \
    (cond == cc_always && rs.is(zero_reg) && rt.rm().is(zero_reg)) ||          \
    (cond != cc_always && (!rs.is(zero_reg) || !rt.rm().is(zero_reg))))


void MacroAssembler::Branch(int16_t offset) { UNREACHABLE(); }


void MacroAssembler::Branch(int16_t offset, Condition cond, Register rs,
                            const Operand& rt) { UNREACHABLE(); }


void MacroAssembler::Branch(Label* L) { UNREACHABLE(); }


void MacroAssembler::Branch(Label* L, Condition cond, Register rs,
                            const Operand& rt) { UNREACHABLE(); }


void MacroAssembler::Branch(Label* L,
                            Condition cond,
                            Register rs,
                            Heap::RootListIndex index) { UNREACHABLE(); }


void MacroAssembler::BranchAndLink(int16_t offset) { UNREACHABLE(); }


void MacroAssembler::BranchAndLink(int16_t offset, Condition cond, Register rs,
                                   const Operand& rt) { UNREACHABLE(); }


void MacroAssembler::BranchAndLink(Label* L) { UNREACHABLE(); }


void MacroAssembler::BranchAndLink(Label* L, Condition cond, Register rs,
                                   const Operand& rt) { UNREACHABLE(); }


void MacroAssembler::Jump(Register target,
                          Condition cond,
                          Register rs,
                          const Operand& rt) { UNREACHABLE(); }


void MacroAssembler::Jump(intptr_t target,
                          RelocInfo::Mode rmode,
                          Condition cond,
                          Register rs,
                          const Operand& rt) { UNREACHABLE(); }


void MacroAssembler::Jump(Address target,
                          RelocInfo::Mode rmode,
                          Condition cond,
                          Register rs,
                          const Operand& rt) { UNREACHABLE(); }


void MacroAssembler::Jump(Handle<Code> code,
                          RelocInfo::Mode rmode,
                          Condition cond,
                          Register rs,
                          const Operand& rt) { UNREACHABLE(); }


int MacroAssembler::CallSize(Register target,
                             Condition cond,
                             Register rs,
                             const Operand& rt) { UNREACHABLE(); return -1; }


void MacroAssembler::Call(Register target,
                          Condition cond,
                          Register rs,
                          const Operand& rt) { UNREACHABLE(); }


int MacroAssembler::CallSize(Address target,
                             RelocInfo::Mode rmode,
                             Condition cond,
                             Register rs,
                             const Operand& rt) { UNREACHABLE(); return -1;}


void MacroAssembler::Call(Address target,
                          RelocInfo::Mode rmode,
                          Condition cond,
                          Register rs,
                          const Operand& rt) { UNREACHABLE(); }


int MacroAssembler::CallSize(Handle<Code> code,
                             RelocInfo::Mode rmode,
                             TypeFeedbackId ast_id,
                             Condition cond,
                             Register rs,
                             const Operand& rt) { UNREACHABLE(); return -1; }


void MacroAssembler::Call(Handle<Code> code,
                          RelocInfo::Mode rmode,
                          TypeFeedbackId ast_id,
                          Condition cond,
                          Register rs,
                          const Operand& rt) { UNREACHABLE(); }


void MacroAssembler::Ret(Condition cond,
                         Register rs,
                         const Operand& rt) { UNREACHABLE(); }


void MacroAssembler::Drop(int count,
                          Condition cond,
                          Register reg,
                          const Operand& op) { UNREACHABLE(); }

void MacroAssembler::Call(Label* target) { UNREACHABLE(); }


void MacroAssembler::Push(Handle<Object> handle) { UNREACHABLE(); }


#ifdef ENABLE_DEBUGGER_SUPPORT

void MacroAssembler::DebugBreak() {
  CEntryStub ces(1);
  ASSERT(AllowThisStubCall(&ces));
  Call(ces.GetCode(isolate()), RelocInfo::DEBUG_BREAK);
}

#endif  // ENABLE_DEBUGGER_SUPPORT


// ---------------------------------------------------------------------------
// Exception handling.

void MacroAssembler::PushTryHandler(StackHandler::Kind kind,
                                    int handler_index) { UNREACHABLE(); }


void MacroAssembler::PopTryHandler() { UNREACHABLE(); }


void MacroAssembler::JumpToHandlerEntry() { UNREACHABLE(); }


void MacroAssembler::Throw(Register value) { UNREACHABLE(); }


void MacroAssembler::ThrowUncatchable(Register value) { UNREACHABLE(); }


void MacroAssembler::Allocate(int object_size,
                              Register result,
                              Register scratch1,
                              Register scratch2,
                              Label* gc_required,
                              AllocationFlags flags) { UNREACHABLE(); }


void MacroAssembler::Allocate(Register object_size,
                              Register result,
                              Register scratch1,
                              Register scratch2,
                              Label* gc_required,
                              AllocationFlags flags) { UNREACHABLE(); }


void MacroAssembler::UndoAllocationInNewSpace(Register object,
                                              Register scratch) { UNREACHABLE(); }


void MacroAssembler::AllocateTwoByteString(Register result,
                                           Register length,
                                           Register scratch1,
                                           Register scratch2,
                                           Register scratch3,
                                           Label* gc_required) { UNREACHABLE(); }


void MacroAssembler::AllocateAsciiString(Register result,
                                         Register length,
                                         Register scratch1,
                                         Register scratch2,
                                         Register scratch3,
                                         Label* gc_required) { UNREACHABLE(); }


void MacroAssembler::AllocateTwoByteConsString(Register result,
                                               Register length,
                                               Register scratch1,
                                               Register scratch2,
                                               Label* gc_required) { UNREACHABLE(); }


void MacroAssembler::AllocateAsciiConsString(Register result,
                                             Register length,
                                             Register scratch1,
                                             Register scratch2,
                                             Label* gc_required) { UNREACHABLE(); }


void MacroAssembler::AllocateTwoByteSlicedString(Register result,
                                                 Register length,
                                                 Register scratch1,
                                                 Register scratch2,
                                                 Label* gc_required) { UNREACHABLE(); }


void MacroAssembler::AllocateAsciiSlicedString(Register result,
                                               Register length,
                                               Register scratch1,
                                               Register scratch2,
                                               Label* gc_required) { UNREACHABLE(); }


// Allocates a heap number or jumps to the label if the young space is full and
// a scavenge is needed.
void MacroAssembler::AllocateHeapNumber(Register result,
                                        Register scratch1,
                                        Register scratch2,
                                        Register heap_number_map,
                                        Label* need_gc,
                                        TaggingMode tagging_mode) { UNREACHABLE(); }



void MacroAssembler::CopyBytes(Register src,
                               Register dst,
                               Register length,
                               Register scratch) { UNREACHABLE(); }


void MacroAssembler::InitializeFieldsWithFiller(Register start_offset,
                                                Register end_offset,
                                                Register filler) { UNREACHABLE(); }


void MacroAssembler::CheckFastElements(Register map,
                                       Register scratch,
                                       Label* fail) { UNREACHABLE(); }


void MacroAssembler::CheckFastObjectElements(Register map,
                                             Register scratch,
                                             Label* fail) { UNREACHABLE(); }


void MacroAssembler::CheckFastSmiElements(Register map,
                                          Register scratch,
                                          Label* fail) { UNREACHABLE(); }


void MacroAssembler::StoreNumberToDoubleElements(Register value_reg,
                                                 Register key_reg,
                                                 Register elements_reg,
                                                 Register scratch1,
                                                 Register scratch2,
                                                 Register scratch3,
                                                 Register scratch4,
                                                 Label* fail,
                                                 int elements_offset) { UNREACHABLE(); }

void MacroAssembler::CheckMap(Register obj,
                              Register scratch,
                              Handle<Map> map,
                              Label* fail,
                              SmiCheckType smi_check_type) { UNREACHABLE(); }


void MacroAssembler::DispatchMap(Register obj,
                                 Register scratch,
                                 Handle<Map> map,
                                 Handle<Code> success,
                                 SmiCheckType smi_check_type) { UNREACHABLE(); }


void MacroAssembler::CheckMap(Register obj,
                              Register scratch,
                              Heap::RootListIndex index,
                              Label* fail,
                              SmiCheckType smi_check_type) { UNREACHABLE(); }



void MacroAssembler::SetCallKind(Register dst, CallKind call_kind) { UNREACHABLE(); }


// -----------------------------------------------------------------------------
// JavaScript invokes.

void MacroAssembler::InvokePrologue(const ParameterCount& expected,
                                    const ParameterCount& actual,
                                    Handle<Code> code_constant,
                                    Register code_reg,
                                    Label* done,
                                    bool* definitely_mismatches,
                                    InvokeFlag flag,
                                    const CallWrapper& call_wrapper,
                                    CallKind call_kind) { UNREACHABLE(); }


void MacroAssembler::InvokeCode(Register code,
                                const ParameterCount& expected,
                                const ParameterCount& actual,
                                InvokeFlag flag,
                                const CallWrapper& call_wrapper,
                                CallKind call_kind) { UNREACHABLE(); }


void MacroAssembler::InvokeCode(Handle<Code> code,
                                const ParameterCount& expected,
                                const ParameterCount& actual,
                                RelocInfo::Mode rmode,
                                InvokeFlag flag,
                                CallKind call_kind) { UNREACHABLE(); }


void MacroAssembler::InvokeFunction(Register function,
                                    const ParameterCount& actual,
                                    InvokeFlag flag,
                                    const CallWrapper& call_wrapper,
                                    CallKind call_kind) { UNREACHABLE(); }


void MacroAssembler::InvokeFunction(Handle<JSFunction> function,
                                    const ParameterCount& expected,
                                    const ParameterCount& actual,
                                    InvokeFlag flag,
                                    const CallWrapper& call_wrapper,
                                    CallKind call_kind) { UNREACHABLE(); }

void MacroAssembler::IsObjectNameType(Register object,
                                      Register scratch,
                                      Label* fail) { UNREACHABLE(); }


// ---------------------------------------------------------------------------
// Support functions.


void MacroAssembler::TryGetFunctionPrototype(Register function,
                                             Register result,
                                             Register scratch,
                                             Label* miss,
                                             bool miss_on_bound_function) { UNREACHABLE(); }


// -----------------------------------------------------------------------------
// Runtime calls.

void MacroAssembler::CallStub(CodeStub* stub,
                              TypeFeedbackId ast_id,
                              Condition cond,
                              Register r1,
                              const Operand& r2) { UNREACHABLE(); }


void MacroAssembler::TailCallStub(CodeStub* stub) { UNREACHABLE(); }


void MacroAssembler::CallApiFunctionAndReturn(ExternalReference function,
                                              int stack_space,
                                              bool returns_handle,
                                              int return_value_offset_from_fp) { UNREACHABLE(); }


bool MacroAssembler::AllowThisStubCall(CodeStub* stub) { UNREACHABLE(); return false;}


void MacroAssembler::IllegalOperation(int num_arguments) { UNREACHABLE(); }


void MacroAssembler::IndexFromHash(Register hash,
                                   Register index) { UNREACHABLE(); }

void MacroAssembler::CallRuntime(const Runtime::Function* f,
                                 int num_arguments) { UNREACHABLE(); }


void MacroAssembler::CallRuntimeSaveDoubles(Runtime::FunctionId id) { UNREACHABLE(); }


void MacroAssembler::CallRuntime(Runtime::FunctionId fid, int num_arguments) { UNREACHABLE(); }


void MacroAssembler::CallExternalReference(const ExternalReference& ext,
                                           int num_arguments) { UNREACHABLE(); }


void MacroAssembler::TailCallExternalReference(const ExternalReference& ext,
                                               int num_arguments,
                                               int result_size) { UNREACHABLE(); }


void MacroAssembler::TailCallRuntime(Runtime::FunctionId fid,
                                     int num_arguments,
                                     int result_size) { UNREACHABLE(); }


void MacroAssembler::JumpToExternalReference(const ExternalReference& builtin) { UNREACHABLE(); }


void MacroAssembler::InvokeBuiltin(Builtins::JavaScript id,
                                   InvokeFlag flag,
                                   const CallWrapper& call_wrapper) { UNREACHABLE(); }


void MacroAssembler::GetBuiltinFunction(Register target,
                                        Builtins::JavaScript id) { UNREACHABLE(); }


void MacroAssembler::GetBuiltinEntry(Register target, Builtins::JavaScript id) { UNREACHABLE(); }


void MacroAssembler::SetCounter(StatsCounter* counter, int value,
                                Register scratch1, Register scratch2) { UNREACHABLE(); }


void MacroAssembler::IncrementCounter(StatsCounter* counter, int value,
                                      Register scratch1, Register scratch2) { UNREACHABLE(); }


void MacroAssembler::DecrementCounter(StatsCounter* counter, int value,
                                      Register scratch1, Register scratch2) { UNREACHABLE(); }


// -----------------------------------------------------------------------------
// Debugging.

void MacroAssembler::Assert(Condition cc, const char* msg,
                            Register rs, Operand rt) { UNREACHABLE(); }


void MacroAssembler::AssertRegisterIsRoot(Register reg,
                                          Heap::RootListIndex index) { UNREACHABLE(); }


void MacroAssembler::AssertFastElements(Register elements) {

}


void MacroAssembler::Check(Condition cc, const char* msg,
                           Register rs, Operand rt) { UNREACHABLE(); }


void MacroAssembler::Abort(const char* msg) { UNREACHABLE(); }


void MacroAssembler::LoadContext(Register dst, int context_chain_length) { UNREACHABLE(); }


void MacroAssembler::LoadTransitionedArrayMapConditional(
    ElementsKind expected_kind,
    ElementsKind transitioned_kind,
    Register map_in_out,
    Register scratch,
    Label* no_map_match) { UNREACHABLE(); }


void MacroAssembler::LoadInitialArrayMap(
    Register function_in, Register scratch,
    Register map_out, bool can_have_holes) { UNREACHABLE(); }


void MacroAssembler::LoadGlobalFunction(int index, Register function) { UNREACHABLE(); }


void MacroAssembler::LoadArrayFunction(Register function) { UNREACHABLE(); }


void MacroAssembler::LoadGlobalFunctionInitialMap(Register function,
                                                  Register map,
                                                  Register scratch) { UNREACHABLE(); }


void MacroAssembler::EnterFrame(StackFrame::Type type) { UNREACHABLE(); }


void MacroAssembler::LeaveFrame(StackFrame::Type type) { UNREACHABLE(); }


void MacroAssembler::EnterExitFrame(bool save_doubles,
                                    int stack_space) { UNREACHABLE(); }


void MacroAssembler::LeaveExitFrame(bool save_doubles,
                                    Register argument_count,
                                    bool do_return) { UNREACHABLE(); }


int MacroAssembler::ActivationFrameAlignment() { UNREACHABLE(); return -1;}


void MacroAssembler::AssertStackIsAligned() { UNREACHABLE(); }

void MacroAssembler::UntagAndJumpIfSmi(Register dst,
                                       Register src,
                                       Label* smi_case) { UNREACHABLE(); }


void MacroAssembler::UntagAndJumpIfNotSmi(Register dst,
                                          Register src,
                                          Label* non_smi_case) { UNREACHABLE(); }

void MacroAssembler::JumpIfSmi(Register value,
                               Label* smi_label,
                               Register scratch) { UNREACHABLE(); }

void MacroAssembler::JumpIfNotSmi(Register value,
                                  Label* not_smi_label,
                                  Register scratch) { UNREACHABLE(); }


void MacroAssembler::JumpIfNotBothSmi(Register reg1,
                                      Register reg2,
                                      Label* on_not_both_smi) { UNREACHABLE(); }


void MacroAssembler::JumpIfEitherSmi(Register reg1,
                                     Register reg2,
                                     Label* on_either_smi) { UNREACHABLE(); }


void MacroAssembler::AssertNotSmi(Register object) { UNREACHABLE(); }


void MacroAssembler::AssertSmi(Register object) { UNREACHABLE(); }


void MacroAssembler::AssertString(Register object) { UNREACHABLE(); }


void MacroAssembler::AssertName(Register object) { UNREACHABLE(); }


void MacroAssembler::AssertRootValue(Register src,
                                     Heap::RootListIndex root_value_index,
                                     const char* message) { UNREACHABLE(); }


void MacroAssembler::JumpIfNotBothSequentialAsciiStrings(Register first,
                                                         Register second,
                                                         Register scratch1,
                                                         Register scratch2,
                                                         Label* failure) { UNREACHABLE(); }


void MacroAssembler::JumpIfBothInstanceTypesAreNotSequentialAscii(
    Register first,
    Register second,
    Register scratch1,
    Register scratch2,
    Label* failure) { UNREACHABLE(); }


void MacroAssembler::JumpIfInstanceTypeIsNotSequentialAscii(Register type,
                                                            Register scratch,
                                                            Label* failure) { UNREACHABLE(); }

void MacroAssembler::PrepareCallCFunction(int num_reg_arguments,
                                          int num_double_arguments,
                                          Register scratch) { UNREACHABLE(); }


void MacroAssembler::PrepareCallCFunction(int num_reg_arguments,
                                          Register scratch) { UNREACHABLE(); }


void MacroAssembler::CallCFunction(ExternalReference function,
                                   int num_reg_arguments,
                                   int num_double_arguments) { UNREACHABLE(); }


void MacroAssembler::CallCFunction(Register function,
                                   int num_reg_arguments,
                                   int num_double_arguments) { UNREACHABLE(); }


void MacroAssembler::CallCFunction(ExternalReference function,
                                   int num_arguments) { UNREACHABLE(); }


void MacroAssembler::CallCFunction(Register function,
                                   int num_arguments) { UNREACHABLE(); }


#undef BRANCH_ARGS_CHECK


void MacroAssembler::CheckPageFlag(
    Register object,
    Register scratch,
    int mask,
    Condition cc,
    Label* condition_met) { UNREACHABLE(); }


void MacroAssembler::CheckMapDeprecated(Handle<Map> map,
                                        Register scratch,
                                        Label* if_deprecated) { UNREACHABLE(); }


void MacroAssembler::JumpIfBlack(Register object,
                                 Register scratch0,
                                 Register scratch1,
                                 Label* on_black) { UNREACHABLE(); }


void MacroAssembler::HasColor(Register object,
                              Register bitmap_scratch,
                              Register mask_scratch,
                              Label* has_color,
                              int first_bit,
                              int second_bit) { UNREACHABLE(); }


// Detect some, but not all, common pointer-free objects.  This is used by the
// incremental write barrier which doesn't care about oddballs (they are always
// marked black immediately so this code is not hit).
void MacroAssembler::JumpIfDataObject(Register value,
                                      Register scratch,
                                      Label* not_data_object) { UNREACHABLE(); }


void MacroAssembler::GetMarkBits(Register addr_reg,
                                 Register bitmap_reg,
                                 Register mask_reg) { UNREACHABLE(); }


void MacroAssembler::EnsureNotWhite(
    Register value,
    Register bitmap_scratch,
    Register mask_scratch,
    Register load_scratch,
    Label* value_is_white_and_not_data) { UNREACHABLE(); }


void MacroAssembler::LoadInstanceDescriptors(Register map,
                                             Register descriptors) { UNREACHABLE(); }


void MacroAssembler::NumberOfOwnDescriptors(Register dst, Register map) { UNREACHABLE(); }


void MacroAssembler::EnumLength(Register dst, Register map) { UNREACHABLE(); }


void MacroAssembler::CheckEnumCache(Register null_value, Label* call_runtime) { UNREACHABLE(); }


void MacroAssembler::ClampUint8(Register output_reg, Register input_reg) { UNREACHABLE(); }


void MacroAssembler::ClampDoubleToUint8(Register result_reg,
                                        DoubleRegister input_reg,
                                        DoubleRegister temp_double_reg) { UNREACHABLE(); }


void MacroAssembler::TestJSArrayForAllocationSiteInfo(
    Register receiver_reg,
    Register scratch_reg,
    Condition cond,
    Label* allocation_info_present) { UNREACHABLE(); }


bool AreAliased(Register r1, Register r2, Register r3, Register r4) {
  if (r1.is(r2)) return true;
  if (r1.is(r3)) return true;
  if (r1.is(r4)) return true;
  if (r2.is(r3)) return true;
  if (r2.is(r4)) return true;
  if (r3.is(r4)) return true;
  return false;
}



CodePatcher::CodePatcher(byte* address, int instructions)
    : address_(address),
      size_(instructions * Assembler::kInstrSize),
      masm_(NULL, address, size_ + Assembler::kGap) {
  // Create a new macro assembler pointing to the address of the code to patch.
  // The size is adjusted with kGap on order for the assembler to generate size
  // bytes of instructions without failing with buffer size constraints.
  ASSERT(masm_.reloc_info_writer.pos() == address_ + size_ + Assembler::kGap);
}


CodePatcher::~CodePatcher() {
  // Indicate that code has changed.
  CPU::FlushICache(address_, size_);

  // Check that the code was patched as expected.
  ASSERT(masm_.pc_ == address_ + size_);
  ASSERT(masm_.reloc_info_writer.pos() == address_ + size_ + Assembler::kGap);
}


void CodePatcher::Emit(Instr instr) { UNREACHABLE(); }


void CodePatcher::Emit(Address addr) { UNREACHABLE(); }


void CodePatcher::ChangeBranchCondition(Condition cond) { UNREACHABLE(); }


} }  // namespace v8::internal

#endif  // V8_TARGET_ARCH_TILEGX
