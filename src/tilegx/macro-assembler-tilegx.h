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

#ifndef V8_TILEGX_MACRO_ASSEMBLER_TILEGX_H_
#define V8_TILEGX_MACRO_ASSEMBLER_TILEGX_H_

#include "assembler.h"
#include "tilegx/assembler-tilegx.h"
#include "v8globals.h"

namespace v8 {
namespace internal {

// MacroAssembler implements a collection of frequently used macros.
class MacroAssembler: public Assembler {
 public:
  // The isolate parameter can be NULL if the macro assembler should
  // not use isolate-dependent functionality. In this case, it's the
  // responsibility of the caller to never invoke such function on the
  // macro assembler.
  MacroAssembler(Isolate* isolate, void* buffer, int size);

  // Activation support.
  void EnterFrame(StackFrame::Type type);
  void LeaveFrame(StackFrame::Type type);

  // Arguments macros.
#define COND_TYPED_ARGS Condition cond, Register r1, const Operand& r2

  // Cases when relocation is not needed.
#define DECLARE_NORELOC_PROTOTYPE(Name, target_type) \
  void Name(target_type target); \
  void Name(target_type target, \
            COND_TYPED_ARGS); \

#define DECLARE_BRANCH_PROTOTYPES(Name) \
  DECLARE_NORELOC_PROTOTYPE(Name, Label*) \
  DECLARE_NORELOC_PROTOTYPE(Name, int16_t)

  DECLARE_BRANCH_PROTOTYPES(Branch)
  DECLARE_BRANCH_PROTOTYPES(BranchAndLink)

#undef DECLARE_BRANCH_PROTOTYPES
#undef COND_TYPED_ARGS

  // Invoke specified builtin JavaScript function. Adds an entry to
  // the unresolved list if the name does not resolve.
  void InvokeBuiltin(Builtins::JavaScript id,
                     InvokeFlag flag,
                     const CallWrapper& call_wrapper = NullCallWrapper());

  // Store the function for the given builtin in the target register.
  void GetBuiltinFunction(Register target, Builtins::JavaScript id);

  // Store the code object for the given builtin in the target register and
  // setup the function in a1.
  void GetBuiltinEntry(Register target, Builtins::JavaScript id);

  Handle<Object> CodeObject() {
    ASSERT(!code_object_.is_null());
    return code_object_;
  }

  // Debug Purpose
  void set_generating_stub(bool value) { generating_stub_ = value; }
  bool generating_stub() { return generating_stub_; }
  void set_allow_stub_calls(bool value) { allow_stub_calls_ = value; }
  bool allow_stub_calls() { return allow_stub_calls_; }
  void set_has_frame(bool value) { has_frame_ = value; }
  bool has_frame() { return has_frame_; }

  // Jump unconditionally to given label.
  void jmp(Label* L) {
    Branch(L);
  }

  // Jump, Call, and Ret pseudo instructions implementing inter-working.
#define COND_ARGS Condition cond = al, Register rs = zero_reg, \
  const Operand& rt = Operand(zero_reg)

  void Jump(Register target, COND_ARGS);
  void Jump(intptr_t target, RelocInfo::Mode rmode, COND_ARGS);
  void Jump(Address target, RelocInfo::Mode rmode, COND_ARGS);
  void Jump(Handle<Code> code, RelocInfo::Mode rmode, COND_ARGS);
  static int CallSize(Register target, COND_ARGS);
  void Call(Register target, COND_ARGS);
  static int CallSize(Address target, RelocInfo::Mode rmode, COND_ARGS);
  void Call(Address target, RelocInfo::Mode rmode, COND_ARGS);
  int CallSize(Handle<Code> code,
               RelocInfo::Mode rmode = RelocInfo::CODE_TARGET,
               TypeFeedbackId ast_id = TypeFeedbackId::None(),
               COND_ARGS);
  void Call(Handle<Code> code,
            RelocInfo::Mode rmode = RelocInfo::CODE_TARGET,
            TypeFeedbackId ast_id = TypeFeedbackId::None(),
            COND_ARGS);
  void Ret(COND_ARGS);

  void Branch(Label* L,
              Condition cond,
              Register rs,
              Heap::RootListIndex index);

#undef COND_ARGS

  void Call(Label* target);

  inline void Move(Register dst, Register src) {
    if (!dst.is(src)) {
      move(dst, src);
    }
  }

  // Lower case push() for compatibility with arch-independent code.
  void push(Register src) {
    addi(sp, sp, -kPointerSize);
    st(sp, src);
  }

  void pop(Register dst) {
    ld(dst, sp);
    addi(sp, sp, kPointerSize);
  }

  // Push a handle.
  void Push(Handle<Object> handle);
  void Push(Smi* smi) { Push(Handle<Smi>(smi, isolate())); }

  // Push two registers. Pushes leftmost register first (to highest address).
  void Push(Register src1, Register src2) {
    addi(sp, sp, -kPointerSize);
    st(sp, src1);
    addi(sp, sp, -kPointerSize);
    st(sp, src2);
  }

  // Jump the register contains a smi.
  void JumpIfSmi(Register value,
                 Label* smi_label,
                 Register scratch = r0);

  // Check if the map of an object is equal to a specified map and branch to a
  // specified target if equal. Skip the smi check if not required (object is
  // known to be a heap object)
  void DispatchMap(Register obj,
                   Register scratch,
                   Handle<Map> map,
                   Handle<Code> success,
                   SmiCheckType smi_check_type);

  // Emit code to discard a non-negative number of pointer-sized elements
  // from the stack, clobbering only the sp register.
  void Drop(int count,
            Condition cond = cc_always,
            Register reg = no_reg,
            const Operand& op = Operand(no_reg));

  // Call a code stub.
  void CallStub(CodeStub* stub,
                TypeFeedbackId ast_id = TypeFeedbackId::None(),
                Condition cond = cc_always,
                Register r1 = zero_reg,
                const Operand& r2 = Operand(zero_reg));

  // Call a runtime routine.
  void CallRuntime(const Runtime::Function* f, int num_arguments);
  void CallRuntimeSaveDoubles(Runtime::FunctionId id);

  // Convenience function: Same as above, but takes the fid instead.
  void CallRuntime(Runtime::FunctionId fid, int num_arguments);

  // Tail call of a runtime routine (jump).
  // Like JumpToExternalReference, but also takes care of passing the number
  // of parameters.
  void TailCallExternalReference(const ExternalReference& ext,
                                 int num_arguments,
                                 int result_size);

  // Convenience function: tail call a runtime routine (jump).
  void TailCallRuntime(Runtime::FunctionId fid,
                       int num_arguments,
                       int result_size);

  // -------------------------------------------------------------------------
  // Exception handling.

  // Push a new try handler and link into try handler chain.
  void PushTryHandler(StackHandler::Kind kind, int handler_index);

  // Unlink the stack handler on top of the stack from the try handler chain.
  // Must preserve the result register.
  void PopTryHandler();

  // Passes thrown value to the handler of top of the try handler chain.
  void Throw(Register value);

#ifdef ENABLE_DEBUGGER_SUPPORT
  // -------------------------------------------------------------------------
  // Debugger Support.

  void DebugBreak();
#endif

 private:
  bool generating_stub_;
  bool allow_stub_calls_;
  bool has_frame_;

  // This handle will be patched with the code object on installation.
  Handle<Object> code_object_;

  static int SafepointRegisterStackIndex(int reg_code);

  // Needs access to SafepointRegisterStackIndex for compiled frame
  // traversal.
  friend class StandardFrame;
};

enum RememberedSetAction { EMIT_REMEMBERED_SET, OMIT_REMEMBERED_SET };
enum SmiCheck { INLINE_SMI_CHECK, OMIT_SMI_CHECK };
enum RAStatus { kRAHasNotBeenSaved, kRAHasBeenSaved };

#ifdef GENERATED_CODE_COVERAGE
#define CODE_COVERAGE_STRINGIFY(x) #x
#define CODE_COVERAGE_TOSTRING(x) CODE_COVERAGE_STRINGIFY(x)
#define __FILE_LINE__ __FILE__ ":" CODE_COVERAGE_TOSTRING(__LINE__)
#define ACCESS_MASM(masm) masm->stop(__FILE_LINE__); masm->
#else
#define ACCESS_MASM(masm) masm->
#endif

} }  // namespace v8::internal

#endif  // V8_TILEGX_MACRO_ASSEMBLER_TILEGX_H_
