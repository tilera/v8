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

void Builtins::Generate_Adaptor(MacroAssembler* masm,
                                CFunctionId id,
                                BuiltinExtraArguments extra_args) {
#if 0
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
#else
  printf("[%s:%d]\n", __FUNCTION__, __LINE__);
  abort();
#endif
}

void Builtins::Generate_JSConstructStubCountdown(MacroAssembler* masm) {
#if 0
  Generate_JSConstructStubHelper(masm, false, true);
#else
  printf("[%s:%d]\n", __FUNCTION__, __LINE__);
  abort();
#endif
}


void Builtins::Generate_JSConstructStubGeneric(MacroAssembler* masm) {
#if 0
  Generate_JSConstructStubHelper(masm, false, false);
#else
  printf("[%s:%d]\n", __FUNCTION__, __LINE__);
  abort();
#endif
}


void Builtins::Generate_JSConstructStubApi(MacroAssembler* masm) {
#if 0
  Generate_JSConstructStubHelper(masm, true, false);
#else
  printf("[%s:%d]\n", __FUNCTION__, __LINE__);
  abort();
#endif
}

void Builtins::Generate_InRecompileQueue(MacroAssembler* masm) {
  printf("[%s:%d]\n", __FUNCTION__, __LINE__);
  abort();
}

void Builtins::Generate_InstallRecompiledCode(MacroAssembler* masm) {
  printf("[%s:%d]\n", __FUNCTION__, __LINE__);
  abort();
}

void Builtins::Generate_ArgumentsAdaptorTrampoline(MacroAssembler* masm) {
  printf("[%s:%d]\n", __FUNCTION__, __LINE__);
  abort();
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
