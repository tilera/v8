// Copyright (c) 1994-2006 Sun Microsystems Inc.
// All Rights Reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met:
//
// - Redistributions of source code must retain the above copyright notice,
// this list of conditions and the following disclaimer.
//
// - Redistribution in binary form must reproduce the above copyright
// notice, this list of conditions and the following disclaimer in the
// documentation and/or other materials provided with the distribution.
//
// - Neither the name of Sun Microsystems or the names of contributors may
// be used to endorse or promote products derived from this software without
// specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
// IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
// THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
// PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
// CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
// EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
// PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
// PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
// LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
// NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

// The original source code covered by the above license above has been
// modified significantly by Google Inc.
// Copyright 2012 the V8 project authors. All rights reserved.


#ifndef V8_TILEGX_ASSEMBLER_TILEGX_H_
#define V8_TILEGX_ASSEMBLER_TILEGX_H_

#include <stdio.h>
#include "assembler.h"
#include "constants-tilegx.h"
#include "serialize.h"

namespace v8 {
namespace internal {

// TileGX has 64 64bit registers.
struct Register {
  static const int kNumRegisters = 64;
  static const int kMaxNumAllocatableRegisters = 50;
  static const int kSizeInBytes = 8;

  inline static int NumAllocatableRegisters();

  static int ToAllocationIndex(Register reg) {
    ASSERT(reg.code() < kMaxNumAllocatableRegisters);
    return reg.code();
  }

  static Register FromAllocationIndex(int index) {
    ASSERT(index >= 0 && index < kMaxNumAllocatableRegisters);
    return from_code(index);
  }

  static const char* AllocationIndexToString(int index) {
    ASSERT(index >= 0 && index < kMaxNumAllocatableRegisters);
    const char* const names[] = {
      "r0",
      "r1",
      "r2",
      "r3",
      "r4",
      "r5",
      "r6",
      "r7",
      "r8",
      "r9",
      "r10",
      "r11",
      "r12",
      "r13",
      "r14",
      "r15",
      "r16",
      "r17",
      "r18",
      "r19",
      "r20",
      "r21",
      "r22",
      "r23",
      "r24",
      "r25",
      "r26",
      "r27",
      "r28",
      "r29",
      "r30",
      "r31",
      "r32",
      "r33",
      "r34",
      "r35",
      "r36",
      "r37",
      "r38",
      "r39",
      "r40",
      "r41",
      "r42",
      "r43",
      "r44",
      "r45",
      "r46",
      "r47",
      "r48",
      "r49",
    };
    return names[index];
  }

  static Register from_code(int code) {
    Register r = { code };
    return r;
  }

  bool is_valid() const { return 0 <= code_ && code_ < kNumRegisters; }
  bool is(Register reg) const { return code_ == reg.code_; }

  int code() const {
    ASSERT(is_valid());
    return code_;
  }

  int bit() const {
    ASSERT(is_valid());
    return 1 << code_;
  }

  void set_code(int code) {
    code_ = code;
    ASSERT(is_valid());
  }

  // Unfortunately we can't make this private in a struct.
  int code_;
};

const int kRegister_pc_Code = 50;
const int kRegister_gp_Code = 51;
const int kRegister_fp_Code = 52;
const int kRegister_tp_Code = 53;
const int kRegister_sp_Code = 54;
const int kRegister_lr_Code = 55;
const int kRegister_no_reg_Code = -1;

const Register pc  = { kRegister_pc_Code };
const Register gp  = { kRegister_gp_Code };
const Register fp  = { kRegister_fp_Code };
const Register tp  = { kRegister_tp_Code };
const Register sp  = { kRegister_sp_Code };
const Register lr  = { kRegister_lr_Code };
const Register no_reg = { kRegister_no_reg_Code };

} }  // namespace v8::internal

#endif  // V8_TILEGX_ASSEMBLER_TILEGX_H_
