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
#include <arch/opcode.h>
#include "assembler.h"
#include "constants-tilegx.h"
#include "serialize.h"

namespace v8 {
namespace internal {

/* Opcode Helper Macros */
#define TILEGX_X_MODE 0

#define FNOP_X0                                                                \
  create_Opcode_X0(RRR_0_OPCODE_X0) |                                          \
      create_RRROpcodeExtension_X0(UNARY_RRR_0_OPCODE_X0) |                    \
      create_UnaryOpcodeExtension_X0(FNOP_UNARY_OPCODE_X0)

#define FNOP_X1                                                                \
  create_Opcode_X1(RRR_0_OPCODE_X1) |                                          \
      create_RRROpcodeExtension_X1(UNARY_RRR_0_OPCODE_X1) |                    \
      create_UnaryOpcodeExtension_X1(FNOP_UNARY_OPCODE_X1)

#define NOP create_Mode(TILEGX_X_MODE) | FNOP_X0 | FNOP_X1

#define ADD_X1                                                                 \
  create_Mode(TILEGX_X_MODE) | create_Opcode_X1(RRR_0_OPCODE_X1) |             \
      create_RRROpcodeExtension_X1(ADD_RRR_0_OPCODE_X1) | FNOP_X0

#define SUB_X1                                                                 \
  create_Mode(TILEGX_X_MODE) | create_Opcode_X1(RRR_0_OPCODE_X1) |             \
      create_RRROpcodeExtension_X1(SUB_RRR_0_OPCODE_X1) | FNOP_X0

#define NOR_X1                                                                 \
  create_Mode(TILEGX_X_MODE) | create_Opcode_X1(RRR_0_OPCODE_X1) |             \
      create_RRROpcodeExtension_X1(NOR_RRR_0_OPCODE_X1) | FNOP_X0

#define OR_X1                                                                  \
  create_Mode(TILEGX_X_MODE) | create_Opcode_X1(RRR_0_OPCODE_X1) |             \
      create_RRROpcodeExtension_X1(OR_RRR_0_OPCODE_X1) | FNOP_X0

#define AND_X1                                                                 \
  create_Mode(TILEGX_X_MODE) | create_Opcode_X1(RRR_0_OPCODE_X1) |             \
      create_RRROpcodeExtension_X1(AND_RRR_0_OPCODE_X1) | FNOP_X0

#define XOR_X1                                                                 \
  create_Mode(TILEGX_X_MODE) | create_Opcode_X1(RRR_0_OPCODE_X1) |             \
      create_RRROpcodeExtension_X1(XOR_RRR_0_OPCODE_X1) | FNOP_X0

#define CMOVNEZ_X0                                                             \
  create_Mode(TILEGX_X_MODE) | create_Opcode_X0(RRR_0_OPCODE_X0) |             \
      create_RRROpcodeExtension_X0(CMOVNEZ_RRR_0_OPCODE_X0) | FNOP_X1

#define CMOVEQZ_X0                                                             \
  create_Mode(TILEGX_X_MODE) | create_Opcode_X0(RRR_0_OPCODE_X0) |             \
      create_RRROpcodeExtension_X0(CMOVEQZ_RRR_0_OPCODE_X0) | FNOP_X1

#define ADDLI_X1                                                               \
  create_Mode(TILEGX_X_MODE) | create_Opcode_X1(ADDLI_OPCODE_X1) | FNOP_X0

#define MOVELI_X1                                                              \
  create_Mode(TILEGX_X_MODE) | create_Opcode_X1(ADDLI_OPCODE_X1) |             \
	create_SrcA_X1(0x3F) | FNOP_X0

#define ADDI_X1                                                                \
  create_Mode(TILEGX_X_MODE) | create_Opcode_X1(IMM8_OPCODE_X1) |              \
      create_Imm8OpcodeExtension_X1(ADDI_IMM8_OPCODE_X1) | FNOP_X0

#define V4INT_L_X1                                                             \
  create_Mode(TILEGX_X_MODE) | create_Opcode_X1(RRR_0_OPCODE_X1) |             \
      create_RRROpcodeExtension_X1(V4INT_L_RRR_0_OPCODE_X1) | FNOP_X0

#define BFEXTU_X0                                                              \
  create_Mode(TILEGX_X_MODE) | create_Opcode_X0(BF_OPCODE_X0) |                \
      create_BFOpcodeExtension_X0(BFEXTU_BF_OPCODE_X0) | FNOP_X1

#define BFEXTS_X0                                                              \
  create_Mode(TILEGX_X_MODE) | create_Opcode_X0(BF_OPCODE_X0) |                \
      create_BFOpcodeExtension_X0(BFEXTS_BF_OPCODE_X0) | FNOP_X1

#define SHL16INSLI_X1                                                          \
  create_Mode(TILEGX_X_MODE) | create_Opcode_X1(SHL16INSLI_OPCODE_X1) | FNOP_X0

#define ST_X1                                                                  \
  create_Mode(TILEGX_X_MODE) | create_Opcode_X1(RRR_0_OPCODE_X1) |             \
      create_RRROpcodeExtension_X1(ST_RRR_0_OPCODE_X1) | create_Dest_X1(0x0) | \
      FNOP_X0

#define LD_X1                                                                  \
  create_Mode(TILEGX_X_MODE) | create_Opcode_X1(RRR_0_OPCODE_X1) |             \
      create_RRROpcodeExtension_X1(UNARY_RRR_0_OPCODE_X1) |                    \
      create_UnaryOpcodeExtension_X1(LD_UNARY_OPCODE_X1) | FNOP_X0

#define JR_X1                                                                  \
  create_Mode(TILEGX_X_MODE) | create_Opcode_X1(RRR_0_OPCODE_X1) |             \
      create_RRROpcodeExtension_X1(UNARY_RRR_0_OPCODE_X1) |                    \
      create_UnaryOpcodeExtension_X1(JR_UNARY_OPCODE_X1) | FNOP_X0

#define JALR_X1                                                                \
  create_Mode(TILEGX_X_MODE) | create_Opcode_X1(RRR_0_OPCODE_X1) |             \
      create_RRROpcodeExtension_X1(UNARY_RRR_0_OPCODE_X1) |                    \
      create_UnaryOpcodeExtension_X1(JALR_UNARY_OPCODE_X1) | FNOP_X0

#define CLZ_X0                                                                 \
  create_Mode(TILEGX_X_MODE) | create_Opcode_X0(RRR_0_OPCODE_X0) |             \
      create_RRROpcodeExtension_X0(UNARY_RRR_0_OPCODE_X0) |                    \
      create_UnaryOpcodeExtension_X0(CNTLZ_UNARY_OPCODE_X0) | FNOP_X1

#define CMPEQ_X1                                                               \
  create_Mode(TILEGX_X_MODE) | create_Opcode_X1(RRR_0_OPCODE_X1) |             \
      create_RRROpcodeExtension_X1(CMPEQ_RRR_0_OPCODE_X1) |                    \
      FNOP_X0

#define CMPNE_X1                                                               \
  create_Mode(TILEGX_X_MODE) | create_Opcode_X1(RRR_0_OPCODE_X1) |             \
      create_RRROpcodeExtension_X1(CMPNE_RRR_0_OPCODE_X1) |                    \
      FNOP_X0

#define CMPLTS_X1                                                              \
  create_Mode(TILEGX_X_MODE) | create_Opcode_X1(RRR_0_OPCODE_X1) |             \
      create_RRROpcodeExtension_X1(CMPLTS_RRR_0_OPCODE_X1) |                   \
      FNOP_X0

#define CMPLES_X1                                                              \
  create_Mode(TILEGX_X_MODE) | create_Opcode_X1(RRR_0_OPCODE_X1) |             \
      create_RRROpcodeExtension_X1(CMPLES_RRR_0_OPCODE_X1) |                   \
      FNOP_X0

#define CMPLTU_X1                                                              \
  create_Mode(TILEGX_X_MODE) | create_Opcode_X1(RRR_0_OPCODE_X1) |             \
      create_RRROpcodeExtension_X1(CMPLTU_RRR_0_OPCODE_X1) |                   \
      FNOP_X0

#define CMPLEU_X1                                                              \
  create_Mode(TILEGX_X_MODE) | create_Opcode_X1(RRR_0_OPCODE_X1) |             \
      create_RRROpcodeExtension_X1(CMPLEU_RRR_0_OPCODE_X1) |                   \
      FNOP_X0

#define CMPLTSI_X1                                                             \
  create_Mode(TILEGX_X_MODE) | create_Opcode_X1(IMM8_OPCODE_X1) |              \
      create_Imm8OpcodeExtension_X1(CMPLTSI_IMM8_OPCODE_X1) |                  \
      FNOP_X0

#define CMPLTUI_X1                                                             \
  create_Mode(TILEGX_X_MODE) | create_Opcode_X1(IMM8_OPCODE_X1) |              \
      create_Imm8OpcodeExtension_X1(CMPLTUI_IMM8_OPCODE_X1) |                  \
      FNOP_X0

#define XORI_X1                                                                \
  create_Mode(TILEGX_X_MODE) | create_Opcode_X1(IMM8_OPCODE_X1) |              \
      create_Imm8OpcodeExtension_X1(XORI_IMM8_OPCODE_X1) |                     \
      FNOP_X0

#define ORI_X1                                                                 \
  create_Mode(TILEGX_X_MODE) | create_Opcode_X1(IMM8_OPCODE_X1) |              \
      create_Imm8OpcodeExtension_X1(ORI_IMM8_OPCODE_X1) |                      \
      FNOP_X0

#define ANDI_X1                                                                \
  create_Mode(TILEGX_X_MODE) | create_Opcode_X1(IMM8_OPCODE_X1) |              \
      create_Imm8OpcodeExtension_X1(ANDI_IMM8_OPCODE_X1) |                     \
      FNOP_X0

#define SHLI_X1                                                                \
  create_Mode(TILEGX_X_MODE) | create_Opcode_X1(SHIFT_OPCODE_X1) |             \
      create_ShiftOpcodeExtension_X1(SHLI_SHIFT_OPCODE_X1) | FNOP_X0

#define SHL_X1                                                                 \
  create_Mode(TILEGX_X_MODE) | create_Opcode_X1(RRR_0_OPCODE_X1) |             \
      create_RRROpcodeExtension_X1(SHL_RRR_0_OPCODE_X1) | FNOP_X0

#define SHRSI_X1                                                               \
  create_Mode(TILEGX_X_MODE) | create_Opcode_X1(SHIFT_OPCODE_X1) |             \
      create_ShiftOpcodeExtension_X1(SHRSI_SHIFT_OPCODE_X1) | FNOP_X0

#define SHRS_X1                                                                \
  create_Mode(TILEGX_X_MODE) | create_Opcode_X1(RRR_0_OPCODE_X1) |             \
      create_RRROpcodeExtension_X1(SHRS_RRR_0_OPCODE_X1) | FNOP_X0

#define SHRUI_X1                                                               \
  create_Mode(TILEGX_X_MODE) | create_Opcode_X1(SHIFT_OPCODE_X1) |             \
      create_ShiftOpcodeExtension_X1(SHRUI_SHIFT_OPCODE_X1) | FNOP_X0

#define SHRU_X1                                                                \
  create_Mode(TILEGX_X_MODE) | create_Opcode_X1(RRR_0_OPCODE_X1) |             \
      create_RRROpcodeExtension_X1(SHRU_RRR_0_OPCODE_X1) | FNOP_X0

#define BEQZ_X1                                                                \
  create_Mode(TILEGX_X_MODE) | create_Opcode_X1(BRANCH_OPCODE_X1) |            \
      create_BrType_X1(BEQZ_BRANCH_OPCODE_X1) | FNOP_X0

#define BNEZ_X1                                                                \
  create_Mode(TILEGX_X_MODE) | create_Opcode_X1(BRANCH_OPCODE_X1) |            \
      create_BrType_X1(BNEZ_BRANCH_OPCODE_X1) | FNOP_X0

#define BGTZ_X1                                                                \
  create_Mode(TILEGX_X_MODE) | create_Opcode_X1(BRANCH_OPCODE_X1) |            \
      create_BrType_X1(BGTZ_BRANCH_OPCODE_X1) | FNOP_X0

#define BGEZ_X1                                                                \
  create_Mode(TILEGX_X_MODE) | create_Opcode_X1(BRANCH_OPCODE_X1) |            \
      create_BrType_X1(BGEZ_BRANCH_OPCODE_X1) | FNOP_X0

#define BLTZ_X1                                                                \
  create_Mode(TILEGX_X_MODE) | create_Opcode_X1(BRANCH_OPCODE_X1) |            \
      create_BrType_X1(BLTZ_BRANCH_OPCODE_X1) | FNOP_X0

#define BLEZ_X1                                                                \
  create_Mode(TILEGX_X_MODE) | create_Opcode_X1(BRANCH_OPCODE_X1) |            \
      create_BrType_X1(BLEZ_BRANCH_OPCODE_X1) | FNOP_X0

#define J_X1                                                                   \
  create_Mode(TILEGX_X_MODE) | create_Opcode_X1(JUMP_OPCODE_X1) |              \
      create_JumpOpcodeExtension_X1(J_JUMP_OPCODE_X1) | FNOP_X0

#define JAL_X1                                                                 \
  create_Mode(TILEGX_X_MODE) | create_Opcode_X1(JUMP_OPCODE_X1) |              \
      create_JumpOpcodeExtension_X1(JAL_JUMP_OPCODE_X1) | FNOP_X0

#define DEST_X0(x) create_Dest_X0(x)
#define SRCA_X0(x) create_SrcA_X0(x)
#define SRCB_X0(x) create_SrcB_X0(x)
#define DEST_X1(x) create_Dest_X1(x)
#define SRCA_X1(x) create_SrcA_X1(x)
#define SRCB_X1(x) create_SrcB_X1(x)
#define IMM16_X1(x) create_Imm16_X1(x)
#define IMM8_X1(x) create_Imm8_X1(x)
#define BFSTART_X0(x) create_BFStart_X0(x)
#define BFEND_X0(x) create_BFEnd_X0(x)
#define SHIFTIMM_X1(x) create_ShAmt_X1(x)
#define JOFF_X1(x) create_JumpOff_X1(x)
#define BOFF_X1(x) create_BrOff_X1(x)

// TileGX has 64 64bit registers.
struct Register {
  static const int kNumRegisters = 64;
  // For Float Register
  static const int kMaxNumRegisters = 64;
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

#define REGISTER(N, C) \
  const int kRegister_ ## N ## _Code = C; \
  const Register N = { C }

REGISTER(r0, 0);
REGISTER(r1, 1);
REGISTER(r2, 2);
REGISTER(r3, 3);
REGISTER(r4, 4);
REGISTER(r5, 5);
REGISTER(r6, 6);
REGISTER(r7, 7);
REGISTER(r8, 8);
REGISTER(r9, 9);
REGISTER(r10, 10);
REGISTER(r11, 11);
REGISTER(r12, 12);
REGISTER(r13, 13);
REGISTER(r14, 14);
REGISTER(r15, 15);
REGISTER(r16, 16);
REGISTER(r17, 17);
REGISTER(r18, 18);
REGISTER(r19, 19);
REGISTER(r20, 20);
REGISTER(r21, 21);
REGISTER(r22, 22);
REGISTER(r23, 23);
REGISTER(r24, 24);
REGISTER(r25, 25);
REGISTER(r26, 26);
REGISTER(r27, 27);
REGISTER(r28, 28);
REGISTER(r29, 29);
REGISTER(r30, 30);
REGISTER(r31, 31);
REGISTER(r32, 32);
REGISTER(r33, 33);
REGISTER(r34, 34);
REGISTER(r35, 35);
REGISTER(r36, 36);
REGISTER(r37, 37);
REGISTER(r38, 38);
REGISTER(r39, 39);
REGISTER(r40, 40);
REGISTER(r41, 41);
REGISTER(r42, 42);
REGISTER(r43, 43);
REGISTER(r44, 44);
REGISTER(r45, 45);
REGISTER(r46, 46);
REGISTER(r47, 47);
REGISTER(r48, 48);
REGISTER(r49, 49);
REGISTER(r50, 50);
REGISTER(r51, 51);
REGISTER(r52, 52);
REGISTER(r53, 53);
REGISTER(r54, 54);
REGISTER(r55, 55);
REGISTER(r56, 56);
REGISTER(r57, 57);
REGISTER(r58, 58);
REGISTER(r59, 59);
REGISTER(r60, 60);
REGISTER(r61, 61);
REGISTER(r62, 62);
REGISTER(r63, 63);

#undef REGISTER


const int kRegister_t0_Code = 25;
const int kRegister_t1_Code = 26;
const int kRegister_t2_Code = 27;
const int kRegister_t3_Code = 28;
const int kRegister_t4_Code = 29;

const int kRegister_tt_Code = 49;
const int kRegister_pc_Code = 50;
const int kRegister_gp_Code = 51;
const int kRegister_fp_Code = 52;
const int kRegister_tp_Code = 53;
const int kRegister_sp_Code = 54;
const int kRegister_lr_Code = 55;
const int kRegister_zero_Code = 55;
const int kRegister_no_reg_Code = -1;

const Register t0  = { kRegister_t0_Code };
const Register t1  = { kRegister_t1_Code };
const Register t2  = { kRegister_t2_Code };
const Register t3  = { kRegister_t3_Code };
const Register t4  = { kRegister_t4_Code };
// 'tt' is treated as TileGX temp register,
// like 'at' in MIPS.
const Register tt  = { kRegister_tt_Code };
const Register pc  = { kRegister_pc_Code };
const Register gp  = { kRegister_gp_Code };
const Register fp  = { kRegister_fp_Code };
const Register tp  = { kRegister_tp_Code };
const Register sp  = { kRegister_sp_Code };
const Register lr  = { kRegister_lr_Code };
const Register zero = { kRegister_zero_Code };
const Register no_reg = { kRegister_no_reg_Code };

// Register aliases.
// cp is assumed to be a callee saved register.
// Defined using #define instead of "static const Register&" because Clang
// complains otherwise when a compilation unit that includes this header
// doesn't use the variables.
#define kRootRegister r45
#define cp r46
#define kLithiumScratchReg r47
#define kLithiumScratchReg2 r48

int ToNumber(Register reg);

Register ToRegister(int num);

// Class Operand represents a shifter operand in data processing instructions.
class Operand BASE_EMBEDDED {
 public:

  // Immediate.
  INLINE(explicit Operand(int64_t immediate,
         RelocInfo::Mode rmode = RelocInfo::NONE64));
  INLINE(explicit Operand(const ExternalReference& f));
  INLINE(explicit Operand(const char* s));
  INLINE(explicit Operand(Object** opp));
  INLINE(explicit Operand(Context** cpp));
  explicit Operand(Handle<Object> handle);
  INLINE(explicit Operand(Smi* value));

  // Register.
  INLINE(explicit Operand(Register rm));

  // Return true if this is a register operand.
  INLINE(bool is_reg() const);

  Register rm() const { return rm_; }

 private:
  Register rm_;
  int64_t imm64_;  // Valid if rm_ == no_reg.
  RelocInfo::Mode rmode_;

  friend class Assembler;
  friend class MacroAssembler;
};

// TileGX reuse Integer reg as float reg.
typedef Register DoubleRegister;

class MemOperand : public Operand {
 public:
  explicit MemOperand(Register rn, int64_t offset = 0);
  int32_t offset() const { return offset_; }

 private:
  int64_t offset_;

  friend class Assembler;
};

class Assembler : public AssemblerBase {
 public:
  // Create an assembler. Instructions and relocation information are emitted
  // into a buffer, with the instructions starting from the beginning and the
  // relocation information starting from the end of the buffer. See CodeDesc
  // for a detailed comment on the layout (globals.h).
  //
  // If the provided buffer is NULL, the assembler allocates and grows its own
  // buffer, and buffer_size determines the initial buffer size. The buffer is
  // owned by the assembler and deallocated upon destruction of the assembler.
  //
  // If the provided buffer is not NULL, the assembler uses the provided buffer
  // for code generation and assumes its size to be buffer_size. If the buffer
  // is too small, a fatal error occurs. No deallocation of the buffer is done
  // upon destruction of the assembler.
  Assembler(Isolate* isolate, void* buffer, int buffer_size);
  virtual ~Assembler() { }

  // Different nop operations are used by the code generator to detect certain
  // states of the generated code.
  enum NopMarkerTypes {
    NON_MARKING_NOP = 0,
    DEBUG_BREAK_NOP,
    // IC markers.
    PROPERTY_ACCESS_INLINED,
    PROPERTY_ACCESS_INLINED_CONTEXT,
    PROPERTY_ACCESS_INLINED_CONTEXT_DONT_DELETE,
    // Helper values.
    LAST_CODE_MARKER,
    FIRST_IC_MARKER = PROPERTY_ACCESS_INLINED,
    // Code aging
    CODE_AGE_MARKER_NOP = 6
  };

  // Insert the smallest number of nop instructions
  // possible to align the pc offset to a multiple
  // of m. m must be a power of 2 (>= 8).
  void Align(int m);

  // Record a comment relocation entry that can be used by a disassembler.
  // Use --code-comments to enable.
  void RecordComment(const char* msg);
  static int RelocateInternalReference(byte* pc, intptr_t pc_delta);

  // Writes a single byte or word of data in the code stream.  Used for
  // inline tables, e.g., jump-tables.
  void db(uint8_t data);
  void dd(uint32_t data);

  // GetCode emits any pending (non-emitted) code and fills the descriptor
  // desc. GetCode() is idempotent; it returns the same result if no other
  // Assembler functions are invoked in between GetCode() calls.
  void GetCode(CodeDesc* desc);

  void bind(Label* L);  // Binds an unbound label L to current code position.
  void bind_to(Label* L, int pos);
  void next(Label* L);
  void print(Label* L);

  // Returns the branch offset to the given label from the current code
  // position. Links the label to the current position if it is still unbound.
  // Manages the jump elimination optimization if the second parameter is true.
  int32_t branch_offset(Label* L, bool jump_elimination_allowed);
  int32_t shifted_branch_offset(Label* L, bool jump_elimination_allowed) {
    int32_t o = branch_offset(L, jump_elimination_allowed);
    ASSERT((o & 7) == 0);   // Assert the offset is aligned.
    return o >> 3;
  }

  uint32_t jump_address(Label* L);

  static const int kInstrSize = sizeof(Instr);

  static Instr instr_at(byte* pc) { return *reinterpret_cast<Instr*>(pc); }
  static void instr_at_put(byte* pc, Instr instr) {
    *reinterpret_cast<Instr*>(pc) = instr;
  }
  Instr instr_at(int pos) { return *reinterpret_cast<Instr*>(buffer_ + pos); }
  void instr_at_put(int pos, Instr instr) {
    *reinterpret_cast<Instr*>(buffer_ + pos) = instr;
  }

  static bool IsBranch(Instr instr);
  static bool IsJ(Instr instr);
  static bool IsJR(Instr instr);
  static bool IsJAL(Instr instr);
  static bool IsMOVELI(Instr instr);
  static bool IsSHL16INSLI(Instr instr);

  static uint32_t GetLabelConst(Instr instr);
  static bool IsEmittedConstant(Instr instr);

  // Return the code target address at a call site from the return address
  // of that call in the instruction stream.
  inline static Address target_address_from_return_address(Address pc);

  // Read/Modify the code target address in the branch/call instruction at pc.
  static Address target_address_at(Address pc);
  static void set_target_address_at(Address pc, Address target);

  // Here we are patching the address in the LUI/ORI instruction pair.
  // These values are used in the serialization process and must be zero for
  // MIPS platform, as Code, Embedded Object or External-reference pointers
  // are split across two consecutive instructions and don't exist separately
  // in the code, so the serializer should not step forwards in memory after
  // a target is resolved and written.
  static const int kSpecialTargetSize = 0;

  // Number of consecutive instructions used to store 32bit constant.
  // Before jump-optimizations, this constant was used in
  // RelocInfo::target_address_address() function to tell serializer address of
  // the instruction that follows LUI/ORI instruction pair. Now, with new jump
  // optimization, where jump-through-register instruction that usually
  // follows LUI/ORI pair is substituted with J/JAL, this constant equals
  // to 3 instructions (LUI+ORI+J/JAL/JR/JALR).
  static const int kInstructionsFor32BitConstant = 3;

  // This sets the branch destination (which gets loaded at the call address).
  // This is for calls and branches within generated code.  The serializer
  // has already deserialized the lui/ori instructions etc.
  inline static void deserialization_set_special_target_at(
      Address instruction_payload, Address target) {
    set_target_address_at(
        instruction_payload - kInstructionsFor32BitConstant * kInstrSize,
        target);
  }

  PositionsRecorder* positions_recorder() { return &positions_recorder_; }

  static const int kMaxRelocSize = RelocInfoWriter::kMaxSize;
  RelocInfoWriter reloc_info_writer;

  static const int kPatchReturnSequenceAddressOffset = 0;

  // Distance between start of patched debug break slot and the emitted address
  // to jump to.
  static const int kPatchDebugBreakSlotAddressOffset =  0 * kInstrSize;

  // Difference between address of current opcode and value read from pc
  // register.
  static const int kPcLoadDelta = 4;

  static const int kPatchDebugBreakSlotReturnOffset = 4 * kInstrSize;

  // Number of instructions used for the JS return sequence. The constant is
  // used by the debugger to patch the JS return sequence.
  static const int kJSReturnSequenceInstructions = 7;
  static const int kDebugBreakSlotInstructions = 4;
  static const int kDebugBreakSlotLength =
      kDebugBreakSlotInstructions * kInstrSize;

  // ---------------------------------------------------------------------------
  // Instruction Encoding
  
  void j(int64_t target);
  void jr(Register target);

  void b(int32_t offset);
  void b(Label* L) { b(branch_offset(L, false)>>3); }
  void beqz(const Register& rs, int32_t offset);
  void beqz(const Register& rs, Label* L) {
    beqz(rs, branch_offset(L, false) >> 3);
  }
  void bnez(const Register& rs, int32_t offset);
  void bnez(const Register& rs, Label* L) {
    bnez(rs, branch_offset(L, false)>>3);
  }
  void bgez(const Register& rs, int32_t offset);
  void bgtz(const Register& rs, int32_t offset);
  void blez(const Register& rs, int32_t offset);
  void bltz(const Register& rs, int32_t offset);

  void cmpeq(const Register& rd, const Register& rsa, const Register& rsb);
  void cmpne(const Register& rd, const Register& rsa, const Register& rsb);
  void cmplts(const Register& rd, const Register& rsa, const Register& rsb);
  void cmpltsi(const Register& rd, const Register& rsa, int8_t imm);
  void cmples(const Register& rd, const Register& rsa, const Register& rsb);
  void cmpltu(const Register& rd, const Register& rsa, const Register& rsb);
  void cmpltui(const Register& rd, const Register& rsa, int8_t imm);
  void cmpleu(const Register& rd, const Register& rsa, const Register& rsb);

  void st(const Register& rd, const MemOperand& rs);
  void st(const Register& rd, const Register& rs);
  void ld(const Register& rd, const MemOperand& rs);
  void ld(const Register& rd, const Register& rs);
  void add(const Register& rd, const Register& rsa, const Register& rsb);
  void sub(const Register& rd, const Register& rsa, const Register& rsb);
  void addi(const Register& rd, const Register& rs, int8_t imm);
  void addli(const Register& rd, const Register& rs, int16_t imm);
  void moveli(const Register& rd, int16_t imm);
  void shl16insli(const Register& rd, const Register& rs, int16_t imm);
  void move(const Register& rt, const Register& rs);

  // Check if an instruction is a branch of some kind.
  static bool IsNop(Instr instr, unsigned int type);

  bool is_near(Label* L);

 protected:

  // Record reloc info for current pc_.
  void RecordRelocInfo(RelocInfo::Mode rmode, intptr_t data = 0);

  int64_t buffer_space() const { return reloc_info_writer.pos() - pc_; }

  // Decode branch instruction at pos and return branch target pos.
  int target_at(int32_t pos);

  // Patch branch instruction at pos to branch to given branch target pos.
  void target_at_put(int32_t pos, int32_t target_pos);

  // Say if we need to relocate with this mode.
  bool MustUseReg(RelocInfo::Mode rmode);

 private:
  // Code generation
  // The relocation writer's position is at least kGap bytes below the end of
  // the generated instructions. This is so that multi-instruction sequences do
  // not have to check for overflow. The same is true for writes of large
  // relocation info entries.
  static const int kGap = 32;

  inline void CheckBuffer();
  void GrowBuffer();
  inline void emit(Instr x);

  PositionsRecorder positions_recorder_;

  int next_buffer_check_;  // pc offset of next buffer check.

  // Emission of the trampoline pool may be blocked in some code sequences.
  int trampoline_pool_blocked_nesting_;  // Block emission if this is not zero.
  int no_trampoline_pool_before_;  // Block emission before this pc offset.

  // Keep track of the last emitted pool to guarantee a maximal distance.
  int last_trampoline_pool_end_;  // pc offset of the end of the last pool.

  // Automatic growth of the assembly buffer may be blocked for some sequences.
  bool block_buffer_growth_;  // Block growth when true.

  // The bound position, before this we cannot do instruction elimination.
  int last_bound_pos_;

  // One trampoline consists of:
  // - space for trampoline slots,
  // - space for labels.
  //
  // Space for trampoline slots is equal to slot_count * 2 * kInstrSize.
  // Space for trampoline slots preceeds space for labels. Each label is of one
  // instruction size, so total amount for labels is equal to
  // label_count *  kInstrSize.
  class Trampoline {
   public:
    Trampoline() {
      start_ = 0;
      next_slot_ = 0;
      free_slot_count_ = 0;
      end_ = 0;
    }
    Trampoline(int start, int slot_count) {
      start_ = start;
      next_slot_ = start;
      free_slot_count_ = slot_count;
      end_ = start + slot_count * kTrampolineSlotsSize;
    }
    int start() {
      return start_;
    }
    int end() {
      return end_;
    }
    int take_slot() {
      int trampoline_slot = kInvalidSlotPos;
      if (free_slot_count_ <= 0) {
        // We have run out of space on trampolines.
        // Make sure we fail in debug mode, so we become aware of each case
        // when this happens.
        ASSERT(0);
        // Internal exception will be caught.
      } else {
        trampoline_slot = next_slot_;
        free_slot_count_--;
        next_slot_ += kTrampolineSlotsSize;
      }
      return trampoline_slot;
    }

   private:
    int start_;
    int end_;
    int next_slot_;
    int free_slot_count_;
  };

  int32_t get_trampoline_entry(int32_t pos);
  int unbound_labels_count_;
  // If trampoline is emitted, generated code is becoming large. As this is
  // already a slow case which can possibly break our code generation for the
  // extreme case, we use this information to trigger different mode of
  // branch instruction generation, where we use jump instructions rather
  // than regular branch instructions.
  bool trampoline_emitted_;
  static const int kTrampolineSlotsSize = 4 * kInstrSize;
  static const int kMaxBranchOffset = (1 << 18) - 1;
  static const int kInvalidSlotPos = -1;

  Trampoline trampoline_;
  bool internal_trampoline_exception_;


  friend class EnsureSpace;
  friend class CodePatcher;
  friend class PositionsRecorder;
};

class EnsureSpace BASE_EMBEDDED {
 public:
  explicit EnsureSpace(Assembler* assembler) {
    assembler->CheckBuffer();
  }
};

// CpuFeatures keeps track of which features are supported by the target CPU.
// Supported features must be enabled by a CpuFeatureScope before use.
class CpuFeatures : public AllStatic {
 public:
  // Detect features of the target CPU. Set safe defaults if the serializer
  // is enabled (snapshots must be portable).
  static void Probe();

  // Check whether a feature is supported by the target CPU.
  static bool IsSupported(CpuFeature f) {
    ASSERT(initialized_);
    return (supported_ & (1u << f)) != 0;
  }

  static bool IsFoundByRuntimeProbingOnly(CpuFeature f) {
    ASSERT(initialized_);
    return (found_by_runtime_probing_only_ &
            (static_cast<uint64_t>(1) << f)) != 0;
  }

  static bool IsSafeForSnapshot(CpuFeature f) {
    return (IsSupported(f) &&
            (!Serializer::enabled() || !IsFoundByRuntimeProbingOnly(f)));
  }

 private:
#ifdef DEBUG
  static bool initialized_;
#endif
  static unsigned supported_;
  static unsigned found_by_runtime_probing_only_;

  friend class ExternalReference;
  DISALLOW_COPY_AND_ASSIGN(CpuFeatures);
};


} }  // namespace v8::internal

#endif  // V8_TILEGX_ASSEMBLER_TILEGX_H_
