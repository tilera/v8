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

const int kRegister_pc_Code = 50;
const int kRegister_gp_Code = 51;
const int kRegister_fp_Code = 52;
const int kRegister_tp_Code = 53;
const int kRegister_sp_Code = 54;
const int kRegister_lr_Code = 55;
const int kRegister_zero_Code = 55;
const int kRegister_no_reg_Code = -1;

const Register pc  = { kRegister_pc_Code };
const Register gp  = { kRegister_gp_Code };
const Register fp  = { kRegister_fp_Code };
const Register tp  = { kRegister_tp_Code };
const Register sp  = { kRegister_sp_Code };
const Register lr  = { kRegister_lr_Code };
const Register zero_reg = { kRegister_zero_Code };
const Register no_reg = { kRegister_no_reg_Code };

// Register aliases.
// cp is assumed to be a callee saved register.
// Defined using #define instead of "static const Register&" because Clang
// complains otherwise when a compilation unit that includes this header
// doesn't use the variables.
#define kRootRegister r0
#define cp r1
#define kLithiumScratchReg r2
#define kLithiumScratchReg2 r3

// Class Operand represents a shifter operand in data processing instructions.
class Operand BASE_EMBEDDED {
 public:

  // Register.
  INLINE(explicit Operand(Register rm));

  // Return true if this is a register operand.
  INLINE(bool is_reg() const);

  Register rm() const { return rm_; }

 private:
  Register rm_;
};

// TileGX reuse Integer reg as float reg.
typedef Register DoubleRegister;

class MemOperand : public Operand {
 public:
  explicit MemOperand(Register rn);

 private:
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

  static const int kInstrSize = sizeof(Instr);

  static Instr instr_at(byte* pc) { return *reinterpret_cast<Instr*>(pc); }
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

  int64_t buffer_space() const { return reloc_info_writer.pos() - pc_; }


  PositionsRecorder* positions_recorder() { return &positions_recorder_; }

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
  
  void st(Register rd, Register rs);
  void ld(Register rd, Register rs);
  void addi(Register rd, Register rs, int8_t imm);
  void move(Register rt, Register rs);

  // Check if an instruction is a branch of some kind.
  static bool IsNop(Instr instr, unsigned int type);

 protected:

  // Record reloc info for current pc_.
  void RecordRelocInfo(RelocInfo::Mode rmode, intptr_t data = 0);

 private:
  // Code generation
  // The relocation writer's position is at least kGap bytes below the end of
  // the generated instructions. This is so that multi-instruction sequences do
  // not have to check for overflow. The same is true for writes of large
  // relocation info entries.
  static const int kGap = 32;

  inline void CheckBuffer();
  void GrowBuffer();

  PositionsRecorder positions_recorder_;

  friend class EnsureSpace;
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
