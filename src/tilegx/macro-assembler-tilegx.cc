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
                              Heap::RootListIndex index) {
  ld(destination, MemOperand(kRootRegister, index << kPointerSizeLog2));
}


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
                                Label* branch) {
  ASSERT(cc == eq || cc == ne);
  And(scratch, object, Operand(ExternalReference::new_space_mask(isolate())));
  Branch(branch, cc, scratch,
         Operand(ExternalReference::new_space_start(isolate())));
}


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
                                         RememberedSetFinalAction and_then) {
  Label done;
  if (emit_debug_code()) {
    Label ok;
    JumpIfNotInNewSpace(object, scratch, &ok);
    stop("Remembered set pointer is in new space");
    bind(&ok);
  }
  // Load store buffer top.
  ExternalReference store_buffer =
      ExternalReference::store_buffer_top(isolate());
  li(r40, Operand(store_buffer));
  ld(scratch, MemOperand(r40));
  // Store pointer to buffer and increment buffer top.
  st(address, MemOperand(scratch));
  Addu(scratch, scratch, kPointerSize);
  // Write back new top of buffer.
  st(scratch, MemOperand(r40));
  // Call stub on end of buffer.
  // Check for end of buffer.
  And(r40, scratch, Operand(StoreBuffer::kStoreBufferOverflowBit));
  if (and_then == kFallThroughAtEnd) {
    Branch(&done, eq, r40, Operand(zero));
  } else {
    ASSERT(and_then == kReturnAtEnd);
    Ret(eq, r40, Operand(zero));
  }
  push(lr);
  StoreBufferOverflowStub store_buffer_overflow =
      StoreBufferOverflowStub(fp_mode);
  CallStub(&store_buffer_overflow);
  pop(lr);
  bind(&done);
  if (and_then == kReturnAtEnd) {
    Ret();
  }
}


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

void MacroAssembler::Addu(Register rd, Register rs, const Operand& rt) {
  if (rt.is_reg()) {
    add(rd, rs, rt.rm());
  } else {
    if (is_int16(rt.imm64_) && !MustUseReg(rt.rmode_)) {
      addli(rd, rs, rt.imm64_);
    } else {
      // li handles the relocation.
      ASSERT(!rs.is(tt));
      li(tt, rt);
      add(rd, rs, tt);
    }
  }
}

void MacroAssembler::Subu(Register rd, Register rs, const Operand& rt) 
{
  if (rt.is_reg()) {
    sub(rd, rs, rt.rm());
  } else {
    if (is_int16(rt.imm64_) && !MustUseReg(rt.rmode_)) {
      addli(rd, rs, -rt.imm64_);
    } else {
      // li handles the relocation.
      ASSERT(!rs.is(tt));
      li(tt, rt);
      sub(rd, rs, tt);
    }
  }
}

void MacroAssembler::Mul(Register rd, Register rs, const Operand& rt) { UNREACHABLE(); }


void MacroAssembler::Mult(Register rs, const Operand& rt) { UNREACHABLE(); }


void MacroAssembler::Multu(Register rs, const Operand& rt) { UNREACHABLE(); }


void MacroAssembler::Div(Register rs, const Operand& rt) { UNREACHABLE(); }


void MacroAssembler::Divu(Register rs, const Operand& rt) { UNREACHABLE(); }


void MacroAssembler::And(Register rd, Register rs, const Operand& rt) {
  if (rt.is_reg()) {
    and_(rd, rs, rt.rm());
  } else {
    if (is_int8(rt.imm64_) && !MustUseReg(rt.rmode_)) {
      andi(rd, rs, rt.imm64_);
    } else {
      // li handles the relocation.
      ASSERT(!rs.is(tt));
      li(tt, rt);
      and_(rd, rs, tt);
    }
  }
}


void MacroAssembler::Or(Register rd, Register rs, const Operand& rt) {
  if (rt.is_reg()) {
    or_(rd, rs, rt.rm());
  } else {
    if (is_uint8(rt.imm64_) && !MustUseReg(rt.rmode_)) {
      ori(rd, rs, rt.imm64_);
    } else {
      // li handles the relocation.
      ASSERT(!rs.is(tt));
      li(tt, rt);
      or_(rd, rs, tt);
    }
  }
}


void MacroAssembler::Xor(Register rd, Register rs, const Operand& rt) {
  if (rt.is_reg()) {
    xor_(rd, rs, rt.rm());
  } else {
    if (is_uint8(rt.imm64_) && !MustUseReg(rt.rmode_)) {
      xori(rd, rs, rt.imm64_);
    } else {
      // li handles the relocation.
      ASSERT(!rs.is(tt));
      li(tt, rt);
      xor_(rd, rs, tt);
    }
  }
}


void MacroAssembler::Nor(Register rd, Register rs, const Operand& rt) { UNREACHABLE(); }


void MacroAssembler::Neg(Register rs, const Operand& rt) { UNREACHABLE(); }


void MacroAssembler::Slt(Register rd, Register rs, const Operand& rt) { UNREACHABLE(); }


void MacroAssembler::Sltu(Register rd, Register rs, const Operand& rt) { UNREACHABLE(); }


void MacroAssembler::Ror(Register rd, Register rs, const Operand& rt) { UNREACHABLE(); }

//------------Pseudo-instructions-------------

void MacroAssembler::li(Register rd, Operand j, int line, LiFlags mode) {
  ASSERT(!j.is_reg());
  if (MustUseReg(j.rmode_)) {
    RecordRelocInfo(j.rmode_, j.imm64_);
  }

  if (j.rmode_ == RelocInfo::EMBEDDED_OBJECT) {
    moveli(rd, (j.imm64_ >> 32) & 0xFFFF, line);
    shl16insli(rd, rd, (j.imm64_ >> 16) & 0xFFFF, line);
    shl16insli(rd, rd, j.imm64_ & 0xFFFF, line);
    return;
  }

  ASSERT(is_lintn(j.imm64_, 48));
  moveli(rd, (j.imm64_ >> 32) & 0xFFFF, line);
  shl16insli(rd, rd, (j.imm64_ >> 16) & 0xFFFF, line);
  shl16insli(rd, rd, j.imm64_ & 0xFFFF, line);
}


void MacroAssembler::MultiPush(RegList regs) {
  int16_t num_to_push = NumberOfBitsSet(regs);
  int16_t stack_offset = num_to_push * kPointerSize;

  Subu(sp, sp, Operand(stack_offset));
  for (int16_t i = kNumRegisters - 1; i >= 0; i--) {
    if ((regs & (1 << i)) != 0) {
      stack_offset -= kPointerSize;
      st(ToRegister(i), MemOperand(sp, stack_offset));
    }
  }
}


void MacroAssembler::MultiPushReversed(RegList regs) {
  int16_t num_to_push = NumberOfBitsSet(regs);
  int16_t stack_offset = num_to_push * kPointerSize;

  Subu(sp, sp, Operand(stack_offset));
  for (int16_t i = 0; i < kNumRegisters; i++) {
    if ((regs & (1 << i)) != 0) {
      stack_offset -= kPointerSize;
      st(ToRegister(i), MemOperand(sp, stack_offset));
    }
  }
}


void MacroAssembler::MultiPop(RegList regs) {
  int16_t stack_offset = 0;

  for (int16_t i = 0; i < kNumRegisters; i++) {
    if ((regs & (1L << i)) != 0) {
      ld(ToRegister(i), MemOperand(sp, stack_offset));
      stack_offset += kPointerSize;
    }
  }
  addi(sp, sp, stack_offset);
}

void MacroAssembler::MultiPopReversed(RegList regs) { UNREACHABLE(); }


void MacroAssembler::MultiPushFPU(RegList regs) { UNREACHABLE(); }


void MacroAssembler::MultiPushReversedFPU(RegList regs) { UNREACHABLE(); }


void MacroAssembler::MultiPopFPU(RegList regs) { UNREACHABLE(); }


void MacroAssembler::MultiPopReversedFPU(RegList regs) { UNREACHABLE(); }


void MacroAssembler::FlushICache(Register address, unsigned instructions) { UNREACHABLE(); }

void MacroAssembler::Jr(Label* L) {

  uint64_t imm64 = jump_address(L);
  {
    // Buffer growth (and relocation) must be blocked for internal references
    // until associated instructions are emitted and available to be patched.
    RecordRelocInfo(RelocInfo::INTERNAL_REFERENCE);
    moveli(tt, (imm64 >> 32) & 0xFFFF);
    shl16insli(tt, tt, (imm64 >> 16) & 0xFFFF);
    shl16insli(tt, tt, imm64 & 0xFFFF);
  }
  jr(tt);
}

// Emulated condtional branches do not emit a nop in the branch delay slot.
//
// BRANCH_ARGS_CHECK checks that conditional jump arguments are correct.
#define BRANCH_ARGS_CHECK(cond, rs, rt) ASSERT(                        \
    (cond == cc_always && rs.is(zero) && rt.rm().is(zero)) ||          \
    (cond != cc_always && (!rs.is(zero) || !rt.rm().is(zero))))


void MacroAssembler::Branch(int16_t offset) {
  b(offset);
}


void MacroAssembler::Branch(int16_t offset, Condition cond, Register rs,
                            const Operand& rt) {
  BranchShort(offset, cond, rs, rt);
}


void MacroAssembler::Branch(Label* L) {
  if (L->is_bound()) {
    if (is_near(L)) {
      BranchShort(L);
    } else {
      Jr(L);
    }
  } 
  
  BranchShort(L);
}

void MacroAssembler::Branch(Label* L, Condition cond, Register rs,
                            const Operand& rt) {
  if (L->is_bound()) {
    if (is_near(L)) {
      BranchShort(L, cond, rs, rt);
    } else {
      Label skip;
      Condition neg_cond = NegateCondition(cond);
      BranchShort(&skip, neg_cond, rs, rt);
      Jr(L);
      bind(&skip);
    }
  } else
    BranchShort(L, cond, rs, rt);
}

void MacroAssembler::BranchShort(int16_t offset) {
  b(offset);
}

void MacroAssembler::BranchShort(Label* L) {
  // We use branch_offset as an argument for the branch instructions to be sure
  // it is called just before generating the branch instruction, as needed.
  b(shifted_branch_offset(L, false));
}

void MacroAssembler::BranchShort(int16_t offset, Condition cond, Register rs,
                                 const Operand& rt) {
  BRANCH_ARGS_CHECK(cond, rs, rt);
  ASSERT(!rs.is(zero));
  Register r2 = no_reg;
  Register scratch = tt;

  if (rt.is_reg()) {
    // NOTE: 'at' can be clobbered by Branch but it is legal to use it as rs or
    // rt.
    BlockTrampolinePoolScope block_trampoline_pool(this);
    r2 = rt.rm_;
    switch (cond) {
      case cc_always:
        b(offset);
        break;
      case eq:
        if (r2.is(zero)) {
          beqz(rs, offset);
        } else {
          cmpeq(scratch, r2, rs);
          bnez(scratch, offset);
        }
        break;
      case ne:
        if (r2.is(zero)) {
          bnez(rs, offset);
        } else {
          cmpne(scratch, r2, rs);
          bnez(scratch, offset);
        }
        break;
      // Signed comparison.
      case greater:
        if (r2.is(zero)) {
          bgtz(rs, offset);
        } else {
          cmplts(scratch, r2, rs);
          bnez(scratch, offset);
        }
        break;
      case greater_equal:
        if (r2.is(zero)) {
          bgez(rs, offset);
        } else {
          cmplts(scratch, rs, r2);
          beqz(scratch, offset);
        }
        break;
      case less:
        if (r2.is(zero)) {
          bltz(rs, offset);
        } else {
          cmplts(scratch, rs, r2);
          bnez(scratch, offset);
        }
        break;
      case less_equal:
        if (r2.is(zero)) {
          blez(rs, offset);
        } else {
          cmplts(scratch, r2, rs);
          beqz(scratch, offset);
        }
        break;
      // Unsigned comparison.
      case Ugreater:
        if (r2.is(zero)) {
          bgtz(rs, offset);
        } else {
          cmpltu(scratch, r2, rs);
          bnez(scratch, offset);
        }
        break;
      case Ugreater_equal:
        if (r2.is(zero)) {
          bgez(rs, offset);
        } else {
          cmpltu(scratch, rs, r2);
          beqz(scratch, offset);
        }
        break;
      case Uless:
        if (r2.is(zero)) {
          // No code needs to be emitted.
          return;
        } else {
          cmpltu(scratch, rs, r2);
          bnez(scratch, offset);
        }
        break;
      case Uless_equal:
        if (r2.is(zero)) {
          b(offset);
        } else {
          cmpltu(scratch, r2, rs);
          beqz(scratch, offset);
        }
        break;
      default:
        UNREACHABLE();
    }
  } else {
    // Be careful to always use shifted_branch_offset only just before the
    // branch instruction, as the location will be remember for patching the
    // target.
    BlockTrampolinePoolScope block_trampoline_pool(this);
    switch (cond) {
      case cc_always:
        b(offset);
        break;
      case eq:
        // We don't want any other register but scratch clobbered.
        if (rt.imm64_ == 0) {
          beqz(rs, offset);
        } else {
          ASSERT(!scratch.is(rs));
          r2 = scratch;
          li(r2, rt);
	  cmpeq(scratch, rs, r2);
          bnez(scratch, offset);
        }
        break;
      case ne:
        // We don't want any other register but scratch clobbered.
        if (rt.imm64_ == 0) {
          bnez(rs, offset);
        } else {
          ASSERT(!scratch.is(rs));
          r2 = scratch;
          li(r2, rt);
	  cmpne(scratch, rs, r2);
          bnez(scratch, offset);
        }
        break;
      // Signed comparison.
      case greater:
        if (rt.imm64_ == 0) {
          bgtz(rs, offset);
        } else {
          r2 = scratch;
          li(r2, rt);
          cmplts(scratch, r2, rs);
          bnez(scratch, offset);
        }
        break;
      case greater_equal:
        if (rt.imm64_ == 0) {
          bgez(rs, offset);
        } else if (is_int16(rt.imm64_)) {
          cmpltsi(scratch, rs, rt.imm64_);
          beqz(scratch, offset);
        } else {
          r2 = scratch;
          li(r2, rt);
          cmplts(scratch, rs, r2);
          beqz(scratch, offset);
        }
        break;
      case less:
        if (rt.imm64_ == 0) {
          bltz(rs, offset);
        } else if (is_int16(rt.imm64_)) {
          cmpltsi(scratch, rs, rt.imm64_);
          bnez(scratch, offset);
        } else {
          r2 = scratch;
          li(r2, rt);
          cmplts(scratch, rs, r2);
          bnez(scratch, offset);
        }
        break;
      case less_equal:
        if (rt.imm64_ == 0) {
          blez(rs, offset);
        } else {
          r2 = scratch;
          li(r2, rt);
          cmplts(scratch, r2, rs);
          beqz(scratch, offset);
       }
       break;
      // Unsigned comparison.
      case Ugreater:
        if (rt.imm64_ == 0) {
          bgtz(rs, offset);
        } else {
          r2 = scratch;
          li(r2, rt);
          cmpltu(scratch, r2, rs);
          bnez(scratch, offset);
        }
        break;
      case Ugreater_equal:
        if (rt.imm64_ == 0) {
          bgez(rs, offset);
        } else if (is_int16(rt.imm64_)) {
          cmpltui(scratch, rs, rt.imm64_);
          beqz(scratch, offset);
        } else {
          r2 = scratch;
          li(r2, rt);
          cmpltu(scratch, rs, r2);
          beqz(scratch, offset);
        }
        break;
      case Uless:
        if (rt.imm64_ == 0) {
          // No code needs to be emitted.
          return;
        } else if (is_int16(rt.imm64_)) {
          cmpltui(scratch, rs, rt.imm64_);
          bnez(scratch, offset);
        } else {
          r2 = scratch;
          li(r2, rt);
          cmpltu(scratch, rs, r2);
          bnez(scratch, offset);
        }
        break;
      case Uless_equal:
        if (rt.imm64_ == 0) {
          b(offset);
        } else {
          r2 = scratch;
          li(r2, rt);
          cmpltu(scratch, r2, rs);
          beqz(scratch, offset);
        }
        break;
      default:
        UNREACHABLE();
    }
  }
}

void MacroAssembler::BranchShort(Label* L, Condition cond, Register rs,
                                 const Operand& rt) {
  BRANCH_ARGS_CHECK(cond, rs, rt);

  int32_t offset = 0;
  Register r2 = no_reg;
  Register scratch = tt;
  if (rt.is_reg()) {
    r2 = rt.rm_;
    // Be careful to always use shifted_branch_offset only just before the
    // branch instruction, as the location will be remember for patching the
    // target.
    switch (cond) {
      case cc_always:
        offset = shifted_branch_offset(L, false);
        b(offset);
        break;
      case eq:
        if (r2.is(zero)) {
          offset = shifted_branch_offset(L, false);
          beqz(rs, offset);
	} else {
          cmpeq(scratch, r2, rs);
          offset = shifted_branch_offset(L, false);
          bnez(scratch, offset);
	}
        break;
      case ne:
        if (r2.is(zero)) {
          offset = shifted_branch_offset(L, false);
          bnez(rs, offset);
	} else {
          cmpne(scratch, r2, rs);
          offset = shifted_branch_offset(L, false);
          bnez(scratch, offset);
	}
        break;
      // Signed comparison.
      case greater:
        if (r2.is(zero)) {
          offset = shifted_branch_offset(L, false);
          bgtz(rs, offset);
        } else {
          cmplts(scratch, r2, rs);
          offset = shifted_branch_offset(L, false);
          bnez(scratch, offset);
        }
        break;
      case greater_equal:
        if (r2.is(zero)) {
          offset = shifted_branch_offset(L, false);
          bgez(rs, offset);
        } else {
          cmplts(scratch, rs, r2);
          offset = shifted_branch_offset(L, false);
          beqz(scratch, offset);
        }
        break;
      case less:
        if (r2.is(zero)) {
          offset = shifted_branch_offset(L, false);
          bltz(rs, offset);
        } else {
          cmplts(scratch, rs, r2);
          offset = shifted_branch_offset(L, false);
          bnez(scratch, offset);
        }
        break;
      case less_equal:
        if (r2.is(zero)) {
          offset = shifted_branch_offset(L, false);
          blez(rs, offset);
        } else {
          cmplts(scratch, r2, rs);
          offset = shifted_branch_offset(L, false);
          beqz(scratch, offset);
        }
        break;
      // Unsigned comparison.
      case Ugreater:
        if (r2.is(zero)) {
          offset = shifted_branch_offset(L, false);
          bgtz(rs, offset);
        } else {
          cmpltu(scratch, r2, rs);
          offset = shifted_branch_offset(L, false);
          bnez(scratch, offset);
        }
        break;
      case Ugreater_equal:
        if (r2.is(zero)) {
          offset = shifted_branch_offset(L, false);
          bgez(rs, offset);
        } else {
          cmpltu(scratch, rs, r2);
          offset = shifted_branch_offset(L, false);
          beqz(scratch, offset);
        }
        break;
      case Uless:
        if (r2.is(zero)) {
          // No code needs to be emitted.
          return;
        } else {
          cmpltu(scratch, rs, r2);
          offset = shifted_branch_offset(L, false);
          bnez(scratch, offset);
        }
        break;
      case Uless_equal:
        if (r2.is(zero)) {
          offset = shifted_branch_offset(L, false);
          b(offset);
        } else {
          cmpltu(scratch, r2, rs);
          offset = shifted_branch_offset(L, false);
          beqz(scratch, offset);
        }
        break;
      default:
        UNREACHABLE();
    }
  } else {
    // Be careful to always use shifted_branch_offset only just before the
    // branch instruction, as the location will be remember for patching the
    // target.
    switch (cond) {
      case cc_always:
        offset = shifted_branch_offset(L, false);
        b(offset);
        break;
      case eq:
        if (rt.imm64_ == 0) {
          offset = shifted_branch_offset(L, false);
          beqz(rs, offset);
	} else {
          ASSERT(!scratch.is(rs));
          r2 = scratch;
          li(r2, rt);
	  cmpeq(scratch, rs, r2);
          offset = shifted_branch_offset(L, false);
          bnez(scratch, offset);
	}
        break;
      case ne:
        if (rt.imm64_ == 0) {
          offset = shifted_branch_offset(L, false);
          bnez(rs, offset);
	} else {
          ASSERT(!scratch.is(rs));
          r2 = scratch;
          li(r2, rt);
	  cmpne(scratch, rs, r2);
          offset = shifted_branch_offset(L, false);
          bnez(scratch, offset);
	}
        break;
      // Signed comparison.
      case greater:
        if (rt.imm64_ == 0) {
          offset = shifted_branch_offset(L, false);
          bgtz(rs, offset);
        } else {
          ASSERT(!scratch.is(rs));
          r2 = scratch;
          li(r2, rt);
          cmplts(scratch, r2, rs);
          offset = shifted_branch_offset(L, false);
          bnez(scratch, offset);
        }
        break;
      case greater_equal:
        if (rt.imm64_ == 0) {
          offset = shifted_branch_offset(L, false);
          bgez(rs, offset);
        } else if (is_int8(rt.imm64_)) {
          cmpltsi(scratch, rs, rt.imm64_);
          offset = shifted_branch_offset(L, false);
          beqz(scratch, offset);
        } else {
          ASSERT(!scratch.is(rs));
          r2 = scratch;
          li(r2, rt);
          cmplts(scratch, rs, r2);
          offset = shifted_branch_offset(L, false);
          beqz(scratch, offset);
        }
        break;
      case less:
        if (rt.imm64_ == 0) {
          offset = shifted_branch_offset(L, false);
          bltz(rs, offset);
        } else if (is_int8(rt.imm64_)) {
          cmpltsi(scratch, rs, rt.imm64_);
          offset = shifted_branch_offset(L, false);
          bnez(scratch, offset);
        } else {
          ASSERT(!scratch.is(rs));
          r2 = scratch;
          li(r2, rt);
          cmplts(scratch, rs, r2);
          offset = shifted_branch_offset(L, false);
          bnez(scratch, offset);
        }
        break;
      case less_equal:
        if (rt.imm64_ == 0) {
          offset = shifted_branch_offset(L, false);
          blez(rs, offset);
        } else {
          ASSERT(!scratch.is(rs));
          r2 = scratch;
          li(r2, rt);
          cmplts(scratch, r2, rs);
          offset = shifted_branch_offset(L, false);
          beqz(scratch, offset);
        }
        break;
      // Unsigned comparison.
      case Ugreater:
        if (rt.imm64_ == 0) {
          offset = shifted_branch_offset(L, false);
          bgtz(rs, offset);
        } else {
          ASSERT(!scratch.is(rs));
          r2 = scratch;
          li(r2, rt);
          cmpltu(scratch, r2, rs);
          offset = shifted_branch_offset(L, false);
          bnez(scratch, offset);
        }
        break;
      case Ugreater_equal:
        if (rt.imm64_ == 0) {
          offset = shifted_branch_offset(L, false);
          bgez(rs, offset);
        } else if (is_int8(rt.imm64_)) {
          cmpltui(scratch, rs, rt.imm64_);
          offset = shifted_branch_offset(L, false);
          beqz(scratch, offset);
        } else {
          ASSERT(!scratch.is(rs));
          r2 = scratch;
          li(r2, rt);
          cmpltu(scratch, rs, r2);
          offset = shifted_branch_offset(L, false);
          beqz(scratch, offset);
        }
        break;
     case Uless:
        if (rt.imm64_ == 0) {
          // No code needs to be emitted.
          return;
        } else if (is_int8(rt.imm64_)) {
          cmpltui(scratch, rs, rt.imm64_);
          offset = shifted_branch_offset(L, false);
          bnez(scratch, offset);
        } else {
          ASSERT(!scratch.is(rs));
          r2 = scratch;
          li(r2, rt);
          cmpltu(scratch, rs, r2);
          offset = shifted_branch_offset(L, false);
          bnez(scratch, offset);
        }
        break;
      case Uless_equal:
        if (rt.imm64_ == 0) {
          offset = shifted_branch_offset(L, false);
          b(offset);
        } else {
          ASSERT(!scratch.is(rs));
          r2 = scratch;
          li(r2, rt);
          cmpltu(scratch, r2, rs);
          offset = shifted_branch_offset(L, false);
          beqz(scratch, offset);
        }
        break;
      default:
        UNREACHABLE();
    }
  }
  // Check that offset could actually hold on an int16_t.
  ASSERT(is_intn(offset, 17));
}

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
                          const Operand& rt) {
  if (cond == cc_always) {
    jr(target);
  } else {
    BRANCH_ARGS_CHECK(cond, rs, rt);
    Branch(2, NegateCondition(cond), rs, rt);
    jr(target);
  }
}


void MacroAssembler::Jump(intptr_t target,
                          RelocInfo::Mode rmode,
                          Condition cond,
                          Register rs,
                          const Operand& rt) {
  Label skip;
  if (cond != cc_always) {
    Branch(&skip, NegateCondition(cond), rs, rt);
  }
  // The first instruction of 'li' may be placed in the delay slot.
  // This is not an issue, t9 is expected to be clobbered anyway.
  li(t9, Operand(target, rmode));
  Jump(t9, al, zero, Operand(zero));
  bind(&skip);
}


void MacroAssembler::Jump(Address target,
                          RelocInfo::Mode rmode,
                          Condition cond,
                          Register rs,
                          const Operand& rt) {
  ASSERT(!RelocInfo::IsCodeTarget(rmode));
  Jump(reinterpret_cast<intptr_t>(target), rmode, cond, rs, rt);
}


void MacroAssembler::Jump(Handle<Code> code,
                          RelocInfo::Mode rmode,
                          Condition cond,
                          Register rs,
                          const Operand& rt) {
  ASSERT(RelocInfo::IsCodeTarget(rmode));
  ALLOW_HANDLE_DEREF(isolate(), "embedding raw address");
  Jump(reinterpret_cast<intptr_t>(code.location()), rmode, cond, rs, rt);
}


int MacroAssembler::CallSize(Register target,
                             Condition cond,
                             Register rs,
                             const Operand& rt) {
  int size = 0;

  if (cond == cc_always) {
    size += 1;
  } else {
    size += 2;
  }

  return size * kInstrSize;
}


void MacroAssembler::Call(Register target,
                          Condition cond,
                          Register rs,
                          const Operand& rt) {
  BlockTrampolinePoolScope block_trampoline_pool(this);
  Label start;
  bind(&start);
  if (cond == cc_always) {
    jalr(target);
  } else {
    BRANCH_ARGS_CHECK(cond, rs, rt);
    Branch(2, NegateCondition(cond), rs, rt);
    jalr(target);
  }

  ASSERT_EQ(CallSize(target, cond, rs, rt),
            SizeOfCodeGeneratedSince(&start));
}

int MacroAssembler::CallSize(Address target,
                             RelocInfo::Mode rmode,
                             Condition cond,
                             Register rs,
                             const Operand& rt) {
  int size = CallSize(tt, cond, rs, rt);
  return size + 3 * kInstrSize;
} 

void MacroAssembler::Call(Address target,
                          RelocInfo::Mode rmode,
                          Condition cond,
                          Register rs,
                          const Operand& rt) {
  BlockTrampolinePoolScope block_trampoline_pool(this);
  Label start;
  bind(&start);
  int64_t target_int = reinterpret_cast<int64_t>(target);
  // Must record previous source positions before the
  // li() generates a new code target.
  positions_recorder()->WriteRecordedPositions();
  li(t0, Operand(target_int, rmode), CONSTANT_SIZE);
  Call(t0, cond, rs, rt);
  ASSERT_EQ(CallSize(target, rmode, cond, rs, rt),
            SizeOfCodeGeneratedSince(&start));
}

int MacroAssembler::CallSize(Handle<Code> code,
                             RelocInfo::Mode rmode,
                             TypeFeedbackId ast_id,
                             Condition cond,
                             Register rs,
                             const Operand& rt) {
  ALLOW_HANDLE_DEREF(isolate(), "using raw address");
  return CallSize(reinterpret_cast<Address>(code.location()),
      rmode, cond, rs, rt);
}


void MacroAssembler::Call(Handle<Code> code,
                          RelocInfo::Mode rmode,
                          TypeFeedbackId ast_id,
                          Condition cond,
                          Register rs,
                          const Operand& rt) {
  BlockTrampolinePoolScope block_trampoline_pool(this);
  Label start;
  bind(&start);
  ASSERT(RelocInfo::IsCodeTarget(rmode));
  if (rmode == RelocInfo::CODE_TARGET && !ast_id.IsNone()) {
    SetRecordedAstId(ast_id);
    rmode = RelocInfo::CODE_TARGET_WITH_ID;
  }
  ALLOW_HANDLE_DEREF(isolate(), "embedding raw address");
  Call(reinterpret_cast<Address>(code.location()), rmode, cond, rs, rt);
  ASSERT_EQ(CallSize(code, rmode, ast_id, cond, rs, rt),
            SizeOfCodeGeneratedSince(&start));
}


void MacroAssembler::Ret(Condition cond,
                         Register rs,
                         const Operand& rt) {
  Jump(lr, cond, rs, rt);
}


void MacroAssembler::DropAndRet(int drop) {
  Ret();
  addli(sp, sp, drop * kPointerSize);
}

void MacroAssembler::DropAndRet(int drop,
                                Condition cond,
                                Register r1,
                                const Operand& r2) {
  // Both Drop and Ret need to be conditional.
  Label skip;
  if (cond != cc_always) {
    Branch(&skip, NegateCondition(cond), r1, r2);
  }

  Drop(drop);
  Ret();

  if (cond != cc_always) {
    bind(&skip);
  }
}

void MacroAssembler::Drop(int count,
                          Condition cond,
                          Register reg,
                          const Operand& op) {
  if (count <= 0) {
    return;
  }

  Label skip;

  if (cond != al) {
     Branch(&skip, NegateCondition(cond), reg, op);
  }

  addli(sp, sp, count * kPointerSize);

  if (cond != al) {
    bind(&skip);
  }
}

void MacroAssembler::Swap(Register reg1,
                          Register reg2,
                          Register scratch) {
  if (scratch.is(no_reg)) {
    Xor(reg1, reg1, Operand(reg2));
    Xor(reg2, reg2, Operand(reg1));
    Xor(reg1, reg1, Operand(reg2));
  } else {
    move(scratch, reg1);
    move(reg1, reg2);
    move(reg2, scratch);
  }
}

void MacroAssembler::Call(Label* target) {
  BranchAndLink(target);
}

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
                                    int handler_index) {
  // For the JSEntry handler, we must preserve r0-r4
  // t1-t3 are available. We will build up the handler from the bottom by
  // pushing on the stack.
  // Set up the code object (t1) and the state (t2) for pushing.
  unsigned state =
      StackHandler::IndexField::encode(handler_index) |
      StackHandler::KindField::encode(kind);
  li(t1, Operand(CodeObject()), __LINE__);
  li(t2, Operand(state), __LINE__);

  // Push the frame pointer, context, state, and code object.
  if (kind == StackHandler::JS_ENTRY) {
    ASSERT_EQ(Smi::FromInt(0), 0);
    // The second zero indicates no context.
    // The first zero is the NULL frame pointer.
    // The operands are reversed to match the order of MultiPush/Pop.
    Push(zero, zero, t2, t1);
  } else {
    MultiPush(t1.bit() | t2.bit() | cp.bit() | fp.bit());
  }

  // Link the current handler as the next handler.
  li(t2, Operand(ExternalReference(Isolate::kHandlerAddress, isolate())));
  ld(t1, MemOperand(t2));
  push(t1);
  // Set this new handler as the current one.
  st(sp, MemOperand(t2), __LINE__);
}


void MacroAssembler::PopTryHandler() {
  //STATIC_ASSERT(StackHandlerConstants::kNextOffset == 0);
  pop(r1);
  Addu(sp, sp, Operand(StackHandlerConstants::kSize - kPointerSize));
  li(tt, Operand(ExternalReference(Isolate::kHandlerAddress, isolate())));
  st(r1, MemOperand(tt));
}


void MacroAssembler::JumpToHandlerEntry() {
  // Compute the handler entry address and jump to it.  The handler table is
  // a fixed array of (smi-tagged) code offsets.
  // v0 = exception, a1 = code object, a2 = state.
  ld(r3, FieldMemOperand(r1, Code::kHandlerTableOffset));  // Handler table.
  Addu(r3, r3, Operand(FixedArray::kHeaderSize - kHeapObjectTag));
  srl(r2, r2, StackHandler::kKindWidth);  // Handler index.
  sll(r2, r2, kPointerSizeLog2);
  Addu(r2, r3, r2);
  ld(r2, MemOperand(r2));  // Smi-tagged offset.
  Addu(r1, r1, Operand(Code::kHeaderSize - kHeapObjectTag));  // Code start.
  sra(tt, r2, kSmiTagSize);
  Addu(tt, tt, r1);
  Jump(tt);  // Jump.
}


void MacroAssembler::Throw(Register value) {
  // Adjust this code if not the case.
  STATIC_ASSERT(StackHandlerConstants::kSize == 5 * kPointerSize);
  STATIC_ASSERT(StackHandlerConstants::kNextOffset == 0);
  STATIC_ASSERT(StackHandlerConstants::kCodeOffset == 1 * kPointerSize);
  STATIC_ASSERT(StackHandlerConstants::kStateOffset == 2 * kPointerSize);
  STATIC_ASSERT(StackHandlerConstants::kContextOffset == 3 * kPointerSize);
  STATIC_ASSERT(StackHandlerConstants::kFPOffset == 4 * kPointerSize);

  // The exception is expected in v0.
  Move(r0, value);

  // Drop the stack pointer to the top of the top handler.
  li(r3, Operand(ExternalReference(Isolate::kHandlerAddress,
                                   isolate())));
  ld(sp, MemOperand(r3));

  // Restore the next handler.
  pop(r2);
  st(r2, MemOperand(r3));

  // Get the code object (a1) and state (a2).  Restore the context and frame
  // pointer.
  MultiPop(r1.bit() | r2.bit() | cp.bit() | fp.bit());

  // If the handler is a JS frame, restore the context to the frame.
  // (kind == ENTRY) == (fp == 0) == (cp == 0), so we could test either fp
  // or cp.
  Label done;
  Branch(&done, eq, cp, Operand(zero));
  st(cp, MemOperand(fp, StandardFrameConstants::kContextOffset));
  bind(&done);

  JumpToHandlerEntry();
}

void MacroAssembler::ThrowUncatchable(Register value) {
  // Adjust this code if not the case.
  STATIC_ASSERT(StackHandlerConstants::kSize == 5 * kPointerSize);
  STATIC_ASSERT(StackHandlerConstants::kNextOffset == 0 * kPointerSize);
  STATIC_ASSERT(StackHandlerConstants::kCodeOffset == 1 * kPointerSize);
  STATIC_ASSERT(StackHandlerConstants::kStateOffset == 2 * kPointerSize);
  STATIC_ASSERT(StackHandlerConstants::kContextOffset == 3 * kPointerSize);
  STATIC_ASSERT(StackHandlerConstants::kFPOffset == 4 * kPointerSize);

  // The exception is expected in r0.
  if (!value.is(r0)) {
    move(r0, value);
  }
  // Drop the stack pointer to the top of the top stack handler.
  li(r3, Operand(ExternalReference(Isolate::kHandlerAddress, isolate())));
  ld(sp, MemOperand(r3));

  // Unwind the handlers until the ENTRY handler is found.
  Label fetch_next, check_kind;
  jmp(&check_kind);
  bind(&fetch_next);
  ld(sp, MemOperand(sp, StackHandlerConstants::kNextOffset));

  bind(&check_kind);
  STATIC_ASSERT(StackHandler::JS_ENTRY == 0);
  ld(r2, MemOperand(sp, StackHandlerConstants::kStateOffset));
  And(r2, r2, Operand(StackHandler::KindField::kMask));
  Branch(&fetch_next, ne, r2, Operand(zero));

  // Set the top handler address to next handler past the top ENTRY handler.
  pop(r2);
  st(r2, MemOperand(r3));

  // Get the code object (a1) and state (a2).  Clear the context and frame
  // pointer (0 was saved in the handler).
  MultiPop(r1.bit() | r2.bit() | cp.bit() | fp.bit());

  JumpToHandlerEntry();
}


void MacroAssembler::Allocate(int object_size,
                              Register result,
                              Register scratch1,
                              Register scratch2,
                              Label* gc_required,
                              AllocationFlags flags) {
  if (!FLAG_inline_new) {
    if (emit_debug_code()) {
      // Trash the registers to simulate an allocation failure.
      li(result, 0x7091);
      li(scratch1, 0x7191);
      li(scratch2, 0x7291);
    }
    jmp(gc_required);
    return;
  }

  ASSERT(!result.is(scratch1));
  ASSERT(!result.is(scratch2));
  ASSERT(!scratch1.is(scratch2));
  ASSERT(!scratch1.is(t9));
  ASSERT(!scratch2.is(t9));
  ASSERT(!result.is(t9));

  // Make object size into bytes.
  if ((flags & SIZE_IN_WORDS) != 0) {
    object_size *= kPointerSize;
  }
  ASSERT_EQ(0, (int)(object_size & kObjectAlignmentMask));

  // Check relative positions of allocation top and limit addresses.
  // ARM adds additional checks to make sure the ldm instruction can be
  // used. On MIPS we don't have ldm so we don't need additional checks either.
  ExternalReference allocation_top =
      AllocationUtils::GetAllocationTopReference(isolate(), flags);
  ExternalReference allocation_limit =
      AllocationUtils::GetAllocationLimitReference(isolate(), flags);

  intptr_t top   =
      reinterpret_cast<intptr_t>(allocation_top.address());
  intptr_t limit =
      reinterpret_cast<intptr_t>(allocation_limit.address());
  ASSERT((limit - top) == kPointerSize);

  // Set up allocation top address and object size registers.
  Register topaddr = scratch1;
  Register obj_size_reg = scratch2;
  li(topaddr, Operand(allocation_top));
  li(obj_size_reg, Operand(object_size));

  // This code stores a temporary value in t9.
  if ((flags & RESULT_CONTAINS_TOP) == 0) {
    // Load allocation top into result and allocation limit into t9.
    ld(result, MemOperand(topaddr));
    ld(t9, MemOperand(topaddr, kPointerSize));
  } else {
    if (emit_debug_code()) {
      // Assert that result actually contains top on entry. t9 is used
      // immediately below so this use of t9 does not cause difference with
      // respect to register content between debug and release mode.
      ld(t9, MemOperand(topaddr));
      Check(eq, "Unexpected allocation top", result, Operand(t9));
    }
    // Load allocation limit into t9. Result already contains allocation top.
    ld(t9, MemOperand(topaddr, limit - top));
  }

  // Calculate new top and bail out if new space is exhausted. Use result
  // to calculate the new top.
  Addu(scratch2, result, Operand(obj_size_reg));
  Branch(gc_required, Ugreater, scratch2, Operand(t9));
  st(scratch2, MemOperand(topaddr));

  // Tag object if requested.
  if ((flags & TAG_OBJECT) != 0) {
    Addu(result, result, Operand(kHeapObjectTag));
  }
}


void MacroAssembler::Allocate(Register object_size,
                              Register result,
                              Register scratch1,
                              Register scratch2,
                              Label* gc_required,
                              AllocationFlags flags) {
  if (!FLAG_inline_new) {
    if (emit_debug_code()) {
      // Trash the registers to simulate an allocation failure.
      li(result, 0x7091);
      li(scratch1, 0x7191);
      li(scratch2, 0x7291);
    }
    jmp(gc_required);
    return;
  }

  ASSERT(!result.is(scratch1));
  ASSERT(!result.is(scratch2));
  ASSERT(!scratch1.is(scratch2));
  ASSERT(!object_size.is(t9));
  ASSERT(!scratch1.is(t9) && !scratch2.is(t9) && !result.is(t9));

  // Check relative positions of allocation top and limit addresses.
  // ARM adds additional checks to make sure the ldm instruction can be
  // used. On MIPS we don't have ldm so we don't need additional checks either.
  ExternalReference allocation_top =
      AllocationUtils::GetAllocationTopReference(isolate(), flags);
  ExternalReference allocation_limit =
      AllocationUtils::GetAllocationLimitReference(isolate(), flags);
  intptr_t top   =
      reinterpret_cast<intptr_t>(allocation_top.address());
  intptr_t limit =
      reinterpret_cast<intptr_t>(allocation_limit.address());
  ASSERT((limit - top) == kPointerSize);

  // Set up allocation top address and object size registers.
  Register topaddr = scratch1;
  li(topaddr, Operand(allocation_top));

  // This code stores a temporary value in t9.
  if ((flags & RESULT_CONTAINS_TOP) == 0) {
    // Load allocation top into result and allocation limit into t9.
    ld(result, MemOperand(topaddr));
    ld(t9, MemOperand(topaddr, kPointerSize));
  } else {
    if (emit_debug_code()) {
      // Assert that result actually contains top on entry. t9 is used
      // immediately below so this use of t9 does not cause difference with
      // respect to register content between debug and release mode.
      ld(t9, MemOperand(topaddr));
      Check(eq, "Unexpected allocation top", result, Operand(t9));
    }
    // Load allocation limit into t9. Result already contains allocation top.
    ld(t9, MemOperand(topaddr, limit - top));
  }

  // Calculate new top and bail out if new space is exhausted. Use result
  // to calculate the new top. Object size may be in words so a shift is
  // required to get the number of bytes.
  if ((flags & SIZE_IN_WORDS) != 0) {
    sll(scratch2, object_size, kPointerSizeLog2);
    Addu(scratch2, result, scratch2);
  } else {
    Addu(scratch2, result, Operand(object_size));
  }
  Branch(gc_required, Ugreater, scratch2, Operand(t9));

  // Update allocation top. result temporarily holds the new top.
  if (emit_debug_code()) {
    And(t9, scratch2, Operand(kObjectAlignmentMask));
    Check(eq, "Unaligned allocation in new space", t9, Operand(zero));
  }
  st(scratch2, MemOperand(topaddr));

  // Tag object if requested.
  if ((flags & TAG_OBJECT) != 0) {
    Addu(result, result, Operand(kHeapObjectTag));
  }
}


void MacroAssembler::UndoAllocationInNewSpace(Register object,
                                              Register scratch) {
  ExternalReference new_space_allocation_top =
      ExternalReference::new_space_allocation_top_address(isolate());

  // Make sure the object has no tag before resetting top.
  And(object, object, Operand(~kHeapObjectTagMask));
#ifdef DEBUG
  // Check that the object un-allocated is below the current top.
  li(scratch, Operand(new_space_allocation_top));
  ld(scratch, MemOperand(scratch));
  Check(less, "Undo allocation of non allocated memory",
      object, Operand(scratch));
#endif
  // Write the address of the object to un-allocate as the current top.
  li(scratch, Operand(new_space_allocation_top));
  st(object, MemOperand(scratch));
}

void MacroAssembler::InitializeNewString(Register string,
                                         Register length,
                                         Heap::RootListIndex map_index,
                                         Register scratch1,
                                         Register scratch2) {
  sll(scratch1, length, kSmiTagSize);
  LoadRoot(scratch2, map_index);
  st(scratch1, FieldMemOperand(string, String::kLengthOffset));
  li(scratch1, Operand(String::kEmptyHashField));
  st(scratch2, FieldMemOperand(string, HeapObject::kMapOffset));
  st(scratch1, FieldMemOperand(string, String::kHashFieldOffset));
}

void MacroAssembler::AllocateTwoByteString(Register result,
                                           Register length,
                                           Register scratch1,
                                           Register scratch2,
                                           Register scratch3,
                                           Label* gc_required) {
  // Calculate the number of bytes needed for the characters in the string while
  // observing object alignment.
  ASSERT((SeqTwoByteString::kHeaderSize & kObjectAlignmentMask) == 0);
  sll(scratch1, length, 1);  // Length in bytes, not chars.
  addi(scratch1, scratch1,
       kObjectAlignmentMask + SeqTwoByteString::kHeaderSize);
  And(scratch1, scratch1, Operand(~kObjectAlignmentMask));

  // Allocate two-byte string in new space.
  Allocate(scratch1,
           result,
           scratch2,
           scratch3,
           gc_required,
           TAG_OBJECT);

  // Set the map, length and hash field.
  InitializeNewString(result,
                      length,
                      Heap::kStringMapRootIndex,
                      scratch1,
                      scratch2);
}

void MacroAssembler::AllocateAsciiString(Register result,
                                         Register length,
                                         Register scratch1,
                                         Register scratch2,
                                         Register scratch3,
                                         Label* gc_required) {
  // Calculate the number of bytes needed for the characters in the string
  // while observing object alignment.
  ASSERT((SeqOneByteString::kHeaderSize & kObjectAlignmentMask) == 0);
  ASSERT(kCharSize == 1);
  addli(scratch1, length, kObjectAlignmentMask + SeqOneByteString::kHeaderSize);
  And(scratch1, scratch1, Operand(~kObjectAlignmentMask));

  // Allocate ASCII string in new space.
  Allocate(scratch1,
           result,
           scratch2,
           scratch3,
           gc_required,
           TAG_OBJECT);

  // Set the map, length and hash field.
  InitializeNewString(result,
                      length,
                      Heap::kAsciiStringMapRootIndex,
                      scratch1,
                      scratch2);
}


void MacroAssembler::AllocateTwoByteConsString(Register result,
                                               Register length,
                                               Register scratch1,
                                               Register scratch2,
                                               Label* gc_required) {
  Allocate(ConsString::kSize, result, scratch1, scratch2, gc_required,
           TAG_OBJECT);
  InitializeNewString(result,
                      length,
                      Heap::kConsStringMapRootIndex,
                      scratch1,
                      scratch2);
}


void MacroAssembler::AllocateAsciiConsString(Register result,
                                             Register length,
                                             Register scratch1,
                                             Register scratch2,
                                             Label* gc_required) {
  Label allocate_new_space, install_map;
  AllocationFlags flags = TAG_OBJECT;

  ExternalReference high_promotion_mode = ExternalReference::
      new_space_high_promotion_mode_active_address(isolate());
  li(scratch1, Operand(high_promotion_mode));
  ld(scratch1, MemOperand(scratch1, 0));
  Branch(&allocate_new_space, eq, scratch1, Operand(zero));

  Allocate(ConsString::kSize,
           result,
           scratch1,
           scratch2,
           gc_required,
           static_cast<AllocationFlags>(flags | PRETENURE_OLD_POINTER_SPACE));

  jmp(&install_map);

  bind(&allocate_new_space);
  Allocate(ConsString::kSize,
           result,
           scratch1,
           scratch2,
           gc_required,
           flags);

  bind(&install_map);

  InitializeNewString(result,
                      length,
                      Heap::kConsAsciiStringMapRootIndex,
                      scratch1,
                      scratch2);
}


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
                                                Register filler) {
  Label loop, entry;
  Branch(&entry);
  bind(&loop);
  st(filler, MemOperand(start_offset));
  Addu(start_offset, start_offset, kPointerSize);
  bind(&entry);
  Branch(&loop, lt, start_offset, Operand(end_offset));
}


void MacroAssembler::CheckFastElements(Register map,
                                       Register scratch,
                                       Label* fail) {
  STATIC_ASSERT(FAST_SMI_ELEMENTS == 0);
  STATIC_ASSERT(FAST_HOLEY_SMI_ELEMENTS == 1);
  STATIC_ASSERT(FAST_ELEMENTS == 2);
  STATIC_ASSERT(FAST_HOLEY_ELEMENTS == 3);
  ld1u(scratch, FieldMemOperand(map, Map::kBitField2Offset));
  Branch(fail, hi, scratch,
         Operand(Map::kMaximumBitField2FastHoleyElementValue));
}


void MacroAssembler::CheckFastObjectElements(Register map,
                                             Register scratch,
                                             Label* fail) {
  STATIC_ASSERT(FAST_SMI_ELEMENTS == 0);
  STATIC_ASSERT(FAST_HOLEY_SMI_ELEMENTS == 1);
  STATIC_ASSERT(FAST_ELEMENTS == 2);
  STATIC_ASSERT(FAST_HOLEY_ELEMENTS == 3);
  ld1u(scratch, FieldMemOperand(map, Map::kBitField2Offset));
  Branch(fail, ls, scratch,
         Operand(Map::kMaximumBitField2FastHoleySmiElementValue));
  Branch(fail, hi, scratch,
         Operand(Map::kMaximumBitField2FastHoleyElementValue));
}


void MacroAssembler::CheckFastSmiElements(Register map,
                                          Register scratch,
                                          Label* fail) {
  STATIC_ASSERT(FAST_SMI_ELEMENTS == 0);
  STATIC_ASSERT(FAST_HOLEY_SMI_ELEMENTS == 1);
  ld1u(scratch, FieldMemOperand(map, Map::kBitField2Offset));
  Branch(fail, hi, scratch,
         Operand(Map::kMaximumBitField2FastHoleySmiElementValue));
}


void MacroAssembler::StoreNumberToDoubleElements(Register value_reg,
                                                 Register key_reg,
                                                 Register elements_reg,
                                                 Register scratch1,
                                                 Register scratch2,
                                                 Register scratch3,
                                                 Register scratch4,
                                                 Label* fail,
                                                 int elements_offset) { UNREACHABLE(); }

void MacroAssembler::CompareMapAndBranch(Register obj,
                                         Register scratch,
                                         Handle<Map> map,
                                         Label* early_success,
                                         Condition cond,
                                         Label* branch_to) {
  ld(scratch, FieldMemOperand(obj, HeapObject::kMapOffset));
  CompareMapAndBranch(scratch, map, early_success, cond, branch_to);
}


void MacroAssembler::CompareMapAndBranch(Register obj_map,
                                         Handle<Map> map,
                                         Label* early_success,
                                         Condition cond,
                                         Label* branch_to) {
  Branch(branch_to, cond, obj_map, Operand(map));
}

void MacroAssembler::CheckMap(Register obj,
                              Register scratch,
                              Handle<Map> map,
                              Label* fail,
                              SmiCheckType smi_check_type) {
  if (smi_check_type == DO_SMI_CHECK) {
    JumpIfSmi(obj, fail);
  }
  Label success;
  CompareMapAndBranch(obj, scratch, map, &success, ne, fail);
  bind(&success);
}

void MacroAssembler::DispatchMap(Register obj,
                                 Register scratch,
                                 Handle<Map> map,
                                 Handle<Code> success,
                                 SmiCheckType smi_check_type) {
  Label fail;
  if (smi_check_type == DO_SMI_CHECK) {
    JumpIfSmi(obj, &fail);
  }
  ld(scratch, FieldMemOperand(obj, HeapObject::kMapOffset));
  Jump(success, RelocInfo::CODE_TARGET, eq, scratch, Operand(map));
  bind(&fail);
}

void MacroAssembler::CheckMap(Register obj,
                              Register scratch,
                              Heap::RootListIndex index,
                              Label* fail,
                              SmiCheckType smi_check_type) {
  if (smi_check_type == DO_SMI_CHECK) {
    JumpIfSmi(obj, fail);
  }
  ld(scratch, FieldMemOperand(obj, HeapObject::kMapOffset));
  LoadRoot(tt, index);
  Branch(fail, ne, scratch, Operand(tt));
}

void MacroAssembler::SetCallKind(Register dst, CallKind call_kind) {
  // This macro takes the dst register to make the code more readable
  // at the call sites. However, the dst register has to be t1 to
  // follow the calling convention which requires the call type to be
  // in t1.
  ASSERT(dst.is(t1));
  if (call_kind == CALL_AS_FUNCTION) {
    li(dst, Operand(Smi::FromInt(1)));
  } else {
    li(dst, Operand(Smi::FromInt(0)));
  }
}

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
                                    CallKind call_kind) {
  bool definitely_matches = false;
  *definitely_mismatches = false;
  Label regular_invoke;

  // Check whether the expected and actual arguments count match. If not,
  // setup registers according to contract with ArgumentsAdaptorTrampoline:
  //  a0: actual arguments count
  //  a1: function (passed through to callee)
  //  a2: expected arguments count
  //  a3: callee code entry

  // The code below is made a lot easier because the calling code already sets
  // up actual and expected registers according to the contract if values are
  // passed in registers.
  ASSERT(actual.is_immediate() || actual.reg().is(a0));
  ASSERT(expected.is_immediate() || expected.reg().is(a2));
  ASSERT((!code_constant.is_null() && code_reg.is(no_reg)) || code_reg.is(a3));

  if (expected.is_immediate()) {
    ASSERT(actual.is_immediate());
    if (expected.immediate() == actual.immediate()) {
      definitely_matches = true;
    } else {
      li(a0, Operand(actual.immediate()));
      const int sentinel = SharedFunctionInfo::kDontAdaptArgumentsSentinel;
      if (expected.immediate() == sentinel) {
        // Don't worry about adapting arguments for builtins that
        // don't want that done. Skip adaption code by making it look
        // like we have a match between expected and actual number of
        // arguments.
        definitely_matches = true;
      } else {
        *definitely_mismatches = true;
        li(a2, Operand(expected.immediate()));
      }
    }
  } else if (actual.is_immediate()) {
    Branch(&regular_invoke, eq, expected.reg(), Operand(actual.immediate()));
    li(a0, Operand(actual.immediate()));
  } else {
    Branch(&regular_invoke, eq, expected.reg(), Operand(actual.reg()));
  }

  if (!definitely_matches) {
    if (!code_constant.is_null()) {
      li(a3, Operand(code_constant));
      addli(a3, a3, Code::kHeaderSize - kHeapObjectTag);
    }

    Handle<Code> adaptor =
        isolate()->builtins()->ArgumentsAdaptorTrampoline();
    if (flag == CALL_FUNCTION) {
      call_wrapper.BeforeCall(CallSize(adaptor));
      SetCallKind(t1, call_kind);
      Call(adaptor);
      call_wrapper.AfterCall();
      if (!*definitely_mismatches) {
        Branch(done);
      }
    } else {
      SetCallKind(t1, call_kind);
      Jump(adaptor, RelocInfo::CODE_TARGET);
    }
    bind(&regular_invoke);
  }
}

void MacroAssembler::InvokeCode(Register code,
                                const ParameterCount& expected,
                                const ParameterCount& actual,
                                InvokeFlag flag,
                                const CallWrapper& call_wrapper,
                                CallKind call_kind) {
  // You can't call a function without a valid frame.
  ASSERT(flag == JUMP_FUNCTION || has_frame());

  Label done;

  bool definitely_mismatches = false;
  InvokePrologue(expected, actual, Handle<Code>::null(), code,
                 &done, &definitely_mismatches, flag,
                 call_wrapper, call_kind);
  if (!definitely_mismatches) {
    if (flag == CALL_FUNCTION) {
      call_wrapper.BeforeCall(CallSize(code));
      SetCallKind(t1, call_kind);
      Call(code);
      call_wrapper.AfterCall();
    } else {
      ASSERT(flag == JUMP_FUNCTION);
      SetCallKind(t1, call_kind);
      Jump(code);
    }
    // Continue here if InvokePrologue does handle the invocation due to
    // mismatched parameter counts.
    bind(&done);
  }
}


void MacroAssembler::InvokeCode(Handle<Code> code,
                                const ParameterCount& expected,
                                const ParameterCount& actual,
                                RelocInfo::Mode rmode,
                                InvokeFlag flag,
                                CallKind call_kind) {
  // You can't call a function without a valid frame.
  ASSERT(flag == JUMP_FUNCTION || has_frame());

  Label done;

  bool definitely_mismatches = false;
  InvokePrologue(expected, actual, code, no_reg,
                 &done, &definitely_mismatches, flag,
                 NullCallWrapper(), call_kind);
  if (!definitely_mismatches) {
    if (flag == CALL_FUNCTION) {
      SetCallKind(t1, call_kind);
      Call(code, rmode);
    } else {
      SetCallKind(t1, call_kind);
      Jump(code, rmode);
    }
    // Continue here if InvokePrologue does handle the invocation due to
    // mismatched parameter counts.
    bind(&done);
  }
}


void MacroAssembler::InvokeFunction(Register function,
                                    const ParameterCount& actual,
                                    InvokeFlag flag,
                                    const CallWrapper& call_wrapper,
                                    CallKind call_kind) {
  // You can't call a function without a valid frame.
  ASSERT(flag == JUMP_FUNCTION || has_frame());

  // Contract with called JS functions requires that function is passed in a1.
  ASSERT(function.is(a1));
  Register expected_reg = a2;
  Register code_reg = a3;

  ld(code_reg, FieldMemOperand(a1, JSFunction::kSharedFunctionInfoOffset));
  ld(cp, FieldMemOperand(a1, JSFunction::kContextOffset));
  ld(expected_reg,
      FieldMemOperand(code_reg,
                      SharedFunctionInfo::kFormalParameterCountOffset));
  sra(expected_reg, expected_reg, kSmiTagSize);
  ld(code_reg, FieldMemOperand(a1, JSFunction::kCodeEntryOffset));

  ParameterCount expected(expected_reg);
  InvokeCode(code_reg, expected, actual, flag, call_wrapper, call_kind);
}


void MacroAssembler::InvokeFunction(Handle<JSFunction> function,
                                    const ParameterCount& expected,
                                    const ParameterCount& actual,
                                    InvokeFlag flag,
                                    const CallWrapper& call_wrapper,
                                    CallKind call_kind) {
  // You can't call a function without a valid frame.
  ASSERT(flag == JUMP_FUNCTION || has_frame());

  // Get the function and setup the context.
  LoadHeapObject(a1, function);
  ld(cp, FieldMemOperand(a1, JSFunction::kContextOffset));

  // We call indirectly through the code field in the function to
  // allow recompilation to take effect without changing any of the
  // call sites.
  ld(a3, FieldMemOperand(a1, JSFunction::kCodeEntryOffset));
  InvokeCode(a3, expected, actual, flag, call_wrapper, call_kind);
}

void MacroAssembler::IsObjectNameType(Register object,
                                      Register scratch,
                                      Label* fail) {
  ld(scratch, FieldMemOperand(object, HeapObject::kMapOffset));
  ld1u(scratch, FieldMemOperand(scratch, Map::kInstanceTypeOffset));
  Branch(fail, hi, scratch, Operand(LAST_NAME_TYPE));
}


// ---------------------------------------------------------------------------
// Support functions.

void MacroAssembler::TryGetFunctionPrototype(Register function,
                                             Register result,
                                             Register scratch,
                                             Label* miss,
                                             bool miss_on_bound_function) {
  // Check that the receiver isn't a smi.
  JumpIfSmi(function, miss);

  // Check that the function really is a function.  Load map into result reg.
  GetObjectType(function, result, scratch);
  Branch(miss, ne, scratch, Operand(JS_FUNCTION_TYPE));

  if (miss_on_bound_function) {
    ld(scratch,
       FieldMemOperand(function, JSFunction::kSharedFunctionInfoOffset));
    ld(scratch,
       FieldMemOperand(scratch, SharedFunctionInfo::kCompilerHintsOffset));
    And(scratch, scratch,
        Operand(Smi::FromInt(1 << SharedFunctionInfo::kBoundFunction)));
    Branch(miss, ne, scratch, Operand(zero));
  }

  // Make sure that the function has an instance prototype.
  Label non_instance;
  ld1u(scratch, FieldMemOperand(result, Map::kBitFieldOffset));
  And(scratch, scratch, Operand(1 << Map::kHasNonInstancePrototype));
  Branch(&non_instance, ne, scratch, Operand(zero));

  // Get the prototype or initial map from the function.
  ld(result,
     FieldMemOperand(function, JSFunction::kPrototypeOrInitialMapOffset));

  // If the prototype or initial map is the hole, don't return it and
  // simply miss the cache instead. This will allow us to allocate a
  // prototype object on-demand in the runtime system.
  LoadRoot(t8, Heap::kTheHoleValueRootIndex);
  Branch(miss, eq, result, Operand(t8));

  // If the function does not have an initial map, we're done.
  Label done;
  GetObjectType(result, scratch, scratch);
  Branch(&done, ne, scratch, Operand(MAP_TYPE));

  // Get the prototype from the initial map.
  ld(result, FieldMemOperand(result, Map::kPrototypeOffset));
  jmp(&done);

  // Non-instance prototype: Fetch prototype from constructor field
  // in initial map.
  bind(&non_instance);
  ld(result, FieldMemOperand(result, Map::kConstructorOffset));

  // All done.
  bind(&done);
}


// -----------------------------------------------------------------------------
// Runtime calls.

void MacroAssembler::CallStub(CodeStub* stub,
                              TypeFeedbackId ast_id,
                              Condition cond,
                              Register r1,
                              const Operand& r2) {
  ASSERT(AllowThisStubCall(stub));  // Stub calls are not allowed in some stubs.
  Call(stub->GetCode(isolate()), RelocInfo::CODE_TARGET, ast_id, cond, r1, r2);
}


bool MacroAssembler::AllowThisStubCall(CodeStub* stub) {
  if (!has_frame_ && stub->SometimesSetsUpAFrame()) return false;
  return allow_stub_calls_ || stub->CompilingCallsToThisStubIsGCSafe(isolate());
}


void MacroAssembler::TailCallStub(CodeStub* stub) {
  ASSERT(allow_stub_calls_ ||
         stub->CompilingCallsToThisStubIsGCSafe(isolate()));
  Jump(stub->GetCode(isolate()), RelocInfo::CODE_TARGET);
}

void MacroAssembler::CallApiFunctionAndReturn(ExternalReference function,
                                              int stack_space,
                                              bool returns_handle,
                                              int return_value_offset_from_fp) { UNREACHABLE(); }


void MacroAssembler::IllegalOperation(int num_arguments) { UNREACHABLE(); }


void MacroAssembler::IndexFromHash(Register hash,
                                   Register index) { UNREACHABLE(); }

void MacroAssembler::CallRuntime(const Runtime::Function* f,
                                 int num_arguments) {
  // All parameters are on the stack. v0 has the return value after call.

  // If the expected number of arguments of the runtime function is
  // constant, we check that the actual number of arguments match the
  // expectation.
  if (f->nargs >= 0 && f->nargs != num_arguments) {
    IllegalOperation(num_arguments);
    return;
  }

  // TODO(1236192): Most runtime routines don't need the number of
  // arguments passed in because it is constant. At some point we
  // should remove this need and make the runtime routine entry code
  // smarter.
  PrepareCEntryArgs(num_arguments);
  PrepareCEntryFunction(ExternalReference(f, isolate()));
  CEntryStub stub(1);
  CallStub(&stub);
}


void MacroAssembler::CallRuntimeSaveDoubles(Runtime::FunctionId id) { UNREACHABLE(); }


void MacroAssembler::CallRuntime(Runtime::FunctionId fid, int num_arguments) {
  CallRuntime(Runtime::FunctionForId(fid), num_arguments);
}

void MacroAssembler::CallExternalReference(const ExternalReference& ext,
                                           int num_arguments) {
  PrepareCEntryArgs(num_arguments);
  PrepareCEntryFunction(ext);

  CEntryStub stub(1);
  CallStub(&stub, TypeFeedbackId::None(), al, zero, Operand(zero));
}


void MacroAssembler::TailCallExternalReference(const ExternalReference& ext,
                                               int num_arguments,
                                               int result_size) {
  // TODO(1236192): Most runtime routines don't need the number of
  // arguments passed in because it is constant. At some point we
  // should remove this need and make the runtime routine entry code
  // smarter.
  PrepareCEntryArgs(num_arguments);
  JumpToExternalReference(ext);
}

void MacroAssembler::TailCallRuntime(Runtime::FunctionId fid,
                                     int num_arguments,
                                     int result_size) {
  TailCallExternalReference(ExternalReference(fid, isolate()),
                            num_arguments,
                            result_size);
}

void MacroAssembler::JumpToExternalReference(const ExternalReference& builtin) {
  PrepareCEntryFunction(builtin);
  CEntryStub stub(1);
  Jump(stub.GetCode(isolate()),
       RelocInfo::CODE_TARGET,
       al,
       zero,
       Operand(zero));
}


void MacroAssembler::InvokeBuiltin(Builtins::JavaScript id,
                                   InvokeFlag flag,
                                   const CallWrapper& call_wrapper) {
  // You can't call a builtin without a valid frame.
  ASSERT(flag == JUMP_FUNCTION || has_frame());

  GetBuiltinEntry(t9, id);
  if (flag == CALL_FUNCTION) {
    call_wrapper.BeforeCall(CallSize(t9));
    SetCallKind(t1, CALL_AS_METHOD);
    Call(t9);
    call_wrapper.AfterCall();
  } else {
    ASSERT(flag == JUMP_FUNCTION);
    SetCallKind(t1, CALL_AS_METHOD);
    Jump(t9);
  }
}


void MacroAssembler::GetBuiltinFunction(Register target,
                                        Builtins::JavaScript id) {
  // Load the builtins object into target register.
  ld(target, MemOperand(cp, Context::SlotOffset(Context::GLOBAL_OBJECT_INDEX)));
  ld(target, FieldMemOperand(target, GlobalObject::kBuiltinsOffset));
  // Load the JavaScript builtin function from the builtins object.
  ld(target, FieldMemOperand(target,
                          JSBuiltinsObject::OffsetOfFunctionWithId(id)));
}

void MacroAssembler::GetBuiltinEntry(Register target, Builtins::JavaScript id) {
  ASSERT(!target.is(a1));
  GetBuiltinFunction(a1, id);
  // Load the code entry point from the builtins object.
  ld(target, FieldMemOperand(a1, JSFunction::kCodeEntryOffset));
}

void MacroAssembler::SetCounter(StatsCounter* counter, int value,
                                Register scratch1, Register scratch2) {
  if (FLAG_native_code_counters && counter->Enabled()) {
    li(scratch1, Operand(value));
    li(scratch2, Operand(ExternalReference(counter)));
    st(scratch1, MemOperand(scratch2));
  }
}

void MacroAssembler::IncrementCounter(StatsCounter* counter, int value,
                                      Register scratch1, Register scratch2) {
  ASSERT(value > 0);
  if (FLAG_native_code_counters && counter->Enabled()) {
    li(scratch2, Operand(ExternalReference(counter)));
    ld(scratch1, MemOperand(scratch2));
    Addu(scratch1, scratch1, Operand(value));
    st(scratch1, MemOperand(scratch2));
  }
}

void MacroAssembler::DecrementCounter(StatsCounter* counter, int value,
                                      Register scratch1, Register scratch2) {
  ASSERT(value > 0);
  if (FLAG_native_code_counters && counter->Enabled()) {
    li(scratch2, Operand(ExternalReference(counter)));
    ld(scratch1, MemOperand(scratch2));
    Subu(scratch1, scratch1, Operand(value));
    st(scratch1, MemOperand(scratch2));
  }
}

// -----------------------------------------------------------------------------
// Debugging.

void MacroAssembler::Assert(Condition cc, const char* msg,
                            Register rs, Operand rt) {
  if (emit_debug_code())
    Check(cc, msg, rs, rt);
}


void MacroAssembler::AssertRegisterIsRoot(Register reg,
                                          Heap::RootListIndex index) {
  if (emit_debug_code()) {
    LoadRoot(tt, index);
    Check(eq, "Register did not match expected root", reg, Operand(tt));
  }
}


void MacroAssembler::AssertFastElements(Register elements) {
  if (emit_debug_code()) {
    ASSERT(!elements.is(tt));
    Label ok;
    push(elements);
    ld(elements, FieldMemOperand(elements, HeapObject::kMapOffset));
    LoadRoot(tt, Heap::kFixedArrayMapRootIndex);
    Branch(&ok, eq, elements, Operand(tt));
    LoadRoot(tt, Heap::kFixedDoubleArrayMapRootIndex);
    Branch(&ok, eq, elements, Operand(tt));
    LoadRoot(tt, Heap::kFixedCOWArrayMapRootIndex);
    Branch(&ok, eq, elements, Operand(tt));
    Abort("JSObject with fast elements map has slow elements");
    bind(&ok);
    pop(elements);
  }
}


void MacroAssembler::Check(Condition cc, const char* msg,
                           Register rs, Operand rt) {
  Label L;
  Branch(&L, cc, rs, rt);
  Abort(msg);
  // Will not return here.
  bind(&L);
}

void MacroAssembler::Abort(const char* msg) {
  Label abort_start;
  bind(&abort_start);
  // We want to pass the msg string like a smi to avoid GC
  // problems, however msg is not guaranteed to be aligned
  // properly. Instead, we pass an aligned pointer that is
  // a proper v8 smi, but also pass the alignment difference
  // from the real pointer as a smi.
  intptr_t p1 = reinterpret_cast<intptr_t>(msg);
  intptr_t p0 = (p1 & ~kSmiTagMask) + kSmiTag;
  ASSERT(reinterpret_cast<Object*>(p0)->IsSmi());
#ifdef DEBUG
  if (msg != NULL) {
    RecordComment("Abort message: ");
    RecordComment(msg);
  }
#endif

  li(a0, Operand(p0));
  push(a0);
  li(a0, Operand(Smi::FromInt(p1 - p0)));
  push(a0);
  // Disable stub call restrictions to always allow calls to abort.
  if (!has_frame_) {
    // We don't actually want to generate a pile of code for this, so just
    // claim there is a stack frame, without generating one.
    FrameScope scope(this, StackFrame::NONE);
    CallRuntime(Runtime::kAbort, 2);
  } else {
    CallRuntime(Runtime::kAbort, 2);
  }
  // Will not return here.
  if (is_trampoline_pool_blocked()) {
    // If the calling code cares about the exact number of
    // instructions generated, we insert padding here to keep the size
    // of the Abort macro constant.
    // Currently in debug mode with debug_code enabled the number of
    // generated instructions is 14, so we use this as a maximum value.
    static const int kExpectedAbortInstructions = 14;
    int abort_instructions = InstructionsGeneratedSince(&abort_start);
    ASSERT(abort_instructions <= kExpectedAbortInstructions);
    while (abort_instructions++ < kExpectedAbortInstructions) {
      nop();
    }
  }
}

void MacroAssembler::LoadContext(Register dst, int context_chain_length) {
  if (context_chain_length > 0) {
    // Move up the chain of contexts to the context containing the slot.
    ld(dst, MemOperand(cp, Context::SlotOffset(Context::PREVIOUS_INDEX)));
    for (int i = 1; i < context_chain_length; i++) {
      ld(dst, MemOperand(dst, Context::SlotOffset(Context::PREVIOUS_INDEX)));
    }
  } else {
    // Slot is in the current function context.  Move it into the
    // destination register in case we store into it (the write barrier
    // cannot be allowed to destroy the context in esi).
    Move(dst, cp);
  }
}


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


void MacroAssembler::EnterFrame(StackFrame::Type type) {
  addi(sp, sp, -5 * kPointerSize);
  li(r40, Operand(Smi::FromInt(type)));
  li(r41, Operand(CodeObject()), CONSTANT_SIZE);
  st(lr, MemOperand(sp, 4 * kPointerSize));
  st(fp, MemOperand(sp, 3 * kPointerSize));
  st(cp, MemOperand(sp, 2 * kPointerSize));
  st(r40, MemOperand(sp, 1 * kPointerSize));
  st(r41, MemOperand(sp, 0 * kPointerSize));
  addi(fp, sp, 3 * kPointerSize);
}


void MacroAssembler::LeaveFrame(StackFrame::Type type) {
  move(sp, fp);
  ld(fp, MemOperand(sp, 0 * kPointerSize));
  ld(lr, MemOperand(sp, 1 * kPointerSize));
  addi(sp, sp, 2 * kPointerSize);
}

void MacroAssembler::EnterExitFrame(bool save_doubles,
                                    int stack_space) {
  // Set up the frame structure on the stack.
  STATIC_ASSERT(2 * kPointerSize == ExitFrameConstants::kCallerSPDisplacement);
  STATIC_ASSERT(1 * kPointerSize == ExitFrameConstants::kCallerPCOffset);
  STATIC_ASSERT(0 * kPointerSize == ExitFrameConstants::kCallerFPOffset);

  // This is how the stack will look:
  // fp + 2 (==kCallerSPDisplacement) - old stack's end
  // [fp + 1 (==kCallerPCOffset)] - saved old ra
  // [fp + 0 (==kCallerFPOffset)] - saved old fp
  // [fp - 1 (==kSPOffset)] - sp of the called function
  // [fp - 2 (==kCodeOffset)] - CodeObject
  // fp - (2 + stack_space + alignment) == sp == [fp - kSPOffset] - top of the
  //   new stack (will contain saved ra)

  // Save registers.
  addi(sp, sp, -4 * kPointerSize);
  st(lr, MemOperand(sp, 3 * kPointerSize));
  st(fp, MemOperand(sp, 2 * kPointerSize));
  addi(fp, sp, 2 * kPointerSize);  // Set up new frame pointer.

  if (emit_debug_code()) {
    st(zero, MemOperand(fp, ExitFrameConstants::kSPOffset));
  }

  // Accessed from ExitFrame::code_slot.
  li(r40, Operand(CodeObject()), CONSTANT_SIZE);
  st(r40, MemOperand(fp, ExitFrameConstants::kCodeOffset));

  // Save the frame pointer and the context in top.
  li(r40, Operand(ExternalReference(Isolate::kCEntryFPAddress, isolate())));
  st(fp, MemOperand(r40));
  li(r40, Operand(ExternalReference(Isolate::kContextAddress, isolate())));
  st(cp, MemOperand(r40));

  const int frame_alignment = MacroAssembler::ActivationFrameAlignment();
#if 0
  if (save_doubles) {
#if 0
    // The stack  must be allign to 0 modulo 8 for stores with sdc1.
    ASSERT(kDoubleSize == frame_alignment);
    if (frame_alignment > 0) {
      ASSERT(IsPowerOf2(frame_alignment));
      And(sp, sp, Operand(-frame_alignment));  // Align stack.
    }
    int space = FPURegister::kMaxNumRegisters * kDoubleSize;
    Subu(sp, sp, Operand(space));
    // Remember: we only need to save every 2nd double FPU value.
    for (int i = 0; i < FPURegister::kMaxNumRegisters; i+=2) {
      FPURegister reg = FPURegister::from_code(i);
      sdc1(reg, MemOperand(sp, i * kDoubleSize));
    }
#else
    UNREACHABLE();
#endif
  }
#endif

  // Reserve place for the return address, stack space and an optional slot
  // (used by the DirectCEntryStub to hold the return value if a struct is
  // returned) and align the frame preparing for calling the runtime function.
  ASSERT(stack_space >= 0);
  Subu(sp, sp, Operand((stack_space + 2) * kPointerSize));
  if (frame_alignment > 0) {
    ASSERT(IsPowerOf2(frame_alignment));
    And(sp, sp, Operand(-frame_alignment));  // Align stack.
  }

  // Set the exit frame sp value to point just before the return address
  // location.
  addi(tt, sp, kPointerSize);
  st(tt, MemOperand(fp, ExitFrameConstants::kSPOffset));
}

void MacroAssembler::LeaveExitFrame(bool save_doubles,
                                    Register argument_count,
                                    bool do_return) {
#if 0
  // Optionally restore all double registers.
  if (save_doubles) {
#if 0
    // Remember: we only need to restore every 2nd double FPU value.
    lw(t8, MemOperand(fp, ExitFrameConstants::kSPOffset));
    for (int i = 0; i < FPURegister::kMaxNumRegisters; i+=2) {
      FPURegister reg = FPURegister::from_code(i);
      ldc1(reg, MemOperand(t8, i  * kDoubleSize + kPointerSize));
    }
#else
    UNREACHABLE();
#endif
  }
#endif

  // Clear top frame.
  li(r40, Operand(ExternalReference(Isolate::kCEntryFPAddress, isolate())));
  st(zero, MemOperand(r40));

  // Restore current context from top and clear it in debug mode.
  li(r40, Operand(ExternalReference(Isolate::kContextAddress, isolate())));
  ld(cp, MemOperand(r40));
#ifdef DEBUG
  st(r3, MemOperand(r40));
#endif

  // Pop the arguments, restore registers, and return.
  move(sp, fp);  // Respect ABI stack constraint.
  ld(fp, MemOperand(sp, ExitFrameConstants::kCallerFPOffset));
  ld(lr, MemOperand(sp, ExitFrameConstants::kCallerPCOffset));

  if (argument_count.is_valid()) {
    sll(r40, argument_count, kPointerSizeLog2);
    add(sp, sp, r40);
  }

  if (do_return) {
    Ret();
    // If returning, the instruction in the delay slot will be the addiu below.
  }
  addi(sp, sp, 8);
}

int MacroAssembler::ActivationFrameAlignment() {
#if defined(V8_HOST_ARCH_TILEGX)
  // Running on the real platform. Use the alignment as mandated by the local
  // environment.
  // Note: This will break if we ever start generating snapshots on one TileGX
  // platform for another TileGX platform with a different alignment.
  return OS::ActivationFrameAlignment();
#else  // defined(V8_HOST_ARCH_TILEGX)
  // No Simulator support for TileGX
  UNREACHABLE();
  return -1;
#endif  // defined(V8_HOST_ARCH_TILEGX)
}


void MacroAssembler::AssertStackIsAligned() {
  if (emit_debug_code()) {
    const int frame_alignment = ActivationFrameAlignment();
    const int frame_alignment_mask = frame_alignment - 1;

    if (frame_alignment > kPointerSize) {
      Label alignment_as_expected;
      ASSERT(IsPowerOf2(frame_alignment));
      andi(tt, sp, frame_alignment_mask);
      Branch(&alignment_as_expected, eq, tt, Operand(zero));
      // Don't use Check here, as it will call Runtime_Abort re-entering here.
      stop("Unexpected stack alignment");
      bind(&alignment_as_expected);
    }
  }
}

void MacroAssembler::UntagAndJumpIfSmi(Register dst,
                                       Register src,
                                       Label* smi_case) { UNREACHABLE(); }


void MacroAssembler::UntagAndJumpIfNotSmi(Register dst,
                                          Register src,
                                          Label* non_smi_case) { UNREACHABLE(); }

void MacroAssembler::JumpIfSmi(Register value,
                               Label* smi_label,
                               Register scratch) {
  ASSERT_EQ(0, kSmiTag);
  andi(scratch, value, kSmiTagMask);
  Branch(smi_label, eq, scratch, Operand(zero));
}

void MacroAssembler::JumpIfNotSmi(Register value,
                                  Label* not_smi_label,
                                  Register scratch) {
  ASSERT_EQ(0, kSmiTag);
  andi(scratch, value, kSmiTagMask);
  Branch(not_smi_label, ne, scratch, Operand(zero));
}

void MacroAssembler::JumpIfNotBothSmi(Register reg1,
                                      Register reg2,
                                      Label* on_not_both_smi) {
  STATIC_ASSERT(kSmiTag == 0);
  ASSERT_EQ(1, (int)kSmiTagMask);
  or_(tt, reg1, reg2);
  JumpIfNotSmi(tt, on_not_both_smi);
}

void MacroAssembler::JumpIfEitherSmi(Register reg1,
                                     Register reg2,
                                     Label* on_either_smi) {
  STATIC_ASSERT(kSmiTag == 0);
  ASSERT_EQ(1, (int)kSmiTagMask);
  // Both Smi tags must be 1 (not Smi).
  and_(tt, reg1, reg2);
  JumpIfSmi(tt, on_either_smi);
}

void MacroAssembler::AssertNotSmi(Register object) {
  if (emit_debug_code()) {
    STATIC_ASSERT(kSmiTag == 0);
    andi(tt, object, kSmiTagMask);
    Check(ne, "Operand is a smi", tt, Operand(zero));
  }
}

void MacroAssembler::AssertSmi(Register object) {
  if (emit_debug_code()) {
    STATIC_ASSERT(kSmiTag == 0);
    andi(tt, object, kSmiTagMask);
    Check(eq, "Operand is a smi", tt, Operand(zero));
  }
}

void MacroAssembler::AssertString(Register object) {
  if (emit_debug_code()) {
    STATIC_ASSERT(kSmiTag == 0);
    And(t0, object, Operand(kSmiTagMask));
    Check(ne, "Operand is a smi and not a string", t0, Operand(zero));
    push(object);
    ld(object, FieldMemOperand(object, HeapObject::kMapOffset));
    ld1u(object, FieldMemOperand(object, Map::kInstanceTypeOffset));
    Check(lo, "Operand is not a string", object, Operand(FIRST_NONSTRING_TYPE));
    pop(object);
  }
}

void MacroAssembler::AssertName(Register object) {
  if (emit_debug_code()) {
    STATIC_ASSERT(kSmiTag == 0);
    And(t0, object, Operand(kSmiTagMask));
    Check(ne, "Operand is a smi and not a name", t0, Operand(zero));
    push(object);
    ld(object, FieldMemOperand(object, HeapObject::kMapOffset));
    ld1u(object, FieldMemOperand(object, Map::kInstanceTypeOffset));
    Check(le, "Operand is not a name", object, Operand(LAST_NAME_TYPE));
    pop(object);
  }
}

void MacroAssembler::AssertRootValue(Register src,
                                     Heap::RootListIndex root_value_index,
                                     const char* message) {
  if (emit_debug_code()) {
    ASSERT(!src.is(tt));
    LoadRoot(tt, root_value_index);
    Check(eq, message, src, Operand(tt));
  }
}


void MacroAssembler::JumpIfNonSmisNotBothSequentialAsciiStrings(
    Register first,
    Register second,
    Register scratch1,
    Register scratch2,
    Label* failure) {
  // Test that both first and second are sequential ASCII strings.
  // Assume that they are non-smis.
  ld(scratch1, FieldMemOperand(first, HeapObject::kMapOffset));
  ld(scratch2, FieldMemOperand(second, HeapObject::kMapOffset));
  ld1u(scratch1, FieldMemOperand(scratch1, Map::kInstanceTypeOffset));
  ld1u(scratch2, FieldMemOperand(scratch2, Map::kInstanceTypeOffset));

  JumpIfBothInstanceTypesAreNotSequentialAscii(scratch1,
                                               scratch2,
                                               scratch1,
                                               scratch2,
                                               failure);
}

void MacroAssembler::JumpIfNotBothSequentialAsciiStrings(Register first,
                                                         Register second,
                                                         Register scratch1,
                                                         Register scratch2,
                                                         Label* failure) {
  // Check that neither is a smi.
  STATIC_ASSERT(kSmiTag == 0);
  And(scratch1, first, Operand(second));
  JumpIfSmi(scratch1, failure);
  JumpIfNonSmisNotBothSequentialAsciiStrings(first,
                                             second,
                                             scratch1,
                                             scratch2,
                                             failure);
}

void MacroAssembler::JumpIfBothInstanceTypesAreNotSequentialAscii(
    Register first,
    Register second,
    Register scratch1,
    Register scratch2,
    Label* failure) {
  int kFlatAsciiStringMask =
      kIsNotStringMask | kStringEncodingMask | kStringRepresentationMask;
  int kFlatAsciiStringTag = ASCII_STRING_TYPE;
  ASSERT(kFlatAsciiStringTag <= 0xffff);  // Ensure this fits 16-bit immed.
  andi(scratch1, first, kFlatAsciiStringMask);
  Branch(failure, ne, scratch1, Operand(kFlatAsciiStringTag));
  andi(scratch2, second, kFlatAsciiStringMask);
  Branch(failure, ne, scratch2, Operand(kFlatAsciiStringTag));
}


void MacroAssembler::JumpIfInstanceTypeIsNotSequentialAscii(Register type,
                                                            Register scratch,
                                                            Label* failure) {
  int kFlatAsciiStringMask =
      kIsNotStringMask | kStringEncodingMask | kStringRepresentationMask;
  int kFlatAsciiStringTag = ASCII_STRING_TYPE;
  And(scratch, type, Operand(kFlatAsciiStringMask));
  Branch(failure, ne, scratch, Operand(kFlatAsciiStringTag));
}

static const int kRegisterPassedArguments = 10;

int MacroAssembler::CalculateStackPassedWords(int num_reg_arguments) {
  int stack_passed_words = 0;

  // Up to ten simple arguments are passed in registers r0..r9.
  if (num_reg_arguments > kRegisterPassedArguments) {
    stack_passed_words += num_reg_arguments - kRegisterPassedArguments;
  }
  return stack_passed_words;
}

void MacroAssembler::PrepareCallCFunction(int num_reg_arguments,
                                          Register scratch) {
  int frame_alignment = ActivationFrameAlignment();

  // Up to ten simple arguments are passed in registers r0..r9.
  // Remaining arguments are pushed on the stack.
  int stack_passed_arguments = CalculateStackPassedWords(num_reg_arguments);
  if (frame_alignment > kPointerSize) {
    // Make stack end at alignment and make room for num_arguments - 4 words
    // and the original value of sp.
    move(scratch, sp);
    Subu(sp, sp, Operand((stack_passed_arguments + 1) * kPointerSize));
    ASSERT(IsPowerOf2(frame_alignment));
    And(sp, sp, Operand(-frame_alignment));
    st(scratch, MemOperand(sp, stack_passed_arguments * kPointerSize));
  } else {
    Subu(sp, sp, Operand(stack_passed_arguments * kPointerSize));
  }
}

void MacroAssembler::CallCFunction(ExternalReference function,
                                   int num_arguments) {
  li(r40, Operand(function));
  CallCFunctionHelper(r40, num_arguments);
}


void MacroAssembler::CallCFunction(Register function,
                                   int num_arguments) {
  CallCFunctionHelper(function, num_arguments);
}

void MacroAssembler::CallCFunctionHelper(Register function,
                                         int num_reg_arguments) {
  ASSERT(has_frame());
  // Make sure that the stack is aligned before calling a C function unless
  // running in the simulator. The simulator has its own alignment check which
  // provides more information.
  // The argument stots are presumed to have been set up by
  // PrepareCallCFunction. The C function must be called via t9, for mips ABI.

#if defined(V8_HOST_ARCH_TILEGX)
  if (emit_debug_code()) {
    int frame_alignment = OS::ActivationFrameAlignment();
    int frame_alignment_mask = frame_alignment - 1;
    if (frame_alignment > kPointerSize) {
      ASSERT(IsPowerOf2(frame_alignment));
      Label alignment_as_expected;
      And(tt, sp, Operand(frame_alignment_mask));
      Branch(&alignment_as_expected, eq, tt, Operand(zero));
      // Don't use Check here, as it will call Runtime_Abort possibly
      // re-entering here.
      stop("Unexpected alignment in CallCFunction");
      bind(&alignment_as_expected);
    }
  }
#endif  // V8_HOST_ARCH_TILEGX

  Call(function);

  int stack_passed_arguments = CalculateStackPassedWords(num_reg_arguments);

  if (OS::ActivationFrameAlignment() > kPointerSize) {
    ld(sp, MemOperand(sp, stack_passed_arguments * kPointerSize));
  } else {
    Addu(sp, sp, Operand(stack_passed_arguments * sizeof(kPointerSize)));
  }
}

#undef BRANCH_ARGS_CHECK


void MacroAssembler::CheckPageFlag(
    Register object,
    Register scratch,
    int mask,
    Condition cc,
    Label* condition_met) {
  And(scratch, object, Operand(~Page::kPageAlignmentMask));
  ld(scratch, MemOperand(scratch, MemoryChunk::kFlagsOffset));
  And(scratch, scratch, Operand(mask));
  Branch(condition_met, cc, scratch, Operand(zero));
}


void MacroAssembler::CheckMapDeprecated(Handle<Map> map,
                                        Register scratch,
                                        Label* if_deprecated) {
  if (map->CanBeDeprecated()) {
    li(scratch, Operand(map));
    ld(scratch, FieldMemOperand(scratch, Map::kBitField3Offset));
    And(scratch, scratch, Operand(Smi::FromInt(Map::Deprecated::kMask)));
    Branch(if_deprecated, ne, scratch, Operand(zero));
  }
}


void MacroAssembler::JumpIfBlack(Register object,
                                 Register scratch0,
                                 Register scratch1,
                                 Label* on_black) {
  HasColor(object, scratch0, scratch1, on_black, 1, 0);  // kBlackBitPattern.
  ASSERT(strcmp(Marking::kBlackBitPattern, "10") == 0);
}


void MacroAssembler::HasColor(Register object,
                              Register bitmap_scratch,
                              Register mask_scratch,
                              Label* has_color,
                              int first_bit,
                              int second_bit) {
  ASSERT(!AreAliased(object, bitmap_scratch, mask_scratch, t8));
  ASSERT(!AreAliased(object, bitmap_scratch, mask_scratch, t9));

  GetMarkBits(object, bitmap_scratch, mask_scratch);

  Label other_color, word_boundary;
  ld(t9, MemOperand(bitmap_scratch, MemoryChunk::kHeaderSize));
  And(t8, t9, Operand(mask_scratch));
  Branch(&other_color, first_bit == 1 ? eq : ne, t8, Operand(zero));
  // Shift left 1 by adding.
  Addu(mask_scratch, mask_scratch, Operand(mask_scratch));
  Branch(&word_boundary, eq, mask_scratch, Operand(zero));
  And(t8, t9, Operand(mask_scratch));
  Branch(has_color, second_bit == 1 ? ne : eq, t8, Operand(zero));
  jmp(&other_color);

  bind(&word_boundary);
  ld(t9, MemOperand(bitmap_scratch, MemoryChunk::kHeaderSize + kPointerSize));
  And(t9, t9, Operand(1));
  Branch(has_color, second_bit == 1 ? ne : eq, t9, Operand(zero));
  bind(&other_color);
}

// Detect some, but not all, common pointer-free objects.  This is used by the
// incremental write barrier which doesn't care about oddballs (they are always
// marked black immediately so this code is not hit).
void MacroAssembler::JumpIfDataObject(Register value,
                                      Register scratch,
                                      Label* not_data_object) {
  ASSERT(!AreAliased(value, scratch, t8, no_reg));
  Label is_data_object;
  ld(scratch, FieldMemOperand(value, HeapObject::kMapOffset));
  LoadRoot(t8, Heap::kHeapNumberMapRootIndex);
  Branch(&is_data_object, eq, t8, Operand(scratch));
  ASSERT(kIsIndirectStringTag == 1 && kIsIndirectStringMask == 1);
  ASSERT(kNotStringTag == 0x80 && kIsNotStringMask == 0x80);
  // If it's a string and it's not a cons string then it's an object containing
  // no GC pointers.
  ld1u(scratch, FieldMemOperand(scratch, Map::kInstanceTypeOffset));
  And(t8, scratch, Operand(kIsIndirectStringMask | kIsNotStringMask));
  Branch(not_data_object, ne, t8, Operand(zero));
  bind(&is_data_object);
}


void MacroAssembler::GetMarkBits(Register addr_reg,
                                 Register bitmap_reg,
                                 Register mask_reg) {
  ASSERT(!AreAliased(addr_reg, bitmap_reg, mask_reg, no_reg));
  And(bitmap_reg, addr_reg, Operand(~Page::kPageAlignmentMask));
  bfextu(mask_reg, addr_reg, kPointerSizeLog2, kPointerSizeLog2 + Bitmap::kBitsPerCellLog2 - 1);
  const int kLowBits = kPointerSizeLog2 + Bitmap::kBitsPerCellLog2;
  bfextu(t8, addr_reg, kLowBits, kLowBits + kPageSizeBits - kLowBits - 1);
  sll(t8, t8, kPointerSizeLog2);
  Addu(bitmap_reg, bitmap_reg, t8);
  li(t8, Operand(1));
  sll(mask_reg, t8, mask_reg);
 }


void MacroAssembler::EnsureNotWhite(
    Register value,
    Register bitmap_scratch,
    Register mask_scratch,
    Register load_scratch,
    Label* value_is_white_and_not_data) {
  ASSERT(!AreAliased(value, bitmap_scratch, mask_scratch, t8));
  GetMarkBits(value, bitmap_scratch, mask_scratch);

  // If the value is black or grey we don't need to do anything.
  ASSERT(strcmp(Marking::kWhiteBitPattern, "00") == 0);
  ASSERT(strcmp(Marking::kBlackBitPattern, "10") == 0);
  ASSERT(strcmp(Marking::kGreyBitPattern, "11") == 0);
  ASSERT(strcmp(Marking::kImpossibleBitPattern, "01") == 0);

  Label done;

  // Since both black and grey have a 1 in the first position and white does
  // not have a 1 there we only need to check one bit.
  ld(load_scratch, MemOperand(bitmap_scratch, MemoryChunk::kHeaderSize));
  And(t8, mask_scratch, load_scratch);
  Branch(&done, ne, t8, Operand(zero));

  if (emit_debug_code()) {
    // Check for impossible bit pattern.
    Label ok;
    // sll may overflow, making the check conservative.
    sll(t8, mask_scratch, 1);
    And(t8, load_scratch, t8);
    Branch(&ok, eq, t8, Operand(zero));
    stop("Impossible marking bit pattern");
    bind(&ok);
  }

  // Value is white.  We check whether it is data that doesn't need scanning.
  // Currently only checks for HeapNumber and non-cons strings.
  Register map = load_scratch;  // Holds map while checking type.
  Register length = load_scratch;  // Holds length of object after testing type.
  Label is_data_object;

  // Check for heap-number
  ld(map, FieldMemOperand(value, HeapObject::kMapOffset));
  LoadRoot(t8, Heap::kHeapNumberMapRootIndex);
  {
    Label skip;
    Branch(&skip, ne, t8, Operand(map));
    li(length, HeapNumber::kSize);
    Branch(&is_data_object);
    bind(&skip);
  }

  // Check for strings.
  ASSERT(kIsIndirectStringTag == 1 && kIsIndirectStringMask == 1);
  ASSERT(kNotStringTag == 0x80 && kIsNotStringMask == 0x80);
  // If it's a string and it's not a cons string then it's an object containing
  // no GC pointers.
  Register instance_type = load_scratch;
  ld1u(instance_type, FieldMemOperand(map, Map::kInstanceTypeOffset));
  And(t8, instance_type, Operand(kIsIndirectStringMask | kIsNotStringMask));
  Branch(value_is_white_and_not_data, ne, t8, Operand(zero));
  // It's a non-indirect (non-cons and non-slice) string.
  // If it's external, the length is just ExternalString::kSize.
  // Otherwise it's String::kHeaderSize + string->length() * (1 or 2).
  // External strings are the only ones with the kExternalStringTag bit
  // set.
  ASSERT_EQ(0, kSeqStringTag & kExternalStringTag);
  ASSERT_EQ(0, kConsStringTag & kExternalStringTag);
  And(t8, instance_type, Operand(kExternalStringTag));
  {
    Label skip;
    Branch(&skip, eq, t8, Operand(zero));
    li(length, ExternalString::kSize);
    Branch(&is_data_object);
    bind(&skip);
  }

  // Sequential string, either ASCII or UC16.
  // For ASCII (char-size of 1) we shift the smi tag away to get the length.
  // For UC16 (char-size of 2) we just leave the smi tag in place, thereby
  // getting the length multiplied by 2.
  ASSERT(kOneByteStringTag == 4 && kStringEncodingMask == 4);
  ASSERT(kSmiTag == 0 && kSmiTagSize == 1);
  ld(t9, FieldMemOperand(value, String::kLengthOffset));
  And(t8, instance_type, Operand(kStringEncodingMask));
  {
    Label skip;
    Branch(&skip, eq, t8, Operand(zero));
    srl(t9, t9, 1);
    bind(&skip);
  }
  Addu(length, t9, Operand(SeqString::kHeaderSize + kObjectAlignmentMask));
  And(length, length, Operand(~kObjectAlignmentMask));

  bind(&is_data_object);
  // Value is a data object, and it is white.  Mark it black.  Since we know
  // that the object is white we can make it black by flipping one bit.
  ld(t8, MemOperand(bitmap_scratch, MemoryChunk::kHeaderSize));
  Or(t8, t8, Operand(mask_scratch));
  st(t8, MemOperand(bitmap_scratch, MemoryChunk::kHeaderSize));

  And(bitmap_scratch, bitmap_scratch, Operand(~Page::kPageAlignmentMask));
  ld(t8, MemOperand(bitmap_scratch, MemoryChunk::kLiveBytesOffset));
  Addu(t8, t8, Operand(length));
  st(t8, MemOperand(bitmap_scratch, MemoryChunk::kLiveBytesOffset));

  bind(&done);
}


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


void MacroAssembler::GetObjectType(Register object,
                                   Register map,
                                   Register type_reg) {
  ld(map, FieldMemOperand(object, HeapObject::kMapOffset));
  ld1u(type_reg, FieldMemOperand(map, Map::kInstanceTypeOffset));
}

void MacroAssembler::SmiTagCheckOverflow(Register reg, Register overflow) {
  ASSERT(!reg.is(overflow));
  move(overflow, reg);  // Save original value.
  SmiTag(reg);
  xor_(overflow, overflow, reg);  // Overflow if (value ^ 2 * value) < 0.
}


void MacroAssembler::SmiTagCheckOverflow(Register dst,
                                         Register src,
                                         Register overflow) {
  if (dst.is(src)) {
    // Fall back to slower case.
    SmiTagCheckOverflow(dst, overflow);
  } else {
    ASSERT(!dst.is(src));
    ASSERT(!dst.is(overflow));
    ASSERT(!src.is(overflow));
    SmiTag(dst, src);
    xor_(overflow, dst, src);  // Overflow if (value ^ 2 * value) < 0.
  }
}

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
