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


void MacroAssembler::Or(Register rd, Register rs, const Operand& rt) { UNREACHABLE(); }


void MacroAssembler::Xor(Register rd, Register rs, const Operand& rt) { UNREACHABLE(); }


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


void MacroAssembler::MultiPushReversed(RegList regs) { UNREACHABLE(); }


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
