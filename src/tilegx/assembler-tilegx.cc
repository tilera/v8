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


#include "v8.h"

#if defined(V8_TARGET_ARCH_TILEGX)

#include "tilegx/assembler-tilegx-inl.h"
#include "serialize.h"

namespace v8 {
namespace internal {

#ifdef DEBUG
bool CpuFeatures::initialized_ = false;
#endif
unsigned CpuFeatures::supported_ = 0;
unsigned CpuFeatures::found_by_runtime_probing_only_ = 0;


ExternalReference ExternalReference::cpu_features() {
  UNIMPLEMENTED();
  ASSERT(CpuFeatures::initialized_);
  return ExternalReference(&CpuFeatures::supported_);
}

void CpuFeatures::Probe() {
  uint64_t standard_features = OS::CpuFeaturesImpliedByPlatform();

  ASSERT(supported_ == 0 || supported_ == standard_features);

#ifdef DEBUG
  initialized_ = true;
#endif

  supported_ |= standard_features;

  if (Serializer::enabled()) {
    // No probing for features if we might serialize (generate snapshot).
    return;
  }

  // No further probing currently, may be extended in future.
}

// Determines the end of the Jump chain (a subset of the label link chain).
const int kEndOfJumpChain = 0;

bool Assembler::IsBranch(Instr instr) {
  uint32_t mode   = get_Mode(instr);
  int32_t X1_OPC = -1;

  if (mode != 0)
    return false;

  X1_OPC = get_Opcode_X1(instr);

  // Checks if the instruction is a branch.
  return X1_OPC == BRANCH_OPCODE_X1;
}

bool Assembler::IsJ(Instr instr) {
  uint32_t mode   = get_Mode(instr);
  int32_t X1_OPC = -1, X1_SUB_OPC = -1;

  if (mode != 0)
    return false;

  X1_OPC = get_Opcode_X1(instr);
  X1_SUB_OPC = get_JumpOpcodeExtension_X1(instr);

  // Checks if the instruction is a jump.
  return X1_OPC == JUMP_OPCODE_X1 && X1_SUB_OPC == J_JUMP_OPCODE_X1;
}

bool Assembler::IsJAL(Instr instr) {
  uint32_t mode   = get_Mode(instr);
  int32_t X1_OPC = -1, X1_SUB_OPC = -1;

  if (mode != 0)
    return false;

  X1_OPC = get_Opcode_X1(instr);
  X1_SUB_OPC = get_JumpOpcodeExtension_X1(instr);

  // Checks if the instruction is a jal.
  return X1_OPC == JUMP_OPCODE_X1 && X1_SUB_OPC == JAL_JUMP_OPCODE_X1;
}

bool Assembler::IsJR(Instr instr) {
  uint32_t mode   = get_Mode(instr);
  int32_t X1_OPC = -1, X1_SUB1_OPC = -1, X1_SUB2_OPC = -1;
  int32_t Y1_OPC = -1, Y1_SUB1_OPC = -1, Y1_SUB2_OPC = -1;

  if (mode != 0) {
    Y1_OPC = get_Opcode_Y1(instr);
    Y1_SUB1_OPC = get_RRROpcodeExtension_Y1(instr);
    Y1_SUB2_OPC = get_UnaryOpcodeExtension_Y1(instr);
  } else {
    X1_OPC = get_Opcode_X1(instr);
    X1_SUB1_OPC = get_RRROpcodeExtension_X1(instr);
    X1_SUB2_OPC = get_UnaryOpcodeExtension_X1(instr);
  }

  // Checks if the instruction is a jr.
  return (X1_OPC == RRR_0_OPCODE_X1 && X1_SUB1_OPC == UNARY_RRR_0_OPCODE_X1
          && X1_SUB2_OPC == JR_UNARY_OPCODE_X1)
         || (Y1_OPC == RRR_1_OPCODE_Y1 && Y1_SUB1_OPC == UNARY_RRR_1_OPCODE_Y1
             && Y1_SUB2_OPC == JR_UNARY_OPCODE_Y1);
}

bool Assembler::IsSHL16INSLI(Instr instr) {
  uint32_t mode   = get_Mode(instr);
  int32_t X0_OPC = -1, X1_OPC = -1;

  if (mode != 0)
    return false;

  X0_OPC = get_Opcode_X0(instr);
  X1_OPC = get_Opcode_X1(instr);

  // Checks if the instruction is a shl16insli which
  // is used for constant loading.
  return X0_OPC == SHL16INSLI_OPCODE_X0 || X1_OPC == SHL16INSLI_OPCODE_X1;
}

bool Assembler::IsMOVELI(Instr instr) {
  uint32_t mode   = get_Mode(instr);
  int32_t X0_OPC = -1, X1_OPC = -1;

  if (mode != 0)
    return false;

  X0_OPC = get_Opcode_X0(instr);
  X1_OPC = get_Opcode_X1(instr);

  // Checks if the instruction is a moveli which is
  // used for constant loading and is actually alias
  // of addli.
  return X0_OPC == ADDLI_OPCODE_X0 || X1_OPC == ADDLI_OPCODE_X1;
}

Assembler::Assembler(Isolate* isolate, void* buffer, int buffer_size)
    : AssemblerBase(isolate, buffer, buffer_size),
      positions_recorder_(this) {
  reloc_info_writer.Reposition(buffer_ + buffer_size_, pc_);

  last_trampoline_pool_end_ = 0;
  no_trampoline_pool_before_ = 0;
  trampoline_pool_blocked_nesting_ = 0;
  // We leave space (16 * kTrampolineSlotsSize)
  // for BlockTrampolinePoolScope buffer.
  next_buffer_check_ = kMaxBranchOffset - kTrampolineSlotsSize * 16;
  internal_trampoline_exception_ = false;
  last_bound_pos_ = 0;

  trampoline_emitted_ = false;
  unbound_labels_count_ = 0;
}

void Assembler::st(const Register& rd, const MemOperand& rs, int line) {
  ASSERT(rd.is_valid() && rs.rm().is_valid() && is_int16(rs.offset_));
  if (rs.offset_ != 0) {
    Instr instr = ADDLI_X1 | DEST_X1(tt.code())
                           | SRCA_X1(rs.rm().code()) | IMM16_X1(rs.offset_);
    emit(instr, line);
    instr = ST_X1 | SRCA_X1(tt.code()) | SRCB_X1(rd.code());
    emit(instr, line);
  } else
    st(rd, rs.rm(), line);
}

void Assembler::st(const Register& rd, const Register& rs, int line) {
  ASSERT(rd.is_valid() && rs.is_valid());
  Instr instr = ST_X1 | SRCA_X1(rs.code()) | SRCB_X1(rd.code());
  emit(instr, line);
}

void Assembler::ld(const Register& rd, const MemOperand& rs, int line) {
  ASSERT(rd.is_valid() && rs.rm().is_valid() && is_int16(rs.offset_));
  if (rs.offset_ != 0) {
    Instr instr = ADDLI_X1 | DEST_X1(tt.code())
                           | SRCA_X1(rs.rm().code()) | IMM16_X1(rs.offset_);
    emit(instr, line);
    instr = LD_X1 | DEST_X1(rd.code()) | SRCA_X1(tt.code());
    emit(instr, line);
  } else
    ld(rd, rs.rm(), line);
}

void Assembler::ld(const Register& rd, const Register& rs, int line) {
  ASSERT(rd.is_valid() && rs.is_valid());
  Instr instr = LD_X1 | DEST_X1(rd.code()) | SRCA_X1(rs.code());
  emit(instr, line);
}

void Assembler::add(const Register& rd, const Register& rsa, const Register& rsb, int line) {
  ASSERT(rd.is_valid() && rsa.is_valid() && rsb.is_valid());
  Instr instr = ADD_X1 | DEST_X1(rd.code())
	               | SRCA_X1(rsa.code()) | SRCB_X1(rsb.code());
  emit(instr, line);
}

void Assembler::sub(const Register& rd, const Register& rsa, const Register& rsb, int line) {
  ASSERT(rd.is_valid() && rsa.is_valid() && rsb.is_valid());
  Instr instr = SUB_X1 | DEST_X1(rd.code())
	               | SRCA_X1(rsa.code()) | SRCB_X1(rsb.code());
  emit(instr, line);
}

void Assembler::addi(const Register& rd, const Register& rs, int8_t imm, int line) {
  ASSERT(rd.is_valid() && rs.is_valid() && is_int8(imm));
  Instr instr = ADDI_X1 | DEST_X1(rd.code())
	                | SRCA_X1(rs.code()) | IMM8_X1(imm);
  emit(instr, line);
}

void Assembler::addli(const Register& rd, const Register& rs, int16_t imm, int line) {
  ASSERT(rd.is_valid() && rs.is_valid() && is_int16(imm));
  Instr instr = ADDLI_X1 | DEST_X1(rd.code())
	                 | SRCA_X1(rs.code()) | IMM16_X1(imm);
  emit(instr, line);
}

void Assembler::shl16insli(const Register& rd, const Register& rs, int16_t imm, int line) {
  ASSERT(rd.is_valid() && rs.is_valid() && is_int16(imm));
  Instr instr = SHL16INSLI_X1 | DEST_X1(rd.code())
	                      | SRCA_X1(rs.code()) | IMM16_X1(imm);
  emit(instr, line);
}

void Assembler::moveli(const Register& rd, int16_t imm, int line) {
  ASSERT(rd.is_valid() && is_int16(imm));
  Instr instr = MOVELI_X1 | DEST_X1(rd.code()) | IMM16_X1(imm);
  emit(instr, line);
}

void Assembler::move(const Register& rd, const Register& rs, int line) {
  ASSERT(rd.is_valid() && rs.is_valid());
  Instr instr = ADD_X1 | DEST_X1(rd.code())
	               | SRCA_X1(zero.code()) | SRCB_X1(rs.code());
  emit(instr, line);
}

void Assembler::GetCode(CodeDesc* desc) {
  ASSERT(pc_ <= reloc_info_writer.pos());  // No overlap.
  // Set up code descriptor.
  desc->buffer = buffer_;
  desc->buffer_size = buffer_size_;
  desc->instr_size = pc_offset();
  desc->reloc_size = (buffer_ + buffer_size_) - reloc_info_writer.pos();
}

void Assembler::print(Label* L) {
  if (L->is_unused()) {
    PrintF("unused label\n");
  } else if (L->is_bound()) {
    PrintF("bound label to %d\n", L->pos());
  } else if (L->is_linked()) {
    Label l = *L;
    PrintF("unbound label");
    while (l.is_linked()) {
      PrintF("@ %d ", l.pos());
      Instr instr = instr_at(l.pos());
      if ((instr & ~kImm16Mask) == 0) {
        PrintF("value\n");
      } else {
        PrintF("0x%08lx\n", (long)instr);
      }
      next(&l);
    }
  } else {
    PrintF("label in inconsistent state (pos = %d)\n", L->pos_);
  }
}

const int kEndOfChain = -8;

int Assembler::target_at(int32_t pos) {
  Instr instr = instr_at(pos);
  if ((instr & ~kImm16Mask) == 0) {
    // Emitted label constant, not part of a branch.
    if (instr == 0) {
       return kEndOfChain;
     } else {
       int32_t imm18 =((instr & static_cast<int32_t>(kImm16Mask)) << 16) >> 14;
       return (imm18 + pos);
     }
  }
  // Check we have a branch or jump instruction.
  printf("==> ");
  print_insn_tilegx((unsigned char *)&instr);
  ASSERT(IsBranch(instr) || IsJ(instr) || IsMOVELI(instr));
  // Do NOT change this to << 3. We rely on arithmetic shifts here, assuming
  // the compiler uses arithmectic shifts for signed integers.
  if (IsBranch(instr)) {
    int32_t imm20 = (((int)get_BrOff_X1(instr)) << 15) >> 12;

    if (imm20 == kEndOfChain) {
      // EndOfChain sentinel is returned directly, not relative to pc or pos.
      return kEndOfChain;
    } else {
      return pos + imm20;
    }
  } else if (IsMOVELI(instr)) {
    Instr instr_moveli = instr_at(pos + 0 * Assembler::kInstrSize);
    Instr instr_shl16insli_0 = instr_at(pos + 1 * Assembler::kInstrSize);
    Instr instr_shl16insli_1 = instr_at(pos + 2 * Assembler::kInstrSize);
    ASSERT(IsMOVELI(instr_moveli));
    ASSERT(IsMOVELI(instr_shl16insli_0));
    ASSERT(IsMOVELI(instr_shl16insli_1));
    int64_t imm = ((int64_t)get_Imm16_X1(instr_moveli)) << 32; 
    imm |= get_Imm16_X1(instr_shl16insli_0) << 16;
    imm |= get_Imm16_X1(instr_shl16insli_1);

    if (imm == kEndOfJumpChain) {
      // EndOfChain sentinel is returned directly, not relative to pc or pos.
      return kEndOfChain;
    } else {
      uint64_t instr_address = reinterpret_cast<int64_t>(buffer_ + pos);
      int64_t delta = instr_address - imm;
      ASSERT(pos > delta);
      return pos - delta;
    }
  } else {
    int64_t imm30 = get_JumpOff_X1(instr) << 3;
    if (imm30 == kEndOfJumpChain) {
      // EndOfChain sentinel is returned directly, not relative to pc or pos.
      return kEndOfChain;
    } else {
      uint64_t instr_address = reinterpret_cast<int64_t>(buffer_ + pos);
      int64_t delta = instr_address - imm30;
      ASSERT(pos > delta);
      return pos - delta;
    }
  }
}

void Assembler::target_at_put(int32_t pos, int32_t target_pos) {
  Instr instr = instr_at(pos);
  if ((instr & ~kImm16Mask) == 0) {
    ASSERT(target_pos == kEndOfChain || target_pos >= 0);
    // Emitted label constant, not part of a branch.
    // Make label relative to Code* of generated Code object.
    instr_at_put(pos, target_pos + (Code::kHeaderSize - kHeapObjectTag));
    return;
  }

  ASSERT(IsBranch(instr) || IsJ(instr) || IsMOVELI(instr));
  if (IsBranch(instr)) {
    int32_t imm20 = target_pos - pos;
    ASSERT((imm20 & 7) == 0);

    instr &= ~kImm16Mask;
    int32_t imm17 = imm20 >> 3;
    ASSERT(is_intn(imm17, 17));

    instr_at_put(pos, (instr & (~create_BrOff_X1(-1))) | create_BrOff_X1(imm17));
  } else if (IsMOVELI(instr)) {
    // TileGX userspace address space is less than 48bit, although
    // we support 64bit in theory.
    Instr instr_moveli = instr_at(pos + 0 * Assembler::kInstrSize);
    Instr instr_shl16insli_0 = instr_at(pos + 1 * Assembler::kInstrSize);
    Instr instr_shl16insli_1 = instr_at(pos + 2 * Assembler::kInstrSize);
    ASSERT(IsSHL16INSLI(instr_shl16insli_0));
    ASSERT(IsSHL16INSLI(instr_shl16insli_1));
    uint64_t imm = reinterpret_cast<uint64_t>(buffer_) + target_pos;
    ASSERT((imm & 7) == 0);

    instr_moveli &= ~create_Imm16_X1(-1);
    instr_shl16insli_0 &= ~create_Imm16_X1(-1);
    instr_shl16insli_1 &= ~create_Imm16_X1(-1);

    instr_at_put(pos + 0 * Assembler::kInstrSize,
                 instr_moveli | create_Imm16_X1(imm >> 32));
    instr_at_put(pos + 1 * Assembler::kInstrSize,
                 instr_shl16insli_0 | create_Imm16_X1(imm >> 16));
    instr_at_put(pos + 2 * Assembler::kInstrSize,
                 instr_shl16insli_1 | create_Imm16_X1(imm));
  } else {
    uint32_t imm30 = reinterpret_cast<uint64_t>(buffer_) + target_pos;
    ASSERT((imm30 & 7) == 0);

    instr &= ~create_JumpOff_X1(-1);
    uint32_t imm27 = imm30 >> 3;
    ASSERT(is_uintn(imm27, 27));

    instr_at_put(pos, instr | create_JumpOff_X1(imm27));
  }
}

// Returns the next free trampoline entry.
void Assembler::bind_to(Label* L, int pos) {
  ASSERT(0 <= pos && pos <= pc_offset());  // Must have valid binding position.
  int32_t trampoline_pos = kInvalidSlotPos;
  if (L->is_linked() && !trampoline_emitted_) {
    unbound_labels_count_--;
    next_buffer_check_ += kTrampolineSlotsSize;
  }

  while (L->is_linked()) {
    int32_t fixup_pos = L->pos();
    int32_t dist = pos - fixup_pos;
    next(L);  // Call next before overwriting link with target at fixup_pos.
    Instr instr = instr_at(fixup_pos);
    if (IsBranch(instr)) {
      if (dist > kMaxBranchOffset) {
        if (trampoline_pos == kInvalidSlotPos) {
          trampoline_pos = get_trampoline_entry(fixup_pos);
          CHECK(trampoline_pos != kInvalidSlotPos);
        }
        ASSERT((trampoline_pos - fixup_pos) <= kMaxBranchOffset);
        target_at_put(fixup_pos, trampoline_pos);
        fixup_pos = trampoline_pos;
        dist = pos - fixup_pos;
      }
      target_at_put(fixup_pos, pos);
    } else {
      ASSERT(IsJ(instr) || IsMOVELI(instr) || IsEmittedConstant(instr));
      target_at_put(fixup_pos, pos);
    }
  }
  L->bind_to(pos);

  // Keep track of the last bound label so we don't eliminate any instructions
  // before a bound label.
  if (pos > last_bound_pos_)
    last_bound_pos_ = pos;
}

void Assembler::next(Label* L) {
  ASSERT(L->is_linked());
  int link = target_at(L->pos());
  if (link == kEndOfChain) {
    L->Unuse();
  } else {
    ASSERT(link >= 0);
    L->link_to(link);
  }
}

void Assembler::bind(Label* L) {
  ASSERT(!L->is_bound());
  bind_to(L, pc_offset());
}

Address Assembler::target_address_at(Address pc) {
  printf("[%s:%d]\n", __FUNCTION__, __LINE__);
  abort();
}

void Assembler::set_target_address_at(Address pc, Address target) {
  printf("[%s:%d]\n", __FUNCTION__, __LINE__);
  abort();
}

void Assembler::Align(int m) {
  UNREACHABLE();
}

void Assembler::db(uint8_t data) {
  UNREACHABLE();
}


void Assembler::dd(uint32_t data) {
  UNREACHABLE();
}

bool Assembler::IsNop(Instr instr, unsigned int type) {
  UNREACHABLE();
  return false;
}

const int RelocInfo::kApplyMask = RelocInfo::kCodeTargetMask |
                                  1 << RelocInfo::INTERNAL_REFERENCE;

int Assembler::RelocateInternalReference(byte* pc, intptr_t pc_delta) {
	UNIMPLEMENTED();
	return -1;
}

void Assembler::RecordComment(const char* msg) {
	UNIMPLEMENTED();
}

MemOperand::MemOperand(Register rm, int64_t offset) : Operand(rm) {
  offset_ = offset;
}

Operand::Operand(Handle<Object> handle) {
#ifdef DEBUG
  Isolate* isolate = Isolate::Current();
#endif
  ALLOW_HANDLE_DEREF(isolate, "using and embedding raw address");
  rm_ = no_reg;
  // Verify all Objects referred by code are NOT in new space.
  Object* obj = *handle;
  ASSERT(!isolate->heap()->InNewSpace(obj));
  if (obj->IsHeapObject()) {
    imm64_ = reinterpret_cast<intptr_t>(handle.location());
    rmode_ = RelocInfo::EMBEDDED_OBJECT;
  } else {
    // No relocation needed.
    imm64_ = reinterpret_cast<intptr_t>(obj);
    rmode_ = RelocInfo::NONE32;
  }
}

bool RelocInfo::IsCodedSpecially() {
  // The deserializer needs to know whether a pointer is specially coded.  Being
  // specially coded on MIPS means that it is a lui/ori instruction, and that is
  // always the case inside code objects.
  UNIMPLEMENTED();
  return true;
}

void Assembler::RecordRelocInfo(RelocInfo::Mode rmode, intptr_t data) {
  // We do not try to reuse pool constants.
  RelocInfo rinfo(pc_, rmode, data, NULL);
  if (!RelocInfo::IsNone(rinfo.rmode())) {
    // Don't record external references unless the heap will be serialized.
    if (rmode == RelocInfo::EXTERNAL_REFERENCE) {
#ifdef DEBUG
      if (!Serializer::enabled()) {
        Serializer::TooLateToEnableNow();
      }
#endif
      if (!Serializer::enabled() && !emit_debug_code()) {
        return;
      }
    }
    ASSERT(buffer_space() >= kMaxRelocSize);  // Too late to grow buffer here.
    if (rmode == RelocInfo::CODE_TARGET_WITH_ID) {
      UNIMPLEMENTED();
    } else {
      reloc_info_writer.Write(&rinfo);
    }
  }

}


void Assembler::b(int32_t offset, int line) {
  beqz(zero, offset, line);
}

void Assembler::beqz(const Register& rs, int32_t offset, int line) {
  ASSERT(rs.is_valid());
  Instr instr = BEQZ_X1 | SRCA_X1(rs.code()) | BOFF_X1(offset);
  emit(instr, line);
}

void Assembler::bnez(const Register& rs, int32_t offset, int line) {
  ASSERT(rs.is_valid());
  Instr instr = BNEZ_X1 | SRCA_X1(rs.code()) | BOFF_X1(offset);
  emit(instr, line);
}

void Assembler::bgez(const Register& rs, int32_t offset, int line) {
  ASSERT(rs.is_valid());
  Instr instr = BGEZ_X1 | SRCA_X1(rs.code()) | BOFF_X1(offset);
  emit(instr, line);
}

void Assembler::bgtz(const Register& rs, int32_t offset, int line) {
  ASSERT(rs.is_valid());
  Instr instr = BGTZ_X1 | SRCA_X1(rs.code()) | BOFF_X1(offset);
  emit(instr, line);
}

void Assembler::blez(const Register& rs, int32_t offset, int line) {
  ASSERT(rs.is_valid());
  Instr instr = BLEZ_X1 | SRCA_X1(rs.code()) | BOFF_X1(offset);
  emit(instr, line);
}

void Assembler::bltz(const Register& rs, int32_t offset, int line) {
  ASSERT(rs.is_valid());
  Instr instr = BLTZ_X1 | SRCA_X1(rs.code()) | BOFF_X1(offset);
  emit(instr, line);
}

void Assembler::cmpeq(const Register& rd, const Register& rsa, const Register& rsb, int line) {
  ASSERT(rd.is_valid() && rsa.is_valid() && rsb.is_valid());
  Instr instr = CMPEQ_X1 | DEST_X1(rd.code()) | SRCA_X1(rsa.code()) | SRCB_X1(rsb.code());
  emit(instr, line);
}

void Assembler::cmpne(const Register& rd, const Register& rsa, const Register& rsb, int line) {
  ASSERT(rd.is_valid() && rsa.is_valid() && rsb.is_valid());
  Instr instr = CMPNE_X1 | DEST_X1(rd.code()) | SRCA_X1(rsa.code()) | SRCB_X1(rsb.code());
  emit(instr, line);
}

void Assembler::cmplts(const Register& rd, const Register& rsa, const Register& rsb, int line) {
  ASSERT(rd.is_valid() && rsa.is_valid() && rsb.is_valid());
  Instr instr = CMPLTS_X1 | DEST_X1(rd.code()) | SRCA_X1(rsa.code()) | SRCB_X1(rsb.code());
  emit(instr, line);
}

void Assembler::cmpltsi(const Register& rd, const Register& rsa, int8_t imm, int line) {
  ASSERT(rd.is_valid() && rsa.is_valid() && is_int8(imm));
  Instr instr = CMPLTSI_X1 | DEST_X1(rd.code()) | SRCA_X1(rsa.code()) | IMM8_X1(imm);
  emit(instr, line);
}

void Assembler::cmples(const Register& rd, const Register& rsa, const Register& rsb, int line) {
  ASSERT(rd.is_valid() && rsa.is_valid() && rsb.is_valid());
  Instr instr = CMPLES_X1 | DEST_X1(rd.code()) | SRCA_X1(rsa.code()) | SRCB_X1(rsb.code());
  emit(instr, line);
}

void Assembler::cmpltu(const Register& rd, const Register& rsa, const Register& rsb, int line) {
  ASSERT(rd.is_valid() && rsa.is_valid() && rsb.is_valid());
  Instr instr = CMPLTU_X1 | DEST_X1(rd.code()) | SRCA_X1(rsa.code()) | SRCB_X1(rsb.code());
  emit(instr, line);
}

void Assembler::cmpltui(const Register& rd, const Register& rsa, int8_t imm, int line) {
  ASSERT(rd.is_valid() && rsa.is_valid() && is_int8(imm));
  Instr instr = CMPLTUI_X1 | DEST_X1(rd.code()) | SRCA_X1(rsa.code()) | IMM8_X1(imm);
  emit(instr, line);
}

void Assembler::cmpleu(const Register& rd, const Register& rsa, const Register& rsb, int line) {
  ASSERT(rd.is_valid() && rsa.is_valid() && rsb.is_valid());
  Instr instr = CMPLEU_X1 | DEST_X1(rd.code()) | SRCA_X1(rsa.code()) | SRCB_X1(rsb.code());
  emit(instr, line);
}

int32_t Assembler::branch_offset(Label* L, bool jump_elimination_allowed) {
  int32_t target_pos;

  if (L->is_bound()) {
    target_pos = L->pos();
  } else {
    if (L->is_linked()) {
      target_pos = L->pos();
      L->link_to(pc_offset());
    } else {
      L->link_to(pc_offset());
      return kEndOfChain;
    }
  }

  int32_t offset = target_pos - pc_offset();
  ASSERT((offset & 7) == 0);
  ASSERT(is_intn(offset >> 3, 17));

  return offset;
}

void Assembler::GrowBuffer() {
	UNIMPLEMENTED();
}

bool Assembler::is_near(Label* L) {
  if (L->is_bound()) {
    return (pc_offset() - L->pos()) < ((1 << 18) - 1);
  }
  return false;
}

uint32_t Assembler::jump_address(Label* L) {
  int64_t target_pos;

  if (L->is_bound()) {
    target_pos = L->pos();
  } else {
    if (L->is_linked()) {
      target_pos = L->pos();  // L's link.
      L->link_to(pc_offset());
    } else {
      L->link_to(pc_offset());
      return kEndOfJumpChain;
    }
  }

  uint64_t imm = reinterpret_cast<uint64_t>(buffer_) + target_pos;
  ASSERT((imm & 7) == 0);

  return imm;
}

void Assembler::jr(Register rs, int line) {
  if (rs.is(lr)) {
    positions_recorder()->WriteRecordedPositions();
  }

  Instr instr = JR_X1 | SRCA_X1(rs.code());
  emit(instr, line);
}

void Assembler::jalr(Register rs, int line) {
  positions_recorder()->WriteRecordedPositions();
  Instr instr = JALR_X1 | SRCA_X1(rs.code());
  emit(instr, line);
}

// We have to use a temporary register for things that can be relocated even
// if they can be encoded in the MIPS's 16 bits of immediate-offset instruction
// space.  There is no guarantee that the relocated location can be similarly
// encoded.
bool Assembler::MustUseReg(RelocInfo::Mode rmode) {
  return !RelocInfo::IsNone(rmode);
}

int ToNumber(Register reg) {
  ASSERT(reg.is_valid());
  const int kNumbers[] = {
    0,
    1,
    2,
    3,
    4,
    5,
    6,
    7,
    8,
    9,
    10,
    11,
    12,
    13,
    14,
    15,
    16,
    17,
    18,
    19,
    20,
    21,
    22,
    23,
    24,
    25,
    26,
    27,
    28,
    29,
    30,
    31,
    32,
    33,
    34,
    35,
    36,
    37,
    38,
    39,
    40,
    41,
    42,
    43,
    44,
    45,
    46,
    47,
    48,
    49,
    50,
    51,
    52,
    53,
    54,
    55,
    56,
    57,
    58,
    59,
    60,
    61,
    62,
    63,
  };
  return kNumbers[reg.code()];
}

Register ToRegister(int num) {
  ASSERT(num >= 0 && num < kNumRegisters);
  const Register kRegisters[] = {
    r0, r1, r2, r3, r4, r5, r6, r7,
    r8, r9, r10, r11, r12, r13, r14, r15,
    r16, r17, r18, r19, r20, r21, r22, r23,
    r24, r25, r26, r27, r28, r29, r30, r31,
    r32, r33, r34, r35, r36, r37, r38, r39,
    r40, r41, r42, r43, r44, r45, r46, r47,
    r48, r49, r50, r51, r52, r53, r54, r55,
    r56, r57, r58, r59, r60, r61, r62, r63
  };
  return kRegisters[num];
}

// Returns the next free trampoline entry.
int32_t Assembler::get_trampoline_entry(int32_t pos) {
  int32_t trampoline_entry = kInvalidSlotPos;

  if (!internal_trampoline_exception_) {
    if (trampoline_.start() > pos) {
     trampoline_entry = trampoline_.take_slot();
    }

    if (kInvalidSlotPos == trampoline_entry) {
      internal_trampoline_exception_ = true;
    }
  }
  return trampoline_entry;
}

uint32_t Assembler::GetLabelConst(Instr instr) {
  return instr & ~kImm16Mask;
}

bool Assembler::IsEmittedConstant(Instr instr) {
  uint32_t label_constant = GetLabelConst(instr);
  return label_constant == 0;  // Emitted label const in reg-exp engine.
}

} }  // namespace v8::internal

#endif  // V8_TARGET_ARCH_TILEGX
