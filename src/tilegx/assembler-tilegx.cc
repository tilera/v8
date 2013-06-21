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

Assembler::Assembler(Isolate* isolate, void* buffer, int buffer_size)
    : AssemblerBase(isolate, buffer, buffer_size),
      positions_recorder_(this) {
  reloc_info_writer.Reposition(buffer_ + buffer_size_, pc_);
}

void Assembler::st(const Register& rd, const MemOperand& rs) {
  ASSERT(rd.is_valid() && rs.rm().is_valid() && is_int16(rs.offset_));
  if (rs.offset_ != 0) {
    Instr instr = ADDLI_X1 | DEST_X1(tt.code())
                           | SRCA_X1(rs.rm().code()) | IMM16_X1(rs.offset_);
    emit(instr);
    instr = ST_X1 | SRCA_X1(tt.code()) | SRCB_X1(rd.code());
    emit(instr);
  } else
    st(rd, rs.rm());
}

void Assembler::st(const Register& rd, const Register& rs) {
  ASSERT(rd.is_valid() && rs.is_valid());
  Instr instr = ST_X1 | SRCA_X1(rs.code()) | SRCB_X1(rd.code());
  emit(instr);
}

void Assembler::ld(const Register& rd, const MemOperand& rs) {
  ASSERT(rd.is_valid() && rs.rm().is_valid() && is_int16(rs.offset_));
  if (rs.offset_ != 0) {
    Instr instr = ADDLI_X1 | DEST_X1(tt.code())
                           | SRCA_X1(rs.rm().code()) | IMM16_X1(rs.offset_);
    emit(instr);
    instr = LD_X1 | DEST_X1(rd.code()) | SRCA_X1(tt.code());
    emit(instr);
  } else
    ld(rd, rs.rm());
}

void Assembler::ld(const Register& rd, const Register& rs) {
  ASSERT(rd.is_valid() && rs.is_valid());
  Instr instr = LD_X1 | DEST_X1(rd.code()) | SRCA_X1(rs.code());
  emit(instr);
}

void Assembler::add(const Register& rd, const Register& rsa, const Register& rsb) {
  ASSERT(rd.is_valid() && rsa.is_valid() && rsb.is_valid());
  Instr instr = ADD_X1 | DEST_X1(rd.code())
	               | SRCA_X1(rsa.code()) | SRCB_X1(rsb.code());
  emit(instr);
}

void Assembler::sub(const Register& rd, const Register& rsa, const Register& rsb) {
  ASSERT(rd.is_valid() && rsa.is_valid() && rsb.is_valid());
  Instr instr = SUB_X1 | DEST_X1(rd.code())
	               | SRCA_X1(rsa.code()) | SRCB_X1(rsb.code());
  emit(instr);
}

void Assembler::addi(const Register& rd, const Register& rs, int8_t imm) {
  ASSERT(rd.is_valid() && rs.is_valid() && is_int8(imm));
  Instr instr = ADDI_X1 | DEST_X1(rd.code())
	                | SRCA_X1(rs.code()) | IMM8_X1(imm);
  emit(instr);
}

void Assembler::addli(const Register& rd, const Register& rs, int16_t imm) {
  ASSERT(rd.is_valid() && rs.is_valid() && is_int16(imm));
  Instr instr = ADDLI_X1 | DEST_X1(rd.code())
	                 | SRCA_X1(rs.code()) | IMM16_X1(imm);
  emit(instr);
}

void Assembler::shl16insli(const Register& rd, const Register& rs, int16_t imm) {
  ASSERT(rd.is_valid() && rs.is_valid() && is_int16(imm));
  Instr instr = SHL16INSLI_X1 | DEST_X1(rd.code())
	                      | SRCA_X1(rs.code()) | IMM16_X1(imm);
  emit(instr);
}

void Assembler::moveli(const Register& rd, int16_t imm) {
  ASSERT(rd.is_valid() && is_int16(imm));
  Instr instr = MOVELI_X1 | DEST_X1(rd.code()) | IMM16_X1(imm);
  emit(instr);
}

void Assembler::move(const Register& rd, const Register& rs) {
  ASSERT(rd.is_valid() && rs.is_valid());
  Instr instr = ADD_X1 | DEST_X1(rd.code())
	               | SRCA_X1(zero.code()) | SRCB_X1(rs.code());
  emit(instr);
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
      ASSERT(IsJ(instr) || IsLui(instr) || IsEmittedConstant(instr));
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

const int kEndOfChain = -8;

void Assembler::b(int32_t offset) {
  beqz(zero, offset);
}

void Assembler::beqz(Register rs, int32_t offset) {
  ASSERT(rs.is_valid());
  Instr instr = BEQZ_X1 | SRCA_X1(rs.code()) | BOFF_X1(offset);
  emit(instr);
}

void Assembler::bnez(Register rs, int32_t offset) {
  ASSERT(rs.is_valid());
  Instr instr = BNEZ_X1 | SRCA_X1(rs.code()) | BOFF_X1(offset);
  emit(instr);
}

void Assembler::bgez(Register rs, int32_t offset) {
  ASSERT(rs.is_valid());
  Instr instr = BGEZ_X1 | SRCA_X1(rs.code()) | BOFF_X1(offset);
  emit(instr);
}

void Assembler::bgtz(Register rs, int32_t offset) {
  ASSERT(rs.is_valid());
  Instr instr = BGTZ_X1 | SRCA_X1(rs.code()) | BOFF_X1(offset);
  emit(instr);
}

void Assembler::blez(Register rs, int32_t offset) {
  ASSERT(rs.is_valid());
  Instr instr = BLEZ_X1 | SRCA_X1(rs.code()) | BOFF_X1(offset);
  emit(instr);
}

void Assembler::bltz(Register rs, int32_t offset) {
  ASSERT(rs.is_valid());
  Instr instr = BLTZ_X1 | SRCA_X1(rs.code()) | BOFF_X1(offset);
  emit(instr);
}

void Assembler::cmpeq(const Register& rd, const Register& rsa, const Register& rsb) {
  ASSERT(rd.is_valid() && rsa.is_valid() && rsb.is_valid());
  Instr instr = CMPEQ_X1 | DEST_X1(rd.code()) | SRCA_X1(rsa.code()) | SRCB_X1(rsb.code());
  emit(instr);
}

void Assembler::cmpne(const Register& rd, const Register& rsa, const Register& rsb) {
  ASSERT(rd.is_valid() && rsa.is_valid() && rsb.is_valid());
  Instr instr = CMPNE_X1 | DEST_X1(rd.code()) | SRCA_X1(rsa.code()) | SRCB_X1(rsb.code());
  emit(instr);
}

void Assembler::cmplts(const Register& rd, const Register& rsa, const Register& rsb) {
  ASSERT(rd.is_valid() && rsa.is_valid() && rsb.is_valid());
  Instr instr = CMPLTS_X1 | DEST_X1(rd.code()) | SRCA_X1(rsa.code()) | SRCB_X1(rsb.code());
  emit(instr);
}

void Assembler::cmpltsi(const Register& rd, const Register& rsa, int8_t imm) {
  ASSERT(rd.is_valid() && rsa.is_valid() && is_int8(imm));
  Instr instr = CMPLTSI_X1 | DEST_X1(rd.code()) | SRCA_X1(rsa.code()) | IMM8_X1(imm);
  emit(instr);
}

void Assembler::cmples(const Register& rd, const Register& rsa, const Register& rsb) {
  ASSERT(rd.is_valid() && rsa.is_valid() && rsb.is_valid());
  Instr instr = CMPLES_X1 | DEST_X1(rd.code()) | SRCA_X1(rsa.code()) | SRCB_X1(rsb.code());
  emit(instr);
}

void Assembler::cmpltu(const Register& rd, const Register& rsa, const Register& rsb) {
  ASSERT(rd.is_valid() && rsa.is_valid() && rsb.is_valid());
  Instr instr = CMPLTU_X1 | DEST_X1(rd.code()) | SRCA_X1(rsa.code()) | SRCB_X1(rsb.code());
  emit(instr);
}

void Assembler::cmpltui(const Register& rd, const Register& rsa, int8_t imm) {
  ASSERT(rd.is_valid() && rsa.is_valid() && is_int8(imm));
  Instr instr = CMPLTUI_X1 | DEST_X1(rd.code()) | SRCA_X1(rsa.code()) | IMM8_X1(imm);
  emit(instr);
}

void Assembler::cmpleu(const Register& rd, const Register& rsa, const Register& rsb) {
  ASSERT(rd.is_valid() && rsa.is_valid() && rsb.is_valid());
  Instr instr = CMPLEU_X1 | DEST_X1(rd.code()) | SRCA_X1(rsa.code()) | SRCB_X1(rsb.code());
  emit(instr);
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
    return (pc_offset() - L->pos()) < (1 << 18 - 1);
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

void Assembler::jr(Register rs) {
  if (rs.is(lr)) {
    positions_recorder()->WriteRecordedPositions();
  }

  Instr instr = JR_X1 | SRCA_X1(rs.code());
  emit(instr);
}

} }  // namespace v8::internal

#endif  // V8_TARGET_ARCH_TILEGX
