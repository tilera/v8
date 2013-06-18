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
	UNIMPLEMENTED();
}

Assembler::Assembler(Isolate* isolate, void* buffer, int buffer_size)
    : AssemblerBase(isolate, buffer, buffer_size),
      positions_recorder_(this) {
	      UNIMPLEMENTED();
}

void Assembler::st(Register rd, Register rs) {
	UNREACHABLE();
}
void Assembler::ld(Register rd, Register rs) {
	UNREACHABLE();
}
void Assembler::addi(Register rd, Register rs, int8_t imm) {
	UNREACHABLE();
}
void Assembler::move(Register rt, Register rs) {
	UNREACHABLE();
}

void Assembler::GetCode(CodeDesc* desc) {
  ASSERT(pc_ <= reloc_info_writer.pos());  // No overlap.
  // Set up code descriptor.
  desc->buffer = buffer_;
  desc->buffer_size = buffer_size_;
  desc->instr_size = pc_offset();
  desc->reloc_size = (buffer_ + buffer_size_) - reloc_info_writer.pos();
}

void Assembler::bind(Label* L) {
  printf("[%s:%d]\n", __FUNCTION__, __LINE__);
  abort();
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

MemOperand::MemOperand(Register rm) : Operand(rm) {
}

bool RelocInfo::IsCodedSpecially() {
  // The deserializer needs to know whether a pointer is specially coded.  Being
  // specially coded on MIPS means that it is a lui/ori instruction, and that is
  // always the case inside code objects.
  UNIMPLEMENTED();
  return true;
}

void Assembler::RecordRelocInfo(RelocInfo::Mode rmode, intptr_t data) {
	UNIMPLEMENTED();
}

void Assembler::GrowBuffer() {
	UNIMPLEMENTED();
}

} }  // namespace v8::internal

#endif  // V8_TARGET_ARCH_TILEGX
