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

#include "tilegx/lithium-codegen-tilegx.h"
#include "tilegx/lithium-gap-resolver-tilegx.h"
#include "code-stubs.h"
#include "stub-cache.h"

namespace v8 {
namespace internal {


class SafepointGenerator : public CallWrapper {
 public:
  SafepointGenerator(LCodeGen* codegen,
                     LPointerMap* pointers,
                     Safepoint::DeoptMode mode)
      : codegen_(codegen),
        pointers_(pointers),
        deopt_mode_(mode) { }
  virtual ~SafepointGenerator() { }

  virtual void BeforeCall(int call_size) const { }

  virtual void AfterCall() const {
    codegen_->RecordSafepoint(pointers_, deopt_mode_);
  }

 private:
  LCodeGen* codegen_;
  LPointerMap* pointers_;
  Safepoint::DeoptMode deopt_mode_;
};


#define __ masm()->

bool LCodeGen::GenerateCode() {
	UNREACHABLE();
	return false;
}


void LCodeGen::FinishCode(Handle<Code> code) {  UNREACHABLE();  }


void LChunkBuilder::Abort(const char* reason) {  UNREACHABLE();  }


void LCodeGen::Comment(const char* format, ...) {  UNREACHABLE();  }


bool LCodeGen::GeneratePrologue() {  UNREACHABLE();  return false;}


bool LCodeGen::GenerateBody() {  UNREACHABLE();  return false;}


bool LCodeGen::GenerateDeferredCode() {  UNREACHABLE();  return false;}


bool LCodeGen::GenerateDeoptJumpTable() {  UNREACHABLE();  return false;}


bool LCodeGen::GenerateSafepointTable() {  UNREACHABLE();  return false;}


Register LCodeGen::ToRegister(int index) const {  UNREACHABLE();
  return Register::FromAllocationIndex(index);
}


DoubleRegister LCodeGen::ToDoubleRegister(int index) const {  UNREACHABLE();
  return Register::FromAllocationIndex(index);
}


Register LCodeGen::ToRegister(LOperand* op) const {  UNREACHABLE();
  return Register::FromAllocationIndex(0);
}


Register LCodeGen::EmitLoadRegister(LOperand* op, Register scratch) {  UNREACHABLE();
  return Register::FromAllocationIndex(0);
}


DoubleRegister LCodeGen::ToDoubleRegister(LOperand* op) const {  UNREACHABLE();
  return Register::FromAllocationIndex(0);
}

Handle<Object> LCodeGen::ToHandle(LConstantOperand* op) const {  UNREACHABLE(); 
  HConstant* constant = chunk_->LookupConstant(op);
  ASSERT(chunk_->LookupLiteralRepresentation(op).IsSmiOrTagged());
  return constant->handle();
}


bool LCodeGen::IsInteger32(LConstantOperand* op) const {  UNREACHABLE();  return false;}


bool LCodeGen::IsSmi(LConstantOperand* op) const {  UNREACHABLE();  return false;}


int LCodeGen::ToInteger32(LConstantOperand* op) const {  UNREACHABLE();  return -1;}


Smi* LCodeGen::ToSmi(LConstantOperand* op) const {  UNREACHABLE();  return NULL;}


double LCodeGen::ToDouble(LConstantOperand* op) const {  UNREACHABLE();  return 0.0;}


Operand LCodeGen::ToOperand(LOperand* op) {  UNREACHABLE();  return Operand(r0);}


MemOperand LCodeGen::ToMemOperand(LOperand* op) const {  UNREACHABLE();  return MemOperand(r0);}


MemOperand LCodeGen::ToHighMemOperand(LOperand* op) const {  UNREACHABLE();  return MemOperand(r0);}


void LCodeGen::WriteTranslation(LEnvironment* environment,
                                Translation* translation,
                                int* pushed_arguments_index,
                                int* pushed_arguments_count) {  UNREACHABLE();  }


void LCodeGen::AddToTranslation(Translation* translation,
                                LOperand* op,
                                bool is_tagged,
                                bool is_uint32,
                                bool arguments_known,
                                int arguments_index,
                                int arguments_count) {  UNREACHABLE();  }


void LCodeGen::CallCode(Handle<Code> code,
                        RelocInfo::Mode mode,
                        LInstruction* instr) {  UNREACHABLE();  }


void LCodeGen::CallCodeGeneric(Handle<Code> code,
                               RelocInfo::Mode mode,
                               LInstruction* instr,
                               SafepointMode safepoint_mode) {  UNREACHABLE();  }


void LCodeGen::CallRuntime(const Runtime::Function* function,
                           int num_arguments,
                           LInstruction* instr) {  UNREACHABLE();  }


void LCodeGen::CallRuntimeFromDeferred(Runtime::FunctionId id,
                                       int argc,
                                       LInstruction* instr) {  UNREACHABLE();  }


void LCodeGen::RegisterEnvironmentForDeoptimization(LEnvironment* environment,
                                                    Safepoint::DeoptMode mode) {

}


void LCodeGen::DeoptimizeIf(Condition cc,
                            LEnvironment* environment,
                            Deoptimizer::BailoutType bailout_type,
                            Register src1,
                            const Operand& src2) {  UNREACHABLE();  }


void LCodeGen::DeoptimizeIf(Condition cc,
                            LEnvironment* environment,
                            Register src1,
                            const Operand& src2) {  UNREACHABLE();  }


void LCodeGen::SoftDeoptimize(LEnvironment* environment,
                              Register src1,
                              const Operand& src2) {  UNREACHABLE();  }


void LCodeGen::RegisterDependentCodeForEmbeddedMaps(Handle<Code> code) {  UNREACHABLE();  }


void LCodeGen::PopulateDeoptimizationData(Handle<Code> code) {  UNREACHABLE();  }


int LCodeGen::DefineDeoptimizationLiteral(Handle<Object> literal) {  UNREACHABLE();  return -1;}


void LCodeGen::PopulateDeoptimizationLiteralsWithInlinedFunctions() {  UNREACHABLE();  }


void LCodeGen::RecordSafepointWithLazyDeopt(
    LInstruction* instr, SafepointMode safepoint_mode) {  UNREACHABLE();  }


void LCodeGen::RecordSafepoint(
    LPointerMap* pointers,
    Safepoint::Kind kind,
    int arguments,
    Safepoint::DeoptMode deopt_mode) {  UNREACHABLE();  }


void LCodeGen::RecordSafepoint(LPointerMap* pointers,
                               Safepoint::DeoptMode deopt_mode) {  UNREACHABLE();  }


void LCodeGen::RecordSafepoint(Safepoint::DeoptMode deopt_mode) {  UNREACHABLE();  }


void LCodeGen::RecordSafepointWithRegisters(LPointerMap* pointers,
                                            int arguments,
                                            Safepoint::DeoptMode deopt_mode) {  UNREACHABLE();  }


void LCodeGen::RecordSafepointWithRegistersAndDoubles(
    LPointerMap* pointers,
    int arguments,
    Safepoint::DeoptMode deopt_mode) {  UNREACHABLE();  }


void LCodeGen::RecordPosition(int position) {  UNREACHABLE();  }


void LCodeGen::DoLabel(LLabel* label) {  UNREACHABLE();  }


void LCodeGen::DoParallelMove(LParallelMove* move) {  UNREACHABLE();  }


void LCodeGen::DoGap(LGap* gap) {  UNREACHABLE();  }


void LCodeGen::DoInstructionGap(LInstructionGap* instr) {  UNREACHABLE();  }


void LCodeGen::DoParameter(LParameter* instr) {  UNREACHABLE();  }


void LCodeGen::DoCallStub(LCallStub* instr) {  UNREACHABLE();  }


void LCodeGen::DoUnknownOSRValue(LUnknownOSRValue* instr) {  UNREACHABLE();  }


void LCodeGen::DoModI(LModI* instr) {  UNREACHABLE();  }


void LCodeGen::DoDivI(LDivI* instr) {  UNREACHABLE();  }


void LCodeGen::DoMultiplyAddD(LMultiplyAddD* instr) {  UNREACHABLE();  }


void LCodeGen::DoMulI(LMulI* instr) {  UNREACHABLE();  }


void LCodeGen::DoBitI(LBitI* instr) {  UNREACHABLE();  }


void LCodeGen::DoShiftI(LShiftI* instr) {  UNREACHABLE();  }


void LCodeGen::DoSubI(LSubI* instr) {  UNREACHABLE();  }


void LCodeGen::DoConstantI(LConstantI* instr) {  UNREACHABLE();  }


void LCodeGen::DoConstantD(LConstantD* instr) {  UNREACHABLE();  }


void LCodeGen::DoConstantT(LConstantT* instr) {  UNREACHABLE();  }


void LCodeGen::DoFixedArrayBaseLength(LFixedArrayBaseLength* instr) {  UNREACHABLE();  }


void LCodeGen::DoMapEnumLength(LMapEnumLength* instr) {  UNREACHABLE();  }


void LCodeGen::DoElementsKind(LElementsKind* instr) {  UNREACHABLE();  }


void LCodeGen::DoValueOf(LValueOf* instr) {  UNREACHABLE();  }


void LCodeGen::DoDateField(LDateField* instr) {  UNREACHABLE();  }


void LCodeGen::DoSeqStringSetChar(LSeqStringSetChar* instr) {  UNREACHABLE();  }


void LCodeGen::DoBitNotI(LBitNotI* instr) {  UNREACHABLE();  }


void LCodeGen::DoThrow(LThrow* instr) {  UNREACHABLE();  }


void LCodeGen::DoAddI(LAddI* instr) {  UNREACHABLE();  }


void LCodeGen::DoMathMinMax(LMathMinMax* instr) {  UNREACHABLE();  }


void LCodeGen::DoArithmeticD(LArithmeticD* instr) {  UNREACHABLE();  }


void LCodeGen::DoArithmeticT(LArithmeticT* instr) {  UNREACHABLE();  }


int LCodeGen::GetNextEmittedBlock() const {  UNREACHABLE();  return -1;}


void LCodeGen::EmitBranch(int left_block, int right_block,
                          Condition cc, Register src1, const Operand& src2) {  UNREACHABLE();  }


void LCodeGen::DoDebugBreak(LDebugBreak* instr) {  UNREACHABLE();  }


void LCodeGen::DoBranch(LBranch* instr) {  UNREACHABLE();  }


void LCodeGen::EmitGoto(int block) {  UNREACHABLE();  }


void LCodeGen::DoGoto(LGoto* instr) {  UNREACHABLE();  }


Condition LCodeGen::TokenToCondition(Token::Value op, bool is_unsigned) {  UNREACHABLE();  return al;}


void LCodeGen::DoCmpIDAndBranch(LCmpIDAndBranch* instr) {  UNREACHABLE();  }


void LCodeGen::DoCmpObjectEqAndBranch(LCmpObjectEqAndBranch* instr) {  UNREACHABLE();  }


void LCodeGen::DoCmpConstantEqAndBranch(LCmpConstantEqAndBranch* instr) {  UNREACHABLE();  }


Condition LCodeGen::EmitIsObject(Register input,
                                 Register temp1,
                                 Register temp2,
                                 Label* is_not_object,
                                 Label* is_object) {  UNREACHABLE();  return al;}


void LCodeGen::DoIsObjectAndBranch(LIsObjectAndBranch* instr) {  UNREACHABLE();  }


Condition LCodeGen::EmitIsString(Register input,
                                 Register temp1,
                                 Label* is_not_string) {  UNREACHABLE();  return al;}


void LCodeGen::DoIsStringAndBranch(LIsStringAndBranch* instr) {  UNREACHABLE();  }


void LCodeGen::DoIsSmiAndBranch(LIsSmiAndBranch* instr) {  UNREACHABLE();  }


void LCodeGen::DoIsUndetectableAndBranch(LIsUndetectableAndBranch* instr) {  UNREACHABLE();  }


void LCodeGen::DoStringCompareAndBranch(LStringCompareAndBranch* instr) {  UNREACHABLE();  }


void LCodeGen::DoHasInstanceTypeAndBranch(LHasInstanceTypeAndBranch* instr) {  UNREACHABLE();  }


void LCodeGen::DoGetCachedArrayIndex(LGetCachedArrayIndex* instr) {  UNREACHABLE();  }


void LCodeGen::DoHasCachedArrayIndexAndBranch(
    LHasCachedArrayIndexAndBranch* instr) {  UNREACHABLE();  }


// Branches to a label or falls through with the answer in flags.  Trashes
// the temp registers, but not the input.
void LCodeGen::EmitClassOfTest(Label* is_true,
                               Label* is_false,
                               Handle<String>class_name,
                               Register input,
                               Register temp,
                               Register temp2) {  UNREACHABLE();  }


void LCodeGen::DoClassOfTestAndBranch(LClassOfTestAndBranch* instr) {  UNREACHABLE();  }


void LCodeGen::DoCmpMapAndBranch(LCmpMapAndBranch* instr) {  UNREACHABLE();  }


void LCodeGen::DoInstanceOf(LInstanceOf* instr) {  UNREACHABLE();  }


void LCodeGen::DoInstanceOfKnownGlobal(LInstanceOfKnownGlobal* instr) {  UNREACHABLE();  }


void LCodeGen::DoDeferredInstanceOfKnownGlobal(LInstanceOfKnownGlobal* instr,
                                               Label* map_check) {  UNREACHABLE();  }


void LCodeGen::DoInstanceSize(LInstanceSize* instr) {  UNREACHABLE();  }


void LCodeGen::DoCmpT(LCmpT* instr) {  UNREACHABLE();  }


void LCodeGen::DoReturn(LReturn* instr) {  UNREACHABLE();  }


void LCodeGen::DoLoadGlobalCell(LLoadGlobalCell* instr) {  UNREACHABLE();  }


void LCodeGen::DoLoadGlobalGeneric(LLoadGlobalGeneric* instr) {  UNREACHABLE();  }


void LCodeGen::DoStoreGlobalCell(LStoreGlobalCell* instr) {  UNREACHABLE();  }


void LCodeGen::DoStoreGlobalGeneric(LStoreGlobalGeneric* instr) {  UNREACHABLE();  }


void LCodeGen::DoLoadContextSlot(LLoadContextSlot* instr) {  UNREACHABLE();  }


void LCodeGen::DoStoreContextSlot(LStoreContextSlot* instr) {  UNREACHABLE();  }


void LCodeGen::DoLoadNamedField(LLoadNamedField* instr) {  UNREACHABLE();  }


void LCodeGen::EmitLoadFieldOrConstantFunction(Register result,
                                               Register object,
                                               Handle<Map> type,
                                               Handle<String> name,
                                               LEnvironment* env) {  UNREACHABLE();  }


void LCodeGen::DoLoadNamedFieldPolymorphic(LLoadNamedFieldPolymorphic* instr) {  UNREACHABLE();  }


void LCodeGen::DoLoadNamedGeneric(LLoadNamedGeneric* instr) {  UNREACHABLE();  }


void LCodeGen::DoLoadFunctionPrototype(LLoadFunctionPrototype* instr) {  UNREACHABLE();  }


void LCodeGen::DoLoadExternalArrayPointer(
    LLoadExternalArrayPointer* instr) {  UNREACHABLE();  }


void LCodeGen::DoAccessArgumentsAt(LAccessArgumentsAt* instr) {  UNREACHABLE();  }


void LCodeGen::DoLoadKeyedExternalArray(LLoadKeyed* instr) {  UNREACHABLE();  }


void LCodeGen::DoLoadKeyedFixedDoubleArray(LLoadKeyed* instr) {  UNREACHABLE();  }


void LCodeGen::DoLoadKeyedFixedArray(LLoadKeyed* instr) {  UNREACHABLE();  }


void LCodeGen::DoLoadKeyed(LLoadKeyed* instr) {  UNREACHABLE();  }


MemOperand LCodeGen::PrepareKeyedOperand(Register key,
                                         Register base,
                                         bool key_is_constant,
                                         int constant_key,
                                         int element_size,
                                         int shift_size,
                                         int additional_index,
                                         int additional_offset) {  UNREACHABLE();  return MemOperand(r0);}


void LCodeGen::DoLoadKeyedGeneric(LLoadKeyedGeneric* instr) {  UNREACHABLE();  }


void LCodeGen::DoArgumentsElements(LArgumentsElements* instr) {  UNREACHABLE();  }


void LCodeGen::DoArgumentsLength(LArgumentsLength* instr) {  UNREACHABLE();  }


void LCodeGen::DoWrapReceiver(LWrapReceiver* instr) {  UNREACHABLE();  }

void LCodeGen::DoApplyArguments(LApplyArguments* instr) {  UNREACHABLE();  }


void LCodeGen::DoPushArgument(LPushArgument* instr) {  UNREACHABLE();  }


void LCodeGen::DoDrop(LDrop* instr) {  UNREACHABLE();  }


void LCodeGen::DoThisFunction(LThisFunction* instr) {  UNREACHABLE();  }


void LCodeGen::DoContext(LContext* instr) {  UNREACHABLE();  }


void LCodeGen::DoOuterContext(LOuterContext* instr) {  UNREACHABLE();  }


void LCodeGen::DoDeclareGlobals(LDeclareGlobals* instr) {  UNREACHABLE();  }


void LCodeGen::DoGlobalObject(LGlobalObject* instr) {  UNREACHABLE();  }


void LCodeGen::DoGlobalReceiver(LGlobalReceiver* instr) {  UNREACHABLE();  }


void LCodeGen::CallKnownFunction(Handle<JSFunction> function,
                                 int formal_parameter_count,
                                 int arity,
                                 LInstruction* instr,
                                 CallKind call_kind,
                                 A1State a1_state) {  UNREACHABLE();  }


void LCodeGen::DoCallConstantFunction(LCallConstantFunction* instr) {  UNREACHABLE();  }


void LCodeGen::DoDeferredMathAbsTaggedHeapNumber(LMathAbs* instr) {  UNREACHABLE();  }


void LCodeGen::EmitIntegerMathAbs(LMathAbs* instr) {  UNREACHABLE();  }


void LCodeGen::DoMathAbs(LMathAbs* instr) {  UNREACHABLE();  }


void LCodeGen::DoMathFloor(LMathFloor* instr) {  UNREACHABLE();  }


void LCodeGen::DoMathRound(LMathRound* instr) {  UNREACHABLE();  }


void LCodeGen::DoMathSqrt(LMathSqrt* instr) {  UNREACHABLE();  }


void LCodeGen::DoMathPowHalf(LMathPowHalf* instr) {  UNREACHABLE();  }


void LCodeGen::DoPower(LPower* instr) {  UNREACHABLE();  }


void LCodeGen::DoRandom(LRandom* instr) {  UNREACHABLE();  }

void LCodeGen::DoDeferredRandom(LRandom* instr) {  UNREACHABLE();  }


void LCodeGen::DoMathExp(LMathExp* instr) {  UNREACHABLE();  }


void LCodeGen::DoMathLog(LMathLog* instr) {  UNREACHABLE();  }


void LCodeGen::DoMathTan(LMathTan* instr) {  UNREACHABLE();  }


void LCodeGen::DoMathCos(LMathCos* instr) {  UNREACHABLE();  }


void LCodeGen::DoMathSin(LMathSin* instr) {  UNREACHABLE();  }


void LCodeGen::DoInvokeFunction(LInvokeFunction* instr) {  UNREACHABLE();  }


void LCodeGen::DoCallKeyed(LCallKeyed* instr) {  UNREACHABLE();  }


void LCodeGen::DoCallNamed(LCallNamed* instr) {  UNREACHABLE();  }


void LCodeGen::DoCallFunction(LCallFunction* instr) {  UNREACHABLE();  }


void LCodeGen::DoCallGlobal(LCallGlobal* instr) {  UNREACHABLE();  }


void LCodeGen::DoCallKnownGlobal(LCallKnownGlobal* instr) {  UNREACHABLE();  }


void LCodeGen::DoCallNew(LCallNew* instr) {  UNREACHABLE();  }


void LCodeGen::DoCallNewArray(LCallNewArray* instr) {  UNREACHABLE();  }


void LCodeGen::DoCallRuntime(LCallRuntime* instr) {  UNREACHABLE();  }


void LCodeGen::DoInnerAllocatedObject(LInnerAllocatedObject* instr) {  UNREACHABLE();  }


void LCodeGen::DoStoreNamedField(LStoreNamedField* instr) {  UNREACHABLE();  }


void LCodeGen::DoStoreNamedGeneric(LStoreNamedGeneric* instr) {  UNREACHABLE();  }


void LCodeGen::DoBoundsCheck(LBoundsCheck* instr) {  UNREACHABLE();  }


void LCodeGen::DoStoreKeyedExternalArray(LStoreKeyed* instr) {  UNREACHABLE();  }


void LCodeGen::DoStoreKeyedFixedDoubleArray(LStoreKeyed* instr) {  UNREACHABLE();  }


void LCodeGen::DoStoreKeyedFixedArray(LStoreKeyed* instr) {  UNREACHABLE();  }


void LCodeGen::DoStoreKeyed(LStoreKeyed* instr) {  UNREACHABLE();  }


void LCodeGen::DoStoreKeyedGeneric(LStoreKeyedGeneric* instr) {  UNREACHABLE();  }


void LCodeGen::DoTransitionElementsKind(LTransitionElementsKind* instr) {  UNREACHABLE();  }


void LCodeGen::DoTrapAllocationMemento(LTrapAllocationMemento* instr) {  UNREACHABLE();  }


void LCodeGen::DoStringAdd(LStringAdd* instr) {  UNREACHABLE();  }


void LCodeGen::DoStringCharCodeAt(LStringCharCodeAt* instr) {  UNREACHABLE();  }


void LCodeGen::DoDeferredStringCharCodeAt(LStringCharCodeAt* instr) {  UNREACHABLE();  }


void LCodeGen::DoStringCharFromCode(LStringCharFromCode* instr) {  UNREACHABLE();  }


void LCodeGen::DoDeferredStringCharFromCode(LStringCharFromCode* instr) {  UNREACHABLE();  }


void LCodeGen::DoStringLength(LStringLength* instr) {  UNREACHABLE();  }


void LCodeGen::DoInteger32ToDouble(LInteger32ToDouble* instr) {  UNREACHABLE();  }


void LCodeGen::DoInteger32ToSmi(LInteger32ToSmi* instr) {  UNREACHABLE();  }


void LCodeGen::DoUint32ToDouble(LUint32ToDouble* instr) {  UNREACHABLE();  }


void LCodeGen::DoNumberTagI(LNumberTagI* instr) {  UNREACHABLE();  }


void LCodeGen::DoNumberTagU(LNumberTagU* instr) {  UNREACHABLE();  }


void LCodeGen::DoDeferredNumberTagI(LInstruction* instr,
                                    LOperand* value,
                                    IntegerSignedness signedness) {  UNREACHABLE();  }


void LCodeGen::DoNumberTagD(LNumberTagD* instr) {  UNREACHABLE();  }


void LCodeGen::DoDeferredNumberTagD(LNumberTagD* instr) {  UNREACHABLE();  }


void LCodeGen::DoSmiTag(LSmiTag* instr) {  UNREACHABLE();  }


void LCodeGen::DoSmiUntag(LSmiUntag* instr) {  UNREACHABLE();  }


void LCodeGen::EmitNumberUntagD(Register input_reg,
                                DoubleRegister result_reg,
                                bool deoptimize_on_undefined,
                                bool deoptimize_on_minus_zero,
                                LEnvironment* env,
                                NumberUntagDMode mode) {  UNREACHABLE();  }


void LCodeGen::DoDeferredTaggedToI(LTaggedToI* instr) {  UNREACHABLE();  }


void LCodeGen::DoTaggedToI(LTaggedToI* instr) {  UNREACHABLE();  }


void LCodeGen::DoNumberUntagD(LNumberUntagD* instr) {  UNREACHABLE();  }


void LCodeGen::DoDoubleToI(LDoubleToI* instr) {  UNREACHABLE();  }


void LCodeGen::DoDoubleToSmi(LDoubleToSmi* instr) {  UNREACHABLE();  }


void LCodeGen::DoCheckSmi(LCheckSmi* instr) {  UNREACHABLE();  }


void LCodeGen::DoCheckNonSmi(LCheckNonSmi* instr) {  UNREACHABLE();  }


void LCodeGen::DoCheckInstanceType(LCheckInstanceType* instr) {  UNREACHABLE();  }


void LCodeGen::DoCheckFunction(LCheckFunction* instr) {  UNREACHABLE();  }


void LCodeGen::DoCheckMapCommon(Register map_reg,
                                Handle<Map> map,
                                LEnvironment* env) {  UNREACHABLE();  }


void LCodeGen::DoCheckMaps(LCheckMaps* instr) {  UNREACHABLE();  }


void LCodeGen::DoClampDToUint8(LClampDToUint8* instr) {  UNREACHABLE();  }


void LCodeGen::DoClampIToUint8(LClampIToUint8* instr) {  UNREACHABLE();  }


void LCodeGen::DoClampTToUint8(LClampTToUint8* instr) {  UNREACHABLE();  }


void LCodeGen::DoCheckPrototypeMaps(LCheckPrototypeMaps* instr) {  UNREACHABLE();  }


void LCodeGen::DoAllocateObject(LAllocateObject* instr) {  UNREACHABLE();  }


void LCodeGen::DoDeferredAllocateObject(LAllocateObject* instr) {  UNREACHABLE();  }


void LCodeGen::DoAllocate(LAllocate* instr) {  UNREACHABLE();  }


void LCodeGen::DoDeferredAllocate(LAllocate* instr) {  UNREACHABLE();  }


void LCodeGen::DoToFastProperties(LToFastProperties* instr) {  UNREACHABLE();  }


void LCodeGen::DoRegExpLiteral(LRegExpLiteral* instr) {  UNREACHABLE();  }


void LCodeGen::DoFunctionLiteral(LFunctionLiteral* instr) {  UNREACHABLE();  }


void LCodeGen::DoTypeof(LTypeof* instr) {  UNREACHABLE();  }


void LCodeGen::DoTypeofIsAndBranch(LTypeofIsAndBranch* instr) {  UNREACHABLE();  }


Condition LCodeGen::EmitTypeofIs(Label* true_label,
                                 Label* false_label,
                                 Register input,
                                 Handle<String> type_name,
                                 Register& cmp1,
                                 Operand& cmp2) {  UNREACHABLE();  return al;}


void LCodeGen::DoIsConstructCallAndBranch(LIsConstructCallAndBranch* instr) {  UNREACHABLE();  }


void LCodeGen::EmitIsConstructCall(Register temp1, Register temp2) {  UNREACHABLE();  }


void LCodeGen::EnsureSpaceForLazyDeopt() {  UNREACHABLE();  }


void LCodeGen::DoLazyBailout(LLazyBailout* instr) {  UNREACHABLE();  }


void LCodeGen::DoDeoptimize(LDeoptimize* instr) {  UNREACHABLE();  }


void LCodeGen::DoDummyUse(LDummyUse* instr) {
  // Nothing to see here, move on!
}


void LCodeGen::DoDeleteProperty(LDeleteProperty* instr) {  UNREACHABLE();  }


void LCodeGen::DoIn(LIn* instr) {  UNREACHABLE();  }


void LCodeGen::DoDeferredStackCheck(LStackCheck* instr) {  UNREACHABLE();  }


void LCodeGen::DoStackCheck(LStackCheck* instr) {  UNREACHABLE();  }


void LCodeGen::DoOsrEntry(LOsrEntry* instr) {  UNREACHABLE();  }


void LCodeGen::DoForInPrepareMap(LForInPrepareMap* instr) {  UNREACHABLE();  }


void LCodeGen::DoForInCacheArray(LForInCacheArray* instr) {  UNREACHABLE();  }


void LCodeGen::DoCheckMapValue(LCheckMapValue* instr) {  UNREACHABLE();  }


void LCodeGen::DoLoadFieldByIndex(LLoadFieldByIndex* instr) {  UNREACHABLE();  }


#undef __

} }  // namespace v8::internal
