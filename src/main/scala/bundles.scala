package vlsu

import chisel3._
import chisel3.util._
import freechips.rocketchip.util.GenericParameterizedBundle
import scala.math._

class VLSUBundle(ap: VLSUArchitecturalParams) extends GenericParameterizedBundle(ap)

/** IO between rob and vlsu. */
class VLSUROBIO(ap: VLSUArchitecturalParams) extends VLSUBundle(ap) {
  val robIdx = Vec(ap.coreWidth, Valid(UInt(ap.robAddrSz.W)))
}

/** Rob tells vlsu commited vuop */
class ROBVLSUIO(ap: VLSUArchitecturalParams) extends VLSUBundle(ap){
  val retireEntries = Flipped(Vec(ap.retireWidth, Valid(new VectorLoadStoreCommit(ap))))
}

class VectorLoadStoreCommit(ap: VLSUArchitecturalParams) extends VLSUBundle(ap){
  val isLoad = Bool()
  val isStore = Bool()
  val qEntryIdx = UInt(max(ap.nVStQIdxBits, ap.nVLdQIdxBits).W)
}

class DispatchVLSUIO(ap: VLSUArchitecturalParams) extends VLSUBundle(ap){
  val vuopDis = Flipped(Vec(ap.coreWidth, Valid(new VLSMicroOP(ap))))
}
class DispatchAck(ap: VLSUArchitecturalParams) extends VLSUBundle(ap){
  val w = max(ap.nVLdQIdxBits, ap.nVStQIdxBits)
  val qIdx = UInt(w.W)
  val robIdx = UInt(ap.robAddrSz.W)
}
class VLSUDispatchIO(ap: VLSUArchitecturalParams) extends VLSUBundle(ap){
  val disVLdQIdx = Output(Vec(ap.coreWidth, Valid(new DispatchAck(ap))))
  val disVStQIdx = Output(Vec(ap.coreWidth, Valid(new DispatchAck(ap))))
  val vLdQFull = Output(Vec(ap.coreWidth, Bool()))
  val vStQFull = Output(Vec(ap.coreWidth, Bool()))
}

class RRVLSUIO(ap: VLSUArchitecturalParams) extends VLSUBundle(ap){
  val vuop = Flipped(Vec(ap.coreWidth, Valid(new VLSMicroOP(ap))))
}

class VLdQEntryBundle(ap: VLSUArchitecturalParams) extends LoadStoreQueueEntryBundleBase(ap){
  val wakeUpVec = Vec(8, Bool())
  val staleRegIdxVec = Vec(8, UInt(ap.nVPregSz.W))
}
class VStQEntryBundle(ap: VLSUArchitecturalParams) extends LoadStoreQueueEntryBundleBase(ap){

}

class LoadStoreQueueEntryBundleBase(ap: VLSUArchitecturalParams) extends VLSUBundle(ap){
  val addr = UInt(ap.coreMaxAddrBits.W)
  val rs2 = UInt(ap.xLen.W)
  val vs1 = UInt(ap.vLen.W)
  /** index For indexed load */
  val vs2 = UInt(ap.vLen.W)
  /** Indicates address of previous split of element for constant stride. */
  val preAddr = UInt(ap.coreMaxAddrBits.W)
  val preSnippet = UInt(ap.vLenb.W)
  val style = new VectorAccessStyle(ap)
  val robIndex = UInt(ap.robAddrSz.W)
  val finishMasks = Vec(8, UInt(ap.vLenb.W))
  val allSucceeded = Bool()
  /** Max segment is 8 */
  val segmentCount = UInt(4.W)
  val totalSegment = UInt(4.W)
  /** total request number within one dest vreg. */
  val totalReq = UInt(ap.vLenb.W)
  /** variable request count within one dest vreg. */
  val reqCount = UInt(ap.vLenb.W)
  /** dstPreg is dest for non-segment load. */
  val pRegVec = Vec(8, UInt(ap.nVPregSz.W))

  val orderFail = Bool()
  val committed = Bool()
  val tlbMasks = Vec(8, UInt(ap.vLenb.W))

  val brMask = UInt(ap.maxBrCount.W)

}

class VLSUMicroControlSignal(ap: VLSUArchitecturalParams) extends VLSUBundle(ap) {
  val accessType = new VectorAccessType
  val accessStyle = new VectorAccessStyle(ap)
}

class VectorAccessStyle(ap : VLSUArchitecturalParams) extends VLSUBundle(ap){
  val isUnitStride: Bool = Bool()
  val isIndexed: Bool = Bool()
  val isConstantStride: Bool = Bool()
  val isSegment: Bool = Bool()
  val isWholeAccess: Bool = Bool()
  val dataEew: UInt = UInt(2.W)
  val indexEew: UInt = UInt(2.W)
  val vStart: UInt = UInt(ap.vLenSz.W)
  val vl: UInt = UInt(ap.vlMax.W)
  val vlmul: UInt = UInt(3.W)
  val indexLmul: UInt = UInt(3.W)
  val nf: UInt = UInt(3.W)
  /** 0 means need to fetch old data for masked elements. */
  val vma: Bool = Bool()
  /** 0 means need to fetch old data for tail elements. */
  val vta: Bool = Bool()
  /** indicates which of this vuop from split uop at dispatch stage. For indexed. */
  val fieldIdx: UInt = UInt(3.W)
}
class VectorAccessType extends Bundle{
  val isLoad: Bool = Bool()
  val isStore: Bool = Bool()
}

class VLSMicroOP(ap: VLSUArchitecturalParams) extends VLSUBundle(ap){
  val vLdQIdx = UInt(ap.nVLdQIdxBits.W)
  val vStQIdx = UInt(ap.nVStQIdxBits.W)
  val robIdx = UInt(ap.robAddrSz.W)
  val vs1: UInt = UInt(ap.vLen.W)
  val vs2: UInt = UInt(ap.vLen.W)
  val rs1: UInt = UInt(ap.xLen.W)
  val rs2: UInt = UInt(ap.xLen.W)
  val vm: UInt = UInt(ap.vLen.W)
  val vpdst: Vec[UInt] = Vec(8, UInt(ap.nVPregSz.W))
  /** hold old reg idx before renaming. For untouched loads. */
  val staleRegIdxes: Vec[UInt] = Vec(8, UInt(ap.nVPregSz.W))
  val uCtrlSig = new VLSUMicroControlSignal(ap)
  val brMask = UInt(ap.maxBrCount.W)
}

class RegAccessControlSignal(ap: VLSUArchitecturalParams) extends VLSUBundle(ap){
  val regIdx: UInt = UInt(ap.nVPregSz.W)
  val snippetIsHead = Bool()
  val snippetIsTail = Bool()
  /** Indicates which part of a vector is under accessing. */
  val finishMaskSnippet: UInt = UInt(ap.vLenb.W)

}

class VLdRequest(ap: VLSUArchitecturalParams) extends VecRequest(ap){}

class VStRequest(ap: VLSUArchitecturalParams) extends VecRequest(ap){}

class VecRequest(ap: VLSUArchitecturalParams) extends VLSUBundle(ap){
  val address: UInt = UInt(ap.addressBits.W)
  def lineAddress: UInt = address(ap.coreMaxAddrBits - 1, ap.offsetBits)
  val regAccessCS = new RegAccessControlSignal(ap)
  val style = new VectorAccessStyle(ap)
  val reqBufferIdx: UInt = UInt(max(ap.nVLdBuffEntryIdxBits, ap.nVStBuffEntryIdxBits).W)
  /** Accessing start point inside the cache line. */
  val lineStartIdx: UInt = UInt(log2Ceil(ap.nCacheLineBytes).W)
  /** Address is translated via tlb. */
  val addressIsPhysical: Bool = Bool()
  /** Indicates whether this request is under processing. */
  val executing: Bool = Bool()
  /** Indicates whether this request is finished. */
  val done: Bool = Bool()
  /** Indicates which segment reg is processing within same reg group, for lmul larger than 0. */
  val segmentIdx: UInt = UInt(3.W)

  /** This load request is observed as probed in L2 cache. */
  val observed: Bool = Bool()
  /** order failed. */
  val orderFail: Bool = Bool()

  val qEntryIdx: UInt = UInt(max(ap.nVLdQIdxBits, ap.nVStQIdxBits).W)
  val committed: Bool = Bool()

  val brMask: UInt = UInt(ap.maxBrCount.W)

  def UnitStrideSnippetsCalculator(offset: UInt)  = {
    val head = Wire(UInt(ap.vLenb.W))
    val body = Wire(UInt(ap.vLenb.W))
    val tail = Wire(UInt(ap.vLenb.W))

    head := (Fill(ap.nCacheLineBytes, 1.U(1.W)) >> offset).asUInt()
    tail := (Fill(ap.nCacheLineBytes, 1.U(1.W)) ## 0.U(ap.vLenb) >> offset).asUInt()(ap.vLenb - 1, 0)
    body := Fill(ap.vLenb, 1.U(1.W)) & (~(head | tail)).asUInt()
    (head, body, tail)
  }
  /** Extract index from vs2 according to voffset and eew.
   * Right shift offset*eew bytes then cut eew bytes off from LSB.*/
  def indexExtractor(indexArray: UInt, IndexEew: UInt, voffset: UInt): UInt = {
    val elementBytes: UInt = (1.U << IndexEew(1,0)).asUInt()
    val voffsetBits: UInt = (voffset << IndexEew) ## 0.U(3.W)
    val indexRaw = indexArray >> voffsetBits
    val indexByteSeq = Seq.tabulate(ap.vLenb)(i => i).map{i =>
      indexRaw(i*8+7, i*8)
    }
    val indexByteVec = VecInit(indexByteSeq)
    val elementMaxBytes = ap.elementMaxBytes
    val elementByteMask = Wire(UInt(elementMaxBytes.W))
    elementByteMask := (1.U << elementBytes).asUInt() - 1.U
    val index = VecInit(elementByteMask.asBools().zipWithIndex.map{
      case(m, i) => Mux(m, indexByteVec(i), 0.U)
    }).asUInt()
    index
  }

  def RequestNexessaryCheck(reqSnippet: UInt, segmentCount: UInt, initSnippet: Vec[UInt])= {
    val oldSnippet = initSnippet(segmentCount)
    val necessary = ((reqSnippet ^ oldSnippet) & reqSnippet).orR()
    val newSnippet = reqSnippet & (~oldSnippet).asUInt()
    (necessary, newSnippet)
  }

  /** Unit stride split example:
   * Snippet is for guiding write back stage after data returned.
   * even if the access data is misaligned to cacheline address.
   * Each snippet is associated to one cache line.
   *              line3    line2     line1     line0
   * Memory:  |---------|---------|---------|---------|
   * Vector:       |end---------------------start| off|
   * accesses:     |tail|---body--|---body--|head|
   * headSnippet:  |0000-0000|0000-0000|0000-1111|
   * bodySnippet:  |0000-0000|0000-1111|1111-0000|
   * bodySnippet:  |0000-1111|1111-0000|0000-0000|
   * tailSnippet:  |1111-0000|0000-0000|0000-0000|
   * This vec load ins is split into 4 request with snippets,
   * which can form into all ones, which indicates its finish.
   * Currently, only 0~1 body is supported.
   */
  def UnitStride(addr: UInt,
                 reqCount: UInt,
                 dstPReg: UInt,
                 segmentCount: UInt,
                 isWholeAccess: Bool, isLoad: Boolean,
                 initialSnippet: Vec[UInt],
                 id: Int): (Bool, VecRequest) = {
    val out = {
      if (isLoad){
        val out = WireInit(0.U.asTypeOf(new VLdRequest(ap)))
        out.qEntryIdx := id.U
        out.observed := false.B
        out.orderFail := false.B
        out
      }else {
        val out = WireInit(0.U.asTypeOf(new VStRequest(ap)))
        out.qEntryIdx := id.U
        out.committed := false.B
        out
      }}
    val isHead: Bool = reqCount === 0.U
    val isTail: Bool = reqCount === (ap.maxReqsInUnitStride - 1).U
    val offset: UInt = addr(ap.offsetBits - 1, 0)
    val (headSnippet, bodySnippet, tailSnippet) = UnitStrideSnippetsCalculator(offset)
    assert((tailSnippet | headSnippet | bodySnippet).andR(), "Wrong snippet result!")
    out.address := addr + (reqCount << ap.offsetBits).asUInt()
    out.segmentIdx := segmentCount(2,0)
    out.style.isUnitStride := true.B
    out.style.isIndexed := false.B
    out.style.isConstantStride := false.B
    out.style.isWholeAccess := isWholeAccess
    out.style.isSegment := false.B
    out.regAccessCS.snippetIsHead := isHead
    out.regAccessCS.snippetIsTail := isTail
    //eew is ineffective in unit-stride
    out.style.dataEew := 0.U
    out.style.indexEew := 0.U
    out.regAccessCS.regIdx := dstPReg
    out.lineStartIdx := offset
    out.addressIsPhysical := false.B
    out.reqBufferIdx := 0.U
    val reqSnippet = Mux(isHead, headSnippet, Mux(isTail, tailSnippet, bodySnippet))
    out.executing := false.B
    out.done := false.B
    val reqNecessary = RequestNexessaryCheck(reqSnippet, segmentCount, initialSnippet)
    out.regAccessCS.finishMaskSnippet := reqNecessary._2
    (reqNecessary._1, out)
  }


  def ConstantStride(addr: UInt,
                     preAddr: UInt,
                     strideBytes: UInt,
                     dataEew: UInt,
                     dstPReg: UInt,
                     segmentCount: UInt,
                     reqCount: UInt,
                     preSnippet: UInt, isLoad: Boolean,
                     initialSnippet: Vec[UInt],
                     id: Int): (Bool, VecRequest, UInt, UInt) = {
    val out = if(isLoad){
      val out = WireInit(0.U.asTypeOf(new VLdRequest(ap)))
      out.qEntryIdx := id.U
      out.observed := false.B
      out.orderFail := false.B
      out
    } else {
      val out = WireInit(0.U.asTypeOf(new VStRequest(ap)))
      out.qEntryIdx := id.U
      out.committed := false.B
      out
    }
    val offset = preAddr(ap.offsetBits - 1, 0)
    val elementAddr = Mux(reqCount === 0.U, addr, preAddr + strideBytes)
    out.executing := false.B
    out.done := false.B
    out.style.isUnitStride := false.B
    out.style.isConstantStride := true.B
    out.style.isIndexed := false.B
    out.style.isSegment := false.B
    out.style.isWholeAccess := false.B
    val elementBytes: UInt = (1.U << dataEew(1,0)).asUInt()
    out.style.dataEew := dataEew
    out.style.indexEew := 0.U
    out.lineStartIdx := offset
    out.addressIsPhysical := false.B
    out.reqBufferIdx := 0.U
    out.address := elementAddr
    val initSnippet: UInt = (1.U << elementBytes).asUInt() - 1.U
    val elementSnippet = Mux(reqCount.orR(), (preSnippet << elementBytes).asUInt(), initSnippet)
    assert(PopCount(elementSnippet) === elementBytes, "1s in snippet should queal to eew bytes.")
    val reqNecessary = RequestNexessaryCheck(elementSnippet, segmentCount, initialSnippet)
    out.regAccessCS.finishMaskSnippet := reqNecessary._2
    (reqNecessary._1, out, elementAddr, elementSnippet)
  }

  def Indexed(addr: UInt,
              indexEew: UInt,
              dataEew: UInt,
              dstPreg: Vec[UInt],
              reqCOunt: UInt,
              indexRegIdx: UInt,
              preSnippet: UInt,
              indexArray: UInt, isLoad: Boolean,
              initialSnippet: Vec[UInt],
              id: Int): (Bool, VecRequest, UInt) = {
    val out = if(isLoad){
      val out = WireInit(0.U.asTypeOf(new VLdRequest(ap)))
      out.qEntryIdx := id.U
      out.observed := false.B
      out.orderFail := false.B
      out
    } else {
      val out = WireInit(0.U.asTypeOf(new VStRequest(ap)))
      out.qEntryIdx := id.U
      out.committed := false.B
      out
    }

    val index: UInt = indexExtractor(indexArray, indexEew, reqCOunt)
    val elementAddr: UInt = (addr.asSInt() + index.asSInt()).asUInt()
    val elementAddrOffset: UInt = elementAddr(ap.offsetBits - 1, 0)
    val dataElementBytes: UInt = (1.U << dataEew(1,0)).asUInt()
    val indexElementBytes: UInt = (1.U << indexEew(1,0)).asUInt()
    val nElementPerDataReg: UInt = ap.vLenb.U / dataElementBytes
    val nElementPerIndexReg: UInt = ap.vLenb.U / indexElementBytes

    val largerData = dataEew(1,0) > indexEew(1,0)
    val dataExpandRate = dataEew(1,0) - indexEew(1,0)
    val largerIndex = indexEew(1,0) > dataEew(1,0)
    val dataShrinkRate = indexEew(1,0) - dataEew(1,0)
    val equal = indexEew === dataEew

    val shrinkHalf = dataShrinkRate === 1.U
    val shrinkQuarter = dataShrinkRate === 2.U
    val shrinkEighth = dataShrinkRate === 3.U
    val expandDouble = dataExpandRate === 1.U
    val expandQuarter = dataExpandRate === 2.U
    val expandOctuple = dataExpandRate === 3.U

    val dataRegIdxShrink = Mux(shrinkHalf, indexRegIdx(2,1), Mux(shrinkQuarter, indexRegIdx(2), 0.U))

    val dataRegIdxExpandBase: UInt = Mux(expandOctuple, 0.U, Mux(expandDouble, indexRegIdx(1,0) << 1, indexRegIdx(0) << 2)).asUInt()
    val dataRegIdxExpandOffset: UInt = reqCOunt / nElementPerDataReg
    val dataRegIdxExpand: UInt = dataRegIdxExpandBase + dataRegIdxExpandOffset

    /** Target reg idx in the group, not idx in vrf. */
    val dataRegIdx = Mux(equal, indexRegIdx, Mux(largerIndex, dataRegIdxShrink, dataRegIdxExpand))

    val initSnippetEqualExpand: UInt = (1.U << dataElementBytes).asUInt() - 1.U
    val initSnippetShrinkHalf: UInt = VecInit(UIntToOH(indexRegIdx(0), 2).asBools().map(b =>
      Mux(b, initSnippetEqualExpand | 0.U((ap.vLenb/2).W), 0.U((ap.vLenb/2).W)))).asUInt()
    val initSnippetShrinkQuarter: UInt = VecInit(UIntToOH(indexRegIdx(1,0)).asBools().map(b =>
      Mux(b, initSnippetEqualExpand | 0.U((ap.vLenb/4).W), 0.U((ap.vLenb/4).W)))).asUInt()
    val initSnippetShrinkEighth: UInt = VecInit(UIntToOH(indexRegIdx, 8).asBools().map(b =>
      Mux(b, initSnippetEqualExpand | 0.U((ap.vLenb/4).W), 0.U((ap.vLenb/8).W)))).asUInt()
    val initSnippetSnippetShrink: UInt = Mux(shrinkHalf, initSnippetShrinkHalf,
      Mux(shrinkQuarter, initSnippetShrinkQuarter, initSnippetShrinkEighth))

    val reqCountOffset: UInt = reqCOunt % nElementPerDataReg
    val initSnippet: UInt = Mux(largerIndex, initSnippetSnippetShrink, initSnippetEqualExpand)
    val elementSnippet: UInt = Mux(reqCountOffset.orR(), (preSnippet << dataElementBytes).asUInt(), initSnippet)
    out.address := elementAddr
    out.executing := false.B
    out.done := false.B
    /** In indexed processing, this indicates regIdx of data. */
    out.segmentIdx := dataRegIdx
    out.style.isUnitStride := false.B
    out.style.isConstantStride := false.B
    out.style.isIndexed := true.B
    out.style.isSegment := false.B
    out.style.isWholeAccess := false.B
    out.style.indexEew := indexEew
    out.style.dataEew := dataEew
    out.regAccessCS.regIdx := dstPreg(dataRegIdx)
    out.lineStartIdx := elementAddrOffset
    out.reqBufferIdx := 0.U
    val reqNecessary = RequestNexessaryCheck(elementSnippet, dataRegIdx, initialSnippet)
    out.regAccessCS.finishMaskSnippet := reqNecessary._2
    (reqNecessary._1, out, elementSnippet)
  }
}

class VLSUReadVRFReq(ap: VLSUArchitecturalParams) extends VLSUBundle(ap){
  val addr = UInt(ap.nVPregSz.W)
}
class VLSUReadVRFResp(ap: VLSUArchitecturalParams) extends VLSUBundle(ap){
  val data = UInt(ap.vLen.W)
}
class VLSUWriteVRFReq(ap: VLSUArchitecturalParams) extends VLSUBundle(ap){
  val addr = UInt(ap.nVPregSz.W)
  val byteMask = UInt(ap.vLenb.W)
  val data = UInt(ap.vLen.W)
}
class VLSUVRFIO(ap: VLSUArchitecturalParams) extends VLSUBundle(ap){
  val write = ValidIO(new VLSUWriteVRFReq(ap))
  val readReq = ValidIO(new VLSUReadVRFReq(ap))
}
class VRFVLSUIO(ap: VLSUArchitecturalParams) extends VLSUBundle(ap){
  val readResp = Flipped(ValidIO(new VLSUReadVRFResp(ap)))
}

class VLdWriteBackRequest(ap: VLSUArchitecturalParams) extends VLSUBundle(ap){
  val req = new VLdRequest(ap)
  val data = UInt(ap.nCacheLineBits.W)
}

class VTLBReq(ap: VLSUArchitecturalParams) extends VLSUBundle(ap){
  val vaddr = UInt(ap.coreMaxAddrBits.W)
  val queueIdx = UInt(max(ap.nVLdQIdxBits, ap.nVStQIdxBits).W)
  val isLoad = Bool()
  val reqBuffIdx = UInt(ap.nVLdBuffEntryIdxBits.W)
  val snippet = UInt(ap.vLenb.W)
  val segmentIdx = UInt(3.W)
}

class VTLBResp(ap: VLSUArchitecturalParams) extends VLSUBundle(ap){
  val paddr = UInt(ap.coreMaxAddrBits.W)
  val queueIdx = UInt(max(ap.nVLdQIdxBits, ap.nVStQIdxBits).W)
  val isLoad = Bool()
  val reqBuffIdx = UInt(ap.nVLdBuffEntryIdxBits.W)
  val snippet = UInt(ap.vLenb.W)
  val hit = Bool()
  val exception = Bool()
  val segmentIdx = UInt(3.W)
}

class AddressCheckerResp(ap: VLSUArchitecturalParams) extends VLSUBundle(ap){
  val vldReqBuffIdx = UInt(ap.nVLdBuffEntries.W)
}

class SMSHRStatus(ap: VLSUArchitecturalParams) extends VLSUBundle(ap){
  val awaiting = Bool()
  val addr = UInt(ap.coreMaxAddrBits.W)
}

/** import from boom core pipeline. [[boom.exu.BrUpdateMasks]] */
class BranchUpdateMasks(ap: VLSUArchitecturalParams) extends VLSUBundle(ap){
  val resolveMask = UInt(ap.maxBrCount.W)
  val mispredictMask = UInt(ap.maxBrCount.W)
}
/** import [[boom.exu.BrResolutionInfo]] */
class BranchResolutionInfo(ap: VLSUArchitecturalParams) extends VLSUBundle(ap){
  val vLdqIdx = UInt(ap.nVLdQIdxBits.W)
  val vStqIdx = UInt(ap.nVStQIdxBits.W)
  val mispredicted = Bool()
}
/** import [[boom.exu.BrUpdateInfo]] */
class BranchUpdateInfo(ap: VLSUArchitecturalParams) extends VLSUBundle(ap){
  val b1 = new BranchUpdateMasks(ap)
  val b2 = new BranchResolutionInfo(ap)
}

/** Top io bundle */
class VLSUTopBundle(ap: VLSUArchitecturalParams) extends VLSUBundle(ap){
  val brUpdate = Input(new BranchUpdateInfo(ap))
  /** vrf io */
  val toVrf = new VLSUVRFIO(ap)
  val fromVrf = new VRFVLSUIO(ap)

  /** Dispatch stage to vlsu. */
  val fromDis = new DispatchVLSUIO(ap)
  val toDis = new VLSUDispatchIO(ap)

  /** Register read stage to vlsu. */
  val fromRr = new RRVLSUIO(ap)
  /** vlsu to Rob */
  val ldToRob = new VLSUROBIO(ap)
  val stToRob = new VLSUROBIO(ap)
  val wakeUpVreg = ValidIO(UInt(ap.nVPregSz.W))
  val fromRob = new ROBVLSUIO(ap)
  val vrfBusyStatus = Input(UInt(ap.nVRegs.W))
}