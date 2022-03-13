package vlsu
import boom.util.maskMatch
import chisel3._
import chisel3.util._
import freechips.rocketchip.util.{DontTouch, leftOR, rightOR}
abstract class VLSUModules(ap: VLSUArchitecturalParams) extends Module with DontTouch{
  def lineAddress(address: UInt): UInt = {
    (address >> ap.offsetBits) ## 0.U(ap.offsetBits.W)
  }
}

class FindFirstOne(width: Int) extends RawModule{
  val io = IO(new Bundle{
    val input = Input(UInt(width.W))
    val result = Output(UInt(width.W))
  })

  val leftOrBits = leftOR(io.input)
  io.result := leftOrBits ^ (leftOrBits << 1)(leftOrBits.getWidth - 1, 0)
}

object FindFirstOne{
  def apply(input: UInt, width: Int): UInt = {
    val ffo = Module(new FindFirstOne(width))
    ffo.io.input := input
    ffo.io.result
  }
}

/** This module stretches data of a cache line into vlen width format accroding to request snippet. */
class UnitStrideDataStretcher(ap: VLSUArchitecturalParams) extends VLSUModules(ap){
  val io = IO(new Bundle{
    val lineDataIn = Input(UInt(ap.nCacheLineBits.W))
    val reqWB = Input(new VLdRequest(ap))
    val vrfWriteReq = Output(new VLSUWriteVRFReq(ap))
  })
  val offset = io.reqWB.address(ap.offsetBits - 1, 0)
  val alignedAddr = !offset.orR()
  val bodyChunks = (ap.vLenb / ap.nCacheLineBytes) - 1
  require(bodyChunks < 2, "body too long to support!")
  io.vrfWriteReq.addr := io.reqWB.regAccessCS.regIdx
  val snippet = io.reqWB.regAccessCS.finishMaskSnippet
  io.vrfWriteReq.byteMask := snippet
  val isHead = io.reqWB.regAccessCS.snippetIsHead
  val isTail = io.reqWB.regAccessCS.snippetIsTail
  /** Number of bytes left from offset to end of the cache line. */
  val offsetLeft = ap.nCacheLineBytes.U - offset
  val offsetBits = (offset << 3).asUInt()
  val headData = (io.lineDataIn >> offsetBits).asUInt()
  val rawData = io.lineDataIn ## 0.U(ap.vLen.W)
  val bodyData = ((rawData >> (bodyChunks * ap.nCacheLineBits)) >> offsetBits).asUInt()
  val tailData = Mux(alignedAddr, bodyData, (rawData >> offsetBits)(ap.vLen - 1, 0))
  io.vrfWriteReq.data := Mux(isHead, headData, Mux(isTail, tailData, bodyData))
}

/** Dedicated arbiter for split vector access request. Winner from last cycle always wins. */
class RequestArbitrator[req <: Data](ap: VLSUArchitecturalParams, request: req, isLoad: Boolean) extends VLSUModules(ap) {
  val outputWidth: Int = ap.coreWidth
  val inputWidth: Int = if (isLoad) ap.nVLdQEntries else ap.nVStQEntries
  val idxBits: Int = log2Ceil(inputWidth)
  val io = IO(new Bundle {
    val inputReq = Flipped(Vec(inputWidth, Decoupled(request)))
    val outputReq = Vec(outputWidth, Decoupled(request))
  })
  val winnersIdx = RegInit(0.U.asTypeOf(Vec(outputWidth, Valid(UInt(idxBits.W)))))
  val inputVlds = VecInit(io.inputReq.map(_.valid)).asUInt()

  var winners = WireInit(0.U(outputWidth.W))
  var players = inputVlds
  var winnerThisSlot = WireInit(0.U(outputWidth.W))
  var winnerLastCycle = WireInit(0.U(outputWidth.W))

  winnerLastCycle = RegNext(VecInit(io.outputReq.map(_.valid))).asUInt()

  val winnerIdxVec = Wire(Vec(outputWidth, UInt(idxBits.W)))
  for (w <- 0 until outputWidth) {
    val winnerStillIn = FindFirstOne(winnerLastCycle & players, inputWidth)
    winnerThisSlot = Mux(winnerStillIn.orR(), winnerStillIn, FindFirstOne(players, inputWidth))
    val winnerIdx = OHToUInt(winnerThisSlot)
    winnerIdxVec(w) := winnerIdx
    io.outputReq(w).valid := winnerThisSlot.orR() && io.inputReq(winnerIdx).valid
  }
}

/** return oldest entry according given width and head pointer. */
class PickOldest(width: Int) extends RawModule{
  val io = IO(new Bundle{
    val reqVec = Input(UInt(width.W))
    val headPtr = Input(UInt(log2Ceil(width).W))
    val tailPtr = Input(UInt(log2Ceil(width).W))
    val result = Valid(UInt(log2Ceil(width).W))
  })
  io.result := 0.U.asTypeOf(Valid(UInt(log2Ceil(width).W)))
  val head = io.headPtr
  val tail = io.tailPtr
  io.result.valid := io.reqVec.orR()
  when(head > tail){
    io.result.bits := OHToUInt(FindFirstOne(io.reqVec & (~rightOR(io.headPtr)).asUInt(), width))
  }.elsewhen(head < tail){
    io.result.bits := OHToUInt(FindFirstOne(io.reqVec, width))
  }
}
object  PickOldest{
  def apply(reqVec: UInt, headPtr: UInt, tailPtr: UInt, inputWidth: Int): Valid[UInt] = {
    val picker = Module(new PickOldest(inputWidth))
    picker.io.reqVec := reqVec
    picker.io.headPtr := headPtr
    picker.io.tailPtr := tailPtr
    picker.io.result
  }
}

/** This arbiter picks up a number of input valid, and transfer them into UInt index. */
class IndexArbiter(inputWidth: Int, outputWidth: Int) extends RawModule{
  val idxWidth = log2Ceil(inputWidth)
  val io = IO(new Bundle{
    val input = Input(UInt(inputWidth.W))
    val output = Output(Vec(outputWidth, Valid(UInt(idxWidth.W))))
  })
  val winnerIdxes = 0.U.asTypeOf(Vec(outputWidth, Valid(UInt(idxWidth.W))))
  var players = (~io.input).asUInt()
  for (i <- 0 until outputWidth){
    val winnerThisSlot = FindFirstOne(players, inputWidth)
    players = players & (~winnerThisSlot).asUInt()
    winnerIdxes(i).bits := OHToUInt(winnerThisSlot)
  }
  val idlesCounts = PopCount(~io.input)
  winnerIdxes.map(_.valid).zipWithIndex.foreach{ case (v, i) =>
    v := i.U < idlesCounts
  }
  io.output := winnerIdxes
}

/** First gneerate new data to write then merge new data with pre data. */
class UnitStrideStoreLineFactory(ap: VLSUArchitecturalParams) extends VLSUModules(ap){
  val io = IO(new Bundle{
    val isHead = Input(Bool())
    val isTail = Input(Bool())
    val storeSnippet = Input(UInt(ap.vLenb.W))
    val address = Input(UInt(ap.coreMaxAddrBits.W))
    val storeData = Input(UInt(ap.vLen.W))
    val lineData = Output(UInt(ap.nCacheLineBits.W))
    val lineMask = Output(UInt(ap.nCacheLineBytes.W))
  })
  val isHead = io.isHead
  val isTail = io.isTail
  val hasBody: Boolean = (ap.vLen/ap.nCacheLineBits) > 1
  val offset = io.address(ap.offsetBits - 1, 0)
  val alignedAddr = !offset.orR()
  val offsetLeft = ap.nCacheLineBytes.U - offset
  val vData = io.storeData
  val vMask = io.storeSnippet
  val headData = ((vData ## 0.U(ap.nCacheLineBits.W)) >> (offsetLeft << 3))(ap.nCacheLineBits - 1, 0)
  val bodyData = if(hasBody) (vData >> (offsetLeft << 3).asUInt())(ap.nCacheLineBits - 1, 0) else 0.U
  val tailData = if(hasBody) Mux(alignedAddr, bodyData, ((vData >> (offsetLeft << 3).asUInt()) >> ap.nCacheLineBits.U).asUInt()(ap.nCacheLineBits - 1, 0))
                 else (vData >> (offsetLeft << 3).asUInt()).asUInt()(ap.nCacheLineBits - 1, 0)
  val headMask = ((vMask ## 0.U(ap.nCacheLineBytes.W)) >> offsetLeft)(ap.nCacheLineBytes - 1, 0)
  val bodyMask = if(hasBody) (vMask >> offsetLeft).asUInt()(ap.nCacheLineBytes - 1, 0) else 0.U
  val tailMask = if(hasBody) Mux(alignedAddr, bodyMask, ((vMask >> offsetLeft).asUInt() >> ap.nCacheLineBytes).asUInt()(ap.nCacheLineBytes - 1, 0))
                 else (vMask >> offsetLeft).asUInt()(ap.nCacheLineBytes - 1, 0)

  io.lineData := Mux(isHead, headData, Mux(isTail, tailData, bodyData))
  io.lineMask := Mux(isHead, headMask, Mux(isTail, tailMask, bodyMask))
}

/** Construct a line with single element snippet. */
class ElementStoreLineFactory(ap: VLSUArchitecturalParams) extends VLSUModules(ap){
  val io = IO(new Bundle{
    val storeSnippet = Input(UInt(ap.vLenb.W))
    val eew = Input(UInt(3.W))
    val address = Input(UInt(ap.coreMaxAddrBits.W))
    val storeData = Input(UInt(ap.vLen.W))
    val lineData = Output(UInt(ap.nCacheLineBits.W))
    val lineMask = Output(UInt(ap.nCacheLineBytes.W))
  })
  val elementBytes = (1.U << io.eew(1,0)).asUInt()
  val offset = io.address(ap.offsetBits - 1, 0)
  /** Indicates start byte in vreg that is to store */
  val vStartIdx: UInt = OHToUInt(FindFirstOne(io.storeSnippet, ap.vLenb))
  val elementBitMask = (1.U << (elementBytes << 3.U).asUInt()).asUInt() - 1.U
  val elementData = (io.storeData >> (vStartIdx << 3.U).asUInt()).asUInt() & elementBitMask
  val offsetLeft = ap.nCacheLineBytes.U - offset
  val data = ((elementData ## 0.U(ap.nCacheLineBits.W)) >> (offsetLeft << 3.U).asUInt()).asUInt()
  io.lineData := data
  val elementByteMask = (1.U << elementBytes).asUInt() - 1.U
  val mask = ((elementByteMask ## 0.U(ap.nCacheLineBytes.W)) >> offsetLeft).asUInt()
  io.lineMask := mask

}

/** in data would overlap old data. */
class LineDataMerger(ap: VLSUArchitecturalParams) extends VLSUModules(ap){
  val io = IO(new Bundle{
    /** newer incoming data that would merge into old data. */
    val inData = Input(UInt(ap.nCacheLineBits.W))
    val inMask = Input(UInt(ap.nCacheLineBytes.W))
    val oldData = Input(UInt(ap.nCacheLineBits.W))
    val oldMask = Input(UInt(ap.nCacheLineBytes.W))
    val newData = Output(UInt(ap.nCacheLineBits.W))
    val newMask = Output(UInt(ap.nCacheLineBytes.W))
  })
  val inMask = io.inMask.asBools()
  val oldMask = io.oldMask.asBools()
  io.newMask := io.inMask | io.oldMask
  val newData = VecInit((inMask zip oldMask).zipWithIndex.map{case ((in,old),i) =>
    val inByte = io.inData(i * 8 + 7, i * 8)
    val oldByte = io.oldData(i*8+7, i*8)
    Mux(in, inByte, Mux(old, oldByte, 0.U(8.W)))
  }).asUInt()
  io.newData := newData
}

class ElementDataStretcher(ap: VLSUArchitecturalParams) extends VLSUModules(ap){
  val io = IO(new Bundle{
    val reqWB = Input(new VLdRequest(ap))
    val lineDataIn = Input(UInt(ap.nCacheLineBits.W))
    val vrfWriteReq = Output(new VLSUWriteVRFReq(ap))
  })
  val len = (1.U << io.reqWB.style.dataEew(1,0)).asUInt()
  val snippet = io.reqWB.regAccessCS.finishMaskSnippet
  io.vrfWriteReq.byteMask := snippet
  val offset = io.reqWB.address(ap.offsetBits - 1, 0)
  val vStartIdx = OHToUInt(FindFirstOne(snippet, ap.vLenb))
  val elementMask = (1.U << (len << 3.U).asUInt()).asUInt() - 1.U
  val elementData = (io.lineDataIn >> (offset << 3.U).asUInt()).asUInt() & elementMask
  /** attach vlen 0s on the right side, then right shift to correct place. */
  val data = ((elementData ## 0.U(ap.vLen.W)) >> ((ap.vLenb.U - vStartIdx) << 3.U).asUInt()).asUInt()
  io.vrfWriteReq.data := data
  io.vrfWriteReq.addr := io.reqWB.regAccessCS.regIdx
}

/** Construct snippet on first allocate into queue entry according to vl, lmul, eew and vstart. */
class SnippetInitializer(ap: VLSUArchitecturalParams) extends VLSUModules(ap){
  val io = IO(new Bundle{
    val ctrl = Input(new VectorAccessStyle(ap))
    val initSnippet = Output(Vec(8, UInt(ap.vLenb.W)))
    val totalSegment = Output(UInt(4.W))
    val totalRequest = Output(UInt((log2Ceil(ap.vLenb) + 1).W))
    val wakeVecInit = Output(UInt(8.W))
  })
  val isIndexed = io.ctrl.isIndexed
  val fieldIdx = io.ctrl.fieldIdx
  val isSegment = io.ctrl.isSegment
  val elementByte = (1.U << io.ctrl.dataEew(1,0)).asUInt()
  /** 00 => 1, 01 => 2, 10 => 4, 11 => 8. */
  val lmul = io.ctrl.vlmul(1,0)
  val lmulValue = (1.U << lmul).asUInt()
  val lmul2: Bool = WireInit(lmul === 1.U)
  val lmul4: Bool = WireInit(lmul === 2.U)
  val lmul8: Bool = WireInit(lmul === 3.U)
  val lmul1: Bool = WireInit(lmul === 0.U)
  val lmulLagerThanOne = !io.ctrl.vlmul(2) && lmul.orR()
  val lmulSmallerThanOne = io.ctrl.vlmul(2)
  val lmulLargerEqualOne = !io.ctrl.vlmul(2)
  val lmulSmallerEqualOne = lmul1 || lmulSmallerThanOne
  val nFields = io.ctrl.nf

  val allOnes = Fill(ap.vLenb, 1.U)
  val segmentVld: Int => Bool = (i: Int) => i.U < (nFields * lmulValue)
  val shrinkRate = 4.U - lmul
  /** byte index of vstart. */
  val startByte = io.ctrl.vStart * elementByte
  /** Indicates if this vlen is valid under current vlmul, parameter is segment index. */
  val lmulVld: Int => Bool = (pregIdx: Int) => pregIdx.U < (1.U << lmul).asUInt()
  /** end byte index of vl. */
  val endBytevl = (io.ctrl.vl * elementByte).asUInt()
  /** calculate end byte index of this vlen when is in one register group, parameter is segment index. */
  val endByteLmulX: Int => UInt = (pregIdx: Int) => WireInit(
    Mux(lmulLagerThanOne && lmulVld(pregIdx), (ap.vLenb.U * (pregIdx + 1).U).asUInt(),
      Mux(lmul1, ap.vLenb.U, (ap.vLenb.U >> shrinkRate).asUInt())))
  /** Compare two end byte index and pick smaller one. parameter is segment index. */
  val endByteIdx: Int => UInt = (pregIdx: Int) => WireInit(
    Mux(endByteLmulX(pregIdx) > endBytevl, endBytevl, endByteLmulX(pregIdx)))
  /** 0 means this byte needs to be accessed. parameter is byte index. */
  val byteVldX: (Int, UInt) => Bool = (byteIdx: Int, endByteIdx: UInt) => WireInit(
    byteIdx.U < startByte || byteIdx.U >= endByteIdx)
  /** snippet when this reg is under same group with previous register. parameter is segment idx. */
  val snippetX: Int => UInt = (pregIdx: Int) => WireInit(
    VecInit(Seq.tabulate(ap.vLenb)(byteIdx => byteIdx + pregIdx * ap.vLenb).map(byteIdx => byteVldX(byteIdx, endByteIdx(pregIdx)))).asUInt())

  io.initSnippet.zipWithIndex.foreach {case (out, i) =>
    out := Mux(io.ctrl.isWholeAccess && lmulVld(i), 0.U, Mux(lmulVld(i), snippetX(i), allOnes))
  }
  io.totalSegment := Mux(isSegment, lmulValue * nFields, Mux(isIndexed, fieldIdx + 1.U, lmulValue))
  val denseAccess = (io.ctrl.isUnitStride && !isSegment) || io.ctrl.isWholeAccess
  io.totalRequest := Mux(denseAccess, ap.maxReqsInUnitStride.U, Mux(isIndexed, ap.vLenb.U >> io.ctrl.indexEew, ap.vLenb.U >> io.ctrl.dataEew))
  val indexEew = io.ctrl.indexEew
  val dataEew = io.ctrl.dataEew
  val largerData = dataEew > indexEew
  val dataExpandRate = dataEew - indexEew
  val largerIdx = indexEew - dataEew
  val equal = indexEew === dataEew
  val dataShrinkRate = indexEew - dataEew
  val offsetMultiReg = largerData

  val shrinkHalf = dataShrinkRate === 1.U
  val shrinkQuarter = dataShrinkRate === 2.U
  val shrinkOctant = dataShrinkRate === 3.U
  val wakeVecShrink = UIntToOH((fieldIdx >> dataShrinkRate).asUInt(), 8)
  val expandDouble = dataExpandRate === 1.U
  val expandQuarter = dataExpandRate === 2.U
  val expandOctuple = dataExpandRate === 3.U
  val wakeVecExpand = Mux(expandOctuple, 0xff.U, Mux(expandQuarter, 0xf.U << (4.U * fieldIdx), 0x3.U << (2.U * fieldIdx))).asUInt()

  val wakeRegIdxIndexed = Mux(equal, UIntToOH(fieldIdx, 8), Mux(largerData, wakeVecExpand, wakeVecShrink))
  io.wakeVecInit := Mux(isIndexed, wakeRegIdxIndexed, Mux(lmul1 || lmulSmallerThanOne, 1.U, Mux(lmul2, 3.U, Mux(lmul4, 0xf.U, 0xff.U))))
}

/** calculate snippet according index reg idx. */
class SnippetIndexedSplitsFilter(ap: VLSUArchitecturalParams) extends VLSUModules(ap){
  val io = IO(new Bundle{
    val ctrl = Input(new VectorAccessStyle(ap))
    val filteredSnippet = Output(Vec(8, UInt(ap.vLenb.W)))
  })
  val isIndexed = io.ctrl.isIndexed
  val indexSegIdx = io.ctrl.fieldIdx
  val indexEew = io.ctrl.indexEew
  val dataEew = io.ctrl.dataEew
  val largerData = dataEew > indexEew
  val dataExpandRate = dataEew - indexEew
  val largerIndex = indexEew > dataEew
  val dataShrinkRage = indexEew - dataEew
  val equal = indexEew === dataEew
  val offsetMultiReg = largerData

  val shrinkHalf = dataShrinkRage === 1.U
  val shrinkQuarter = dataShrinkRage === 2.U
  val shrinkOctant = dataShrinkRage === 3.U
  val expandDouble = dataExpandRate === 1.U
  val expandQuarter = dataExpandRate === 2.U
  val expandOctuple = dataExpandRate === 3.U
  val allOnes = Fill(ap.vLenb, 1.U(1.W))

  val indexOH = UIntToOH(indexSegIdx, 8)

  /** snippet INSIDE picked reg snippet. */
  val partialMask: UInt = Mux(shrinkHalf, Fill(ap.vLenb/2, 1.U(1.W)) << ((ap.vLenb/2).U * indexSegIdx(0)),
    Mux(shrinkQuarter, Fill(ap.vLenb/4, 1.U(1.W)) << (ap.vLenb/4).U * indexSegIdx(1,0),
      Fill(ap.vLenb/8, 1.U(1.W)) << (ap.vLenb/8).U * indexSegIdx(2,0))).asUInt()
  val wakeVecExpand: UInt = Mux(expandOctuple, 0xff.U,
    Mux(expandQuarter, 0xf.U << (4.U * indexSegIdx(0)), 0x3.U << (2.U * indexSegIdx(1,0)))).asUInt()
  val wakeVecShrink: UInt = UIntToOH((indexSegIdx >> dataShrinkRage).asUInt(), 8)
  io.filteredSnippet.zipWithIndex.foreach{ case (io, regIdx) =>
    val snippet = Mux(!isIndexed, 0.U, Mux(largerData, Mux(wakeVecExpand(regIdx), 0.U, allOnes),
    Mux(equal, Mux(wakeVecShrink(regIdx), 0.U, allOnes), Mux(wakeVecShrink(regIdx), ~partialMask, allOnes))))
    io := snippet
  }
}

/** Construct active snippet for element access according to vm and lmul, nf and new vl. */
class SnippetVectorMaskAdjuster(ap: VLSUArchitecturalParams) extends VLSUModules(ap) {
  val io = IO(new Bundle{
    val ctrl = Input(new VectorAccessStyle(ap))
    val vm = Input(UInt(ap.vLen.W))
    /** adjust snippet, bit x is 0 means this bytes of data needs to be accessed. */
    val adjusterSnippet = Output(Vec(8, UInt(ap.vLenb.W)))
  })
  io.adjusterSnippet := 0.U.asTypeOf(Vec(8, UInt(ap.vLenb.W)))
  val elementBytes = (1.U << io.ctrl.dataEew(1, 0)).asUInt()
  val elenB = (1.U << io.ctrl.dataEew(1,0)).asUInt()
  val isSegment = io.ctrl.isSegment
  val elen1 = io.ctrl.dataEew === 0.U
  val elen2 = io.ctrl.dataEew === 1.U
  val elen4 = io.ctrl.dataEew === 2.U
  val elen8 = io.ctrl.dataEew === 3.U
  val lmul = io.ctrl.vlmul(1,0)
  val lmulOHs = WireInit(0.U(8.W))
  lmulOHs := (1.U << lmul).asUInt() - 1.U
  val elementsPerRegElen1 = ap.vLenb
  val elementsPerRegElen2 = ap.vLenb / 2
  val elementsPerRegElen4 = ap.vLenb / 4
  val elementsPerRegElen8 = ap.vLenb / 8
  val lmul2 = WireInit(lmul === 1.U)
  val lmul4 = WireInit(lmul === 2.U)
  val lmul8 = WireInit(lmul === 3.U)
  val lmul1 = WireInit(lmul === 0.U)
  val lmulLargerThanOne = !io.ctrl.vlmul(2) && lmul.orR()
  val nFields = io.ctrl.nf + 1.U
  val lmulValue = (1.U << lmul).asUInt()
  val shrinkRate = 4.U - lmul
  val elementVldVec = io.vm
  /** Indicates if this vlen is valid under current vlmul. */
  val lmulVld: Int => Bool = (pregIdx: Int) => pregIdx.U < (1.U << lmul).asUInt()
  /** if register is valid in lmul and nf combinations. */
  val segmentVld: Int => Bool = (i: Int) => i.U < (nFields * lmulValue)
  /** end byte index of vl. */
  val endBytevl = (io.ctrl.vl * elementBytes).asUInt()
  /** byte index of vstart. */
  val startByte = io.ctrl.vStart * elementBytes

  /** caculate end byte index of this vlen when is in one register group. parameter is segment index. */
  val endByteLmulX: Int => UInt = (pregIdx: Int) => WireInit(
    Mux(lmulLargerThanOne && lmulVld(pregIdx), (ap.vLenb.U * (pregIdx + 1).U).asUInt(),
      Mux(lmul1, ap.vLenb.U, (ap.vLenb.U >> shrinkRate).asUInt())))
  /** Compare two end byte index and pick smaller one. parameter is segment index. */
  val endByteIdx: Int => UInt = (pregIdx: Int) => WireInit(
    Mux(endByteLmulX(pregIdx) > endBytevl, endBytevl, endByteLmulX(pregIdx)))

  /** 0 means this byte need to be accessed, parameter is byte index. */
  val byteVldX: (Int, UInt) => Bool = (byteIdx: Int, endByteIdx: UInt) => WireInit(
    byteIdx.U < startByte || byteIdx.U >= endByteIdx)

  /** This is normal look up to vm and check if is valid according to vl. */
  val snippetX: Int => UInt = (pregIdx: Int) => WireInit(
    VecInit(Seq.tabulate(ap.vLenb)(byteIdx => byteIdx + pregIdx * ap.vLenb).map(globalByteIdx => {
      val byteVldByVM: Bool =
        Mux(elen1, elementVldVec(globalByteIdx),
          Mux(elen2, elementVldVec(globalByteIdx >> 1),
            Mux(elen4, elementVldVec(globalByteIdx >> 2),
              elementVldVec(globalByteIdx >> 3))))

      val byteVldByVL = !byteVldX(globalByteIdx, endByteIdx(pregIdx))
      // 0 means need access.
      val byteVld = Mux(byteVldByVL, !byteVldByVM, true.B)
      byteVld
    })).asUInt())

  io.adjusterSnippet.zipWithIndex.foreach{case (out, pregIdx) =>
  out := Mux(io.ctrl.isWholeAccess, Fill(ap.vLenb, !lmulVld(pregIdx)), snippetX(pregIdx))}
}

/** logics combinations to split request from load store queue entry. */
class RequestSpliter(ap: VLSUArchitecturalParams, isLoad: Boolean, id: Int) extends VLSUModules(ap){
  val io = IO(new Bundle{
    val reg = if(isLoad) Input(new VLdQEntryBundle(ap)) else Input(new VStQEntryBundle(ap))
    val uReq = if(isLoad) Valid(new VLdRequest(ap)) else Valid(new VStRequest(ap))
    val newAddr = Output(UInt(ap.coreMaxAddrBits.W))
    val newSnippet = Output(UInt(ap.vLenb.W))
  })
  val reg = io.reg
  val lmul = reg.style.vlmul(1,0)
  val lmulValue = (1.U << reg.style.vlmul(1,0)).asUInt()
  val lmulLargerEqualOne = !reg.style.vlmul(2)
  val lmulLargerThanOne = lmulLargerEqualOne && lmul =/= 0.U
  val segmentModLmul = WireInit(reg.segmentCount % lmulValue)

  io.newAddr := 0.U
  io.newSnippet := 0.U
  io.uReq := 0.U.asTypeOf(if (isLoad) Valid(new VLdRequest(ap)) else Valid(new VStRequest(ap)))
  when(reg.style.isUnitStride || reg.style.isWholeAccess){
    val baseAddr = Mux(reg.style.isWholeAccess, reg.addr + (reg.segmentCount << log2Ceil(ap.vLenb).U).asUInt(),
      reg.addr + Mux(lmulLargerEqualOne, ap.vLenb.U * segmentModLmul, 0.U))
    val (reqNecessary, req) = io.uReq.bits.UnitStride(baseAddr, reg.reqCount, reg.pRegVec(reg.segmentCount),
      reg.segmentCount, reg.style.isWholeAccess, true, reg.finishMasks, id)
    io.uReq.valid := reqNecessary
    io.uReq.bits := req
  }.elsewhen(reg.style.isConstantStride){
    /** when segmentCount === lmul, reset to base addr. */
    val newAddr: UInt = Mux(lmulLargerThanOne && segmentModLmul === 0.U, reg.addr, reg.preAddr)
    val (reqNecessary, req, addr, snippet) = io.uReq.bits.ConstantStride(reg.addr, newAddr, reg.rs2, reg.style.dataEew,
      reg.pRegVec(reg.segmentCount), reg.segmentCount, reg.reqCount, reg.preSnippet, true, reg.finishMasks, id)
    io.uReq.valid := reqNecessary
    io.uReq.bits := req
    io.newAddr := addr
    io.newSnippet := snippet
  }.elsewhen(reg.style.isIndexed){
    val (reqNecessary, req, snippet) = io.uReq.bits.Indexed(addr = reg.addr,
      indexEew = reg.style.indexEew,
      dataEew = reg.style.dataEew,
      dstPreg = reg.pRegVec,
      reqCOunt = reg.reqCount,
      indexRegIdx = reg.segmentCount,
      preSnippet = reg.preSnippet,
      indexArray = reg.vs2,
      isLoad = true,
      initialSnippet = reg.finishMasks, id = id)
    io.uReq.valid := reqNecessary
    io.uReq.bits := req
    io.newSnippet := snippet
  }
}

object GetNewBranchMask{
  def apply(brUpdate: BranchUpdateInfo, brMask: UInt): UInt = {
    brMask & (~brUpdate.b1.resolveMask).asUInt()
  }
}
object IsKilledByBranch{
  def apply(brUpdate: BranchUpdateInfo, brMask: UInt): Bool = {
    maskMatch(brUpdate.b1.mispredictMask, brMask)
  }
}
