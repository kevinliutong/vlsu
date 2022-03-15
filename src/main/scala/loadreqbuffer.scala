package vlsu
import chisel3._
import chisel3.util._
/** This module hold all element access for all vector store or load uop. */
class LoadRequestBuffer(ap: VLSUArchitecturalParams) extends VLSUModules(ap){
  val io = IO(new Bundle{
    val brUpdate = Input(new BranchUpdateInfo(ap))
    /** vld request from vldq winner. */
    val reqIncoming = Flipped(Vec(ap.coreWidth, Decoupled(new VLdRequest(ap))))
    /** when a line is fetch back from L2, the buffer will be scanned to pick all request with same address */
    val wbBypassQuery = Flipped(Valid(UInt(ap.coreMaxAddrBits.W)))
    /** Till wb controller which buffer entry is fit for bypass write back. */
    val wbBypassResp = Vec(ap.nVLdBuffEntries, Valid(new VLdRequest(ap)))
    /** query addr checker to process data fetch. */
    val reqOutgoing = Decoupled(new VLdRequest(ap))
    /** query tlb to fetch paddr. */
    val vtlbReq = Decoupled(new VTLBReq(ap))
    /** resp from vtlb, if miss, then this entry should re-issue. */
    val vtlbResp = Flipped(Valid(new VTLBResp(ap)))
    /** from wb Controller telling us which buffer entry has finished. */
    val reqWBDone = Flipped(Valid(UInt(ap.nVLdBuffEntryIdxBits.W)))
    /** from vldq telling us which buffer entry need to retire due to commit.*/
    val fromRob = new ROBVLSUIO(ap)
  })
  /** Instances of entries. */
  val ldEntries = Seq.tabulate(ap.nVLdBuffEntries)(i => Module(new LoadBufferEntry(ap, i)))
  /** Input IO to entries. */
  val entryInputs = WireInit(0.U.asTypeOf(Vec(ap.nVLdBuffEntries, Valid(new VLdRequest(ap)))))
  /** idle entries to allocate. */
  val entryVlds = VecInit(ldEntries.map(_.io.status.valid)).asUInt()
  // pick corewidth idle entries to alocate.
  val idleEntryArbiter = Module(new IndexArbiter(ap.nVLdBuffEntries, ap.coreWidth))
  idleEntryArbiter.io.input := entryVlds
  val idleEntryIndexes = idleEntryArbiter.io.output
  /** From all idle entries, pick out one and convert it into index then pass back to queue. */
  val vLdReqArbiter = Module(new Arbiter(new VLdRequest(ap), ap.nVLdBuffEntries))
  ldEntries.zipWithIndex.foreach{case (e,i) =>
    vLdReqArbiter.io.in(i) <> e.io.reqOutgoing
    e.io.wbBypassQuery <> io.wbBypassQuery
    io.wbBypassResp(i) <> e.io.wbBypassResp
    e.io.reqIncoming <> entryInputs(i)
    e.io.reqWBDone := io.reqWBDone
    e.io.fromVTLB := io.vtlbResp
    e.io.fromRob := io.fromRob
    e.io.brUpdate := io.brUpdate
  }
  (io.reqIncoming.map(_.ready) zip idleEntryIndexes.map(_.valid)).foreach{ case(r, v) => r := v}

  //New req allocate.
  var inputIdx = WireInit(0.U.asTypeOf(UInt(log2Ceil(ap.coreWidth).W)))
  for (w <- 0 until ap.coreWidth){
    val vld = io.reqIncoming(w).valid && idleEntryIndexes(inputIdx).valid
    when(vld){
      entryInputs(idleEntryIndexes(inputIdx).bits) := io.reqIncoming(w)
    }
    inputIdx = Mux(vld, inputIdx + 1.U, inputIdx)
  }
  //Request output
  io.reqOutgoing <> vLdReqArbiter.io.out
  io.vtlbReq.valid := vLdReqArbiter.io.out.valid && !vLdReqArbiter.io.out.bits.addressIsPhysical
  io.vtlbReq.bits.vaddr := vLdReqArbiter.io.out.bits.address
  io.vtlbReq.bits.reqBuffIdx := vLdReqArbiter.io.out.bits.reqBufferIdx
  io.vtlbReq.bits.isLoad := true.B
  io.vtlbReq.bits.queueIdx := vLdReqArbiter.io.out.bits.qEntryIdx
  io.vtlbReq.bits.snippet := vLdReqArbiter.io.out.bits.regAccessCS.finishMaskSnippet
  io.vtlbReq.bits.segmentIdx := vLdReqArbiter.io.out.bits.segmentIdx
}

class LoadBufferEntry(ap: VLSUArchitecturalParams, idx: Int) extends VLSUModules(ap){
  val io = IO(new Bundle{
    val brUpdate = Input(new BranchUpdateInfo(ap))
    /** request from vldq arb winner. */
    val reqIncoming = Flipped(Valid(new VLdRequest(ap)))
    /** expose us to buffer for new request allocate from vldq. */
    val status = Valid(new VLdRequest(ap))
    /** request issue down to address checker. */
    val reqOutgoing = Decoupled(new VLdRequest(ap))
    /** address query from wb controller for bypass wb. */
    val wbBypassQuery = Flipped(Valid(UInt(ap.coreMaxAddrBits.W)))
    /** tell wb if querying address hit us. */
    val wbBypassResp = Valid(new VLdRequest(ap))
    /** from wb controller to tell us which entry has finished. */
    val reqWBDone = Flipped(Valid(UInt(ap.nVLdBuffEntryIdxBits.W)))
    /** resp from vtlb at same cycle. */
    val fromVTLB = Flipped(Valid(new VTLBResp(ap)))
    val fromRob = new ROBVLSUIO(ap)
  })
  io.wbBypassResp.valid := false.B
  io.wbBypassResp.bits := 0.U.asTypeOf(new VLdRequest(ap))
  val reg = RegInit(0.U.asTypeOf(Valid(new VLdRequest(ap))))
  val isKilledByBranch = IsKilledByBranch(io.brUpdate, reg.bits.brMask)
  reg.bits.reqBufferIdx := idx.U
  io.status := reg
  io.reqOutgoing.valid := (reg.valid && !reg.bits.executing && !reg.bits.done && !isKilledByBranch)
  io.reqOutgoing.bits := reg.bits
  io.reqOutgoing.bits.reqBufferIdx := idx.U
  io.reqOutgoing.bits.brMask := reg.bits.brMask
  when(reg.valid && io.reqOutgoing.fire() && (reg.bits.addressIsPhysical || io.fromVTLB.bits.hit)){
    reg.bits.executing := true.B
  }
  when(io.wbBypassQuery.valid && lineAddress(io.wbBypassQuery.bits) === lineAddress(reg.bits.address) &&
  !reg.bits.done && reg.valid && !isKilledByBranch){
    io.wbBypassResp.valid := true.B
    io.wbBypassResp.bits := reg.bits
  }
  when(io.reqIncoming.valid){
    assert(!reg.valid, s"Entry ${idx} should be idle when allocate new request.")
    reg.valid := true.B
    reg.bits := io.reqIncoming.bits
  }
  when(io.reqWBDone.valid && io.reqWBDone.bits === idx.U && reg.valid){
    reg.bits.done := true.B
    reg.bits.executing := false.B
  }
  val commitHit = VecInit(io.fromRob.retireEntries.map{i => i.valid && i.bits.isLoad && i.bits.qEntryIdx === reg.bits.qEntryIdx}).asUInt().orR()
  when(reg.valid && commitHit && !isKilledByBranch){
    assert(Mux(reg.valid, reg.bits.done, true.B), "committing an unfinished req.")
    reg.valid := false.B
  }
  when(reg.valid && isKilledByBranch){
    reg.valid := false.B
  }
}