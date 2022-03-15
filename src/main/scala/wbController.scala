package vlsu
import chisel3._
import chisel3.util._
class wbController(ap: VLSUArchitecturalParams) extends VLSUModules(ap) {
  val io = IO(new Bundle{
    val wbReqIncoming = Flipped(Decoupled(new VLdWriteBackRequest(ap)))
    val toVRF = Valid(new VLSUWriteVRFReq(ap))
    /** Query req buffer with memory addr that we gonna write back. */
    val wbBypassQuery = Valid(UInt(ap.coreMaxAddrBits.W))
    /** Tell vldq handler which entry has partly finished. */
    val finishAck = Valid(new VLdRequest(ap))
    /** Tell req buffer which entry has finished. */
    val reqWBDone = Valid(UInt(ap.nVLdBuffEntryIdxBits.W))

    /** Data path for queue entry writing stale data for undisturbed load. */
    val wbReqFromQEntry = Flipped(Decoupled(new VLSUWriteVRFReq(ap)))

    /** Vec input is to detect order fail. */
    val wbBypassResp = Flipped(Vec(ap.nVLdBuffEntries, Valid(new VLdRequest(ap))))
  })
  val sIdle :: sNormalWB :: sBypassWB :: Nil = Enum(3)
  val state = RegInit(sIdle)
  io.toVRF := 0.U.asTypeOf(Valid(new VLSUWriteVRFReq(ap)))
  io.wbBypassQuery := 0.U.asTypeOf(Valid(UInt(ap.coreMaxAddrBits.W)))
  io.reqWBDone := 0.U.asTypeOf(Valid(UInt(ap.nVLdBuffEntryIdxBits.W)))
  io.finishAck := 0.U.asTypeOf(Valid(new VLdRequest(ap)))
  val reg = RegInit(0.U.asTypeOf(Valid(new VLdWriteBackRequest(ap))))

  when(io.wbReqIncoming.fire()){
    reg.valid := true.B
    reg.bits := io.wbReqIncoming.bits
    state := sNormalWB
  }
  // Calculation of data to vrf here.
  val awaitingVLdReq = WireInit(0.U.asTypeOf(new VLdRequest(ap)))
  val awaitingVRFReq = WireInit(0.U.asTypeOf(new VLSUWriteVRFReq(ap)))
  val dataStretcherUnitStride = Module(new UnitStrideDataStretcher(ap))
  dataStretcherUnitStride.io.lineDataIn := reg.bits.data
  dataStretcherUnitStride.io.reqWB := awaitingVLdReq

  val dataStretcherElemental = Module(new ElementDataStretcher(ap))
  dataStretcherElemental.io.lineDataIn := reg.bits.data
  dataStretcherElemental.io.reqWB := awaitingVLdReq
  val bypassRespVlds = VecInit(io.wbBypassResp.map(_.valid)).asUInt()
  val hasBypassChance = bypassRespVlds.orR()
  val bypassWBIdx = OHToUInt(FindFirstOne(bypassRespVlds, ap.nVLdBuffEntries))

  awaitingVRFReq := Mux(awaitingVLdReq.style.isUnitStride, dataStretcherUnitStride.io.vrfWriteReq, dataStretcherElemental.io.vrfWriteReq)
  when(state === sNormalWB){
    //Write back to VRF.
    io.toVRF.valid := true.B
    awaitingVLdReq := reg.bits.req
    //tell vldq handler we finished one req.
    io.finishAck.valid := true.B
    io.finishAck.bits := reg.bits.req
    //tell req-buffer we finished one req.
    io.reqWBDone.valid := true.B
    io.reqWBDone.bits := reg.bits.req.reqBufferIdx
    state := sBypassWB
  }.elsewhen(state === sBypassWB){
    // Send query to req-buff handler, resp returns at same cycle.
    io.wbBypassQuery.valid := true.B
    io.wbBypassQuery.bits := reg.bits.req.address
    io.toVRF.valid := hasBypassChance
    awaitingVLdReq := io.wbBypassResp(bypassWBIdx).bits
    io.finishAck.valid := hasBypassChance
    io.finishAck.bits := io.wbBypassResp(bypassWBIdx).bits
    io.reqWBDone.valid := hasBypassChance
    io.reqWBDone.bits := bypassWBIdx
    when(!hasBypassChance){
      state := sIdle
    }
  }
  io.wbReqIncoming.ready := (state === sIdle)
  val idling = state === sIdle
  io.wbReqFromQEntry.ready := idling
  when(io.wbReqFromQEntry.fire()){
    io.toVRF.valid := true.B
  }
  io.toVRF.bits := Mux(io.wbReqFromQEntry.fire(), io.wbReqFromQEntry.bits, awaitingVRFReq)
}
