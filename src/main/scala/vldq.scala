package vlsu

import chisel3._
import chisel3.util._
import boom.util._

class VLdQueueHandler(ap: VLSUArchitecturalParams) extends VLSUModules(ap){
  val io = IO(new Bundle{
    val brUpdate = Input(new BranchUpdateInfo(ap))
    /** Vector uop from dispatch stage. */
    val vuopDis = Flipped(Vec(ap.coreWidth, Valid(new VLSMicroOP(ap))))
    /** Vector uop from register-read stage with data/address */
    val vuopRR = Flipped(Vec(ap.coreWidth, Valid(new VLSMicroOP(ap))))
    /** Winner req from entries */
    val vldReq = Vec(ap.coreWidth, Decoupled(new VLdRequest(ap)))
    /** Finish ack from wb controller */
    val finishAck = Flipped(Valid(new VLdRequest(ap)))
    /** Tell core pipeline index of newly allocated load queue entry. */
    val disVLdQIdx = Output(Vec(ap.coreWidth, Valid(new DispatchAck(ap))))
    /** Tell core stop dispatch when no entry left. */
    val vLdQFull = Output(Vec(ap.coreWidth, Bool()))
    /** Tell Rob that one vuop has done all elements loads. */
    val toRob = new VLSUROBIO(ap)
    /** Retire finished load queue entry. */
    val fromRob = new ROBVLSUIO(ap)
    /** release done reg for next process. */
    val wakeUp = ValidIO(UInt(ap.nVPregSz.W))

    /** for untouched load, we need to copy original data and write back to new reg. */
    val vrfReadReq = Decoupled(new VLSUReadVRFReq(ap))
    val vrfReadResp = Flipped(Valid(new VLSUReadVRFResp(ap)))
    val vrfWriteReq = Decoupled(new VLSUWriteVRFReq(ap))

    val vrfBusyStatus = Input(UInt(ap.nVRegs.W))
  })
  val nEntries = ap.nVLdQEntries
  val nIdxBits = ap.nVLdQIdxBits
  val headPtr = RegInit(0.U(nIdxBits.W))
  val tailPrt = RegInit(0.U(nIdxBits.W))

  val nonUnitStride = RegInit(0.U.asTypeOf(Vec(nEntries, Bool())))
  io.toRob := 0.U.asTypeOf(new VLSUROBIO(ap))
  val vldReqArb = Module(new RequestArbitrator(ap, new VLdRequest(ap), true))
  val vuopDisInputs = WireInit(0.U.asTypeOf(Vec(nEntries, Decoupled(new VLSMicroOP(ap)))))
  val vuopRRInputs = WireInit(0.U.asTypeOf(Vec(nEntries, Decoupled(new VLSMicroOP(ap)))))

  val reqCandidates = WireInit(0.U.asTypeOf(Vec(nEntries, Decoupled(new VLdRequest(ap)))))
  val toRobVec = WireInit(0.U.asTypeOf(Vec(nEntries, Decoupled(UInt(ap.robAddrSz.W)))))
  val wakeUpVec = WireInit(0.U.asTypeOf(Vec(nEntries, Decoupled(UInt(ap.nVPregSz.W)))))
}

class VLdQEntry(ap: VLSUArchitecturalParams, id: Int) extends VLSUModules(ap){
  val io = IO(new Bundle{
    val brUpdate = Input(new BranchUpdateInfo(ap))
    val entryId = Output(UInt(ap.nVLdQIdxBits.W))
    /** arrived vuop from dispatch stage. */
    val vuopDis = Flipped(Valid(new VLSMicroOP(ap)))
    /** Arrived vuop from register-read stage. */
    val vuopRr = Flipped(Valid(new VLSMicroOP(ap)))
    /** Output request for a single cache line. */
    val uReq = Decoupled(new VLdRequest(ap))

    /** Write back controller tells vld queue finished access. */
    val finishAck = Flipped(Valid(new VLdRequest(ap)))
    /** tess rob that this entry has all finished. */
    val robAck = Decoupled(UInt(ap.robAddrSz.W))
    val allDone = Output(Bool())
    val fromRob = new ROBVLSUIO(ap)
    val headPtr = Input(UInt(ap.nVLdQIdxBits.W))
    val tailPrt = Input(UInt(ap.nVLdQIdxBits.W))
    val nonUnitStrideOHs = Input(UInt(ap.nVLdQEntries.W))
    /** wake up core pipeline when single register is all done. */
    val wakeUp = Decoupled(UInt(ap.nVPregSz.W))

    val vrfReadReq = Decoupled(new VLSUReadVRFReq(ap))
    val vrfReadResp = Flipped(Valid(new VLSUReadVRFResp(ap)))
    val vrfWriteReq = Decoupled(new VLSUWriteVRFReq(ap))
    val vrfBusyStatus = Input(UInt(ap.nVRegs.W))
  })

  io.uReq.valid := false.B
  io.uReq.bits := 0.U.asTypeOf(new VLdRequest(ap))
  io.robAck.valid := false.B
  io.robAck.bits := 0.U
  io.wakeUp.valid := false.B
  io.wakeUp.bits := 0.U
  io.vrfReadReq.valid := false.B
  io.vrfReadReq.bits := 0.U.asTypeOf(new VLSUReadVRFReq(ap))
  io.vrfWriteReq.valid := false.B
  io.vrfWriteReq.bits := 0.U.asTypeOf(new VLSUWriteVRFReq(ap))
  /** This regist */
}