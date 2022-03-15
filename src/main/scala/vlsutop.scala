package vlsu
import chisel3._
import chisel3.util._
import freechips.rocketchip.config.Parameters
import boom.common._
import freechips.rocketchip.devices.tilelink.TLTestRAM
import freechips.rocketchip.diplomacy._
import freechips.rocketchip.subsystem.{BaseSubsystem, BaseSubsystemModuleImp, InSubsystem, TilesLocated}
import freechips.rocketchip.tilelink._
class VLSU(generalParameters: VLSUGeneralParameters, boomParams: BoomCoreParams)(implicit p: Parameters) extends LazyModule() {
  val vmshrs = generalParameters.nLmshrs + generalParameters.nSmshrs
  val nL2BeatBytes = generalParameters.nL2DataBeatBytes
  val node = TLClientNode(Seq(TLMasterPortParameters.v1(
    Seq(TLMasterParameters.v1(
      name = s"l1vlsu",
      sourceId = IdRange(0, vmshrs),
      supportsProbe = TransferSizes.none
    ))
  )))(ValName(s"fromL1vlsu"))
  lazy val module = new VLSUImp(this, generalParameters, boomParams)
}
/** this module describe top architecture of vlsu */
class VLSUImp(outer: VLSU, generalParameters: VLSUGeneralParameters, boomParams: BoomCoreParams) extends LazyModuleImp(outer){
  val ap = new VLSUArchitecturalParams(generalParameters, boomParams, outer.node.edges.out(0))
  val (tlOut, _) = outer.node.out(0)
  val io = IO(new VLSUTopBundle(ap))

  //Address-Gen and split
  val vldqHandler = Module(new VLdQueueHandler(ap))
  vldqHandler.io.vuopDis <> io.fromDis.vuopDis
  vldqHandler.io.vuopRR <> io.fromRr.vuop
  io.toDis.vLdQFull := vldqHandler.io.vLdQFull
  io.toDis.disVLdQIdx := vldqHandler.io.disVLdQIdx
  io.ldToRob <> vldqHandler.io.toRob
  vldqHandler.io.fromRob := io.fromRob
  io.wakeUpVreg <> vldqHandler.io.wakeUp
  vldqHandler.io.vrfReadResp := io.fromVrf.readResp
  vldqHandler.io.vrfBusyStatus := io.vrfBusyStatus
  vldqHandler.io.brUpdate := io.brUpdate

  val vstqHandler = Module(new VStQueueHandler(ap))
  vstqHandler.io.vuopDis <> io.fromDis.vuopDis
  vstqHandler.io.vuopRR <> io.fromRr.vuop
  io.toDis.vStQFull := vstqHandler.io.vStQFull
  io.toDis.disVStQIdx := vstqHandler.io.disVStQIdx
  io.stToRob <> vstqHandler.io.toRob
  vstqHandler.io.fromRob := io.fromRob
  vstqHandler.io.brUpdate := io.brUpdate
  //____________
  //Address read
  val loadReqBuff = Module(new LoadRequestBuffer(ap))
  loadReqBuff.io.reqIncoming <> vldqHandler.io.vldReq
  loadReqBuff.io.fromRob := io.fromRob
  loadReqBuff.io.brUpdate := io.brUpdate

  val storeReqBuff = Module(new StoreRequestBuffer(ap))
  storeReqBuff.io.reqIncoming <> vstqHandler.io.vstReq
  storeReqBuff.io.fromRob := io.fromRob
  //store req always win because its req also goes into addr checker.
  io.toVrf.readReq.valid := storeReqBuff.io.vrfReadReq.valid || vldqHandler.io.vrfReadReq.valid
  io.toVrf.readReq.bits := Mux(storeReqBuff.io.vrfReadReq.valid, storeReqBuff.io.vrfReadReq.bits, vldqHandler.io.vrfReadReq.bits)
  vldqHandler.io.vrfReadReq.ready := !storeReqBuff.io.vrfReadReq.valid
  storeReqBuff.io.brUpdate := io.brUpdate

  val vtlb = Module(new VTLB(ap))
  vtlb.io.req(0) <> loadReqBuff.io.vtlbReq
  vtlb.io.req(1) <> storeReqBuff.io.vtlbReq
  loadReqBuff.io.vtlbResp := vtlb.io.resp(0)
  storeReqBuff.io.vtlbResp := vtlb.io.resp(1)
  vstqHandler.io.fromVTLB := vtlb.io.resp(1)
  //____________
  //Address-match
  val loadAddrChecker = Module(new VectorLoadAddressChecker(ap))
  loadAddrChecker.io.vtlbResp <> vtlb.io.resp(0)
  loadAddrChecker.io.reqIncoming <> loadReqBuff.io.reqOutgoing
  loadAddrChecker.io.brUpdate := io.brUpdate

  val storeAddrChecker = Module(new VectorStoreAddressChecker(ap))
  storeAddrChecker.io.vtlbResp <> vtlb.io.resp(0)
  storeAddrChecker.io.reqIncoming <> storeReqBuff.io.reqOutgoing
  storeReqBuff.io.dataAccomplished := storeAddrChecker.io.storeResp
  vstqHandler.io.dataAccomplished := storeAddrChecker.io.storeResp
  val mshrFile = Module(new MSHRFiles(ap))
  mshrFile.io.lmshrAllocateReq <> loadAddrChecker.io.mshrAllocate
  mshrFile.io.smshrAllocateReq <> storeAddrChecker.io.mshrAllocate
  mshrFile.io.brUpdate := io.brUpdate
  tlOut.a <> mshrFile.io.tlOutA
  mshrFile.io.tlInD <> tlOut.d
  loadAddrChecker.io.lmshrStatus <> mshrFile.io.lmshrStatus
  storeAddrChecker.io.smshrStatus <> mshrFile.io.smshrStatus
  mshrFile.io.vrfReadResp := io.fromVrf.readResp
  //_______________
  //Data write back
  val wbCtrl = Module(new wbController(ap))
  wbCtrl.io <> DontCare
  wbCtrl.io.wbReqIncoming <> mshrFile.io.writeBackReq
  loadReqBuff.io.wbBypassQuery <> wbCtrl.io.wbBypassQuery
  wbCtrl.io.wbBypassResp <> loadReqBuff.io.wbBypassResp
  loadReqBuff.io.reqWBDone <> wbCtrl.io.reqWBDone
  vldqHandler.io.finishAck <> wbCtrl.io.finishAck
  io.toVrf.write <> wbCtrl.io.toVRF
  // write stale data for undisturbed load.
  wbCtrl.io.wbReqFromQEntry <> vldqHandler.io.vrfWriteReq
}

class VLSUHarness(gp: VLSUGeneralParameters) (implicit p: Parameters) extends Module{
  val lazyvlsu = LazyModule(new VLSUHarnessSystem(gp)(p))
  val vlsu = Module(lazyvlsu.module)
  val ap = vlsu.ap
  val io = IO(new Bundle{
    val topIO = new VLSUTopBundle(ap)
  })
  vlsu.io.fromRr := io.topIO.fromRr
  io.topIO.toDis := vlsu.io.toDis
  vlsu.io.fromDis := io.topIO.fromDis

  io.topIO.toVrf := vlsu.io.toVrf
  vlsu.io.fromVrf := io.topIO.fromVrf

  io.topIO.ldToRob := vlsu.io.ldToRob
  io.topIO.stToRob := vlsu.io.stToRob
  vlsu.io.fromRob := io.topIO.fromRob

  io.topIO.wakeUpVreg := vlsu.io.wakeUpVreg

  vlsu.io.vrfBusyStatus := io.topIO.vrfBusyStatus
  vlsu.io.brUpdate := io.topIO.brUpdate
}

class VLSUHarnessSystem(genparam: VLSUGeneralParameters)(implicit p: Parameters) extends BaseSubsystem{
  override lazy val module = new VLSUHarnessSystemImp(this)
  val gp = genparam
  val bp = p(TilesLocated(InSubsystem)).head.tileParams.core.asInstanceOf[BoomCoreParams]
  val vlsu = LazyModule(new VLSU(gp, bp))
  val vlsuWidget = LazyModule(new TLWidthWidget(64)).suggestName("vlsuWidthWidget")
  sbus.inwardNode := vlsuWidget.node := vlsu.node
  val ddr = LazyModule(new TLTestRAM(AddressSet(0x80000000L, 0xfffffffL), true, 64))
  val ddrWidget = LazyModule(new TLWidthWidget(mbus.beatBytes)).suggestName("ddrWidthWidget")
  ddr.node := ddrWidget.node := mbus.outwardNode
}

class VLSUHarnessSystemImp[+L <: VLSUHarnessSystem](_outer: L) extends BaseSubsystemModuleImp(_outer){
  val ap = new VLSUArchitecturalParams(outer.gp, outer.bp, outer.vlsu.node.edges.out(0))
  val io = IO(new VLSUTopBundle(ap))
  io <> outer.vlsu.module.io
}