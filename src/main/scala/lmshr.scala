package vlsu
import chisel3._
import chisel3.util._
import freechips.rocketchip.tilelink._
class LMSHR(ap: VLSUArchitecturalParams, idx: Int) extends VLSUModules(ap) {
  val io = IO(new Bundle{
    val brUpdate = Input(new BranchUpdateInfo(ap))
    val vLdReq = Flipped(Valid(new VLdRequest(ap)))
    val tlOutA = Decoupled(new TLBundleA(ap.l2BundleParams))
    val tlInD = Flipped(Decoupled(new TLBundleD(ap.l2BundleParams)))
    val status = Valid(UInt(ap.coreMaxAddrBits.W))
    val writeBackReq = Decoupled(new VLdWriteBackRequest(ap))
  })
  require(io.tlOutA.bits.data.getWidth == ap.nCacheLineBits, "one cache line at same cycle.")
  io.tlOutA.valid := false.B
  io.tlOutA.bits := 0.U.asTypeOf(new TLBundleA(ap.l2BundleParams))
  io.status := 0.U.asTypeOf(Valid(UInt(ap.coreMaxAddrBits.W)))
  io.tlOutA.valid := false.B
  io.writeBackReq.valid := false.B
  io.writeBackReq.bits := 0.U.asTypeOf(new VLdWriteBackRequest(ap))
  val lineBuffer = RegInit(0.U.asTypeOf(UInt(ap.nCacheLineBits.W)))
  val sIdle :: sWaitArb :: sWaitData :: sWaitWB :: Nil = Enum(4)
  val state = RegInit(sIdle)
  val reg = RegInit(0.U.asTypeOf(Valid(new VLdRequest(ap))))
  val isKilledByBranch = IsKilledByBranch(io.brUpdate, reg.bits.brMask)
  when(state === sIdle){
    when(io.vLdReq.valid){
      reg <> io.vLdReq
      reg.bits.brMask := GetNewBranchMask(io.brUpdate, io.vLdReq.bits.brMask)
      state := sWaitArb
    }
  }.elsewhen(state === sWaitArb){
    io.tlOutA.valid := true.B && !isKilledByBranch
    io.tlOutA.bits := DontCare
    io.tlOutA.bits := ap.tlEdge.Get(idx.U, lineAddress(reg.bits.address), log2Ceil(ap.nCacheLineBytes).U)._2
    when(io.tlOutA.fire()){
      state := sWaitData
    }
  }.elsewhen(state === sWaitData) {
    when(io.tlInD.fire() && io.tlInD.bits.source === idx.U){
      lineBuffer := io.tlInD.bits.data
      state := sWaitWB
    }
  }.elsewhen(state === sWaitWB){
    io.writeBackReq.valid := true.B
    io.writeBackReq.bits.data := lineBuffer.asUInt()
    io.writeBackReq.bits.req := reg.bits
    when(io.writeBackReq.fire()){state := sIdle}
  }
  when(state =/= sIdle){
    io.status.valid := true.B
    io.status.bits := reg.bits.address
  }
  io.tlInD.ready := true.B
}
