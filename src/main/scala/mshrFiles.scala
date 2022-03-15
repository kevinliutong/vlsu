package vlsu
import chisel3._
import chisel3.util._
import freechips.rocketchip.diplomacy._
import freechips.rocketchip.tilelink._
class MSHRFiles(ap: VLSUArchitecturalParams) extends VLSUModules(ap) {
  val io = IO(new Bundle{
    val brUpdate = Input(new BranchUpdateInfo(ap))
    val lmshrStatus = Vec(ap.nLmshrs, Valid(UInt(ap.coreMaxAddrBits.W)))
    val lmshrAllocateReq = Flipped(Vec(ap.nLmshrs, Valid(new VLdRequest(ap))))
    val smshrStatus = Vec(ap.nSmshrs, Valid(new SMSHRStatus(ap)))
    val smshrAllocateReq = Flipped(Vec(ap.nSmshrs, Valid(new VStRequest(ap))))
    val tlOutA = Decoupled(new TLBundleA(ap.l2BundleParams))
    val tlInD = Flipped(Decoupled(new TLBundleD(ap.l2BundleParams)))
    val writeBackReq = Decoupled(new VLdWriteBackRequest(ap))
    val vrfReadResp = Flipped(Valid(new VLSUReadVRFResp(ap)))
  })

  val tlArbiterA = Module(new Arbiter(new TLBundleA(ap.l2BundleParams), ap.nLmshrs + ap.nSmshrs))
  io.tlOutA <> tlArbiterA.io.out
  val wbArbiter = Module(new Arbiter(new VLdWriteBackRequest(ap), ap.nLmshrs))
  io.writeBackReq <> wbArbiter.io.out

  val lmshrs = Seq.tabulate(ap.nLmshrs){i =>
    val mshr = Module(new LMSHR(ap, i))
    mshr.io.brUpdate := io.brUpdate
    mshr.io.vLdReq <> io.lmshrAllocateReq(i)
    tlArbiterA.io.in(i) <> mshr.io.tlOutA
    mshr.io.tlInD <> io.tlInD
    io.lmshrStatus(i) <> mshr.io.status
    wbArbiter.io.in(i) <> mshr.io.writeBackReq
    mshr
  }
  val smshrs = Seq.tabulate(ap.nSmshrs){i =>
    val mshr = Module(new SMSHR(ap, i))
    mshr.io.vStReq <> io.smshrAllocateReq(i)
    tlArbiterA.io.in(i + ap.nLmshrs) <> mshr.io.tlOutA
    mshr.io.tlInD <> io.tlInD
    io.smshrStatus(i) <> mshr.io.status
    mshr.io.fromVRF := io.vrfReadResp
    mshr
  }
}
