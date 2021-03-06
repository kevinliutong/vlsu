package vlsu
import chisel3._
import chisel3.util._
/** Check requesting addr among lmshr and scalar dcache.
 * If miss, allocate a mshr.
 * If hit lmshr, do nothing.
 * If hit scalar dcache, padd data to wb-controller
 * */
class VectorLoadAddressChecker(ap: VLSUArchitecturalParams) extends VLSUModules(ap){
  val io = IO(new Bundle{
    val brUpdate = Input(new BranchUpdateInfo(ap))
    /** Receive processing address from lmshrs */
    val lmshrStatus = Flipped(Vec(ap.nLmshrs, Valid(UInt(ap.coreMaxAddrBits.W))))
    /** requests from req-buffer winner. */
    val reqIncoming = Flipped(DecoupledIO(new VLdRequest(ap)))
    /** allocate an idle mshr if not hit any */
    val mshrAllocate = Vec(ap.nLmshrs, Valid(new VLdRequest(ap)))
    val vtlbResp = Flipped(Valid(new VTLBResp(ap)))
  })
  io.mshrAllocate := 0.U.asTypeOf(Vec(ap.nLmshrs, Valid(new VLdRequest(ap))))
  val mshrVlds = io.lmshrStatus.map(_.valid)

  val reg = RegInit(0.U.asTypeOf(Valid(new VLdRequest(ap))))
  val idleLmshrs = VecInit(io.lmshrStatus.map(!_.valid)).asUInt()
  val idleLmshr = idleLmshrs.orR()
  val allocateMshrIdx = OHToUInt(FindFirstOne(idleLmshrs, ap.nLmshrs))

  when(reg.valid && idleLmshr){
    io.mshrAllocate(allocateMshrIdx).valid := true.B
    io.mshrAllocate(allocateMshrIdx).bits := reg.bits
    reg.valid := false.B
  }
  /** Indicates if new request is capable to process. */
  val newAddrSound = io.reqIncoming.valid && ((io.vtlbResp.valid && io.vtlbResp.bits.hit) || io.reqIncoming.bits.addressIsPhysical)
  val newAddr = Mux(io.reqIncoming.valid && io.reqIncoming.bits.addressIsPhysical, io.reqIncoming.bits.address,
    Mux(io.reqIncoming.valid && io.vtlbResp.valid && io.vtlbResp.bits.hit, io.vtlbResp.bits.paddr, 0.U))
  val newAddrHitMshrs = VecInit(io.lmshrStatus.map{mshr => mshr.valid && (newAddr >> ap.offsetBits).asUInt() === (mshr.bits >> ap.offsetBits).asUInt()}).asUInt()
  val newAddrHitMshr = newAddrHitMshrs.orR()
  val newAddrHitReg = reg.valid && ((reg.bits.address >> ap.offsetBits).asUInt() === (newAddr >> ap.offsetBits).asUInt())
  val newAddrMissAll = !newAddrHitMshr && !newAddrHitReg
  val mshrEnough = PopCount(idleLmshr) > 1.U
  /** 1. reg is valid, new addr hit mshr or reg, then ack, no take in
   *                   new addr misses all, and there is no idle mshr, then no ack, no take in.
   *  2. reg is idle,  new addr hit mshr, then ack, no take in.
   *                   new addr miss all, then ack, take in.
   * */
  val takeIn = newAddrMissAll && (!reg.valid || mshrEnough)
  /** back pressure. */
  val sendAck = !newAddrMissAll || !reg.valid || mshrEnough
  when(io.reqIncoming.valid && takeIn){
    reg.valid := true.B
    reg.bits := io.reqIncoming.bits
    reg.bits.brMask := GetNewBranchMask(io.brUpdate, io.reqIncoming.bits.brMask)
    reg.bits.address := newAddr
  }
  /** addr in reg should not hit any busy mshr. */
  io.reqIncoming.ready := io.reqIncoming.valid && sendAck && newAddrSound
}
